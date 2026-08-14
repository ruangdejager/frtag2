/*
 * Flash.c
 *
 * AT25EU0041A SPI NOR flash device driver.
 * Follows the fr9 flash.c command pattern (JEDEC-compatible command set).
 *
 * All SPI transactions use the driver macros in Flash_Driver.h which
 * route through the HAL SPI2 layer in hal_spi.h.
 */

#include "Flash.h"
#include "Flash_Config.h"
#include "Flash_Driver.h"
#include "dbg_log.h"
#include "cmsis_os2.h"

/* Deep-power-down state. The flash is parked in DPD whenever idle (the DbgLog
 * consumer issues FLASH_vDeepPowerDown() after draining its queue) so it draws
 * ~µA through the long STOP2 sleep periods instead of standby current. Any
 * access transparently wakes it first via FLASH_vEnsureAwake(). */
static bool bInDpd = false;

/* When true, FLASH_vDeepPowerDown() is suppressed and the chip is kept
 * awake. Set for the duration of an OTA session — see
 * FLASH_vInhibitDeepPowerDown() in Flash.h. */
static bool bDpdInhibited = false;

/* This device is shared: Log.c (driven by DBGLOG_vConsumerTask, its own
 * thread) and OtaStore.c (driven by the AppTask) both call straight into
 * this driver's public functions. Every one of those functions is a
 * multi-step SPI transaction bracketed by CS select/deselect; on a
 * preemptive scheduler, a task switch mid-transaction lets a second task
 * start its own Select/Write/Read/Deselect sequence on the same physical
 * CS/MOSI/MISO lines while the first transaction is still open, corrupting
 * both. Splitting the address space between the log partition and the OTA
 * scratchpad (the assumption an earlier version of this file's comment
 * made) does NOT protect against this — SPI bus ownership, not the target
 * address, is what needs to be exclusive. Recursive because the internal
 * helpers below (FLASH_bWaitReady -> FLASH_bDeviceBusy ->
 * FLASH_u8ReadStatusReg, etc.) call back into other public entry points
 * from the same task while it already holds the lock. */
static osMutexId_t xFlashMutex = NULL;

static void FLASH_vLock(void)
{
    if (xFlashMutex == NULL)
    {
        const osMutexAttr_t attr = { .attr_bits = osMutexRecursive };
        xFlashMutex = osMutexNew(&attr);
    }
    osMutexAcquire(xFlashMutex, osWaitForever);
}

static void FLASH_vUnlock(void)
{
    osMutexRelease(xFlashMutex);
}

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

/* Resume from deep-power-down on demand. No-op when already awake, so only the
 * first access after a DPD pays the resume command + tRES settling delay. */
static void FLASH_vEnsureAwake(void)
{
    if (!bInDpd) return;
    uint8_t cmd = FLASH_CMD_RESUME;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
    osDelay(1);            /* tRES — device ready after resume */
    bInDpd = false;
}

/* Normal "ready before issuing a command" wait. Bounded so a wedged/garbled
 * status read can never hang the DbgLog consumer forever (the old unbounded
 * `while busy` loop is what turned a single bad SPI transaction into
 * "logging just stopped"). */
#define FLASH_WAIT_READY_TIMEOUT_MS   3000U

/* Chip erase of the full 512 KB device can legitimately take several
 * seconds, so its post-erase wait gets its own, much longer, budget. */
#define FLASH_CHIP_ERASE_TIMEOUT_MS   30000U

/* Bounded by real elapsed time, not loop iterations: FLASH_bDeviceBusy()
 * issues two SPI transactions, each individually bounded by SPI_TIMEOUT
 * (1000 ms). If the SPI2 bus is stalled (e.g. contention with another
 * board's SPI sharing these lines), a single FLASH_bDeviceBusy() call can
 * itself take ~2 s and FLASH_u8ReadStatusReg defaults to "busy" on failure
 * - an iteration-counted loop would then take ~u32TimeoutMs * 2s to expire
 * (3000 ms -> ~100 minutes), which looks indistinguishable from a hang. */
static bool FLASH_bWaitReadyTimeout(uint32_t u32TimeoutMs)
{
    uint32_t u32Start = osKernelGetTickCount();
    while (FLASH_bDeviceBusy())
    {
        if ((osKernelGetTickCount() - u32Start) >= u32TimeoutMs)
        {
            DBG("FLASH: busy >%lu ms - aborting operation\r\n", (unsigned long)u32TimeoutMs);
            return false;
        }
        osDelay(1);
    }
    return true;
}

static bool FLASH_bWaitReady(void)
{
    return FLASH_bWaitReadyTimeout(FLASH_WAIT_READY_TIMEOUT_MS);
}

static void FLASH_vWriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}

/* Clear the Write Enable Latch (WRDI). Program, Erase and Write-Status-Register
 * are ONLY accepted while WEL=1, so the chip must never be left idle with WEL
 * set: a single failed/garbled command transaction otherwise leaves the device
 * armed, and any subsequent stray WRSR on a glitchy bus could latch SRP0/BP -
 * which, because WP# is grounded on this board, permanently bricks the part for
 * writes. Calling this on every write error path keeps the armed window down to
 * the few microseconds between our WREN and our own command. */
static void FLASH_vWriteDisable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_DISABLE;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}

/* Clear all non-volatile block protection (Global Unprotect): WREN, then write
 * 00h to both Status Register 1 and 2 in one CS-bracketed WRSR (01h) - clearing
 * BP0..BP4, SRP0 and CMP. Without this a chip whose protection bits ever got
 * set - e.g. SRP0/BP latched by a garbled SPI command - rejects every program
 * and erase forever while reads still work, so its log freezes and even
 * chip-erase is silently ignored.
 *
 * NOTE: this only succeeds while the status register is writable. If SRP0=1 and
 * the hardwired WP# pin is held low the device is in Hardware-Protected mode and
 * the WRSR is ignored - WP# must be driven high (it has no MCU GPIO on this
 * board) before recovery is possible. Returns true once the device reports
 * unprotected. */
static bool FLASH_bGlobalUnprotect(void)
{
    uint8_t cmd[3] = { FLASH_CMD_WRITE_STATUS,
                       FLASH_STATUS_UNPROTECT,   /* SR1: BP, SRP0 */
                       FLASH_STATUS_UNPROTECT }; /* SR2: CMP, SRP1 */

    if (!FLASH_bWaitReady())
        return false;
    BSP_LED_Off(LED_YELLOW);
    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 3) == HAL_OK);
    FLASH_DRIVER_vDeselect();

    if (!bOk)
    {
        FLASH_vWriteDisable();    /* never leave the device armed (WEL=1) */
        return false;
    }

    if (!FLASH_bWaitReady())      /* WRSR is itself a self-timed write */
        return false;

    return (FLASH_u8ReadStatusReg() & FLASH_STATUS_PROTECTED) == 0U;
}

/* Set once at init from the JEDEC ID read. When false (no chip fitted, or
 * SPI2 floating with nothing attached), every operation below returns
 * immediately without touching SPI - avoiding the bounded-but-still-3s
 * FLASH_bWaitReady() wait on every single call (LOG_vInit alone probes
 * every sector, so that wait multiplied by FLASH_NUM_SECTORS would turn
 * boot into a multi-minute stall). */
static bool bDevicePresent = false;

/* Read Status Register 2 (35h) - SRP1 / QE / CMP. Diagnostic only. */
static uint8_t FLASH_u8ReadStatusReg2(void)
{
    uint8_t cmd = FLASH_CMD_READ_STATUS2;
    uint8_t sr2 = 0U;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    (void)FLASH_DRIVER_vRead(&sr2, 1);
    FLASH_DRIVER_vDeselect();
    return sr2;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void FLASH_vInit(void)
{
    FLASH_vLock();

    FLASH_vReleaseDeepPowerDown();
    osDelay(1);

    /* Read and report the raw JEDEC ID so a mismatch shows what came back:
     * 00s => no clock/data (AF or wiring), FFs => MISO idle / CS not asserting,
     * other => CPOL/CPHA or wrong part. */
    uint8_t cmd   = FLASH_CMD_JEDEC_ID;
    uint8_t id[3] = {0};
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vRead(id, 3);
    FLASH_DRIVER_vDeselect();



    bDevicePresent = (id[0] == FLASH_MANUFACTURER_ID);

    if (!bDevicePresent)
    {
        DBG("FLASH: JEDEC ID mismatch - got %02X %02X %02X (expected mfr %02X) - "
            "no flash fitted, flash logging disabled\r\n",
            id[0], id[1], id[2], FLASH_MANUFACTURER_ID);
        FLASH_vUnlock();
        return;
    }

    DBG("FLASH: JEDEC ID %02X %02X %02X OK\r\n", id[0], id[1], id[2]);

    /* Report protection state and self-heal a chip stuck write-protected.
     * Block protection (BP0..BP4) and SRP0 are non-volatile and block
     * program/erase while leaving reads working, so a device that got into
     * this state would otherwise freeze its log and ignore chip-erase forever.
     * SR2 (SRP1/CMP) distinguishes the recoverable Hardware-Protected case
     * from the permanent OTP case, and is needed to interpret BP. */
    uint8_t sr1 = FLASH_u8ReadStatusReg();
    uint8_t sr2 = FLASH_u8ReadStatusReg2();

    DBG("FLASH: SR1=%02X (BP=%02X SRP0=%u) SR2=%02X (SRP1=%u QE=%u CMP=%u)\r\n",
        sr1,
        (unsigned)((sr1 & FLASH_SR1_BP_MASK) >> 2),
        (unsigned)((sr1 & FLASH_SR1_SRP0) ? 1U : 0U),
        sr2,
        (unsigned)((sr2 & FLASH_SR2_SRP1) ? 1U : 0U),
        (unsigned)((sr2 & FLASH_SR2_QE)   ? 1U : 0U),
        (unsigned)((sr2 & FLASH_SR2_CMP)  ? 1U : 0U));

    if (sr1 & FLASH_STATUS_PROTECTED)
    {
        DBG("FLASH: write-protected - performing global unprotect\r\n");
        if (FLASH_bGlobalUnprotect())
            DBG("FLASH: unprotected, SR1=%02X\r\n", FLASH_u8ReadStatusReg());
        else
            DBG("FLASH: unprotect FAILED, SR1=%02X - SR hardware-locked, "
                "drive WP# pin HIGH and retry\r\n", FLASH_u8ReadStatusReg());
    }

    /* Start from a known-safe, disarmed state: with WP# grounded the only thing
     * standing between us and a permanent write-lock is never accepting a stray
     * WRSR, which requires WEL=0 at idle. */
    FLASH_vWriteDisable();

//    FLASH_vChipErase();

    FLASH_vUnlock();
}

bool FLASH_bDeviceBusy(void)
{
    FLASH_vLock();
    bool bBusy = (FLASH_u8ReadStatusReg() & FLASH_STATUS_WIP) != 0U;
    FLASH_vUnlock();
    return bBusy;
}

uint8_t FLASH_u8ReadStatusReg(void)
{
    FLASH_vLock();
    FLASH_vEnsureAwake();   /* chokepoint: every read/write/erase polls here */
    uint8_t cmd = FLASH_CMD_READ_STATUS;
    /* Default to "busy" so that a failed/short SPI read can never be
     * mistaken for "ready" — FLASH_bWaitReady would otherwise issue the
     * next command onto a device that might still be mid-program/erase. */
    uint8_t status = FLASH_STATUS_WIP;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    if (FLASH_DRIVER_vRead(&status, 1) != HAL_OK)
        status = FLASH_STATUS_WIP;
    FLASH_DRIVER_vDeselect();
    FLASH_vUnlock();
    return status;
}

bool FLASH_bVerifyDevice(void)
{
    return FLASH_bVerifyDeviceEx(NULL, NULL);
}

bool FLASH_bVerifyDeviceEx(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    uint8_t id[3] = {0};
    bool    bOk   = false;
    uint8_t u8Try = 0U;

    FLASH_vLock();
    FLASH_vEnsureAwake();

    /* Retried, not single-shot. A single JEDEC read can come back wrong while
     * the part is perfectly healthy — the same transient this codebase already
     * documents elsewhere (see OTA_LORA_CHUNK_GAP_MS: a supply-rail sag makes
     * a too-soon flash read return corrupt bytes). Reporting FAIL off one bad
     * read was declaring every unit's flash dead while it was demonstrably
     * reading, writing and holding the log fine.
     *
     * The raw bytes are handed back so the caller can log what actually came
     * off the wire, which is what distinguishes the cases:
     *   00 00 00 -> no clock/data (AF, wiring, bus not restored)
     *   FF FF FF -> MISO idle / CS never asserted
     *   other    -> CPOL/CPHA, timing, or genuinely the wrong part */
    for (u8Try = 1U; u8Try <= FLASH_JEDEC_READ_ATTEMPTS; u8Try++)
    {
        uint8_t cmd = FLASH_CMD_JEDEC_ID;

        id[0] = id[1] = id[2] = 0U;
        FLASH_DRIVER_vSelect();
        FLASH_DRIVER_vWrite(&cmd, 1);
        FLASH_DRIVER_vRead(id, 3);
        FLASH_DRIVER_vDeselect();

        if (id[0] == FLASH_MANUFACTURER_ID) { bOk = true; break; }

        osDelay(2);   /* let the rail settle before re-reading */
    }

    FLASH_vUnlock();

    if (pu8Id != NULL)
    {
        pu8Id[0] = id[0]; pu8Id[1] = id[1]; pu8Id[2] = id[2];
    }
    if (pu8Attempts != NULL)
        *pu8Attempts = u8Try > FLASH_JEDEC_READ_ATTEMPTS ? FLASH_JEDEC_READ_ATTEMPTS : u8Try;

    return bOk;
}

bool FLASH_vRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
    FLASH_vLock();
    if (!bDevicePresent)
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_READ,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 4) == HAL_OK) &&
               (FLASH_DRIVER_vRead(buf, len) == HAL_OK);
    FLASH_DRIVER_vDeselect();
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vPageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    FLASH_vLock();
    if (!bDevicePresent)
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_PAGE_PROGRAM,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 4) == HAL_OK) &&
               (FLASH_DRIVER_vWrite(buf, len) == HAL_OK);
    FLASH_DRIVER_vDeselect();
    if (!bOk)
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vSectorErase(uint32_t addr)
{
    FLASH_vLock();
    if (!bDevicePresent)
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_SECTOR_ERASE,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 4) == HAL_OK);
    FLASH_DRIVER_vDeselect();
    if (!bOk)
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vBlockErase(uint32_t addr)
{
    FLASH_vLock();
    if (!bDevicePresent)
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_BLOCK_ERASE,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 4) == HAL_OK);
    FLASH_DRIVER_vDeselect();
    if (!bOk)
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vChipErase(void)
{
    FLASH_vLock();
    if (!bDevicePresent)
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd = FLASH_CMD_CHIP_ERASE;
    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    /* A chip erase is exactly the operation used to recover a stuck device, so
     * clear any latched block protection first - otherwise the erase is
     * silently ignored and the old contents survive. Only touches the SR when
     * protection is actually set, to avoid needless status-register wear. */
    if (FLASH_u8ReadStatusReg() & FLASH_STATUS_PROTECTED)
        (void)FLASH_bGlobalUnprotect();

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(&cmd, 1) == HAL_OK);
    FLASH_DRIVER_vDeselect();
    if (!bOk)
    {
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
        FLASH_vUnlock();
        return false;
    }

    bool bReady = FLASH_bWaitReadyTimeout(FLASH_CHIP_ERASE_TIMEOUT_MS);
    FLASH_vUnlock();
    return bReady;
}

void FLASH_vInhibitDeepPowerDown(bool bInhibit)
{
    FLASH_vLock();
    bDpdInhibited = bInhibit;
    if (bInhibit)
        FLASH_vEnsureAwake();   /* wake now; keep it awake for the session */
    FLASH_vUnlock();
}

void FLASH_vDeepPowerDown(void)
{
    FLASH_vLock();
    if (bDpdInhibited)
    {
        FLASH_vUnlock();
        return;     /* OTA in progress — keep the chip awake */
    }
    if (bInDpd)
    {
        FLASH_vUnlock();
        return;     /* already parked — don't churn SPI */
    }
    uint8_t cmd = FLASH_CMD_DEEP_PWR_DOWN;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
    bInDpd = true;
    FLASH_vUnlock();
}

void FLASH_vReleaseDeepPowerDown(void)
{
    FLASH_vLock();
    uint8_t cmd = FLASH_CMD_RESUME;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
    bInDpd = false;
    FLASH_vUnlock();
}
