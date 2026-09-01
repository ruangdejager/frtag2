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

/* Read-failure telemetry — see Flash_ReadFailReason_t in Flash.h for why a
 * bare false return was not enough. Recorded under the flash mutex by
 * FLASH_vRead, read out (and cleared) by whoever is diagnosing. Details are
 * kept for the FIRST failure of a batch, so a later cascade cannot bury what
 * actually went wrong first. */
static volatile uint16_t               u16RdFailCount  = 0U;
static volatile Flash_ReadFailReason_t tRdFailReason   = FLASH_RDFAIL_NONE;
static volatile uint32_t               u32RdFailAddr   = 0U;
static volatile uint8_t                u8RdFailHal     = 0U;

/* Call with the flash mutex held. */
static void FLASH_vRecordReadFail(Flash_ReadFailReason_t tReason,
                                  uint32_t u32Addr, uint8_t u8Hal)
{
    if (u16RdFailCount == 0U)
    {
        tRdFailReason = tReason;
        u32RdFailAddr = u32Addr;
        u8RdFailHal   = u8Hal;
    }
    if (u16RdFailCount < UINT16_MAX) u16RdFailCount++;
}

uint16_t FLASH_u16GetAndClearReadFails(Flash_ReadFailReason_t *ptReason,
                                       uint32_t *pu32Addr,
                                       uint8_t  *pu8HalStatus)
{
    FLASH_vLock();
    uint16_t u16Count = u16RdFailCount;
    if (ptReason     != NULL) *ptReason     = tRdFailReason;
    if (pu32Addr     != NULL) *pu32Addr     = u32RdFailAddr;
    if (pu8HalStatus != NULL) *pu8HalStatus = u8RdFailHal;
    u16RdFailCount = 0U;
    tRdFailReason  = FLASH_RDFAIL_NONE;
    u32RdFailAddr  = 0U;
    u8RdFailHal    = 0U;
    FLASH_vUnlock();
    return u16Count;
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

/* Likewise a 64 KB block erase — well beyond a page program, well under a
 * chip erase. */
#define FLASH_BLOCK_ERASE_TIMEOUT_MS  10000U

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

/* The driver's usability gate. When false (no chip fitted, or SPI2 floating
 * with nothing attached), every operation below returns immediately without
 * touching SPI - avoiding the bounded-but-still-3s FLASH_bWaitReady() wait on
 * every single call (LOG_vInit alone probes every sector, so that wait
 * multiplied by FLASH_NUM_SECTORS would turn boot into a multi-minute stall).
 *
 * NOT latched for the life of the boot. It used to be: assigned once, from one
 * unretried JEDEC read in FLASH_vInit, and never revisited. A single transient
 * bad read therefore disabled logging and every FOTA erase/write until the unit
 * was physically power-cycled - which is what happened on 2026-08-31 (log
 * frozen mid-campaign for 18 h, FWDONE=ERR with 0 B delivered on every
 * subsequent session, while SelfTest kept printing flash=OK). Recovery now runs
 * from FLASH_bEnsurePresent() below, off ordinary traffic. */
static bool bDevicePresent = false;

/* Health/telemetry - see Flash_Health_t in Flash.h. Deliberately RAM + backup
 * register only: when the flash is the thing that is broken, the flash log is
 * not available to record why. */
static bool     bEverAbsent         = false;
static bool     bWriteProtectedSeen = false;
static bool     bUnprotectFailed    = false;
static bool     bEraseVerifyFail    = false;
static uint8_t  au8LastId[3]        = {0};
static uint8_t  u8LastAttempts      = 0U;
static uint16_t u16ProbeFailures    = 0U;
static uint16_t u16Recoveries       = 0U;
static uint16_t u16EraseVerifyFails = 0U;

/* Cooldown bookkeeping for the automatic re-probe. bProbedOnce distinguishes
 * "never probed" from "probed at tick 0", so the first recovery attempt is not
 * gated behind a full interval. */
static uint32_t u32LastProbeTick    = 0U;
static bool     bProbedOnce         = false;

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

/* Mirror the fault state into a TAMP backup register. Survives reset and
 * brownout, needs no external flash, and is readable by the next boot - the
 * only durable record available when the log device itself is the casualty.
 * The magic in the top bits keeps an uninitialised or unrelated value from
 * being mistaken for health data. */
static void FLASH_vPersistHealth(void)
{
    uint32_t u32Word = FLASH_HEALTH_BKP_MAGIC
                     | (bDevicePresent      ? FLASH_HEALTH_BIT_PRESENT     : 0UL)
                     | (bEverAbsent         ? FLASH_HEALTH_BIT_EVERABSENT  : 0UL)
                     | (bWriteProtectedSeen ? FLASH_HEALTH_BIT_PROTECTED   : 0UL)
                     | (bUnprotectFailed    ? FLASH_HEALTH_BIT_UNPROT_FAIL : 0UL)
                     | (bEraseVerifyFail    ? FLASH_HEALTH_BIT_ERASEVFAIL  : 0UL)
                     | ((uint32_t)au8LastId[0] << 8);

    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP4R = u32Word;
}

/* Retried JEDEC-ID probe. Call with the flash mutex held and the device awake.
 * Single source of truth for "is the right part answering" - FLASH_vInit,
 * FLASH_bVerifyDeviceEx and the recovery path all route through this, so none
 * of them can drift back to a one-shot read. */
static bool FLASH_bProbeLocked(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    uint8_t id[3] = {0};
    bool    bOk   = false;
    uint8_t u8Try = 0U;

    /* A single JEDEC read can come back wrong while the part is perfectly
     * healthy - the same transient this codebase already documents elsewhere
     * (see OTA_LORA_CHUNK_GAP_MS: a supply-rail sag makes a too-soon flash read
     * return corrupt bytes). Reporting FAIL off one bad read was declaring
     * every unit's flash dead while it was demonstrably reading, writing and
     * holding the log fine.
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

    if (u8Try > FLASH_JEDEC_READ_ATTEMPTS) u8Try = FLASH_JEDEC_READ_ATTEMPTS;

    au8LastId[0]   = id[0];
    au8LastId[1]   = id[1];
    au8LastId[2]   = id[2];
    u8LastAttempts = u8Try;
    if (!bOk && (u16ProbeFailures < UINT16_MAX)) u16ProbeFailures++;

    if (pu8Id != NULL) { pu8Id[0] = id[0]; pu8Id[1] = id[1]; pu8Id[2] = id[2]; }
    if (pu8Attempts != NULL) *pu8Attempts = u8Try;

    return bOk;
}

/* Post-identification bring-up: report protection state, self-heal a chip stuck
 * write-protected, and leave the device disarmed. Call with the mutex held,
 * only once the part has answered. Factored out of FLASH_vInit so the recovery
 * path gets it too - a device whose gate re-opens mid-run has not been through
 * init and may never have had its protection bits cleared. */
static void FLASH_vBringUpLocked(void)
{
    /* Block protection (BP0..BP4) and SRP0 are non-volatile and block
     * program/erase while leaving reads working, so a device that got into this
     * state would otherwise freeze its log and ignore chip-erase forever.
     * SR2 (SRP1/CMP) distinguishes the recoverable Hardware-Protected case from
     * the permanent OTP case, and is needed to interpret BP. */
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
        bWriteProtectedSeen = true;
        DBG("FLASH: write-protected - performing global unprotect\r\n");
        if (FLASH_bGlobalUnprotect())
        {
            bUnprotectFailed = false;
            DBG("FLASH: unprotected, SR1=%02X\r\n", FLASH_u8ReadStatusReg());
        }
        else
        {
            bUnprotectFailed = true;
            DBG("FLASH: unprotect FAILED, SR1=%02X - SR hardware-locked, "
                "drive WP# pin HIGH and retry\r\n", FLASH_u8ReadStatusReg());
        }
    }

    /* Start from a known-safe, disarmed state: with WP# grounded the only thing
     * standing between us and a permanent write-lock is never accepting a stray
     * WRSR, which requires WEL=0 at idle. */
    FLASH_vWriteDisable();
}

/* The gate, with self-recovery. Call with the mutex held. Cheap in the normal
 * case (one bool test); when the gate is shut it re-probes at most once per
 * FLASH_REPROBE_INTERVAL_MS and, on success, brings the device back up and
 * re-opens the gate.
 *
 * This is what makes a transient survivable without a power cycle - the whole
 * point, given a field unit cannot be reprogrammed by wire and depends on FOTA
 * to ever be fixed again. A genuinely absent chip still costs only one
 * throttled probe per interval, which is what the gate was protecting against
 * in the first place. */
static bool FLASH_bEnsurePresent(void)
{
    if (bDevicePresent)
        return true;

    uint32_t u32Now = osKernelGetTickCount();
    if (bProbedOnce && ((u32Now - u32LastProbeTick) < FLASH_REPROBE_INTERVAL_MS))
        return false;

    u32LastProbeTick = u32Now;
    bProbedOnce      = true;

    FLASH_vEnsureAwake();
    if (!FLASH_bProbeLocked(NULL, NULL))
    {
        FLASH_vPersistHealth();
        return false;
    }

    bDevicePresent = true;
    if (u16Recoveries < UINT16_MAX) u16Recoveries++;
    DBG("FLASH: device recovered - JEDEC %02X %02X %02X, gate re-opened "
        "(%u failed probes since boot)\r\n",
        au8LastId[0], au8LastId[1], au8LastId[2], (unsigned)u16ProbeFailures);

    FLASH_vBringUpLocked();
    FLASH_vPersistHealth();
    return true;
}

/* Confirm an erase actually blanked the region. Call with the mutex held, after
 * the erase has completed. Uses a raw read rather than FLASH_vRead so a verify
 * failure does not pollute the read-failure telemetry with something that is
 * not a read fault. Returns false if the bytes are not all-ones, or if the
 * read-back itself could not be performed - in both cases the erase must not be
 * reported as successful. */
static bool FLASH_bVerifyErasedLocked(uint32_t u32Addr, uint32_t u32Len)
{
    uint8_t  au8Buf[FLASH_ERASE_VERIFY_BYTES];
    uint16_t u16Chunk = (uint16_t)((u32Len < (uint32_t)FLASH_ERASE_VERIFY_BYTES)
                                   ? u32Len : (uint32_t)FLASH_ERASE_VERIFY_BYTES);

    /* Sample the first and last chunk of the region. A protected device fails
     * uniformly so one sample would do, but a partially-completed erase (a
     * power dip mid-operation) leaves the tail unerased - and the tail is
     * exactly what a sequential writer fills last and would notice last. */
    uint32_t au32Offsets[2];
    au32Offsets[0] = 0UL;
    au32Offsets[1] = (u32Len > (uint32_t)u16Chunk) ? (u32Len - (uint32_t)u16Chunk) : 0UL;

    if (u16Chunk == 0U)
        return true;

    for (uint8_t i = 0U; i < 2U; i++)
    {
        uint32_t u32At = u32Addr + au32Offsets[i];
        uint8_t  cmd[4] = {
            FLASH_CMD_READ,
            (uint8_t)((u32At >> 16) & 0xFFU),
            (uint8_t)((u32At >>  8) & 0xFFU),
            (uint8_t)((u32At      ) & 0xFFU),
        };

        FLASH_DRIVER_vSelect();
        HAL_StatusTypeDef tCmd  = FLASH_DRIVER_vWrite(cmd, 4);
        HAL_StatusTypeDef tData = (tCmd == HAL_OK)
                                ? FLASH_DRIVER_vRead(au8Buf, u16Chunk) : HAL_OK;
        FLASH_DRIVER_vDeselect();

        if ((tCmd != HAL_OK) || (tData != HAL_OK))
            return false;

        for (uint16_t j = 0U; j < u16Chunk; j++)
        {
            if (au8Buf[j] != FLASH_ERASED_BYTE)
                return false;
        }

        if (au32Offsets[1] == au32Offsets[0])
            break;   /* region smaller than one chunk - one sample covers it */
    }

    return true;
}

/* Issue one erase command and wait for it to complete. Call with the mutex held
 * and the device known present. Shared by all three erase entry points so the
 * WREN / command / disarm-on-failure / wait sequence exists in exactly one
 * place - and so the retry below re-issues an identical operation. */
static bool FLASH_bIssueEraseLocked(const uint8_t *pu8Cmd, uint8_t u8CmdLen,
                                    uint32_t u32TimeoutMs)
{
    if (!FLASH_bWaitReady())
        return false;

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite((uint8_t *)pu8Cmd, u8CmdLen) == HAL_OK);
    FLASH_DRIVER_vDeselect();

    if (!bOk)
    {
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
        return false;
    }

    /* Commit before returning - see FLASH_vPageWrite for why a self-contained
     * write matters. */
    return FLASH_bWaitReadyTimeout(u32TimeoutMs);
}

/* Erase a region and PROVE it happened, self-healing a protected chip.
 *
 * A block-protected device accepts the erase command, never raises WIP and
 * returns to ready, so the command sequence above succeeds while the old
 * contents survive untouched. Reporting that as a successful erase is how a
 * stale FOTA scratch region gets written over and only fails much later at the
 * whole-image XOR - or not at all.
 *
 * On a verify failure the prime suspect is latched block protection, so it is
 * cleared and the erase retried once, here, rather than surfacing a failure the
 * caller would only retry on the next campaign. A field unit cannot be
 * reprogrammed by wire, so it is worth spending one extra erase to avoid
 * needing another whole FOTA window. */
static bool FLASH_bEraseRegionLocked(const uint8_t *pu8Cmd, uint8_t u8CmdLen,
                                     uint32_t u32Base, uint32_t u32Len,
                                     uint32_t u32TimeoutMs)
{
    if (!FLASH_bIssueEraseLocked(pu8Cmd, u8CmdLen, u32TimeoutMs))
        return false;

    if (FLASH_bVerifyErasedLocked(u32Base, u32Len))
        return true;

    uint8_t sr1 = FLASH_u8ReadStatusReg();
    DBG("FLASH: erase @%06lX len %lu did NOT blank - SR1=%02X%s\r\n",
        (unsigned long)u32Base, (unsigned long)u32Len, sr1,
        (sr1 & FLASH_STATUS_PROTECTED) ? " (block-protected)" : "");

    bool bRetryWorthwhile = false;
    if (sr1 & FLASH_STATUS_PROTECTED)
    {
        bWriteProtectedSeen = true;
        if (FLASH_bGlobalUnprotect())
        {
            bUnprotectFailed = false;
            bRetryWorthwhile = true;
            DBG("FLASH: protection cleared - retrying the erase once\r\n");
        }
        else
        {
            bUnprotectFailed = true;
            DBG("FLASH: unprotect FAILED - SR hardware-locked, drive WP# HIGH\r\n");
        }
    }

    if (bRetryWorthwhile &&
        FLASH_bIssueEraseLocked(pu8Cmd, u8CmdLen, u32TimeoutMs) &&
        FLASH_bVerifyErasedLocked(u32Base, u32Len))
    {
        DBG("FLASH: erase @%06lX succeeded after clearing protection\r\n",
            (unsigned long)u32Base);
        /* Still recorded: the chip needed rescuing, and a unit that keeps
         * needing it is telling us something the success return hides. */
        bEraseVerifyFail = true;
        if (u16EraseVerifyFails < UINT16_MAX) u16EraseVerifyFails++;
        FLASH_vPersistHealth();
        return true;
    }

    bEraseVerifyFail = true;
    if (u16EraseVerifyFails < UINT16_MAX) u16EraseVerifyFails++;
    FLASH_vPersistHealth();
    return false;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void FLASH_vInit(void)
{
    FLASH_vLock();

    /* Report what the PREVIOUS boot left behind before overwriting it. This is
     * the only fault record that survives a reset when the flash itself is the
     * casualty — the flash log obviously cannot hold "the flash stopped
     * working", and the UART line that used to carry it is unavailable on a
     * deployed unit. A watchdog reset or brownout otherwise erases all evidence
     * that the tag had been running gated-off. */
    uint32_t u32Prev = TAMP->BKP4R;
    if ((u32Prev & FLASH_HEALTH_BKP_MAGIC_MASK) == FLASH_HEALTH_BKP_MAGIC)
    {
        if ((u32Prev & (FLASH_HEALTH_BIT_EVERABSENT |
                        FLASH_HEALTH_BIT_PROTECTED  |
                        FLASH_HEALTH_BIT_UNPROT_FAIL|
                        FLASH_HEALTH_BIT_ERASEVFAIL)) != 0UL)
        {
            DBG("FLASH: previous boot health - present=%u everAbsent=%u wprot=%u "
                "unprotFail=%u eraseVfyFail=%u lastMfr=%02X\r\n",
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_PRESENT)     ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_EVERABSENT)  ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_PROTECTED)   ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_UNPROT_FAIL) ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_ERASEVFAIL)  ? 1U : 0U),
                (unsigned)((u32Prev >> 8) & 0xFFU));
        }
    }

    FLASH_vReleaseDeepPowerDown();
    osDelay(1);

    /* RETRIED, never a single shot. This one assignment decides whether every
     * write and erase in the driver is permitted, so it must not be decided by
     * a read that this codebase already documents as untrustworthy in
     * isolation. The unretried version here is what gated a healthy tag's flash
     * off for 18 h on 2026-08-31: one bad read at boot, log frozen, and every
     * FOTA erase failing instantly with FWDONE=ERR / 0 B delivered.
     *
     * The raw bytes are still reported on a mismatch so a genuine fault stays
     * diagnosable: 00s => no clock/data (AF or wiring), FFs => MISO idle / CS
     * not asserting, other => CPOL/CPHA or wrong part. */
    uint8_t id[3]      = {0};
    uint8_t u8Attempts = 0U;
    uint8_t u8Round    = 0U;

    for (u8Round = 1U; u8Round <= FLASH_BOOT_PROBE_ROUNDS; u8Round++)
    {
        bDevicePresent = FLASH_bProbeLocked(id, &u8Attempts);
        if (bDevicePresent)
            break;
        if (u8Round < FLASH_BOOT_PROBE_ROUNDS)
            osDelay(FLASH_BOOT_PROBE_SETTLE_MS);
    }

    bProbedOnce      = true;
    u32LastProbeTick = osKernelGetTickCount();

    if (!bDevicePresent)
    {
        bEverAbsent = true;
        DBG("FLASH: JEDEC ID mismatch after %u rounds x %u attempts - got "
            "%02X %02X %02X (expected mfr %02X) - flash logging and FOTA staging "
            "disabled until a re-probe succeeds\r\n",
            (unsigned)FLASH_BOOT_PROBE_ROUNDS, (unsigned)FLASH_JEDEC_READ_ATTEMPTS,
            id[0], id[1], id[2], FLASH_MANUFACTURER_ID);
        FLASH_vPersistHealth();
        FLASH_vUnlock();
        return;
    }

    if ((u8Attempts > 1U) || (u8Round > 1U))
        DBG("FLASH: JEDEC ID %02X %02X %02X OK on round %u attempt %u "
            "(earlier read(s) transient - the chip is fine, the first read was not)\r\n",
            id[0], id[1], id[2], (unsigned)u8Round, (unsigned)u8Attempts);
    else
        DBG("FLASH: JEDEC ID %02X %02X %02X OK\r\n", id[0], id[1], id[2]);

    /* Report protection state and self-heal a chip stuck write-protected. */
    FLASH_vBringUpLocked();
    FLASH_vPersistHealth();

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
    FLASH_vLock();
    FLASH_vEnsureAwake();
    bool bOk = FLASH_bProbeLocked(pu8Id, pu8Attempts);
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_bDevicePresent(void)
{
    FLASH_vLock();
    bool bPresent = bDevicePresent;
    FLASH_vUnlock();
    return bPresent;
}

bool FLASH_bRecoverDevice(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    FLASH_vLock();
    FLASH_vEnsureAwake();

    bProbedOnce      = true;
    u32LastProbeTick = osKernelGetTickCount();

    bool bWasPresent = bDevicePresent;
    bool bNowPresent = FLASH_bProbeLocked(pu8Id, pu8Attempts);

    if (bNowPresent)
    {
        bDevicePresent = true;
        if (!bWasPresent)
        {
            if (u16Recoveries < UINT16_MAX) u16Recoveries++;
            DBG("FLASH: device recovered on demand - gate re-opened\r\n");
        }
        /* Re-run bring-up unconditionally: a chip that has just answered may
         * never have been through init (recovered mid-run), and one that has
         * may since have latched protection. Both cases need the SR checked and
         * the device left disarmed. */
        FLASH_vBringUpLocked();
    }
    else
    {
        bEverAbsent = true;

        /* Deliberately does NOT close an already-open gate.
         *
         * The two directions are not symmetric. Wrongly leaving the gate open
         * costs nothing - the individual operations fail on their own, bounded
         * and reported. Wrongly closing it disables logging and FOTA staging
         * wholesale, on a unit that cannot be reprogrammed by wire, which is
         * precisely the failure being fixed here. Since a probe can fail on a
         * perfectly healthy chip (that is the whole premise of the retry
         * logic), letting one failed probe shut a working driver would just
         * reintroduce the bug through a different door.
         *
         * A chip that has genuinely stopped answering still surfaces: reads
         * record FLASH_RDFAIL_* telemetry and erases fail their verify, both of
         * which are visible in the health word. */
        if (bWasPresent)
            DBG("FLASH: probe failed but gate left OPEN - a single failed probe "
                "is not evidence of a dead chip (id=%02X %02X %02X)\r\n",
                au8LastId[0], au8LastId[1], au8LastId[2]);
    }

    FLASH_vPersistHealth();
    FLASH_vUnlock();
    return bDevicePresent;
}

void FLASH_vGetHealth(Flash_Health_t *ptHealth)
{
    if (ptHealth == NULL) return;

    FLASH_vLock();
    ptHealth->bPresent            = bDevicePresent;
    ptHealth->bEverAbsent         = bEverAbsent;
    ptHealth->bWriteProtected     = bWriteProtectedSeen;
    ptHealth->bUnprotectFailed    = bUnprotectFailed;
    ptHealth->bEraseVerifyFail    = bEraseVerifyFail;
    ptHealth->au8LastId[0]        = au8LastId[0];
    ptHealth->au8LastId[1]        = au8LastId[1];
    ptHealth->au8LastId[2]        = au8LastId[2];
    ptHealth->u8LastAttempts      = u8LastAttempts;
    ptHealth->u16ProbeFailures    = u16ProbeFailures;
    ptHealth->u16Recoveries       = u16Recoveries;
    ptHealth->u16EraseVerifyFails = u16EraseVerifyFails;
    FLASH_vUnlock();
}

bool FLASH_vRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
    {
        FLASH_vRecordReadFail(FLASH_RDFAIL_ABSENT, addr, 0U);
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
        FLASH_vRecordReadFail(FLASH_RDFAIL_WAITREADY, addr, 0U);
        FLASH_vUnlock();
        return false;
    }

    /* Command and payload transfers kept separate rather than &&-chained, so
     * the tally can say which half failed: a command failure means the bus
     * refused a 4-byte write, a payload failure means it died partway through
     * a read the device had already accepted. Different faults. */
    FLASH_DRIVER_vSelect();
    HAL_StatusTypeDef tCmd  = FLASH_DRIVER_vWrite(cmd, 4);
    HAL_StatusTypeDef tData = (tCmd == HAL_OK) ? FLASH_DRIVER_vRead(buf, len) : HAL_OK;
    FLASH_DRIVER_vDeselect();

    bool bOk = (tCmd == HAL_OK) && (tData == HAL_OK);
    if (!bOk)
        FLASH_vRecordReadFail((tCmd != HAL_OK) ? FLASH_RDFAIL_CMD : FLASH_RDFAIL_DATA,
                              addr,
                              (uint8_t)((tCmd != HAL_OK) ? tCmd : tData));
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vPageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
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
    else
        /* Wait for the program to actually commit before returning. This used
         * to be deferred to the *next* operation's pre-wait, which meant the
         * last write of any burst was left in flight with nothing guarding it
         * — a concurrent deep-power-down or a reset in that window truncated
         * the page. Making the write self-contained is what removes that
         * whole class of silent corruption; the cost is the ~1-3 ms we were
         * already paying, just accounted to the write that incurs it. */
        bOk = FLASH_bWaitReady();
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vSectorErase(uint32_t addr)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
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

    /* Erased + verified, with a protection-clearing retry — see
     * FLASH_bEraseRegionLocked. The region checked is the whole sector the
     * address falls in, which is what the device actually erases. */
    bool bOk = FLASH_bEraseRegionLocked(
                   cmd, 4U,
                   addr & ~((uint32_t)FLASH_SECTOR_SIZE_BYTES - 1UL),
                   (uint32_t)FLASH_SECTOR_SIZE_BYTES,
                   FLASH_WAIT_READY_TIMEOUT_MS);
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vBlockErase(uint32_t addr)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
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

    /* A 64 KB erase is far slower than a sector erase, so it gets its own
     * budget rather than the generic pre-command one. Verified and retried the
     * same way — see FLASH_bEraseRegionLocked. */
    bool bOk = FLASH_bEraseRegionLocked(
                   cmd, 4U,
                   addr & ~((uint32_t)FLASH_BLOCK_SIZE_BYTES - 1UL),
                   (uint32_t)FLASH_BLOCK_SIZE_BYTES,
                   FLASH_BLOCK_ERASE_TIMEOUT_MS);
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vChipErase(void)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
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

    /* Verified like the other two, and doubly worth it here: chip erase is the
     * recovery operation, so silently "succeeding" while doing nothing is the
     * worst failure mode this driver has. Reading back all 512 KB to prove it
     * would be absurd, so FLASH_bVerifyErasedLocked samples the head and tail
     * of the device - where a protected or interrupted erase shows up. */
    bool bReady = FLASH_bEraseRegionLocked(
                      &cmd, 1U, 0UL,
                      (uint32_t)FLASH_NUM_SECTORS * (uint32_t)FLASH_SECTOR_SIZE_BYTES,
                      FLASH_CHIP_ERASE_TIMEOUT_MS);

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

    /* NEVER park the device over a live program/erase. 0xB9 issued while WIP
     * is set ABORTS the operation in progress, and the page it was committing
     * is left half-programmed: the bytes already burned stand, the rest stay
     * 0xFF. Nothing reports an error — the data is simply wrong from then on.
     *
     * This is reachable because the write primitives used to return with the
     * program still in flight (see FLASH_vPageWrite) while this call comes
     * from the DbgLog consumer on a *different* task (Log.c) — so a log drain
     * landing inside an OTA image write silently corrupted that page. The
     * mutex serialises the SPI commands but says nothing about WIP, so it
     * never prevented this on its own.
     *
     * Skip the park rather than block: parking is a pure power optimisation
     * and the caller is a background drain, so the next idle pass gets it. */
    if (FLASH_bDeviceBusy())
    {
        FLASH_vUnlock();
        return;
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
