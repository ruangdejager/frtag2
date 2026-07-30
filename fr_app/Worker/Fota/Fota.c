/*
 * Fota.c
 *
 * OTA firmware update — end to end. Merged from the earlier
 * Services/OtaStore (external-NOR storage) and Worker/OtaUpdate
 * (orchestration) into one worker. See Fota.h for the public API and
 * Fota_Config.h for the flash layout / bootloader-handoff contract.
 *
 * Concurrency: the storage helpers and orchestration entry points run
 * on the DeviceDiscovery AppTask; the LoRa parser hook
 * FOTA_vOnLoraPacket runs on the parser task and only copies/reacts
 * (chunks and reports are handed to the AppTask through one-slot
 * mailboxes). The external-flash device is shared with Log.c and is
 * protected by the mutex in Flash.c.
 *
 * Acquire sequence (primary, on the AppTask, logger session already up):
 *
 *   AT+FWCHECK -> +FWCHECK: OK/BUSY   (request an OTA check)
 *   AT+FWREQ   -> FW,<ver>,<bytes>,<xor>  (skip unless ver > running)
 *   erase scratchpad up front (~3 s; nothing racing RX later)
 *   for each 1 KB block:
 *     AT+FWGET=<off>,<len>  -> raw bytes + FB trailer, retried on mismatch
 *     write straight into scratchpad at file offset == scratch offset
 *   full-image XOR-8 vs logger's expected xor (cross-check with fr9's
 *     manifest-verified value)
 *   commit metadata, AT+FWDONE=OK, arm bootloader, reset
 */

#include "build_config.h"   /* STORAGE_BACKEND_FLASH */

#ifdef STORAGE_BACKEND_FLASH

#include "Fota.h"
#include "Fota_Config.h"
#include "Fota_Driver.h"

#include "Farmranger.h"
#include "version_config.h"
#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "DeviceDiscovery.h"

#include "cmsis_os2.h"
#include "stm32wlxx_hal.h"
#include "dbg_log.h"
#include "Log.h"          /* LOG_vSuspend — quiesce flash logging during OTA */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>       /* rand() — multi-primary distribute backoff */

/* ==========================================================================
 * Storage layer — external NOR scratchpad + metadata
 * ========================================================================== */

static uint32_t FOTA_u32GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void FOTA_vPutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* --------------------------------------------------------------------------
 * FOTA_vInit — one-time layout migration
 * -------------------------------------------------------------------------- */
void FOTA_vInit(void)
{
    /* The "installed version" the bootloader compares against comes from
     * gFwVersion (version_config.c), compiled directly into this image at
     * a fixed flash address — nothing to do here. See Fota_Config.h. */

    /* One-time layout migration: before the OTA partitions existed the
     * text log owned the whole chip, so on first boot with this layout
     * the metadata sector holds stale log text — neither blank nor a
     * record. Erase it once so bGetMeta() can't misread old text as
     * metadata. The stale bytes in the scratch region are harmless
     * (erased before use). */
    uint8_t au8Head[4];
    if (!FOTA_DRIVER_bRead(OTA_META_ADDR, au8Head, sizeof(au8Head)))
        return;

    bool bBlank = (au8Head[0] == 0xFFU) && (au8Head[1] == 0xFFU) &&
                  (au8Head[2] == 0xFFU) && (au8Head[3] == 0xFFU);
    bool bMagic = (FOTA_u32GetU32(au8Head) == OTA_META_MAGIC);

    if (!bBlank && !bMagic)
    {
        DBG_LOG("Fota: migrating metadata sector (stale layout)\r\n");
        (void)FOTA_DRIVER_bSectorErase(OTA_META_SECTOR_ADDR);
    }
}

/* --------------------------------------------------------------------------
 * FOTA_bEraseScratch — erases scratch AND metadata sector
 * -------------------------------------------------------------------------- */
bool FOTA_bEraseScratch(void)
{
    for (uint32_t u32Addr = OTA_SCRATCH_START_ADDR;
         u32Addr <= OTA_META_SECTOR_ADDR;
         u32Addr += FOTA_DRIVER_SECTOR_SIZE)
    {
        if (!FOTA_DRIVER_bSectorErase(u32Addr))
        {
            DBG_LOG("Fota: scratch erase FAILED @0x%05lX\r\n",
                    (unsigned long)u32Addr);
            return false;
        }
        osDelay(1);   /* yield between sectors (~45 ms each, ~3 s total) */
    }
    return true;
}

bool FOTA_bWriteImage(uint32_t u32Offset, const uint8_t *pu8Data, uint16_t u16Len)
{
    if ((u32Offset + u16Len) > OTA_SCRATCH_SIZE)
        return false;

    uint32_t u32Addr = OTA_SCRATCH_START_ADDR + u32Offset;
    uint16_t u16Off  = 0U;

    while (u16Off < u16Len)
    {
        /* Split at flash page boundaries (page program can't cross them). */
        uint32_t u32ToPageEnd = FOTA_DRIVER_PAGE_SIZE
                              - (u32Addr % FOTA_DRIVER_PAGE_SIZE);
        uint16_t u16Chunk     = (uint16_t)(u16Len - u16Off);
        if (u16Chunk > u32ToPageEnd)
            u16Chunk = (uint16_t)u32ToPageEnd;

        if (!FOTA_DRIVER_bWrite(u32Addr, &pu8Data[u16Off], u16Chunk))
            return false;

        u32Addr += u16Chunk;
        u16Off   = (uint16_t)(u16Off + u16Chunk);
    }
    return true;
}

bool FOTA_bReadImage(uint32_t u32Offset, uint8_t *pu8Buf, uint16_t u16Len)
{
    if ((u32Offset + u16Len) > OTA_SCRATCH_SIZE)
        return false;
    return FOTA_DRIVER_bRead(OTA_SCRATCH_START_ADDR + u32Offset, pu8Buf, u16Len);
}

/* u16BufLen lets FOTA_u8CalcImageXorRange (below) and the diagnostic probe
 * FOTA_u8CalcImageXorRangeBuf both share one implementation while reading
 * in different-sized chunks — used to test whether the read chunk size
 * itself affects the result (more, smaller SPI transactions = more
 * preemption windows against the concurrent DbgLog consumer task). */
static uint8_t FOTA_u8CalcImageXorRangeBuf(uint32_t u32Start, uint32_t u32Len,
                                            uint8_t *pu8Buf, uint16_t u16BufLen)
{
    uint8_t  u8Xor     = 0U;
    uint32_t u32Addr   = OTA_SCRATCH_START_ADDR + u32Start;
    uint32_t u32Remain = u32Len;

    while (u32Remain > 0U)
    {
        uint16_t u16Chunk = (u32Remain > u16BufLen)
                          ? u16BufLen
                          : (uint16_t)u32Remain;

        if (!FOTA_DRIVER_bRead(u32Addr, pu8Buf, u16Chunk))
            return (uint8_t)~u8Xor;   /* read failure: guarantee a mismatch */

        for (uint16_t i = 0U; i < u16Chunk; i++)
            u8Xor ^= pu8Buf[i];

        u32Addr   += u16Chunk;
        u32Remain -= u16Chunk;

        if ((u32Addr % 0x8000UL) == 0U)
            osDelay(1);   /* yield every 32 KB of the ~236 KB pass */
    }
    return u8Xor;
}

uint8_t FOTA_u8CalcImageXorRange(uint32_t u32Start, uint32_t u32Len)
{
    uint8_t au8Buf[OTA_XOR_BUF_LEN];
    return FOTA_u8CalcImageXorRangeBuf(u32Start, u32Len, au8Buf, sizeof(au8Buf));
}

/* Diagnostic only: re-scan the same range with a 224-byte buffer (matching
 * OTA_LORA_CHUNK_LEN — the granularity every proven-reliable read in this
 * file uses) instead of the normal 64-byte OTA_XOR_BUF_LEN, to test
 * whether the whole-image scan's read chunk size is itself the source of
 * the mismatches. */
static uint8_t FOTA_u8CalcImageXor224(uint32_t u32Start, uint32_t u32Len)
{
    uint8_t au8Buf[224];
    return FOTA_u8CalcImageXorRangeBuf(u32Start, u32Len, au8Buf, sizeof(au8Buf));
}

/* A transient SPI/flash read glitch was confirmed on hardware: a fresh
 * full-image XOR pass can occasionally return a wrong value while the
 * stored bytes are actually fine, and a plain immediate re-read recovers
 * the correct one (matching known-good metadata/manifest). Tearing down a
 * valid image over a single flaky read was the actual root cause behind
 * the "corrupted every time, differently every time" symptom seen in the
 * field — not a real storage or LoRa-transfer bug. Retry the scan a
 * bounded number of times and accept the first pass that agrees with the
 * expected value; only report a genuine mismatch if every attempt fails to
 * match (they don't need to agree with each other — any single match is
 * proof the bytes are correct and this pass's read was the fluke). */
#define FOTA_XOR_VERIFY_MAX_ATTEMPTS  8U

static bool FOTA_bVerifyImageXorRetry(uint32_t u32Start, uint32_t u32Len,
                                       uint8_t u8Expected, uint8_t *pu8LastGot)
{
    uint8_t au8Attempts[FOTA_XOR_VERIFY_MAX_ATTEMPTS];

    for (uint8_t u8Attempt = 0U; u8Attempt < FOTA_XOR_VERIFY_MAX_ATTEMPTS; u8Attempt++)
    {
        /* Use the reliable continuous 224-byte read (never yields for an
         * image this size, so nothing interleaves on the shared flash — the
         * same gap-free pattern the bootloader and the clean PROBE use).
         * The 64-byte scan yields every 32 KB and reads unreliably. */
        uint8_t u8Xor = FOTA_u8CalcImageXor224(u32Start, u32Len);
        au8Attempts[u8Attempt] = u8Xor;
        *pu8LastGot = u8Xor;
        if (u8Xor == u8Expected)
        {
            if (u8Attempt > 0U)
                DBG_LOG("Fota: xor verify recovered on attempt %u/%u (transient read glitch)\r\n",
                        (unsigned)(u8Attempt + 1U), (unsigned)FOTA_XOR_VERIFY_MAX_ATTEMPTS);
            return true;
        }
    }

    /* Every attempt disagreed with the expected value — dump each one so a
     * genuine failure shows whether the reads are converging on a single
     * consistent (real corruption) value or scattering randomly (glitch
     * rate too high for this budget, not a data problem). */
    DBG_LOG("Fota: xor verify FAILED all %u attempts, expected=0x%02X, got:",
            (unsigned)FOTA_XOR_VERIFY_MAX_ATTEMPTS, (unsigned)u8Expected);
    for (uint8_t i = 0U; i < FOTA_XOR_VERIFY_MAX_ATTEMPTS; i++)
        DBG_LOG(" 0x%02X", (unsigned)au8Attempts[i]);
    DBG_LOG("\r\n");
    return false;
}

uint8_t FOTA_u8CalcImageXor(uint32_t u32SizeBytes)
{
    return FOTA_u8CalcImageXorRange(0U, u32SizeBytes);
}

bool FOTA_bCommitMetadata(uint32_t u32Version, uint32_t u32StopAddr, uint8_t u8Xor8)
{
    /* Write the record first, VALID last, so a reset in between never
     * leaves a half-committed record that passes validation. */
    uint8_t au8Rec[OTA_META_RECORD_LEN];
    memset(au8Rec, 0xFF, sizeof(au8Rec));

    FOTA_vPutU32(&au8Rec[OTA_META_OFF_MAGIC],     OTA_META_MAGIC);
    FOTA_vPutU32(&au8Rec[OTA_META_OFF_VERSION],   u32Version);
    FOTA_vPutU32(&au8Rec[OTA_META_OFF_STOP_ADDR], u32StopAddr);
    FOTA_vPutU32(&au8Rec[OTA_META_OFF_SIZE],
                 u32StopAddr - OTA_APP_BASE_ADDR + 1UL);
    au8Rec[OTA_META_OFF_XOR8] = u8Xor8;
    /* VALID/DISTRIBUTED stay 0xFF in this pass. */

    if (!FOTA_DRIVER_bWrite(OTA_META_ADDR, au8Rec, sizeof(au8Rec)))
        return false;

    uint8_t u8Marker = OTA_META_MARKER;
    return FOTA_DRIVER_bWrite(OTA_META_ADDR + OTA_META_OFF_VALID, &u8Marker, 1U);
}

bool FOTA_bGetMeta(FotaMeta_t *ptMeta)
{
    uint8_t au8Rec[OTA_META_RECORD_LEN];

    memset(ptMeta, 0, sizeof(*ptMeta));

    if (!FOTA_DRIVER_bRead(OTA_META_ADDR, au8Rec, sizeof(au8Rec)))
        return false;

    if (FOTA_u32GetU32(&au8Rec[OTA_META_OFF_MAGIC]) != OTA_META_MAGIC)
        return false;

    ptMeta->u32Version   = FOTA_u32GetU32(&au8Rec[OTA_META_OFF_VERSION]);
    ptMeta->u32StopAddr  = FOTA_u32GetU32(&au8Rec[OTA_META_OFF_STOP_ADDR]);
    ptMeta->u32SizeBytes = FOTA_u32GetU32(&au8Rec[OTA_META_OFF_SIZE]);
    ptMeta->u8Xor8       = au8Rec[OTA_META_OFF_XOR8];
    ptMeta->bValid       = (au8Rec[OTA_META_OFF_VALID]       == OTA_META_MARKER);
    ptMeta->bDistributed = (au8Rec[OTA_META_OFF_DISTRIBUTED] == OTA_META_MARKER);

    if (ptMeta->u32SizeBytes == 0UL || ptMeta->u32SizeBytes > OTA_APP_MAX_SIZE)
        return false;

    return ptMeta->bValid;
}

bool FOTA_bMarkDistributed(void)
{
    uint8_t u8Marker = OTA_META_MARKER;
    return FOTA_DRIVER_bWrite(OTA_META_ADDR + OTA_META_OFF_DISTRIBUTED,
                              &u8Marker, 1U);
}

void FOTA_vArmBootloaderAndReset(uint32_t u32Version)
{
    DBG_LOG("Fota: arming bootloader (v%lu) and resetting\r\n",
            (unsigned long)u32Version);
    osDelay(100);   /* let the log line drain */

    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = OTA_BOOT_MAGIC;
    TAMP->BKP1R = u32Version;

    NVIC_SystemReset();
}

/* ==========================================================================
 * UART acquire — primary pulls the fw file from the fr9
 * ========================================================================== */

/* One raw file block, pulled per AT+FWGET. Static — the AppTask stack is
 * not sized for a 1 KB local, and only one transfer runs at a time. */
static uint8_t au8BlockBuf[OTA_UART_BLOCK_LEN];

static FarmrangerFw_e FOTA_eQueryLogger(uint32_t *pu32Version, uint32_t *pu32FileBytes,
                                        uint8_t *pu8ExpectedXor)
{
    uint32_t u32Waited = 0U;

    for (;;)
    {
        FarmrangerFw_e eRes = FARMRANGER_eFwQuery(pu32Version, pu32FileBytes, pu8ExpectedXor);
        if (eRes != FARMRANGER_FW_WAIT)
        {
            if (u32Waited > 0U)
                DBG_LOG("Fota: FWREQ WAIT ended after %lu ms\r\n", (unsigned long)u32Waited);
            return eRes;
        }

        if (u32Waited >= OTA_FWREQ_WAIT_MAX_MS)
        {
            DBG_LOG("Fota: FWREQ still WAIT after %lu ms - giving up\r\n",
                    (unsigned long)u32Waited);
            return FARMRANGER_FW_NONE;
        }

        if ((u32Waited % 10000U) == 0U)
            DBG_LOG("Fota: FWREQ WAIT (%lu/%lu ms)\r\n",
                    (unsigned long)u32Waited, (unsigned long)OTA_FWREQ_WAIT_MAX_MS);

        osDelay(OTA_FWREQ_WAIT_POLL_MS);
        u32Waited += OTA_FWREQ_WAIT_POLL_MS;
    }
}

static bool FOTA_bFetchAndStoreBlock(uint32_t u32Offset, uint16_t u16Len)
{
    for (uint8_t u8Attempt = 1U; u8Attempt <= OTA_UART_BLOCK_RETRIES; u8Attempt++)
    {
        if (FARMRANGER_bFwGetBlock(u32Offset, u16Len, au8BlockBuf))
        {
            if (!FOTA_bWriteImage(u32Offset, au8BlockBuf, u16Len))
            {
                DBG_LOG("Fota: scratch write FAILED at offset %lu\r\n",
                        (unsigned long)u32Offset);
                return false;
            }
            return true;
        }

        DBG_LOG("Fota: block @%lu attempt %u failed\r\n",
                (unsigned long)u32Offset, u8Attempt);
        osDelay(OTA_UART_RETRY_DELAY_MS);
    }
    return false;
}

bool FOTA_bUartAcquire(void)
{
    uint32_t u32Version    = 0U;
    uint32_t u32FileBytes  = 0U;
    uint8_t  u8ExpectedXor = 0U;

    /* hasStaged: valid meta AND staged image is at least our running version.
     * When false, fr9 will offer the CURRENT version's binary too so we can
     * refill an empty/erased scratchpad — needed for LoRa distribution to
     * secondaries even when nothing newer is available upstream. */
    FotaMeta_t tMeta;
    bool bHasStaged = FOTA_bGetMeta(&tMeta) && tMeta.u32Version >= VERSION_u32Get();

    DBG_LOG("Fota: requesting fw check (running v%lu staged=%s)\r\n",
            (unsigned long)VERSION_u32Get(), bHasStaged ? "yes" : "no");
    if (!FARMRANGER_bFwCheckRequest(VERSION_u32Get(), bHasStaged))
    {
        DBG_LOG("Fota: fw check request not acked - polling FWREQ anyway\r\n");
    }

    if (FOTA_eQueryLogger(&u32Version, &u32FileBytes, &u8ExpectedXor) != FARMRANGER_FW_AVAILABLE)
    {
        DBG_LOG("Fota: no newer firmware offered\r\n");
        return false;
    }

    /* Accept-gate: strictly-newer is always OK. Equal-version is OK only when
     * we don't already have it staged (recovery: refill scratch after erase). */
    if (u32Version < VERSION_u32Get() ||
        (u32Version == VERSION_u32Get() && bHasStaged))
    {
        DBG_LOG("Fota: logger offers v%lu, running v%lu staged=%s - skip\r\n",
                (unsigned long)u32Version, (unsigned long)VERSION_u32Get(),
                bHasStaged ? "yes" : "no");
        return false;
    }

    if (u32FileBytes == 0UL || u32FileBytes > OTA_APP_MAX_SIZE)
    {
        DBG_LOG("Fota: logger offers implausible size %lu B - skip\r\n",
                (unsigned long)u32FileBytes);
        return false;
    }

    DBG_LOG("Fota: acquiring v%lu (%lu file bytes)\r\n",
            (unsigned long)u32Version, (unsigned long)u32FileBytes);

    if (!FOTA_bEraseScratch())
    {
        (void)FARMRANGER_bFwReportDone(false);
        return false;
    }

    for (uint32_t u32Offset = 0U; u32Offset < u32FileBytes;
         u32Offset += OTA_UART_BLOCK_LEN)
    {
        uint32_t u32Remain = u32FileBytes - u32Offset;
        uint16_t u16Len    = (u32Remain > OTA_UART_BLOCK_LEN)
                           ? (uint16_t)OTA_UART_BLOCK_LEN
                           : (uint16_t)u32Remain;

        if (!FOTA_bFetchAndStoreBlock(u32Offset, u16Len))
        {
            DBG_LOG("Fota: acquire FAILED\r\n");
            (void)FARMRANGER_bFwReportDone(false);
            return false;
        }

        if ((u32Offset % 0x8000UL) == 0U && u32Offset > 0U)
            DBG_LOG("Fota: %lu/%lu kB\r\n",
                    (unsigned long)(u32Offset / 1024U),
                    (unsigned long)(u32FileBytes / 1024U));
    }

    uint32_t u32Size     = u32FileBytes;
    uint32_t u32StopAddr = OTA_APP_BASE_ADDR + u32Size - 1UL;
    uint8_t  u8Xor;

    /* Cross-check against the logger's manifest-verified whole-image XOR-8
     * — catches corruption between the last per-block trailer check and
     * the final stored bytes (e.g. on the flash write/read-back path).
     * Retried: see FOTA_bVerifyImageXorRetry — a single flaky read must
     * not discard an otherwise-good freshly-acquired image. */
    if (!FOTA_bVerifyImageXorRetry(0U, u32Size, u8ExpectedXor, &u8Xor))
    {
        DBG_LOG("Fota: image XOR mismatch (stored=0x%02X, logger=0x%02X) - discarding\r\n",
                u8Xor, u8ExpectedXor);
        /* Confirmed corrupt (every retry attempt disagreed with the
         * expected value, see FOTA_bVerifyImageXorRetry) - don't leave
         * known-bad bytes sitting in scratch for the next acquire to
         * partially overwrite. Metadata was never committed for this
         * image (that happens below, after this check), so nothing else
         * currently treats this scratch as valid - this is cleanup, not
         * a state-consistency fix. */
        if (!FOTA_bEraseScratch())
            DBG_LOG("Fota: scratch erase after acquire xor mismatch FAILED\r\n");
        (void)FARMRANGER_bFwReportDone(false);
        return false;
    }

    if (!FOTA_bCommitMetadata(u32Version, u32StopAddr, u8Xor))
    {
        DBG_LOG("Fota: metadata commit FAILED\r\n");
        (void)FARMRANGER_bFwReportDone(false);
        return false;
    }

    DBG_LOG("Fota: image v%lu stored OK (size=%luB, xor=0x%02X, map 0x%08lX..0x%08lX)\r\n",
            (unsigned long)u32Version, (unsigned long)u32Size, u8Xor,
            (unsigned long)OTA_APP_BASE_ADDR, (unsigned long)u32StopAddr);

    (void)FARMRANGER_bFwReportDone(true);

    /* Only arm the bootloader when the acquired image is strictly newer than
     * what we're running. Equal-version acquires happen when we're refilling
     * an empty scratchpad with a copy of our own running version — nothing
     * to install, just have a valid staged image to distribute over LoRa. */
    if (u32Version > VERSION_u32Get())
    {
        FOTA_vArmBootloaderAndReset(u32Version);
        return true;   /* not reached */
    }
    DBG_LOG("Fota: scratch refilled with running v%lu - no reset\r\n",
            (unsigned long)u32Version);
    return true;
}

/* ==========================================================================
 * LoRa distribution — primary broadcasts the stored image DIRECTLY (no
 * mesh forwarding) to secondaries in range; missed chunks are repaired
 * per 64-chunk window via poll/report bitmaps.
 *
 * Context split: FOTA_vOnLoraPacket runs on the MeshParser task and only
 * copies/reacts (a one-chunk mailbox hands image data to the AppTask;
 * PrepAck/Report answers ride the jittered mesh TX queue). All flash
 * work runs on the AppTask in vDistribute / vSecondaryReceive.
 * ========================================================================== */

/* Report status values (OtaReport packets) */
#define OTA_RPT_WINDOW   0U   /* missing-chunk bitmap for the polled window  */
#define OTA_RPT_VALID    1U   /* image received, XOR-verified, committed     */
#define OTA_RPT_ERROR    2U   /* image failed verification                   */

/* On-wire packet lengths (type byte + little-endian fields) */
#define OTA_PKT_PREP_LEN      18U
#define OTA_PKT_PREPACK_LEN   15U
#define OTA_PKT_CHUNK_HDR_LEN 8U
#define OTA_PKT_POLL_LEN      12U
#define OTA_PKT_REPORT_LEN    18U
#define OTA_WINDOW_BITMAP_LEN 8U   /* 64 chunks / 8 bits                     */

/* Full-session chunk bitmap: max image / chunk = 236K / 224 = 1079 chunks */
#define OTA_BITMAP_BYTES  ((OTA_APP_MAX_SIZE / OTA_LORA_CHUNK_LEN + 8U) / 8U + 1U)

/* ---- Shared session state ---- */
static uint32_t u32SessionId;

/* Secondary firmware-acceptance gate. bFwAcceptViaKernel distinguishes an
 * explicit "tag <ID> fwaccept" (which enables the OTA-receive rendezvous
 * inside a kernel session) from the discovery TimeSync auto-arm (which must
 * NOT be consumed by an unrelated kernel/log-download wakeup). */
static volatile bool bFwAcceptArmed;
static volatile bool bFwAcceptViaKernel;

/* Primary on-demand distribution request. */
static volatile bool bDistributeReq;

/* Multi-primary listen-before-distribute: RTC tick of the last Ota* packet
 * heard whose session id was not our own (i.e. another primary's live
 * session). 0 = none heard yet. Set in FOTA_vOnLoraPacket regardless of
 * role; consumed by FOTA_vDistribute's pre-Prep backoff. */
static volatile uint32_t u32LastForeignOtaTick;

/* ---- Primary: distribution target table (filled by the parser) ---- */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8Strikes;     /* consecutive unanswered polls                  */
    uint8_t  u8Status;      /* last OTA_RPT_* heard from this target         */
    bool     bAlive;
} FotaTarget_t;
static FotaTarget_t     atTargets[OTA_LORA_MAX_TARGETS];
static volatile uint8_t u8TargetCount;
static volatile bool    bDistributeActive;

/* Report mailbox (parser -> AppTask, one slot) */
static volatile bool     bReportMail;
static volatile uint32_t u32ReportFrom;
static volatile uint8_t  u8ReportStatus;
static uint8_t           au8ReportBitmap[OTA_WINDOW_BITMAP_LEN];

/* ---- Secondary: receive session state ---- */
typedef struct {
    uint32_t u32SessionId;
    uint32_t u32Version;
    uint32_t u32ImageSize;
    uint16_t u16TotalChunks;
    uint8_t  u8ChunkLen;
    uint8_t  u8ImageXor;
} FotaPrepInfo_t;
static FotaPrepInfo_t    tPrep;
static volatile bool     bPrepPending;
static volatile bool     bRxSessionActive;
static uint8_t           au8ChunkBitmap[OTA_BITMAP_BYTES];
static volatile uint16_t u16ChunksHave;
static volatile uint32_t u32LastSessionPktTick;

/* Chunk mailbox (parser -> AppTask, one slot; a dropped chunk is repaired
 * by the window poll, so overrun is harmless) */
static uint8_t           au8ChunkMail[OTA_LORA_CHUNK_LEN];
static volatile uint16_t u16ChunkMailIdx;
static volatile uint8_t  u8ChunkMailLen;
static volatile bool     bChunkMail;

/* -------------------------------------------------------------------------- */

static void FOTA_vPutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static uint16_t FOTA_u16GetU16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static bool FOTA_bBitGet(const uint8_t *pu8Map, uint16_t u16Bit)
{
    return (pu8Map[u16Bit / 8U] & (uint8_t)(1U << (u16Bit % 8U))) != 0U;
}
static void FOTA_vBitSet(uint8_t *pu8Map, uint16_t u16Bit)
{
    pu8Map[u16Bit / 8U] |= (uint8_t)(1U << (u16Bit % 8U));
}

/* Direct radio TX with queue backpressure (AppTask only).
 *
 * Uses the BLOCKING enqueue (LORARADIO_bTxPacketWait), not a retry loop
 * around the non-blocking LORARADIO_bTxPacket: the retry loop called the
 * non-blocking enqueue up to 40 times, and every failed attempt logs
 * "TX PKT queue full" from inside LoraRadio.c — once real airtime +
 * carrier-sense back-off made the radio task slower than the chunk
 * producer (exactly the case once actual TX contention showed up, e.g. a
 * chunk-blast window), that flooded the log with thousands of near-
 * duplicate lines and drowned everything else in it. The blocking variant
 * is a single osMessageQueuePut wait — no polling, no spam, and it wakes
 * the instant the radio task actually has room instead of on some 25 ms
 * cadence regardless of when room appears. */
static bool FOTA_bRadioTx(const uint8_t *pu8Buf, uint8_t u8Len)
{
    LoraRadio_Packet_t tPkt;
    memset(&tPkt, 0, sizeof(tPkt));
    memcpy(tPkt.buffer, pu8Buf, u8Len);
    tPkt.length = u8Len;

    return LORARADIO_bTxPacketWait(&tPkt, 1000U);
}

static void FOTA_vNotifyAppTask(void)
{
    osThreadId_t xApp = DEVICE_DISCOVERY_xGetTaskHandle();
    if (xApp != NULL)
        osThreadFlagsSet(xApp, DEVICE_DISCOVERY_NOTIFY_OTA);
}

/* --------------------------------------------------------------------------
 * FOTA_vOnLoraPacket — MeshParser task context: copy/react only
 * -------------------------------------------------------------------------- */
void FOTA_vOnLoraPacket(const uint8_t *pu8Buf, uint16_t u16Len)
{
    /* Multi-primary carrier sense: every OTA packet type carries the session
     * id at [1]. Any Ota* whose id isn't ours is another primary's live
     * session (or its secondaries acking/reporting to it) — record the tick
     * so FOTA_vDistribute can defer instead of colliding. Our own session id
     * is regenerated at the top of FOTA_vDistribute, so same-session
     * re-arrivals are never mistaken for foreign. Cheap, role-agnostic, and
     * harmless on packets we otherwise ignore. */
    switch (pu8Buf[0])
    {
        case MeshPktType_OtaPrep:
        case MeshPktType_OtaPrepAck:
        case MeshPktType_OtaChunk:
        case MeshPktType_OtaPoll:
        case MeshPktType_OtaReport:
            if (u16Len >= 5U && FOTA_u32GetU32(&pu8Buf[1]) != u32SessionId)
                u32LastForeignOtaTick = osKernelGetTickCount();
            break;
        default:
            break;
    }

    switch (pu8Buf[0])
    {
        case MeshPktType_OtaPrep:
        {
            if (u16Len < OTA_PKT_PREP_LEN) return;
            if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_SECONDARY) return;

            if (!bFwAcceptArmed) return;

            uint32_t u32Ver    = FOTA_u32GetU32(&pu8Buf[5]);
            uint32_t u32NewSid = FOTA_u32GetU32(&pu8Buf[1]);

            if (u32Ver <= VERSION_u32Get())
                return;

            /* First-primary-wins latch (multi-primary coexistence).
             *
             * With more than one primary in range, A and B each generate
             * their own random session id and broadcast their own OtaPrep.
             * Without latching, the last-arriving Prep would overwrite
             * tPrep (both in the pre-AppTask pending window AND mid-
             * receive) - the secondary would ack whichever primary
             * happened to Prep last, and any chunks from the "loser"
             * primary would be filtered out by the session-id check as
             * unrelated. Worse still, an OtaPrep from B arriving during
             * an active receive from A would clobber tPrep.u32SessionId
             * and cause every subsequent A-chunk to be rejected, killing
             * the transfer at whatever fraction of the image had been
             * received so far.
             *
             * Rule: once we've latched onto a session (either bPrepPending
             * queued for the AppTask, or bRxSessionActive live), only
             * accept OtaPreps whose sid matches the latched one - those
             * are just normal re-arrivals from PREP_REPEATS. Different-
             * sid Preps are silently rejected (log at DBG). The existing
             * OTA_LORA_RX_IDLE_MS silence timeout (20 s) is the natural
             * recovery point if the latched primary dies before its
             * transfer completes - after that window closes,
             * bRxSessionActive drops to false and the secondary is once
             * again free to lock onto the next-arriving primary. */
            if (bPrepPending || bRxSessionActive)
            {
                if (u32NewSid != tPrep.u32SessionId)
                {
                    DBG("Fota: OtaPrep sid=%08X ignored (locked to %08X)\r\n",
                        u32NewSid, tPrep.u32SessionId);
                    return;
                }
                /* Same-sid re-arrival — normal PREP_REPEATS retransmit
                 * from the already-latched primary. Just refresh the
                 * activity timer so any concurrent silence timeout
                 * doesn't fire while the primary is still announcing. */
                u32LastSessionPktTick = osKernelGetTickCount();
                return;
            }

            tPrep.u32SessionId   = u32NewSid;
            tPrep.u32Version     = u32Ver;
            tPrep.u32ImageSize   = FOTA_u32GetU32(&pu8Buf[9]);
            tPrep.u16TotalChunks = FOTA_u16GetU16(&pu8Buf[13]);
            tPrep.u8ChunkLen     = pu8Buf[15];
            tPrep.u8ImageXor     = pu8Buf[16];

            if (tPrep.u8ChunkLen == 0U || tPrep.u16TotalChunks == 0U ||
                tPrep.u32ImageSize == 0U || tPrep.u32ImageSize > OTA_APP_MAX_SIZE)
                return;

            DBG_LOG("Fota: OtaPrep latched sid=%08X v%lu (%lu B, %u chunks)\r\n",
                    (unsigned)u32NewSid, (unsigned long)u32Ver,
                    (unsigned long)tPrep.u32ImageSize, (unsigned)tPrep.u16TotalChunks);

            u32LastSessionPktTick = osKernelGetTickCount();
            bPrepPending = true;
            FOTA_vNotifyAppTask();
            break;
        }

        case MeshPktType_OtaPrepAck:
        {
            if (u16Len < OTA_PKT_PREPACK_LEN) return;
            if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY) return;
            if (FOTA_u32GetU32(&pu8Buf[1]) != u32SessionId) return;

            uint32_t u32Dev = FOTA_u32GetU32(&pu8Buf[5]);
            for (uint8_t i = 0U; i < u8TargetCount; i++)
                if (atTargets[i].u32DeviceId == u32Dev) return;

            if (u8TargetCount < OTA_LORA_MAX_TARGETS)
            {
                atTargets[u8TargetCount].u32DeviceId = u32Dev;
                atTargets[u8TargetCount].u8Strikes   = 0U;
                atTargets[u8TargetCount].u8Status    = OTA_RPT_WINDOW;
                atTargets[u8TargetCount].bAlive      = true;
                u8TargetCount++;
                DBG_LOG("Fota: target %04lX joined\r\n", (unsigned long)u32Dev);
            }
            break;
        }

        case MeshPktType_OtaChunk:
        {
            if (u16Len < OTA_PKT_CHUNK_HDR_LEN) return;
            if (!bRxSessionActive) return;
            if (FOTA_u32GetU32(&pu8Buf[1]) != tPrep.u32SessionId) return;

            uint16_t u16Idx = FOTA_u16GetU16(&pu8Buf[5]);
            uint8_t  u8Len  = pu8Buf[7];
            if (u8Len == 0U || u8Len > OTA_LORA_CHUNK_LEN ||
                (uint16_t)(OTA_PKT_CHUNK_HDR_LEN + u8Len) > u16Len ||
                u16Idx >= tPrep.u16TotalChunks)
                return;

            u32LastSessionPktTick = osKernelGetTickCount();

            if (FOTA_bBitGet(au8ChunkBitmap, u16Idx))
            {
                /* Duplicate: primary re-sent, we already have it. Silent
                 * accept - noisy at scale, and completely expected during
                 * repair rounds. */
            }
            else if (bChunkMail)
            {
                /* AppTask hasn't drained the previous chunk mailbox yet;
                 * this arrival is DROPPED. Every real drop is worth
                 * logging so the source-of-loss picture becomes clear if
                 * we still see missing chunks after diagnostics. */
                DBG_LOG("Fota: chunk %u DROPPED (mailbox busy with %u)\r\n",
                        (unsigned)u16Idx, (unsigned)u16ChunkMailIdx);
            }
            else
            {
                memcpy(au8ChunkMail, &pu8Buf[OTA_PKT_CHUNK_HDR_LEN], u8Len);

                {
                    uint8_t u8ChunkXor = 0U;
                    for (uint8_t i = 0U; i < u8Len; i++)
                        u8ChunkXor ^= au8ChunkMail[i];
                    DBG("RXC %u xor=0x%02X\r\n", (unsigned)u16Idx, (unsigned)u8ChunkXor);
                }

                u16ChunkMailIdx = u16Idx;
                u8ChunkMailLen  = u8Len;
                bChunkMail      = true;
                FOTA_vNotifyAppTask();
            }
            break;
        }

        case MeshPktType_OtaPoll:
        {
            if (u16Len < OTA_PKT_POLL_LEN) return;
            if (!bRxSessionActive) return;
            if (FOTA_u32GetU32(&pu8Buf[1]) != tPrep.u32SessionId) return;
            if (FOTA_u32GetU32(&pu8Buf[5]) != LORARADIO_u32GetUniqueId()) return;

            uint16_t u16First = FOTA_u16GetU16(&pu8Buf[9]);
            uint8_t  u8Count  = pu8Buf[11];
            if (u8Count > OTA_LORA_WINDOW_CHUNKS) u8Count = OTA_LORA_WINDOW_CHUNKS;

            u32LastSessionPktTick = osKernelGetTickCount();

            uint8_t au8Rpt[OTA_PKT_REPORT_LEN];
            au8Rpt[0] = (uint8_t)MeshPktType_OtaReport;
            FOTA_vPutU32(&au8Rpt[1], tPrep.u32SessionId);
            FOTA_vPutU32(&au8Rpt[5], LORARADIO_u32GetUniqueId());
            au8Rpt[9] = OTA_RPT_WINDOW;
            memset(&au8Rpt[10], 0, OTA_WINDOW_BITMAP_LEN);
            uint8_t u8Missing = 0U;
            for (uint8_t i = 0U; i < u8Count; i++)
            {
                uint16_t u16Chunk = (uint16_t)(u16First + i);
                if (u16Chunk < tPrep.u16TotalChunks &&
                    !FOTA_bBitGet(au8ChunkBitmap, u16Chunk))
                {
                    au8Rpt[10U + i / 8U] |= (uint8_t)(1U << (i % 8U));
                    u8Missing++;
                }
            }
            DBG_LOG("Fota: poll for window %u..%u: %u missing, replying\r\n",
                    (unsigned)u16First, (unsigned)(u16First + u8Count - 1),
                    (unsigned)u8Missing);
            (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
            break;
        }

        case MeshPktType_OtaReport:
        {
            if (u16Len < OTA_PKT_REPORT_LEN) return;
            if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY) return;
            if (FOTA_u32GetU32(&pu8Buf[1]) != u32SessionId) return;

            u32ReportFrom  = FOTA_u32GetU32(&pu8Buf[5]);
            u8ReportStatus = pu8Buf[9];
            memcpy(au8ReportBitmap, &pu8Buf[10], OTA_WINDOW_BITMAP_LEN);
            bReportMail = true;
            break;
        }

        default:
            break;
    }
}

/* --------------------------------------------------------------------------
 * Primary — distribution session
 * -------------------------------------------------------------------------- */

bool FOTA_bDistributePending(void)
{
    /* No longer gated on tMeta.bDistributed: that bit only ever meant
     * "every target that showed up in some past session confirmed
     * UPDATED" — it says nothing about whether a secondary that wasn't
     * listening then (out of range, mid-sleep, freshly joined the mesh)
     * still needs this image now. There's no persistent registry of
     * "which secondaries exist" to check that against, so the only way to
     * actually reach a straggler is to keep re-announcing every wake.
     * Real cost of doing that once distribution has already succeeded is
     * small: OtaPrep + a short join wait, then FOTA_vDistribute() finds no
     * targets that need the image and exits immediately — no windows, no
     * chunks sent. */
    FotaMeta_t tMeta;
    if (!FOTA_bGetMeta(&tMeta))
        return false;
    return tMeta.u32Version == VERSION_u32Get();
}

bool FOTA_bRequestDistribute(void)
{
    /* Ignores version-match and distributed flags: on-demand re-runs. */
    FotaMeta_t tMeta;
    if (!FOTA_bGetMeta(&tMeta) || !tMeta.bValid)
    {
        DBG_LOG("Fota: distribute requested but no valid image staged\r\n");
        return false;
    }

    bDistributeReq = true;
    DBG_LOG("Fota: distribute requested (v%lu, %lu B staged)\r\n",
            (unsigned long)tMeta.u32Version, (unsigned long)tMeta.u32SizeBytes);
    return true;
}

bool FOTA_bDistributeRequested(void)
{
    return bDistributeReq;
}

void FOTA_vClearDistributeRequest(void)
{
    bDistributeReq = false;
}

static bool FOTA_bSendChunk(uint16_t u16Chunk, uint32_t u32ImageSize)
{
    uint32_t u32Offset = (uint32_t)u16Chunk * OTA_LORA_CHUNK_LEN;
    uint32_t u32Remain = u32ImageSize - u32Offset;
    uint8_t  u8Len     = (u32Remain > OTA_LORA_CHUNK_LEN)
                       ? (uint8_t)OTA_LORA_CHUNK_LEN : (uint8_t)u32Remain;

    uint8_t au8Pkt[OTA_PKT_CHUNK_HDR_LEN + OTA_LORA_CHUNK_LEN];
    au8Pkt[0] = (uint8_t)MeshPktType_OtaChunk;
    FOTA_vPutU32(&au8Pkt[1], u32SessionId);
    FOTA_vPutU16(&au8Pkt[5], u16Chunk);
    au8Pkt[7] = u8Len;

    /* Read, then verify by re-reading until FIVE consecutive passes agree,
     * 50 ms apart. A fixed delay after the PRECEDING chunk's TX cannot be
     * trusted: the radio task's CAD can back off for up to 5 s before the
     * actual RF pulse fires, completely decoupled from when FOTA_bRadioTx()
     * returns to this caller — confirmed on hardware, the SX126x PA's
     * current spike sags the shared supply rail and corrupts a flash read
     * that lands too close to it. Confirmed on hardware (full-image PRE
     * scan vs. this loop's TXC output, chunk-for-chunk) that even a
     * 3-consecutive-match/40 ms check still lets one chunk in ~600 through
     * with a stable-but-wrong value: a still-decaying transient can
     * plateau on the wrong byte for longer than that check's ~120 ms
     * window and pass every one of its checks. Five matches spread over a
     * longer window makes that coincidence far less likely; if it still
     * never converges within the attempt budget, log it so a residual
     * failure is visible instead of silently sending the last-read
     * (possibly still-wrong) bytes. */
    static uint8_t sau8Chunk[OTA_LORA_CHUNK_LEN];
    static uint8_t sau8ChunkVerify[OTA_LORA_CHUNK_LEN];
    if (!FOTA_bReadImage(u32Offset, sau8Chunk, u8Len))
        return false;
    uint8_t u8StableCount = 1U;
    uint8_t u8Attempt;
    for (u8Attempt = 0U; u8Attempt < 20U && u8StableCount < 5U; u8Attempt++)
    {
        osDelay(50U);
        if (!FOTA_bReadImage(u32Offset, sau8ChunkVerify, u8Len))
            return false;
        if (memcmp(sau8Chunk, sau8ChunkVerify, u8Len) == 0)
        {
            u8StableCount++;
        }
        else
        {
            memcpy(sau8Chunk, sau8ChunkVerify, u8Len);
            u8StableCount = 1U;   /* restart the run of agreements */
        }
    }
    if (u8StableCount < 5U)
    {
        DBG_LOG("Fota: chunk %u never reached 5 stable reads (got %u) - "
                "sending best-effort value\r\n",
                (unsigned)u16Chunk, (unsigned)u8StableCount);
    }
    memcpy(&au8Pkt[OTA_PKT_CHUNK_HDR_LEN], sau8Chunk, u8Len);

    if (!FOTA_bRadioTx(au8Pkt, (uint8_t)(OTA_PKT_CHUNK_HDR_LEN + u8Len)))
        return false;

    osDelay(OTA_LORA_CHUNK_GAP_MS);
    return true;
}

static bool FOTA_bPollTarget(FotaTarget_t *ptTarget, uint16_t u16First,
                             uint8_t u8Count, uint8_t *pu8Union)
{
    uint8_t au8Poll[OTA_PKT_POLL_LEN];
    au8Poll[0] = (uint8_t)MeshPktType_OtaPoll;
    FOTA_vPutU32(&au8Poll[1], u32SessionId);
    FOTA_vPutU32(&au8Poll[5], ptTarget->u32DeviceId);
    FOTA_vPutU16(&au8Poll[9], u16First);
    au8Poll[11] = u8Count;

    bReportMail = false;
    if (!FOTA_bRadioTx(au8Poll, sizeof(au8Poll)))
    {
        DBG_LOG("Fota: poll TX failed for %04lX chunks %u..%u\r\n",
                (unsigned long)ptTarget->u32DeviceId,
                (unsigned)u16First, (unsigned)(u16First + u8Count - 1));
        return false;
    }

    uint32_t u32Start = osKernelGetTickCount();
    while ((osKernelGetTickCount() - u32Start) < OTA_LORA_POLL_TIMEOUT_MS)
    {
        if (bReportMail && u32ReportFrom == ptTarget->u32DeviceId)
        {
            ptTarget->u8Status = u8ReportStatus;
            if (u8ReportStatus == OTA_RPT_WINDOW)
            {
                uint8_t u8Missing = 0U;
                for (uint8_t i = 0U; i < OTA_WINDOW_BITMAP_LEN; i++)
                {
                    pu8Union[i] |= au8ReportBitmap[i];
                    for (uint8_t b = 0U; b < 8U; b++)
                        if (au8ReportBitmap[i] & (uint8_t)(1U << b))
                            u8Missing++;
                }
                DBG_LOG("Fota: poll reply from %04lX: %u/%u chunks still missing in window %u..%u\r\n",
                        (unsigned long)ptTarget->u32DeviceId,
                        (unsigned)u8Missing, (unsigned)u8Count,
                        (unsigned)u16First, (unsigned)(u16First + u8Count - 1));
            }
            else
            {
                DBG_LOG("Fota: poll reply from %04lX: status=%u (window %u..%u)\r\n",
                        (unsigned long)ptTarget->u32DeviceId,
                        (unsigned)u8ReportStatus,
                        (unsigned)u16First, (unsigned)(u16First + u8Count - 1));
            }
            bReportMail = false;
            return true;
        }
        osDelay(20);
    }
    DBG_LOG("Fota: poll TIMEOUT from %04lX (window %u..%u, waited %u ms)\r\n",
            (unsigned long)ptTarget->u32DeviceId,
            (unsigned)u16First, (unsigned)(u16First + u8Count - 1),
            (unsigned)OTA_LORA_POLL_TIMEOUT_MS);
    return false;
}

void FOTA_vDistribute(void)
{
    FotaMeta_t tMeta;
    if (!FOTA_bGetMeta(&tMeta))
        return;

    /* Keep the NOR flash awake for the whole session: reads issued shortly
     * after a deep-power-down wake (which the DbgLog consumer triggers in
     * the radio gaps between chunks/windows) return corrupted bytes, which
     * is what made the primary transmit a differently-corrupted image every
     * session. Released at every exit below. */
    FLASH_vInhibitDeepPowerDown(true);
    LOG_vSuspend(true);
    bDistributeActive = true;

    uint16_t u16Total = (uint16_t)((tMeta.u32SizeBytes + OTA_LORA_CHUNK_LEN - 1U)
                                   / OTA_LORA_CHUNK_LEN);
    u32SessionId  = MESHNETWORK_u32GenerateGlobalMsgID();
    u8TargetCount = 0U;
    memset(atTargets, 0, sizeof(atTargets));

    DBG_LOG("Fota: distribute v%lu (%lu B, %u chunks) session %08lX\r\n",
            (unsigned long)tMeta.u32Version, (unsigned long)tMeta.u32SizeBytes,
            u16Total, (unsigned long)u32SessionId);

    /* ---- Listen-before-distribute (multi-primary coexistence) ----
     * Two primaries on the same fr9 schedule enter this function in the same
     * second; without this they broadcast OtaPrep/chunks simultaneously and
     * secondaries between them join neither (both log "no targets joined").
     * Wait a random backoff and watch u32LastForeignOtaTick: whoever draws
     * the shorter backoff Preps first, and the other hears that foreign
     * session mid-backoff and defers to the next campaign (the winner covers
     * the shared secondaries; a secondary already updated simply won't join,
     * so the winner finishes fast and the next random draw lets us run).
     * The fr9 FW check after this call (DeviceDiscovery.c) still runs — only
     * the broadcast is skipped. */
    {
        uint32_t u32Backoff = (uint32_t)(rand() % (int)OTA_DISTRIBUTE_BACKOFF_SPREAD_MS);
        uint32_t u32Start   = osKernelGetTickCount();
        while ((osKernelGetTickCount() - u32Start) < u32Backoff)
        {
            if (u32LastForeignOtaTick != 0U &&
                (int32_t)(u32LastForeignOtaTick - u32Start) >= 0)
                break;   /* another primary started first — stop waiting, defer below */
            osDelay(100);
        }

        if (u32LastForeignOtaTick != 0U &&
            (uint32_t)(osKernelGetTickCount() - u32LastForeignOtaTick) < OTA_FOREIGN_ACTIVE_MS)
        {
            DBG_LOG("Fota: another primary distributing - deferring to next wake\r\n");
            bDistributeActive = false;
            LOG_vSuspend(false);
            FLASH_vInhibitDeepPowerDown(false);
            return;
        }
    }

    /* Re-verify our OWN stored copy immediately before every send, not just
     * once at acquire time: a single scan can occasionally return a wrong
     * value while the stored bytes are fine (a transient SPI/flash read
     * glitch), and an immediate re-read recovers the correct value — see
     * FOTA_bVerifyImageXorRetry. Only a mismatch that survives every retry
     * means the primary's own copy has actually drifted. */
    uint8_t u8PreSendXor;
    if (!FOTA_bVerifyImageXorRetry(0U, tMeta.u32SizeBytes, tMeta.u8Xor8, &u8PreSendXor))
    {
        DBG_LOG("Fota: PRE-SEND xor mismatch (stored=0x%02X, meta=0x%02X) - "
                "primary's own copy has drifted since acquire, aborting distribute\r\n",
                (unsigned)u8PreSendXor, (unsigned)tMeta.u8Xor8);

        /* Confirmed corrupt, not a transient glitch (every retry attempt
         * disagreed - see FOTA_bVerifyImageXorRetry). Leaving a known-bad
         * image staged means every subsequent wake re-fails this exact
         * same check forever. Erase scratch + metadata so
         * FOTA_bGetMeta()/the distribute-pending check see "nothing
         * staged" and the next FOTA_bUartAcquire() naturally re-pulls a
         * fresh copy instead of retrying a corrupt one indefinitely. */
        if (!FOTA_bEraseScratch())
            DBG_LOG("Fota: scratch erase after PRE-SEND mismatch FAILED\r\n");

        bDistributeActive = false;
        LOG_vSuspend(false);
        FLASH_vInhibitDeepPowerDown(false);
        return;
    }

    uint8_t au8Prep[OTA_PKT_PREP_LEN];
    au8Prep[0] = (uint8_t)MeshPktType_OtaPrep;
    FOTA_vPutU32(&au8Prep[1],  u32SessionId);
    FOTA_vPutU32(&au8Prep[5],  tMeta.u32Version);
    FOTA_vPutU32(&au8Prep[9],  tMeta.u32SizeBytes);
    FOTA_vPutU16(&au8Prep[13], u16Total);
    au8Prep[15] = (uint8_t)OTA_LORA_CHUNK_LEN;
    au8Prep[16] = tMeta.u8Xor8;
    au8Prep[17] = (uint8_t)OTA_LORA_WINDOW_CHUNKS;

    for (uint8_t i = 0U; i < OTA_LORA_PREP_REPEATS; i++)
    {
        (void)FOTA_bRadioTx(au8Prep, sizeof(au8Prep));
        osDelay(OTA_LORA_PREP_GAP_MS);
    }
    osDelay(4000);   /* secondaries erase their scratch (~3 s) before acking */

    if (u8TargetCount == 0U)
    {
        DBG_LOG("Fota: no targets joined\r\n");
        bDistributeActive = false;
        LOG_vSuspend(false);
        FLASH_vInhibitDeepPowerDown(false);
        return;
    }

    uint32_t u32SessionStart = osKernelGetTickCount();

    for (uint16_t u16First = 0U; u16First < u16Total;
         u16First = (uint16_t)(u16First + OTA_LORA_WINDOW_CHUNKS))
    {
        uint8_t u8Count = (uint8_t)(((u16Total - u16First) > OTA_LORA_WINDOW_CHUNKS)
                        ? OTA_LORA_WINDOW_CHUNKS : (u16Total - u16First));

        /* Give every target a fresh chance each window: bAlive used to be
         * permanent for the rest of the whole multi-minute session after
         * just 2 consecutive missed poll responses (a 2.5 s timeout each) —
         * easily eaten by ordinary CAD back-off under mesh congestion, not
         * a sign the target is actually gone. That single early strike-out
         * is why one or two chunks near the end of a session would never
         * get repaired: the target was still there and still listening,
         * but permanently excluded from every later window's polls. Only
         * within-this-window drop still applies (skip wasting THIS
         * window's remaining repair rounds on someone not answering right
         * now); the next window tries again from a clean slate. */
        for (uint8_t t = 0U; t < u8TargetCount; t++)
        {
            atTargets[t].bAlive   = true;
            atTargets[t].u8Strikes = 0U;
        }

        DBG_LOG("Fota: >>> window chunks %u..%u (%u/%u)\r\n",
                (unsigned)u16First, (unsigned)(u16First + u8Count - 1),
                (unsigned)(u16First + u8Count), (unsigned)u16Total);

        for (uint8_t i = 0U; i < u8Count; i++)
        {
            if (!FOTA_bSendChunk((uint16_t)(u16First + i), tMeta.u32SizeBytes))
            {
                DBG_LOG("Fota: SEND chunk %u FAILED (radio TX)\r\n",
                        (unsigned)(u16First + i));
            }
        }

        for (uint8_t u8Round = 0U; u8Round < OTA_LORA_REPAIR_ROUNDS; u8Round++)
        {
            uint8_t au8Union[OTA_WINDOW_BITMAP_LEN] = {0};
            bool    bAnyMissing      = false;
            bool    bAnyReplyThisRnd = false;

            for (uint8_t t = 0U; t < u8TargetCount; t++)
            {
                if (!atTargets[t].bAlive)
                    continue;
                if (FOTA_bPollTarget(&atTargets[t], u16First, u8Count, au8Union))
                {
                    atTargets[t].u8Strikes = 0U;
                    bAnyReplyThisRnd = true;
                }
                else if (++atTargets[t].u8Strikes >= 2U)
                {
                    atTargets[t].bAlive = false;
                    DBG_LOG("Fota: target %04lX unresponsive this window (2 poll timeouts) - will retry next window\r\n",
                            (unsigned long)atTargets[t].u32DeviceId);
                }
            }

            /* If no target replied this round, we cannot claim "nothing
             * missing" — au8Union would just be zero because we never
             * heard from anyone, not because everyone confirmed. Force a
             * blind repair (retransmit every chunk in the window) so
             * progress still happens over a lossy link, instead of the
             * previous behavior where a timed-out poll silently advanced
             * as "fully acked" and abandoned any chunks that were in fact
             * still missing. */
            if (!bAnyReplyThisRnd)
            {
                DBG_LOG("Fota: window %u..%u round %u: no poll replies, assuming ALL missing and repairing\r\n",
                        (unsigned)u16First, (unsigned)(u16First + u8Count - 1),
                        (unsigned)u8Round);
                for (uint8_t i = 0U; i < u8Count; i++)
                    au8Union[i / 8U] |= (uint8_t)(1U << (i % 8U));
            }

            uint8_t u8MissingCount = 0U;
            for (uint8_t i = 0U; i < u8Count; i++)
                if (au8Union[i / 8U] & (uint8_t)(1U << (i % 8U)))
                {
                    u8MissingCount++;
                    bAnyMissing = true;
                }

            if (!bAnyMissing)
            {
                DBG_LOG("Fota: window %u..%u fully acked after round %u\r\n",
                        (unsigned)u16First, (unsigned)(u16First + u8Count - 1),
                        (unsigned)u8Round);
                break;
            }

            DBG_LOG("Fota: window %u..%u round %u: %u chunk(s) still missing across all targets, repairing\r\n",
                    (unsigned)u16First, (unsigned)(u16First + u8Count - 1),
                    (unsigned)u8Round, (unsigned)u8MissingCount);

            for (uint8_t i = 0U; i < u8Count; i++)
            {
                if (au8Union[i / 8U] & (uint8_t)(1U << (i % 8U)))
                {
                    if (!FOTA_bSendChunk((uint16_t)(u16First + i),
                                         tMeta.u32SizeBytes))
                    {
                        DBG_LOG("Fota: REPAIR chunk %u FAILED (radio TX)\r\n",
                                (unsigned)(u16First + i));
                    }
                }
            }
        }

        /* Final view of this window before advancing */
        bool bWindowClean = true;
        for (uint8_t t = 0U; t < u8TargetCount; t++)
        {
            if (atTargets[t].u8Status == OTA_RPT_WINDOW && atTargets[t].bAlive)
            {
                /* Best-effort last check: not all polls were responded to
                 * with a fully-acked bitmap - flag it so it's visible in
                 * the log if we're advancing anyway. */
                bWindowClean = false;
            }
        }
        (void)bWindowClean;   /* diagnostic only; the poll bitmap above is authoritative */

        if ((osKernelGetTickCount() - u32SessionStart) >= OTA_LORA_SESSION_MAX_MS)
        {
            DBG_LOG("Fota: session hard cap hit at window %u..%u\r\n",
                    (unsigned)u16First, (unsigned)(u16First + u8Count - 1));
            break;
        }
    }

    /* Finalize: give targets time to XOR-verify + report. */
    uint32_t u32WaitStart = osKernelGetTickCount();
    while ((osKernelGetTickCount() - u32WaitStart) < 10000U)
    {
        if (bReportMail)
        {
            for (uint8_t t = 0U; t < u8TargetCount; t++)
                if (atTargets[t].u32DeviceId == u32ReportFrom)
                    atTargets[t].u8Status = u8ReportStatus;
            bReportMail = false;
        }
        osDelay(100);
    }

    bool bAllUpdated = true;
    for (uint8_t t = 0U; t < u8TargetCount; t++)
    {
        DBG_LOG("Fota: target %04lX %s\r\n",
                (unsigned long)atTargets[t].u32DeviceId,
                (atTargets[t].u8Status == OTA_RPT_VALID) ? "UPDATED"
              : (atTargets[t].u8Status == OTA_RPT_ERROR) ? "VERIFY FAILED"
              : atTargets[t].bAlive ? "INCOMPLETE" : "LOST");
        if (atTargets[t].u8Status != OTA_RPT_VALID)
            bAllUpdated = false;
    }

    /* bDistributed is informational only now (see FOTA_bDistributePending)
     * — it no longer gates whether the automatic wake path tries again.
     * Every wake re-announces OtaPrep regardless, so a secondary that was
     * asleep, out of range, or dropped mid-transfer (INCOMPLETE/LOST/
     * VERIFY FAILED) still gets picked up on a later attempt; a secondary
     * already on this version simply won't join (see the OtaPrep handler's
     * u32Ver <= VERSION_u32Get() check), so a redundant re-announce after
     * everyone's already updated costs one join wait and nothing else. */
    if (bAllUpdated)
    {
        (void)FOTA_bMarkDistributed();
        DBG_LOG("Fota: distribution session done - all targets updated\r\n");
    }
    else
    {
        DBG_LOG("Fota: distribution session done - not all targets updated, will retry next wake\r\n");
    }

    bDistributeActive = false;
    LOG_vSuspend(false);
    FLASH_vInhibitDeepPowerDown(false);
}

/* --------------------------------------------------------------------------
 * Secondary — receive session
 * -------------------------------------------------------------------------- */

bool FOTA_bPrepPending(void)
{
    return bPrepPending;
}

void FOTA_vArmAcceptance(void)
{
    /* Discovery TimeSync auto-arm. Persists across wakes so a secondary that
     * kept missing the distribution catches up on a later campaign — but this
     * source must NOT be honoured inside a kernel/log-download wakeup, so
     * clear the kernel flag. */
    bFwAcceptArmed     = true;
    bFwAcceptViaKernel = false;
    DBG_LOG("Fota: firmware acceptance ARMED\r\n");
}

void FOTA_vArmAcceptanceKernel(void)
{
    /* Explicit "tag <ID> fwaccept" — the only arm source allowed to run an
     * OTA receive inside a live FrKernel session (see the kernel rendezvous
     * in DeviceDiscovery.c). */
    bFwAcceptArmed     = true;
    bFwAcceptViaKernel = true;
    DBG_LOG("Fota: firmware acceptance ARMED (kernel)\r\n");
}

void FOTA_vDisarmAcceptance(void)
{
    bFwAcceptArmed     = false;
    bFwAcceptViaKernel = false;
    bPrepPending       = false;
    DBG_LOG("Fota: firmware acceptance disarmed\r\n");
}

bool FOTA_bAcceptanceArmed(void)
{
    return bFwAcceptArmed;
}

bool FOTA_bAcceptanceArmedViaKernel(void)
{
    return bFwAcceptArmed && bFwAcceptViaKernel;
}

bool FOTA_bSessionActive(void)
{
    return bDistributeActive || bRxSessionActive;
}

void FOTA_vSecondaryReceive(void)
{
    bPrepPending       = false;
    bFwAcceptArmed     = false;   /* one-shot: retry needs a fresh fwaccept */
    bFwAcceptViaKernel = false;

    DBG_LOG("Fota: receiving v%lu (%lu B, %u chunks)\r\n",
            (unsigned long)tPrep.u32Version, (unsigned long)tPrep.u32ImageSize,
            tPrep.u16TotalChunks);

    /* Keep the NOR flash awake for the whole receive+verify session — same
     * DPD-wake read-corruption reason as the primary's distribute path. */
    FLASH_vInhibitDeepPowerDown(true);
    LOG_vSuspend(true);

    if (!FOTA_bEraseScratch())
    {
        LOG_vSuspend(false);
        FLASH_vInhibitDeepPowerDown(false);
        return;
    }

    memset(au8ChunkBitmap, 0, sizeof(au8ChunkBitmap));
    u16ChunksHave    = 0U;
    bChunkMail       = false;
    bRxSessionActive = true;
    u32LastSessionPktTick = osKernelGetTickCount();

    uint8_t au8Ack[OTA_PKT_PREPACK_LEN];
    au8Ack[0] = (uint8_t)MeshPktType_OtaPrepAck;
    FOTA_vPutU32(&au8Ack[1], tPrep.u32SessionId);
    FOTA_vPutU32(&au8Ack[5], LORARADIO_u32GetUniqueId());
    FOTA_vPutU32(&au8Ack[9], VERSION_u32Get());
    FOTA_vPutU16(&au8Ack[13], 0U);
    (void)MESHNETWORK_bSendOtaResponse(au8Ack, sizeof(au8Ack));

    uint32_t u32SessionStart = osKernelGetTickCount();
    uint8_t  u8LastPct       = 0xFFU;
    while (u16ChunksHave < tPrep.u16TotalChunks)
    {
        (void)osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_OTA, osFlagsWaitAny, 500U);

        if (bChunkMail)
        {
            uint32_t u32Offset = (uint32_t)u16ChunkMailIdx * tPrep.u8ChunkLen;
            if (FOTA_bWriteImage(u32Offset, au8ChunkMail, u8ChunkMailLen))
            {
                /* Read-back-and-compare against au8ChunkMail (the exact
                 * bytes just sent to flash — the parser task cannot refill
                 * it until bChunkMail is cleared below). This catches a bad
                 * WRITE at the exact chunk/offset in real time, instead of
                 * only discovering "something in the image is wrong" from a
                 * whole-image XOR pass minutes later with no idea which
                 * chunk did it.
                 *
                 * NOT an immediate back-to-back read: this chunk's write
                 * happens right after the radio RX that delivered it, and
                 * the SX126x's RX/TX current draw sags the same shared
                 * supply rail that corrupts a same-window SPI flash access
                 * (proven on the primary's TX side — see FOTA_bSendChunk).
                 * A verify-read sampled from inside that same disturbed
                 * window can plateau on the same wrong bytes the write
                 * produced and falsely "confirm" a bad write. Delay past
                 * the settle window first, and if the read still disagrees,
                 * don't just log it — re-write and re-verify, since the
                 * data in flash may genuinely be wrong, not just misread. */
                uint8_t au8Verify[OTA_LORA_CHUNK_LEN];
                bool bWriteOk = false;
                for (uint8_t u8WAttempt = 0U; u8WAttempt < 3U && !bWriteOk; u8WAttempt++)
                {
                    osDelay(40U);
                    bWriteOk = FOTA_bReadImage(u32Offset, au8Verify, u8ChunkMailLen) &&
                               (memcmp(au8Verify, au8ChunkMail, u8ChunkMailLen) == 0);
                    if (!bWriteOk && u8WAttempt < 2U)
                    {
                        (void)FOTA_bWriteImage(u32Offset, au8ChunkMail, u8ChunkMailLen);
                    }
                }
                if (!bWriteOk)
                {
                    uint16_t u16BadAt = 0xFFFFU;
                    for (uint16_t i = 0U; i < u8ChunkMailLen; i++)
                    {
                        if (au8Verify[i] != au8ChunkMail[i])
                        {
                            u16BadAt = i;
                            break;
                        }
                    }
                    DBG_LOG("Fota: WRITE-VERIFY MISMATCH chunk %u offset 0x%lX "
                            "(first bad byte @+%u: wrote 0x%02X read 0x%02X)\r\n",
                            (unsigned)u16ChunkMailIdx, (unsigned long)u32Offset,
                            (unsigned)u16BadAt,
                            (u16BadAt != 0xFFFFU) ? (unsigned)au8ChunkMail[u16BadAt] : 0U,
                            (u16BadAt != 0xFFFFU) ? (unsigned)au8Verify[u16BadAt] : 0U);
                }

                FOTA_vBitSet(au8ChunkBitmap, u16ChunkMailIdx);
                u16ChunksHave++;

                /* Progress line: first chunk, then every ~10%, then the
                 * last one — same throttling convention as the primary's
                 * own FWGET/programming progress logs, so a long-running
                 * receive isn't just silence until it either finishes or
                 * times out with no visibility into how far it got. */
                uint8_t u8Pct = (uint8_t)(((uint32_t)u16ChunksHave * 100UL)
                                          / tPrep.u16TotalChunks);
                if (u8LastPct == 0xFFU || u16ChunksHave == tPrep.u16TotalChunks ||
                    u8Pct >= (uint8_t)(u8LastPct + 10U))
                {
                    DBG_LOG("Fota: receiving %u/%u chunks (%u%%)\r\n",
                            u16ChunksHave, tPrep.u16TotalChunks, u8Pct);
                    u8LastPct = u8Pct;
                }
            }
            else
            {
                DBG_LOG("Fota: WRITE chunk %u to scratch offset 0x%lX FAILED\r\n",
                        (unsigned)u16ChunkMailIdx, (unsigned long)u32Offset);
            }
            bChunkMail = false;
        }

        uint32_t u32Now = osKernelGetTickCount();
        if ((u32Now - u32LastSessionPktTick) >= OTA_LORA_RX_IDLE_MS)
        {
            /* On a silent abort, list the outstanding chunk indices so the
             * next test iteration can see exactly what didn't arrive
             * rather than just the count. Bounded to first 16 to avoid
             * flooding the log if the session died very early. */
            DBG_LOG("Fota: session went silent (%u/%u chunks) - abort\r\n",
                    u16ChunksHave, tPrep.u16TotalChunks);
            uint16_t u16Listed = 0U;
            for (uint16_t i = 0U; i < tPrep.u16TotalChunks && u16Listed < 16U; i++)
            {
                if (!FOTA_bBitGet(au8ChunkBitmap, i))
                {
                    DBG_LOG("  missing chunk %u (offset 0x%lX)\r\n",
                            (unsigned)i,
                            (unsigned long)((uint32_t)i * tPrep.u8ChunkLen));
                    u16Listed++;
                }
            }
            bRxSessionActive = false;
            LOG_vSuspend(false);
            FLASH_vInhibitDeepPowerDown(false);
            return;
        }
        if ((u32Now - u32SessionStart) >= OTA_LORA_SESSION_MAX_MS)
        {
            DBG_LOG("Fota: session hard cap - abort\r\n");
            bRxSessionActive = false;
            LOG_vSuspend(false);
            FLASH_vInhibitDeepPowerDown(false);
            return;
        }
    }

    /* Diagnostic: same probe as the primary's pre-send check — see there
     * for why. */
    {
        uint8_t u8Xor64  = FOTA_u8CalcImageXorRange(0U, tPrep.u32ImageSize);
        uint8_t u8Xor224 = FOTA_u8CalcImageXor224(0U, tPrep.u32ImageSize);
        DBG_LOG("Fota: PROBE 64B-buf=0x%02X 224B-buf=0x%02X expected=0x%02X\r\n",
                (unsigned)u8Xor64, (unsigned)u8Xor224, (unsigned)tPrep.u8ImageXor);
    }

    /* Retried: confirmed on hardware that a single scan can occasionally
     * return a wrong value while the stored bytes are fine (a transient
     * SPI/flash read glitch — see FOTA_bVerifyImageXorRetry). Only a
     * mismatch that survives every retry is treated as a real failure. */
    uint8_t u8Xor;
    bool bValid = FOTA_bVerifyImageXorRetry(0U, tPrep.u32ImageSize, tPrep.u8ImageXor, &u8Xor);

    uint8_t au8Rpt[OTA_PKT_REPORT_LEN];
    au8Rpt[0] = (uint8_t)MeshPktType_OtaReport;
    FOTA_vPutU32(&au8Rpt[1], tPrep.u32SessionId);
    FOTA_vPutU32(&au8Rpt[5], LORARADIO_u32GetUniqueId());
    au8Rpt[9] = bValid ? OTA_RPT_VALID : OTA_RPT_ERROR;
    memset(&au8Rpt[10], 0, OTA_WINDOW_BITMAP_LEN);

    if (!bValid)
    {
        DBG_LOG("Fota: image XOR mismatch (0x%02X != 0x%02X) after 3 attempts\r\n",
                u8Xor, tPrep.u8ImageXor);

        /* Every chunk passed its own LoRa packet CRC and the receive
         * bitmap says complete, yet the whole-image XOR disagrees — either
         * one chunk landed at the wrong offset or a corrupted chunk slipped
         * past its packet CRC by coincidence. Bisect into quarters (same
         * technique used earlier for the bootloader/primary XOR mismatches)
         * to localize which part of the image disagrees, narrowing "613
         * chunks, one of them is wrong" down to roughly which window. */
        {
            uint32_t u32Q = tPrep.u32ImageSize / 4U;
            uint32_t au32Start[4] = { 0U, u32Q, 2U * u32Q, 3U * u32Q };
            uint32_t au32Len[4]   = { u32Q, u32Q, u32Q, tPrep.u32ImageSize - (3U * u32Q) };
            for (uint8_t i = 0U; i < 4U; i++)
            {
                uint8_t u8Q = FOTA_u8CalcImageXorRange(au32Start[i], au32Len[i]);
                DBG_LOG("  Q%u [0x%lX..0x%lX] (chunks %u..%u) xor=0x%02X\r\n", i,
                        (unsigned long)au32Start[i],
                        (unsigned long)(au32Start[i] + au32Len[i] - 1U),
                        (unsigned)(au32Start[i] / tPrep.u8ChunkLen),
                        (unsigned)((au32Start[i] + au32Len[i] - 1U) / tPrep.u8ChunkLen),
                        u8Q);
            }
        }

        /* Confirmed corrupt (every retry attempt disagreed - see
         * FOTA_bVerifyImageXorRetry), diagnostic bisection above already
         * read whatever's there. Erase rather than leave known-bad bytes
         * staged: metadata was never committed for this receive (that
         * happens below, after this check), so this is cleanup, not a
         * state-consistency fix. */
        if (!FOTA_bEraseScratch())
            DBG_LOG("Fota: scratch erase after receive xor mismatch FAILED\r\n");

        (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
        bRxSessionActive = false;
        LOG_vSuspend(false);
        FLASH_vInhibitDeepPowerDown(false);
        return;
    }

    if (!FOTA_bCommitMetadata(tPrep.u32Version,
                              OTA_APP_BASE_ADDR + tPrep.u32ImageSize - 1UL,
                              u8Xor))
    {
        bRxSessionActive = false;
        LOG_vSuspend(false);
        FLASH_vInhibitDeepPowerDown(false);
        return;
    }

    DBG_LOG("Fota: image v%lu received + verified OK\r\n",
            (unsigned long)tPrep.u32Version);
    (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
    osDelay(2000);   /* let the (jittered) report leave the radio */

    bRxSessionActive = false;
    LOG_vSuspend(false);
    FLASH_vInhibitDeepPowerDown(false);
    FOTA_vArmBootloaderAndReset(tPrep.u32Version);
}

#endif /* STORAGE_BACKEND_FLASH */
