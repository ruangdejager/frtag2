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

#include <string.h>

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

uint8_t FOTA_u8CalcImageXorRange(uint32_t u32Start, uint32_t u32Len)
{
    uint8_t  au8Buf[OTA_XOR_BUF_LEN];
    uint8_t  u8Xor     = 0U;
    uint32_t u32Addr   = OTA_SCRATCH_START_ADDR + u32Start;
    uint32_t u32Remain = u32Len;

    while (u32Remain > 0U)
    {
        uint16_t u16Chunk = (u32Remain > sizeof(au8Buf))
                          ? (uint16_t)sizeof(au8Buf)
                          : (uint16_t)u32Remain;

        if (!FOTA_DRIVER_bRead(u32Addr, au8Buf, u16Chunk))
            return (uint8_t)~u8Xor;   /* read failure: guarantee a mismatch */

        for (uint16_t i = 0U; i < u16Chunk; i++)
            u8Xor ^= au8Buf[i];

        u32Addr   += u16Chunk;
        u32Remain -= u16Chunk;

        if ((u32Addr % 0x8000UL) == 0U)
            osDelay(1);   /* yield every 32 KB of the ~236 KB pass */
    }
    return u8Xor;
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

    DBG_LOG("Fota: requesting fw check (running v%lu)\r\n",
            (unsigned long)VERSION_u32Get());
    if (!FARMRANGER_bFwCheckRequest(VERSION_u32Get()))
    {
        DBG_LOG("Fota: fw check request not acked - polling FWREQ anyway\r\n");
    }

    if (FOTA_eQueryLogger(&u32Version, &u32FileBytes, &u8ExpectedXor) != FARMRANGER_FW_AVAILABLE)
    {
        DBG_LOG("Fota: no newer firmware offered\r\n");
        return false;
    }

    if (u32Version <= VERSION_u32Get())
    {
        DBG_LOG("Fota: logger offers v%lu, running v%lu - skip\r\n",
                (unsigned long)u32Version, (unsigned long)VERSION_u32Get());
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
    uint8_t  u8Xor       = FOTA_u8CalcImageXor(u32Size);

    /* Cross-check against the logger's manifest-verified whole-image XOR-8
     * — catches corruption between the last per-block trailer check and
     * the final stored bytes (e.g. on the flash write/read-back path). */
    if (u8Xor != u8ExpectedXor)
    {
        DBG_LOG("Fota: image XOR mismatch (stored=0x%02X, logger=0x%02X) - discarding\r\n",
                u8Xor, u8ExpectedXor);
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
    FOTA_vArmBootloaderAndReset(u32Version);
    return true;   /* not reached */
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

/* Secondary firmware-acceptance gate. */
static volatile bool bFwAcceptArmed;

/* Primary on-demand distribution request. */
static volatile bool bDistributeReq;

/* ---- Primary: distribution target table (filled by the parser) ---- */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8Strikes;     /* consecutive unanswered polls                  */
    uint8_t  u8Status;      /* last OTA_RPT_* heard from this target         */
    bool     bAlive;
} FotaTarget_t;
static FotaTarget_t     atTargets[OTA_LORA_MAX_TARGETS];
static volatile uint8_t u8TargetCount;

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

/* Direct radio TX with queue backpressure (AppTask only). */
static bool FOTA_bRadioTx(const uint8_t *pu8Buf, uint8_t u8Len)
{
    LoraRadio_Packet_t tPkt;
    memset(&tPkt, 0, sizeof(tPkt));
    memcpy(tPkt.buffer, pu8Buf, u8Len);
    tPkt.length = u8Len;

    for (uint8_t u8Try = 0U; u8Try < 40U; u8Try++)
    {
        if (LORARADIO_bTxPacket(&tPkt))
            return true;
        osDelay(25);
    }
    return false;
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
    switch (pu8Buf[0])
    {
        case MeshPktType_OtaPrep:
        {
            if (u16Len < OTA_PKT_PREP_LEN) return;
            if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_SECONDARY) return;

            if (!bFwAcceptArmed) return;

            uint32_t u32Ver = FOTA_u32GetU32(&pu8Buf[5]);
            if (u32Ver <= VERSION_u32Get())
                return;

            tPrep.u32SessionId   = FOTA_u32GetU32(&pu8Buf[1]);
            tPrep.u32Version     = u32Ver;
            tPrep.u32ImageSize   = FOTA_u32GetU32(&pu8Buf[9]);
            tPrep.u16TotalChunks = FOTA_u16GetU16(&pu8Buf[13]);
            tPrep.u8ChunkLen     = pu8Buf[15];
            tPrep.u8ImageXor     = pu8Buf[16];

            if (tPrep.u8ChunkLen == 0U || tPrep.u16TotalChunks == 0U ||
                tPrep.u32ImageSize == 0U || tPrep.u32ImageSize > OTA_APP_MAX_SIZE)
                return;

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

            if (!bChunkMail && !FOTA_bBitGet(au8ChunkBitmap, u16Idx))
            {
                memcpy(au8ChunkMail, &pu8Buf[OTA_PKT_CHUNK_HDR_LEN], u8Len);
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
            for (uint8_t i = 0U; i < u8Count; i++)
            {
                uint16_t u16Chunk = (uint16_t)(u16First + i);
                if (u16Chunk < tPrep.u16TotalChunks &&
                    !FOTA_bBitGet(au8ChunkBitmap, u16Chunk))
                {
                    au8Rpt[10U + i / 8U] |= (uint8_t)(1U << (i % 8U));
                }
            }
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
    FotaMeta_t tMeta;
    if (!FOTA_bGetMeta(&tMeta))
        return false;
    return (tMeta.u32Version == VERSION_u32Get()) && !tMeta.bDistributed;
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

    if (!FOTA_bReadImage(u32Offset, &au8Pkt[OTA_PKT_CHUNK_HDR_LEN], u8Len))
        return false;

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
        return false;

    uint32_t u32Start = osKernelGetTickCount();
    while ((osKernelGetTickCount() - u32Start) < OTA_LORA_POLL_TIMEOUT_MS)
    {
        if (bReportMail && u32ReportFrom == ptTarget->u32DeviceId)
        {
            ptTarget->u8Status = u8ReportStatus;
            if (u8ReportStatus == OTA_RPT_WINDOW)
            {
                for (uint8_t i = 0U; i < OTA_WINDOW_BITMAP_LEN; i++)
                    pu8Union[i] |= au8ReportBitmap[i];
            }
            bReportMail = false;
            return true;
        }
        osDelay(20);
    }
    return false;
}

void FOTA_vDistribute(void)
{
    FotaMeta_t tMeta;
    if (!FOTA_bGetMeta(&tMeta))
        return;

    uint16_t u16Total = (uint16_t)((tMeta.u32SizeBytes + OTA_LORA_CHUNK_LEN - 1U)
                                   / OTA_LORA_CHUNK_LEN);
    u32SessionId  = MESHNETWORK_u32GenerateGlobalMsgID();
    u8TargetCount = 0U;
    memset(atTargets, 0, sizeof(atTargets));

    DBG_LOG("Fota: distribute v%lu (%lu B, %u chunks) session %08lX\r\n",
            (unsigned long)tMeta.u32Version, (unsigned long)tMeta.u32SizeBytes,
            u16Total, (unsigned long)u32SessionId);

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
        return;
    }

    uint32_t u32SessionStart = osKernelGetTickCount();

    for (uint16_t u16First = 0U; u16First < u16Total;
         u16First = (uint16_t)(u16First + OTA_LORA_WINDOW_CHUNKS))
    {
        uint8_t u8Count = (uint8_t)(((u16Total - u16First) > OTA_LORA_WINDOW_CHUNKS)
                        ? OTA_LORA_WINDOW_CHUNKS : (u16Total - u16First));

        for (uint8_t i = 0U; i < u8Count; i++)
            (void)FOTA_bSendChunk((uint16_t)(u16First + i), tMeta.u32SizeBytes);

        for (uint8_t u8Round = 0U; u8Round < OTA_LORA_REPAIR_ROUNDS; u8Round++)
        {
            uint8_t au8Union[OTA_WINDOW_BITMAP_LEN] = {0};
            bool    bAnyMissing = false;

            for (uint8_t t = 0U; t < u8TargetCount; t++)
            {
                if (!atTargets[t].bAlive)
                    continue;
                if (FOTA_bPollTarget(&atTargets[t], u16First, u8Count, au8Union))
                {
                    atTargets[t].u8Strikes = 0U;
                }
                else if (++atTargets[t].u8Strikes >= 2U)
                {
                    atTargets[t].bAlive = false;
                    DBG_LOG("Fota: target %04lX dropped (silent)\r\n",
                            (unsigned long)atTargets[t].u32DeviceId);
                }
            }

            for (uint8_t i = 0U; i < u8Count && !bAnyMissing; i++)
                if (au8Union[i / 8U] & (uint8_t)(1U << (i % 8U)))
                    bAnyMissing = true;

            if (!bAnyMissing)
                break;

            for (uint8_t i = 0U; i < u8Count; i++)
            {
                if (au8Union[i / 8U] & (uint8_t)(1U << (i % 8U)))
                    (void)FOTA_bSendChunk((uint16_t)(u16First + i),
                                          tMeta.u32SizeBytes);
            }
        }

        if ((osKernelGetTickCount() - u32SessionStart) >= OTA_LORA_SESSION_MAX_MS)
        {
            DBG_LOG("Fota: session hard cap hit\r\n");
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

    for (uint8_t t = 0U; t < u8TargetCount; t++)
    {
        DBG_LOG("Fota: target %04lX %s\r\n",
                (unsigned long)atTargets[t].u32DeviceId,
                (atTargets[t].u8Status == OTA_RPT_VALID) ? "UPDATED"
              : (atTargets[t].u8Status == OTA_RPT_ERROR) ? "VERIFY FAILED"
              : atTargets[t].bAlive ? "INCOMPLETE" : "LOST");
    }

    (void)FOTA_bMarkDistributed();
    DBG_LOG("Fota: distribution session done\r\n");
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
    bFwAcceptArmed = true;
    DBG_LOG("Fota: firmware acceptance ARMED\r\n");
}

void FOTA_vDisarmAcceptance(void)
{
    bFwAcceptArmed = false;
    bPrepPending   = false;
    DBG_LOG("Fota: firmware acceptance disarmed\r\n");
}

bool FOTA_bAcceptanceArmed(void)
{
    return bFwAcceptArmed;
}

void FOTA_vSecondaryReceive(void)
{
    bPrepPending = false;
    bFwAcceptArmed = false;   /* one-shot: retry needs a fresh fwaccept */

    DBG_LOG("Fota: receiving v%lu (%lu B, %u chunks)\r\n",
            (unsigned long)tPrep.u32Version, (unsigned long)tPrep.u32ImageSize,
            tPrep.u16TotalChunks);

    if (!FOTA_bEraseScratch())
        return;

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
    while (u16ChunksHave < tPrep.u16TotalChunks)
    {
        (void)osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_OTA, osFlagsWaitAny, 500U);

        if (bChunkMail)
        {
            uint32_t u32Offset = (uint32_t)u16ChunkMailIdx * tPrep.u8ChunkLen;
            if (FOTA_bWriteImage(u32Offset, au8ChunkMail, u8ChunkMailLen))
            {
                FOTA_vBitSet(au8ChunkBitmap, u16ChunkMailIdx);
                u16ChunksHave++;
            }
            bChunkMail = false;
        }

        uint32_t u32Now = osKernelGetTickCount();
        if ((u32Now - u32LastSessionPktTick) >= OTA_LORA_RX_IDLE_MS)
        {
            DBG_LOG("Fota: session went silent (%u/%u chunks) - abort\r\n",
                    u16ChunksHave, tPrep.u16TotalChunks);
            bRxSessionActive = false;
            return;
        }
        if ((u32Now - u32SessionStart) >= OTA_LORA_SESSION_MAX_MS)
        {
            DBG_LOG("Fota: session hard cap - abort\r\n");
            bRxSessionActive = false;
            return;
        }
    }

    uint8_t u8Xor = FOTA_u8CalcImageXor(tPrep.u32ImageSize);
    bool bValid = (u8Xor == tPrep.u8ImageXor);

    uint8_t au8Rpt[OTA_PKT_REPORT_LEN];
    au8Rpt[0] = (uint8_t)MeshPktType_OtaReport;
    FOTA_vPutU32(&au8Rpt[1], tPrep.u32SessionId);
    FOTA_vPutU32(&au8Rpt[5], LORARADIO_u32GetUniqueId());
    au8Rpt[9] = bValid ? OTA_RPT_VALID : OTA_RPT_ERROR;
    memset(&au8Rpt[10], 0, OTA_WINDOW_BITMAP_LEN);

    if (!bValid)
    {
        DBG_LOG("Fota: image XOR mismatch (0x%02X != 0x%02X)\r\n",
                u8Xor, tPrep.u8ImageXor);
        (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
        bRxSessionActive = false;
        return;
    }

    if (!FOTA_bCommitMetadata(tPrep.u32Version,
                              OTA_APP_BASE_ADDR + tPrep.u32ImageSize - 1UL,
                              u8Xor))
    {
        bRxSessionActive = false;
        return;
    }

    DBG_LOG("Fota: image v%lu received + verified OK\r\n",
            (unsigned long)tPrep.u32Version);
    (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
    osDelay(2000);   /* let the (jittered) report leave the radio */

    bRxSessionActive = false;
    FOTA_vArmBootloaderAndReset(tPrep.u32Version);
}

#endif /* STORAGE_BACKEND_FLASH */
