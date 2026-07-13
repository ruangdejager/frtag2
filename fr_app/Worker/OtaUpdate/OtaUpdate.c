/*
 * OtaUpdate.c
 *
 * OTA update orchestration — UART acquire path. See OtaUpdate.h.
 *
 * Acquire sequence (all on the AppTask, logger session already up):
 *
 *   AT+FWREQ  -> FW,<ver>,<bytes> (skip unless ver > running version)
 *   erase scratchpad (before any data, so no mid-stream erase stalls RX)
 *   for each 1 KB block: AT+FWGET=<off>,<len> -> raw bytes + FB trailer,
 *     retried on any mismatch; verified bytes are fed through HexDecode
 *     into the OtaStore scratchpad
 *   HexDecode EOF/line-count check, streamed XOR-8 over the image
 *   commit metadata, AT+FWDONE=OK, arm bootloader, reset
 *
 * The per-block trailer (offset + XOR-8) plus bounded in-flight data (one
 * block per request) is the transport-robustness lesson from the discovery
 * upload's "LOG TIMEOUT" history applied in the opposite direction.
 */

#include "build_config.h"   /* STORAGE_BACKEND_FLASH (was storage_config.h) */

#ifdef STORAGE_BACKEND_FLASH

#include "OtaUpdate.h"
#include "OtaUpdate_Config.h"

#include "Farmranger.h"
#include "HexDecode.h"
#include "OtaStore.h"
#include "OtaStore_Config.h"
#include "Version.h"
#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "DeviceDiscovery.h"

#include "cmsis_os2.h"
#include "dbg_log.h"

#include <string.h>

/* One raw file block, pulled per AT+FWGET. Static — the AppTask stack is not
 * sized for a 1 KB local, and only one transfer ever runs at a time. */
static uint8_t au8BlockBuf[OTA_UART_BLOCK_LEN];

/* -------------------------------------------------------------------------- */

/* HexDecode sink: decoded image bytes go straight to the scratchpad. */
static bool OTAUPDATE_bStoreDecoded(uint32_t u32Offset, const uint8_t *pu8Data, uint8_t u8Len)
{
    return OTASTORE_bWriteImage(u32Offset, pu8Data, u8Len);
}

/* Query the logger, tolerating FW,WAIT while it warms its modem. */
static FarmrangerFw_e OTAUPDATE_eQueryLogger(uint32_t *pu32Version, uint32_t *pu32FileBytes)
{
    uint32_t u32Waited = 0U;

    for (;;)
    {
        FarmrangerFw_e eRes = FARMRANGER_eFwQuery(pu32Version, pu32FileBytes);
        if (eRes != FARMRANGER_FW_WAIT)
            return eRes;

        if (u32Waited >= OTA_FWREQ_WAIT_MAX_MS)
            return FARMRANGER_FW_NONE;

        osDelay(OTA_FWREQ_WAIT_POLL_MS);
        u32Waited += OTA_FWREQ_WAIT_POLL_MS;
    }
}

/* Pull one block (with retries) and feed it through the HEX decoder.
 * Returns false when the block could not be fetched or failed to decode. */
static bool OTAUPDATE_bFetchAndDecodeBlock(uint32_t u32Offset, uint16_t u16Len)
{
    for (uint8_t u8Attempt = 1U; u8Attempt <= OTA_UART_BLOCK_RETRIES; u8Attempt++)
    {
        if (FARMRANGER_bFwGetBlock(u32Offset, u16Len, au8BlockBuf))
        {
            /* Block arrived intact (offset + XOR-8 verified). Decode it; a
             * decode failure here means the FILE is bad (the transport was
             * just verified) — checkpoint/restore can't fix that, so abort. */
            for (uint16_t i = 0U; i < u16Len; i++)
            {
                if (!HEXDECODE_bOnByte(au8BlockBuf[i]))
                {
                    DBG_LOG("OtaUpdate: decode error at file offset %lu\r\n",
                            (unsigned long)(u32Offset + i));
                    return false;
                }
            }
            return true;
        }

        DBG_LOG("OtaUpdate: block @%lu attempt %u failed\r\n",
                (unsigned long)u32Offset, u8Attempt);
        osDelay(OTA_UART_RETRY_DELAY_MS);
    }
    return false;
}

/* --------------------------------------------------------------------------
 * OTAUPDATE_bUartAcquire
 * -------------------------------------------------------------------------- */
bool OTAUPDATE_bUartAcquire(void)
{
    uint32_t u32Version   = 0U;
    uint32_t u32FileBytes = 0U;

    if (OTAUPDATE_eQueryLogger(&u32Version, &u32FileBytes) != FARMRANGER_FW_AVAILABLE)
        return false;   /* nothing on offer */

    if (u32Version <= VERSION_u32Get())
    {
        DBG_LOG("OtaUpdate: logger offers v%lu, running v%lu - skip\r\n",
                (unsigned long)u32Version, (unsigned long)VERSION_u32Get());
        return false;
    }

    DBG_LOG("OtaUpdate: acquiring v%lu (%lu file bytes)\r\n",
            (unsigned long)u32Version, (unsigned long)u32FileBytes);

    /* Erase the whole scratchpad up front (~3 s) so no sector erase can
     * stall reception mid-transfer. */
    if (!OTASTORE_bEraseScratch())
    {
        (void)FARMRANGER_bFwReportDone(false);
        return false;
    }

    HEXDECODE_vInit(OTAUPDATE_bStoreDecoded);

    /* Pull the file block by block; the next request is the previous
     * block's acknowledgement, so in-flight data is bounded to one block. */
    for (uint32_t u32Offset = 0U; u32Offset < u32FileBytes;
         u32Offset += OTA_UART_BLOCK_LEN)
    {
        uint32_t u32Remain = u32FileBytes - u32Offset;
        uint16_t u16Len    = (u32Remain > OTA_UART_BLOCK_LEN)
                           ? (uint16_t)OTA_UART_BLOCK_LEN
                           : (uint16_t)u32Remain;

        if (!OTAUPDATE_bFetchAndDecodeBlock(u32Offset, u16Len))
        {
            DBG_LOG("OtaUpdate: acquire FAILED\r\n");
            (void)FARMRANGER_bFwReportDone(false);
            return false;
        }

        /* Progress line every 32 KB of file. */
        if ((u32Offset % 0x8000UL) == 0U && u32Offset > 0U)
            DBG_LOG("OtaUpdate: %lu/%lu kB\r\n",
                    (unsigned long)(u32Offset / 1024U),
                    (unsigned long)(u32FileBytes / 1024U));
    }

    /* Whole file consumed — the decoder must have seen EOF with a matching
     * line count. */
    if (!HEXDECODE_bDone())
    {
        DBG_LOG("OtaUpdate: hex file incomplete (no EOF/line-count match)\r\n");
        (void)FARMRANGER_bFwReportDone(false);
        return false;
    }

    uint32_t u32StopAddr = HEXDECODE_u32StopAddr();
    uint32_t u32Size     = u32StopAddr - OTA_APP_BASE_ADDR + 1UL;
    uint8_t  u8Xor       = OTASTORE_u8CalcImageXor(u32Size);

    if (!OTASTORE_bCommitMetadata(u32Version, u32StopAddr, u8Xor))
    {
        DBG_LOG("OtaUpdate: metadata commit FAILED\r\n");
        (void)FARMRANGER_bFwReportDone(false);
        return false;
    }

    DBG_LOG("OtaUpdate: image v%lu stored OK (size=%luB, xor=0x%02X, map 0x%08lX..0x%08lX)\r\n",
            (unsigned long)u32Version, (unsigned long)u32Size, u8Xor,
            (unsigned long)OTA_APP_BASE_ADDR, (unsigned long)u32StopAddr);

    (void)FARMRANGER_bFwReportDone(true);

    /* Hand off to the (future) bootloader — never returns. */
    OTASTORE_vArmBootloaderAndReset(u32Version);
    return true;   /* not reached */
}

/* ==========================================================================
 * LoRa distribution — primary broadcasts the stored image DIRECTLY (no mesh
 * forwarding) to secondaries in range; missed chunks are repaired per
 * 64-chunk window via poll/report bitmaps.
 *
 * Context split: OTAUPDATE_vOnLoraPacket runs on the MeshParser task and
 * only copies/reacts (a one-chunk mailbox hands image data to the AppTask;
 * PrepAck/Report answers ride the jittered mesh TX queue). All flash work
 * runs on the AppTask in vDistribute / vSecondaryReceive.
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

/* Secondary firmware-acceptance gate (see OtaUpdate.h). A secondary drops
 * every OtaPrep unless armed, so a deliberate "tag <ID> fwaccept" over a
 * kernel session is required before any firmware is taken. */
static volatile bool bFwAcceptArmed;

/* Primary on-demand distribution request (see OtaUpdate.h). */
static volatile bool bDistributeReq;

/* ---- Primary: distribution target table (filled by the parser) ---- */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8Strikes;     /* consecutive unanswered polls                  */
    uint8_t  u8Status;      /* last OTA_RPT_* heard from this target         */
    bool     bAlive;
} OtaTarget_t;
static OtaTarget_t   atTargets[OTA_LORA_MAX_TARGETS];
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
} OtaPrepInfo_t;
static OtaPrepInfo_t     tPrep;
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

static void OTAUPDATE_vPutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static uint16_t OTAUPDATE_u16GetU16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static void OTAUPDATE_vPutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;         p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t OTAUPDATE_u32GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool OTAUPDATE_bBitGet(const uint8_t *pu8Map, uint16_t u16Bit)
{
    return (pu8Map[u16Bit / 8U] & (uint8_t)(1U << (u16Bit % 8U))) != 0U;
}
static void OTAUPDATE_vBitSet(uint8_t *pu8Map, uint16_t u16Bit)
{
    pu8Map[u16Bit / 8U] |= (uint8_t)(1U << (u16Bit % 8U));
}

/* Direct radio TX with queue backpressure (AppTask only). */
static bool OTAUPDATE_bRadioTx(const uint8_t *pu8Buf, uint8_t u8Len)
{
    LoraRadio_Packet_t tPkt;
    memset(&tPkt, 0, sizeof(tPkt));
    memcpy(tPkt.buffer, pu8Buf, u8Len);
    tPkt.length = u8Len;

    for (uint8_t u8Try = 0U; u8Try < 40U; u8Try++)
    {
        if (LORARADIO_bTxPacket(&tPkt))
            return true;
        osDelay(25);   /* radio TX queue full — let the radio task drain */
    }
    return false;
}

/* Wake the AppTask (secondary receive session / prep announcement). */
static void OTAUPDATE_vNotifyAppTask(void)
{
    osThreadId_t xApp = DEVICE_DISCOVERY_xGetTaskHandle();
    if (xApp != NULL)
        osThreadFlagsSet(xApp, DEVICE_DISCOVERY_NOTIFY_OTA);
}

/* --------------------------------------------------------------------------
 * OTAUPDATE_vOnLoraPacket — MeshParser task context: copy/react only
 * -------------------------------------------------------------------------- */
void OTAUPDATE_vOnLoraPacket(const uint8_t *pu8Buf, uint16_t u16Len)
{
    switch (pu8Buf[0])
    {
        case MeshPktType_OtaPrep:
        {
            if (u16Len < OTA_PKT_PREP_LEN) return;
            if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_SECONDARY) return;

            /* Acceptance gate: a secondary takes firmware ONLY after a
             * deliberate "tag <ID> fwaccept" arms it. Unarmed devices ignore
             * every prep so firmware can never be pushed to the field
             * unattended. */
            if (!bFwAcceptArmed) return;

            uint32_t u32Ver = OTAUPDATE_u32GetU32(&pu8Buf[5]);
            if (u32Ver <= VERSION_u32Get())
                return;   /* already running this or newer — not for us */

            tPrep.u32SessionId   = OTAUPDATE_u32GetU32(&pu8Buf[1]);
            tPrep.u32Version     = u32Ver;
            tPrep.u32ImageSize   = OTAUPDATE_u32GetU32(&pu8Buf[9]);
            tPrep.u16TotalChunks = OTAUPDATE_u16GetU16(&pu8Buf[13]);
            tPrep.u8ChunkLen     = pu8Buf[15];
            tPrep.u8ImageXor     = pu8Buf[16];
            /* pu8Buf[17] = window size (informational) */

            if (tPrep.u8ChunkLen == 0U || tPrep.u16TotalChunks == 0U ||
                tPrep.u32ImageSize == 0U || tPrep.u32ImageSize > OTA_APP_MAX_SIZE)
                return;

            u32LastSessionPktTick = osKernelGetTickCount();
            bPrepPending = true;
            OTAUPDATE_vNotifyAppTask();
            break;
        }

        case MeshPktType_OtaPrepAck:
        {
            if (u16Len < OTA_PKT_PREPACK_LEN) return;
            if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY) return;
            if (OTAUPDATE_u32GetU32(&pu8Buf[1]) != u32SessionId) return;

            uint32_t u32Dev = OTAUPDATE_u32GetU32(&pu8Buf[5]);
            for (uint8_t i = 0U; i < u8TargetCount; i++)
                if (atTargets[i].u32DeviceId == u32Dev) return;   /* known */

            if (u8TargetCount < OTA_LORA_MAX_TARGETS)
            {
                atTargets[u8TargetCount].u32DeviceId = u32Dev;
                atTargets[u8TargetCount].u8Strikes   = 0U;
                atTargets[u8TargetCount].u8Status    = OTA_RPT_WINDOW;
                atTargets[u8TargetCount].bAlive      = true;
                u8TargetCount++;
                DBG_LOG("OtaUpdate: target %04lX joined\r\n", (unsigned long)u32Dev);
            }
            break;
        }

        case MeshPktType_OtaChunk:
        {
            if (u16Len < OTA_PKT_CHUNK_HDR_LEN) return;
            if (!bRxSessionActive) return;
            if (OTAUPDATE_u32GetU32(&pu8Buf[1]) != tPrep.u32SessionId) return;

            uint16_t u16Idx = OTAUPDATE_u16GetU16(&pu8Buf[5]);
            uint8_t  u8Len  = pu8Buf[7];
            if (u8Len == 0U || u8Len > OTA_LORA_CHUNK_LEN ||
                (uint16_t)(OTA_PKT_CHUNK_HDR_LEN + u8Len) > u16Len ||
                u16Idx >= tPrep.u16TotalChunks)
                return;

            u32LastSessionPktTick = osKernelGetTickCount();

            /* One-slot mailbox: if the AppTask hasn't consumed the previous
             * chunk yet, drop this one — the window repair recovers it. */
            if (!bChunkMail && !OTAUPDATE_bBitGet(au8ChunkBitmap, u16Idx))
            {
                memcpy(au8ChunkMail, &pu8Buf[OTA_PKT_CHUNK_HDR_LEN], u8Len);
                u16ChunkMailIdx = u16Idx;
                u8ChunkMailLen  = u8Len;
                bChunkMail      = true;
                OTAUPDATE_vNotifyAppTask();
            }
            break;
        }

        case MeshPktType_OtaPoll:
        {
            if (u16Len < OTA_PKT_POLL_LEN) return;
            if (!bRxSessionActive) return;
            if (OTAUPDATE_u32GetU32(&pu8Buf[1]) != tPrep.u32SessionId) return;
            if (OTAUPDATE_u32GetU32(&pu8Buf[5]) != LORARADIO_u32GetUniqueId()) return;

            uint16_t u16First = OTAUPDATE_u16GetU16(&pu8Buf[9]);
            uint8_t  u8Count  = pu8Buf[11];
            if (u8Count > OTA_LORA_WINDOW_CHUNKS) u8Count = OTA_LORA_WINDOW_CHUNKS;

            u32LastSessionPktTick = osKernelGetTickCount();

            /* Report the window's missing chunks as a bitmap. */
            uint8_t au8Rpt[OTA_PKT_REPORT_LEN];
            au8Rpt[0] = (uint8_t)MeshPktType_OtaReport;
            OTAUPDATE_vPutU32(&au8Rpt[1], tPrep.u32SessionId);
            OTAUPDATE_vPutU32(&au8Rpt[5], LORARADIO_u32GetUniqueId());
            au8Rpt[9] = OTA_RPT_WINDOW;
            memset(&au8Rpt[10], 0, OTA_WINDOW_BITMAP_LEN);
            for (uint8_t i = 0U; i < u8Count; i++)
            {
                uint16_t u16Chunk = (uint16_t)(u16First + i);
                if (u16Chunk < tPrep.u16TotalChunks &&
                    !OTAUPDATE_bBitGet(au8ChunkBitmap, u16Chunk))
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
            if (OTAUPDATE_u32GetU32(&pu8Buf[1]) != u32SessionId) return;

            u32ReportFrom  = OTAUPDATE_u32GetU32(&pu8Buf[5]);
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

bool OTAUPDATE_bDistributePending(void)
{
    OtaMeta_t tMeta;
    if (!OTASTORE_bGetMeta(&tMeta))
        return false;
    return (tMeta.u32Version == VERSION_u32Get()) && !tMeta.bDistributed;
}

bool OTAUPDATE_bRequestDistribute(void)
{
    /* Only arm the request when there's actually a valid image to send —
     * distribution reads it straight from the ext-flash scratchpad. Unlike
     * the auto path (bDistributePending) this ignores the version-match and
     * distributed flags: an on-demand request re-sends whatever VALID image
     * is staged, so a bench operator can re-run a distribution at will. */
    OtaMeta_t tMeta;
    if (!OTASTORE_bGetMeta(&tMeta) || !tMeta.bValid)
    {
        DBG_LOG("OtaUpdate: distribute requested but no valid image staged\r\n");
        return false;
    }

    bDistributeReq = true;
    DBG_LOG("OtaUpdate: distribute requested (v%lu, %lu B staged)\r\n",
            (unsigned long)tMeta.u32Version, (unsigned long)tMeta.u32SizeBytes);
    return true;
}

bool OTAUPDATE_bDistributeRequested(void)
{
    return bDistributeReq;
}

void OTAUPDATE_vClearDistributeRequest(void)
{
    bDistributeReq = false;
}

/* Send one image chunk read from the scratchpad. */
static bool OTAUPDATE_bSendChunk(uint16_t u16Chunk, uint32_t u32ImageSize)
{
    uint32_t u32Offset = (uint32_t)u16Chunk * OTA_LORA_CHUNK_LEN;
    uint32_t u32Remain = u32ImageSize - u32Offset;
    uint8_t  u8Len     = (u32Remain > OTA_LORA_CHUNK_LEN)
                       ? (uint8_t)OTA_LORA_CHUNK_LEN : (uint8_t)u32Remain;

    uint8_t au8Pkt[OTA_PKT_CHUNK_HDR_LEN + OTA_LORA_CHUNK_LEN];
    au8Pkt[0] = (uint8_t)MeshPktType_OtaChunk;
    OTAUPDATE_vPutU32(&au8Pkt[1], u32SessionId);
    OTAUPDATE_vPutU16(&au8Pkt[5], u16Chunk);
    au8Pkt[7] = u8Len;

    if (!OTASTORE_bReadImage(u32Offset, &au8Pkt[OTA_PKT_CHUNK_HDR_LEN], u8Len))
        return false;

    if (!OTAUPDATE_bRadioTx(au8Pkt, (uint8_t)(OTA_PKT_CHUNK_HDR_LEN + u8Len)))
        return false;

    osDelay(OTA_LORA_CHUNK_GAP_MS);
    return true;
}

/* Poll one target and merge its missing-window bitmap into pu8Union.
 * Returns true when the target answered. */
static bool OTAUPDATE_bPollTarget(OtaTarget_t *ptTarget, uint16_t u16First,
                                  uint8_t u8Count, uint8_t *pu8Union)
{
    uint8_t au8Poll[OTA_PKT_POLL_LEN];
    au8Poll[0] = (uint8_t)MeshPktType_OtaPoll;
    OTAUPDATE_vPutU32(&au8Poll[1], u32SessionId);
    OTAUPDATE_vPutU32(&au8Poll[5], ptTarget->u32DeviceId);
    OTAUPDATE_vPutU16(&au8Poll[9], u16First);
    au8Poll[11] = u8Count;

    bReportMail = false;
    if (!OTAUPDATE_bRadioTx(au8Poll, sizeof(au8Poll)))
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

void OTAUPDATE_vDistribute(void)
{
    OtaMeta_t tMeta;
    if (!OTASTORE_bGetMeta(&tMeta))
        return;

    uint16_t u16Total = (uint16_t)((tMeta.u32SizeBytes + OTA_LORA_CHUNK_LEN - 1U)
                                   / OTA_LORA_CHUNK_LEN);
    u32SessionId  = MESHNETWORK_u32GenerateGlobalMsgID();
    u8TargetCount = 0U;
    memset(atTargets, 0, sizeof(atTargets));

    DBG_LOG("OtaUpdate: distribute v%lu (%lu B, %u chunks) session %08lX\r\n",
            (unsigned long)tMeta.u32Version, (unsigned long)tMeta.u32SizeBytes,
            u16Total, (unsigned long)u32SessionId);

    /* Announce: repeated OtaPrep instead of this slot's DReq; secondaries
     * that hear it (and run an older version) erase their scratch and ack. */
    uint8_t au8Prep[OTA_PKT_PREP_LEN];
    au8Prep[0] = (uint8_t)MeshPktType_OtaPrep;
    OTAUPDATE_vPutU32(&au8Prep[1],  u32SessionId);
    OTAUPDATE_vPutU32(&au8Prep[5],  tMeta.u32Version);
    OTAUPDATE_vPutU32(&au8Prep[9],  tMeta.u32SizeBytes);
    OTAUPDATE_vPutU16(&au8Prep[13], u16Total);
    au8Prep[15] = (uint8_t)OTA_LORA_CHUNK_LEN;
    au8Prep[16] = tMeta.u8Xor8;
    au8Prep[17] = (uint8_t)OTA_LORA_WINDOW_CHUNKS;

    for (uint8_t i = 0U; i < OTA_LORA_PREP_REPEATS; i++)
    {
        (void)OTAUPDATE_bRadioTx(au8Prep, sizeof(au8Prep));
        osDelay(OTA_LORA_PREP_GAP_MS);
    }
    /* Secondaries erase their scratch (~3 s) before acking; allow for it. */
    osDelay(4000);

    if (u8TargetCount == 0U)
    {
        /* Nobody in range wants this image — try again at future slots
         * until someone does. (Not marked distributed.) */
        DBG_LOG("OtaUpdate: no targets joined\r\n");
        return;
    }

    uint32_t u32SessionStart = osKernelGetTickCount();

    /* Windowed blast + poll + repair. */
    for (uint16_t u16First = 0U; u16First < u16Total;
         u16First = (uint16_t)(u16First + OTA_LORA_WINDOW_CHUNKS))
    {
        uint8_t u8Count = (uint8_t)(((u16Total - u16First) > OTA_LORA_WINDOW_CHUNKS)
                        ? OTA_LORA_WINDOW_CHUNKS : (u16Total - u16First));

        for (uint8_t i = 0U; i < u8Count; i++)
            (void)OTAUPDATE_bSendChunk((uint16_t)(u16First + i), tMeta.u32SizeBytes);

        for (uint8_t u8Round = 0U; u8Round < OTA_LORA_REPAIR_ROUNDS; u8Round++)
        {
            uint8_t au8Union[OTA_WINDOW_BITMAP_LEN] = {0};
            bool    bAnyMissing = false;

            for (uint8_t t = 0U; t < u8TargetCount; t++)
            {
                if (!atTargets[t].bAlive)
                    continue;
                if (OTAUPDATE_bPollTarget(&atTargets[t], u16First, u8Count, au8Union))
                {
                    atTargets[t].u8Strikes = 0U;
                }
                else if (++atTargets[t].u8Strikes >= 2U)
                {
                    atTargets[t].bAlive = false;
                    DBG_LOG("OtaUpdate: target %04lX dropped (silent)\r\n",
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
                    (void)OTAUPDATE_bSendChunk((uint16_t)(u16First + i),
                                               tMeta.u32SizeBytes);
            }
        }

        if ((osKernelGetTickCount() - u32SessionStart) >= OTA_LORA_SESSION_MAX_MS)
        {
            DBG_LOG("OtaUpdate: session hard cap hit\r\n");
            break;
        }
    }

    /* Finalize: give the targets time to XOR-verify (+ send their verdicts,
     * which the parser records into the target table via the report path). */
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
        DBG_LOG("OtaUpdate: target %04lX %s\r\n",
                (unsigned long)atTargets[t].u32DeviceId,
                (atTargets[t].u8Status == OTA_RPT_VALID) ? "UPDATED"
              : (atTargets[t].u8Status == OTA_RPT_ERROR) ? "VERIFY FAILED"
              : atTargets[t].bAlive ? "INCOMPLETE" : "LOST");
    }

    (void)OTASTORE_bMarkDistributed();
    DBG_LOG("OtaUpdate: distribution session done\r\n");
}

/* --------------------------------------------------------------------------
 * Secondary — receive session
 * -------------------------------------------------------------------------- */

bool OTAUPDATE_bPrepPending(void)
{
    return bPrepPending;
}

void OTAUPDATE_vArmAcceptance(void)
{
    bFwAcceptArmed = true;
    DBG_LOG("OtaUpdate: firmware acceptance ARMED\r\n");
}

void OTAUPDATE_vDisarmAcceptance(void)
{
    bFwAcceptArmed = false;
    bPrepPending   = false;   /* drop any prep heard before disarm */
    DBG_LOG("OtaUpdate: firmware acceptance disarmed\r\n");
}

bool OTAUPDATE_bAcceptanceArmed(void)
{
    return bFwAcceptArmed;
}

void OTAUPDATE_vSecondaryReceive(void)
{
    bPrepPending = false;

    /* One-shot: an attempted receive consumes the arm. A retry needs a fresh
     * "tag <ID> fwaccept" — the same deliberate opt-in as the first time. */
    bFwAcceptArmed = false;

    DBG_LOG("OtaUpdate: receiving v%lu (%lu B, %u chunks)\r\n",
            (unsigned long)tPrep.u32Version, (unsigned long)tPrep.u32ImageSize,
            tPrep.u16TotalChunks);

    /* Erase during the announce window so chunk writes never wait on it. */
    if (!OTASTORE_bEraseScratch())
        return;

    memset(au8ChunkBitmap, 0, sizeof(au8ChunkBitmap));
    u16ChunksHave    = 0U;
    bChunkMail       = false;
    bRxSessionActive = true;
    u32LastSessionPktTick = osKernelGetTickCount();

    /* Join the session (jittered — many secondaries answer one prep). */
    uint8_t au8Ack[OTA_PKT_PREPACK_LEN];
    au8Ack[0] = (uint8_t)MeshPktType_OtaPrepAck;
    OTAUPDATE_vPutU32(&au8Ack[1], tPrep.u32SessionId);
    OTAUPDATE_vPutU32(&au8Ack[5], LORARADIO_u32GetUniqueId());
    OTAUPDATE_vPutU32(&au8Ack[9], VERSION_u32Get());
    OTAUPDATE_vPutU16(&au8Ack[13], 0U);   /* nextNeeded: resume support later */
    (void)MESHNETWORK_bSendOtaResponse(au8Ack, sizeof(au8Ack));

    /* Receive loop: the parser mailboxes chunks; this task stores them. */
    uint32_t u32SessionStart = osKernelGetTickCount();
    while (u16ChunksHave < tPrep.u16TotalChunks)
    {
        (void)osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_OTA, osFlagsWaitAny, 500U);

        if (bChunkMail)
        {
            uint32_t u32Offset = (uint32_t)u16ChunkMailIdx * tPrep.u8ChunkLen;
            if (OTASTORE_bWriteImage(u32Offset, au8ChunkMail, u8ChunkMailLen))
            {
                OTAUPDATE_vBitSet(au8ChunkBitmap, u16ChunkMailIdx);
                u16ChunksHave++;
            }
            bChunkMail = false;
        }

        uint32_t u32Now = osKernelGetTickCount();
        if ((u32Now - u32LastSessionPktTick) >= OTA_LORA_RX_IDLE_MS)
        {
            DBG_LOG("OtaUpdate: session went silent (%u/%u chunks) - abort\r\n",
                    u16ChunksHave, tPrep.u16TotalChunks);
            bRxSessionActive = false;
            return;   /* scratch not VALID => harmless */
        }
        if ((u32Now - u32SessionStart) >= OTA_LORA_SESSION_MAX_MS)
        {
            DBG_LOG("OtaUpdate: session hard cap - abort\r\n");
            bRxSessionActive = false;
            return;
        }
    }

    /* Complete: verify against the announced image XOR-8. */
    uint8_t u8Xor = OTASTORE_u8CalcImageXor(tPrep.u32ImageSize);
    bool bValid = (u8Xor == tPrep.u8ImageXor);

    uint8_t au8Rpt[OTA_PKT_REPORT_LEN];
    au8Rpt[0] = (uint8_t)MeshPktType_OtaReport;
    OTAUPDATE_vPutU32(&au8Rpt[1], tPrep.u32SessionId);
    OTAUPDATE_vPutU32(&au8Rpt[5], LORARADIO_u32GetUniqueId());
    au8Rpt[9] = bValid ? OTA_RPT_VALID : OTA_RPT_ERROR;
    memset(&au8Rpt[10], 0, OTA_WINDOW_BITMAP_LEN);

    if (!bValid)
    {
        DBG_LOG("OtaUpdate: image XOR mismatch (0x%02X != 0x%02X)\r\n",
                u8Xor, tPrep.u8ImageXor);
        (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
        bRxSessionActive = false;
        return;
    }

    if (!OTASTORE_bCommitMetadata(tPrep.u32Version,
                                  OTA_APP_BASE_ADDR + tPrep.u32ImageSize - 1UL,
                                  u8Xor))
    {
        bRxSessionActive = false;
        return;
    }

    DBG_LOG("OtaUpdate: image v%lu received + verified OK\r\n",
            (unsigned long)tPrep.u32Version);
    (void)MESHNETWORK_bSendOtaResponse(au8Rpt, sizeof(au8Rpt));
    osDelay(2000);   /* let the (jittered) report leave the radio */

    bRxSessionActive = false;

    /* Hand off to the (future) bootloader — never returns. */
    OTASTORE_vArmBootloaderAndReset(tPrep.u32Version);
}

#endif /* STORAGE_BACKEND_FLASH */
