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

#include "storage_config.h"

#ifdef STORAGE_BACKEND_FLASH

#include "OtaUpdate.h"
#include "OtaUpdate_Config.h"

#include "Farmranger.h"
#include "HexDecode.h"
#include "OtaStore.h"
#include "OtaStore_Config.h"
#include "Version.h"

#include "cmsis_os2.h"
#include "dbg_log.h"

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

#endif /* STORAGE_BACKEND_FLASH */
