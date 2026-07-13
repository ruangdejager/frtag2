/*
 * OtaStore.c
 *
 * OTA image store on the external NOR flash. See OtaStore_Config.h for the
 * partition map, metadata layout and the bootloader handoff contract.
 *
 * Concurrency: all callers run on the DeviceDiscovery AppTask (the OTA
 * acquire / distribute / receive state machines are sequential phases of a
 * wake slot), so no lock is needed beyond what the Flash driver provides.
 * The DbgLog consumer shares the flash device but only touches the log
 * partition (>= 0x3C000).
 */

#include "build_config.h"   /* STORAGE_BACKEND_FLASH (was storage_config.h) */

#ifdef STORAGE_BACKEND_FLASH

#include "OtaStore.h"
#include "OtaStore_Config.h"
#include "OtaStore_Driver.h"

#include "cmsis_os2.h"
#include "stm32wlxx_hal.h"
#include "dbg_log.h"

#include <string.h>

/* -------------------------------------------------------------------------- */

static uint32_t OTASTORE_u32GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void OTASTORE_vPutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* --------------------------------------------------------------------------
 * OTASTORE_vInit
 * -------------------------------------------------------------------------- */
void OTASTORE_vInit(void)
{
    /* One-time layout migration: before the OTA partitions existed the text
     * log owned the whole chip, so on first boot with this layout the
     * metadata sector holds stale log text — neither blank nor a record.
     * Erase it once so bGetMeta() can't misread old text as metadata. The
     * stale bytes in the scratch region are harmless (erased before use). */
    uint8_t au8Head[4];
    if (!OTASTORE_DRIVER_bRead(OTA_META_ADDR, au8Head, sizeof(au8Head)))
        return;

    bool bBlank = (au8Head[0] == 0xFFU) && (au8Head[1] == 0xFFU) &&
                  (au8Head[2] == 0xFFU) && (au8Head[3] == 0xFFU);
    bool bMagic = (OTASTORE_u32GetU32(au8Head) == OTA_META_MAGIC);

    if (!bBlank && !bMagic)
    {
        DBG_LOG("OtaStore: migrating metadata sector (stale layout)\r\n");
        (void)OTASTORE_DRIVER_bSectorErase(OTA_META_SECTOR_ADDR);
    }
}

/* --------------------------------------------------------------------------
 * OTASTORE_bEraseScratch
 * -------------------------------------------------------------------------- */
bool OTASTORE_bEraseScratch(void)
{
    /* Also erases the metadata sector: a new transfer invalidates whatever
     * image was stored before (its VALID marker must not survive). */
    for (uint32_t u32Addr = OTA_SCRATCH_START_ADDR;
         u32Addr <= OTA_META_SECTOR_ADDR;
         u32Addr += OTASTORE_DRIVER_SECTOR_SIZE)
    {
        if (!OTASTORE_DRIVER_bSectorErase(u32Addr))
        {
            DBG_LOG("OtaStore: scratch erase FAILED @0x%05lX\r\n",
                    (unsigned long)u32Addr);
            return false;
        }
        osDelay(1);   /* yield between sectors (~45 ms each, ~3 s total) */
    }
    return true;
}

/* --------------------------------------------------------------------------
 * OTASTORE_bWriteImage
 * -------------------------------------------------------------------------- */
bool OTASTORE_bWriteImage(uint32_t u32Offset, const uint8_t *pu8Data, uint16_t u16Len)
{
    if ((u32Offset + u16Len) > OTA_SCRATCH_SIZE)
        return false;   /* image would overrun the scratchpad */

    uint32_t u32Addr = OTA_SCRATCH_START_ADDR + u32Offset;
    uint16_t u16Off  = 0U;

    while (u16Off < u16Len)
    {
        /* Split at flash page boundaries (page program can't cross them). */
        uint32_t u32ToPageEnd = OTASTORE_DRIVER_PAGE_SIZE
                              - (u32Addr % OTASTORE_DRIVER_PAGE_SIZE);
        uint16_t u16Chunk     = (uint16_t)(u16Len - u16Off);
        if (u16Chunk > u32ToPageEnd)
            u16Chunk = (uint16_t)u32ToPageEnd;

        if (!OTASTORE_DRIVER_bWrite(u32Addr, &pu8Data[u16Off], u16Chunk))
            return false;

        u32Addr += u16Chunk;
        u16Off   = (uint16_t)(u16Off + u16Chunk);
    }
    return true;
}

/* --------------------------------------------------------------------------
 * OTASTORE_bReadImage
 * -------------------------------------------------------------------------- */
bool OTASTORE_bReadImage(uint32_t u32Offset, uint8_t *pu8Buf, uint16_t u16Len)
{
    if ((u32Offset + u16Len) > OTA_SCRATCH_SIZE)
        return false;
    return OTASTORE_DRIVER_bRead(OTA_SCRATCH_START_ADDR + u32Offset, pu8Buf, u16Len);
}

/* --------------------------------------------------------------------------
 * OTASTORE_u8CalcImageXor
 * -------------------------------------------------------------------------- */
uint8_t OTASTORE_u8CalcImageXor(uint32_t u32SizeBytes)
{
    uint8_t  au8Buf[OTA_XOR_BUF_LEN];
    uint8_t  u8Xor    = 0U;
    uint32_t u32Addr  = OTA_SCRATCH_START_ADDR;
    uint32_t u32Remain = u32SizeBytes;

    while (u32Remain > 0U)
    {
        uint16_t u16Chunk = (u32Remain > sizeof(au8Buf))
                          ? (uint16_t)sizeof(au8Buf)
                          : (uint16_t)u32Remain;

        if (!OTASTORE_DRIVER_bRead(u32Addr, au8Buf, u16Chunk))
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

/* --------------------------------------------------------------------------
 * OTASTORE_bCommitMetadata
 * -------------------------------------------------------------------------- */
bool OTASTORE_bCommitMetadata(uint32_t u32Version, uint32_t u32StopAddr, uint8_t u8Xor8)
{
    /* The metadata sector was erased with the scratch; write the record
     * first, VALID last, so a reset in between never leaves a half-committed
     * record that passes validation. */
    uint8_t au8Rec[OTA_META_RECORD_LEN];
    memset(au8Rec, 0xFF, sizeof(au8Rec));

    OTASTORE_vPutU32(&au8Rec[OTA_META_OFF_MAGIC],     OTA_META_MAGIC);
    OTASTORE_vPutU32(&au8Rec[OTA_META_OFF_VERSION],   u32Version);
    OTASTORE_vPutU32(&au8Rec[OTA_META_OFF_STOP_ADDR], u32StopAddr);
    OTASTORE_vPutU32(&au8Rec[OTA_META_OFF_SIZE],
                     u32StopAddr - OTA_APP_BASE_ADDR + 1UL);
    au8Rec[OTA_META_OFF_XOR8] = u8Xor8;
    /* VALID/CONSUMED/DISTRIBUTED stay 0xFF in this pass. */

    if (!OTASTORE_DRIVER_bWrite(OTA_META_ADDR, au8Rec, sizeof(au8Rec)))
        return false;

    uint8_t u8Marker = OTA_META_MARKER;
    return OTASTORE_DRIVER_bWrite(OTA_META_ADDR + OTA_META_OFF_VALID, &u8Marker, 1U);
}

/* --------------------------------------------------------------------------
 * OTASTORE_bGetMeta
 * -------------------------------------------------------------------------- */
bool OTASTORE_bGetMeta(OtaMeta_t *ptMeta)
{
    uint8_t au8Rec[OTA_META_RECORD_LEN];

    memset(ptMeta, 0, sizeof(*ptMeta));

    if (!OTASTORE_DRIVER_bRead(OTA_META_ADDR, au8Rec, sizeof(au8Rec)))
        return false;

    if (OTASTORE_u32GetU32(&au8Rec[OTA_META_OFF_MAGIC]) != OTA_META_MAGIC)
        return false;

    ptMeta->u32Version   = OTASTORE_u32GetU32(&au8Rec[OTA_META_OFF_VERSION]);
    ptMeta->u32StopAddr  = OTASTORE_u32GetU32(&au8Rec[OTA_META_OFF_STOP_ADDR]);
    ptMeta->u32SizeBytes = OTASTORE_u32GetU32(&au8Rec[OTA_META_OFF_SIZE]);
    ptMeta->u8Xor8       = au8Rec[OTA_META_OFF_XOR8];
    ptMeta->bValid       = (au8Rec[OTA_META_OFF_VALID]       == OTA_META_MARKER);
    ptMeta->bConsumed    = (au8Rec[OTA_META_OFF_CONSUMED]    == OTA_META_MARKER);
    ptMeta->bDistributed = (au8Rec[OTA_META_OFF_DISTRIBUTED] == OTA_META_MARKER);

    /* Size sanity: reject a record whose image can't map into the app region. */
    if (ptMeta->u32SizeBytes == 0UL || ptMeta->u32SizeBytes > OTA_APP_MAX_SIZE)
        return false;

    return ptMeta->bValid;
}

/* --------------------------------------------------------------------------
 * OTASTORE_bMarkDistributed
 * -------------------------------------------------------------------------- */
bool OTASTORE_bMarkDistributed(void)
{
    uint8_t u8Marker = OTA_META_MARKER;
    return OTASTORE_DRIVER_bWrite(OTA_META_ADDR + OTA_META_OFF_DISTRIBUTED,
                                  &u8Marker, 1U);
}

/* --------------------------------------------------------------------------
 * OTASTORE_vArmBootloaderAndReset
 * -------------------------------------------------------------------------- */
void OTASTORE_vArmBootloaderAndReset(uint32_t u32Version)
{
    DBG_LOG("OtaStore: arming bootloader (v%lu) and resetting\r\n",
            (unsigned long)u32Version);
    osDelay(100);   /* let the log line drain */

    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = OTA_BOOT_MAGIC;
    TAMP->BKP1R = u32Version;

    NVIC_SystemReset();
}

#endif /* STORAGE_BACKEND_FLASH */
