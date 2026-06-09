/*
 * Log.c
 *
 * Circular FIFO text log on external NOR flash.
 *
 * Synchronous flash primitive: LOG_vWrite() programs bytes directly. It is
 * called only from the single DbgLog consumer task (see DbgLog.c), which owns
 * all buffering — so callers of DBG/LOG/DBG_LOG never touch the flash and never
 * see its latency.
 *
 * Flash FIFO: u32WriteAddr is the head, u32StartAddr is the tail. On every
 * sector-boundary crossing the sector is erased first; once wrapped, the tail
 * advances past each freshly-erased sector so FIFO order is maintained.
 *
 * Init: binary search at sector granularity to find the first erased sector,
 * then a byte-scan within it for the exact write head.
 */

#include "Log.h"
#include "Flash.h"
#include "Flash_Config.h"
#include "Debug.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static uint32_t u32WriteAddr;
static uint32_t u32StartAddr;
static bool     bWrapped;

/* -------------------------------------------------------------------------- */

/*
 * Binary search over [0..FLASH_NUM_SECTORS) to locate the first sector whose
 * first byte is 0xFF (erased). Returns the sector index, or FLASH_NUM_SECTORS
 * if all sectors have data.
 */
static uint32_t LOG_u32FindFirstErasedSector(void)
{
    uint32_t lo = 0U;
    uint32_t hi = FLASH_NUM_SECTORS;

    while (lo < hi)
    {
        uint32_t mid  = (lo + hi) / 2U;
        uint32_t addr = LOG_FLASH_START_ADDR + mid * FLASH_SECTOR_SIZE_BYTES;
        uint8_t  byte;
        FLASH_vRead(addr, &byte, 1);
        if (byte == 0xFFU)
            hi = mid;
        else
            lo = mid + 1U;
    }
    return lo;
}

/* -------------------------------------------------------------------------- */

void LOG_vInit(void)
{
    uint32_t sectorIdx = LOG_u32FindFirstErasedSector();

    if (sectorIdx == 0U)
    {
        /* Log is empty */
        u32WriteAddr = LOG_FLASH_START_ADDR;
        u32StartAddr = LOG_FLASH_START_ADDR;
        bWrapped     = false;
    }
    else if (sectorIdx == FLASH_NUM_SECTORS)
    {
        /* All sectors have data — log has fully wrapped */
        u32WriteAddr = LOG_FLASH_START_ADDR;
        u32StartAddr = LOG_FLASH_START_ADDR;
        bWrapped     = true;
    }
    else
    {
        /* Fine-scan within the first erased sector for exact byte boundary */
        uint32_t sectorStart = LOG_FLASH_START_ADDR + sectorIdx * FLASH_SECTOR_SIZE_BYTES;
        uint32_t sectorEnd   = sectorStart + FLASH_SECTOR_SIZE_BYTES;
        uint32_t addr        = sectorStart;
        uint8_t  byte;

        while (addr < sectorEnd)
        {
            FLASH_vRead(addr, &byte, 1);
            if (byte == 0xFFU) break;
            addr++;
        }

        u32WriteAddr = addr;
        u32StartAddr = LOG_FLASH_START_ADDR;
        bWrapped     = false;
    }
}

/* -------------------------------------------------------------------------- */

void LOG_vWrite(const char *buf, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        /* Erase the sector when crossing into a new one */
        if ((u32WriteAddr % FLASH_SECTOR_SIZE_BYTES) == 0U)
        {
            FLASH_vSectorErase(u32WriteAddr);
            if (bWrapped)
            {
                /* Advance tail past the freshly-erased sector */
                u32StartAddr = u32WriteAddr + FLASH_SECTOR_SIZE_BYTES;
                if (u32StartAddr >= LOG_FLASH_END_ADDR)
                    u32StartAddr = LOG_FLASH_START_ADDR;
            }
        }

        FLASH_vPageWrite(u32WriteAddr, (const uint8_t *)&buf[i], 1U);
        u32WriteAddr++;

        if (u32WriteAddr >= LOG_FLASH_END_ADDR)
        {
            u32WriteAddr = LOG_FLASH_START_ADDR;
            bWrapped     = true;
        }
    }
}

/* -------------------------------------------------------------------------- */

uint32_t LOG_u32GetUsedBytes(void)
{
    if (!bWrapped)
        return u32WriteAddr - LOG_FLASH_START_ADDR;

    if (u32WriteAddr >= u32StartAddr)
        return u32WriteAddr - u32StartAddr;

    return (LOG_FLASH_END_ADDR - u32StartAddr) + (u32WriteAddr - LOG_FLASH_START_ADDR);
}

uint8_t LOG_u8GetUsedPercent(void)
{
    return (uint8_t)(LOG_u32GetUsedBytes() * 100UL / FLASH_CAPACITY_BYTES);
}

/* -------------------------------------------------------------------------- */

void LOG_vErase(void)
{
    FLASH_vChipErase();
    u32WriteAddr = LOG_FLASH_START_ADDR;
    u32StartAddr = LOG_FLASH_START_ADDR;
    bWrapped     = false;
}

/* -------------------------------------------------------------------------- */

void LOG_vStreamToDebug(void)
{
    char hdr[64];
    int  n = snprintf(hdr, sizeof(hdr),
                      "\r\n==== EXT-FLASH LOG DUMP (%lu bytes) ====\r\n",
                      (unsigned long)LOG_u32GetUsedBytes());
    if (n > 0)
        DEBUG_vPutBuffer((const uint8_t *)hdr, (uint16_t)n);

    uint32_t u32Remaining = LOG_u32GetUsedBytes();
    uint32_t u32Addr      = u32StartAddr;        /* FIFO tail = oldest byte */
    uint8_t  au8Chunk[64];

    while (u32Remaining > 0U)
    {
        uint16_t u16Chunk = (u32Remaining > sizeof(au8Chunk))
                          ? (uint16_t)sizeof(au8Chunk)
                          : (uint16_t)u32Remaining;

        if ((u32Addr + u16Chunk) > LOG_FLASH_END_ADDR)
            u16Chunk = (uint16_t)(LOG_FLASH_END_ADDR - u32Addr);

        FLASH_vRead(u32Addr, au8Chunk, u16Chunk);
        DEBUG_vPutBuffer(au8Chunk, u16Chunk);

        u32Addr += u16Chunk;
        if (u32Addr >= LOG_FLASH_END_ADDR)
            u32Addr = LOG_FLASH_START_ADDR;      /* wrap */

        u32Remaining -= u16Chunk;
    }

    static const char end[] = "\r\n==== EXT-FLASH LOG DUMP END ====\r\n";
    DEBUG_vPutBuffer((const uint8_t *)end, (uint16_t)(sizeof(end) - 1U));
}
