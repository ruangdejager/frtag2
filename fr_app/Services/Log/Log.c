/*
 * Log.c
 *
 * Circular FIFO text log on external NOR flash.
 *
 * Init: binary search at sector granularity (max 128 SPI reads) to find
 * the first erased sector, then byte-scan within that sector for the exact
 * write head.  Much faster than a full 512 KB linear scan.
 *
 * Write: bytes are written one at a time.  On every sector-boundary
 * crossing the sector is erased first.  When already wrapped, the tail
 * pointer advances past each freshly-erased sector so the log maintains
 * FIFO order.
 */

#include "Log.h"
#include "Flash.h"
#include "Flash_Config.h"
#include <stdbool.h>

static uint32_t u32WriteAddr;
static uint32_t u32StartAddr;
static bool     bWrapped;

/* -------------------------------------------------------------------------- */

/*
 * Binary search over [0..FLASH_NUM_SECTORS) to locate the first sector whose
 * first byte is 0xFF (erased).
 *
 * Returns the sector index, or FLASH_NUM_SECTORS if all sectors have data.
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
        return;
    }

    if (sectorIdx == FLASH_NUM_SECTORS)
    {
        /* All sectors have data — log has fully wrapped */
        u32WriteAddr = LOG_FLASH_START_ADDR;
        u32StartAddr = LOG_FLASH_START_ADDR;
        bWrapped     = true;
        return;
    }

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

/* -------------------------------------------------------------------------- */

void LOG_vWrite(const char *buf, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        /* Erase sector when crossing into a new one */
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
