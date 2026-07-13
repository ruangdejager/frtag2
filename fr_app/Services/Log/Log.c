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

#include "build_config.h"

#ifdef STORAGE_BACKEND_FLASH

#include "Log.h"
#include "Flash.h"
#include "Flash_Config.h"
#include "Debug.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Number of 4 KB sectors in the log PARTITION (not the whole device — the
 * lower sectors belong to the OTA scratchpad, see OtaStore_Config.h). */
#define LOG_NUM_SECTORS  (LOG_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE_BYTES)

static uint32_t u32WriteAddr;
static uint32_t u32StartAddr;
static bool     bWrapped;

/* -------------------------------------------------------------------------- */

/*
 * Returns the byte at offset (FLASH_SECTOR_SIZE_BYTES - 1) of the given
 * sector — i.e. its last byte.
 */
static uint8_t LOG_u8ReadSectorTailByte(uint32_t sector)
{
    uint8_t  byte;
    uint32_t addr = LOG_FLASH_START_ADDR + sector * FLASH_SECTOR_SIZE_BYTES
                  + (FLASH_SECTOR_SIZE_BYTES - 1U);
    FLASH_vRead(addr, &byte, 1);
    return byte;
}

/*
 * Find the FIFO write-head sector: the one (and only) sector that still has
 * unused 0xFF padding at its tail. All other sectors — whether holding the
 * current generation's data or an older, not-yet-overwritten generation —
 * are filled right to their last byte. Returns LOG_NUM_SECTORS if no such
 * sector is found (the degenerate case where every sector is filled to
 * exactly its last byte).
 */
static uint32_t LOG_u32FindHeadByTailPadding(void)
{
    for (uint32_t sector = 0U; sector < LOG_NUM_SECTORS; sector++)
    {
        if (LOG_u8ReadSectorTailByte(sector) == 0xFFU)
            return sector;
    }
    return LOG_NUM_SECTORS;
}

/* -------------------------------------------------------------------------- */

void LOG_vInit(void)
{
    /* Pass 1: first byte of every sector. A sector whose first byte is
     * 0xFF is virgin — never written since the last chip erase. */
    uint32_t u32FirstVirgin  = LOG_NUM_SECTORS;
    uint32_t u32VirginCount  = 0U;

    for (uint32_t sector = 0U; sector < LOG_NUM_SECTORS; sector++)
    {
        uint8_t  byte;
        FLASH_vRead(LOG_FLASH_START_ADDR + sector * FLASH_SECTOR_SIZE_BYTES, &byte, 1);
        if (byte == 0xFFU)
        {
            if (u32FirstVirgin == LOG_NUM_SECTORS)
                u32FirstVirgin = sector;
            u32VirginCount++;
        }
    }

    uint32_t u32HeadSector;
    uint32_t u32TailSector;

    if ((u32FirstVirgin != LOG_NUM_SECTORS) &&
        (u32VirginCount == (LOG_NUM_SECTORS - u32FirstVirgin)))
    {
        /* The virgin sectors form a clean suffix up to the partition end — the
         * FIFO has never wrapped. The write head is the last *written*
         * sector (or sector 0 if the log is completely empty); the tail
         * (oldest data) is fixed at sector 0. */
        u32HeadSector = (u32FirstVirgin == 0U) ? 0U : (u32FirstVirgin - 1U);
        u32TailSector = 0U;
        bWrapped      = false;
    }
    else
    {
        /* The FIFO has wrapped at least once: every sector holds data from
         * some generation, so the write head can no longer be identified by
         * its first byte. Find it instead by its trailing 0xFF padding. The
         * sector right after it holds the oldest surviving data. */
        u32HeadSector = LOG_u32FindHeadByTailPadding();
        if (u32HeadSector == LOG_NUM_SECTORS)
        {
            /* Degenerate: every sector filled to its last byte. Fall back to
             * sector 0 — the next write will erase it and re-establish a
             * normal FIFO layout. */
            u32HeadSector = 0U;
        }
        u32TailSector = (u32HeadSector + 1U) % LOG_NUM_SECTORS;
        bWrapped      = true;
    }

    /* Fine-scan the head sector for the exact write-pointer byte offset. */
    uint32_t sectorStart = LOG_FLASH_START_ADDR + u32HeadSector * FLASH_SECTOR_SIZE_BYTES;
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
    u32StartAddr = LOG_FLASH_START_ADDR + u32TailSector * FLASH_SECTOR_SIZE_BYTES;
}

/* -------------------------------------------------------------------------- */

/*
 * Writes are batched into page-aligned chunks (one FLASH_vPageWrite per
 * chunk) instead of one SPI page-program transaction per byte. A 50-byte
 * log line used to mean 50 separate select/cmd/wait-ready SPI transactions
 * — each one a window in which a campaign burst could preempt the consumer
 * task past the SPI timeout and leave a truncated command on the bus. One
 * (or, across a page boundary, two) transactions per line closes almost all
 * of that window. If a chunk write fails (device busy/unresponsive beyond
 * the bounded wait), the remainder of this call is dropped rather than
 * risking a write at a stale address.
 */
void LOG_vWrite(const char *buf, uint16_t len)
{
    uint16_t off = 0U;

    while (off < len)
    {
        /* Erase the sector when crossing into a new one */
        if ((u32WriteAddr % FLASH_SECTOR_SIZE_BYTES) == 0U)
        {
            if (!FLASH_vSectorErase(u32WriteAddr))
                return;

            if (bWrapped)
            {
                /* Advance tail past the freshly-erased sector */
                u32StartAddr = u32WriteAddr + FLASH_SECTOR_SIZE_BYTES;
                if (u32StartAddr >= LOG_FLASH_END_ADDR)
                    u32StartAddr = LOG_FLASH_START_ADDR;
            }
        }

        uint32_t u32ToPageEnd   = FLASH_PAGE_SIZE_BYTES   - (u32WriteAddr % FLASH_PAGE_SIZE_BYTES);
        uint32_t u32ToSectorEnd = FLASH_SECTOR_SIZE_BYTES - (u32WriteAddr % FLASH_SECTOR_SIZE_BYTES);
        uint32_t u32Max         = (u32ToPageEnd < u32ToSectorEnd) ? u32ToPageEnd : u32ToSectorEnd;

        uint16_t u16Chunk = (uint16_t)(len - off);
        if (u16Chunk > u32Max)
            u16Chunk = (uint16_t)u32Max;

        if (!FLASH_vPageWrite(u32WriteAddr, (const uint8_t *)&buf[off], u16Chunk))
            return;

        u32WriteAddr += u16Chunk;
        off          += u16Chunk;

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
    return (uint8_t)(LOG_u32GetUsedBytes() * 100UL / LOG_FLASH_SIZE_BYTES);
}

/* -------------------------------------------------------------------------- */

void LOG_vErase(void)
{
    /* Per-sector erase of the log PARTITION only. A chip erase would also
     * wipe the OTA image scratchpad + metadata in the lower sectors. */
    for (uint32_t u32Addr = LOG_FLASH_START_ADDR;
         u32Addr < LOG_FLASH_END_ADDR;
         u32Addr += FLASH_SECTOR_SIZE_BYTES)
    {
        if (!FLASH_vSectorErase(u32Addr))
            break;
    }
    u32WriteAddr = LOG_FLASH_START_ADDR;
    u32StartAddr = LOG_FLASH_START_ADDR;
    bWrapped     = false;
}

/* -------------------------------------------------------------------------- */

void LOG_vStreamToDebug(void)
{
    LOG_vStreamViaSink(DEBUG_vPutBuffer);
}

void LOG_vStreamViaSink(void (*sink)(const uint8_t *data, uint16_t len))
{
    char hdr[64];
    int  n = snprintf(hdr, sizeof(hdr),
                      "\r\n==== EXT-FLASH LOG DUMP (%lu bytes) ====\r\n",
                      (unsigned long)LOG_u32GetUsedBytes());
    if (n > 0)
        sink((const uint8_t *)hdr, (uint16_t)n);

    uint32_t u32Remaining = LOG_u32GetUsedBytes();
    uint32_t u32Addr      = u32StartAddr;        /* FIFO tail = oldest byte */
    /* 200 B: comfortably under the LoRa FrKernel per-packet payload cap
     * (LORA_MAX_PACKET_SIZE - 3 = 253 B), so a chunk normally maps to exactly
     * one radio packet, while still being far fewer, larger flash reads /
     * sink calls than the old 64 B chunking (which throttled throughput over
     * both the debug UART and, worse, LoRa). A sink that needs a smaller
     * unit (or gets handed a larger one, e.g. MicroSD's block-sized chunks)
     * is free to split further itself. */
    uint8_t  au8Chunk[200];

    while (u32Remaining > 0U)
    {
        uint16_t u16Chunk = (u32Remaining > sizeof(au8Chunk))
                          ? (uint16_t)sizeof(au8Chunk)
                          : (uint16_t)u32Remaining;

        if ((u32Addr + u16Chunk) > LOG_FLASH_END_ADDR)
            u16Chunk = (uint16_t)(LOG_FLASH_END_ADDR - u32Addr);

        FLASH_vRead(u32Addr, au8Chunk, u16Chunk);
        sink(au8Chunk, u16Chunk);

        u32Addr += u16Chunk;
        if (u32Addr >= LOG_FLASH_END_ADDR)
            u32Addr = LOG_FLASH_START_ADDR;      /* wrap */

        u32Remaining -= u16Chunk;
    }

    static const char end[] = "\r\n==== EXT-FLASH LOG DUMP END ====\r\n";
    sink((const uint8_t *)end, (uint16_t)(sizeof(end) - 1U));
}

/* -------------------------------------------------------------------------- */

void LOG_vPark(void)
{
    /* Park the NOR flash in deep-power-down for the idle/STOP2 period. No-op
     * if it was never woken; the next flash access transparently resumes it. */
    FLASH_vDeepPowerDown();
}

#endif /* STORAGE_BACKEND_FLASH */
