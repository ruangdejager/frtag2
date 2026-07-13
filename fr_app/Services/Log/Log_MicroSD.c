/*
 * Log_MicroSD.c
 *
 * Circular FIFO text log on the MicroSD card's LOG region (raw blocks, no
 * filesystem). Provides the same Log.h API as the NOR-flash Log.c; exactly
 * one of the two is an active translation unit (build_config.h).
 *
 * Layout of the LOG region (blocks [SD_LOG_LBA_START, SD_ACC_LBA_START)):
 *   - block SD_LOG_LBA_START          : superblock (head/tail pointers)
 *   - blocks SD_LOG_LBA_START+1 ..end : circular ring of 512-byte data blocks
 *
 * SD has no erase/0xFF semantics, so the flash log's "scan for erased padding"
 * recovery doesn't apply and scanning 32k blocks at boot would be far too slow.
 * Instead the head/tail live in the superblock, rewritten whenever a data block
 * fills or the device is parked. The block currently being filled is staged in
 * RAM (au8Cur); LOG_vPark() flushes that partial block + superblock so the most
 * recent lines survive a reset. Only the unflushed bytes written since the last
 * park (always < 512) can be lost on an unexpected reset.
 */

#include "build_config.h"

#ifdef STORAGE_BACKEND_MICROSD

#include "Log.h"
#include "MicroSD.h"
#include "MicroSD_Config.h"
#include "Debug.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Ring of data blocks (the superblock occupies the first block of the region). */
#define LOGSD_SUPER_LBA     (SD_LOG_LBA_START)
#define LOGSD_DATA_FIRST    (SD_LOG_LBA_START + 1UL)
#define LOGSD_DATA_END      (SD_ACC_LBA_START)
#define LOGSD_DATA_COUNT    (LOGSD_DATA_END - LOGSD_DATA_FIRST)

#define LOGSD_SUPER_MAGIC   0x31474F4CUL   /* "LOG1" */

static uint8_t  au8Cur[SD_BLOCK_SIZE];     /* current (partially filled) block  */
static uint8_t  au8Scratch[SD_BLOCK_SIZE]; /* superblock I/O + stream read buf  */
static uint16_t u16CurLen;                 /* valid bytes in au8Cur (0..512)    */
static uint32_t u32WriteLba;               /* data block being filled           */
static uint32_t u32StartLba;               /* oldest data block (FIFO tail)      */
static bool     bInited;

/* -------------------------------------------------------------------------- */

static void LOGSD_vPutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t LOGSD_u32GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t LOGSD_u32Advance(uint32_t u32Lba)
{
    u32Lba++;
    if (u32Lba >= LOGSD_DATA_END)
        u32Lba = LOGSD_DATA_FIRST;
    return u32Lba;
}

/* Number of full data blocks held (modular distance tail->head). */
static uint32_t LOGSD_u32FullBlocks(void)
{
    uint32_t u32WriteIdx = u32WriteLba - LOGSD_DATA_FIRST;
    uint32_t u32StartIdx = u32StartLba - LOGSD_DATA_FIRST;
    return (u32WriteIdx + LOGSD_DATA_COUNT - u32StartIdx) % LOGSD_DATA_COUNT;
}

static void LOGSD_vPersistSuper(void)
{
    memset(au8Scratch, 0, sizeof(au8Scratch));
    LOGSD_vPutU32(&au8Scratch[0],  LOGSD_SUPER_MAGIC);
    LOGSD_vPutU32(&au8Scratch[4],  u32WriteLba);
    LOGSD_vPutU32(&au8Scratch[8],  u32StartLba);
    LOGSD_vPutU32(&au8Scratch[12], (uint32_t)u16CurLen);
    (void)MICROSD_bWriteBlock(LOGSD_SUPER_LBA, au8Scratch);
}

static void LOGSD_vReset(void)
{
    u32WriteLba = LOGSD_DATA_FIRST;
    u32StartLba = LOGSD_DATA_FIRST;
    u16CurLen   = 0U;
    memset(au8Cur, 0, sizeof(au8Cur));
    LOGSD_vPersistSuper();
}

/* -------------------------------------------------------------------------- */

void LOG_vInit(void)
{
    bInited = false;

    if (!MICROSD_bIsReady())
        return;   /* MICROSD_vInit() failed earlier; leave logging disabled */

    if (MICROSD_bReadBlock(LOGSD_SUPER_LBA, au8Scratch) &&
        (LOGSD_u32GetU32(&au8Scratch[0]) == LOGSD_SUPER_MAGIC))
    {
        u32WriteLba = LOGSD_u32GetU32(&au8Scratch[4]);
        u32StartLba = LOGSD_u32GetU32(&au8Scratch[8]);
        u16CurLen   = (uint16_t)LOGSD_u32GetU32(&au8Scratch[12]);

        /* Sanity-clamp; fall back to a clean ring on anything inconsistent. */
        if (u32WriteLba < LOGSD_DATA_FIRST || u32WriteLba >= LOGSD_DATA_END ||
            u32StartLba < LOGSD_DATA_FIRST || u32StartLba >= LOGSD_DATA_END ||
            u16CurLen > SD_BLOCK_SIZE)
        {
            LOGSD_vReset();
        }
        else if (u16CurLen > 0U)
        {
            /* Re-load the persisted partial block so appends continue after it. */
            if (!MICROSD_bReadBlock(u32WriteLba, au8Cur))
                u16CurLen = 0U;
        }
    }
    else
    {
        LOGSD_vReset();   /* fresh / unrecognised card */
    }

    bInited = true;
}

/* -------------------------------------------------------------------------- */

void LOG_vWrite(const char *buf, uint16_t len)
{
    if (!bInited)
        return;

    uint16_t u16Off = 0U;
    while (u16Off < len)
    {
        uint16_t u16Space = (uint16_t)(SD_BLOCK_SIZE - u16CurLen);
        uint16_t u16Chunk = (uint16_t)(len - u16Off);
        if (u16Chunk > u16Space)
            u16Chunk = u16Space;

        memcpy(&au8Cur[u16CurLen], &buf[u16Off], u16Chunk);
        u16CurLen = (uint16_t)(u16CurLen + u16Chunk);
        u16Off    = (uint16_t)(u16Off + u16Chunk);

        if (u16CurLen == SD_BLOCK_SIZE)
        {
            /* Block full: commit it and advance the head, dropping the oldest
             * block if the ring is now full. */
            if (!MICROSD_bWriteBlock(u32WriteLba, au8Cur))
                return;   /* write failed — abandon the rest of this call */

            uint32_t u32Next = LOGSD_u32Advance(u32WriteLba);
            if (u32Next == u32StartLba)
                u32StartLba = LOGSD_u32Advance(u32StartLba);
            u32WriteLba = u32Next;
            u16CurLen   = 0U;

            LOGSD_vPersistSuper();
        }
    }
}

/* -------------------------------------------------------------------------- */

uint32_t LOG_u32GetUsedBytes(void)
{
    if (!bInited)
        return 0U;
    return LOGSD_u32FullBlocks() * SD_BLOCK_SIZE + u16CurLen;
}

uint8_t LOG_u8GetUsedPercent(void)
{
    if (!bInited)
        return 0U;
    uint64_t u64Cap = (uint64_t)LOGSD_DATA_COUNT * SD_BLOCK_SIZE;
    return (uint8_t)((uint64_t)LOG_u32GetUsedBytes() * 100U / u64Cap);
}

/* -------------------------------------------------------------------------- */

void LOG_vErase(void)
{
    if (!MICROSD_bIsReady())
        return;
    LOGSD_vReset();
    bInited = true;
}

/* -------------------------------------------------------------------------- */

void LOG_vPark(void)
{
    if (!bInited)
    {
        MICROSD_vIdle();
        return;
    }

    /* Flush the partial block (and the pointers) so the latest lines persist. */
    if (u16CurLen > 0U)
    {
        if (MICROSD_bWriteBlock(u32WriteLba, au8Cur))
            LOGSD_vPersistSuper();
    }

    MICROSD_vIdle();
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
                      "\r\n==== MICROSD LOG DUMP (%lu bytes) ====\r\n",
                      (unsigned long)LOG_u32GetUsedBytes());
    if (n > 0)
        sink((const uint8_t *)hdr, (uint16_t)n);

    if (bInited)
    {
        uint32_t u32Blocks = LOGSD_u32FullBlocks();
        uint32_t u32Lba    = u32StartLba;
        for (uint32_t i = 0U; i < u32Blocks; i++)
        {
            if (MICROSD_bReadBlock(u32Lba, au8Scratch))
                sink(au8Scratch, SD_BLOCK_SIZE);
            u32Lba = LOGSD_u32Advance(u32Lba);
        }
        if (u16CurLen > 0U)
            sink(au8Cur, u16CurLen);
    }

    static const char end[] = "\r\n==== MICROSD LOG DUMP END ====\r\n";
    sink((const uint8_t *)end, (uint16_t)(sizeof(end) - 1U));
}

#endif /* STORAGE_BACKEND_MICROSD */
