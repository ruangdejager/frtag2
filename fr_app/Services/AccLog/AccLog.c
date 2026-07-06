/*
 * AccLog.c
 *
 * Accelerometer data logger to the MicroSD ACC region. See AccLog.h.
 *
 * ACC region layout (blocks [SD_ACC_LBA_START, card end)):
 *   - block SD_ACC_LBA_START          : superblock (write head + partial len)
 *   - blocks SD_ACC_LBA_START+1 ..end : linear stream of 512-byte data blocks
 *
 * Linear / append-only: when the write head reaches the end of the card the
 * store is FULL and further data is dropped (it never wraps), unlike the log.
 * Records are packed into a byte stream across block boundaries; the offline
 * parser walks the stream by magic + count.
 */

#include "storage_config.h"

#ifdef STORAGE_BACKEND_MICROSD

#include "AccLog.h"
#include "MicroSD.h"
#include "MicroSD_Config.h"
#include "dbg_log.h"

#include <stdbool.h>
#include <string.h>

#define ACCSD_SUPER_LBA     (SD_ACC_LBA_START)
#define ACCSD_DATA_FIRST    (SD_ACC_LBA_START + 1UL)
#define ACCSD_SUPER_MAGIC   0x31434341UL   /* "ACC1" */

static uint8_t  au8AccCur[SD_BLOCK_SIZE];    /* current (partial) data block     */
static uint8_t  au8AccScratch[SD_BLOCK_SIZE];/* superblock I/O                   */
static uint8_t  au8Rec[ACCLOG_REC_MAX];      /* record being assembled this tick */
static uint16_t u16RecLen;
static uint8_t  u8SampleCount;
static bool     bTickOpen;                   /* a Begin/End record is in progress */

static uint16_t u16CurLen;       /* valid bytes in au8AccCur (0..512)            */
static uint32_t u32WriteLba;     /* data block being filled                      */
static uint32_t u32EndLba;       /* one past the last usable block (card end)    */
static bool     bInited;
static bool     bFull;
static bool     bFullLogged;
static volatile bool bEraseReq;   /* set by FrKernel, executed on the movement task */

/* -------------------------------------------------------------------------- */

static void ACCSD_vPutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t ACCSD_u32GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void ACCSD_vPersistSuper(void)
{
    memset(au8AccScratch, 0, sizeof(au8AccScratch));
    ACCSD_vPutU32(&au8AccScratch[0], ACCSD_SUPER_MAGIC);
    ACCSD_vPutU32(&au8AccScratch[4], u32WriteLba);
    ACCSD_vPutU32(&au8AccScratch[8], (uint32_t)u16CurLen);
    (void)MICROSD_bWriteBlock(ACCSD_SUPER_LBA, au8AccScratch);
}

static void ACCSD_vResetHead(void)
{
    u32WriteLba = ACCSD_DATA_FIRST;
    u16CurLen   = 0U;
    bFull       = (u32WriteLba >= u32EndLba);
    bFullLogged = false;
    memset(au8AccCur, 0, sizeof(au8AccCur));
    ACCSD_vPersistSuper();
}

/* Append bytes to the block stream, flushing full blocks. Stops (sets bFull)
 * at the end of the card — never wraps. */
static void ACCSD_vAppend(const uint8_t *p, uint16_t len)
{
    uint16_t u16Off = 0U;
    while (u16Off < len)
    {
        if (bFull)
            return;

        uint16_t u16Space = (uint16_t)(SD_BLOCK_SIZE - u16CurLen);
        uint16_t u16Chunk = (uint16_t)(len - u16Off);
        if (u16Chunk > u16Space)
            u16Chunk = u16Space;

        memcpy(&au8AccCur[u16CurLen], &p[u16Off], u16Chunk);
        u16CurLen = (uint16_t)(u16CurLen + u16Chunk);
        u16Off    = (uint16_t)(u16Off + u16Chunk);

        if (u16CurLen == SD_BLOCK_SIZE)
        {
            if (!MICROSD_bWriteBlock(u32WriteLba, au8AccCur))
                return;
            u32WriteLba++;
            u16CurLen = 0U;
            if (u32WriteLba >= u32EndLba)
                bFull = true;   /* end of card — stop, do not wrap */
        }
    }
}

/* -------------------------------------------------------------------------- */

void ACCLOG_vInit(void)
{
    bInited     = false;
    u16RecLen   = 0U;
    u8SampleCount = 0U;

    if (!MICROSD_bIsReady())
        return;

    u32EndLba = MICROSD_u32BlockCount();
    if (u32EndLba <= ACCSD_DATA_FIRST)
        return;   /* no usable ACC region */

    if (MICROSD_bReadBlock(ACCSD_SUPER_LBA, au8AccScratch) &&
        (ACCSD_u32GetU32(&au8AccScratch[0]) == ACCSD_SUPER_MAGIC))
    {
        u32WriteLba = ACCSD_u32GetU32(&au8AccScratch[4]);
        u16CurLen   = (uint16_t)ACCSD_u32GetU32(&au8AccScratch[8]);

        if (u32WriteLba < ACCSD_DATA_FIRST || u32WriteLba > u32EndLba ||
            u16CurLen > SD_BLOCK_SIZE)
        {
            ACCSD_vResetHead();
        }
        else
        {
            bFull = (u32WriteLba >= u32EndLba);
            if (u16CurLen > 0U && !bFull)
            {
                if (!MICROSD_bReadBlock(u32WriteLba, au8AccCur))
                    u16CurLen = 0U;
            }
        }
    }
    else
    {
        ACCSD_vResetHead();
    }

    bInited = true;
}

/* -------------------------------------------------------------------------- */

void ACCLOG_vBeginTick(uint64_t u64Utc)
{
    u8SampleCount = 0U;
    u16RecLen     = 0U;
    bTickOpen     = false;

    if (!bInited)
        return;

    /* Service a pending 'sd clear' here, on the movement task, so the reset of
     * the ACC write head can't race this task's own acc writes. */
    if (bEraseReq)
    {
        bEraseReq = false;
        ACCSD_vResetHead();
    }

    bTickOpen = true;

    /* Reserve the header; count is back-patched in EndTick. */
    au8Rec[0] = ACCLOG_REC_MAGIC0;
    au8Rec[1] = ACCLOG_REC_MAGIC1;
    au8Rec[2] = (uint8_t)(u64Utc);
    au8Rec[3] = (uint8_t)(u64Utc >> 8);
    au8Rec[4] = (uint8_t)(u64Utc >> 16);
    au8Rec[5] = (uint8_t)(u64Utc >> 24);
    au8Rec[6] = (uint8_t)(u64Utc >> 32);
    au8Rec[7] = (uint8_t)(u64Utc >> 40);
    au8Rec[8] = (uint8_t)(u64Utc >> 48);
    au8Rec[9] = (uint8_t)(u64Utc >> 56);
    /* au8Rec[10] = count (set in EndTick) */
    u16RecLen = ACCLOG_REC_HDR_LEN;
}

void ACCLOG_vAddSample(const uint8_t *pu8Raw6)
{
    if (!bTickOpen || u8SampleCount >= ACCLOG_MAX_SAMPLES)
        return;

    memcpy(&au8Rec[u16RecLen], pu8Raw6, ACCLOG_SAMPLE_LEN);
    u16RecLen = (uint16_t)(u16RecLen + ACCLOG_SAMPLE_LEN);
    u8SampleCount++;
}

void ACCLOG_vEndTick(void)
{
    bool bWasOpen = bTickOpen;
    bTickOpen = false;

    if (!bWasOpen || u8SampleCount == 0U)
        return;

    if (bFull)
    {
        if (!bFullLogged)
        {
            DBG_LOG("AccLog: ACC region full, logging stopped\r\n");
            bFullLogged = true;
        }
        return;
    }

    au8Rec[10] = u8SampleCount;       /* back-patch count */
    ACCSD_vAppend(au8Rec, u16RecLen);

    /* Flush the partial block + pointers every tick so each second persists. */
    if (!bFull && u16CurLen > 0U)
    {
        if (MICROSD_bWriteBlock(u32WriteLba, au8AccCur))
            ACCSD_vPersistSuper();
    }
    else
    {
        ACCSD_vPersistSuper();
    }
}

/* -------------------------------------------------------------------------- */

void ACCLOG_vRequestErase(void)
{
    /* Deferred: the actual reset runs at the next tick on the movement task
     * (see ACCLOG_vBeginTick) so it can't race that task's acc writes. */
    bEraseReq = true;
}

#endif /* STORAGE_BACKEND_MICROSD */
