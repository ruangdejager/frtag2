/*
 * Log.c
 *
 * Circular FIFO text log on external NOR flash.
 *
 * Asynchronous design:
 *   LOG_vWrite()  — producers (DBGLOG_vPutLog) push bytes into a small RAM ring
 *                   buffer. Fast and bounded: a large burst returns immediately
 *                   and, if the ring is full, excess bytes are dropped rather
 *                   than blocking the caller.
 *   LOG_vDrainTask — a low-priority background task pops bytes from the ring and
 *                   commits them to flash at flash speed (byte-by-byte page
 *                   programs with per-sector erase). The slow ~1 ms/byte flash
 *                   latency therefore never appears on a caller's path.
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

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/* ---- Flash FIFO state (owned by the drain task) ---- */
static uint32_t u32WriteAddr;
static uint32_t u32StartAddr;
static bool     bWrapped;

/* ---- RAM ring buffer (producers → drain task) ---- */
#define LOG_RING_SIZE   2048U
#define LOG_DRAIN_FLAG  0x0001U

static uint8_t           au8Ring[LOG_RING_SIZE];
static volatile uint16_t u16RingHead;   /* next write slot (producers)  */
static volatile uint16_t u16RingTail;   /* next read slot (drain task)  */
static osThreadId_t      xLogDrainTask_handle = NULL;

static void LOG_vDrainTask(void *arg);

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

    u16RingHead = 0U;
    u16RingTail = 0U;

    static const osThreadAttr_t drain_attr = {
        .name       = "LogDrain",
        .stack_size = configMINIMAL_STACK_SIZE * 2 * sizeof(StackType_t),
        .priority   = osPriorityLow,   /* drain in the background */
    };
    xLogDrainTask_handle = osThreadNew(LOG_vDrainTask, NULL, &drain_attr);
    configASSERT(xLogDrainTask_handle != NULL);
}

/* -------------------------------------------------------------------------- */

/* Commit a single byte to the flash FIFO (drain-task context only). */
static void LOG_vFlushByte(uint8_t byte)
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

    FLASH_vPageWrite(u32WriteAddr, &byte, 1U);
    u32WriteAddr++;

    if (u32WriteAddr >= LOG_FLASH_END_ADDR)
    {
        u32WriteAddr = LOG_FLASH_START_ADDR;
        bWrapped     = true;
    }
}

static void LOG_vDrainTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        osThreadFlagsWait(LOG_DRAIN_FLAG, osFlagsWaitAny, osWaitForever);

        /* Drain everything currently queued. Single consumer: only this task
         * advances u16RingTail. */
        while (u16RingTail != u16RingHead)
        {
            uint8_t byte = au8Ring[u16RingTail];
            LOG_vFlushByte(byte);
            u16RingTail = (uint16_t)((u16RingTail + 1U) % LOG_RING_SIZE);
        }
    }
}

/* -------------------------------------------------------------------------- */

/*
 * Producer: enqueue bytes into the RAM ring and wake the drain task. Bounded
 * and non-blocking — if the ring is full the remaining bytes are dropped.
 * The critical section makes this safe for multiple producer tasks.
 */
void LOG_vWrite(const char *buf, uint16_t len)
{
    if (xLogDrainTask_handle == NULL)
        return;   /* not initialised yet */

    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < len; i++)
    {
        uint16_t u16Next = (uint16_t)((u16RingHead + 1U) % LOG_RING_SIZE);
        if (u16Next == u16RingTail)
            break;                       /* ring full — drop remainder */
        au8Ring[u16RingHead] = (uint8_t)buf[i];
        u16RingHead = u16Next;
    }
    taskEXIT_CRITICAL();

    osThreadFlagsSet(xLogDrainTask_handle, LOG_DRAIN_FLAG);
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
    taskENTER_CRITICAL();
    u16RingHead = 0U;
    u16RingTail = 0U;
    taskEXIT_CRITICAL();

    FLASH_vChipErase();
    u32WriteAddr = LOG_FLASH_START_ADDR;
    u32StartAddr = LOG_FLASH_START_ADDR;
    bWrapped     = false;
}

/* -------------------------------------------------------------------------- */

void LOG_vStreamToDebug(void)
{
    uint32_t u32Remaining = LOG_u32GetUsedBytes();
    uint32_t u32Addr      = u32StartAddr;        /* FIFO tail = oldest byte */
    uint8_t  au8Chunk[64];

    while (u32Remaining > 0U)
    {
        uint16_t u16Chunk = (u32Remaining > sizeof(au8Chunk))
                          ? (uint16_t)sizeof(au8Chunk)
                          : (uint16_t)u32Remaining;

        /* Do not read across the FIFO end in a single read */
        if ((u32Addr + u16Chunk) > LOG_FLASH_END_ADDR)
            u16Chunk = (uint16_t)(LOG_FLASH_END_ADDR - u32Addr);

        FLASH_vRead(u32Addr, au8Chunk, u16Chunk);
        DEBUG_vPutBuffer(au8Chunk, u16Chunk);

        u32Addr += u16Chunk;
        if (u32Addr >= LOG_FLASH_END_ADDR)
            u32Addr = LOG_FLASH_START_ADDR;      /* wrap */

        u32Remaining -= u16Chunk;
    }
}
