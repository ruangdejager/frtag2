/*
 * DbgLog.c
 *
 * Application-level debug/log output with a single buffered consumer.
 *
 *   Producers (any task): DBGLOG_vPut() formats the line with vsnprintf and
 *   pushes [dest][len][payload] into a RAM ring buffer under a short mutex,
 *   then returns. The whole message is enqueued atomically; if the ring is
 *   full the message is dropped (never a partial line, never a block).
 *
 *   Consumer (one task): drains the ring and performs the slow output —
 *   UART byte-stream (Debug service) and/or external-flash text log (Log
 *   service). All transport latency lives here, off every producer's path.
 */

#include "DbgLog.h"
#include "Debug.h"
#include "Log.h"
#ifdef LISTENER_MODE
#include "Farmranger.h"
#endif

#include "stm32wlxx.h"     /* __get_IPSR() */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

/* ---- Ring buffer (power-of-two size for cheap masking) ---- */
#define DBGLOG_RING_SIZE   4096U
#define DBGLOG_RING_MASK   (DBGLOG_RING_SIZE - 1U)
#define DBGLOG_MSG_MAX     255U          /* length field is one byte */
#define DBGLOG_WAKE_FLAG   0x0001U

static uint8_t           au8Ring[DBGLOG_RING_SIZE];
static volatile uint16_t u16Head;        /* producer publishes here */
static volatile uint16_t u16Tail;        /* consumer advances here  */

static char              acFmt[DBGLOG_MSG_MAX + 1];   /* shared format scratch */
static osMutexId_t       xFmtMutex   = NULL;
static osThreadId_t      xConsumer   = NULL;
static volatile bool     bDumpRequested = false;

static void DBGLOG_vConsumerTask(void *arg);

/* -------------------------------------------------------------------------- */

void DBGLOG_vInit(void)
{
#if defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
    DEBUG_vInit();
#endif

    xFmtMutex = osMutexNew(NULL);
    configASSERT(xFmtMutex != NULL);

    static const osThreadAttr_t consumer_attr = {
        .name       = "DbgLog",
        .stack_size = configMINIMAL_STACK_SIZE * 4 * sizeof(StackType_t),
        .priority   = osPriorityLow,
    };
    xConsumer = osThreadNew(DBGLOG_vConsumerTask, NULL, &consumer_attr);
    configASSERT(xConsumer != NULL);
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vPut(uint8_t dest, const char *format, ...)
{
    /* vsnprintf + mutex are not ISR-safe; drop ISR-context lines. Also skip
     * until the consumer/mutex exist (calls before DBGLOG_vInit). */
    if (__get_IPSR() != 0U)  return;
    if (xConsumer == NULL)   return;

    osMutexAcquire(xFmtMutex, osWaitForever);

    va_list ap;
    va_start(ap, format);
    int len = vsnprintf(acFmt, sizeof(acFmt), format, ap);
    va_end(ap);

    if (len > 0)
    {
        if (len > (int)DBGLOG_MSG_MAX) len = (int)DBGLOG_MSG_MAX;

        uint16_t u16Used = (uint16_t)((u16Head - u16Tail) & DBGLOG_RING_MASK);
        uint16_t u16Free = (uint16_t)(DBGLOG_RING_SIZE - 1U - u16Used);

        /* All-or-nothing: only enqueue if the whole framed message fits. */
        if (u16Free >= (uint16_t)(2 + len))
        {
            uint16_t h = u16Head;
            au8Ring[h] = dest;              h = (uint16_t)((h + 1) & DBGLOG_RING_MASK);
            au8Ring[h] = (uint8_t)len;      h = (uint16_t)((h + 1) & DBGLOG_RING_MASK);
            for (int i = 0; i < len; i++)
            {
                au8Ring[h] = (uint8_t)acFmt[i];
                h = (uint16_t)((h + 1) & DBGLOG_RING_MASK);
            }
            u16Head = h;                   /* publish atomically */
        }
        /* else: ring full — drop this message */
    }

    osMutexRelease(xFmtMutex);

    osThreadFlagsSet(xConsumer, DBGLOG_WAKE_FLAG);
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vPutDebug(const char *format, ...)
{
    va_list ap;
    char    buf[DBGLOG_MSG_MAX + 1];
    va_start(ap, format);
    int len = vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    if (len > 0)
        DBGLOG_vPut(DBGLOG_DEST_UART, "%s", buf);
}

void DBGLOG_vPutLog(const char *format, ...)
{
    va_list ap;
    char    buf[DBGLOG_MSG_MAX + 1];
    va_start(ap, format);
    int len = vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    if (len > 0)
        DBGLOG_vPut(DBGLOG_DEST_FLASH, "%s", buf);
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vRequestDump(void)
{
    bDumpRequested = true;
    if (xConsumer != NULL)
        osThreadFlagsSet(xConsumer, DBGLOG_WAKE_FLAG);
}

/* -------------------------------------------------------------------------- */

static void DBGLOG_vConsumerTask(void *arg)
{
    (void)arg;
    uint8_t au8Msg[DBGLOG_MSG_MAX + 1];

    for (;;)
    {
        osThreadFlagsWait(DBGLOG_WAKE_FLAG, osFlagsWaitAny, osWaitForever);

        /* Stream the persisted flash log on request — done here so the consumer
         * remains the only writer of the debug transport. */
        if (bDumpRequested)
        {
            bDumpRequested = false;
            LOG_vStreamToDebug();
        }

        /* Drain every complete frame currently published. */
        while (((u16Head - u16Tail) & DBGLOG_RING_MASK) != 0U)
        {
            uint16_t t = u16Tail;
            uint8_t  dest = au8Ring[t];  t = (uint16_t)((t + 1) & DBGLOG_RING_MASK);
            uint8_t  len  = au8Ring[t];  t = (uint16_t)((t + 1) & DBGLOG_RING_MASK);

            for (uint8_t i = 0; i < len; i++)
            {
                au8Msg[i] = au8Ring[t];
                t = (uint16_t)((t + 1) & DBGLOG_RING_MASK);
            }
            u16Tail = t;                  /* release the consumed frame */

            if (dest & DBGLOG_DEST_UART)
            {
#ifdef LISTENER_MODE
                FARMRANGER_vPutString(au8Msg, len);
#else
                DEBUG_vPutBuffer(au8Msg, len);
#endif
            }
            if (dest & DBGLOG_DEST_FLASH)
            {
                LOG_vWrite((const char *)au8Msg, len);
            }
        }
    }
}
