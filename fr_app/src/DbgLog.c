/*
 * DbgLog.c
 *
 * Application-level debug and log output.
 * Owns vsnprintf formatting; delegates raw byte output to:
 *   Debug service  (DBGLOG_vPutDebug → UART or LISTENER)
 *   Log service    (DBGLOG_vPutLog   → external flash circular FIFO)
 */

#include "DbgLog.h"
#include "Debug.h"
#include "Log.h"
#ifdef LISTENER_MODE
#include "Farmranger.h"
#endif

#include "stm32wlxx.h"     /* __get_IPSR() */
#include "cmsis_os2.h"
#include "FreeRTOS.h"      /* configASSERT */

#include <stdarg.h>
#include <stdio.h>

/* Separate format buffers so a debug (UART) and a log (flash) format can't
 * clobber each other, and so concurrent callers of each path are isolated. */
static char acDbgBuf[128];
static char acLogBuf[128];

/* Serializes the flash-log path: every DBG()/LOG()/DBG_LOG() from every task
 * now funnels through DBGLOG_vPutLog(), and a flash write is a long byte-by-byte
 * SPI sequence — without this two tasks would interleave SPI transactions and
 * corrupt acLogBuf. */
static osMutexId_t xLogMutex = NULL;

/* -------------------------------------------------------------------------- */

void DBGLOG_vInit(void)
{
#if defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
    DEBUG_vInit();
#endif
    xLogMutex = osMutexNew(NULL);
    configASSERT(xLogMutex != NULL);
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vPutDebug(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int len = vsnprintf(acDbgBuf, sizeof(acDbgBuf), format, ap);
    va_end(ap);
    if (len <= 0) return;

#ifdef LISTENER_MODE
    FARMRANGER_vPutString((const uint8_t *)acDbgBuf, (uint16_t)len);
#else
    DEBUG_vPutBuffer((const uint8_t *)acDbgBuf, (uint16_t)len);
#endif
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vPutLog(const char *format, ...)
{
    /* The flash write path blocks (osDelay in FLASH_vWaitReady) and takes a
     * mutex — neither is legal from an ISR, so drop ISR-context log lines.
     * Also skip until the mutex exists (DBG calls before DBGLOG_vInit). */
    if (__get_IPSR() != 0U)  return;
    if (xLogMutex == NULL)   return;

    osMutexAcquire(xLogMutex, osWaitForever);

    va_list ap;
    va_start(ap, format);
    int len = vsnprintf(acLogBuf, sizeof(acLogBuf), format, ap);
    va_end(ap);
    if (len > 0)
        LOG_vWrite(acLogBuf, (uint16_t)len);

    osMutexRelease(xLogMutex);
}
