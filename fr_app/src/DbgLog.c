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

#include <stdarg.h>
#include <stdio.h>

static char acBuf[128];

/* -------------------------------------------------------------------------- */

void DBGLOG_vInit(void)
{
#if defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
    DEBUG_vInit();
#endif
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vPutDebug(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int len = vsnprintf(acBuf, sizeof(acBuf), format, ap);
    va_end(ap);
    if (len <= 0) return;

#ifdef LISTENER_MODE
    FARMRANGER_vPutString((const uint8_t *)acBuf, (uint16_t)len);
#else
    DEBUG_vPutBuffer((const uint8_t *)acBuf, (uint16_t)len);
#endif
}

/* -------------------------------------------------------------------------- */

void DBGLOG_vPutLog(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int len = vsnprintf(acBuf, sizeof(acBuf), format, ap);
    va_end(ap);
    if (len > 0)
        LOG_vWrite(acBuf, (uint16_t)len);
}
