/*
 * DbgLog.h
 *
 * Application-level debug and log macro layer.
 * Owns all vsnprintf formatting and routes output to the Debug and/or
 * Log services.
 *
 *   DBG(fmt, ...)      — debug output only (UART / LISTENER, gated by flags)
 *   LOG(fmt, ...)      — text log to external flash only
 *   DBG_LOG(fmt, ...)  — both
 */

#ifndef INC_DBGLOG_H_
#define INC_DBGLOG_H_

void DBGLOG_vInit(void);
void DBGLOG_vPutDebug(const char *format, ...);
void DBGLOG_vPutLog(const char *format, ...);

/* DBG     — debug transport only (UART / LISTENER), compiled out otherwise.
 * LOG     — external-flash text log only.
 * DBG_LOG — both. Use DBG_LOG at operational sites whose output should persist
 *           across power cycles; plain DBG for transient diagnostics.
 * The flash path is asynchronous (RAM ring + background drain task), so even a
 * large DBG_LOG burst returns quickly and never blocks the caller. */
#ifdef LISTENER_MODE
#  define DBG(x, ...)    DBGLOG_vPutDebug(x, ##__VA_ARGS__)
#elif defined(ENABLE_DBG_UART)
#  define DBG(x, ...)    DBGLOG_vPutDebug(x, ##__VA_ARGS__)
#else
#  define DBG(x, ...)
#endif

#define LOG(x, ...)      DBGLOG_vPutLog(x, ##__VA_ARGS__)
#define DBG_LOG(x, ...)  do { DBGLOG_vPutDebug(x, ##__VA_ARGS__); DBGLOG_vPutLog(x, ##__VA_ARGS__); } while (0)

#endif /* INC_DBGLOG_H_ */
