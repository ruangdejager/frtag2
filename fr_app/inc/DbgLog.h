/*
 * DbgLog.h
 *
 * Application-level debug and log macro layer.
 *
 * All three macros format with vsnprintf and push the result into a single RAM
 * ring buffer; one background "DbgLog" consumer task drains the ring and does
 * the slow output (UART byte-stream and/or external-flash write). Producers
 * therefore never block on the slow transports and concurrent callers from
 * different tasks cannot corrupt or lose each other's lines (each message is
 * enqueued atomically; only on a full ring is a whole message dropped).
 *
 *   DBG(fmt, ...)      — debug transport only (UART / LISTENER, gated by flags)
 *   LOG(fmt, ...)      — external-flash text log only
 *   DBG_LOG(fmt, ...)  — both
 */

#ifndef INC_DBGLOG_H_
#define INC_DBGLOG_H_

#include <stdint.h>

/* Output destinations (bitmask) */
#define DBGLOG_DEST_UART   (1U << 0)
#define DBGLOG_DEST_FLASH  (1U << 1)
#define DBGLOG_DEST_BOTH   (DBGLOG_DEST_UART | DBGLOG_DEST_FLASH)

void DBGLOG_vInit(void);

/* Format and enqueue one line for the given destination(s). Non-blocking. */
void DBGLOG_vPut(uint8_t dest, const char *format, ...);

/* Ask the consumer task to stream the whole external-flash log over the debug
 * transport. Done from the consumer so it stays the single UART writer (no
 * interleaving with live log output). */
void DBGLOG_vRequestDump(void);

/* Compatibility wrappers (UART-only / flash-only). */
void DBGLOG_vPutDebug(const char *format, ...);
void DBGLOG_vPutLog(const char *format, ...);

#ifdef LISTENER_MODE
#  define DBG(x, ...)    DBGLOG_vPut(DBGLOG_DEST_UART, x, ##__VA_ARGS__)
#elif defined(ENABLE_DBG_UART)
#  define DBG(x, ...)    DBGLOG_vPut(DBGLOG_DEST_UART, x, ##__VA_ARGS__)
#else
#  define DBG(x, ...)
#endif

#define LOG(x, ...)      DBGLOG_vPut(DBGLOG_DEST_FLASH, x, ##__VA_ARGS__)
#define DBG_LOG(x, ...)  DBGLOG_vPut(DBGLOG_DEST_BOTH,  x, ##__VA_ARGS__)

#endif /* INC_DBGLOG_H_ */
