/*
 * Debug.h
 *
 * Raw byte debug output/input service.
 * Provides transport-level output (and, for UART, input) only — no
 * formatting. All formatting lives in DbgLog.c (application layer).
 *
 * Transport selection (define exactly one at project level):
 *   DEBUG_OUTPUT_UART — USART2 debug UART (default active choice)
 *   DEBUG_OUTPUT_USB  — USB CDC (not yet implemented; stub bodies provided)
 *   (neither defined) — all functions compile to stubs; no output produced
 */

#ifndef SERVICES_DEBUG_DEBUG_H_
#define SERVICES_DEBUG_DEBUG_H_

#include <stdint.h>
#include <stdbool.h>

void DEBUG_vInit(void);
void DEBUG_vDeInit(void);
void DEBUG_vStart(void);
void DEBUG_vStop(void);
void DEBUG_vPutByte(uint8_t byte);
void DEBUG_vPutBuffer(const uint8_t *buf, uint16_t len);

#ifdef DEBUG_OUTPUT_UART
bool DEBUG_bRxDataAvailable(void);
bool DEBUG_bReadByte(uint8_t *pByte);
#else
static inline bool DEBUG_bRxDataAvailable(void)    { return false; }
static inline bool DEBUG_bReadByte(uint8_t *pByte) { (void)pByte; return false; }
#endif

#endif /* SERVICES_DEBUG_DEBUG_H_ */
