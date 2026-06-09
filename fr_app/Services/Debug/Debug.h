/*
 * Debug.h
 *
 * Raw byte debug output service.
 * Provides transport-level output only — no formatting.
 * All formatting lives in DbgLog.c (application layer).
 *
 * Transport selection (define exactly one at project level):
 *   DEBUG_OUTPUT_UART — USART2 debug UART (default active choice)
 *   DEBUG_OUTPUT_USB  — USB CDC (not yet implemented; stub bodies provided)
 *   (neither defined) — all functions compile to stubs; no output produced
 */

#ifndef SERVICES_DEBUG_DEBUG_H_
#define SERVICES_DEBUG_DEBUG_H_

#include <stdint.h>

void DEBUG_vInit(void);
void DEBUG_vDeInit(void);
void DEBUG_vStart(void);
void DEBUG_vStop(void);
void DEBUG_vPutByte(uint8_t byte);
void DEBUG_vPutBuffer(const uint8_t *buf, uint16_t len);

#endif /* SERVICES_DEBUG_DEBUG_H_ */
