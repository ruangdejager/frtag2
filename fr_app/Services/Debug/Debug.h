/*
 * Debug.h
 *
 * Raw byte debug output/input service.
 * Provides transport-level output (and, for UART, input) only — no
 * formatting. All formatting lives in DbgLog.c (application layer).
 *
 * Transport selection (fr_app/inc/config/build_config.h; at most one):
 *   DEBUG_OUTPUT_UART — USART2 debug UART (default active choice)
 *   DEBUG_OUTPUT_USB  — USB CDC (not yet implemented; stub bodies provided)
 *   (neither defined) — all functions compile to stubs; no output produced
 */

#ifndef SERVICES_DEBUG_DEBUG_H_
#define SERVICES_DEBUG_DEBUG_H_

#include <stdint.h>
#include <stdbool.h>

#include "build_config.h"

void DEBUG_vInit(void);
void DEBUG_vDeInit(void);
void DEBUG_vStart(void);
void DEBUG_vStop(void);
void DEBUG_vPutByte(uint8_t byte);
void DEBUG_vPutBuffer(const uint8_t *buf, uint16_t len);
void DEBUG_vPutBufferBlocking(const uint8_t *buf, uint16_t len);

/* "Quiet" is an advisory flag, not a transport-level mute -- it does not
 * gate DEBUG_vPutBuffer itself. It exists so a raw passthrough that's
 * temporarily the sole intended thing on the wire (e.g. FrKernel's LoRa
 * bridge relaying an ext-flash log dump) can ask ambient periodic output
 * (e.g. platform.c's 1 Hz heartbeat marker) to hold off for its duration;
 * each such producer checks DEBUG_bIsQuiet() itself before writing. */
void DEBUG_vSetQuiet(bool quiet);
bool DEBUG_bIsQuiet(void);

#ifdef DEBUG_OUTPUT_UART
bool DEBUG_bRxDataAvailable(void);
bool DEBUG_bReadByte(uint8_t *pByte);
#else
static inline bool DEBUG_bRxDataAvailable(void)    { return false; }
static inline bool DEBUG_bReadByte(uint8_t *pByte) { (void)pByte; return false; }
#endif

#endif /* SERVICES_DEBUG_DEBUG_H_ */
