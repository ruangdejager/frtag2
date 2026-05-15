/*
 * Debug.h
 *
 * Raw byte debug output/input service.
 * DEBUG_OUTPUT_UART must be explicitly defined to activate UART output.
 * All other cases (undefined or DEBUG_OUTPUT_USB) compile to stubs.
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
