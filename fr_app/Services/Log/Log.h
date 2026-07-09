/*
 * Log.h
 *
 * Circular FIFO text log on external NOR flash (AT25EU0041A, 512 KB).
 *
 * FIFO semantics: u32WriteAddr is the head, u32StartAddr is the tail.
 * When the log wraps and a sector is erased, the tail advances past the
 * erased sector so the oldest data is transparently replaced.
 */

#ifndef SERVICES_LOG_LOG_H_
#define SERVICES_LOG_LOG_H_

#include <stdint.h>

#define LOG_FLASH_START_ADDR    0x000000UL
#define LOG_FLASH_END_ADDR      0x080000UL   /* 512 KB */

void     LOG_vInit(void);
void     LOG_vWrite(const char *buf, uint16_t len);
uint32_t LOG_u32GetUsedBytes(void);
uint8_t  LOG_u8GetUsedPercent(void);
void     LOG_vErase(void);

/* Release the backing storage device for the idle/STOP2 period that follows
 * (NOR flash -> deep-power-down; MicroSD -> flush any partial block + deselect).
 * Called by the DbgLog consumer as its last act before it blocks. */
void     LOG_vPark(void);

/* Stream the entire log FIFO (oldest → newest) out the debug transport via
 * DEBUG_vPutBuffer. Intended as a test/readback hook; call when no task is
 * concurrently writing the log (e.g. at boot right after LOG_vInit). */
void     LOG_vStreamToDebug(void);

/* Same content and chunking as LOG_vStreamToDebug, but each chunk is handed
 * to the given sink instead of hardcoding DEBUG_vPutBuffer -- e.g. FrKernel's
 * LoRa interface redirects "tag flash stream" over the radio instead of the
 * debug UART, since the requester has no UART session to read it from. */
void     LOG_vStreamViaSink(void (*sink)(const uint8_t *data, uint16_t len));

#endif /* SERVICES_LOG_LOG_H_ */
