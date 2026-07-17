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
#include <stdbool.h>

/* Log partition: the upper 272 KB of the 512 KB device. The lower region
 * (0x00000-0x3BFFF) belongs to the OTA image scratchpad + metadata — see
 * fr_app/Worker/Fota/Fota_Config.h for the full partition map. */
#define LOG_FLASH_START_ADDR    0x03C000UL
#define LOG_FLASH_END_ADDR      0x080000UL
#define LOG_FLASH_SIZE_BYTES    (LOG_FLASH_END_ADDR - LOG_FLASH_START_ADDR)

void     LOG_vInit(void);
void     LOG_vWrite(const char *buf, uint16_t len);

/* Suspend (true) or resume (false) persisting log text to the NOR flash.
 * While suspended, LOG_vWrite() is a no-op — the DbgLog consumer still
 * streams every line to the live UART, but nothing is programmed into the
 * flash log partition. Used to bracket an OTA session: interleaving log
 * writes to this shared chip between the OTA image reads corrupts those
 * reads (the bootloader, which is the sole flash user during its verify,
 * reads the same image cleanly), so we stop touching the flash from the
 * logger for the multi-minute transfer. */
void     LOG_vSuspend(bool bSuspend);
uint32_t LOG_u32GetUsedBytes(void);
uint8_t  LOG_u8GetUsedPercent(void);

/* Erase the log PARTITION only (per-sector, LOG_FLASH_START_ADDR..END_ADDR).
 * Leaves the OTA image scratchpad + metadata (below LOG_FLASH_START_ADDR)
 * untouched. */
void     LOG_vErase(void);

/* Erase the WHOLE NOR device — log partition AND the OTA image scratchpad +
 * metadata below it (a single chip-erase command, faster than per-sector
 * for the full 512 KB). Destroys any staged-but-not-yet-installed OTA
 * image along with the log; only intended for a deliberate full field-unit
 * reset ("tag flash eraseall"), not routine log maintenance. */
void     LOG_vEraseAll(void);

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
