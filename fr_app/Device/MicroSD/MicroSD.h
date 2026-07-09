/*
 * MicroSD.h
 *
 * Public API for the MicroSD card (SPI mode, raw 512-byte block access).
 *
 * Mutually exclusive with the NOR flash (build_config.h). Only compiled
 * into an active translation unit when STORAGE_BACKEND_MICROSD is selected.
 */

#ifndef DEVICE_MICROSD_MICROSD_H_
#define DEVICE_MICROSD_MICROSD_H_

#include <stdint.h>
#include <stdbool.h>

/* Bring up the card: power-up clocks, CMD0, CMD8, ACMD41, CMD58 (capacity
 * class) and CMD16. Returns true once the card is ready for block I/O. */
bool     MICROSD_vInit(void);

/* True if a previous MICROSD_vInit() succeeded and the card is usable. */
bool     MICROSD_bIsReady(void);

/* Read / write one 512-byte logical block. buf must be SD_BLOCK_SIZE bytes.
 * Return false on timeout / transfer error without disturbing the card. */
bool     MICROSD_bReadBlock(uint32_t u32Lba, uint8_t *pu8Buf);
bool     MICROSD_bWriteBlock(uint32_t u32Lba, const uint8_t *pu8Buf);

/* Total card capacity in 512-byte blocks (from the CSD), or 0 if unknown. */
uint32_t MICROSD_u32BlockCount(void);

/* Park the bus (deselect + idle clocks). Parallels FLASH_vDeepPowerDown so the
 * log consumer can release the device before the system idles. */
void     MICROSD_vIdle(void);

#endif /* DEVICE_MICROSD_MICROSD_H_ */
