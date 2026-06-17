/*
 * Flash.h
 *
 * Public API for the AT25EU0041A external NOR flash device driver.
 */

#ifndef DEVICE_FLASH_FLASH_H_
#define DEVICE_FLASH_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

void    FLASH_vInit(void);
bool    FLASH_bDeviceBusy(void);
uint8_t FLASH_u8ReadStatusReg(void);
bool    FLASH_bVerifyDevice(void);          /* reads JEDEC ID; DBG-warns on mismatch */

/* All operations below wait for the device to go ready (bounded timeout)
 * before issuing their command, and return false without touching the
 * flash if the device is still busy or the SPI transaction failed -
 * never issue a command onto a busy/unresponsive device. */
bool    FLASH_vRead(uint32_t addr, uint8_t *buf, uint16_t len);
bool    FLASH_vPageWrite(uint32_t addr, const uint8_t *buf, uint16_t len);
bool    FLASH_vSectorErase(uint32_t addr);  /* erases 4 KB sector containing addr  */
bool    FLASH_vBlockErase(uint32_t addr);   /* erases 64 KB block containing addr  */
bool    FLASH_vChipErase(void);

void    FLASH_vDeepPowerDown(void);
void    FLASH_vReleaseDeepPowerDown(void);

#endif /* DEVICE_FLASH_FLASH_H_ */
