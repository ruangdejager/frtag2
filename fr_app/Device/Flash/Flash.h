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

/* Inhibit (true) or re-allow (false) deep-power-down parking. While
 * inhibited, FLASH_vDeepPowerDown() is a no-op and the device is woken
 * immediately, so the chip stays continuously awake. Used to bracket an
 * OTA session: reads issued shortly after a DPD wake were observed to
 * return corrupted bytes (the bootloader, which never parks the chip
 * mid-verify, reads the same image cleanly), so for the multi-minute OTA
 * transfer we keep the flash awake the whole time. */
void    FLASH_vInhibitDeepPowerDown(bool bInhibit);

#endif /* DEVICE_FLASH_FLASH_H_ */
