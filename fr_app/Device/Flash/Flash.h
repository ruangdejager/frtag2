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

/* As FLASH_bVerifyDevice, but retried (FLASH_JEDEC_READ_ATTEMPTS) and
 * reporting evidence: pu8Id receives the last 3 ID bytes read, pu8Attempts the
 * number of tries used. Both optional (NULL to ignore). Use this where the
 * result is going to be shown to a human or acted on — a bare pass/fail off a
 * single read is not trustworthy enough to call a chip dead. */
bool    FLASH_bVerifyDeviceEx(uint8_t *pu8Id, uint8_t *pu8Attempts);

/* Why a FLASH_vRead() returned false. Callers used to get a bare bool, which
 * is not enough to diagnose anything: FOTA_u8CalcImageXorRangeBuf collapsed
 * every one of these into "the checksum is wrong", so a field log could not
 * tell a failed SPI transfer from genuinely bad stored bytes — the exact
 * ambiguity that left the 2026-08-31 "xor verify recovered on attempt 2/8"
 * events undiagnosable. */
typedef enum
{
    FLASH_RDFAIL_NONE = 0,   /* no failure recorded since the last read-out  */
    FLASH_RDFAIL_ABSENT,     /* bDevicePresent false - JEDEC ID never matched */
    FLASH_RDFAIL_WAITREADY,  /* still WIP after FLASH_WAIT_READY_TIMEOUT_MS   */
    FLASH_RDFAIL_CMD,        /* SPI transmit of the 4-byte read command failed */
    FLASH_RDFAIL_DATA,       /* SPI receive of the payload failed             */
} Flash_ReadFailReason_t;

/* Read-failure tally since the last call; reading clears it (same idiom as
 * LORARADIO_u16GetAndClearCadTimeouts — a counter rather than a log line per
 * occurrence, because FLASH_vRead is called ~1900 times per whole-image XOR
 * pass and logging inside that would perturb the timing being measured).
 *
 * The details reported belong to the FIRST failure since the last read-out,
 * not the most recent: what went wrong first is what explains the rest.
 * All three out-params are optional (NULL to ignore). pu8HalStatus carries
 * the HAL_StatusTypeDef of the failing transfer for the CMD/DATA reasons,
 * which is what distinguishes a HAL_TIMEOUT (task starved across the 1000 ms
 * SPI_TIMEOUT) from a HAL_ERROR (peripheral fault). */
uint16_t FLASH_u16GetAndClearReadFails(Flash_ReadFailReason_t *ptReason,
                                       uint32_t *pu32Addr,
                                       uint8_t  *pu8HalStatus);

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
