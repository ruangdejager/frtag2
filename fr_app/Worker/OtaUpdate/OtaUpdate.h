/*
 * OtaUpdate.h
 *
 * OTA update orchestration. Runs entirely on the DeviceDiscovery AppTask —
 * each entry point is a sequential phase of a wake slot, no task of its own.
 *
 * UART acquire (primary): pull the firmware file from the fr9 logger over
 * UART (block-by-block, verified), decode into the ext-flash scratchpad,
 * validate, then arm the bootloader and reset. Called at the end of a normal
 * wake slot while the logger session is still up.
 */

#ifndef WORKER_OTAUPDATE_OTAUPDATE_H_
#define WORKER_OTAUPDATE_OTAUPDATE_H_

#include <stdbool.h>
#include <stdint.h>

/* Primary: pull the fw file from the logger if it offers a newer version.
 * On a fully acquired + validated image this arms the bootloader and RESETS
 * (never returns); it returns false on "nothing to fetch" and on failure
 * (failure is reported to the logger with AT+FWDONE=ERR). */
bool OTAUPDATE_bUartAcquire(void);

#endif /* WORKER_OTAUPDATE_OTAUPDATE_H_ */
