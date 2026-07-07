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

/* ---- LoRa distribution (direct, non-mesh; MeshPktType 6..10) ---- */

/* Primary: true when the stored image should be distributed this wake slot
 * (metadata VALID, image version == the RUNNING version — i.e. the primary
 * already updated itself — and not yet marked distributed). */
bool OTAUPDATE_bDistributePending(void);

/* Primary: run one distribution session (announce, per-window blast + poll +
 * repair, finalize). Replaces the discovery campaign for this slot. */
void OTAUPDATE_vDistribute(void);

/* Parser hook for the OTA packet types (MeshNetwork dispatch; parser task
 * context — copies and reacts only, storage work happens on the AppTask). */
void OTAUPDATE_vOnLoraPacket(const uint8_t *pu8Buf, uint16_t u16Len);

/* Secondary: true when an OtaPrep for a newer firmware has been heard. */
bool OTAUPDATE_bPrepPending(void);

/* Secondary: chunk-receive session (AppTask). On a complete, XOR-verified
 * image this commits metadata, arms the bootloader and RESETS; otherwise
 * returns so the caller can go back to sleep. */
void OTAUPDATE_vSecondaryReceive(void);

#endif /* WORKER_OTAUPDATE_OTAUPDATE_H_ */
