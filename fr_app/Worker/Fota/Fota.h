/*
 * Fota.h
 *
 * OTA firmware update, end to end. Combines what were previously two
 * modules (Services/OtaStore for external-NOR storage + Worker/OtaUpdate
 * for orchestration) into a single worker.
 *
 * Responsibilities:
 *   - external-NOR scratchpad: erase / write / read image bytes
 *   - streamed XOR-8 verify
 *   - metadata record: commit / read / mark distributed
 *   - bootloader handoff (TAMP backup regs) and reset
 *   - UART acquire (primary pulls the image from the fr9 logger)
 *   - LoRa distribution session (primary broadcasts to secondaries)
 *   - LoRa receive session (secondary), with a firmware-acceptance gate
 *
 * All storage and orchestration entry points run on the DeviceDiscovery
 * AppTask; the LoRa parser hook FOTA_vOnLoraPacket runs on the parser
 * task context and only copies/reacts.
 *
 * Flash layout and the metadata / handoff contract live in Fota_Config.h.
 */

#ifndef WORKER_FOTA_FOTA_H_
#define WORKER_FOTA_FOTA_H_

#include <stdbool.h>
#include <stdint.h>

/* Decoded metadata record (see Fota_Config.h for the on-flash layout). */
typedef struct {
    uint32_t u32Version;        /* MMmmpp                                    */
    uint32_t u32StopAddr;       /* last written STM32 address (incl. base)   */
    uint32_t u32SizeBytes;      /* stopAddr - base + 1                       */
    uint8_t  u8Xor8;
    bool     bValid;            /* VALID marker present                      */
    bool     bDistributed;      /* primary already ran the LoRa session      */
} FotaMeta_t;

/* --------------------------------------------------------------------------
 * Init / storage layer
 * -------------------------------------------------------------------------- */

/* One-time layout migration (erases a metadata sector still holding stale
 * text-log data from the pre-OTA partition layout). Also latches this
 * build's version into TAMP->BKP3R so the bootloader knows what version
 * is currently installed. Call once at boot after FLASH_vInit(). */
void FOTA_vInit(void);

/* Erase the whole image scratchpad (~3 s; yields between sectors). */
bool FOTA_bEraseScratch(void);

/* Write image bytes at a scratch offset (= STM32 address - OTA_APP_BASE_ADDR). */
bool FOTA_bWriteImage(uint32_t u32Offset, const uint8_t *pu8Data, uint16_t u16Len);

/* Read image bytes from a scratch offset. */
bool FOTA_bReadImage(uint32_t u32Offset, uint8_t *pu8Buf, uint16_t u16Len);

/* Streamed 8-bit XOR over scratch bytes 0..u32SizeBytes-1. */
uint8_t FOTA_u8CalcImageXor(uint32_t u32SizeBytes);

/* Streamed 8-bit XOR over scratch bytes [u32Start, u32Start+u32Len). */
uint8_t FOTA_u8CalcImageXorRange(uint32_t u32Start, uint32_t u32Len);

/* Write the metadata record and VALID marker (the commit point). */
bool FOTA_bCommitMetadata(uint32_t u32Version, uint32_t u32StopAddr, uint8_t u8Xor8);

/* Read + validate the metadata record. Returns true when magic and VALID
 * are both present. */
bool FOTA_bGetMeta(FotaMeta_t *ptMeta);

/* Mark the stored image as distributed (primary, after the LoRa session). */
bool FOTA_bMarkDistributed(void);

/* Arm the bootloader handoff (TAMP backup registers) and reset. Never
 * returns. */
void FOTA_vArmBootloaderAndReset(uint32_t u32Version);

/* --------------------------------------------------------------------------
 * UART acquire (primary)
 * -------------------------------------------------------------------------- */

/* Primary: pull the fw file from the logger if it offers a newer version.
 * On a fully acquired + validated image this arms the bootloader and RESETS
 * (never returns); returns false on "nothing to fetch" or on failure
 * (failure is reported to the logger with AT+FWDONE=ERR). */
bool FOTA_bUartAcquire(void);

/* --------------------------------------------------------------------------
 * LoRa distribution (primary)
 * -------------------------------------------------------------------------- */

/* Primary: true when the stored image should be distributed this wake slot
 * (metadata VALID, image version == the running version, not yet
 * distributed). */
bool FOTA_bDistributePending(void);

/* Primary: run one distribution session (announce, per-window blast + poll
 * + repair, finalize). Replaces the discovery campaign for this slot. */
void FOTA_vDistribute(void);

/* On-demand distribution ("tag <ID> fwdistribute"): re-send whatever VALID
 * image is staged, one-shot. */
bool FOTA_bRequestDistribute(void);
bool FOTA_bDistributeRequested(void);
void FOTA_vClearDistributeRequest(void);

/* Parser hook for the OTA packet types (MeshNetwork dispatch; parser task
 * context — copies and reacts only). */
void FOTA_vOnLoraPacket(const uint8_t *pu8Buf, uint16_t u16Len);

/* --------------------------------------------------------------------------
 * LoRa receive (secondary)
 * -------------------------------------------------------------------------- */

/* Secondary: true when an OtaPrep for a newer firmware has been heard. */
bool FOTA_bPrepPending(void);

/* Secondary: chunk-receive session (AppTask). On a complete, XOR-verified
 * image this commits metadata, arms the bootloader and RESETS; otherwise
 * returns so the caller can go back to sleep. */
void FOTA_vSecondaryReceive(void);

/* Secondary firmware-acceptance gate: unarmed devices ignore every OtaPrep.
 * Armed over a live FrKernel session ("tag <ID> fwaccept"); cleared by
 * "tag <ID> fwaccept off", by a completed/attempted receive, or by reset. */
void FOTA_vArmAcceptance(void);
void FOTA_vDisarmAcceptance(void);
bool FOTA_bAcceptanceArmed(void);

#endif /* WORKER_FOTA_FOTA_H_ */
