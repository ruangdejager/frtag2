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
 * Two arm sources:
 *   - FOTA_vArmAcceptance(): the discovery TimeSync auto-arm. Persists across
 *     wakes so a secondary catches up on a later campaign. NOT honoured inside
 *     a kernel wakeup (would let a stray OtaPrep during a log-download session
 *     start a pointless receive).
 *   - FOTA_vArmAcceptanceKernel(): explicit "tag <ID> fwaccept" — the only
 *     source that enables the OTA-receive rendezvous inside a live FrKernel
 *     session.
 * Both cleared by "tag <ID> fwaccept off", by a completed/attempted receive,
 * or by reset. FOTA_bAcceptanceArmedViaKernel() is true only for the latter. */
void FOTA_vArmAcceptance(void);
void FOTA_vArmAcceptanceKernel(void);
void FOTA_vDisarmAcceptance(void);
bool FOTA_bAcceptanceArmed(void);
bool FOTA_bAcceptanceArmedViaKernel(void);

/* Discard an OtaPrep that was latched in a previous campaign and never
 * serviced. Call at campaign start: such a Prep can no longer be acted on,
 * and while it lingers the session-id latch refuses every new OtaPrep — which
 * is how a unit can silently stop accepting firmware indefinitely. No-op
 * during a live receive. */
void FOTA_vDropStalePrep(void);

/* False once a PRE-SEND verify of the staged image has failed and not yet
 * been cleared by a good one — i.e. we hold an image we could not confirm we
 * can actually deliver. The TimeSync advertisement is gated on this so a
 * primary in that state stops telling the fleet an update exists; otherwise
 * every secondary arms for firmware that never arrives. The image itself is
 * kept (see OTA_PRESEND_FAIL_ERASE_THRESHOLD) — this is only about what we
 * advertise. */
/* Does the staged image's own fw_info record agree with the version it is being
 * staged, distributed or installed under? Every other version the tag sees is
 * an assertion BY something (an fr9 manifest, an AT+FWREQ answer, a LoRa Prep
 * header); this is the only one that travels inside the payload and so cannot
 * be wrong while the payload is right.
 *
 * Returns false if the versions differ, if the scratch read fails, or if the
 * record is blank — all of which mean "identity not established", which callers
 * must treat as a rejection. pu32Actual (optional) receives the version the
 * image declares, or 0 when it could not be read, purely so the caller can log
 * what it actually found. */
bool FOTA_bStagedImageIsVersion(uint32_t u32Claimed, uint32_t *pu32Actual);

bool FOTA_bStagedImageTrusted(void);

/* --------------------------------------------------------------------------
 * Session priority
 * -------------------------------------------------------------------------- */

/* True while a distribute (primary) or receive (secondary) session is
 * actively running. MeshNetwork's dispatch drops every non-OTA packet type
 * while this is true, so ordinary mesh traffic (beacons, DReq/DAck,
 * TimeSync, FrKernel) can't add radio/CPU contention during a transfer —
 * see FOTA_bSessionActive() call site in MeshNetwork.c. */
bool FOTA_bSessionActive(void);

#endif /* WORKER_FOTA_FOTA_H_ */
