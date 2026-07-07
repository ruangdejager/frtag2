/*
 * OtaStore.h
 *
 * OTA image store on the external NOR flash — scratchpad writes, streamed
 * XOR-8 verification, metadata record management, and the bootloader boot
 * flag. Layout and handoff contract in OtaStore_Config.h.
 */

#ifndef SERVICES_OTASTORE_OTASTORE_H_
#define SERVICES_OTASTORE_OTASTORE_H_

#include <stdbool.h>
#include <stdint.h>

/* Decoded metadata record (see OtaStore_Config.h for the on-flash layout). */
typedef struct {
    uint32_t u32Version;        /* MMmmpp                                    */
    uint32_t u32StopAddr;       /* last written STM32 address (incl. base)   */
    uint32_t u32SizeBytes;      /* stopAddr - base + 1                       */
    uint8_t  u8Xor8;
    bool     bValid;            /* VALID marker present                      */
    bool     bConsumed;         /* bootloader already flashed this image     */
    bool     bDistributed;      /* primary already ran the LoRa session      */
} OtaMeta_t;

/* One-time layout migration: erases a metadata sector still holding stale
 * text-log data from the pre-OTA partition layout. Call once at boot after
 * FLASH_vInit(). */
void OTASTORE_vInit(void);

/* Erase the whole image scratchpad (59 sectors, ~3 s; yields between
 * sectors). Call BEFORE a transfer so no mid-stream erase can stall RX. */
bool OTASTORE_bEraseScratch(void);

/* Write image bytes at a scratch offset (= STM32 address - OTA_APP_BASE_ADDR).
 * Splits across flash page boundaries; rejects writes beyond the scratchpad. */
bool OTASTORE_bWriteImage(uint32_t u32Offset, const uint8_t *pu8Data, uint16_t u16Len);

/* Read image bytes from a scratch offset. */
bool OTASTORE_bReadImage(uint32_t u32Offset, uint8_t *pu8Buf, uint16_t u16Len);

/* Streamed 8-bit XOR over scratch bytes 0..u32SizeBytes-1. */
uint8_t OTASTORE_u8CalcImageXor(uint32_t u32SizeBytes);

/* Write the metadata record and the VALID marker (caller has verified the
 * image; this is the commit point). */
bool OTASTORE_bCommitMetadata(uint32_t u32Version, uint32_t u32StopAddr, uint8_t u8Xor8);

/* Read + validate the metadata record. Returns true when magic and VALID
 * are present; fills *ptMeta either way it can. */
bool OTASTORE_bGetMeta(OtaMeta_t *ptMeta);

/* Mark the stored image as distributed (primary, after the LoRa session). */
bool OTASTORE_bMarkDistributed(void);

/* Arm the bootloader handoff (TAMP backup registers) and reset. Never
 * returns. This is the scope boundary until the bootloader exists. */
void OTASTORE_vArmBootloaderAndReset(uint32_t u32Version);

#endif /* SERVICES_OTASTORE_OTASTORE_H_ */
