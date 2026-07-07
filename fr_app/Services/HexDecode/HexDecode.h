/*
 * HexDecode.h
 *
 * Streaming decoder for the Farmranger OTA file format: a "VS,<linecount>\r\n"
 * header followed by standard Intel HEX text. Ported from the fr9's proven
 * fota.c / hex_ln_to_bin.c pair; the algorithm and address arithmetic are
 * kept identical so both platforms consume the same file format.
 *
 * Feed the raw file bytes to HEXDECODE_bOnByte(); each decoded data record
 * is emitted through the write callback at (hexAddr - OTA_APP_BASE_ADDR).
 * When the EOF record has arrived and the line count matches the header,
 * HEXDECODE_bDone() returns true and the stop address / byte counters are
 * available for the verification pass.
 *
 * Checkpoint/restore snapshots the full decoder state so a transfer block
 * that fails mid-way (lost bytes, checksum mismatch) can be re-fed from the
 * block start without restarting the whole file.
 */

#ifndef SERVICES_HEXDECODE_HEXDECODE_H_
#define SERVICES_HEXDECODE_HEXDECODE_H_

#include <stdbool.h>
#include <stdint.h>

/* Sink for decoded image bytes. u32Offset is scratch-relative
 * (= STM32 address - OTA_APP_BASE_ADDR). Return false to abort the decode. */
typedef bool (*HexDecodeWriteFn)(uint32_t u32Offset, const uint8_t *pu8Data, uint8_t u8Len);

/* Reset the decoder and set the image-byte sink. */
void HEXDECODE_vInit(HexDecodeWriteFn pfnWrite);

/* Feed one file byte. Returns false on a fatal decode error (bad HEX line,
 * image out of bounds, sink failure) — the decoder then stays in its error
 * state until re-init or restore. */
bool HEXDECODE_bOnByte(uint8_t u8Byte);

/* True once the EOF record arrived and the line count matches the header. */
bool HEXDECODE_bDone(void);

/* Last written STM32 address (incl. OTA_APP_BASE_ADDR); 0 until data seen. */
uint32_t HEXDECODE_u32StopAddr(void);

/* Total decoded data bytes (excludes gaps between segments). */
uint32_t HEXDECODE_u32DataBytes(void);

/* Snapshot / restore the full decoder state (block-retry support). */
void HEXDECODE_vCheckpoint(void);
void HEXDECODE_vRestore(void);

#endif /* SERVICES_HEXDECODE_HEXDECODE_H_ */
