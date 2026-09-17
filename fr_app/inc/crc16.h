/*
 * crc16.h
 *
 *  Created on: Sep 17, 2026
 *      Author: Ruan de Jager
 *
 * CRC-16/CCITT-FALSE, used as the integrity check on the binary discovery
 * upload to the fr9 (AT+BLOG).
 *
 * This value is a WIRE CONTRACT with the fr9, which carries a byte-identical
 * copy of this file at fr_app/inc/crc16.h + fr_app/src/crc16.c. Keep the two
 * in step; they are two copies on purpose, because the alternative is a shared
 * submodule between two repositories that are flashed independently.
 *
 * Three things it deliberately is not:
 *
 * - Not LORARADIO_u8CRC8_Calculate(). That is an XOR-8 and it is a wire
 *   contract with every mesh node already deployed - LoraRadio.c says outright
 *   that changing it needs a coordinated SWD reflash of the whole fleet. It
 *   also cannot detect an even number of identical bit errors or any byte
 *   reordering, which for a stream of fixed-width records is the failure mode
 *   worth catching.
 * - Not the STM32WL CRC peripheral, even though HAL_CRC_vInit() already runs
 *   and nothing uses it. Making two different MCUs agree through two HAL
 *   configurations means matching polynomial, initial value and input/output
 *   bit reversal, and getting one of those wrong presents as "every transfer
 *   fails its checksum" with nothing to point at.
 * - Not table-driven. The payload is at most ~1.5 kB and is folded twice per
 *   upload; a 256-entry table would cost 512 bytes of flash to save time
 *   nobody is waiting on, on a part with ~1.3 kB of RAM left.
 *
 * Known answer: CRC16_u16Ccitt("123456789", 9) == 0x29B1.
 */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Poly 0x1021, init 0xFFFF, no input/output reflection, no final xor.
uint16_t CRC16_u16Ccitt(const uint8_t * pu8Data, uint16_t u16Len);

//! Incremental form, for a payload that is built or received in chunks.
//! Seed the first call with CRC16_CCITT_INIT.
#define CRC16_CCITT_INIT	UINT16_C(0xFFFF)
uint16_t CRC16_u16CcittUpdate(uint16_t u16Crc, const uint8_t * pu8Data, uint16_t u16Len);

#ifdef __cplusplus
}
#endif

#endif /* INC_CRC16_H_ */
