/*
 * crc16.c
 *
 *  Created on: Sep 17, 2026
 *      Author: Ruan de Jager
 *
 * See crc16.h for why this is plain C and not the CRC peripheral, and why it
 * is not LORARADIO_u8CRC8_Calculate.
 *
 * Bitwise, not table-driven. The payload it covers is at most ~1.5 kB and is
 * folded once per transfer, so the 256-entry table would cost 512 bytes of
 * flash to save a few hundred microseconds nobody is waiting on.
 */

#include "crc16.h"

#include <stddef.h>

uint16_t CRC16_u16CcittUpdate(uint16_t u16Crc, const uint8_t * pu8Data, uint16_t u16Len)
{
	uint16_t i;
	uint8_t  u8Bit;

	if (pu8Data == NULL) return u16Crc;

	for (i = 0; i < u16Len; i++)
	{
		u16Crc ^= (uint16_t)((uint16_t)pu8Data[i] << 8);

		for (u8Bit = 0; u8Bit < 8; u8Bit++)
		{
			if (u16Crc & UINT16_C(0x8000))
			{
				u16Crc = (uint16_t)((u16Crc << 1) ^ UINT16_C(0x1021));
			}
			else
			{
				u16Crc = (uint16_t)(u16Crc << 1);
			}
		}
	}

	return u16Crc;
}

uint16_t CRC16_u16Ccitt(const uint8_t * pu8Data, uint16_t u16Len)
{
	return CRC16_u16CcittUpdate(CRC16_CCITT_INIT, pu8Data, u16Len);
}
