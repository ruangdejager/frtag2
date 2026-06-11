/*
 * Flash_Driver.h
 *
 * Board abstraction layer for the AT25EU0041A NOR flash on SPI2.
 * All calls route through the HAL SPI2 macros defined in hal_spi.h.
 */

#ifndef DEVICE_FLASH_FLASH_DRIVER_H_
#define DEVICE_FLASH_FLASH_DRIVER_H_

#include "hal_spi.h"

#define FLASH_DRIVER_vSelect()              HAL_SPI_FLASH_vSelect()
#define FLASH_DRIVER_vDeselect()            HAL_SPI_FLASH_vDeselect()
#define FLASH_DRIVER_vWrite(buf, len)       HAL_SPI_FLASH_vSpiWritePacket((buf), (len))
#define FLASH_DRIVER_vRead(buf, len)        HAL_SPI_FLASH_vSpiReadPacket((buf), (len))
#define FLASH_DRIVER_vWriteRead(tx,rx,len)  HAL_SPI_FLASH_vSpiReadWrite((tx), (rx), (len))

#endif /* DEVICE_FLASH_FLASH_DRIVER_H_ */
