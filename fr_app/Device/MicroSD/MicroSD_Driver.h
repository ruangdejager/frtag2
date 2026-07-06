/*
 * MicroSD_Driver.h
 *
 * Board abstraction layer for the MicroSD card on SPI2. The card shares the
 * SPI2 bus and PA15 CS with the NOR-flash footprint (mutually exclusive HW),
 * so all calls route through the shared HAL SPI2 macros in hal_spi.h.
 */

#ifndef DEVICE_MICROSD_MICROSD_DRIVER_H_
#define DEVICE_MICROSD_MICROSD_DRIVER_H_

#include "hal_spi.h"

#define MICROSD_DRIVER_vSelect()             HAL_SPI_SD_vSelect()
#define MICROSD_DRIVER_vDeselect()           HAL_SPI_SD_vDeselect()
#define MICROSD_DRIVER_vWrite(buf, len)      HAL_SPI_SD_vSpiWritePacket((buf), (len))
#define MICROSD_DRIVER_vRead(buf, len)       HAL_SPI_SD_vSpiReadPacket((buf), (len))
#define MICROSD_DRIVER_vWriteRead(tx,rx,len) HAL_SPI_SD_vSpiReadWrite((tx), (rx), (len))
#define MICROSD_DRIVER_vSetSpeed(pre)        HAL_SPI_SD_vSetSpeed((pre))

#endif /* DEVICE_MICROSD_MICROSD_DRIVER_H_ */
