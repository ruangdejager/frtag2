/*
 * hal_spi.h
 *
 * SPI driver for the accelerometer (LIS2DH or compatible) on SPI1.
 */

#ifndef INC_HAL_SPI_H_
#define INC_HAL_SPI_H_

#include <stdint.h>
#include "stm32wle5xx.h"
#include "stm32wlxx_hal.h"
#include "Acc_Config.h"

extern SPI_HandleTypeDef hAccSpi;

#define SPI_TIMEOUT 1000

void HAL_SPI_vInit(void);
void HAL_SPI_vDeInit(void);
void HAL_SPI_OnWake(void);

#define HAL_SPI_ACC_vSpiWritePacket(txData, len)       HAL_SPI_Transmit(&hAccSpi, txData, len, SPI_TIMEOUT)
#define HAL_SPI_ACC_vSpiReadPacket(rxData, len)        HAL_SPI_Receive(&hAccSpi, rxData, len, SPI_TIMEOUT)
#define HAL_SPI_ACC_vSpiReadWrite(txData, rxData, len) HAL_SPI_TransmitReceive(&hAccSpi, txData, rxData, len, SPI_TIMEOUT)
#define HAL_SPI_ACC_vSelect()                          HAL_GPIO_WritePin(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, RESET)
#define HAL_SPI_ACC_vDeselect()                        HAL_GPIO_WritePin(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, SET)

#endif /* INC_HAL_SPI_H_ */
