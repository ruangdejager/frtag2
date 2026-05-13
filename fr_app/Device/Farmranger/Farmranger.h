/*
 * Farmranger.h
 *
 * Farmranger device layer — UART-based AT command interface to the
 * companion logger board.
 *
 * Uses CMSIS-RTOS v2 throughout.  The HAL_UART_vTxCompleteISR callback
 * releases a semaphore (ISR-safe) to unblock the data-send path.
 */

#ifndef DEVICE_FARMRANGER_FARMRANGER_H_
#define DEVICE_FARMRANGER_FARMRANGER_H_

#include <stdbool.h>
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_bsp.h"
#include "MeshNetwork.h"

/* ---- Driver macros ---- */
#define FR_DRIVER_vInitFRDevice(drv)        HAL_UART_vSetup(drv, GPS_UART, FLOWCONTROL_NONE)
#define FR_DRIVER_vEnableUart(drv)          HAL_UART_vEnable(drv)
#define FR_DRIVER_vDisableUart(drv)         HAL_UART_vDisable(drv)
#define FR_DRIVER_vUartPutByte(drv, byte)   HAL_UART_vTxPutByte(drv, byte)
#define FR_DRIVER_vIntEnable()              HAL_GPIO_WritePin(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_SET)
#define FR_DRIVER_vIntDisable()             HAL_GPIO_WritePin(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_RESET)

/* ---- Public API ---- */
void     FARMRANGER_vInit(void);
void     FARMRANGER_vUartOnWake(void);
void     FARMRANGER_vRxTask(void *parameters);
void     FARMRANGER_vNotifyOnRX(void);         /* Called from UART ISR — do not call directly */
bool     FARMRANGER_bDeviceOn(void);
void     FARMRANGER_vDeviceOff(void);
uint64_t FARMRANGER_u64RequestTimestamp(void);
uint8_t  FARMRANGER_u8RequestInterval(void);
bool     FARMRANGER_bLogData(MeshDiscoveredNeighbor_t *neighbors, uint16_t count);

#ifdef LISTENER_MODE
void FARMRANGER_vPutString(const uint8_t *data, uint16_t len);
#endif

#endif /* DEVICE_FARMRANGER_FARMRANGER_H_ */
