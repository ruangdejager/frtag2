/*
 * debug_uart_output.h
 *
 * UART-based debug terminal output driver.
 */

#ifndef INC_DEBUG_UART_OUTPUT_H_
#define INC_DEBUG_UART_OUTPUT_H_

#include <stdint.h>
#include <stddef.h>

#include "hal_bsp.h"
#include "hal_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_DRIVER_vDeinitGpio() \
{ \
    HAL_GPIO_DeInit(BSP_DEBUG_UART_TX_PORT, BSP_DEBUG_UART_TX_PIN); \
    HAL_GPIO_DeInit(BSP_DEBUG_UART_RX_PORT, BSP_DEBUG_UART_RX_PIN); \
}

void DBG_UART_vInit(void);
void DBG_UART_vStart(void);
void DBG_UART_vStop(void);
void DBG_UART_vPutByte(uint8_t byte);
void TERM_vPut(const char *format, ...);

#ifdef LISTENER_MODE
void LISTENER_vPut(const char *format, ...);
#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_DEBUG_UART_OUTPUT_H_ */
