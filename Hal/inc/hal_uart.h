/*
 * hal_uart.h
 *
 * Interrupt-driven UART driver with ring-buffer TX/RX.
 * Supports DEBUG_UART (USART1) and GPS_UART (USART2).
 */

#ifndef INC_HAL_UART_H_
#define INC_HAL_UART_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32wle5xx.h"
#include "hal_delay.h"

/* Ring-buffer sizes — must be a power of two */
#define USART_RX_BUFFER_SIZE    128
#define USART_TX_BUFFER_SIZE    128
#define USART_RX_BUFFER_MASK    (USART_RX_BUFFER_SIZE - 1)
#define USART_TX_BUFFER_MASK    (USART_TX_BUFFER_SIZE - 1)

/* RX buffer low-water mark: de-assert RTS this many bytes before full */
#define USART_RX_BUFFER_FREE_SPACE_LWM  4

typedef void (*setRtsAssert)(bool dir);
typedef bool (*getCTSPin)(void);

typedef enum {
    DEBUG_UART,
    GPS_UART,
} hal_uart_id_t;

typedef enum {
    FLOWCONTROL_NONE,
    FLOWCONTROL_FUNCTION,
} hal_uart_fc_t;

typedef uint16_t usart_buf_ptr_t;

typedef struct USART_Buffer
{
    volatile uint8_t        RX[USART_RX_BUFFER_SIZE];
    volatile uint8_t        TX[USART_TX_BUFFER_SIZE];
    volatile usart_buf_ptr_t RX_Head;
    volatile usart_buf_ptr_t RX_Tail;
    volatile usart_buf_ptr_t TX_Head;
    volatile usart_buf_ptr_t TX_Tail;
} USART_Buffer_t;

typedef struct Usart_and_buffer
{
    USART_TypeDef  *usart;
    hal_uart_id_t   uart_id;
    USART_Buffer_t  buffer;
    hal_uart_fc_t   flowcontrol;
    setRtsAssert    flowControlSetAssertRts;
    getCTSPin       flowControlGetCTS;
} hal_uart_t;

void HAL_UART_vInit(void);
/* Latch the device role (read once from the strap at boot) so HAL_UART_vInit()
 * can choose the USART1 baud/swap without re-reading PB12. Call before the
 * first HAL_UART_vInit(). */
void HAL_UART_vSetRole(bool bPrimary);
void HAL_UART_vSetup(hal_uart_t *drv, hal_uart_id_t uart_id, hal_uart_fc_t flowcontrol);
void HAL_UART_vClearBuffer(hal_uart_t *drv);
void HAL_UART_vSetFlowControlFunctions(hal_uart_t *drv, setRtsAssert setRtsFunction, getCTSPin getCtsFunction);
void HAL_UART_vEnable(hal_uart_t *drv);
void HAL_UART_vDisable(hal_uart_t *drv);
bool HAL_UART_u8TxFreeSpace(hal_uart_t *drv);
bool HAL_UART_u8TxBufferEmpty(hal_uart_t *drv);
bool HAL_UART_bTxIdle(hal_uart_t *drv);
bool HAL_UART_vTxPutByte(hal_uart_t *drv, uint8_t data);
void HAL_UART_vTxPutBuffer(hal_uart_t *drv, const uint8_t *data, uint16_t length);
void HAL_UART_vTxPutBufferBlocking(hal_uart_t *drv, const uint8_t *data, uint16_t length);
bool HAL_UART_bRxDataAvailable(hal_uart_t *drv);
bool UART_bReadByte(hal_uart_t *pHandle, uint8_t *pu8Byte);
void HAL_UART_vEnableTXInterrupt(hal_uart_t *drv);
void HAL_UART_vInterrupt(USART_TypeDef *USARTx);
void HAL_UART_vSetSpeed(hal_uart_t *drv, uint32_t new_speed);
void HAL_UART_vTxCompleteISR(hal_uart_t *drv);

#endif /* INC_HAL_UART_H_ */
