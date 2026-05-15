/*
 * Debug.c
 *
 * Raw byte debug output service.
 * DEBUG_OUTPUT_UART must be explicitly defined to activate UART output.
 * All other cases (undefined or DEBUG_OUTPUT_USB) compile to stubs.
 */

#include "Debug.h"

#ifdef DEBUG_OUTPUT_UART

#include "hal_uart.h"

static hal_uart_t sDriverData;

void DEBUG_vInit(void)
{
    HAL_UART_vSetup(&sDriverData, DEBUG_UART, FLOWCONTROL_NONE);
    HAL_UART_vEnable(&sDriverData);
}

void DEBUG_vDeInit(void)
{
    HAL_UART_vDisable(&sDriverData);
}

void DEBUG_vStart(void)
{
    HAL_UART_vEnable(&sDriverData);
}

void DEBUG_vStop(void)
{
    HAL_UART_vDisable(&sDriverData);
}

void DEBUG_vPutByte(uint8_t byte)
{
    while (!HAL_UART_vTxPutByte(&sDriverData, byte));
}

void DEBUG_vPutBuffer(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        DEBUG_vPutByte(buf[i]);
}

#else /* DEBUG_OUTPUT_UART not defined — stubs (covers USB and undefined cases) */

void DEBUG_vInit(void)                                  {}
void DEBUG_vDeInit(void)                                {}
void DEBUG_vStart(void)                                 {}
void DEBUG_vStop(void)                                  {}
void DEBUG_vPutByte(uint8_t byte)                       { (void)byte; }
void DEBUG_vPutBuffer(const uint8_t *buf, uint16_t len) { (void)buf; (void)len; }

#endif /* DEBUG_OUTPUT_UART */
