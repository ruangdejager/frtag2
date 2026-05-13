/*
 * debug_uart_output.c
 *
 * Debug terminal output via USART1.
 *
 * TERM_vPut()     — printf-style output over the debug UART.
 * LISTENER_vPut() — available when LISTENER_MODE is defined; forwards
 *                   output through the Farmranger UART channel.
 */

#include <stdarg.h>
#include "debug_uart_output.h"
#include "platform.h"
#include "stdio.h"
#include "string.h"
#include "flashLog.h"
#ifdef LISTENER_MODE
#include "Farmranger.h"
#endif

static hal_uart_t sDriverData;
static char acString[128];

void DBG_UART_vInit(void)
{
    HAL_UART_vSetup(&sDriverData, DEBUG_UART, FLOWCONTROL_NONE);
    HAL_UART_vEnable(&sDriverData);
}

inline void DBG_UART_vStart(void) { HAL_UART_vEnable(&sDriverData);  }
inline void DBG_UART_vStop(void)  { HAL_UART_vDisable(&sDriverData); }

void DBG_UART_vPutByte(uint8_t byte)
{
    HAL_UART_vTxPutByte(&sDriverData, byte);
}

void TERM_vPut(const char *format, ...)
{
    va_list  ap;
    int16_t  i16BytesWritten;
    uint8_t  u8Len  = 96;
    char    *pacStr = acString;

    va_start(ap, format);
    i16BytesWritten = vsnprintf(pacStr, u8Len, format, ap);
    va_end(ap);

    if (i16BytesWritten < 0)
        return;

    i16BytesWritten = min(i16BytesWritten, u8Len);

    for (int i = 0; i < (int)strlen(acString); i++)
        while (!HAL_UART_vTxPutByte(&sDriverData, acString[i]));
}

#ifdef LISTENER_MODE
void LISTENER_vPut(const char *format, ...)
{
    va_list ap;
    char    buf[128];
    va_start(ap, format);
    int len = vsnprintf(buf, sizeof(buf) - 1, format, ap);
    va_end(ap);
    if (len > 0)
        FARMRANGER_vPutString((const uint8_t *)buf, (uint16_t)len);
}
#endif
