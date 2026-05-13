/*
 * uart_callbacks.c
 *
 * UART1 RX dispatch for frtag2.
 * USART1 is shared between GPS (secondary role) and Farmranger (primary role).
 * This file overrides the hal_uart weak callback and routes to the correct handler.
 */

#include "DeviceDiscovery.h"
#include "Farmranger.h"
#include "GPS.h"

void UART1_vNotifyOnRX(void)
{
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY)
        FARMRANGER_vNotifyOnRX();
    else
        GPS_vNotifyOnRX();
}
