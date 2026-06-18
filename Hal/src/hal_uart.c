/*
 * hal_uart.c
 *
 * Interrupt-driven UART driver with ring-buffer TX/RX for the frtag2 board.
 *
 * USART2 = debug UART (PA2/PA3).
 * USART1 = GPS / Farmranger UART (PB6/PB7), mutually exclusive by device role.
 *
 * Per-UART receive notifications are dispatched through weak callback stubs
 * (UART1_vNotifyOnRX / UART2_vNotifyOnRX). Override these in the application
 * layer to forward bytes to the relevant module without creating a compile-time
 * dependency here.
 */

#include "hal_uart.h"
#include "hal_bsp.h"
#include "hal_gpio.h"

#include "stm32wlxx.h"
#include "stm32wlxx_ll_usart.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_hal_rcc.h"

#include "cmsis_os2.h"

static hal_uart_t *uart1_buffer = NULL;
static hal_uart_t *uart2_buffer = NULL;

/* Device role, latched once at boot from the strap (HAL_UART_vSetRole) so the
 * USART1 baud/swap can be chosen without re-reading PB12 on every HAL_UART_vInit
 * — the strap pin is tristated right after that single read. */
static bool s_bPrimaryRole = false;

static usart_buf_ptr_t UART_u8RxBufUsedSpace(hal_uart_t *pHandle);
static usart_buf_ptr_t UART_u8RxBufFreeSpace(hal_uart_t *pHandle);

/* Weak RX-notification callbacks — override in the application. */
__attribute__((weak)) void UART1_vNotifyOnRX(void) {}
__attribute__((weak)) void UART2_vNotifyOnRX(void) {}

/* --------------------------------------------------------------------------
 * HAL_UART_vSetRole
 * Latch the device role once, after it has been read from the strap at boot.
 * Lets HAL_UART_vInit() pick the USART1 baud/swap without touching PB12 again,
 * so the strap pin can be permanently tristated after that single read.
 * -------------------------------------------------------------------------- */
void HAL_UART_vSetRole(bool bPrimary)
{
    s_bPrimaryRole = bPrimary;
}

/* --------------------------------------------------------------------------
 * HAL_UART_vInit
 * One-time peripheral initialisation (clock + LL config). Call once at boot.
 * -------------------------------------------------------------------------- */
void HAL_UART_vInit(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* Debug / Farmranger UART (USART2) */
    USART_InitStruct.BaudRate             = BSP_DEBUG_UART_BAUD;
    USART_InitStruct.DataWidth            = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits             = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity               = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection    = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl  = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling         = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(BSP_DEBUG_USART_INSTANCE, &USART_InitStruct);
    LL_USART_SetHWFlowCtrl(BSP_DEBUG_USART_INSTANCE, LL_USART_HWCONTROL_NONE);
    LL_USART_ConfigAsyncMode(BSP_DEBUG_USART_INSTANCE);
    LL_USART_Enable(BSP_DEBUG_USART_INSTANCE);
    NVIC_SetPriority(BSP_DEBUG_USART_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));

    /* GPS / Farmranger UART (USART1), mutually exclusive by device role:
     * a SECONDARY runs GNSS (9600, MAX-M10S default); a PRIMARY has no GPS and
     * uses this UART as the Farmranger link to the fr9 board, whose UART is
     * fixed at 115200. The role was latched once at boot from the strap
     * (HAL_UART_vSetRole, fed by DEVICE_DISCOVERY_vConfigDeviceRole), and the
     * strap pin (PB12) has since been tristated — so use the cached role here
     * instead of re-reading it. This and every wake-path re-init therefore
     * leave PB12 untouched.
     *
     * On a PRIMARY the pins are also TX/RX-swapped: the daughterboard wiring
     * to the fr9 Farmranger UART crosses PB6/PB7 the opposite way to the
     * GNSS module wiring, so the MCU-side SWAP bit corrects for it. */
    bool bPrimary = s_bPrimaryRole;

    USART_InitStruct.BaudRate = bPrimary ? BSP_FARMRANGER_UART_BAUD : BSP_GPS_UART_BAUD;
    LL_USART_Init(BSP_GPS_USART_INSTANCE, &USART_InitStruct);
    LL_USART_SetHWFlowCtrl(BSP_GPS_USART_INSTANCE, LL_USART_HWCONTROL_NONE);
    LL_USART_ConfigAsyncMode(BSP_GPS_USART_INSTANCE);
    LL_USART_SetTXRXSwap(BSP_GPS_USART_INSTANCE,
                          bPrimary ? LL_USART_TXRX_SWAPPED : LL_USART_TXRX_STANDARD);
    LL_USART_Enable(BSP_GPS_USART_INSTANCE);
    NVIC_SetPriority(BSP_GPS_USART_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));

    /* Leave clocks disabled until needed — re-enabled by HAL_UART_vSetup */
    __HAL_RCC_USART1_CLK_DISABLE();
    __HAL_RCC_USART2_CLK_DISABLE();
}

/* --------------------------------------------------------------------------
 * HAL_UART_vSetup — bind a hal_uart_t instance to a physical UART
 * -------------------------------------------------------------------------- */
void HAL_UART_vSetup(hal_uart_t *drv, hal_uart_id_t uart_id, hal_uart_fc_t flowcontrol)
{
    bool clkWasDisabled = false;

    drv->flowcontrol      = flowcontrol;
    drv->buffer.RX_Head   = 0;
    drv->buffer.RX_Tail   = 0;
    drv->buffer.TX_Head   = 0;
    drv->buffer.TX_Tail   = 0;

    switch (uart_id)
    {
        case DEBUG_UART:
            clkWasDisabled = (__HAL_RCC_USART2_IS_CLK_ENABLED() != 1);
            __HAL_RCC_USART2_CLK_ENABLE();
            drv->usart   = BSP_DEBUG_USART_INSTANCE;
            uart2_buffer = drv;
            HAL_NVIC_EnableIRQ(BSP_DEBUG_USART_IRQn);
            break;
        case GPS_UART:
            clkWasDisabled = (__HAL_RCC_USART1_IS_CLK_ENABLED() != 1);
            __HAL_RCC_USART1_CLK_ENABLE();
            drv->usart   = BSP_GPS_USART_INSTANCE;
            uart1_buffer = drv;
            HAL_NVIC_EnableIRQ(BSP_GPS_USART_IRQn);
            break;
        default:
            break;
    }

    drv->uart_id = uart_id;
    LL_USART_EnableIT_RXNE(drv->usart);

    if (clkWasDisabled)
    {
        if (drv->usart == USART1)
            __HAL_RCC_USART1_CLK_DISABLE();
        else if (drv->usart == USART2)
            __HAL_RCC_USART2_CLK_DISABLE();
    }
}

void HAL_UART_vSetFlowControlFunctions(hal_uart_t *drv,
                                       setRtsAssert setRtsFunction,
                                       getCTSPin    getCtsFunction)
{
    drv->flowControlGetCTS        = getCtsFunction;
    drv->flowControlSetAssertRts  = setRtsFunction;
}

void HAL_UART_vEnable(hal_uart_t *drv)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;

    switch (drv->uart_id)
    {
        case DEBUG_UART:
            __HAL_RCC_USART2_CLK_ENABLE();
            GPIO_InitStruct.Alternate = BSP_DEBUG_UART_AF;
            GPIO_InitStruct.Pin       = BSP_DEBUG_UART_TX_PIN | BSP_DEBUG_UART_RX_PIN;
            HAL_GPIO_Init(BSP_DEBUG_UART_TX_PORT, &GPIO_InitStruct);
            NVIC_EnableIRQ(BSP_DEBUG_USART_IRQn);
            break;
        case GPS_UART:
            __HAL_RCC_USART1_CLK_ENABLE();
            GPIO_InitStruct.Alternate = BSP_GPS_UART_AF;
            GPIO_InitStruct.Pin       = BSP_GPS_UART_TX_PIN | BSP_GPS_UART_RX_PIN;
            HAL_GPIO_Init(BSP_GPS_UART_TX_PORT, &GPIO_InitStruct);
            NVIC_EnableIRQ(BSP_GPS_USART_IRQn);
            break;
        default:
            break;
    }

    LL_USART_Enable(drv->usart);
    HAL_UART_vClearBuffer(drv);
}

void HAL_UART_vDisable(hal_uart_t *drv)
{
    LL_USART_Disable(drv->usart);

    switch (drv->uart_id)
    {
        case DEBUG_UART:
            HAL_GPIO_DeInit(BSP_DEBUG_UART_TX_PORT,
                            BSP_DEBUG_UART_TX_PIN | BSP_DEBUG_UART_RX_PIN);
            NVIC_DisableIRQ(BSP_DEBUG_USART_IRQn);
            __HAL_RCC_USART2_CLK_DISABLE();
            break;
        case GPS_UART:
            HAL_GPIO_DeInit(BSP_GPS_UART_TX_PORT,
                            BSP_GPS_UART_TX_PIN | BSP_GPS_UART_RX_PIN);
            NVIC_DisableIRQ(BSP_GPS_USART_IRQn);
            __HAL_RCC_USART1_CLK_DISABLE();
            break;
        default:
            break;
    }
}

void HAL_UART_vClearBuffer(hal_uart_t *drv)
{
    drv->buffer.RX_Head = 0;
    drv->buffer.RX_Tail = 0;
    drv->buffer.TX_Head = 0;
    drv->buffer.TX_Tail = 0;
}

bool HAL_UART_u8TxFreeSpace(hal_uart_t *drv)
{
    usart_buf_ptr_t tempHead = (drv->buffer.TX_Head + 1) & USART_TX_BUFFER_MASK;
    usart_buf_ptr_t tempTail = drv->buffer.TX_Tail;
    return (tempHead != tempTail);
}

bool HAL_UART_u8TxBufferEmpty(hal_uart_t *drv)
{
    return (drv->buffer.TX_Head == drv->buffer.TX_Tail);
}

bool HAL_UART_vTxPutByte(hal_uart_t *drv, uint8_t data)
{
    if (!HAL_UART_u8TxFreeSpace(drv))
        return false;

    usart_buf_ptr_t tempTX_Head = drv->buffer.TX_Head;
    drv->buffer.TX[tempTX_Head] = data;
    drv->buffer.TX_Head = (tempTX_Head + 1) & USART_TX_BUFFER_MASK;
    HAL_UART_vEnableTXInterrupt(drv);
    return true;
}

void HAL_UART_vTxPutBuffer(hal_uart_t *drv, const uint8_t *data, uint16_t length)
{
    for (uint16_t idx = 0; idx < length; idx++)
    {
        /* Push into the interrupt-driven TX ring; only yield when the ring is
         * full (waiting for the ISR to drain), not after every byte. Otherwise
         * output is throttled to ~1 KB/s, which makes a flash-log dump take
         * minutes. With this, throughput is limited by the UART baud rate. */
        while (!HAL_UART_vTxPutByte(drv, data[idx]))
            osDelay(1);
    }
}

/*
 * HAL_UART_vTxPutBufferBlocking
 *
 * Polling transmit that bypasses the TX ring buffer and its IRQ entirely —
 * safe to call with global interrupts disabled or from a fault/overflow
 * handler where the scheduler and the TX-complete ISR will never run again.
 */
void HAL_UART_vTxPutBufferBlocking(hal_uart_t *drv, const uint8_t *data, uint16_t length)
{
    for (uint16_t idx = 0; idx < length; idx++)
    {
        while (!LL_USART_IsActiveFlag_TXE(drv->usart));
        LL_USART_TransmitData8(drv->usart, data[idx]);
    }
    while (!LL_USART_IsActiveFlag_TC(drv->usart));
}

bool HAL_UART_bRxDataAvailable(hal_uart_t *usart_data)
{
    usart_buf_ptr_t tempHead = usart_data->buffer.RX_Head;
    usart_buf_ptr_t tempTail = usart_data->buffer.RX_Tail;

    if (tempHead == tempTail)
    {
        if (usart_data->flowcontrol == FLOWCONTROL_FUNCTION)
            usart_data->flowControlSetAssertRts(true);
        return false;
    }
    return true;
}

bool UART_bReadByte(hal_uart_t *pHandle, uint8_t *pu8Byte)
{
    if (pHandle == NULL)
        return false;

    if (!HAL_UART_bRxDataAvailable(pHandle))
        return false;

    *pu8Byte = pHandle->buffer.RX[pHandle->buffer.RX_Tail];
    pHandle->buffer.RX_Tail = (pHandle->buffer.RX_Tail + 1) & USART_RX_BUFFER_MASK;
    return true;
}

void HAL_UART_vEnableTXInterrupt(hal_uart_t *drv)
{
    LL_USART_EnableIT_TXE(drv->usart);
}

static usart_buf_ptr_t UART_u8RxBufUsedSpace(hal_uart_t *pHandle)
{
    usart_buf_ptr_t tempHead = pHandle->buffer.RX_Head;
    usart_buf_ptr_t tempTail = pHandle->buffer.RX_Tail;
    return ((tempHead - tempTail) & USART_RX_BUFFER_MASK);
}

static usart_buf_ptr_t UART_u8RxBufFreeSpace(hal_uart_t *pHandle)
{
    return ((USART_RX_BUFFER_SIZE - 1) - UART_u8RxBufUsedSpace(pHandle));
}

/* --------------------------------------------------------------------------
 * HAL_UART_vInterrupt — universal ISR handler, call from USARTx_IRQHandler
 * -------------------------------------------------------------------------- */
void HAL_UART_vInterrupt(USART_TypeDef *USARTx)
{
    hal_uart_t *drv = NULL;

    if      (USARTx == USART1) drv = uart1_buffer;
    else if (USARTx == USART2) drv = uart2_buffer;

    if (drv == NULL) return;

    /* --- RX --- */
    if (LL_USART_IsEnabledIT_RXNE(drv->usart) &&
        LL_USART_IsActiveFlag_RXNE(drv->usart))
    {
        uint8_t data = LL_USART_ReceiveData8(drv->usart);

        usart_buf_ptr_t tempRX_Head = (drv->buffer.RX_Head + 1) & USART_RX_BUFFER_MASK;
        if (tempRX_Head != drv->buffer.RX_Tail)
        {
            drv->buffer.RX[drv->buffer.RX_Head] = data;
            drv->buffer.RX_Head = tempRX_Head;
        }
        /* else: buffer full — drop byte */

        if (UART_u8RxBufFreeSpace(drv) <= USART_RX_BUFFER_FREE_SPACE_LWM)
        {
            if ((drv->flowcontrol == FLOWCONTROL_FUNCTION) &&
                (drv->flowControlSetAssertRts != NULL))
                drv->flowControlSetAssertRts(false);
        }

        /* Dispatch to registered application callback */
        if (USARTx == USART1)      UART1_vNotifyOnRX();
        else if (USARTx == USART2) UART2_vNotifyOnRX();
    }

    /* --- TX --- */
    if (LL_USART_IsEnabledIT_TXE(drv->usart) &&
        LL_USART_IsActiveFlag_TXE(drv->usart))
    {
        usart_buf_ptr_t tempTX_Tail = drv->buffer.TX_Tail;
        if (drv->buffer.TX_Head == tempTX_Tail)
        {
            LL_USART_DisableIT_TXE(drv->usart);
            HAL_UART_vTxCompleteISR(drv);
        }
        else
        {
            if ((drv->flowcontrol == FLOWCONTROL_FUNCTION) &&
                (drv->flowControlGetCTS != NULL) &&
                (drv->flowControlGetCTS()))
            {
                LL_USART_DisableIT_TXE(drv->usart);
            }
            else
            {
                uint8_t data = drv->buffer.TX[tempTX_Tail];
                drv->buffer.TX_Tail = (tempTX_Tail + 1) & USART_TX_BUFFER_MASK;
                LL_USART_TransmitData8(drv->usart, data);
            }
        }
    }

    /* Clear miscellaneous status flags */
    LL_USART_ClearFlag_CM(drv->usart);
    LL_USART_ClearFlag_EOB(drv->usart);
    LL_USART_ClearFlag_FE(drv->usart);
    LL_USART_ClearFlag_IDLE(drv->usart);
    LL_USART_ClearFlag_LBD(drv->usart);
    LL_USART_ClearFlag_NE(drv->usart);
    LL_USART_ClearFlag_ORE(drv->usart);
    LL_USART_ClearFlag_PE(drv->usart);
    LL_USART_ClearFlag_RTO(drv->usart);
    LL_USART_ClearFlag_TCBGT(drv->usart);
    LL_USART_ClearFlag_WKUP(drv->usart);
    LL_USART_ClearFlag_nCTS(drv->usart);
}

void HAL_UART_vSetSpeed(hal_uart_t *drv, uint32_t new_speed)
{
    if (drv->usart == USART1)
    {
        USART1->CR1 &= ~USART_CR1_UE;
        USART1->BRR  = (uint16_t)(__LL_USART_DIV_SAMPLING16(
                            LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE),
                            LL_USART_PRESCALER_DIV1, new_speed));
        USART1->CR1 |= USART_CR1_UE;
    }
    else if (drv->usart == USART2)
    {
        USART2->CR1 &= ~USART_CR1_UE;
        USART2->BRR  = (uint16_t)(__LL_USART_DIV_SAMPLING16(
                            LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE),
                            LL_USART_PRESCALER_DIV1, new_speed));
        USART2->CR1 |= USART_CR1_UE;
    }
}

__attribute__((weak))
void HAL_UART_vTxCompleteISR(hal_uart_t *drv)
{
    (void)drv;
    /* Default: do nothing */
}
