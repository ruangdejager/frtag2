/*
 * stm32wlxx_it.c
 *
 * Interrupt Service Routines for frtag2.
 */

#include "main.h"
#include "stm32wlxx_it.h"
#include "hal_uart.h"
#include "hal_adc.h"
#include "hal_bsp.h"

extern ADC_HandleTypeDef       hadc;
extern RTC_HandleTypeDef       hrtc;
extern SUBGHZ_HandleTypeDef    hsubghz;
extern TIM_HandleTypeDef       htim2;
extern TIM_HandleTypeDef       htim16;

void NMI_Handler(void)
{
    while (1) { }
}

void HardFault_Handler(void)
{
    volatile uint32_t cfsr     = SCB->CFSR;
    volatile uint32_t hfsr     = SCB->HFSR;
    volatile uint32_t mmfar    = SCB->MMFAR;
    volatile uint32_t bfar     = SCB->BFAR;
    volatile uint32_t flash_sr = FLASH->SR;
    volatile uint32_t flash_cr = FLASH->CR;

    (void)cfsr; (void)hfsr; (void)mmfar; (void)bfar;
    (void)flash_sr; (void)flash_cr;

    __BKPT(0);
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    volatile uint32_t cfsr     = SCB->CFSR;
    volatile uint32_t hfsr     = SCB->HFSR;
    volatile uint32_t bfar     = SCB->BFAR;
    volatile uint32_t flash_sr = FLASH->SR;
    volatile uint32_t flash_cr = FLASH->CR;

    (void)cfsr; (void)hfsr; (void)bfar;
    (void)flash_sr; (void)flash_cr;

    __BKPT(0);
    while (1) { }
}

void UsageFault_Handler(void)
{
    while (1) { }
}

void DebugMon_Handler(void) { }

void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void RTC_Alarm_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(&hrtc);
}

void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim16);
}

/** USART1 — GPS / Farmranger UART (mutually exclusive by device role) */
void USART1_IRQHandler(void)
{
    HAL_UART_vInterrupt(USART1);
}

/** USART2 — Debug UART */
void USART2_IRQHandler(void)
{
    HAL_UART_vInterrupt(USART2);
}

void SUBGHZ_Radio_IRQHandler(void)
{
    HAL_SUBGHZ_IRQHandler(&hsubghz);
}
