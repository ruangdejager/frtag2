/*
 * hal_delay.c
 *
 * Busy-wait delays.
 *   HAL_vDelayUs — SysTick-based microsecond delay.
 *   HAL_vDelayMs — HAL tick-based millisecond delay.
 *
 * Neither function should be used from ISR context or when the scheduler
 * is running and a preemptive delay is acceptable; use osDelay() instead.
 */

#include "hal_delay.h"
#include "stm32wlxx.h"

void HAL_vDelayUs(volatile uint32_t count)
{
    uint32_t clk_cycle_start = SysTick->VAL;
    count *= (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((SysTick->VAL - clk_cycle_start) < count);
}

void HAL_vDelayMs(uint32_t count)
{
    uint32_t target = HAL_GetTick() + count;
    while (HAL_GetTick() <= target);
}
