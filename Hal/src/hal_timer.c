/*
 * hal_timer.c
 *
 * TIM2 configured as a 1 ms free-running counter.
 *
 * Prescaler is derived from PCLK1 so the counter wraps every ~49 days
 * (uint32) or ~65 s (uint16).
 */

#include "hal_timer.h"
#include "stm32wlxx.h"

TIM_HandleTypeDef htim2;

static uint16_t _ms_counter   = 0;
static uint32_t _u32MsCounter = 0;

/* --------------------------------------------------------------------------
 * HAL_TIMER_vInit
 * -------------------------------------------------------------------------- */
void HAL_TIMER_vInit(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = HAL_RCC_GetPCLK1Freq() / 100000;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 100;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    HAL_TIM_Base_Start_IT(&htim2);
}

/* Called from HAL_TIM_PeriodElapsedCallback (below) when TIM2 fires. */
void HAL_TIMER_vOnPeriodElapsed(void)
{
    _ms_counter++;
    _u32MsCounter++;
}

/* --------------------------------------------------------------------------
 * HAL_TIM_PeriodElapsedCallback
 * Overrides the __weak default in stm32wlxx_hal_tim.c.
 * TIM16 is the HAL timebase — its interrupt is handled directly in
 * stm32wlxx_it.c (TIM16_IRQHandler → HAL_IncTick), so it does not reach
 * this callback.  Only TIM2 (application 1 ms counter) arrives here.
 * -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        HAL_TIMER_vOnPeriodElapsed();
}

uint16_t HAL_TIMER_u16_GetValue(void)  { return _ms_counter;    }
uint32_t HAL_TIMER_u32GetValue(void)   { return _u32MsCounter;  }

/* --------------------------------------------------------------------------
 * MSP callbacks — clock + NVIC
 * -------------------------------------------------------------------------- */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if (tim_baseHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
    else if (tim_baseHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_DISABLE();
    }
}
