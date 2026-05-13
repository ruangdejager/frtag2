/*
 * timer_if.c
 *
 * Hardware timer interface for the SubGHz_Phy radio driver chain.
 *
 * The radio driver (radio.c) calls TimerInit / TimerSetValue / TimerStart /
 * TimerStop, which map via SubGHz_Phy/Target/timer.h to UTIL_TIMER_Create /
 * UTIL_TIMER_SetPeriod / UTIL_TIMER_Start / UTIL_TIMER_Stop in
 * Utilities/timer/stm32_timer.c.  stm32_timer.c dispatches through the
 * UTIL_TimerDriver function-pointer table defined here.
 *
 * Implementation strategy (RTOS project):
 *   - A single FreeRTOS one-shot software timer fires UTIL_TIMER_IRQ_Handler()
 *     at the next expiry computed by stm32_timer.c.
 *   - Elapsed-time queries use HAL_GetTick() (1 ms resolution, wraps ~49 days).
 *   - No RTC Alarm A usage — avoids conflict with hal_rtc.c.
 *
 * stm32_systime.c back-end functions are stubs because SysTime is not used
 * by this project.
 */

#include "timer_if.h"
#include "stm32_timer.h"
#include "stm32_systime.h"
#include "stm32wlxx_hal.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "timers.h"

#include <stddef.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Private state
 * -------------------------------------------------------------------------- */

static TimerHandle_t xRadioIfTimer  = NULL;
static uint32_t      u32TimerCtx    = 0;   /* HAL_GetTick() at SetTimerContext */
static bool          bInitialised   = false;

/* --------------------------------------------------------------------------
 * FreeRTOS timer callback — calls stm32_timer.c IRQ handler
 * -------------------------------------------------------------------------- */
static void prvRadioTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    UTIL_TIMER_IRQ_Handler();
}

/* --------------------------------------------------------------------------
 * UTIL_TimerDriver implementation
 * -------------------------------------------------------------------------- */

UTIL_TIMER_Status_t TIMER_IF_Init(void)
{
    if (!bInitialised)
    {
        xRadioIfTimer = xTimerCreate("RadioIF",
                                     pdMS_TO_TICKS(1000),   /* placeholder period */
                                     pdFALSE,               /* one-shot */
                                     NULL,
                                     prvRadioTimerCallback);
        if (xRadioIfTimer == NULL)
        {
            return UTIL_TIMER_HW_ERROR;
        }
        bInitialised = true;
    }
    return UTIL_TIMER_OK;
}

UTIL_TIMER_Status_t TIMER_IF_StartTimer(uint32_t timeout_ticks)
{
    /* stm32_timer.c passes ticks — ms2Tick returns ms directly (1:1) */
    uint32_t delay_ms = (timeout_ticks == 0u) ? 1u : timeout_ticks;

    if (xRadioIfTimer == NULL)
    {
        return UTIL_TIMER_HW_ERROR;
    }

    /* xTimerChangePeriod also starts the timer */
    if (xTimerChangePeriod(xRadioIfTimer,
                           pdMS_TO_TICKS(delay_ms),
                           0u) != pdPASS)
    {
        return UTIL_TIMER_HW_ERROR;
    }
    return UTIL_TIMER_OK;
}

UTIL_TIMER_Status_t TIMER_IF_StopTimer(void)
{
    if (xRadioIfTimer != NULL)
    {
        xTimerStop(xRadioIfTimer, 0u);
    }
    return UTIL_TIMER_OK;
}

uint32_t TIMER_IF_SetTimerContext(void)
{
    u32TimerCtx = HAL_GetTick();
    return u32TimerCtx;
}

uint32_t TIMER_IF_GetTimerContext(void)
{
    return u32TimerCtx;
}

uint32_t TIMER_IF_GetTimerElapsedTime(void)
{
    return HAL_GetTick() - u32TimerCtx;
}

uint32_t TIMER_IF_GetTimerValue(void)
{
    return HAL_GetTick();
}

uint32_t TIMER_IF_GetMinimumTimeout(void)
{
    return 1u;  /* 1 ms minimum */
}

/* 1 tick == 1 ms */
uint32_t TIMER_IF_Convert_ms2Tick(uint32_t timeMilliSec)
{
    return timeMilliSec;
}

uint32_t TIMER_IF_Convert_Tick2ms(uint32_t tick)
{
    return tick;
}

void TIMER_IF_DelayMs(uint32_t delay)
{
    HAL_Delay(delay);
}

/* --------------------------------------------------------------------------
 * UTIL_SYSTIMDriver stubs — stm32_systime.c is not called in this project
 * -------------------------------------------------------------------------- */
uint32_t TIMER_IF_GetTime(uint16_t *subSeconds)
{
    if (subSeconds != NULL) { *subSeconds = 0u; }
    return HAL_GetTick() / 1000u;
}

void TIMER_IF_BkUp_Write_Seconds(uint32_t Seconds)    { (void)Seconds; }
uint32_t TIMER_IF_BkUp_Read_Seconds(void)             { return 0u; }
void TIMER_IF_BkUp_Write_SubSeconds(uint32_t SubSec)  { (void)SubSec; }
uint32_t TIMER_IF_BkUp_Read_SubSeconds(void)          { return 0u; }

/* --------------------------------------------------------------------------
 * UTIL_TimerDriver table — consumed by stm32_timer.c
 * -------------------------------------------------------------------------- */
const UTIL_TIMER_Driver_s UTIL_TimerDriver =
{
    TIMER_IF_Init,
    NULL,                          /* DeInitTimer — not needed */

    TIMER_IF_StartTimer,
    TIMER_IF_StopTimer,

    TIMER_IF_SetTimerContext,
    TIMER_IF_GetTimerContext,

    TIMER_IF_GetTimerElapsedTime,
    TIMER_IF_GetTimerValue,
    TIMER_IF_GetMinimumTimeout,

    TIMER_IF_Convert_ms2Tick,
    TIMER_IF_Convert_Tick2ms,
};

/* --------------------------------------------------------------------------
 * UTIL_SYSTIMDriver table — consumed by stm32_systime.c (stub)
 * -------------------------------------------------------------------------- */
const UTIL_SYSTIM_Driver_s UTIL_SYSTIMDriver =
{
    TIMER_IF_BkUp_Write_Seconds,
    TIMER_IF_BkUp_Read_Seconds,
    TIMER_IF_BkUp_Write_SubSeconds,
    TIMER_IF_BkUp_Read_SubSeconds,
    TIMER_IF_GetTime,
};
