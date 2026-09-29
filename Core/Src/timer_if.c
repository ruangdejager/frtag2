/*
 * timer_if.c
 *
 * Hardware timer interface for the SubGHz_Phy radio driver chain and the
 * LoRaWAN MAC.
 *
 * The radio driver (radio.c) and LoRaMac call TimerInit / TimerSetValue /
 * TimerStart / TimerStop, which map via SubGHz_Phy/Target/timer.h to
 * UTIL_TIMER_Create / UTIL_TIMER_SetPeriod / UTIL_TIMER_Start /
 * UTIL_TIMER_Stop in Utilities/timer/stm32_timer.c.  stm32_timer.c dispatches
 * through the UTIL_TimerDriver function-pointer table defined here.
 *
 * Implementation strategy (RTOS project):
 *   - A single FreeRTOS one-shot software timer fires UTIL_TIMER_IRQ_Handler()
 *     at the next expiry computed by stm32_timer.c. Statically allocated: the
 *     heap has no room to spare (see RadioTestMode.c's heap notes).
 *   - Time is the FreeRTOS tick (1 ms), NOT HAL_GetTick(). HAL_GetTick() runs
 *     off TIM16, which stops in STOP2, so between two LoRaWAN uplinks it
 *     advanced only by the seconds the tag happened to be awake. LoRaMac's
 *     duty-cycle and join-backoff bookkeeping measure real elapsed time with
 *     it, and with a clock that barely moves across a 15 min sleep it would
 *     have kept refusing uplinks as "duty cycle restricted" long after the
 *     band had in fact recovered. The FreeRTOS tick is stepped across every
 *     STOP2 from the RTC (vPortSuppressTicksAndSleep in hal_system.c), so it
 *     is the one clock here that is both fine-grained and sleep-proof.
 *   - No RTC Alarm usage — avoids conflict with hal_rtc.c.
 *
 * stm32_systime.c back-end: seconds since boot from the same tick, extended to
 * 64 bits with FreeRTOS's own overflow count so it does not step backwards
 * when the 32-bit ms tick wraps (~49.7 days). The backup-register "real time"
 * offset is kept in RAM: nothing here asks the network for DeviceTime, so it
 * stays 0 and SysTimeGet() == SysTimeGetMcuTime().
 */

#include "timer_if.h"
#include "stm32_timer.h"
#include "stm32_systime.h"
#include "stm32wlxx_hal.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os2.h"

#include <stddef.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Private state
 * -------------------------------------------------------------------------- */

static StaticTimer_t tRadioIfTimerBuf;
static TimerHandle_t xRadioIfTimer  = NULL;
static uint32_t      u32TimerCtx    = 0;   /* tick at SetTimerContext */

static uint32_t      u32BkUpSeconds    = 0;
static uint32_t      u32BkUpSubSeconds = 0;

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
    if (xRadioIfTimer == NULL)
    {
        xRadioIfTimer = xTimerCreateStatic("RadioIF",
                                           pdMS_TO_TICKS(1000),   /* placeholder period */
                                           pdFALSE,               /* one-shot */
                                           NULL,
                                           prvRadioTimerCallback,
                                           &tRadioIfTimerBuf);
        if (xRadioIfTimer == NULL)
        {
            return UTIL_TIMER_HW_ERROR;
        }
    }
    return UTIL_TIMER_OK;
}

UTIL_TIMER_Status_t TIMER_IF_StartTimer(uint32_t timeout_ticks)
{
    /* stm32_timer.c passes ticks — ms2Tick returns ms directly (1:1) */
    uint32_t delay_ms = (timeout_ticks == 0u) ? 1u : timeout_ticks;
    BaseType_t xOk;

    if (xRadioIfTimer == NULL)
    {
        return UTIL_TIMER_HW_ERROR;
    }

    /* xTimerChangePeriod also starts the timer. The radio IRQ is deferred to
     * task context (radio_conf.h), so this is normally reached from a task
     * or from the timer task itself - but keep an ISR caller safe too. */
    if (__get_IPSR() != 0U)
    {
        BaseType_t xWoken = pdFALSE;
        xOk = xTimerChangePeriodFromISR(xRadioIfTimer, pdMS_TO_TICKS(delay_ms), &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
    else
    {
        xOk = xTimerChangePeriod(xRadioIfTimer, pdMS_TO_TICKS(delay_ms), 0u);
    }
    return (xOk == pdPASS) ? UTIL_TIMER_OK : UTIL_TIMER_HW_ERROR;
}

UTIL_TIMER_Status_t TIMER_IF_StopTimer(void)
{
    if (xRadioIfTimer != NULL)
    {
        if (__get_IPSR() != 0U)
        {
            BaseType_t xWoken = pdFALSE;
            (void)xTimerStopFromISR(xRadioIfTimer, &xWoken);
            portYIELD_FROM_ISR(xWoken);
        }
        else
        {
            (void)xTimerStop(xRadioIfTimer, 0u);
        }
    }
    return UTIL_TIMER_OK;
}

uint32_t TIMER_IF_SetTimerContext(void)
{
    u32TimerCtx = osKernelGetTickCount();
    return u32TimerCtx;
}

uint32_t TIMER_IF_GetTimerContext(void)
{
    return u32TimerCtx;
}

uint32_t TIMER_IF_GetTimerElapsedTime(void)
{
    return osKernelGetTickCount() - u32TimerCtx;
}

uint32_t TIMER_IF_GetTimerValue(void)
{
    return osKernelGetTickCount();
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
 * UTIL_SYSTIMDriver back-end
 * -------------------------------------------------------------------------- */
uint32_t TIMER_IF_GetTime(uint16_t *subSeconds)
{
    /* vTaskSetTimeOutState snapshots the tick AND the kernel's overflow count
     * together inside one critical section, which is exactly the pair needed
     * for a 64-bit millisecond count that cannot tear across a wrap. Task
     * context only - LoRaMac reads SysTime from its process loop, never from
     * an ISR. */
    TimeOut_t tNow;
    vTaskSetTimeOutState(&tNow);
    uint64_t u64Ms = ((uint64_t)(uint32_t)tNow.xOverflowCount << 32)
                   | (uint64_t)(uint32_t)tNow.xTimeOnEntering;

    if (subSeconds != NULL) { *subSeconds = (uint16_t)(u64Ms % 1000u); }
    return (uint32_t)(u64Ms / 1000u);
}

void TIMER_IF_BkUp_Write_Seconds(uint32_t Seconds)    { u32BkUpSeconds = Seconds; }
uint32_t TIMER_IF_BkUp_Read_Seconds(void)             { return u32BkUpSeconds; }
void TIMER_IF_BkUp_Write_SubSeconds(uint32_t SubSec)  { u32BkUpSubSeconds = SubSec; }
uint32_t TIMER_IF_BkUp_Read_SubSeconds(void)          { return u32BkUpSubSeconds; }

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
 * UTIL_SYSTIMDriver table — consumed by stm32_systime.c
 * -------------------------------------------------------------------------- */
const UTIL_SYSTIM_Driver_s UTIL_SYSTIMDriver =
{
    TIMER_IF_BkUp_Write_Seconds,
    TIMER_IF_BkUp_Read_Seconds,
    TIMER_IF_BkUp_Write_SubSeconds,
    TIMER_IF_BkUp_Read_SubSeconds,
    TIMER_IF_GetTime,
};
