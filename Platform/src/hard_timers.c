/*
 * hard_timers.c
 *
 * Lightweight software timer abstraction backed by either the TIM2
 * millisecond counter or the 1 Hz RTC tick counter.
 *
 * Timer expiry uses half-the-range comparison to correctly handle
 * wrap-around of the underlying 16-bit tick counter.
 */

#include "hard_timers.h"
#include "platform_rtc.h"
#include "hal_timer.h"

static uint16_t _timer_get_ticks(clockidentifier_t clockid)
{
    switch (clockid)
    {
        case TIMER_MILLI:    return (uint16_t)HAL_TIMER_u16_GetValue();
        case TIMER_RTC_TICK: return (uint16_t)RTC_u64GetTicks();
        default:             return 0;
    }
}

void TIMERS_vTimerCreate(TIMERS_timer_t *timer, clockidentifier_t clockid)
{
    timer->clockid = clockid;
    timer->state   = TIMER_STOP;
}

void TIMERS_vTimerStart(TIMERS_timer_t *timer, tmr_tick_t period)
{
    timer->state      = TIMER_START;
    timer->expireTick = _timer_get_ticks(timer->clockid) + period;
}

bool TIMERS_bTimerIsExpired(TIMERS_timer_t *timer)
{
    if (timer->state == TIMER_STOP)   return false;
    if (timer->state == TIMER_EXPIRE) return true;

    tmr_tick_t tTickCounter = _timer_get_ticks(timer->clockid);

    if (tTickCounter >= timer->expireTick)
    {
        if (((tTickCounter - timer->expireTick) & (1 << (SYSCLK_TICK_SIZE_BITS - 1))) != 0)
            return false;
    }
    else
    {
        if (((timer->expireTick - tTickCounter) & (1 << (SYSCLK_TICK_SIZE_BITS - 1))) == 0)
            return false;
    }

    timer->state = TIMER_EXPIRE;
    return true;
}

void TIMERS_vTimerStop(TIMERS_timer_t *timer)
{
    timer->state = TIMER_STOP;
}
