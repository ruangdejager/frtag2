/*
 * hard_timers.h
 *
 * Lightweight software timer abstraction.
 * Timers can be backed by either the millisecond hardware counter (TIM2)
 * or the 1 Hz RTC tick counter.
 *
 * Maximum period for TIMER_MILLI: 32 s  (2^16 / 2 ms)
 */

#ifndef INC_HARD_TIMERS_H_
#define INC_HARD_TIMERS_H_

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t clockidentifier_t;
#define TIMER_MILLI     1
#define TIMER_RTC_TICK  2

#define TIMER_MILLI_MAX_TIME  32   /* seconds */

typedef uint16_t tmr_tick_t;
#define SYSCLK_TICK_SIZE_BITS  (sizeof(tmr_tick_t) * 8)

typedef enum {
    TIMER_STOP   = 0,
    TIMER_START,
    TIMER_EXPIRE,
} timer_state;

typedef struct {
    tmr_tick_t          expireTick;
    clockidentifier_t   clockid;
    timer_state         state;
} TIMERS_timer_t;

void TIMERS_vTimerCreate(TIMERS_timer_t *timer, clockidentifier_t clockid);
void TIMERS_vTimerStart(TIMERS_timer_t *timer, tmr_tick_t period);
bool TIMERS_bTimerIsExpired(TIMERS_timer_t *timer);
void TIMERS_vTimerStop(TIMERS_timer_t *timer);

#endif /* INC_HARD_TIMERS_H_ */
