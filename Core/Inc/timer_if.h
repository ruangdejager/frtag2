/*
 * timer_if.h
 *
 * Hardware timer interface for the SubGHz_Phy radio driver.
 * Provides UTIL_TimerDriver — the function-pointer table consumed by
 * Utilities/timer/stm32_timer.c (which radio.c calls via TimerInit/Start/Stop).
 *
 * Implementation: FreeRTOS one-shot software timer, HAL_GetTick() for
 * elapsed-time queries.  No RTC Alarm A conflict with hal_rtc.c.
 */

#ifndef __TIMER_IF_H__
#define __TIMER_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32_timer.h"

UTIL_TIMER_Status_t TIMER_IF_Init(void);
UTIL_TIMER_Status_t TIMER_IF_StartTimer(uint32_t timeout);
UTIL_TIMER_Status_t TIMER_IF_StopTimer(void);

uint32_t TIMER_IF_SetTimerContext(void);
uint32_t TIMER_IF_GetTimerContext(void);
uint32_t TIMER_IF_GetTimerElapsedTime(void);
uint32_t TIMER_IF_GetTimerValue(void);
uint32_t TIMER_IF_GetMinimumTimeout(void);

uint32_t TIMER_IF_Convert_ms2Tick(uint32_t timeMilliSec);
uint32_t TIMER_IF_Convert_Tick2ms(uint32_t tick);

void     TIMER_IF_DelayMs(uint32_t delay);

/* stm32_systime.c back-end (stub — systime not used in this project) */
uint32_t TIMER_IF_GetTime(uint16_t *subSeconds);
void     TIMER_IF_BkUp_Write_Seconds(uint32_t Seconds);
uint32_t TIMER_IF_BkUp_Read_Seconds(void);
void     TIMER_IF_BkUp_Write_SubSeconds(uint32_t SubSeconds);
uint32_t TIMER_IF_BkUp_Read_SubSeconds(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_IF_H__ */
