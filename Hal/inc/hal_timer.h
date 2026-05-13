/*
 * hal_timer.h
 *
 * TIM2 1 ms tick counter.
 */

#ifndef HAL_TIMER_H_
#define HAL_TIMER_H_

#include <stdint.h>
#include "stm32wlxx.h"

extern TIM_HandleTypeDef htim2;

void     HAL_TIMER_vInit(void);
uint16_t HAL_TIMER_u16_GetValue(void);
uint32_t HAL_TIMER_u32GetValue(void);
void     HAL_TIMER_vOnPeriodElapsed(void);  /* called from HAL_TIM_PeriodElapsedCallback in hal_timer.c */

#endif /* HAL_TIMER_H_ */
