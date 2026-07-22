/*
 * hal_rtc.h
 *
 * RTC driver: 1 Hz wakeup interrupt and optional alarm callbacks.
 */

#ifndef HAL_RTC_H_
#define HAL_RTC_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32wlxx_hal.h"

typedef void (*rtc_tick_callback_t)(uint64_t);
typedef void (*rtc_hourAlarm_callback_t)(void);

extern RTC_HandleTypeDef hrtc;

void HAL_RTC_vInit(void);
uint64_t HAL_RTC_u64GetValue(void);

/* Millisecond-of-day read directly from the RTC hardware (calendar TR + SSR).
 * Survives STOP2 (LSE-clocked) and does not depend on the software second
 * counter, so it is safe to call from vPortSuppressTicksAndSleep() with the
 * RTC wakeup ISR still pending. Resolution ≈ 1/(SynchPrediv+1) s. Wraps at
 * midnight (86_400_000); callers must handle the wrap across a sleep. */
uint32_t HAL_RTC_u32GetMsOfDay(void);
void HAL_RTC_vRegisterWKUPCallback(rtc_tick_callback_t function);
void HAL_RTC_vRegisterAlarmACallback(rtc_hourAlarm_callback_t function);
void HAL_RTC_vRegisterAlarmBCallback(rtc_hourAlarm_callback_t function);
void HAL_RTC_vSetWakeupInterval(uint32_t interval);
RTC_HandleTypeDef *HAL_RTC_pGetHandle(void);
void HAL_RTC_vDisableWKUPInterrupt(void);
void HAL_RTC_vEnableWKUPInterrupt(void);
void HAL_RTC_vDeactivateAlarmA(void);
void HAL_RTC_vApplyCalibration(int16_t ppm_correction);

#endif /* HAL_RTC_H_ */
