/*
 * hal_system.h
 *
 * System clock, power, and sleep management for the frtag board.
 */

#ifndef INC_HAL_SYSTEM_H_
#define INC_HAL_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

void SystemClock_Config(void);
void HAL_SYSTEM_vSleepWakeOnRtc(void);
bool SYSTEM_bCheckSleepModeStatus(void);
void SYSTEM_vSleepLockAcquire(void);
void SYSTEM_vSleepLockRelease(void);
void SYSTEM_vActivateDeepSleep(void);
void SYSTEM_vDeactivateDeepSleep(void);
bool SYSTEM_bIsDeepSleepActive(void);
void Error_Handler(void);

/* Enters STOP2 and restores clocks/peripherals on wake. Called from the
 * tickless-idle sleep path (vPortSuppressTicksAndSleep) with interrupts
 * already masked and deep-sleep eligibility already decided. */
void HAL_SYSTEM_vEnterStop2(void);

#endif /* INC_HAL_SYSTEM_H_ */
