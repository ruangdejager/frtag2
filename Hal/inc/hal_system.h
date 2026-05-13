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

/* Weak hooks called by PreSleepProcessing / PostSleepProcessing.
 * Override in the application layer to perform role-specific peripheral
 * reinitialisation around STOP2 sleep. */
void HAL_SYSTEM_vOnPreSleep(void);
void HAL_SYSTEM_vOnPostWake(void);

#endif /* INC_HAL_SYSTEM_H_ */
