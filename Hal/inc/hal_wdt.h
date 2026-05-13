/*
 * hal_wdt.h
 *
 * Independent Watchdog (IWDG) driver.
 */

#ifndef INC_HAL_WDT_H_
#define INC_HAL_WDT_H_

#include "stm32wlxx.h"

void HAL_WDT_vInit(void);
void HAL_WDT_vToSleepCurrentTest(void);
void HAL_WDT_vReturnAfterSleepCurrentTest(void);
void HAL_WDT_vReset(void);

#endif /* INC_HAL_WDT_H_ */
