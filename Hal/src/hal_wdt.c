/*
 * hal_wdt.c
 *
 * Independent Watchdog (IWDG) driver.
 *
 * Normal operation: ~1.6 s timeout (prescaler 64, reload 4095).
 * Sleep current test: prescaler extended to 256 (~6.5 s) while current
 * measurements are taken, then restored.
 *
 * The IWDG cannot be stopped after it is started; period changes are
 * reflected by re-initialising with the new prescaler.
 */

#include "hal_wdt.h"

static IWDG_HandleTypeDef hiwdg;

void HAL_WDT_vInit(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Window    = 4095;
    hiwdg.Init.Reload    = 4095;
    HAL_IWDG_Init(&hiwdg);
    HAL_IWDG_Refresh(&hiwdg);
}

inline void HAL_WDT_vToSleepCurrentTest(void)
{
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    HAL_IWDG_Init(&hiwdg);
    HAL_IWDG_Refresh(&hiwdg);
}

inline void HAL_WDT_vReturnAfterSleepCurrentTest(void)
{
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    HAL_IWDG_Init(&hiwdg);
    HAL_IWDG_Refresh(&hiwdg);
}

void HAL_WDT_vReset(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
