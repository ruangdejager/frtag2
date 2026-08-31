/*
 * hal_wdt.c
 *
 * Independent Watchdog (IWDG) driver.
 *
 * Timeout = (Reload + 1) * Prescaler / LSI, with LSI = 32 kHz (LSI_VALUE, and
 * RCC_CSR's LSIPRE /128 divider is never enabled in this project):
 *
 *   Normal operation:   (4095+1) *  64 / 32000 = 8.192 s
 *   Sleep current test: (4095+1) * 256 / 32000 = 32.768 s
 *
 * Both figures in this header used to be wrong — it claimed ~1.6 s and ~6.5 s,
 * which are the numbers for a reload of 800, not the 4095 actually programmed
 * below. Worth stating explicitly because the real budget is what decides how
 * long any single task may run without yielding to the 1 Hz heartbeat that
 * calls HAL_WDT_vReset() (see PLATFORM_vHeartbeatDispatchTask).
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
