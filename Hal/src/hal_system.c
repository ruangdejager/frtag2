/*
 * hal_system.c
 *
 * System clock configuration and STOP2 sleep management.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "hal_system.h"
#include "hal_spi.h"
#include "hal_adc.h"
#include "tag_hal.h"
#include "dbg_log.h"
#include "init.h"

#include "cmsis_os2.h"
#include "DeviceDiscovery.h"

static bool bSleepActive = true;
static volatile uint32_t gSleepLockCount = 0;

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Set regulator to scale 1 for 48 MHz operation */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /* HSI + LSE oscillators, PLL → 48 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN            = 12;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                     | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_SYSTEM_vSleepWakeOnRtc(void)
{
    HAL_GPIO_vOnSleep();

    HAL_SuspendTick();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    HAL_ResumeTick();

    SystemClock_Config();
    HAL_GPIO_OnWake();
}

/* PreSleepProcessing / PostSleepProcessing are defined in
 * Core/Src/app_freertos.c (the CubeIDE FreeRTOS hook location).
 * They call HAL_SYSTEM_vOnPreSleep() / HAL_SYSTEM_vOnPostWake() below.
 */

/* HAL_SYSTEM_vOnPreSleep — called by PreSleepProcessing in app_freertos.c.
 * Enters STOP2 and restores clocks/peripherals on wake. */
void HAL_SYSTEM_vOnPreSleep(void)
{
    if (!INIT_bIsSleepReady()) return;

    if (bSleepActive)
    {
        BSP_LED_Off(LED_RED);
        HAL_GPIO_vOnSleep();
        HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
        SystemClock_Config();

        /* Re-init SPI for secondary devices with accelerometer after STOP2 wake.
         * SPI clocks are stopped in STOP2 and must be reconfigured. */
#ifdef ENABLE_MOVE
        if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
            HAL_SPI_vInit();
#endif

        HAL_ADC_vInit();
        HAL_GPIO_OnWake();
        BSP_LED_On(LED_RED);
    }
}

/* HAL_SYSTEM_vOnPostWake — called by PostSleepProcessing in app_freertos.c. */
__attribute__((weak)) void HAL_SYSTEM_vOnPostWake(void) {}

bool SYSTEM_bCheckSleepModeStatus(void)
{
    return (gSleepLockCount == 0);
}

void SYSTEM_vSleepLockAcquire(void)
{
    taskENTER_CRITICAL();
    gSleepLockCount++;
    taskEXIT_CRITICAL();
}

void SYSTEM_vSleepLockRelease(void)
{
    taskENTER_CRITICAL();
    if (gSleepLockCount > 0)
        gSleepLockCount--;
    taskEXIT_CRITICAL();
}

void SYSTEM_vActivateDeepSleep(void)
{
    bSleepActive = true;
}

void SYSTEM_vDeactivateDeepSleep(void)
{
    bSleepActive = false;
}

bool SYSTEM_bIsDeepSleepActive(void)
{
    return bSleepActive;
}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}
