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
#include "hal_uart.h"
#include "hal_rtc.h"
#include "tag_hal.h"
#include "dbg_log.h"
#include "init.h"

#include "cmsis_os2.h"
#include "DeviceDiscovery.h"

#define MS_PER_DAY   86400000U

static bool bSleepActive = false;
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

/* --------------------------------------------------------------------------
 * HAL_SYSTEM_vEnterStop2
 * Parks the core in STOP2 and restores clocks/peripherals on wake. Assumes
 * the caller already holds a critical section (interrupts masked) and has
 * decided that deep sleep is permitted. The pending wake interrupt (RTC
 * heartbeat, radio, UART, accel) fires once the caller re-enables interrupts.
 *
 * SystemClock_Config() runs here with interrupts masked; its oscillator-ready
 * polls exit on the ready flags (which set within microseconds), not on the
 * HAL tick, so the frozen SysTick during this window is not a problem.
 * -------------------------------------------------------------------------- */
void HAL_SYSTEM_vEnterStop2(void)
{
    /* Red LED off while in STOP2; back on the moment we wake. The LED is
     * therefore a direct visual indicator of the power mode: off == STOP2,
     * on == running in any other mode. */
    BSP_LED_Off(LED_RED);

    HAL_GPIO_vOnSleep();

    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    SystemClock_Config();

    /* Restore peripherals whose clocks / state are lost through STOP2.
     * Called with interrupts still masked (cpsid i in vPortSuppressTicksAndSleep)
     * so there is no race with the DbgLog consumer or any other task. */
    HAL_SPI_vInit();    /* SPI1 (ACC) + SPI2 (flash) — GPIO AF config lost in sleep */
    HAL_UART_vInit();   /* USART2 debug UART — needs re-init after APB clock gate */
    HAL_ADC_vInit();
    HAL_GPIO_OnWake();

    BSP_LED_On(LED_RED);
}

/* --------------------------------------------------------------------------
 * vPortSuppressTicksAndSleep
 * Custom tickless-idle hook overriding the weak ARM_CM3 port implementation.
 *
 * The stock implementation reprograms SysTick as the wake timer, but SysTick
 * is stopped in STOP2, so it cannot be used here. Instead:
 *
 *   DEEP path  (deep-sleep active, no sleep-lock held, init complete):
 *     stop SysTick → snapshot RTC → STOP2 → snapshot RTC on wake →
 *     advance the FreeRTOS tick by the elapsed RTC time → restart SysTick.
 *     The 1 Hz RTC heartbeat (or any peripheral IRQ) ends the sleep, so the
 *     effective sleep ceiling is one heartbeat (≤ 1 s).
 *
 *   LIGHT path (anything else — active radio/GPS/ADC windows):
 *     a plain WFI with SysTick left running, so all sub-second FreeRTOS
 *     timing (osDelay, software timers) stays exact.
 *
 * The tick rate is 1000 Hz, so 1 RTC millisecond == 1 FreeRTOS tick.
 * -------------------------------------------------------------------------- */
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    /* Enter a critical section without masking the interrupts that must still
     * be able to wake the core (cpsid i, not basepri). */
    __asm volatile("cpsid i" ::: "memory");
    __asm volatile("dsb");
    __asm volatile("isb");

    /* A task may have become ready between the idle check and here. */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep)
    {
        __asm volatile("cpsie i" ::: "memory");
        return;
    }

    bool bDeep = INIT_bIsSleepReady()
              && bSleepActive
              && (gSleepLockCount == 0U);

    if (bDeep)
    {
        /* ---- DEEP path: STOP2, reconcile tick from the RTC ---- */
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     /* stop the kernel tick */

        uint32_t u32T0 = HAL_RTC_u32GetMsOfDay();

        HAL_SYSTEM_vEnterStop2();

        uint32_t u32T1      = HAL_RTC_u32GetMsOfDay();
        uint32_t u32Elapsed = (u32T1 >= u32T0)
                            ? (u32T1 - u32T0)
                            : (MS_PER_DAY - u32T0 + u32T1);   /* midnight wrap */

        /* Never step past the next scheduled FreeRTOS event. */
        if (u32Elapsed > (uint32_t)xExpectedIdleTime)
            u32Elapsed = (uint32_t)xExpectedIdleTime;

        /* Restart the kernel tick cleanly, then book the slept time. */
        SysTick->VAL   = 0U;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

        vTaskStepTick((TickType_t)u32Elapsed);

        __asm volatile("cpsie i" ::: "memory");
    }
    else
    {
        /* ---- LIGHT path: core-gated WFI, SysTick keeps running ---- */
        __asm volatile("dsb" ::: "memory");
        __asm volatile("wfi");
        __asm volatile("isb");
        __asm volatile("cpsie i" ::: "memory");
    }
}

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
