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
#include "hal_rtc.h"
#include "tag_hal.h"
#include "dbg_log.h"
#include "init.h"

#include "cmsis_os2.h"
#include "DeviceDiscovery.h"

#define MS_PER_DAY   86400000U

static volatile uint32_t gSleepLockCount = 0;

/* Which LED indicates STOP2 residency (see HAL_SYSTEM_vSetSleepIndicatorLed
 * in hal_system.h). Red by default; DeviceDiscovery flips to yellow while
 * in ProductionSleep so the bench can tell "asleep in prodsleep" from
 * "asleep between normal wakes" at a glance. */
static volatile HalSystemSleepLed_e eSleepLed = HAL_SYSTEM_SLEEP_LED_RED;

void HAL_SYSTEM_vSetSleepIndicatorLed(HalSystemSleepLed_e eLed)
{
    if (eLed == eSleepLed) return;

    /* Switch cleanly: the old LED gets forced off (regardless of whether
     * we're currently sleeping) and the new LED comes on because we're
     * demonstrably out of STOP2 right now — anything else would have to
     * poll from an ISR, and a task calling this is by definition awake. */
    if (eSleepLed == HAL_SYSTEM_SLEEP_LED_RED)
    {
        BSP_LED_Off(LED_RED);
        BSP_LED_On(LED_YELLOW);
    }
    else
    {
        BSP_LED_Off(LED_YELLOW);
        BSP_LED_On(LED_RED);
    }
    eSleepLed = eLed;
}

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
    /* The "sleep indicator" LED goes off while in STOP2 and comes back on
     * the moment we wake — a direct visual indicator of the power mode:
     * off == STOP2, on == running in any other mode. The choice of which
     * LED plays that role is state (see HAL_SYSTEM_vSetSleepIndicatorLed):
     * red normally, swapped to yellow during ProductionSleep so the bench
     * can tell the two "asleep" modes apart at a glance.
     *
     * The other LED is NOT touched here (its retention across STOP2 is
     * relied on for other purposes — e.g. the Movement shake-sequence
     * confirmation flash, bounded by a short osDelay with no sleep lock
     * held; STOP2 has no minimum-idle-time floor, so it can fire mid-flash,
     * and force-clearing here would extinguish that flash almost
     * immediately after it was set). */
    if (eSleepLed == HAL_SYSTEM_SLEEP_LED_RED) BSP_LED_Off(LED_RED);
    else                                       BSP_LED_Off(LED_YELLOW);

    HAL_GPIO_vOnSleep();

    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    SystemClock_Config();

    /* Restore peripherals after STOP2.  Called with interrupts still masked
     * (cpsid i in vPortSuppressTicksAndSleep) so there is no race with any task.
     *
     * Order matters: HAL_GPIO_OnWake() must run first so that GPIOA/B/C clocks
     * are enabled before any later (lazy) HAL_SPI re-init calls HAL_GPIO_Init()
     * for the SPI pins. HAL_GPIO_vOnSleep() parks the peripheral AF pins
     * (SPI/UART) and PB12 as analog before gating the clocks; HAL_GPIO_OnWake()
     * restores the debug UART (USART2) here, while the SPI1/SPI2 pins are
     * restored later by the lazy SPI re-init (see below) and the ADC pins are
     * already analog. USART1 (GPS/Farmranger) pins stay analog until those
     * subsystems are next powered on. All other pins (driven outputs, role
     * strap) keep their retained MODER/level across STOP2.
     *
     * USART2 needs no explicit re-init: STOP2 retains both the APB1ENR clock-
     * enable bit and the full USART2 register state (BRR, CR1, CR2, CR3).
     * SystemClock_Config() above restores the APB bus, so USART2 is already live
     * with its pre-sleep baud-rate and interrupt settings.  Calling HAL_UART_vInit()
     * here would be actively harmful — it ends with __HAL_RCC_USART2_CLK_DISABLE()
     * (boot-time design: clock off until vSetup/vEnable) which silences the TX path.
     *
     * SPI and ADC are NOT re-initialised here. Re-running their full init (the
     * ADC's includes a calibration) on every 1 Hz heartbeat wake was pure waste:
     * the ADC is sampled only every ~10 s and the flash bus only when something
     * logs. Instead we just flag both as parked; each is restored lazily on its
     * first use after the wake (HAL_ADC_vLock / the SPI chip-select chokepoints),
     * so an idle wake that touches neither pays nothing. The GPIO clocks are
     * re-enabled by HAL_GPIO_OnWake() above, so a later lazy re-init can restore
     * the parked SPI pins. */
    HAL_GPIO_OnWake();
    HAL_SPI_vMarkParked();   /* SPI1 (ACC) + SPI2 (flash): restore on first select */
    HAL_ADC_vMarkParked();   /* ADC: restore + recalibrate on first HAL_ADC_vLock  */

    if (eSleepLed == HAL_SYSTEM_SLEEP_LED_RED) BSP_LED_On(LED_RED);
    else                                       BSP_LED_On(LED_YELLOW);
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

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}
