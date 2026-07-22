/*
 * hal_rtc.c
 *
 * RTC initialisation and 1 Hz wakeup interrupt.
 *
 * The wakeup ISR increments an internal counter and notifies the platform
 * heartbeat task using CMSIS-RTOS v2 thread flags, avoiding a direct
 * dependency on native FreeRTOS task-notification primitives.
 */

#include "hal_rtc.h"
#include "stm32wlxx.h"
#include "stm32wlxx_ll_rtc.h"
#include "hal_system.h"

#include "cmsis_os2.h"

#include "platform.h"

RTC_HandleTypeDef hrtc;

static uint64_t hal_rtc_counter = 7200-5;

/* ---- UTC persistence across resets ----
 *
 * TAMP->BKP*R are in the VBAT-backed domain and survive any reset that
 * doesn't remove backup-domain power (OTA-triggered NVIC_SystemReset,
 * watchdog, brown-out, etc). We persist the last known UTC into two
 * slots so a reset doesn't send our clock back to boot-time
 * hal_rtc_counter (~7200s → 1970-01-01 02:00) and force us to wait for
 * the next TimeSync just to know what date it is.
 *
 * Layout (STM32WLE5 TAMP has BKP0..BKP31):
 *   BKP0R/BKP1R : owned by Fota (OTA arm magic + staged version)
 *   BKP2R       : bootloader version (written by bootloader on every boot)
 *   BKP3R       : deliberately unused, see Fota_Config.h
 *   BKP4R       : UTC low 32 bits
 *   BKP5R       : UTC high 32 bits
 *
 * A magic marker isn't stored separately — the sanity check on read
 * (2020..2100 range) is enough to reject a cold-boot 0.
 *
 * TAMP writes require the backup domain to be unlocked; the RTC's
 * MspInit already enables the backup-domain clock (LSE) but the
 * DBP-enable is done here on demand, since it's cheap and self-
 * contained. */
static bool HAL_RTC_bUtcInRange(uint64_t v)
{
    return (v >= 1577836800ULL) &&    /* 2020-01-01 */
           (v <= 4102444800ULL);      /* 2100-01-01 */
}

bool HAL_RTC_bLoadPersistedUtc(uint64_t *pu64Utc)
{
    HAL_PWR_EnableBkUpAccess();
    uint32_t u32Lo = TAMP->BKP4R;
    uint32_t u32Hi = TAMP->BKP5R;
    uint64_t v = ((uint64_t)u32Hi << 32) | (uint64_t)u32Lo;
    if (!HAL_RTC_bUtcInRange(v)) return false;
    if (pu64Utc) *pu64Utc = v;
    return true;
}

void HAL_RTC_vPersistUtc(uint64_t u64Utc)
{
    /* Skip garbage — never persist a value we would refuse to restore. */
    if (!HAL_RTC_bUtcInRange(u64Utc)) return;
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP4R = (uint32_t)(u64Utc & 0xFFFFFFFFULL);
    TAMP->BKP5R = (uint32_t)(u64Utc >> 32);
}
static rtc_tick_callback_t       one_second_callback = NULL;
static rtc_hourAlarm_callback_t  one_minute_callback = NULL;
static rtc_hourAlarm_callback_t  one_hour_callback   = NULL;

/* --------------------------------------------------------------------------
 * HAL_RTC_vInit
 * -------------------------------------------------------------------------- */
void HAL_RTC_vInit(void)
{
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;
    hrtc.Init.SynchPrediv    = 255;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hrtc) != HAL_OK)
        Error_Handler();

    /* 1 Hz RTC wakeup */
    HAL_RTC_vSetWakeupInterval(1);
}

/* --------------------------------------------------------------------------
 * HAL_RTCEx_WakeUpTimerEventCallback
 * Called from RTC_WKUP_IRQHandler via HAL. Increments the counter and
 * wakes the heartbeat dispatcher using CMSIS v2 thread flags.
 * -------------------------------------------------------------------------- */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    hal_rtc_counter++;

    /* Notify the heartbeat task — osThreadFlagsSet is ISR-safe in CMSIS v2 */
    osThreadId_t task = PLATFORM_tGetHeartbeatDispatchTaskHandle();
    if (task != NULL)
        osThreadFlagsSet(task, 0x0001U);

    if (one_second_callback != NULL)
        one_second_callback(hal_rtc_counter);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    if (one_minute_callback != NULL)
        one_minute_callback();
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
    if (one_hour_callback != NULL)
        one_hour_callback();
}

void HAL_RTC_vDeactivateAlarmA(void)
{
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
}

uint64_t HAL_RTC_u64GetValue(void)
{
    return hal_rtc_counter;
}

/* --------------------------------------------------------------------------
 * HAL_RTC_u32GetMsOfDay
 * Reads the live RTC hardware calendar (hh:mm:ss) plus the sub-second
 * downcounter (SSR) and returns milliseconds since local midnight. Reading
 * SSR locks the calendar shadow registers; reading DR afterwards releases
 * the lock. Used by the tickless-idle sleep path to measure elapsed time
 * across STOP2, where SysTick is stopped and the software second counter is
 * not yet updated (its ISR is still pending).
 * -------------------------------------------------------------------------- */
uint32_t HAL_RTC_u32GetMsOfDay(void)
{
    uint32_t ssr = RTC->SSR & 0xFFFFU;
    uint32_t tr  = RTC->TR;
    (void)RTC->DR;                       /* release shadow-register lock */

    uint32_t hh = ((tr & RTC_TR_HT)  >> RTC_TR_HT_Pos)  * 10U + ((tr & RTC_TR_HU)  >> RTC_TR_HU_Pos);
    uint32_t mm = ((tr & RTC_TR_MNT) >> RTC_TR_MNT_Pos) * 10U + ((tr & RTC_TR_MNU) >> RTC_TR_MNU_Pos);
    uint32_t ss = ((tr & RTC_TR_ST)  >> RTC_TR_ST_Pos)  * 10U + ((tr & RTC_TR_SU)  >> RTC_TR_SU_Pos);

    uint32_t prediv_s = hrtc.Init.SynchPrediv;        /* 255 → 256 sub-ticks/s */
    uint32_t sub_ms   = ((prediv_s - ssr) * 1000U) / (prediv_s + 1U);

    return (((hh * 3600U) + (mm * 60U) + ss) * 1000U) + sub_ms;
}

void HAL_RTC_vRegisterWKUPCallback(rtc_tick_callback_t function)
{
    one_second_callback = function;
}

void HAL_RTC_vRegisterAlarmACallback(rtc_hourAlarm_callback_t function)
{
    one_minute_callback = function;
}

void HAL_RTC_vRegisterAlarmBCallback(rtc_hourAlarm_callback_t function)
{
    one_hour_callback = function;
}

/* --------------------------------------------------------------------------
 * RTC MSP init — clocks and NVIC
 * -------------------------------------------------------------------------- */
void HAL_RTC_MspInit(RTC_HandleTypeDef *rtcHandle)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (rtcHandle->Instance == RTC)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
            Error_Handler();

        __HAL_RCC_RTC_ENABLE();
        __HAL_RCC_RTCAPB_CLK_ENABLE();

        HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef *rtcHandle)
{
    if (rtcHandle->Instance == RTC)
    {
        __HAL_RCC_RTC_DISABLE();
        HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
        HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
    }
}

void HAL_RTC_vDisableWKUPInterrupt(void)
{
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
}

void HAL_RTC_vEnableWKUPInterrupt(void)
{
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

void HAL_RTC_vSetWakeupInterval(uint32_t interval)
{
    if (interval == 1)
        HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
    else
        HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, interval * 2048,
                                    RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);
}

RTC_HandleTypeDef *HAL_RTC_pGetHandle(void)
{
    return &hrtc;
}

void HAL_RTC_vApplyCalibration(int16_t ppm_correction)
{
    if (ppm_correction < -487) ppm_correction = -487;
    if (ppm_correction >  487) ppm_correction =  487;

    uint16_t cal_value = (uint16_t)(ppm_correction * 0.95f);
    HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_32SEC,
                             RTC_SMOOTHCALIB_PLUSPULSES_RESET, cal_value);
}
