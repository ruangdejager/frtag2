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
