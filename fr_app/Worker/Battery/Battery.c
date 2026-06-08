/*
 * Battery.c
 *
 * Battery voltage measurement — CMSIS-RTOS v2 task layer.
 *
 * Three tasks:
 *   BatSampleTask       — triggered by thread flags; performs one ADC sample.
 *   BatSchedTask        — subscribes to 1-second heartbeat; triggers sample
 *                         every 10 ticks with delta-limiting.
 *   BatPurgeTask        — one-shot: forces a single sample then fills the
 *                         entire averaging buffer with that value.
 *
 * Thread flags used on BatSampleTask:
 *   BAT_NOTIFY_DELTA_LIMIT    — take a delta-limited sample
 *   BAT_NOTIFY_PURGE_REQUEST  — take an immediate purge sample
 *
 * Thread flag used on BatPurgeTask:
 *   BAT_NOTIFY_PURGE_DONE     — sample task has finished the purge sample
 */

#include "Battery.h"
#include "hal_bsp.h"
#include "hal_system.h"
#include "Battery_Driver.h"
#include "Battery_Config.h"

#include "platform.h"
#include "dbg_log.h"
#include "math_func.h"
#include "hard_timers.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE, taskENTER/EXIT_CRITICAL */
#include "task.h"

#include <limits.h>

/* ---- Thread flags ---- */
#define BAT_NOTIFY_DELTA_LIMIT    (1UL << 0)
#define BAT_NOTIFY_PURGE_REQUEST  (1UL << 1)
#define BAT_NOTIFY_PURGE_DONE     (1UL << 2)

/* ---- Task stack / priority ---- */
#define BAT_SAMPLETASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)
#define BAT_PURGETASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)
#define BAT_SCHEDULETASK_STACK_SIZE  (configMINIMAL_STACK_SIZE)

/* ---- CMSIS-RTOS v2 handles ---- */
static osThreadId_t BAT_vSampleTask_handle;
static osThreadId_t BAT_vBufferPurgeTask_handle;
static osThreadId_t BAT_vCheckSampleScheduleTask_handle;

/* ---- Averaging buffer ---- */
uint16_t au16BatAvgBuf[BAT_AVG_FACTOR];
uint8_t  u8BatAvgIdx;
uint16_t u16BatPreviousSample;

/* ---- Forward declarations ---- */
static void BAT_vSampleTask(void *pvParameters);
static void BAT_vBufferPurgeTask(void *pvParameters);
static void BAT_vCheckSampleScheduleTask(void *pvParameters);
static bool BAT_bConvert(hal_adc_channel_t channel, uint16_t *pu16Result);

/* --------------------------------------------------------------------------
 * BAT_bConvert — one blocking ADC conversion on the given channel.
 * The ADC must already be enabled. Returns true and writes *pu16Result on
 * success; false if the end-of-conversion flag never arrived.
 * -------------------------------------------------------------------------- */
static bool BAT_bConvert(hal_adc_channel_t channel, uint16_t *pu16Result)
{
    uint8_t u8DelayMs = 3;

    BAT_DRIVER_vCleanInterrupt();
    HAL_ADC_vStartConversion(channel);

    while (!BAT_DRIVER_bGetInterruptFlag() && u8DelayMs--)
        osDelay(1);

    if (!BAT_DRIVER_bGetInterruptFlag())
        return false;

    *pu16Result = BAT_DRIVER_u16GetResult();
    return true;
}

/* --------------------------------------------------------------------------
 * BAT_vInit
 * -------------------------------------------------------------------------- */
void BAT_vInit(void)
{
    static const osThreadAttr_t sample_attr = {
        .name       = "BatSampleTask",
        .stack_size = BAT_SAMPLETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    static const osThreadAttr_t sched_attr = {
        .name       = "BatSchedTask",
        .stack_size = BAT_SCHEDULETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    BAT_vSampleTask_handle =
        osThreadNew(BAT_vSampleTask, NULL, &sample_attr);
    BAT_vCheckSampleScheduleTask_handle =
        osThreadNew(BAT_vCheckSampleScheduleTask, NULL, &sched_attr);

    configASSERT(BAT_vSampleTask_handle              != NULL);
    configASSERT(BAT_vCheckSampleScheduleTask_handle != NULL);

    /* Immediately fill the averaging buffer with a fresh sample */
    BAT_vPurgeBuffer();
}

/* --------------------------------------------------------------------------
 * BAT_vSampleTask — triggered by thread flags, performs ADC sampling
 * -------------------------------------------------------------------------- */
static void BAT_vSampleTask(void *pvParameters)
{
    (void)pvParameters;

    bool     bDeltaLimit;
    bool     bPurge;
    uint8_t  u8DelayMs;

    for (;;)
    {
        uint32_t ulNotifiedValue = osThreadFlagsWait(
            BAT_NOTIFY_DELTA_LIMIT | BAT_NOTIFY_PURGE_REQUEST,
            osFlagsWaitAny,
            osWaitForever);

        SYSTEM_vSleepLockAcquire();

        bDeltaLimit = (ulNotifiedValue & BAT_NOTIFY_DELTA_LIMIT)   != 0;
        bPurge      = (ulNotifiedValue & BAT_NOTIFY_PURGE_REQUEST) != 0;

        /* Enable bias and wait for settling */
        BAT_DRIVER_vEnableBiasCircuit();
        osDelay(40);

        /* Wait for ADC ready */
        u8DelayMs = 3;
        while (BAT_DRIVER_bIsEnabled() && u8DelayMs--)
            osDelay(1);

        if (u8DelayMs == 0)
        {
            DBG("bat: adc enable ERROR");
            BAT_DRIVER_vDisable();
        }

        BAT_DRIVER_vEnable();

        /* Read both the battery divider and VREFINT in the same enabled window.
         * VREFINT yields the true VDDA, making the battery result independent
         * of the actual 1.8 V rail voltage. */
        uint16_t u16BatRaw  = 0;
        uint16_t u16VrefRaw = 0;
        bool bConvOk = BAT_bConvert(BAT_VOLTAGE_CHANNEL, &u16BatRaw)
                    && BAT_bConvert(VREFINT_CHANNEL,     &u16VrefRaw);

        BAT_DRIVER_vDisable();
        BAT_DRIVER_vDisableBiasCircuit();

        if (bConvOk)
        {
            uint16_t u16Vdda = HAL_ADC_u16VddaFromVrefint(u16VrefRaw);

            /* Vbatt = adc_bat * VDDA / full_scale * divider_ratio.
             * Computed in one 64-bit expression to avoid intermediate
             * truncation; the divider ratio comes from BAT_DIVIDER_NUM/DEN. */
            uint16_t u16BatMv = (uint16_t)
                (((uint64_t)u16BatRaw * u16Vdda * BAT_DIVIDER_NUM)
                 / ((uint64_t)BAT_ADC_FULL_SCALE * BAT_DIVIDER_DEN));

            au16BatAvgBuf[u8BatAvgIdx] = u16BatMv;

            if (bDeltaLimit &&
                (au16BatAvgBuf[u8BatAvgIdx] < (u16BatPreviousSample - BAT_SAMPLE_VALUE_DELTA_MAX)))
            {
                au16BatAvgBuf[u8BatAvgIdx] = u16BatPreviousSample - BAT_SAMPLE_VALUE_DELTA_MAX;
            }

            u16BatPreviousSample = au16BatAvgBuf[u8BatAvgIdx];
            u8BatAvgIdx = (u8BatAvgIdx + 1) % BAT_AVG_FACTOR;
        }

        if (bPurge)
        {
            osThreadId_t purge = NULL;

            taskENTER_CRITICAL();
            purge                      = BAT_vBufferPurgeTask_handle;
            BAT_vBufferPurgeTask_handle = NULL;   /* prevent double-notify */
            taskEXIT_CRITICAL();

            if (purge != NULL)
                osThreadFlagsSet(purge, BAT_NOTIFY_PURGE_DONE);
            else
                DBG("bat: purge notify — no purge handle");
        }

        SYSTEM_vSleepLockRelease();
    }
}

/* --------------------------------------------------------------------------
 * BAT_vBufferPurgeTask — one-shot: request purge sample, fill buffer, exit
 * -------------------------------------------------------------------------- */
static void BAT_vBufferPurgeTask(void *pvParameters)
{
    (void)pvParameters;

    /* 1. Ask sample task to take a purge sample */
    osThreadFlagsSet(BAT_vSampleTask_handle, BAT_NOTIFY_PURGE_REQUEST);

    /* 2. Wait for sample task to finish */
    for (;;)
    {
        uint32_t notif = osThreadFlagsWait(BAT_NOTIFY_PURGE_DONE,
                                           osFlagsWaitAny,
                                           osWaitForever);
        if (!(notif & osFlagsError))
            break;
    }

    /* 3. Fill entire buffer with this single sample */
    for (uint8_t i = 0; i < BAT_AVG_FACTOR; i++)
        au16BatAvgBuf[i] = u16BatPreviousSample;

    u8BatAvgIdx = 0;

    taskENTER_CRITICAL();
    BAT_vBufferPurgeTask_handle = NULL;
    taskEXIT_CRITICAL();

    /* 4. Self-terminate — one-shot task */
    osThreadTerminate(osThreadGetId());
}

/* --------------------------------------------------------------------------
 * BAT_vCheckSampleScheduleTask — heartbeat-driven sample scheduler
 * -------------------------------------------------------------------------- */
static void BAT_vCheckSampleScheduleTask(void *pvParameters)
{
    (void)pvParameters;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    uint8_t u8UpdateCnt = 0;

    for (;;)
    {
        /* Wait for 1-second heartbeat notification */
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        if (u8UpdateCnt % 10 == 0)
        {
            u8UpdateCnt = 0;
            osThreadFlagsSet(BAT_vSampleTask_handle, BAT_NOTIFY_DELTA_LIMIT);
        }
        u8UpdateCnt++;
    }
}

/* --------------------------------------------------------------------------
 * BAT_vPurgeBuffer — launch a one-shot purge task
 * -------------------------------------------------------------------------- */
void BAT_vPurgeBuffer(void)
{
    static const osThreadAttr_t purge_attr = {
        .name       = "BatPurgeTask",
        .stack_size = BAT_PURGETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    osThreadId_t purgeHandle = osThreadNew(BAT_vBufferPurgeTask, NULL, &purge_attr);
    configASSERT(purgeHandle != NULL);

    taskENTER_CRITICAL();
    BAT_vBufferPurgeTask_handle = purgeHandle;
    taskEXIT_CRITICAL();
}

/* --------------------------------------------------------------------------
 * BAT_u16GetVoltage — average of last BAT_AVG_FACTOR samples, in mV
 * -------------------------------------------------------------------------- */
uint16_t BAT_u16GetVoltage(void)
{
    /* The averaging buffer already holds VREFINT-compensated millivolts
     * (computed per sample in BAT_vSampleTask), so just return the mean. */
    uint32_t u32Temp = 0;
    for (uint8_t i = 0; i < BAT_AVG_FACTOR; i++)
        u32Temp += au16BatAvgBuf[i];

    return (uint16_t)(u32Temp / (uint32_t)BAT_AVG_FACTOR);
}

/* --------------------------------------------------------------------------
 * BAT_u8ConvertVoltageToPercentage
 * -------------------------------------------------------------------------- */
uint8_t BAT_u8ConvertVoltageToPercentage(uint16_t u16Voltage)
{
    uint32_t u32Temp = max(u16Voltage, 3350);
    u32Temp -= 3350;
    u32Temp  = 100 * u32Temp;
    u32Temp  = min(u32Temp / (4100 - 3350), 100);
    return (uint8_t)u32Temp;
}

/* --------------------------------------------------------------------------
 * BAT_u8GetPercentage
 * -------------------------------------------------------------------------- */
uint8_t BAT_u8GetPercentage(void)
{
    return BAT_u8ConvertVoltageToPercentage(BAT_u16GetVoltage());
}
