/*
 * SolarPower.c
 *
 * Solar power measurement — CMSIS-RTOS v2 task layer.
 *
 * Two tasks:
 *   SOLAR_vSampleTask            — triggered by thread flag; performs one
 *                                  VSOLAR + RSENSE ADC sample and accumulates
 *                                  coulombs.
 *   SOLAR_vCheckSampleScheduleTask — subscribes to the 1-second heartbeat;
 *                                  triggers a sample every 10 ticks.
 *
 * The ADC is shared with the Battery worker. HAL_ADC_vLock / vUnlock
 * serialize access so only one measurement runs at a time.
 *
 * No averaging buffer — VSOLAR and RSENSE change faster than a sliding
 * window would be useful. The last measured value is always available
 * via the public getters.
 */

#include "SolarPower.h"
#include "SolarPower_Driver.h"
#include "SolarPower_Config.h"

#include "hal_system.h"
#include "platform.h"
#include "math_func.h"
#include "dbg_log.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---- Thread flags ---- */
#define SOLAR_NOTIFY_SAMPLE     (1UL << 0)

/* ---- Task stack / priority ---- */
#define SOLAR_SAMPLETASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)
#define SOLAR_SCHEDTASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)

/* ---- CMSIS-RTOS v2 handles ---- */
static osThreadId_t SOLAR_vSampleTask_handle;
static osThreadId_t SOLAR_vCheckSampleScheduleTask_handle;

/* ---- Last measured values (mV) ---- */
static volatile uint16_t u16VsolarLastMV;
static volatile uint16_t u16RsenseLastMV;

/* ---- Coulomb accumulator ---- */
static float fCoulombs;

/* ---- Forward declarations ---- */
static void SOLAR_vSampleTask(void *pvParameters);
static void SOLAR_vCheckSampleScheduleTask(void *pvParameters);

/* --------------------------------------------------------------------------
 * SOLAR_vInit
 * -------------------------------------------------------------------------- */
void SOLAR_vInit(void)
{
    static const osThreadAttr_t sample_attr = {
        .name       = "SolarSampleTask",
        .stack_size = SOLAR_SAMPLETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    static const osThreadAttr_t sched_attr = {
        .name       = "SolarSchedTask",
        .stack_size = SOLAR_SCHEDTASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    SOLAR_vSampleTask_handle =
        osThreadNew(SOLAR_vSampleTask, NULL, &sample_attr);
    SOLAR_vCheckSampleScheduleTask_handle =
        osThreadNew(SOLAR_vCheckSampleScheduleTask, NULL, &sched_attr);

    configASSERT(SOLAR_vSampleTask_handle              != NULL);
    configASSERT(SOLAR_vCheckSampleScheduleTask_handle != NULL);
}

/* --------------------------------------------------------------------------
 * SOLAR_vSampleTask — triggered by thread flag, performs ADC sampling
 * -------------------------------------------------------------------------- */
static void SOLAR_vSampleTask(void *pvParameters)
{
    (void)pvParameters;

    uint8_t  u8DelayMs;
    uint16_t u16RawVsolar;
    uint16_t u16RawRsense;

    for (;;)
    {
        osThreadFlagsWait(SOLAR_NOTIFY_SAMPLE, osFlagsWaitAny, osWaitForever);

        SOLAR_DRIVER_vLock();
        SYSTEM_vSleepLockAcquire();

        SOLAR_DRIVER_vEnable();

        /* ---- VSOLAR ---- */
        SOLAR_DRIVER_vCleanInterrupt();
        SOLAR_DRIVER_vStartVsolar();

        u8DelayMs = 3;
        while (!SOLAR_DRIVER_bGetInterruptFlag() && u8DelayMs--)
            osDelay(1);

        u16RawVsolar = SOLAR_DRIVER_u16GetResult();

        /* ---- RSENSE ---- */
        SOLAR_DRIVER_vCleanInterrupt();
        SOLAR_DRIVER_vStartRsense();

        u8DelayMs = 3;
        while (!SOLAR_DRIVER_bGetInterruptFlag() && u8DelayMs--)
            osDelay(1);

        u16RawRsense = SOLAR_DRIVER_u16GetResult();

        SOLAR_DRIVER_vDisable();

        SYSTEM_vSleepLockRelease();
        SOLAR_DRIVER_vUnlock();

        /* ---- Store converted values ---- */
        u16VsolarLastMV = (uint16_t)MATH_FUNC_i16ConvLin(u16RawVsolar,
                                                          SOLAR_VSOLAR_ADC_M_NUM,
                                                          SOLAR_VSOLAR_ADC_M_DEN,
                                                          SOLAR_VSOLAR_ADC_C);
        u16RsenseLastMV = (uint16_t)MATH_FUNC_i16ConvLin(u16RawRsense,
                                                          SOLAR_RSENSE_ADC_M_NUM,
                                                          SOLAR_RSENSE_ADC_M_DEN,
                                                          SOLAR_RSENSE_ADC_C);

        DBG("solar: Vsolar=%u mV  Vrsense=%u mV  I=%ld mA  P=%lu mW\r\n",
            u16VsolarLastMV, u16RsenseLastMV,
            SOLAR_i32GetCurrentMA(), SOLAR_u32GetPowerMW());

        /* ---- Coulomb accumulation: Q += I(A) × T(s) ---- */
        taskENTER_CRITICAL();
        fCoulombs += (SOLAR_i32GetCurrentMA() / 1000.0f) * SOLAR_SAMPLE_INTERVAL;
        taskEXIT_CRITICAL();
    }
}

/* --------------------------------------------------------------------------
 * SOLAR_vCheckSampleScheduleTask — heartbeat-driven sample scheduler
 * -------------------------------------------------------------------------- */
static void SOLAR_vCheckSampleScheduleTask(void *pvParameters)
{
    (void)pvParameters;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    uint8_t u8UpdateCnt = 0;

    for (;;)
    {
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        if (u8UpdateCnt % SOLAR_SAMPLE_INTERVAL == 0)
        {
            u8UpdateCnt = 0;
            osThreadFlagsSet(SOLAR_vSampleTask_handle, SOLAR_NOTIFY_SAMPLE);
        }
        u8UpdateCnt++;
    }
}

/* --------------------------------------------------------------------------
 * Public getters
 * -------------------------------------------------------------------------- */
uint16_t SOLAR_u16GetVSolarMV(void)
{
    return u16VsolarLastMV;
}

uint16_t SOLAR_u16GetVRSenseMV(void)
{
    return u16RsenseLastMV;
}

int32_t SOLAR_i32GetCurrentMA(void)
{
    /* I(mA) = Vrsense(mV) / R(Ω) = Vrsense(mV) × 100 / RSENSE_OHM_X100 */
    return (int32_t)u16RsenseLastMV * 100 / SOLAR_RSENSE_OHM_X100;
}

uint32_t SOLAR_u32GetPowerMW(void)
{
    /* P(mW) = Vsolar(mV) × I(mA) / 1000 */
    return (uint32_t)u16VsolarLastMV * (uint32_t)SOLAR_i32GetCurrentMA() / 1000UL;
}

float SOLAR_fGetCoulombs(void)
{
    taskENTER_CRITICAL();
    float f = fCoulombs;
    taskEXIT_CRITICAL();
    return f;
}

void SOLAR_vResetCoulombs(void)
{
    taskENTER_CRITICAL();
    fCoulombs = 0.0f;
    taskEXIT_CRITICAL();
}
