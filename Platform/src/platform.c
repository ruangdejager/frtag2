/*
 * platform.c
 *
 * 1 Hz heartbeat dispatcher.
 *
 * The heartbeat task waits for a CMSIS v2 thread-flag set by the RTC
 * wakeup ISR (via HAL_RTCEx_WakeUpTimerEventCallback → osThreadFlagsSet).
 * On each tick it resets the watchdog, runs time/movement housekeeping,
 * then notifies all subscribed application tasks by setting their 0x0001
 * thread flag.
 *
 * Tasks subscribe at startup with PLATFORM_bSubscribeToHeartbeat().
 * They wait for the flag with:
 *     osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
 */

#include "FreeRTOS.h"
#include "task.h"
#include "platform.h"
#include "dbg_log.h"
#include "hal_rtc.h"
#include "hal_wdt.h"

#include "LoraRadio.h"
#include "MeshNetwork.h"
#include "DeviceDiscovery.h"
#include "Movement.h"
#include "radio_driver.h"
#include "LoraRadio_Driver.h"

#include <time.h>
#include <limits.h>

typedef struct {
    osThreadId_t task;
    bool         enabled;
    uint32_t     flags;
} Subscriber_t;

static Subscriber_t  tSubscriber[MAX_SUBSCRIBERS];
static osMutexId_t   SubMutex;
static osThreadId_t  HeartbeatDispatchTask_handle;

/* --------------------------------------------------------------------------
 * PLATFORM_vInit
 * Creates the heartbeat task and the subscriber-list mutex.
 * Call before osKernelStart().
 * -------------------------------------------------------------------------- */
void PLATFORM_vInit(void)
{
    static const osThreadAttr_t heartbeat_attr = {
        .name       = "Heartbeat",
        .stack_size = configMINIMAL_STACK_SIZE * 8 * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    HeartbeatDispatchTask_handle = osThreadNew(PLATFORM_vHeartbeatDispatchTask,
                                               NULL, &heartbeat_attr);
    configASSERT(HeartbeatDispatchTask_handle != NULL);

    SubMutex = osMutexNew(NULL);
    configASSERT(SubMutex != NULL);
}

/* --------------------------------------------------------------------------
 * PLATFORM_vHeartbeatDispatchTask
 * Runs at 1 Hz, driven by the RTC wakeup ISR.
 * -------------------------------------------------------------------------- */
void PLATFORM_vHeartbeatDispatchTask(void *parameters)
{
    (void)parameters;

    for (;;)
    {
        /* Wait for flag set by HAL_RTCEx_WakeUpTimerEventCallback */
        osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);

        /* Reset the watchdog timer */
        HAL_WDT_vReset();

        /* Housekeeping ---------------------------------------------------- */
        TIME_vTick();
        if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
        {
#ifdef ENABLE_MOVE
            MOVE_bTick();
#endif
        }
        /* ------------------------------------------------------------------ */

        /* Notify all enabled subscribers ----------------------------------- */
        osMutexAcquire(SubMutex, osWaitForever);
        for (int i = 0; i < MAX_SUBSCRIBERS; i++)
        {
            if (!tSubscriber[i].task || !tSubscriber[i].enabled)
                continue;
            osThreadFlagsSet(tSubscriber[i].task, 0x0001U);
        }
        osMutexRelease(SubMutex);
        /* ------------------------------------------------------------------ */

        /* Print current UTC time to debug output */
        time_t rawtime = (time_t)RTC_u64GetUTC();
        struct tm ts;
        char buf[32];
        ts = *localtime(&rawtime);
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ts);
        DBG("%s\r\n", buf);
    }
}

/* --------------------------------------------------------------------------
 * Subscriber management
 * -------------------------------------------------------------------------- */
bool PLATFORM_bSubscribeToHeartbeat(osThreadId_t task, uint32_t flags)
{
    osMutexAcquire(SubMutex, osWaitForever);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++)
    {
        if (!tSubscriber[i].task)
        {
            tSubscriber[i].task    = task;
            tSubscriber[i].enabled = true;
            tSubscriber[i].flags   = flags;
            osMutexRelease(SubMutex);
            return true;
        }
    }
    osMutexRelease(SubMutex);
    return false;
}

void PLATFORM_vEnableHeartbeat(osThreadId_t task)
{
    osMutexAcquire(SubMutex, osWaitForever);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++)
        if (tSubscriber[i].task == task)
            tSubscriber[i].enabled = true;
    osMutexRelease(SubMutex);
}

void PLATFORM_vDisableHeartbeat(osThreadId_t task)
{
    osMutexAcquire(SubMutex, osWaitForever);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++)
        if (tSubscriber[i].task == task)
            tSubscriber[i].enabled = false;
    osMutexRelease(SubMutex);
}

osThreadId_t PLATFORM_tGetHeartbeatDispatchTaskHandle(void)
{
    return HeartbeatDispatchTask_handle;
}
