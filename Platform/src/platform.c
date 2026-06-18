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
#include "hal_system.h"

#include "LoraRadio.h"
#include "MeshNetwork.h"
#include "DeviceDiscovery.h"
#include "radio_driver.h"
#include "LoraRadio_Driver.h"

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
        .stack_size = configMINIMAL_STACK_SIZE * 4 * sizeof(StackType_t),
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

        /* 1 Hz timeline marker on the debug UART. Only meaningful while the
         * device is actively awake — a sleep lock is held, so the debug UART has
         * been brought up by the module that took the lock. During idle wakes
         * (no lock, about to drop back into STOP2) emitting it would format a
         * verbose timestamped line and clock ~30 bytes out of USART2 every
         * second for nothing, so skip it. SYSTEM_bCheckSleepModeStatus() returns
         * true when no lock is held. */
        if (!SYSTEM_bCheckSleepModeStatus())
            DBG("\r\n");
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
