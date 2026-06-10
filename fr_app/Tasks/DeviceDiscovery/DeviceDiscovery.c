/*
 * DeviceDiscovery.c
 *
 * Application task — orchestrates mesh discovery, GPS fix, and data upload.
 *
 * Uses CMSIS-RTOS v2 throughout:
 *   - osEventFlags for synchronized wake-up (DISCOVERY_WAKEUP_BIT)
 *   - osThreadFlags for TimeSync notifications (DEVICE_DISCOVERY_NOTIFY_TIMESYNC)
 *   - osDelay instead of vTaskDelay / pdMS_TO_TICKS
 *   - osKernelGetTickCount instead of xTaskGetTickCount
 *   - osThreadGetId instead of xTaskGetCurrentTaskHandle
 */

#include "LoraRadio.h"
#include "DeviceDiscovery.h"
#include "MeshNetwork.h"
#include "Farmranger.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE */
#include "task.h"

#include "main.h"

#include <stdio.h>
#include <stdlib.h>

#include "dbg_log.h"
#include "platform_rtc.h"
#include "hal_rtc.h"
#include "platform.h"
#include "hal_system.h"
#include "Power.h"
#include "flashLog.h"
#include "GPS.h"
#include "SolarPower.h"
#include "SolarPower_Config.h"
#include "Debug.h"
#include "FrKernel.h"

/* ---- Private defines ---- */
#define APP_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 10)

/* ---- CMSIS-RTOS v2 objects ---- */
static osEventFlagsId_t  xDiscoveryEventFlags;
static osThreadId_t      DeviceDiscoveryAppTask_handle;
static osThreadId_t      DeviceDiscoveryWakeupTask_handle;

/* ---- Device state ---- */
DeviceRole_e              eDeviceRole;
uint32_t                  u32DreqId;
static volatile ProductionState_e eProductionState = PRODUCTION_READY;

/* ---- Forward declarations ---- */
static void DEVICE_DISCOVERY_vRecoveryMode(void);
static void DEVICE_DISCOVERY_vSendTS(void);
static void DEVICE_DISCOVERY_vCheckWakeupScheduleTask(void *pvParameters);

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vInit
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vInit(void)
{
    xDiscoveryEventFlags = osEventFlagsNew(NULL);
    configASSERT(xDiscoveryEventFlags != NULL);

    static const osThreadAttr_t app_attr = {
        .name       = "DevDiscoveryApp",
        .stack_size = APP_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    static const osThreadAttr_t wakeup_attr = {
        .name       = "CheckWakeupSched",
        .stack_size = configMINIMAL_STACK_SIZE * 2 * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    DeviceDiscoveryAppTask_handle =
        osThreadNew(DEVICE_DISCOVERY_vAppTask, NULL, &app_attr);
    DeviceDiscoveryWakeupTask_handle =
        osThreadNew(DEVICE_DISCOVERY_vCheckWakeupScheduleTask, NULL, &wakeup_attr);

    configASSERT(DeviceDiscoveryAppTask_handle    != NULL);
    configASSERT(DeviceDiscoveryWakeupTask_handle != NULL);

    uint32_t u32ModifiedCSR = u32GetCSR() >> 5;
    EVTLOG(LOG_RESET_CAUSE, u32ModifiedCSR);

    DBG_LOG("DeviceDiscovery: Initialized.\r\n");
    EVTLOG(LOG_DISCOVERY_INIT, eDeviceRole);

    if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        DBG_LOG("DeviceDiscovery: Device Role = PRIMARY\r\n");
    else
        DBG_LOG("DeviceDiscovery: Device Role = SECONDARY\r\n");
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vAppTask — main discovery state machine
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vAppTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        /* ----------------------------------------------------------------
         * Wait for wake-up: scheduled discovery or kernel wakeup (shake)
         * ---------------------------------------------------------------- */
        uint32_t u32Flags = osEventFlagsWait(xDiscoveryEventFlags,
                                             DISCOVERY_WAKEUP_BIT | DISCOVERY_KERNEL_BIT,
                                             osFlagsWaitAny,
                                             osWaitForever);

        bool bKernelWakeup = ((u32Flags & osFlagsError) == 0U) &&
                             ((u32Flags & DISCOVERY_KERNEL_BIT) != 0U);

        if (!bKernelWakeup)
        {

        /* GPS is NOT acquired here — the wake-schedule task pre-triggers it
         * 3 minutes before this wake. Consumers (MeshNetwork payload,
         * logger metadata, etc.) read the last known fix on demand via
         * GPS_bGetLastKnownFix() and use its age to decide. The AppTask
         * never blocks on a GPS fix. */

        /* ---- Prepare for new campaign ---- */
        MESHNETWORK_vClearDiscoveredNeighbors();
        MESHNETWORK_vResetDreqWaveCnt();

        DBG_LOG("DeviceDiscovery %X: Woke up for discovery.\r\n",
            LORARADIO_u32GetUniqueId());

        osDelay(APP_WAKEUP_BUFFER_MS);
        EVTLOG(LOG_DISCOVERY_START, eDeviceRole);

#ifdef LISTENER_MODE

        /* Listener: passively observe the full discovery window */
        DBG_LOG("DeviceDiscovery %X: LISTENER MODE - monitoring for %d ms\r\n",
            LORARADIO_u32GetUniqueId(), APP_DISCOVERY_WINDOW_TIMEOUT_MS);

        {
            osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC);
            uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                           osFlagsWaitAny,
                                           APP_DISCOVERY_WINDOW_TIMEOUT_MS);

            if (!(r & osFlagsError))
                DBG_LOG("DeviceDiscovery %X: Listener received TimeSync\r\n",
                    LORARADIO_u32GetUniqueId());
            else
                DBG_LOG("DeviceDiscovery %X: Listener window timed out\r\n",
                    LORARADIO_u32GetUniqueId());
        }

        /* Report devices observed during the listener window */
        {
            MeshDiscoveredNeighbor_t tNeighbors[MESH_MAX_NEIGHBORS];
            uint16_t u16NeighborCount = 0;

            if (MESHNETWORK_bGetDiscoveredNeighbors(tNeighbors, MESH_MAX_NEIGHBORS,
                                                    &u16NeighborCount))
            {
                DBG_LOG("DeviceDiscovery %X: Listener observed %u device(s).\r\n",
                    LORARADIO_u32GetUniqueId(), u16NeighborCount);
                EVTLOG(LOG_DISCOVERY_COUNT, u16NeighborCount);
                for (uint16_t i = 0; i < u16NeighborCount; i++)
                {
                    DBG_LOG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave);
                }
            }
            else
            {
                DBG_LOG("DeviceDiscovery %X: Error retrieving neighbor table.\r\n",
                    LORARADIO_u32GetUniqueId());
            }
        }

#else /* normal PRIMARY / SECONDARY behavior */

        if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        {
            bool bDiscoveryFinished = false;
            DBG_LOG("DeviceDiscovery: Primary starting discovery campaign\r\n");

            while (!bDiscoveryFinished)
            {
                u32DreqId = MESHNETWORK_u32GenerateGlobalMsgID();
                bool bBeaconSeenThisWave = false;

                MESHNETWORK_vIncrDreqWaveCnt();
                MESHNETWORK_bStartDiscoveryRound(u32DreqId);

                uint32_t tLastBeaconTick = MESHNETWORK_u32GetLastBeaconHeardTick();

                for (;;)
                {
                    osDelay(500);

                    uint32_t tNow           = osKernelGetTickCount();
                    uint32_t tMeshLastBeacon = MESHNETWORK_u32GetLastBeaconHeardTick();

                    if (tMeshLastBeacon != tLastBeaconTick)
                    {
                        bBeaconSeenThisWave  = true;
                        tLastBeaconTick      = tMeshLastBeacon;
                    }

                    if ((tNow - tLastBeaconTick) > MESH_DISCOVERY_IDLE_MS)
                        break;
                }

                if (!bBeaconSeenThisWave)
                {
                    bDiscoveryFinished = true;
                    MESHNETWORK_vStopPrimaryAck();
                }
                else
                {
                    DBG_LOG("DeviceDiscovery: Primary extending discovery with new DReq wave\r\n");
                }
            }
        }
        else
        {
            DBG_LOG("DeviceDiscovery %X: Secondary waiting for timesync.\r\n",
                LORARADIO_u32GetUniqueId());

            osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC);
            uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                           osFlagsWaitAny,
                                           APP_DISCOVERY_WINDOW_TIMEOUT_MS);

            if (!(r & osFlagsError))
                DBG_LOG("DeviceDiscovery: Secondary %04X: TimeSync received\r\n",
                    LORARADIO_u32GetUniqueId());
            else
                DBG_LOG("DeviceDiscovery: Secondary %04X: TimeSync timed out\r\n",
                    LORARADIO_u32GetUniqueId());

            MESHNETWORK_vStopBeaconing(u32DreqId);
        }

#endif /* LISTENER_MODE */

        /* ----------------------------------------------------------------
         * Discovery complete — log and upload (primary only)
         * ---------------------------------------------------------------- */
        DBG_LOG("DeviceDiscovery %X: Discovery complete.\r\n",
            LORARADIO_u32GetUniqueId());
        EVTLOG(LOG_DISCOVERY_CMPLT, eDeviceRole);

#ifndef LISTENER_MODE
        if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        {
            MeshDiscoveredNeighbor_t tNeighbors[MESH_MAX_NEIGHBORS];
            uint16_t u16NeighborCount = 0;

            if (MESHNETWORK_bGetDiscoveredNeighbors(tNeighbors, MESH_MAX_NEIGHBORS,
                                                    &u16NeighborCount))
            {
                DBG_LOG("DeviceDiscovery %X: Final UNION: %u neighbors.\r\n",
                    LORARADIO_u32GetUniqueId(), u16NeighborCount);
                EVTLOG(LOG_DISCOVERY_COUNT, u16NeighborCount);
                for (uint16_t i = 0; i < u16NeighborCount; i++)
                {
                    DBG_LOG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave);
                }
            }
            else
            {
                DBG_LOG("DeviceDiscovery %X: Error retrieving neighbor table.\r\n",
                    LORARADIO_u32GetUniqueId());
            }

#ifndef ENABLE_DBG_UART
            /* ---- Logger connection + upload ---- */
            DEVICE_DISCOVERY_DRIVER_bConnectLogger();

            DBG_LOG("DeviceDiscovery %X: Logger connected.\r\n",
                LORARADIO_u32GetUniqueId());

            if (DEVICE_DISCOVERY_bSendDiscoveryData(tNeighbors, u16NeighborCount))
                DBG_LOG("DeviceDiscovery %X: Log SUCCESS.\r\n", LORARADIO_u32GetUniqueId());
            else
            {
                DBG_LOG("DeviceDiscovery %X: Log FAILED.\r\n", LORARADIO_u32GetUniqueId());
                osDelay(2000);
            }

            /* ---- Timestamp sync ---- */
            uint64_t now = DEVICE_DISCOVERY_DRIVER_u64RequestTS();
            if (now > 0)
                RTC_vSetUTC(now);
            else
                DBG_LOG("DeviceDiscovery: Failed to get timestamp\r\n");

            /* ---- Wake-interval update ---- */
            uint8_t u8WakeInterval = DEVICE_DISCOVERY_DRIVER_u8RequestInterval();
            if      (u8WakeInterval == 15)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_15_MIN);
            else if (u8WakeInterval == 30)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_30_MIN);
            else if (u8WakeInterval == 60)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_60_MIN);
            else if (u8WakeInterval == 120) MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_120_MIN);

            DEVICE_DISCOVERY_DRIVER_vDisconnectLogger();
#endif /* ENABLE_DBG_UART */

            /* ---- Send TimeSync to secondaries ---- */
            DEVICE_DISCOVERY_vSendTS();
            EVTLOG(LOG_TX_TS, 1);

            osDelay(5000);
        }
        else
        {
            osDelay(5000);
        }

        /* ---- Recovery mode check (secondary only) ---- */
        {
            uint64_t now        = HAL_RTC_u64GetValue();
            uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

            if ((eDeviceRole == DEVICE_ROLE_SECONDARY) &&
                ((now - last_heard) > (uint64_t)LOST_PRIMARY_TIMEOUT_MIN * 60))
            {
                DBG_LOG("DeviceDiscovery: ENTERING RECOVERY MODE.\r\n");
                EVTLOG(LOG_DISCOVERY_RECOVER, 1);
                DEVICE_DISCOVERY_vRecoveryMode();
            }
        }
#endif /* LISTENER_MODE */

        } /* end if (!bKernelWakeup) */

        /* ---- Deep sleep ---- */
        MESHNETWORK_vResetNodeRole();

        EVTLOG(LOG_DEVICE_ENTERING_SLEEP, eDeviceRole);

        /* Hold off sleep while an active FrKernel session is in progress.
         * The user must issue "tag release" (or 5-min inactivity auto-releases). */
        if (FRKERNEL_bIsConnected())
        {
            DBG("DeviceDiscovery: FrKernel session active — waiting for release...\r\n");
            while (FRKERNEL_bIsConnected())
                osDelay(500);
        }

        DBG_LOG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
        osDelay(100);
//        BSP_LED_Off(LED_YELLOW);
        LORARADIO_vEnterDeepSleep();

        /* Campaign complete — release the sleep lock taken when this wake
         * was triggered (Discovery wake trigger or ProductionSleep exit). */
        SYSTEM_vSleepLockRelease();
    }
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vCheckWakeupScheduleTask
 *
 * Subscribes to the 1-second platform heartbeat.  Each tick it checks
 * whether the current UTC timestamp aligns with the configured wake interval;
 * if so (and power class permits) it wakes the discovery campaign.
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vCheckWakeupScheduleTask(void *pvParameters)
{
    (void)pvParameters;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
        /* Block until the platform heartbeat fires (any flag) */
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        uint64_t u64Utc       = RTC_u64GetUTC();
        uint32_t u32IntervalS = (uint32_t)MESHNETWORK_u8GetWakeupInterval() * 60U;
        uint32_t u32Phase     = (uint32_t)(u64Utc % (uint64_t)u32IntervalS);

        /* ---- ProductionSleep: secondary only — primary has no solar panel ---- */
        if (eDeviceRole == DEVICE_ROLE_SECONDARY && eProductionState == PRODUCTION_SLEEP)
        {
            if (SOLAR_u32GetPowerMW() >= SOLAR_ACTIVATION_POWER_MW)
            {
                eProductionState = PRODUCTION_ACTIVE;
                DBG_LOG("DeviceDiscovery: Solar activation (%lu mW) — exiting ProductionSleep\r\n",
                    SOLAR_u32GetPowerMW());
                /* Hold off deep sleep until the resulting campaign completes
                 * (released at the end of DEVICE_DISCOVERY_vAppTask's loop). */
                SYSTEM_vSleepLockAcquire();
                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_WAKEUP_BIT);
            }
            continue;
        }

//        BSP_LED_Toggle(LED_YELLOW);
        /* --- Discovery wake trigger (phase == 0) --- */
        if (u32Phase == 0U)
        {
            /* Scheduled wakes still fire in LOW; only RECOVERY suppresses them.
             * GPS gates strictly on NORMAL itself, so a LOW wake just produces
             * a discovery campaign without a fresh fix attempt. */
            if (POWER_tGetState() & (POWER_CLASS_NORMAL | POWER_CLASS_LOW))
            {
                /* Hold off deep sleep until the discovery campaign completes
                 * (released at the end of DEVICE_DISCOVERY_vAppTask's loop). */
                SYSTEM_vSleepLockAcquire();

                if (eDeviceRole == DEVICE_ROLE_PRIMARY)
                    FARMRANGER_vUartOnWake();

                HAL_UART_vInit();
                DEBUG_vInit();

                LORARADIO_vWakeUp();

                DBG_LOG("\r\n--- WAKEUP ---\r\n");
                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_WAKEUP_BIT);

            }
        }

#ifdef ENABLE_GPS
        /* --- GPS pre-trigger (SECONDARY only) ---
         * GPS is not fitted on PRIMARY boards. Fire-and-forget 3 minutes
         * before each scheduled wake so a fresh fix is cached by the time
         * the AppTask runs. The AppTask never blocks on a GPS result. */
        if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
            u32IntervalS > DEVICE_DISCOVERY_GPS_PRETRIGGER_S &&
            u32Phase == (u32IntervalS - DEVICE_DISCOVERY_GPS_PRETRIGGER_S))
        {
            /* Hold off deep sleep until GPS auto-shuts-down, times out, or is
             * otherwise turned off — released inside GPS_vPowerOff(). */
            SYSTEM_vSleepLockAcquire();
            HAL_UART_vInit();
            DEBUG_vInit();

            DBG("DeviceDiscovery: GPS pre-trigger\r\n");
            /* auto-shutdown on completion, bounded to GPS_PRETRIGGER_S so it
             * can never run forever */
            GPS_vRequestFix(true, 120);
        }
#endif
    }
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vSendTS
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vSendTS(void)
{
    DBG_LOG("\r\n--- START TIMESYNC ---\r\n");
    MESHNETWORK_vSendTimeSync(RTC_u64GetUTC(), MESHNETWORK_tGetWakeupInterval());
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vTriggerKernelWakeup
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vTriggerKernelWakeup(void)
{
    /* Hold off deep sleep until the resulting (no-op) campaign completes —
     * released at the end of DEVICE_DISCOVERY_vAppTask's loop. */
    SYSTEM_vSleepLockAcquire();
    LORARADIO_vWakeUp();
    DBG("\r\n--- KERNEL WAKEUP ---\r\n");
    osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_KERNEL_BIT);
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vConfigDeviceRole
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vConfigDeviceRole(void)
{
    eDeviceRole = HAL_GPIO_ReadPin(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN)
                  ? DEVICE_ROLE_PRIMARY
                  : DEVICE_ROLE_SECONDARY;
    HAL_GPIO_DeInit(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN);
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_eGetDeviceRole
 * -------------------------------------------------------------------------- */
DeviceRole_e DEVICE_DISCOVERY_eGetDeviceRole(void)
{
    return eDeviceRole;
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_xGetTaskHandle
 * -------------------------------------------------------------------------- */
osThreadId_t DEVICE_DISCOVERY_xGetTaskHandle(void)
{
    return DeviceDiscoveryAppTask_handle;
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vEnterProductionSleep
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vEnterProductionSleep(void)
{
    if (eDeviceRole != DEVICE_ROLE_SECONDARY)
    {
        DBG_LOG("DeviceDiscovery: ProductionSleep not applicable on primary device\r\n");
        return;
    }
    eProductionState = PRODUCTION_SLEEP;
    DBG_LOG("DeviceDiscovery: Entering ProductionSleep\r\n");
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_eGetProductionState
 * -------------------------------------------------------------------------- */
ProductionState_e DEVICE_DISCOVERY_eGetProductionState(void)
{
    return eProductionState;
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vRecoveryMode
 *
 * Listens for up to 2 hours for the primary to reappear.
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vRecoveryMode(void)
{
    DBG_LOG("DeviceDiscovery: Node %X recovery; LISTENING FOR PRIMARY.\r\n",
        LORARADIO_u32GetUniqueId());

    for (uint16_t i = 0; i < 120 * 60; i++)
    {
        osDelay(1000);

        uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

        if (last_heard != 0 &&
            (HAL_RTC_u64GetValue() - last_heard) < (uint64_t)LOST_PRIMARY_TIMEOUT_MIN * 60)
        {
            DBG_LOG("DeviceDiscovery: Node %X recovered; PRIMARY FOUND.\r\n",
                LORARADIO_u32GetUniqueId());
            EVTLOG(LOG_DISCOVERY_RECOVER, 2);
            return;
        }
    }

    DBG_LOG("DeviceDiscovery: Node %X not recovered; NO PRIMARY FOUND.\r\n",
        LORARADIO_u32GetUniqueId());
    EVTLOG(LOG_DISCOVERY_RECOVER, 3);
    MESHNETWORK_vUpdatePrimaryLastSeen();
}
