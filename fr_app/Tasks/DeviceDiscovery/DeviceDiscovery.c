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

/* ---- Private defines ---- */
#define APP_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 10)

/* ---- CMSIS-RTOS v2 objects ---- */
static osEventFlagsId_t  xDiscoveryEventFlags;
static osThreadId_t      DeviceDiscoveryAppTask_handle;
static osThreadId_t      DeviceDiscoveryWakeupTask_handle;

/* ---- Device state ---- */
DeviceRole_e eDeviceRole;
uint32_t     u32DreqId;

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
        .stack_size = configMINIMAL_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityBelowNormal,
    };

    DeviceDiscoveryAppTask_handle =
        osThreadNew(DEVICE_DISCOVERY_vAppTask, NULL, &app_attr);
    DeviceDiscoveryWakeupTask_handle =
        osThreadNew(DEVICE_DISCOVERY_vCheckWakeupScheduleTask, NULL, &wakeup_attr);

    configASSERT(DeviceDiscoveryAppTask_handle    != NULL);
    configASSERT(DeviceDiscoveryWakeupTask_handle != NULL);

    uint32_t u32ModifiedCSR = u32GetCSR() >> 5;
    LOG(LOG_RESET_CAUSE, u32ModifiedCSR);

    DBG("DeviceDiscovery: Initialized.\r\n");
    LOG(LOG_DISCOVERY_INIT, eDeviceRole);

    if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        DBG("DeviceDiscovery: Device Role = PRIMARY\r\n");
    else
        DBG("DeviceDiscovery: Device Role = SECONDARY\r\n");
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

#ifdef ENABLE_GPS
        /* ---- GPS fix ---- */
        GPS_vStart(osThreadGetId());
        osThreadFlagsWait(GPS_NOTIFY_FIX_OK | GPS_NOTIFY_FIX_TIMEOUT,
                          osFlagsWaitAny,
                          10000);

        gnss_coord_deg_t lat, lon;
        if (GPS_bGetCoordinates(&lat, &lon))
        {
            /* lat.fDegrees, lon.fDegrees — or use i32MicroDeg for integer math */
        }
        GPS_vStop();
#endif

        /* ---- Prepare for new campaign ---- */
        MESHNETWORK_vClearDiscoveredNeighbors();
        MESHNETWORK_vResetDreqWaveCnt();

        DBG("DeviceDiscovery %X: Woke up for discovery.\r\n",
            LORARADIO_u32GetUniqueId());

        osDelay(APP_WAKEUP_BUFFER_MS);
        LOG(LOG_DISCOVERY_START, eDeviceRole);

#ifdef LISTENER_MODE

        /* Listener: passively observe the full discovery window */
        DBG("DeviceDiscovery %X: LISTENER MODE - monitoring for %d ms\r\n",
            LORARADIO_u32GetUniqueId(), APP_DISCOVERY_WINDOW_TIMEOUT_MS);

        {
            osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC);
            uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                           osFlagsWaitAny,
                                           APP_DISCOVERY_WINDOW_TIMEOUT_MS);

            if (!(r & osFlagsError))
                DBG("DeviceDiscovery %X: Listener received TimeSync\r\n",
                    LORARADIO_u32GetUniqueId());
            else
                DBG("DeviceDiscovery %X: Listener window timed out\r\n",
                    LORARADIO_u32GetUniqueId());
        }

        /* Report devices observed during the listener window */
        {
            MeshDiscoveredNeighbor_t tNeighbors[MESH_MAX_NEIGHBORS];
            uint16_t u16NeighborCount = 0;

            if (MESHNETWORK_bGetDiscoveredNeighbors(tNeighbors, MESH_MAX_NEIGHBORS,
                                                    &u16NeighborCount))
            {
                DBG("DeviceDiscovery %X: Listener observed %u device(s).\r\n",
                    LORARADIO_u32GetUniqueId(), u16NeighborCount);
                LOG(LOG_DISCOVERY_COUNT, u16NeighborCount);
                for (uint16_t i = 0; i < u16NeighborCount; i++)
                {
                    DBG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave);
                }
            }
            else
            {
                DBG("DeviceDiscovery %X: Error retrieving neighbor table.\r\n",
                    LORARADIO_u32GetUniqueId());
            }
        }

#else /* normal PRIMARY / SECONDARY behavior */

        if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        {
            bool bDiscoveryFinished = false;
            DBG("DeviceDiscovery: Primary starting discovery campaign\r\n");

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
                    DBG("DeviceDiscovery: Primary extending discovery with new DReq wave\r\n");
                }
            }
        }
        else
        {
            DBG("DeviceDiscovery %X: Secondary waiting for timesync.\r\n",
                LORARADIO_u32GetUniqueId());

            osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC);
            uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                           osFlagsWaitAny,
                                           APP_DISCOVERY_WINDOW_TIMEOUT_MS);

            if (!(r & osFlagsError))
                DBG("DeviceDiscovery: Secondary %04X: TimeSync received\r\n",
                    LORARADIO_u32GetUniqueId());
            else
                DBG("DeviceDiscovery: Secondary %04X: TimeSync timed out\r\n",
                    LORARADIO_u32GetUniqueId());

            MESHNETWORK_vStopBeaconing(u32DreqId);
        }

#endif /* LISTENER_MODE */

        /* ----------------------------------------------------------------
         * Discovery complete — log and upload (primary only)
         * ---------------------------------------------------------------- */
        DBG("DeviceDiscovery %X: Discovery complete.\r\n",
            LORARADIO_u32GetUniqueId());
        LOG(LOG_DISCOVERY_CMPLT, eDeviceRole);

#ifndef LISTENER_MODE
        if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        {
            MeshDiscoveredNeighbor_t tNeighbors[MESH_MAX_NEIGHBORS];
            uint16_t u16NeighborCount = 0;

            if (MESHNETWORK_bGetDiscoveredNeighbors(tNeighbors, MESH_MAX_NEIGHBORS,
                                                    &u16NeighborCount))
            {
                DBG("DeviceDiscovery %X: Final UNION: %u neighbors.\r\n",
                    LORARADIO_u32GetUniqueId(), u16NeighborCount);
                LOG(LOG_DISCOVERY_COUNT, u16NeighborCount);
                for (uint16_t i = 0; i < u16NeighborCount; i++)
                {
                    DBG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave);
                }
            }
            else
            {
                DBG("DeviceDiscovery %X: Error retrieving neighbor table.\r\n",
                    LORARADIO_u32GetUniqueId());
            }

#ifndef ENABLE_DBG_UART
            /* ---- Logger connection + upload ---- */
            DEVICE_DISCOVERY_DRIVER_bConnectLogger();

            DBG("DeviceDiscovery %X: Logger connected.\r\n",
                LORARADIO_u32GetUniqueId());

            if (DEVICE_DISCOVERY_bSendDiscoveryData(tNeighbors, u16NeighborCount))
                DBG("DeviceDiscovery %X: Log SUCCESS.\r\n", LORARADIO_u32GetUniqueId());
            else
            {
                DBG("DeviceDiscovery %X: Log FAILED.\r\n", LORARADIO_u32GetUniqueId());
                osDelay(2000);
            }

            /* ---- Timestamp sync ---- */
            uint64_t now = DEVICE_DISCOVERY_DRIVER_u64RequestTS();
            if (now > 0)
                RTC_vSetUTC(now);
            else
                DBG("DeviceDiscovery: Failed to get timestamp\r\n");

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
            LOG(LOG_TX_TS, 1);

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
                DBG("DeviceDiscovery: ENTERING RECOVERY MODE.\r\n");
                LOG(LOG_DISCOVERY_RECOVER, 1);
                DEVICE_DISCOVERY_vRecoveryMode();
            }
        }
#endif /* LISTENER_MODE */

        } /* end if (!bKernelWakeup) */

        /* ---- Deep sleep ---- */
        MESHNETWORK_vResetNodeRole();

        LOG(LOG_DEVICE_ENTERING_SLEEP, eDeviceRole);
        DBG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
        osDelay(100);
        BSP_LED_Off(LED_YELLOW);
        LORARADIO_vEnterDeepSleep();
        SYSTEM_vActivateDeepSleep();
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

        if (RTC_u64GetUTC() % ((uint64_t)MESHNETWORK_u8GetWakeupInterval() * 60) == 0)
        {
            if (POWER_tGetState() & POWER_CLASS_NORMAL)
            {
                SYSTEM_vDeactivateDeepSleep();

                if (eDeviceRole == DEVICE_ROLE_PRIMARY)
                    FARMRANGER_vUartOnWake();

#ifdef ENABLE_DBG_UART
                HAL_UART_vInit();
                DBG_UART_vInit();
#endif
                LORARADIO_vWakeUp();

                DBG("\r\n--- WAKEUP ---\r\n");
                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_WAKEUP_BIT);
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vSendTS
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vSendTS(void)
{
    DBG("\r\n--- START TIMESYNC ---\r\n");
    MESHNETWORK_vSendTimeSync(RTC_u64GetUTC(), MESHNETWORK_tGetWakeupInterval());
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vTriggerKernelWakeup
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vTriggerKernelWakeup(void)
{
    SYSTEM_vDeactivateDeepSleep();
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
 * DEVICE_DISCOVERY_vRecoveryMode
 *
 * Listens for up to 2 hours for the primary to reappear.
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vRecoveryMode(void)
{
    DBG("DeviceDiscovery: Node %X recovery; LISTENING FOR PRIMARY.\r\n",
        LORARADIO_u32GetUniqueId());

    for (uint16_t i = 0; i < 120 * 60; i++)
    {
        osDelay(1000);

        uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

        if (last_heard != 0 &&
            (HAL_RTC_u64GetValue() - last_heard) < (uint64_t)LOST_PRIMARY_TIMEOUT_MIN * 60)
        {
            DBG("DeviceDiscovery: Node %X recovered; PRIMARY FOUND.\r\n",
                LORARADIO_u32GetUniqueId());
            LOG(LOG_DISCOVERY_RECOVER, 2);
            return;
        }
    }

    DBG("DeviceDiscovery: Node %X not recovered; NO PRIMARY FOUND.\r\n",
        LORARADIO_u32GetUniqueId());
    LOG(LOG_DISCOVERY_RECOVER, 3);
    MESHNETWORK_vUpdatePrimaryLastSeen();
}
