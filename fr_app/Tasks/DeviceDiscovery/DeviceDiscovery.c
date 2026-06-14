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
#ifdef ENABLE_LOW_POWER_RECOVERY
static void DEVICE_DISCOVERY_vRecoveryMode(void);
#endif
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

        /* Multi-primary: arm the "first TimeSync only" gate for this
         * wake cycle. Subsequent TimeSyncs from other primaries during
         * this campaign will be forwarded but not re-applied. */
        MESHNETWORK_vResetTimeSyncAccepted();

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
        /* R6: drop any TX left over from the previous campaign (e.g. a
         * late-jittered forward) so it can't fire at the start of this one. */
        MESHNETWORK_vFlushTxQueue();

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
                    DBG_LOG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d  Move:%u  Lat:%ld  Lon:%ld\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave,
                        tNeighbors[i].u8MoveState,
                        tNeighbors[i].bGpsValid ? (long)tNeighbors[i].i32LatUDeg : 0L,
                        tNeighbors[i].bGpsValid ? (long)tNeighbors[i].i32LonUDeg : 0L);
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
            bool    bDiscoveryFinished = false;
            uint8_t u8WaveCount        = 0;
            DBG_LOG("DeviceDiscovery: Primary starting discovery campaign\r\n");

            while (!bDiscoveryFinished)
            {
                u32DreqId = MESHNETWORK_u32GenerateGlobalMsgID();
                bool bBeaconSeenThisWave = false;

                MESHNETWORK_vIncrDreqWaveCnt();
                MESHNETWORK_bStartDiscoveryRound(u32DreqId);
                u8WaveCount++;

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

                if (!bBeaconSeenThisWave || u8WaveCount >= APP_PRIMARY_MAX_WAVES)
                {
                    if (bBeaconSeenThisWave)
                        DBG_LOG("DeviceDiscovery: Primary wave cap (%u) reached\r\n",
                            APP_PRIMARY_MAX_WAVES);
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

            /* R3: end the campaign on the FIRST of:
             *   - TimeSync received (clean end),
             *   - 10 s of mesh radio silence while NOT beaconing (UNKNOWN that
             *     heard nothing, or FORWARDER once the mesh goes quiet),
             *   - the 180 s hard cap.
             * While beaconing the silence rule is suppressed; that path ends via
             * MeshNetwork's beacon cap, which flips the node to FORWARDER, after
             * which the silence rule resumes. u32SilenceRef starts at the
             * campaign start (fair first window) and advances to the latest
             * discovery packet from ANY primary (multi-primary safe). */
            uint32_t u32CampaignStart = osKernelGetTickCount();
            uint32_t u32SilenceRef    = u32CampaignStart;
            bool     bTimeSync        = false;

            for (;;)
            {
                uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                               osFlagsWaitAny,
                                               APP_SECONDARY_POLL_MS);
                if (!(r & osFlagsError)) { bTimeSync = true; break; }

                uint32_t u32Now = osKernelGetTickCount();

                /* advance the silence reference to the newest discovery packet */
                uint32_t u32LastPkt = MESHNETWORK_u32GetLastDiscoveryPktTick();
                if ((int32_t)(u32LastPkt - u32SilenceRef) > 0)
                    u32SilenceRef = u32LastPkt;

                if ((uint32_t)(u32Now - u32CampaignStart) >= APP_DISCOVERY_WINDOW_TIMEOUT_MS)
                    break;   /* hard cap */

                if (!MESHNETWORK_bIsBeaconing() &&
                    (uint32_t)(u32Now - u32SilenceRef) >= APP_SECONDARY_SILENCE_MS)
                    break;   /* radio silence, not beaconing */
            }

            if (bTimeSync)
                DBG_LOG("DeviceDiscovery: Secondary %04X: TimeSync received\r\n",
                    LORARADIO_u32GetUniqueId());
            else
                DBG_LOG("DeviceDiscovery: Secondary %04X: campaign end (silence/cap)\r\n",
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
                    DBG_LOG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d  Move:%u  Lat:%ld  Lon:%ld\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave,
                        tNeighbors[i].u8MoveState,
                        tNeighbors[i].bGpsValid ? (long)tNeighbors[i].i32LatUDeg : 0L,
                        tNeighbors[i].bGpsValid ? (long)tNeighbors[i].i32LonUDeg : 0L);
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

#ifdef ENABLE_LOW_POWER_RECOVERY
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
#endif /* ENABLE_LOW_POWER_RECOVERY */
#endif /* LISTENER_MODE */

        } /* end if (!bKernelWakeup) */
        else
        {
            /* Kernel wakeup (shake-sequence): give the user a window to start
             * a FrKernel session before falling back to sleep. Without this,
             * FRKERNEL_bIsConnected() below is still false at the instant we
             * wake (no command has been sent yet) and we'd go straight back
             * to sleep. */
            DBG_LOG("DeviceDiscovery: Kernel wakeup - waiting up to %u s for FrKernel session...\r\n",
                DEVICE_DISCOVERY_KERNEL_WAKEUP_WINDOW_MS / 1000U);

            uint32_t u32WaitedMs = 0U;
            while (!FRKERNEL_bIsConnected() && u32WaitedMs < DEVICE_DISCOVERY_KERNEL_WAKEUP_WINDOW_MS)
            {
                osDelay(500);
                u32WaitedMs += 500U;
            }

            if (!FRKERNEL_bIsConnected())
                DBG_LOG("DeviceDiscovery: Kernel wakeup window expired, no session - sleeping\r\n");
        }

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
#ifdef ENABLE_SOLAR_POWER_SENSE
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
#else
            /* Panel power sensing is disabled (broken RSENSE front-end); gate
             * ProductionSleep exit on panel VOLTAGE instead. */
            if (SOLAR_u16GetVSolarMV() >= SOLAR_ACTIVATION_VSOLAR_MV)
            {
                eProductionState = PRODUCTION_ACTIVE;
                DBG_LOG("DeviceDiscovery: Solar activation (%u mV) — exiting ProductionSleep\r\n",
                    SOLAR_u16GetVSolarMV());
                /* Hold off deep sleep until the resulting campaign completes
                 * (released at the end of DEVICE_DISCOVERY_vAppTask's loop). */
                SYSTEM_vSleepLockAcquire();
                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_WAKEUP_BIT);
            }
#endif
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
            u32Phase == (u32IntervalS - DEVICE_DISCOVERY_GPS_PRETRIGGER_S) &&
            (POWER_tGetState() & POWER_CLASS_NORMAL))
        {
            /* Hold off deep sleep until GPS auto-shuts-down, times out, or is
             * otherwise turned off — released inside GPS_vPowerOff().
             *
             * The power-class check above is required, not just an
             * optimisation: GPS_vRequestFix() refuses outright with
             * GPS_RESULT_NO_POWER when the board isn't in POWER_CLASS_NORMAL,
             * and on that path it returns without ever calling
             * GPS_vPowerOff(). If we acquired the lock unconditionally, that
             * refusal would leak it forever — gSleepLockCount would never
             * return to 0, deep sleep would be permanently disabled, and
             * every subsequent pre-trigger would silently refuse the same
             * way (no "session armed" line ever again). */
            SYSTEM_vSleepLockAcquire();
            HAL_UART_vInit();
            DEBUG_vInit();

            DBG("DeviceDiscovery: GPS pre-trigger\r\n");
            /* auto-shutdown on completion, bounded to GPS_PRETRIGGER_S so it
             * can never run forever */
            GPS_vRequestFix(true, 90);
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
#ifdef FRKERNEL_INTERFACE_LORA
    LORARADIO_vWakeUp();
#else
    HAL_UART_vInit();
    DEBUG_vInit();
#endif
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

#ifdef ENABLE_LOW_POWER_RECOVERY
/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vRecoveryMode  — tiered low-duty-cycle recovery
 *
 * Replaces the former 2-hour continuous-RX loop with four tiers that trade
 * progressively more battery for progressively lower primary-detect rate.
 *
 * Tier 1 – Sniff      ( 8 h – 24 h silence)
 *   Passive 10 % duty cycle: 2 s radio on / 18 s off.  No TX / beaconing.
 *   DBeacon heard in the 2-s window → 100 % duty cycle (radio stays on).
 *   100 % mode exits back to sniff after 20 s with no beacon activity, or
 *   exits recovery immediately on TimeSync.
 *   Budget: RECOVER_SNIFF_CYCLES sniff cycles (≈ 10 min) per wake.
 *
 * Tier 2 – Soft       (24 h – 72 h)
 *   RECOVER_SOFT_WALK_N × RECOVER_SOFT_RX_MS probes separated by
 *   interval/N gaps (linear offset walk across the full wakeup period).
 *
 * Tier 3 – Sparse     (72 h – 7 d)
 *   30-s RX; every other scheduled wake is skipped (net ≈ 2× interval).
 *
 * Tier 4 – Deep probe (> 7 d)
 *   60-s RX every RECOVER_DEEP_PERIOD_S seconds; all other wakes skipped.
 *
 * All tiers: TimeSync received → return immediately (caller continues to
 * normal post-discovery path, then deep sleep).
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vRecoveryMode(void)
{
    uint64_t u64Now       = HAL_RTC_u64GetValue();
    uint64_t u64LastHeard = MESHNETWORK_u64GetLastPrimaryHeardTick();
    uint64_t u64SilenceS  = (u64Now > u64LastHeard) ? (u64Now - u64LastHeard) : 0ULL;

    EVTLOG(LOG_DISCOVERY_RECOVER, 1);
    DBG("DeviceDiscovery: Recovery mode — node %X, silence %llu s\r\n",
        LORARADIO_u32GetUniqueId(), u64SilenceS);

    /* Discard any stale TimeSync notification left from the discovery cycle */
    osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC);

    /* ----------------------------------------------------------------
     * Tier 1 – Sniff  (8 h – 24 h)
     * Passive 10 % duty cycle: 2 s RX on / 18 s radio off.
     * No beaconing is triggered — only the MeshNetwork parser task runs.
     *
     * Any DBeacon heard in the 2-s window escalates to 100 % duty cycle:
     *   - Radio stays on continuously.
     *   - Every RECOVER_ESCALATE_POLL_MS we check whether beacon activity
     *     has been seen recently.
     *   - If no DBeacon for RECOVER_ESCALATE_IDLE_MS (20 s) → drop back.
     *   - TimeSync received at any point → exit recovery immediately.
     *
     * RECOVER_SNIFF_CYCLES caps the per-wake sniff budget (≈ 10 min).
     * ---------------------------------------------------------------- */
    if (u64SilenceS < RECOVER_SILENCE_SOFT_S)
    {
        DBG("DeviceDiscovery: Recovery SNIFF tier\r\n");

        uint8_t u8Cycles = 0;
        while (u8Cycles < RECOVER_SNIFF_CYCLES)
        {
            LORARADIO_vWakeUp();

            /* Sample discovery-packet tick before the 2-s window */
            uint32_t u32BeaconBefore = MESHNETWORK_u32GetLastDiscoveryPktTick();

            /* 2-s passive RX — MeshNetwork parser task handles any packet */
            uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                           osFlagsWaitAny, RECOVER_SNIFF_RX_MS);
            if (!(r & osFlagsError))
            {
                DBG("DeviceDiscovery: Recovery SNIFF: TimeSync — exiting recovery\r\n");
                EVTLOG(LOG_DISCOVERY_RECOVER, 2);
                return;
            }

            /* Check whether any discovery packet arrived during the window */
            uint32_t u32BeaconTick = MESHNETWORK_u32GetLastDiscoveryPktTick();
            if (u32BeaconTick != u32BeaconBefore)
            {
                /* ---- 100 % duty cycle: radio stays on ---- */
                DBG("DeviceDiscovery: Recovery SNIFF: activity — 100%% RX\r\n");

                uint32_t u32IdleStartMs = osKernelGetTickCount();

                for (;;)
                {
                    /* Poll for TimeSync; use short interval to stay responsive */
                    r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                         osFlagsWaitAny, RECOVER_ESCALATE_POLL_MS);
                    if (!(r & osFlagsError))
                    {
                        DBG("DeviceDiscovery: Recovery 100%%: TimeSync — exiting\r\n");
                        EVTLOG(LOG_DISCOVERY_RECOVER, 2);
                        return;
                    }

                    /* Reset idle timer whenever any discovery packet is seen */
                    uint32_t u32NewTick = MESHNETWORK_u32GetLastDiscoveryPktTick();
                    if (u32NewTick != u32BeaconTick)
                    {
                        u32BeaconTick    = u32NewTick;
                        u32IdleStartMs   = osKernelGetTickCount();
                    }

                    /* 20 s with no activity → drop back to 10 % sniff */
                    if ((osKernelGetTickCount() - u32IdleStartMs) >= RECOVER_ESCALATE_IDLE_MS)
                    {
                        DBG("DeviceDiscovery: Recovery 100%%: idle — resuming sniff\r\n");
                        break;
                    }
                }

                /* Restart sniff budget; radio is still on for the next window */
                u8Cycles = 0;
                continue;
            }

            /* Quiet window — radio off for a jittered 13–23 s, then try again.
             * Jitter spreads sniff cycles across the fleet so that two stranded
             * tags do not lock into the same 20-s phase indefinitely. */
            LORARADIO_vEnterDeepSleep();
            uint32_t u32SleepMs = RECOVER_SNIFF_SLEEP_MIN_MS +
                LORARADIO_u32GetRandomNumber(RECOVER_SNIFF_SLEEP_MAX_MS
                                             - RECOVER_SNIFF_SLEEP_MIN_MS);
            osDelay(u32SleepMs);
            u8Cycles++;
        }

        /* Sniff budget exhausted this wake; radio is in deep sleep.
         * AppTask main loop will handle the next osEventFlagsWait. */
        DBG("DeviceDiscovery: Recovery SNIFF: budget exhausted this wake\r\n");
        EVTLOG(LOG_DISCOVERY_RECOVER, 3);
        return;
    }

    /* ----------------------------------------------------------------
     * Tier 2 – Soft  (24 h – 72 h)
     * RECOVER_SOFT_WALK_N × 30-s probes with interval/N gaps between
     * them (linear offset walk across the configured wakeup interval).
     * ---------------------------------------------------------------- */
    if (u64SilenceS < RECOVER_SILENCE_SPARSE_S)
    {
        DBG("DeviceDiscovery: Recovery SOFT tier\r\n");

        uint32_t u32OffsetMs = (uint32_t)MESHNETWORK_u8GetWakeupInterval()
                               * 60U * 1000U / RECOVER_SOFT_WALK_N;

        for (uint8_t i = 0; i < RECOVER_SOFT_WALK_N; i++)
        {
            LORARADIO_vWakeUp();
            uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                           osFlagsWaitAny, RECOVER_SOFT_RX_MS);
            if (!(r & osFlagsError))
            {
                DBG("DeviceDiscovery: Recovery SOFT: TimeSync — exiting recovery\r\n");
                EVTLOG(LOG_DISCOVERY_RECOVER, 2);
                return;
            }
            LORARADIO_vEnterDeepSleep();
            if (i < (RECOVER_SOFT_WALK_N - 1U))
                osDelay(u32OffsetMs);
        }

        DBG("DeviceDiscovery: Recovery SOFT: walk complete, no primary\r\n");
        EVTLOG(LOG_DISCOVERY_RECOVER, 3);
        return;
    }

    /* ----------------------------------------------------------------
     * Tier 3 – Sparse  (72 h – 7 d)
     * 30-s RX scan; every other scheduled wake is skipped, giving a
     * net detection period of approximately 2× the wakeup interval.
     * ---------------------------------------------------------------- */
    if (u64SilenceS < RECOVER_SILENCE_DEEP_S)
    {
        static bool bSparseSkip = false;

        if (bSparseSkip)
        {
            bSparseSkip = false;
            DBG("DeviceDiscovery: Recovery SPARSE: skipping wake\r\n");
            return;
        }
        bSparseSkip = true;

        DBG("DeviceDiscovery: Recovery SPARSE tier\r\n");

        uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                       osFlagsWaitAny, RECOVER_SPARSE_RX_MS);
        if (!(r & osFlagsError))
        {
            DBG("DeviceDiscovery: Recovery SPARSE: TimeSync — exiting recovery\r\n");
            EVTLOG(LOG_DISCOVERY_RECOVER, 2);
            bSparseSkip = false;   /* reset skip state on clean exit */
            return;
        }

        DBG("DeviceDiscovery: Recovery SPARSE: no primary this scan\r\n");
        EVTLOG(LOG_DISCOVERY_RECOVER, 3);
        return;
    }

    /* ----------------------------------------------------------------
     * Tier 4 – Deep probe  (> 7 d)
     * 60-s RX scan every RECOVER_DEEP_PERIOD_S seconds (6 h).
     * All other scheduled wakes are skipped entirely.
     * ---------------------------------------------------------------- */
    {
        static uint64_t u64LastDeepProbeS = 0ULL;

        if ((u64Now - u64LastDeepProbeS) < RECOVER_DEEP_PERIOD_S)
        {
            DBG("DeviceDiscovery: Recovery DEEP: skipping wake\r\n");
            return;
        }
        u64LastDeepProbeS = u64Now;

        DBG("DeviceDiscovery: Recovery DEEP tier\r\n");

        uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
                                       osFlagsWaitAny, RECOVER_DEEP_RX_MS);
        if (!(r & osFlagsError))
        {
            DBG("DeviceDiscovery: Recovery DEEP: TimeSync — exiting recovery\r\n");
            EVTLOG(LOG_DISCOVERY_RECOVER, 2);
            return;
        }

        DBG("DeviceDiscovery: Recovery DEEP: no primary found\r\n");
        EVTLOG(LOG_DISCOVERY_RECOVER, 3);
    }
}
#endif /* ENABLE_LOW_POWER_RECOVERY */
