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
         * Wait for synchronized wake-up signal from wakeup-schedule task
         * ---------------------------------------------------------------- */
        osEventFlagsWait(xDiscoveryEventFlags,
                         DISCOVERY_WAKEUP_BIT,
                         osFlagsWaitAny,   /* auto-clears the bit */
                         osWaitForever);


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

    LOG(LOG_DISCOVERY_RECOVER, 1);
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
                LOG(LOG_DISCOVERY_RECOVER, 2);
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
                        LOG(LOG_DISCOVERY_RECOVER, 2);
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
        LOG(LOG_DISCOVERY_RECOVER, 3);
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
                LOG(LOG_DISCOVERY_RECOVER, 2);
                return;
            }
            LORARADIO_vEnterDeepSleep();
            if (i < (RECOVER_SOFT_WALK_N - 1U))
                osDelay(u32OffsetMs);
        }

        DBG("DeviceDiscovery: Recovery SOFT: walk complete, no primary\r\n");
        LOG(LOG_DISCOVERY_RECOVER, 3);
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
            LOG(LOG_DISCOVERY_RECOVER, 2);
            bSparseSkip = false;   /* reset skip state on clean exit */
            return;
        }

        DBG("DeviceDiscovery: Recovery SPARSE: no primary this scan\r\n");
        LOG(LOG_DISCOVERY_RECOVER, 3);
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
            LOG(LOG_DISCOVERY_RECOVER, 2);
            return;
        }

        DBG("DeviceDiscovery: Recovery DEEP: no primary found\r\n");
        LOG(LOG_DISCOVERY_RECOVER, 3);
    }
}
