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
#include "build_config.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE */
#include "task.h"

#include "main.h"

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

#include <stdlib.h>   /* rand() / srand() — basic-mode beacon jitter */

/* STORAGE_BACKEND_FLASH comes from build_config.h (included above). */
#ifdef STORAGE_BACKEND_FLASH
#  include "Fota.h"
#  include "version_config.h"
#endif

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

    /* Seed rand() from the LoRa unique id so basic-mode beacon jitter
     * differs from one node to the next on the very first roll (same
     * boot moment, different id -> different sequence). Used only for
     * the basic-mode 5..15 s beacon spacing; no security implications. */
    srand((unsigned int)LORARADIO_u32GetUniqueId());

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

    /* Capture the reset cause — first clue in a field post-mortem. */
    uint32_t u32ModifiedCSR = u32GetCSR() >> 5;
    DBG_LOG("DeviceDiscovery: reset cause CSR=0x%lX\r\n", u32ModifiedCSR);
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
                                             DISCOVERY_WAKEUP_BIT | DISCOVERY_KERNEL_BIT | DISCOVERY_BASIC_BEACON_BIT,
                                             osFlagsWaitAny,
                                             osWaitForever);

        /* Multi-primary: arm the "first TimeSync only" gate for this
         * wake cycle. Subsequent TimeSyncs from other primaries during
         * this campaign will be forwarded but not re-applied. */
        MESHNETWORK_vResetTimeSyncAccepted();

        bool bKernelWakeup = ((u32Flags & osFlagsError) == 0U) &&
                             ((u32Flags & DISCOVERY_KERNEL_BIT) != 0U);
        bool bBasicBeaconWake = ((u32Flags & osFlagsError) == 0U) &&
                                ((u32Flags & DISCOVERY_BASIC_BEACON_BIT) != 0U);

        /* --- Basic-mode 10 s beacon-only wake (SECONDARY) ---
         * Very short TX-only path: bring the radio out of sleep (already
         * done by the wake-schedule task before it set our flag), TX one
         * BasicBeacon, put the radio back to sleep, release the sleep
         * lock. No campaign, no RX, no logger, no GPS. */
        if (bBasicBeaconWake)
        {
            HAL_UART_vInit();
            DEBUG_vInit();
            MESHNETWORK_vSendBasicBeacon();
            osDelay(200);            /* let the TX finish on the wire */
            LORARADIO_vEnterDeepSleep();
            SYSTEM_vSleepLockRelease();
            continue;
        }

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

        /* The primary's discovery campaign always runs — a staged image
         * pending distribution used to skip it entirely (an "OTA distribution
         * slot" that sent OtaPrep where the DReq would have gone), which
         * meant TimeSync (and the staged-fw-version it now carries — see
         * MESHNETWORK_vHandleTimeSync auto-arm) never went out on any wake
         * where a distribution was pending. Since that's every wake from the
         * moment this node finishes its own self-update until distribution
         * succeeds, it permanently starved secondaries of the one signal
         * that arms them, and every distribute attempt found "no targets
         * joined" forever. The campaign now always runs first; distribution
         * (if pending) is attempted right after this wake's own TimeSync,
         * so freshly-armed secondaries are still awake for it — see the
         * "wait for OtaPrep" window on the secondary side below. */
        if (eDeviceRole == DEVICE_ROLE_PRIMARY &&
            MESHNETWORK_eGetDiscoveryMode() == DISCOVERY_MODE_BASIC)
        {
            /* Basic-mode primary: passive 60 s RX window. No DReq campaign,
             * no beacon TX, no forwarding. Incoming BasicBeacons are merged
             * into the RAM store by the parser (MESHNETWORK_vHandleBasicBeacon).
             * The RX window runs every DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S
             * (15 min), independent of WakeupInterval; the accumulated store
             * is flushed to fr9 in the post-campaign block below only on
             * WakeupInterval boundaries. */
            DBG_LOG("DeviceDiscovery: Primary basic-mode listen (%u ms)\r\n",
                (unsigned)DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_MS);
            osDelay(DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_MS);
            DBG_LOG("DeviceDiscovery: basic listen end - %u unique nodes in RAM store\r\n",
                (unsigned)MESHNETWORK_u16GetBasicNeighborCount());
        }
        else if (eDeviceRole == DEVICE_ROLE_PRIMARY)
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
            /* Basic mode: this big-interval wake coincides with a primary
             * listen boundary. If WakeupInterval == the 15 min listen
             * period, EVERY primary listen lands on a secondary big wake —
             * so TX one beacon here too, otherwise the primary would never
             * hear this secondary (the surrounding TX-window beacons are
             * suppressed while this wake holds the sleep lock). Harmless in
             * the larger-interval case (just one extra beacon the primary
             * also hears). The RX wait below then catches the primary's
             * TimeSync for RTC + any mode/interval/gps change. */
            if (MESHNETWORK_eGetDiscoveryMode() == DISCOVERY_MODE_BASIC)
                MESHNETWORK_vSendBasicBeacon();

            DBG_LOG("DeviceDiscovery %X: Secondary waiting for timesync.\r\n",
                LORARADIO_u32GetUniqueId());

            osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC |
                               DEVICE_DISCOVERY_NOTIFY_OTA);

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
                uint32_t r = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_TIMESYNC |
                                               DEVICE_DISCOVERY_NOTIFY_OTA,
                                               osFlagsWaitAny,
                                               APP_SECONDARY_POLL_MS);
                if (!(r & osFlagsError))
                {
#ifdef STORAGE_BACKEND_FLASH
                    /* OTA prep announced instead of a DReq: this wake slot
                     * becomes a firmware-receive session. On a complete +
                     * verified image the call resets into the bootloader;
                     * otherwise fall through to the normal sleep path. */
                    if ((r & DEVICE_DISCOVERY_NOTIFY_OTA) && FOTA_bPrepPending())
                    {
                        FOTA_vSecondaryReceive();
                        break;
                    }
#endif
                    if (r & DEVICE_DISCOVERY_NOTIFY_TIMESYNC)
                    {
                        bTimeSync = true;
                        break;
                    }
                    continue;   /* stray OTA flag without a pending prep */
                }

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
            {
                DBG_LOG("DeviceDiscovery: Secondary %04X: TimeSync received\r\n",
                    LORARADIO_u32GetUniqueId());

#ifdef STORAGE_BACKEND_FLASH
                /* If armed (this wake's TimeSync auto-arm, or a still-armed
                 * leftover from a prior wake), the primary may follow this
                 * very TimeSync with a distribute session in the SAME wake
                 * (see the primary path above) — stay awake for a bounded
                 * window to catch that OtaPrep instead of immediately
                 * falling through to sleep, which is what silently starved
                 * every distribute attempt of any targets before. */
                if (FOTA_bAcceptanceArmed())
                {
                    DBG_LOG("DeviceDiscovery: Secondary %04X: armed - waiting up to %u ms for OtaPrep\r\n",
                        LORARADIO_u32GetUniqueId(), APP_OTA_PREP_WAIT_MS);

                    uint32_t u32WaitStart = osKernelGetTickCount();
                    while ((osKernelGetTickCount() - u32WaitStart) < APP_OTA_PREP_WAIT_MS)
                    {
                        uint32_t r2 = osThreadFlagsWait(DEVICE_DISCOVERY_NOTIFY_OTA,
                                                        osFlagsWaitAny, APP_SECONDARY_POLL_MS);
                        if (!(r2 & osFlagsError) && FOTA_bPrepPending())
                        {
                            FOTA_vSecondaryReceive();
                            break;
                        }
                    }
                }
#endif
            }
            else
                DBG_LOG("DeviceDiscovery: Secondary %04X: campaign end (silence/cap)\r\n",
                    LORARADIO_u32GetUniqueId());

            /* B4: stop OUR beacon dreq. The global u32DreqId is only ever
             * written on the primary — on a secondary it is stale/zero, so the
             * old exact-dreq stop was a silent no-op and the node kept
             * beaconing into the post-campaign delay (until the beacon cap). */
            MESHNETWORK_vStopBeaconingSelf();
        }

        /* ----------------------------------------------------------------
         * Discovery complete — log and upload (primary only)
         * ---------------------------------------------------------------- */
        DBG_LOG("DeviceDiscovery %X: Discovery complete.\r\n",
            LORARADIO_u32GetUniqueId());
        MESHNETWORK_vLogCampaignStats("campaign");
        EVTLOG(LOG_DISCOVERY_CMPLT, eDeviceRole);

        if (eDeviceRole == DEVICE_ROLE_PRIMARY &&
            MESHNETWORK_eGetDiscoveryMode() == DISCOVERY_MODE_BASIC)
        {
            /* Basic-mode flush schedule: the RAM store is uploaded to fr9
             * only at each WakeupInterval boundary. In between (basic
             * listen wakes on the 15 min BASIC_LISTEN_PERIOD_S cadence
             * that don't happen to land on a boundary), just keep the
             * store and sleep. Boundary detection: seconds-into-current-
             * interval < the listen period; anything larger means we're
             * partway between boundaries. */
            uint32_t u32Interval = (uint32_t)MESHNETWORK_u8GetWakeupInterval() * 60U;
            uint64_t u64Utc      = RTC_u64GetUTC();
            uint32_t u32Phase    = (uint32_t)(u64Utc % (uint64_t)u32Interval);
            bool bIsFlushBoundary = (u32Phase < DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S);

            if (!bIsFlushBoundary)
            {
                DBG_LOG("DeviceDiscovery: basic-mode listen-only wake (phase=%lus of %lus, no flush)\r\n",
                    (unsigned long)u32Phase, (unsigned long)u32Interval);
                osDelay(1000);
                /* Fall through: no logger connect, no upload, no clear —
                 * the store keeps accumulating until the next WakeupInterval
                 * boundary landing. */
            }
            else
            {
                MeshBasicNeighbor_t tBasic[MESH_MAX_BASIC_NEIGHBORS];
                uint16_t u16Count = 0U;
                MESHNETWORK_bGetBasicNeighbors(tBasic, MESH_MAX_BASIC_NEIGHBORS, &u16Count);
                DBG_LOG("DeviceDiscovery: basic-mode flush - uploading %u neighbors\r\n",
                    (unsigned)u16Count);
                for (uint16_t i = 0U; i < u16Count; i++)
                {
                    DBG_LOG("  ID:%08X  MsgId:%08X  Bat:%u  Move:%u  FwPatch:%u  Lat:%ld  Lon:%ld  AgeS:%lu\r\n",
                        tBasic[i].u32DeviceId,
                        tBasic[i].u32BeaconMsgId,
                        tBasic[i].u16BatMv,
                        tBasic[i].u8MoveState,
                        tBasic[i].u8FwPatch,
                        tBasic[i].bGpsValid ? (long)tBasic[i].i32LatUDeg : 0L,
                        tBasic[i].bGpsValid ? (long)tBasic[i].i32LonUDeg : 0L,
                        tBasic[i].bGpsValid ? (unsigned long)tBasic[i].u32GpsAgeS : 0UL);
                }

                DEVICE_DISCOVERY_DRIVER_bConnectLogger();
                DBG_LOG("DeviceDiscovery %X: Logger connected (basic-mode flush).\r\n",
                    LORARADIO_u32GetUniqueId());

                if (FARMRANGER_bLogBasicData(tBasic, u16Count))
                    DBG_LOG("DeviceDiscovery %X: Basic-mode log SUCCESS.\r\n",
                        LORARADIO_u32GetUniqueId());
                else
                    DBG_LOG("DeviceDiscovery %X: Basic-mode log FAILED.\r\n",
                        LORARADIO_u32GetUniqueId());

                /* Clear store now so the next 15-min window starts fresh -
                 * a partial upload followed by a re-heard node would
                 * otherwise be double-reported. */
                MESHNETWORK_vClearBasicNeighbors();

                /* Timestamp sync + settings fetch (same as advanced path). */
                uint64_t now = DEVICE_DISCOVERY_DRIVER_u64RequestTS();
                if (now > 0)
                    RTC_vSetUTC(now);
                uint8_t u8WI = 0U;
                bool    bBM  = false;
                bool    bGE  = true;
                if (DEVICE_DISCOVERY_DRIVER_bRequestSettings(&u8WI, &bBM, &bGE))
                {
                    if      (u8WI == 15)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_15_MIN);
                    else if (u8WI == 30)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_30_MIN);
                    else if (u8WI == 60)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_60_MIN);
                    else if (u8WI == 120) MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_120_MIN);
                    MESHNETWORK_vSetDiscoveryMode(bBM ? DISCOVERY_MODE_BASIC
                                                     : DISCOVERY_MODE_ADVANCED);
                    MESHNETWORK_vSetGpsEnabled(bGE);
                    DBG_LOG("DeviceDiscovery: settings applied - interval=%u mode=%s gps=%u\r\n",
                        (unsigned)u8WI, bBM ? "basic" : "advanced", (unsigned)bGE);
                }

                DEVICE_DISCOVERY_DRIVER_vDisconnectLogger();

                /* TimeSync out over LoRa — carries the (possibly just-
                 * updated) mode + gps flags so secondaries see any mode
                 * flip-back next time they open their big-interval RX
                 * window. */
                DEVICE_DISCOVERY_vSendTS();
                EVTLOG(LOG_TX_TS, 1);
            }
            osDelay(2000);
        }
        else if (eDeviceRole == DEVICE_ROLE_PRIMARY)
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
                    DBG_LOG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  Wave:%d  Move:%u  Lat:%ld  Lon:%ld  FwPatch:%u\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BatMv,
                        tNeighbors[i].u8Wave,
                        tNeighbors[i].u8MoveState,
                        tNeighbors[i].bGpsValid ? (long)tNeighbors[i].i32LatUDeg : 0L,
                        tNeighbors[i].bGpsValid ? (long)tNeighbors[i].i32LonUDeg : 0L,
                        tNeighbors[i].u8FwPatch);
                }
            }
            else
            {
                DBG_LOG("DeviceDiscovery %X: Error retrieving neighbor table.\r\n",
                    LORARADIO_u32GetUniqueId());
            }

            /* ---- Logger connection + upload (fr9 Farmranger board) ---- */
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

            /* ---- Settings update (interval + discovery mode + gps) ----
             * Single AT+SETREQ round trip returns all three; the local
             * defaults (whatever was last applied — cold boot: ADVANCED
             * + gps on) stay in force on any parse failure. Only wake
             * intervals in the small fixed set map to enum values; other
             * values leave the interval untouched. */
            uint8_t u8WakeInterval = 0U;
            bool    bBasicMode    = false;
            bool    bGpsEnabled   = true;
            if (DEVICE_DISCOVERY_DRIVER_bRequestSettings(&u8WakeInterval,
                                                        &bBasicMode,
                                                        &bGpsEnabled))
            {
                if      (u8WakeInterval == 15)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_15_MIN);
                else if (u8WakeInterval == 30)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_30_MIN);
                else if (u8WakeInterval == 60)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_60_MIN);
                else if (u8WakeInterval == 120) MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_120_MIN);

                MESHNETWORK_vSetDiscoveryMode(bBasicMode ? DISCOVERY_MODE_BASIC
                                                        : DISCOVERY_MODE_ADVANCED);
                MESHNETWORK_vSetGpsEnabled(bGpsEnabled);

                DBG_LOG("DeviceDiscovery: settings applied - interval=%u min, mode=%s, gps=%u\r\n",
                        (unsigned)u8WakeInterval,
                        bBasicMode ? "basic" : "advanced",
                        (unsigned)bGpsEnabled);
            }
            else
            {
                DBG_LOG("DeviceDiscovery: AT+SETREQ failed - keeping previous settings\r\n");
            }

            /* ---- Send TimeSync to secondaries ---- *
             * Sent BEFORE the OTA check so secondaries end their campaign
             * here rather than waiting out the primary's fr9 round-trip
             * (which, with the AT+FWCHECK GitHub Pages check, can now run
             * to OTA_FWREQ_WAIT_MAX_MS). Only the primary talks to fr9 at
             * all — secondaries have no Farmranger UART link. */
            DEVICE_DISCOVERY_vSendTS();
            EVTLOG(LOG_TX_TS, 1);

#ifdef STORAGE_BACKEND_FLASH
            /* ---- LoRa distribution to secondaries (if a staged image is
             * ready) ----
             * Right after TimeSync, before the fr9 UART check: secondaries
             * that just auto-armed off this wake's TimeSync are still awake
             * for a bounded window waiting for exactly this OtaPrep (see
             * APP_OTA_PREP_WAIT_MS on the secondary side) — putting the fr9
             * round trip (up to OTA_FWREQ_WAIT_MAX_MS) first would burn
             * through that window for nothing. */
            if (FOTA_bDistributePending())
            {
                FOTA_vDistribute();
            }
            else
            {
                /* Silent skip here used to be indistinguishable in the log
                 * from "distribute never even got checked" — log exactly
                 * why nothing happened instead of just falling through to
                 * the fr9 check with no trace. Only two reasons left to
                 * skip: FOTA_bDistributePending() no longer treats "every
                 * target from a past session confirmed" as a permanent
                 * stop, since that has no way to know about a secondary
                 * that wasn't listening then — see there for why. */
                FotaMeta_t tFotaMeta;
                if (!FOTA_bGetMeta(&tFotaMeta))
                    DBG_LOG("Fota: distribute check - no valid image staged\r\n");
                else if (tFotaMeta.u32Version != VERSION_u32Get())
                    DBG_LOG("Fota: distribute check - staged v%lu != running v%lu, skip\r\n",
                            (unsigned long)tFotaMeta.u32Version, (unsigned long)VERSION_u32Get());
            }

            /* ---- OTA firmware pull (logger session still up) ----
             * Ask fr9 to check GitHub Pages for a newer image, then poll
             * AT+FWREQ (which answers FW,WAIT while fr9's check/download is
             * in flight). If the logger offers a newer tag firmware, acquire
             * it into the ext-flash scratchpad. On success this arms the
             * bootloader and RESETS — nothing after it runs this wake. */
            FOTA_bUartAcquire();
#endif

            DEVICE_DISCOVERY_DRIVER_vDisconnectLogger();

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
         * The user must issue "tag release" (or 5-min inactivity auto-releases).
         *
         * This loop is also the OTA-over-kernel-session rendezvous:
         *   - Secondary armed with "tag <ID> fwaccept" live-listens here for an
         *     OtaPrep and runs the receive in-place (resets into the bootloader
         *     on a verified image).
         *   - Primary asked with "tag <ID> fwdistribute" (its session kept the
         *     device awake past the campaign) distributes the staged image from
         *     ext flash here.
         * The session's s_bConnected is what parked the AppTask in this loop in
         * the first place, so no extra wake plumbing is needed. */
        if (FRKERNEL_bIsConnected())
        {
            DBG("DeviceDiscovery: FrKernel session active — waiting for release...\r\n");
            while (FRKERNEL_bIsConnected())
            {
#ifdef STORAGE_BACKEND_FLASH
                if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
                    FOTA_bAcceptanceArmed() && FOTA_bPrepPending())
                {
                    /* Verified image -> resets into the bootloader (no return);
                     * failure returns here and the session keeps holding. */
                    FOTA_vSecondaryReceive();
                }
                else if (eDeviceRole == DEVICE_ROLE_PRIMARY &&
                         FOTA_bDistributeRequested())
                {
                    FOTA_vClearDistributeRequest();
                    FOTA_vDistribute();
                }
#endif
                osDelay(500);
            }
        }

        DBG_LOG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
        osDelay(100);
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

    /* B6: slot latches. The old exact `phase == 0` compare missed a wake
     * whenever the RTC was stepped FORWARD across a boundary (TimeSync /
     * logger sync) and double-fired when stepped BACKWARD across one.
     * Fire once per interval slot (utc / interval), strictly increasing:
     *   - slot > lastFired          -> fire and latch (catches forward jumps
     *                                  and ordinary boundaries, even if the
     *                                  exact boundary second was missed)
     *   - slot == lastFired         -> already ran this slot (backward step
     *                                  re-entering the previous slot: no
     *                                  double campaign)
     *   - slot << lastFired (> 1)   -> large backward correction / interval
     *                                  change: resync the latch, no fire
     * Armed (not fired) on the first heartbeat so boot behaviour matches the
     * old code: first campaign at the next boundary. */
    static uint64_t u64LastFiredSlot = 0;
    static bool     bSlotValid       = false;
    static uint64_t u64LastGpsSlot   = 0;
    static bool     bGpsSlotValid    = false;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
        /* Block until the platform heartbeat fires (any flag) */
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        uint64_t u64Utc       = RTC_u64GetUTC();
        uint32_t u32IntervalS = (uint32_t)MESHNETWORK_u8GetWakeupInterval() * 60U;

        /* Effective wake cadence for the "slot fired" latch below:
         *   PRIMARY in basic mode uses a fixed 15 min BASIC_LISTEN_PERIOD_S
         *     (independent of WakeupInterval, per plan) so it can accumulate
         *     basic beacons more often than the flush schedule.
         *   Everyone else uses the full WakeupInterval (15/30/60/120 min).
         * Secondary in basic mode still uses WakeupInterval for its big
         * wake — the 10 s beacon TX is a separate schedule handled
         * below via DISCOVERY_BASIC_BEACON_BIT. */
        uint32_t u32EffectiveIntervalS = u32IntervalS;
        if (eDeviceRole == DEVICE_ROLE_PRIMARY &&
            MESHNETWORK_eGetDiscoveryMode() == DISCOVERY_MODE_BASIC)
        {
            u32EffectiveIntervalS = DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S;
        }
        uint64_t u64Slot      = u64Utc / (uint64_t)u32EffectiveIntervalS;

        /* ---- ProductionSleep: secondary only — primary has no solar panel ---- */
        if (eDeviceRole == DEVICE_ROLE_SECONDARY && eProductionState == PRODUCTION_SLEEP)
        {
#ifdef ENABLE_SOLAR_POWER_SENSE
            if (SOLAR_u32GetPowerMW() >= SOLAR_ACTIVATION_POWER_MW)
            {
                eProductionState = PRODUCTION_ACTIVE;

                /* Hold off deep sleep until the resulting kernel window
                 * completes (released at the end of
                 * DEVICE_DISCOVERY_vAppTask's loop). Re-arm the debug UART
                 * BEFORE logging so the activation is observable live —
                 * STOP2 parked its pins analog and this branch is otherwise
                 * the only wake path that forgets to restore them. */
                SYSTEM_vSleepLockAcquire();
                HAL_UART_vInit();
                DEBUG_vInit();

                DBG_LOG("DeviceDiscovery: Solar activation (%lu mW) — exiting ProductionSleep\r\n",
                    SOLAR_u32GetPowerMW());

                /* Fire a GPS fix immediately on takeoff, not just at the next
                 * scheduled 3-min pre-trigger: a device that just spent an
                 * unbounded stretch in ProductionSleep (no RTC-correcting
                 * TimeSync while asleep) is exactly the case most likely to
                 * have a drifted RTC, so get it corrected as early as
                 * possible instead of waiting. Own dedicated sleep-lock
                 * reference (released inside GPS_vPowerOff()), independent
                 * of the kernel-window lock acquired above (released
                 * separately at the AppTask loop end) — see the GPS
                 * pre-trigger below for the same acquire/POWER_CLASS_NORMAL
                 * pattern. */
                if (POWER_tGetState() & POWER_CLASS_NORMAL)
                {
                    SYSTEM_vSleepLockAcquire();
                    DBG_LOG("DeviceDiscovery: GPS acquire on ProductionSleep wake (5 min timeout)\r\n");
                    GPS_vRequestFix(true, 300);
                }

                /* Open a FrKernel window instead of an autonomous discovery
                 * campaign (same path the shake-wakeup trigger uses); the
                 * kernel's own 5-min inactivity timeout — or an explicit
                 * "tag release" — is what returns the device to normal
                 * scheduled-wake sleep from here. */
                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_KERNEL_BIT);
            }
#else
            /* Panel power sensing is disabled (broken RSENSE front-end); gate
             * ProductionSleep exit on panel VOLTAGE instead. */
            if (SOLAR_u16GetVSolarMV() >= SOLAR_ACTIVATION_VSOLAR_MV)
            {
                eProductionState = PRODUCTION_ACTIVE;

                /* See the ENABLE_SOLAR_POWER_SENSE branch above for why the
                 * UART is re-armed here and why this opens a kernel window
                 * rather than a discovery campaign. */
                SYSTEM_vSleepLockAcquire();
                HAL_UART_vInit();
                DEBUG_vInit();

                DBG_LOG("DeviceDiscovery: Solar activation (%u mV) — exiting ProductionSleep\r\n",
                    SOLAR_u16GetVSolarMV());

                /* See the ENABLE_SOLAR_POWER_SENSE branch above for why this
                 * fires a GPS fix immediately (RTC correction on takeoff)
                 * with its own dedicated sleep-lock reference. */
                if (POWER_tGetState() & POWER_CLASS_NORMAL)
                {
                    SYSTEM_vSleepLockAcquire();
                    DBG_LOG("DeviceDiscovery: GPS acquire on ProductionSleep wake (5 min timeout)\r\n");
                    GPS_vRequestFix(true, 300);
                }

                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_KERNEL_BIT);
            }
#endif
            continue;
        }

        /* --- Discovery wake trigger: once per interval slot (B6) --- */
        bool bFireDiscovery = false;
        if (!bSlotValid)
        {
            u64LastFiredSlot = u64Slot;          /* boot: arm, don't fire */
            bSlotValid       = true;
        }
        else if (u64Slot > u64LastFiredSlot + 1ULL)
        {
            /* Big forward jump. In practice this is a clock resync -- typically
             * the AT+TSREQ that ended the first campaign after a boot with no
             * RTC backup (fake epoch small u64Utc -> real ~1.7e9). Without
             * this, the wake immediately after the sync would fire a whole
             * second campaign against a fr9 that JUST finished processing
             * the first one, so we'd get two "Total devices discovered" log
             * entries a few seconds apart even though the interval is minutes.
             * Just resync the latch and skip the fire -- the next real slot
             * boundary is the one worth waking for. */
            u64LastFiredSlot = u64Slot;
        }
        else if (u64Slot > u64LastFiredSlot)
        {
            u64LastFiredSlot = u64Slot;
            bFireDiscovery   = true;
        }
        else if ((u64LastFiredSlot - u64Slot) > 1ULL)
        {
            u64LastFiredSlot = u64Slot;          /* big backward correction: resync */
        }

        if (bFireDiscovery)
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

                /* Clear any node role left over from off-schedule mesh activity
                 * (e.g. a DReq from a primary's catch-up campaign that started
                 * this node beaconing between scheduled wakes). Done before the
                 * radio comes up so this campaign begins from a clean UNKNOWN
                 * role and beacons the current dreq rather than a stale one. */
                MESHNETWORK_vResetNodeRole();

                LORARADIO_vWakeUp();

                DBG_LOG("\r\n--- WAKEUP ---\r\n");

                /* One solar reading per production wake instead of every 10 s
                 * (see LOG_SOLAR_PERIODIC in SolarPower_Config.h). Primary has
                 * no solar panel. */
                if (eDeviceRole == DEVICE_ROLE_SECONDARY)
                {
#ifdef ENABLE_SOLAR_POWER_SENSE
                    DBG_LOG("solar: Vsolar=%u mV  P=%lu mW\r\n",
                        SOLAR_u16GetVSolarMV(), SOLAR_u32GetPowerMW());
#else
                    DBG_LOG("solar: Vsolar=%u mV\r\n", SOLAR_u16GetVSolarMV());
#endif
                }

                osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_WAKEUP_BIT);

            }
        }

        /* --- Basic-mode jittered beacon TX window (SECONDARY only) ---
         * When in DISCOVERY_MODE_BASIC, TX beacons ONLY during the window
         * straddling the primary's 15 min listen: from TX_GUARD_S before
         * the boundary through the 60 s listen + TX_GUARD_S after. Outside
         * that window the radio stays asleep (the ~10x efficiency win over
         * the old always-on 10 s cadence). Within it, beacons are jittered
         * ~5..10 s apart so multiple secondaries don't collide and the
         * primary gets several chances to hear each one.
         *
         * Phase is against BASIC_LISTEN_PERIOD_S (the primary's fixed
         * listen cadence), NOT this secondary's WakeupInterval.
         *
         * Two guards, both required:
         *  - !bFireDiscovery: don't also fire on the exact boundary
         *    heartbeat, which is a big-interval RX wake (that path TXes
         *    its own beacon at the start — see the AppTask secondary
         *    branch — so the primary still hears us even when
         *    WakeupInterval == the 15 min listen period).
         *  - SYSTEM_bCheckSleepModeStatus(): only fire when otherwise
         *    idle (no sleep lock held). Without this, a beacon fired while
         *    a big wake / GPS / kernel session already holds the lock would
         *    acquire a second lock that the one-shot AppTask handler never
         *    balances -> gSleepLockCount leaks and the device never sleeps. */
        static uint64_t u64NextBasicBeaconUtc = 0ULL;
        if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
            MESHNETWORK_eGetDiscoveryMode() == DISCOVERY_MODE_BASIC &&
            !bFireDiscovery &&
            SYSTEM_bCheckSleepModeStatus())
        {
            uint32_t u32Phase = (uint32_t)(u64Utc % (uint64_t)DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S);
            bool bInTxWindow =
                (u32Phase >= (DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S - DEVICE_DISCOVERY_BASIC_TX_GUARD_S)) ||
                (u32Phase <= (DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_S + DEVICE_DISCOVERY_BASIC_TX_GUARD_S));

            if (bInTxWindow)
            {
                if (u64NextBasicBeaconUtc == 0ULL || u64Utc >= u64NextBasicBeaconUtc)
                {
                    /* 5..10 s uniform. rand() is seeded per device from its
                     * unique id (DEVICE_DISCOVERY_vInit), so two secondaries
                     * pick different offsets. */
                    uint32_t u32NextDelayS = 5U + (uint32_t)(rand() % 6);
                    u64NextBasicBeaconUtc = u64Utc + (uint64_t)u32NextDelayS;

                    if (POWER_tGetState() & (POWER_CLASS_NORMAL | POWER_CLASS_LOW))
                    {
                        SYSTEM_vSleepLockAcquire();
                        LORARADIO_vWakeUp();
                        osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_BASIC_BEACON_BIT);
                    }
                }
            }
            else
            {
                /* Outside the TX window — arm so the first beacon of the
                 * next window fires immediately on entry rather than
                 * waiting out a stale (pre-window) target time. */
                u64NextBasicBeaconUtc = 0ULL;
            }
        }
        else if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
                 MESHNETWORK_eGetDiscoveryMode() != DISCOVERY_MODE_BASIC)
        {
            /* Reset on flip-back to advanced so a later flip to basic
             * starts fresh rather than firing on a stale target. */
            u64NextBasicBeaconUtc = 0ULL;
        }

        /* --- GPS pre-trigger (SECONDARY only) ---
         * GPS is not fitted on PRIMARY boards. Fire-and-forget 3 minutes
         * before each scheduled wake so a fresh fix is cached by the time
         * the AppTask runs. The AppTask never blocks on a GPS result.
         *
         * B6: slot-latched like the discovery trigger — the old exact
         * `phase == interval - 180` compare skipped the pre-trigger whenever
         * that one heartbeat second was missed. Fire once inside the last
         * PRETRIGGER window of each interval; the slot being pre-triggered is
         * the UPCOMING one ((utc + lead) / interval). */
        bool bFireGps = false;
        if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
            u32IntervalS > DEVICE_DISCOVERY_GPS_PRETRIGGER_S)
        {
            uint32_t u32Phase   = (uint32_t)(u64Utc % (uint64_t)u32IntervalS);
            uint64_t u64GpsSlot = (u64Utc + DEVICE_DISCOVERY_GPS_PRETRIGGER_S)
                                  / (uint64_t)u32IntervalS;
            if (!bGpsSlotValid)
            {
                u64LastGpsSlot = u64GpsSlot;     /* boot: arm, don't fire */
                bGpsSlotValid  = true;
            }
            else if (u64GpsSlot > u64LastGpsSlot + 1ULL)
            {
                /* Big forward jump -- clock resync; see the discovery slot
                 * latch above for full reasoning. Just resync, don't fire. */
                u64LastGpsSlot = u64GpsSlot;
            }
            else if (u32Phase >= (u32IntervalS - DEVICE_DISCOVERY_GPS_PRETRIGGER_S) &&
                     u64GpsSlot > u64LastGpsSlot)
            {
                u64LastGpsSlot = u64GpsSlot;
                bFireGps       = true;
            }
            else if (u64LastGpsSlot > u64GpsSlot + 1ULL)
            {
                u64LastGpsSlot = u64GpsSlot;     /* big backward correction: resync */
            }
        }

        if (bFireGps && (POWER_tGetState() & POWER_CLASS_NORMAL))
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
            GPS_vRequestFix(true, 120);
        }
    }
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vSendTS
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vSendTS(void)
{
    DBG_LOG("\r\n--- START TIMESYNC ---\r\n");

    /* Advertise whatever image is currently staged in ext flash (0 if none
     * valid) so every secondary that hears this campaign's TimeSync learns
     * an update exists and can auto-arm itself — see
     * MESHNETWORK_vHandleTimeSync. This is the same image
     * FOTA_vDistribute() will later broadcast on its own dedicated wake
     * slot; announcing it here costs 4 bytes and one flash-metadata read,
     * no dedicated round trip. */
    uint32_t u32StagedVer = 0U;
#ifdef STORAGE_BACKEND_FLASH
    FotaMeta_t tMeta;
    if (FOTA_bGetMeta(&tMeta) && tMeta.bValid)
        u32StagedVer = tMeta.u32Version;
#endif

    MESHNETWORK_vSendTimeSync(RTC_u64GetUTC(), MESHNETWORK_tGetWakeupInterval(),
                               u32StagedVer,
                               MESHNETWORK_eGetDiscoveryMode(),
                               MESHNETWORK_bGetGpsEnabled());
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vTriggerKernelWakeup
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vTriggerKernelWakeup(void)
{
    /* Hold off deep sleep until the resulting (no-op) campaign completes —
     * released at the end of DEVICE_DISCOVERY_vAppTask's loop.
     *
     * Both the debug UART and the radio are re-armed unconditionally here,
     * matching the scheduled-wake path below — not one or the other picked
     * by FRKERNEL_INTERFACE_LORA vs UART. DBG_LOG visibility has nothing to
     * do with which transport FrKernel commands arrive on: a LORA-interface
     * secondary still needs its debug UART alive for bench observation, and
     * the radio must come out of its between-campaigns deep sleep on EVERY
     * kernel wakeup regardless of interface, or a LoRa-delivered command
     * (e.g. a broadcast "tag -devicereq") arrives at a radio that's still
     * off and is never received. */
    SYSTEM_vSleepLockAcquire();
    HAL_UART_vInit();
    DEBUG_vInit();
    LORARADIO_vWakeUp();
    DBG("\r\n--- KERNEL WAKEUP ---\r\n");

    /* This is the single entry point for shake-triggered wakeups (only ever
     * called from the secondary-only shake-sequence in Movement.c, same
     * role restriction as ProductionSleep — no role guard needed here). A
     * shake already opens a kernel session regardless of eProductionState,
     * but without this it never leaves ProductionSleep: once the session
     * ends the device would fall back to waiting for Vsolar instead of
     * resuming normal scheduled discovery. Mirrors the solar wake path. */
    if (eProductionState == PRODUCTION_SLEEP)
    {
        eProductionState = PRODUCTION_ACTIVE;
        DBG_LOG("DeviceDiscovery: Kernel wakeup — exiting ProductionSleep\r\n");

        /* Same rationale as the solar wake path: get the RTC corrected as
         * early as possible after an unbounded ProductionSleep stretch,
         * not just at the next scheduled pre-trigger. Own dedicated
         * sleep-lock reference, released inside GPS_vPowerOff(). */
        if (POWER_tGetState() & POWER_CLASS_NORMAL)
        {
            SYSTEM_vSleepLockAcquire();
            DBG_LOG("DeviceDiscovery: GPS acquire on ProductionSleep wake (5 min timeout)\r\n");
            GPS_vRequestFix(true, 300);
        }
    }

    osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_KERNEL_BIT);
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vConfigDeviceRole
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vConfigDeviceRole(void)
{
    /* The role strap was set up as input-pulldown by HAL_GPIO_vInit(); read it
     * once here. After this the role is fixed for the life of the device, so the
     * pin is no longer needed — tristate it (analog, no pull) so it draws no
     * current (a PRIMARY ties the strap HIGH; leaving the pulldown active would
     * sink ~VDD/R_pd continuously) and is never read again. */
    eDeviceRole = HAL_GPIO_ReadPin(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN)
                  ? DEVICE_ROLE_PRIMARY
                  : DEVICE_ROLE_SECONDARY;
    HAL_GPIO_vInitAnalogNoPull(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN);
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
