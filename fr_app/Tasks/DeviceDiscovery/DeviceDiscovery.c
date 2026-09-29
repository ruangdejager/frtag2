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
#include "RadioTestMode.h"   /* R&D radio link/range test gates */
#include "LoRaWan.h"

#include <stdlib.h>   /* srand() */

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

/* SolarSleep variant of PRODUCTION_SLEEP: identical super-deep-sleep state
 * (same yellow STOP2 indicator, same suppressed discovery/acc logging), but
 * a rising panel voltage must NOT wake it — only the shake sequence may.
 * That lets a flat unit sit with the whole system disabled and take a full
 * charge off the panel instead of waking the moment the sun hits it.
 *
 * Only meaningful while eProductionState == PRODUCTION_SLEEP. The schedule
 * task skips its solar-activation check entirely while this is set, so the
 * two solar exit paths cannot run; the kernel (shake) wake path is the only
 * reachable exit and clears it. */
static volatile bool bSolarWakeDisabled = false;

/* True from the moment the AppTask picks up an event until it is back
 * waiting for the next one. The wake-schedule task will not start a LoRaWAN
 * report while this is set: the AppTask is the one that would run it, and
 * a wake or kernel session it is still in the middle of may own the radio. */
static volatile bool bAppTaskBusy = false;

/* LoRaWAN mode: UTC second by which the pending report must have STARTED
 * (the uplink slot's boundary minus LORAWAN_UPLINK_DEADLINE_LEAD_S). Set by
 * the wake-schedule task with DISCOVERY_LORAWAN_UPLINK_BIT; the AppTask
 * drops a report it only gets to after this, since by then the mesh wake is
 * about to take the radio back. 32-bit on purpose: written by one task and
 * read by another, and a uint64_t is two separate stores on this core. UTC
 * seconds fit in 32 bits until 2106. */
static volatile uint32_t u32LoRaWanDeadlineUtc = 0U;

/* ---- Forward declarations ---- */
#ifdef ENABLE_LOW_POWER_RECOVERY
static void DEVICE_DISCOVERY_vRecoveryMode(void);
#endif
static void DEVICE_DISCOVERY_vEnterSleepMode(bool bAllowSolarWake);
static void DEVICE_DISCOVERY_vSendTS(void);
static void DEVICE_DISCOVERY_vCheckWakeupScheduleTask(void *pvParameters);

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vInit
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vInit(void)
{
    xDiscoveryEventFlags = osEventFlagsNew(NULL);
    configASSERT(xDiscoveryEventFlags != NULL);

    /* Seed rand() from the LoRa unique id so jitter drawn from it differs
     * from one node to the next (same boot moment, different id -> different
     * sequence). No security implications. */
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
    {
        /* Logged at every boot, whatever the mode: it is what a tag has to be
         * registered in TTN under before LoRaWAN mode can do anything. */
        uint8_t au8Eui[8];
        LORAWAN_vGetDevEui(au8Eui);
        DBG_LOG("DeviceDiscovery: Device Role = SECONDARY, LoRaWAN DevEUI=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
            au8Eui[0], au8Eui[1], au8Eui[2], au8Eui[3],
            au8Eui[4], au8Eui[5], au8Eui[6], au8Eui[7]);
    }
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
        bAppTaskBusy = false;
        uint32_t u32Flags = osEventFlagsWait(xDiscoveryEventFlags,
                                             DISCOVERY_WAKEUP_BIT | DISCOVERY_KERNEL_BIT | DISCOVERY_LORAWAN_UPLINK_BIT,
                                             osFlagsWaitAny,
                                             osWaitForever);
        bAppTaskBusy = true;

        /* --- LoRaWAN-mode report (SECONDARY) ---
         * One LoRaWAN session between two mesh wakes: attach the radio to the
         * LoRaWAN MAC, join or send the report, hand the radio back to the
         * mesh asleep, release the lock the schedule task took. No campaign,
         * no mesh RX. Handled ahead of everything below because it shares
         * nothing with a wake. */
        bool bLoRaWanReport = ((u32Flags & osFlagsError) == 0U) &&
                              ((u32Flags & DISCOVERY_LORAWAN_UPLINK_BIT) != 0U);
        if (bLoRaWanReport &&
            ((u32Flags & (DISCOVERY_WAKEUP_BIT | DISCOVERY_KERNEL_BIT)) == 0U))
        {
            HAL_UART_vInit();
            DEBUG_vInit();

            uint64_t u64Now      = RTC_u64GetUTC();
            uint64_t u64Deadline = (uint64_t)u32LoRaWanDeadlineUtc;
            if (u64Now > u64Deadline)
            {
                DBG_LOG("DeviceDiscovery: LoRaWAN report dropped - %lu s past its slot deadline\r\n",
                    (unsigned long)(u64Now - u64Deadline));
            }
            else if (RADIOTESTMODE_bActive() || FRKERNEL_bIsConnected())
            {
                DBG_LOG("DeviceDiscovery: LoRaWAN report skipped - radio in use\r\n");
            }
            else
            {
                /* Everything up to 5 s before the mesh wake is ours; the
                 * session caps itself at LORAWAN_SESSION_MAX_MS anyway. */
                uint32_t u32BudgetMs =
                    (uint32_t)(u64Deadline - u64Now) * 1000U +
                    (LORAWAN_UPLINK_DEADLINE_LEAD_S - 5U) * 1000U;
                (void)LORAWAN_bSendReport(u32BudgetMs);
            }

            SYSTEM_vSleepLockRelease();
            continue;
        }
        if (bLoRaWanReport)
        {
            /* The same wait also returned a wake or kernel event - one
             * osEventFlagsWait consumes every flag it reports. The wake owns
             * the radio from here, so the report is dropped; its sleep lock
             * still has to be given back. */
            DBG_LOG("DeviceDiscovery: LoRaWAN report dropped - coincided with a wake\r\n");
            SYSTEM_vSleepLockRelease();
        }

        /* Multi-primary: arm the "first TimeSync only" gate for this
         * wake cycle. Subsequent TimeSyncs from other primaries during
         * this campaign will be forwarded but not re-applied. */
        MESHNETWORK_vResetTimeSyncAccepted();

        bool bKernelWakeup = ((u32Flags & osFlagsError) == 0U) &&
                             ((u32Flags & DISCOVERY_KERNEL_BIT) != 0U);

        /* A radio test owns the radio AND the Farmranger link for its whole
         * duration, so a campaign started now can only do harm: its DReq will
         * not transmit (MESHNETWORK_bSendPacket refuses while a test runs), so
         * every wave times out hearing nothing, and the logger round-trip at
         * the end then fights the test for the same fr9 session and PA0
         * attention line. Two owners toggling that line produced exactly the
         * observed symptom on the fr9 - a burst of sessions at rx=0B, one
         * cancelling the other, with the occasional stray byte from a transfer
         * cut off when the other side de-inits the UART pins mid-character.
         *
         * The scheduler is already gated, so this only catches a wake whose
         * flag was set just before the test was entered. */
        if (RADIOTESTMODE_bActive())
        {
            DBG_LOG("DeviceDiscovery: radio test active - skipping campaign\r\n");
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
        MESHNETWORK_vFlushTxQueue(false);
#ifdef STORAGE_BACKEND_FLASH
        /* Likewise drop an OtaPrep latched last campaign but never serviced —
         * left set, it makes the session-id latch reject this campaign's Prep
         * and the unit stops accepting firmware entirely. */
        FOTA_vDropStalePrep();
#endif

        DBG_LOG("DeviceDiscovery %X: Woke up for discovery.\r\n",
            LORARADIO_u32GetUniqueId());

        osDelay(APP_WAKEUP_BUFFER_MS);
        EVTLOG(LOG_DISCOVERY_START, eDeviceRole);

        /* The mode this wake runs under. Read once: the primary's own
         * AT+SETREQ below can change it mid-wake, and a secondary's TimeSync
         * can too, but what this wake does - campaign or not, upload or not -
         * follows the mode it woke into. */
        const DiscoveryMode_e eWakeMode = MESHNETWORK_eGetDiscoveryMode();

        /* Wall time for the whole discovery, wake to "complete" below — not to
         * be confused with either role branch's own internal u32CampaignStart
         * (those exist purely to enforce that branch's own deadline). This one
         * spans everything: wave loop or TimeSync wait, whichever role
         * runs. Read again once we reach "Discovery complete" and handed to
         * the fr9 in the AT+BLOG header, so the server finally knows how long
         * a round actually took instead of estimating it from log arrival
         * times. */
        uint32_t u32CampaignStartMs = osKernelGetTickCount();

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
            eWakeMode == DISCOVERY_MODE_LORAWAN)
        {
            /* LoRaWAN-mode primary: no campaign. The tags report straight to
             * TTN; this wake exists only for the fr9 round trip below (time,
             * settings) and the TimeSync + firmware distribution that follow
             * it, so go straight there. */
            DBG_LOG("DeviceDiscovery: Primary LoRaWAN mode - no campaign, TimeSync only\r\n");
        }
        else if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        {
            bool     bDiscoveryFinished = false;
            uint8_t  u8WaveCount        = 0;
            bool     bDeadlineHit       = false;
            /* Two barren waves in a row end the campaign; one does not — a lone
             * quiet wave at the frontier is as often a late/collided deep answer
             * as an empty ring. See the wave-end branch below. */
            bool     bPrevWaveBarren    = false;
            uint32_t u32CampaignStart   = osKernelGetTickCount();
            DBG_LOG("DeviceDiscovery: Primary starting discovery campaign\r\n");
            /* The constants that shaped this campaign, recorded once so a
             * downloaded log can be tied to the build that produced it. Every
             * number below has been retuned at least once during this work;
             * without them a campaign's timings cannot be checked against the
             * rules that were meant to be in force. */
            DBG_LOG("DeviceDiscovery: campaign cfg v%u.%u.%u waves=%u..%u floor=%u+%u/ring cap=%u idle=%u hold=%u deadline=%u ack=%u/%u jitter=%u..%u\r\n",
                (unsigned)VERSION_SW_MAJOR, (unsigned)VERSION_SW_MINOR,
                (unsigned)VERSION_SW_PATCH,
                (unsigned)APP_PRIMARY_MIN_WAVES, (unsigned)APP_PRIMARY_MAX_WAVES,
                (unsigned)MESH_DISCOVERY_MIN_WAVE_MS,
                (unsigned)MESH_DISCOVERY_WAVE_ALLOWANCE_MS,
                (unsigned)MESH_DISCOVERY_MIN_WAVE_CAP_MS,
                (unsigned)MESH_DISCOVERY_IDLE_MS,
                (unsigned)MESH_DISCOVERY_UNACKED_HOLD_MS,
                (unsigned)APP_PRIMARY_CAMPAIGN_MAX_MS,
                (unsigned)MESH_PRIMARY_ACK_INTERVAL_MS,
                (unsigned)MESH_MAX_ACK_IDS_PER_PACKET,
                (unsigned)MESH_TX_JITTER_MAX_MS, (unsigned)MESH_TX_JITTER_BUSY_MS);

            while (!bDiscoveryFinished)
            {
                u32DreqId = MESHNETWORK_u32GenerateGlobalMsgID();
                bool bBeaconSeenThisWave = false;

                MESHNETWORK_vIncrDreqWaveCnt();
                /* Return value now checked. A DReq that never reached the
                 * queue (encode failure, queue full, radio test) used to leave
                 * the primary listening out a whole idle window for a wave it
                 * had not actually asked for - and MESHNETWORK_vStartPrimaryAck
                 * lives past those failure returns, so that wave sent no D-Ack
                 * either. txDrop is non-zero in the field, so this is not
                 * hypothetical. One retry, then give up on the campaign rather
                 * than burn the rest of the waves silently. */
                if (!MESHNETWORK_bStartDiscoveryRound(u32DreqId))
                {
                    DBG_LOG("DeviceDiscovery: Primary DReq %08X not queued - retrying\r\n",
                        (unsigned)u32DreqId);
                    osDelay(500);
                    u32DreqId = MESHNETWORK_u32GenerateGlobalMsgID();
                    if (!MESHNETWORK_bStartDiscoveryRound(u32DreqId))
                    {
                        DBG_LOG("DeviceDiscovery: Primary cannot send DReq - ending campaign\r\n");
                        MESHNETWORK_vStopPrimaryAck();
                        break;
                    }
                }
                u8WaveCount++;

                uint32_t tLastBeaconTick = MESHNETWORK_u32GetLastBeaconHeardTick();
                uint32_t u32WaveStart    = osKernelGetTickCount();
                /* "Is this wave still finding anyone?" - the signal the
                 * wave-end test below needs, and the one beacon silence stops
                 * being a proxy for as the herd grows. Tracked on the stack, so
                 * no new statics. */
                uint16_t u16PrevCount    = MESHNETWORK_u16GetNeighborCount();
                uint32_t u32LastNewTick  = u32WaveStart;
                /* Wave-start baselines for the per-wave log line below. */
                uint16_t u16WaveUnion0   = u16PrevCount;
                uint16_t u16WaveBeacons0 = MESHNETWORK_u16GetBeaconsHeard();
                /* Which rule ended this wave. Recorded at the break rather than
                 * inferred afterwards - telling QUIET from STALE, or either
                 * from the hold expiring, is not recoverable from timestamps,
                 * and it was the thing most missing when reviewing the last
                 * set of field logs. */
                const char *pcWaveEnd    = "?";
                uint32_t u32WaveFloorAtEnd = 0U;

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

                    uint32_t u32WaveMs = tNow - u32WaveStart;

                    uint16_t u16NowCount = MESHNETWORK_u16GetNeighborCount();
                    if (u16NowCount != u16PrevCount)
                    {
                        u16PrevCount   = u16NowCount;
                        u32LastNewTick = tNow;
                    }

                    /* The campaign deadline is the ONE hard stop, and it is
                     * checked here rather than only between waves because a
                     * wave can legitimately outlast any per-wave bound while
                     * the herd is still answering it. Every exit from this loop
                     * has to leave the ack timer stopped - it is periodic, and
                     * one left running keeps queueing D-Acks through the fr9
                     * session and into the FOTA chunk stream.
                     *
                     * There used to be a flat per-wave ceiling here too, which
                     * broke out without consulting tLastBeaconTick at all - so
                     * a wave still being answered was cut off and the next DReq
                     * went out over the top of it. See
                     * MESH_DISCOVERY_UNACKED_HOLD_MS for why that was worse than
                     * one lost beacon: it re-anchors every mid-cadence node and
                     * resets the whole herd's beacon backoff at once. */
                    if ((tNow - u32CampaignStart) >= APP_PRIMARY_CAMPAIGN_MAX_MS)
                    {
                        bDeadlineHit = true;
                        pcWaveEnd    = "DEADLINE";
                        break;
                    }

                    /* Listen floor for THIS wave: the flat base plus an
                     * allowance for every ring of depth the herd has already
                     * proven. A wave is looking for the ring one deeper than
                     * the deepest answer so far, and the round trip out and
                     * back grows with that depth - 2 s to the first ring and
                     * 8 s to the fourth in one measured campaign - so a flat
                     * floor is always too short for exactly the outermost
                     * tags, which are the ones the extra waves exist to reach.
                     * See MESH_DISCOVERY_WAVE_ALLOWANCE_MS.
                     *
                     * Recomputed every poll rather than fixed at wave start: a
                     * beacon from a deeper ring arriving mid-wave is itself the
                     * evidence that the ring behind IT will need longer, and in
                     * a herd still being mapped that is the common case. */
                    uint32_t u32WaveFloorMs =
                        MESH_DISCOVERY_MIN_WAVE_MS +
                        ((uint32_t)MESHNETWORK_u8GetMaxDiscoveredWave() *
                         MESH_DISCOVERY_WAVE_ALLOWANCE_MS);
                    if (u32WaveFloorMs > MESH_DISCOVERY_MIN_WAVE_CAP_MS)
                        u32WaveFloorMs = MESH_DISCOVERY_MIN_WAVE_CAP_MS;

                    /* Silence only means "the herd has finished answering"
                     * once two other things are true.
                     *
                     * MIN_WAVE: the wave clock starts when the DReq is
                     * ENQUEUED, and between enqueue and air a packet waits out
                     * its jitter, whatever is queued ahead of it, and the radio
                     * layer's carrier sense. Without a floor the idle window
                     * could expire before the DReq had even been transmitted,
                     * which is the 3 s / 0 neighbour campaign in the logs.
                     *
                     * UN-ACKED: a node the primary has heard but not yet put
                     * in a D-Ack has no reason to stop beaconing, so ending
                     * its wave is the one thing guaranteed to be wrong. This
                     * is the direct form of what the idle window used to have
                     * to infer from timing and could not - and it is what lets
                     * the beacon cadence back off (MESH_BEACON_BASE_MS) past
                     * any idle value we could afford.
                     *
                     * Of the three, the un-acked hold is time-boxed by
                     * MESH_DISCOVERY_UNACKED_HOLD_MS and per-node by
                     * MESH_ACK_TRIES_MAX, because bAcked is cleared on every
                     * re-beacon and a node whose acks never reach it would
                     * otherwise re-arm the condition indefinitely.
                     *
                     * STALE is the other half, and it is what makes "do not cut
                     * off a wave that is still being answered" hold at herd
                     * scale. Beacon silence is a fine proxy for "the herd has
                     * finished" at nine nodes; at thirty to fifty it is not,
                     * because re-beacons from nodes already in the table arrive
                     * indefinitely and the air never goes quiet for a full
                     * MESH_DISCOVERY_IDLE_MS. On silence alone a single wave
                     * would then run to the campaign deadline and starve every
                     * later wave, so the frontier would never advance - worse
                     * than the flat ceiling this replaced.
                     *
                     * So the wave ends on whichever comes first: the air going
                     * quiet, or the wave going a whole listen floor without
                     * turning up a node the table did not already have. The
                     * floor is the right span to wait because it is already the
                     * measured round trip to the next ring out - if nothing new
                     * has answered in that long, the ring behind the last find
                     * is not answering either. A late deep beacon resets the
                     * clock and the wave carries on, which is the case that
                     * matters; noise from known nodes does not. */
                    bool bQuiet    = (tNow - tLastBeaconTick) > MESH_DISCOVERY_IDLE_MS;
                    bool bStale    = (tNow - u32LastNewTick)  >= u32WaveFloorMs;
                    bool bUnacked     = MESHNETWORK_bHasUnackedNeighbors();
                    bool bUnackedHold = bUnacked &&
                                        (u32WaveMs < MESH_DISCOVERY_UNACKED_HOLD_MS);

                    if (u32WaveMs >= u32WaveFloorMs && !bUnackedHold &&
                        (bQuiet || bStale))
                    {
                        /* QUIET: the air actually went silent. STALE: still
                         * busy, but a whole floor passed with nothing the table
                         * did not already have. At herd scale expect STALE to
                         * dominate; at nine nodes it should almost never
                         * appear, and if it does, something regressed. */
                        /* Still un-acked rows here means the only reason we
                         * got past the gate is that the hold timed out - which
                         * is a different story from the herd having finished,
                         * so it gets its own name. */
                        pcWaveEnd = bUnacked ? "HOLD-EXPIRED"
                                             : (bQuiet ? "QUIET" : "STALE");
                        u32WaveFloorAtEnd = u32WaveFloorMs;
                        break;
                    }

                    /* Radio test entered mid-campaign. Bail out rather than
                     * run the waves out: MESHNETWORK_bSendPacket returns false
                     * for the duration of a test, so the DReq never actually
                     * goes out, no beacon can be heard in reply, and every wave
                     * would sit out its full listen floor - six of them,
                     * followed by a logger session that fights the test for the
                     * same Farmranger link and PA0 line. */
                    if (RADIOTESTMODE_bActive())
                    {
                        pcWaveEnd = "RADIOTEST";
                        break;
                    }
                }

                const uint16_t u16UnionNow = MESHNETWORK_u16GetNeighborCount();

                /* "Barren" now means this wave added no node the table did not
                 * already have — not "no beacon was heard".
                 *
                 * bBeaconSeenThisWave keys off MESHNETWORK_u32GetLastBeaconHeard
                 * Tick, and that tick is stamped on a beacon that DEDUPES OUT
                 * (MeshNetwork.c, inside the FORWARD_bHasSeen early return) as
                 * well as on a real table insert. So a duplicate relayed copy of
                 * an already-counted beacon — the exact traffic the beacon-relay
                 * bound exists to limit, and which rises with herd size — used to
                 * reset the barren run and keep the campaign extending.
                 *
                 * Note what is NOT wrong with the old signal: hearing a beacon
                 * that another primary's DReq solicited. A tag does not answer a
                 * specific primary — any primary's DReq starts it beaconing, any
                 * primary records and acks whatever it hears, and the per-primary
                 * unions are merged server-side. So a beacon this primary had not
                 * yet recorded is a real find no matter who asked for it, and the
                 * union count treats it as one. It is only the DUPLICATE that was
                 * being miscounted. The union count is also per-primary by
                 * construction, since it is this primary's own table.
                 *
                 * The union count is the honest signal and costs nothing: it is
                 * only ever incremented on a real insert, under the same mutex
                 * that stamps the tick, so a wave that added no row genuinely
                 * discovered nobody. Un-acked nodes are NOT affected by this —
                 * they hold the WAVE open via bUnackedHold above, which runs
                 * before any of this, so a node still awaiting its ack cannot be
                 * cut short by a barren campaign end. */
                const bool bNoNewThisWave = (u16UnionNow == u16WaveUnion0);

                /* One line per wave. end= is the rule that fired; new= is how
                 * many nodes this wave actually added, which is the only
                 * measure of whether a wave earned its airtime. barren= is the
                 * campaign-end predicate itself, and beacons= is kept beside it
                 * because the two disagreeing is precisely the dedupe traffic
                 * described above. WAVECAP and BARREN are deliberately absent
                 * from end=: those end the CAMPAIGN, not the wave, and are
                 * logged by the branches below. */
                DBG_LOG("DeviceDiscovery: wave %u/%u dreq=%08X floor=%lu dur=%lu end=%s beacons=%u new=%u union=%u unacked=%u barren=%u prevBarren=%u beaconSeen=%u\r\n",
                    (unsigned)u8WaveCount, (unsigned)APP_PRIMARY_MAX_WAVES,
                    (unsigned)u32DreqId,
                    (unsigned long)u32WaveFloorAtEnd,
                    (unsigned long)(osKernelGetTickCount() - u32WaveStart),
                    pcWaveEnd,
                    (unsigned)(MESHNETWORK_u16GetBeaconsHeard() - u16WaveBeacons0),
                    (unsigned)(u16UnionNow - u16WaveUnion0),
                    (unsigned)u16UnionNow,
                    (unsigned)MESHNETWORK_u16GetUnackedCount(),
                    (unsigned)bNoNewThisWave,
                    (unsigned)bPrevWaveBarren,
                    (unsigned)bBeaconSeenThisWave);

                if (RADIOTESTMODE_bActive())
                {
                    bDiscoveryFinished = true;
                    MESHNETWORK_vStopPrimaryAck();
                }
                else if (bDeadlineHit)
                {
                    DBG_LOG("DeviceDiscovery: Primary campaign deadline (%u ms) reached after %u waves\r\n",
                        (unsigned)APP_PRIMARY_CAMPAIGN_MAX_MS, (unsigned)u8WaveCount);
                    bDiscoveryFinished = true;
                    MESHNETWORK_vStopPrimaryAck();
                }
                /* A single barren wave no longer ends the campaign: the frontier
                 * advances one ring per wave, deep answers can arrive late (past
                 * that wave's floor), and one wave's lone beacon can be lost to a
                 * collision — so "this wave found nobody new" has meant "nobody
                 * answered in time" as often as "nobody is there". End only on
                 * TWO barren waves in a row (past APP_PRIMARY_MIN_WAVES), or the
                 * wave cap. A tight herd still ends promptly: each productive
                 * wave resets the run, and once the herd is truly mapped two
                 * barren waves follow.
                 *
                 * The two rules do not stack below the floor: because the AND
                 * already requires u8WaveCount >= MIN_WAVES (3), the earliest
                 * barren end is still wave 3 and it needs waves 2 AND 3 both
                 * barren — identical to the old single-barren rule for that
                 * case. The rule only buys a retry from wave 4 onward; it does
                 * not protect the direct-earshot rings.
                 *
                 * "Barren" is bNoNewThisWave (no new union row), NOT "no beacon
                 * heard" — see where it is derived above for why the beacon tick
                 * was the wrong signal. */
                else if ((bNoNewThisWave && bPrevWaveBarren &&
                          u8WaveCount >= APP_PRIMARY_MIN_WAVES) ||
                         u8WaveCount >= APP_PRIMARY_MAX_WAVES)
                {
                    if (u8WaveCount >= APP_PRIMARY_MAX_WAVES)
                        DBG_LOG("DeviceDiscovery: Primary wave cap (%u) reached barren=%u prevBarren=%u\r\n",
                            APP_PRIMARY_MAX_WAVES,
                            (unsigned)bNoNewThisWave, (unsigned)bPrevWaveBarren);
                    else
                        DBG_LOG("DeviceDiscovery: Primary ending - two barren waves after %u waves\r\n",
                            (unsigned)u8WaveCount);
                    bDiscoveryFinished = true;
                    MESHNETWORK_vStopPrimaryAck();
                }
                else
                {
                    DBG_LOG("DeviceDiscovery: Primary extending discovery with new DReq wave\r\n");
                }

                bPrevWaveBarren = bNoNewThisWave;
            }
        }
        else
        {
            /* In LoRaWAN mode this wake is only here to catch the primary's
             * TimeSync (RTC + any mode/interval/gps change + staged firmware)
             * - the report itself went out over LoRaWAN before the wake. */
            DBG_LOG("DeviceDiscovery %X: Secondary waiting for timesync (%s).\r\n",
                LORARADIO_u32GetUniqueId(),
                (eWakeMode == DISCOVERY_MODE_LORAWAN) ? "lorawan" : "advanced");

            osThreadFlagsClear(DEVICE_DISCOVERY_NOTIFY_TIMESYNC |
                               DEVICE_DISCOVERY_NOTIFY_OTA);

            /* R3: end the campaign on the FIRST of:
             *   - TimeSync received (clean end),
             *   - 10 s of mesh radio silence while NOT beaconing (UNKNOWN that
             *     heard nothing, or FORWARDER once the mesh goes quiet),
             *   - the 205 s hard cap.
             * While beaconing the silence rule is suppressed, and since the
             * beacon count cap was removed a node that is never acked keeps
             * beaconing to the end of the window — so for that node this 205 s
             * cap is the terminator, and MESHNETWORK_vStopBeaconingSelf() below
             * is what stops it. u32SilenceRef starts at the campaign start (fair
             * first window) and advances to the latest discovery packet from ANY
             * primary (multi-primary safe). */
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

                /* prodsleep/solarsleep commanded mid-campaign: abandon it and
                 * head for sleep instead of running out the window. */
                if (eProductionState == PRODUCTION_SLEEP)
                    break;

                /* Radio test entered mid-campaign: same treatment. The test
                 * owns the radio from here, so this window can only run out
                 * its clock hearing nothing. */
                if (RADIOTESTMODE_bActive())
                    break;

                if ((uint32_t)(u32Now - u32CampaignStart) >= APP_DISCOVERY_WINDOW_TIMEOUT_MS)
                    break;   /* hard cap */

                /* Silence only ends the campaign for a node that has heard
                 * NOTHING. Once a DReq has been heard — the wave-1 flood
                 * reaches the whole mesh for this purpose — we know a campaign
                 * is running and that we are inside its footprint, so quiet air
                 * means "the frontier has not reached my ring yet", not
                 * "nothing is out there". Bailing there is what lost the deep
                 * tags: u32SilenceRef starts at campaign start, so a node
                 * awaiting its wave was already 10 s into its own death clock
                 * at wake-up. Such a node now holds until TimeSync or the
                 * 205 s hard cap above.
                 *
                 * A node that truly hears nothing keeps the old behaviour
                 * unchanged, so out-of-range units cost no extra power — which
                 * is why the flood is worth its airtime.
                 *
                 * LoRaWAN mode sends nothing for silence to be measured
                 * against - no DReq, no beacons, only the TimeSync this wait
                 * is for - so a fixed wait replaces the silence rule there.
                 * See APP_LORAWAN_TIMESYNC_WAIT_MS. A DReq heard anyway (an
                 * advanced-mode primary in range) falls back to the advanced
                 * rules above: stay for its campaign. */
                if (eWakeMode == DISCOVERY_MODE_LORAWAN)
                {
                    if (!MESHNETWORK_bCampaignHeard() &&
                        (uint32_t)(u32Now - u32CampaignStart) >= APP_LORAWAN_TIMESYNC_WAIT_MS)
                        break;   /* no TimeSync in the LoRaWAN wait window */
                }
                else if (!MESHNETWORK_bIsBeaconing() &&
                         !MESHNETWORK_bCampaignHeard() &&
                         (uint32_t)(u32Now - u32SilenceRef) >= APP_SECONDARY_SILENCE_MS)
                    break;   /* radio silence, nothing heard, not beaconing */
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

        /* Whole-discovery duration, wake to here — see u32CampaignStartMs
         * above. Passed to the fr9 in the AT+BLOG header. */
        uint32_t u32CampaignDurationS =
            (osKernelGetTickCount() - u32CampaignStartMs) / 1000U;

        /* Drop whatever mesh traffic is still queued for TX. The campaign it
         * belonged to is over, so sending it is pointless — and on a
         * heavy-forwarding node the radio task would otherwise spend minutes
         * carrier-sensing through that backlog, deaf the whole time (carrier
         * sense runs the chip in CAD, not RX). That deafness is what made one
         * unit miss the primary's post-campaign TimeSync in 39 of 47
         * campaigns and so never arm FOTA acceptance at all. Flushed here,
         * before the stats line, so the drop shows up in this campaign's
         * tally. */
        /* true: keep a pending TimeSync relay. On a secondary this flush runs
         * the instant a TimeSync ends its campaign, which is exactly while its
         * own relay of that TimeSync is still waiting out its TX jitter -
         * dropping it is what left the nodes behind it unsynced. */
        MESHNETWORK_vFlushTxQueue(true);
        LORARADIO_vFlushTxQueue();

        MESHNETWORK_vLogCampaignStats("campaign");
        /* Primary only: who this campaign heard but never managed to silence,
         * with each one's ack-try count. A short or empty roster is the goal;
         * a long one with tries=MESH_ACK_TRIES_MAX says the acks are not
         * reaching those nodes, and a long one with tries=0 says ack
         * throughput ran out before the campaign did. */
        if (eDeviceRole == DEVICE_ROLE_PRIMARY)
            MESHNETWORK_vLogUnackedNeighbors();
        EVTLOG(LOG_DISCOVERY_CMPLT, eDeviceRole);

        if (eDeviceRole == DEVICE_ROLE_PRIMARY)
        {
            /* LoRaWAN mode ran no campaign and has nothing to upload - the
             * tags' reports went to TTN. The rest of the fr9 session (time,
             * settings, TimeSync, firmware) runs the same in both modes. */
            const bool bUpload = (eWakeMode == DISCOVERY_MODE_ADVANCED);

            MeshDiscoveredNeighbor_t tNeighbors[MESH_MAX_NEIGHBORS];
            uint16_t u16NeighborCount = 0;

            if (!bUpload)
            {
                /* nothing to report */
            }
            else if (MESHNETWORK_bGetDiscoveredNeighbors(tNeighbors, MESH_MAX_NEIGHBORS,
                                                         &u16NeighborCount))
            {
                DBG_LOG("DeviceDiscovery %X: Final UNION: %u neighbors.\r\n",
                    LORARADIO_u32GetUniqueId(), u16NeighborCount);
                EVTLOG(LOG_DISCOVERY_COUNT, u16NeighborCount);
                for (uint16_t i = 0; i < u16NeighborCount; i++)
                {
                    /* RssiSrc: the node whose DReq gave this neighbour its
                     * reported RSSI — 0 if that neighbour's firmware predates
                     * the field. Together with RSSI it says where the tag's
                     * good path actually is, which hop count alone does not. */
                    DBG_LOG("  ID:%X  Hops:%X  Wave:%d  RSSI:%d  RssiSrc:%04X  Bat:%d  Move:%u  Lat:%ld  Lon:%ld  FwPatch:%u\r\n",
                        tNeighbors[i].u32DeviceId,
                        tNeighbors[i].u8HopCount,
                        tNeighbors[i].u8Wave,
                        tNeighbors[i].i16Rssi,
                        tNeighbors[i].u16BestRssiSrcId,
                        tNeighbors[i].u16BatMv,
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

            /* ---- Logger session (fr9 Farmranger board) ----
             *
             * Skipped entirely if a radio test was entered mid-campaign. The
             * test drives the same Farmranger link and the same PA0 attention
             * line, one beacon at a time; running this round-trip alongside it
             * means two owners raising and dropping that line, and two
             * producers on a 4-deep AT queue whose put timeout is 100 ms. The
             * fr9 sees the result as a burst of sessions that carry no bytes,
             * each cancelled by the other side, plus the odd stray character
             * from a transfer cut off when the other DeviceOff de-inits the
             * UART pins mid-send.
             *
             * The neighbour table is not lost by skipping - it stays in the
             * mesh store and goes up on the next campaign after the test. */
            if (RADIOTESTMODE_bActive())
            {
                DBG_LOG("DeviceDiscovery: radio test active - skipping logger session\r\n");
            }
            else
            {
                /* ---- Logger connection + upload (fr9 Farmranger board) ----
                 *
                 * The connect result is now checked. It used to be discarded,
                 * and that was worth fixing while touching this: without "RDY"
                 * the fr9 is not in a session at all, and the whole campaign
                 * was streamed into a dead link before anything noticed - only
                 * the upload's own timeouts eventually said so. The binary
                 * path makes the check unavoidable anyway, because it needs
                 * the CONTENT of the ready line, not just its arrival.
                 *
                 * Only the upload is skipped on failure. The requests below
                 * are left to fail on their own timeouts exactly as they do
                 * today; turning a dead link into an early exit from the whole
                 * session is a bigger behavioural change than belongs here. */
                bool bLoggerUp = DEVICE_DISCOVERY_DRIVER_bConnectLogger();
                bool bLogOk    = false;

                if (!bLoggerUp)
                {
                    DBG_LOG("DeviceDiscovery %X: Logger connect FAILED (no RDY) - upload skipped.\r\n",
                        LORARADIO_u32GetUniqueId());
                }
                else if (bUpload)
                {
                    DBG_LOG("DeviceDiscovery %X: Logger connected (binary upload).\r\n",
                        LORARADIO_u32GetUniqueId());

                    bLogOk = DEVICE_DISCOVERY_bSendDiscoveryDataBin(tNeighbors, u16NeighborCount,
                        u32CampaignDurationS);
                }
                else
                {
                    DBG_LOG("DeviceDiscovery %X: Logger connected (LoRaWAN mode - no upload).\r\n",
                        LORARADIO_u32GetUniqueId());
                }

                if (!bUpload)
                {
                    /* no upload attempted */
                }
                else if (bLogOk)
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
                bool    bLoRaWanMode  = false;
                bool    bGpsEnabled   = true;
                if (DEVICE_DISCOVERY_DRIVER_bRequestSettings(&u8WakeInterval,
                                                            &bLoRaWanMode,
                                                            &bGpsEnabled))
                {
                    if      (u8WakeInterval == 15)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_15_MIN);
                    else if (u8WakeInterval == 30)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_30_MIN);
                    else if (u8WakeInterval == 60)  MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_60_MIN);
                    else if (u8WakeInterval == 120) MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_120_MIN);
                    else if (u8WakeInterval == 240) MESHNETWORK_vSetWakeupInterval(WAKEUP_INTERVAL_240_MIN);

                    MESHNETWORK_vSetDiscoveryMode(bLoRaWanMode ? DISCOVERY_MODE_LORAWAN
                                                              : DISCOVERY_MODE_ADVANCED);
                    MESHNETWORK_vSetGpsEnabled(bGpsEnabled);

                    DBG_LOG("DeviceDiscovery: settings applied - interval=%u min, mode=%s, gps=%u\r\n",
                            (unsigned)u8WakeInterval,
                            bLoRaWanMode ? "lorawan" : "advanced",
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
            }

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

#ifdef STORAGE_BACKEND_FLASH
            /* Drop any leftover discovery TimeSync auto-arm before the kernel
             * session opens: this wake is for kernel commands, not OTA. Left
             * armed, a stray OtaPrep here would latch (bPrepPending) and — since
             * the kernel rendezvous only serves an explicit "tag fwaccept" — sit
             * un-consumed, then reject the next campaign's real OtaPrep as a
             * different-sid session ("locked to"), wedging that secondary out of
             * OTA. The next discovery TimeSync re-arms it anyway, so nothing is
             * lost. An explicit "tag <ID> fwaccept" during the session re-arms
             * via the kernel path. */
            if (!FOTA_bAcceptanceArmedViaKernel())
                FOTA_vDisarmAcceptance();
#endif

            uint32_t u32WaitedMs = 0U;
            while (!FRKERNEL_bIsConnected() && u32WaitedMs < DEVICE_DISCOVERY_KERNEL_WAKEUP_WINDOW_MS)
            {
                /* A prodsleep/solarsleep command taken during this window is
                 * an order to sleep NOW — don't keep the device awake for the
                 * rest of the 3 min holding the kernel sleep lock. (The
                 * command's own auto-release can otherwise be missed entirely
                 * by this 500 ms poll, since it clears the session flag ~100 ms
                 * after setting it.) */
                if (eProductionState == PRODUCTION_SLEEP)
                    break;

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
                    FOTA_bAcceptanceArmedViaKernel() && FOTA_bPrepPending())
                {
                    /* Only an EXPLICIT "tag <ID> fwaccept" (kernel arm) runs an
                     * OTA receive inside a kernel session. A leftover discovery
                     * TimeSync auto-arm must not be consumed here: it would let
                     * a stray OtaPrep during a log-download wakeup start a
                     * pointless receive that no primary is actually feeding
                     * (the "receiving v... 0/N chunks - abort" seen in field
                     * logs). Verified image -> resets into the bootloader (no
                     * return); failure returns here and the session keeps
                     * holding. */
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

        /* Both roles wake on WakeupInterval in both modes. In LoRaWAN mode
         * the reports run on their own 15 min schedule below, independent of
         * this one. */
        uint64_t u64Slot      = u64Utc / (uint64_t)u32IntervalS;

        /* ---- R&D radio link/range test: participate in nothing ----
         * Same shape as the ProductionSleep gate below: one continue skips
         * the scheduled discovery wake, the LoRaWAN report window and the
         * GPS pre-trigger. Unlike ProductionSleep this does not sleep — the
         * mode holds its own sleep lock — it simply stops the schedule from
         * starting campaigns underneath a test that owns the radio. */
        if (RADIOTESTMODE_bActive())
            continue;

        /* ---- ProductionSleep: secondary only — primary has no solar panel ---- */
        if (eDeviceRole == DEVICE_ROLE_SECONDARY && eProductionState == PRODUCTION_SLEEP)
        {
            /* SolarSleep ("tag solarsleep"): shake-to-wake only. Skip the
             * solar-activation check below entirely — the whole point is to
             * stay disabled while the panel charges the battery, so a rising
             * Vsolar/Psolar must not pull us out. The continue still applies,
             * so no scheduled discovery wake fires either. Exit is via the
             * shake sequence -> DEVICE_DISCOVERY_vTriggerKernelWakeup(). */
            if (bSolarWakeDisabled)
                continue;

#ifdef ENABLE_SOLAR_POWER_SENSE
            if (SOLAR_u32GetPowerMW() >= SOLAR_ACTIVATION_POWER_MW)
            {
                eProductionState = PRODUCTION_ACTIVE;
                /* Swap the STOP2-indicator LED back to red now that we're
                 * no longer in ProductionSleep — see the setter's docs. */
                HAL_SYSTEM_vSetSleepIndicatorLed(HAL_SYSTEM_SLEEP_LED_RED);

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
                 * possible instead of waiting. GPS_vRequestFix() now owns
                 * its own deep-sleep lock (taken on cold start, released in
                 * GPS_vPowerOff()), independent of the kernel-window lock
                 * acquired above (released separately at the AppTask loop
                 * end). */
                if (POWER_tGetState() & POWER_CLASS_NORMAL)
                {
                    DBG_LOG("DeviceDiscovery: GPS acquire on ProductionSleep wake (5 min timeout)\r\n");
                    /* bForce=true: RTC correction after unbounded
                     * ProductionSleep must run even if the user disabled
                     * routine GPS via fr9. */
                    GPS_vRequestFix(true, 300, true);
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
                /* Swap the STOP2-indicator LED back to red now that we're
                 * no longer in ProductionSleep — see the setter's docs. */
                HAL_SYSTEM_vSetSleepIndicatorLed(HAL_SYSTEM_SLEEP_LED_RED);

                /* See the ENABLE_SOLAR_POWER_SENSE branch above for why the
                 * UART is re-armed here and why this opens a kernel window
                 * rather than a discovery campaign. */
                SYSTEM_vSleepLockAcquire();
                HAL_UART_vInit();
                DEBUG_vInit();

                DBG_LOG("DeviceDiscovery: Solar activation (%u mV) — exiting ProductionSleep\r\n",
                    SOLAR_u16GetVSolarMV());

                /* See the ENABLE_SOLAR_POWER_SENSE branch above for why this
                 * fires a GPS fix immediately (RTC correction on takeoff);
                 * GPS_vRequestFix() owns its own sleep lock. */
                if (POWER_tGetState() & POWER_CLASS_NORMAL)
                {
                    DBG_LOG("DeviceDiscovery: GPS acquire on ProductionSleep wake (5 min timeout)\r\n");
                    /* bForce=true: RTC correction after unbounded
                     * ProductionSleep must run even if the user disabled
                     * routine GPS via fr9. */
                    GPS_vRequestFix(true, 300, true);
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

        /* --- LoRaWAN-mode reports (SECONDARY only) ---
         * One report per LORAWAN_UPLINK_PERIOD_S, in the window that closes
         * just before each 15 min UTC boundary B (LoRaWan.h has the timeline):
         *
         *   B - LORAWAN_GPS_LEAD_S              request a GPS fix
         *   first second the GPS is idle again  start the report (fix or not)
         *   B - LORAWAN_UPLINK_DEADLINE_LEAD_S  start it now regardless
         *
         * Wake boundaries are multiples of 15 min too, so the window always
         * closes before any mesh wake can open. The mesh wake is untouched by
         * this and still runs on WakeupInterval, for the TimeSync.
         *
         * A report is only started while the AppTask is idle - it is the task
         * that will run it, and if it is still inside a wake or a kernel
         * session that session may own the radio. It retries every second
         * until the deadline, then gives the slot up. */
        static uint64_t u64LastReportSlot = 0ULL;
        static uint64_t u64LastGpsReqSlot = 0ULL;
        if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
            MESHNETWORK_eGetDiscoveryMode() == DISCOVERY_MODE_LORAWAN)
        {
            uint64_t u64UpSlot = (u64Utc / (uint64_t)LORAWAN_UPLINK_PERIOD_S) + 1ULL;
            uint64_t u64BoundaryUtc = u64UpSlot * (uint64_t)LORAWAN_UPLINK_PERIOD_S;
            uint32_t u32ToBoundaryS = (uint32_t)(u64BoundaryUtc - u64Utc);   /* 1..period */

            bool bInWindow = (u32ToBoundaryS <= LORAWAN_GPS_LEAD_S) &&
                             (u32ToBoundaryS >= LORAWAN_UPLINK_DEADLINE_LEAD_S);

            if (bInWindow && u64UpSlot != u64LastReportSlot)
            {
                if (u64UpSlot != u64LastGpsReqSlot)
                {
                    /* Window entry: ask for the fix. GPS_vRequestFix applies
                     * the fr9 GPS-disable flag and the power class itself, and
                     * owns its own sleep lock. Its TTFF timeout ends the
                     * session by the deadline, which is what the "GPS idle"
                     * trigger below keys on. */
                    u64LastGpsReqSlot = u64UpSlot;
                    if (MESHNETWORK_bGetGpsEnabled() && (POWER_tGetState() & POWER_CLASS_NORMAL))
                    {
                        HAL_UART_vInit();
                        DEBUG_vInit();
                        DBG("DeviceDiscovery: LoRaWAN report GPS pre-trigger\r\n");
                        GPS_vRequestFix(true, LORAWAN_GPS_TTFF_TIMEOUT_S, false);
                    }
                }
                else if (!GPS_bIsActive() ||
                         u32ToBoundaryS <= LORAWAN_UPLINK_DEADLINE_LEAD_S)
                {
                    if (!bAppTaskBusy && !RADIOTESTMODE_bActive() && !FRKERNEL_bIsConnected() &&
                        (POWER_tGetState() & (POWER_CLASS_NORMAL | POWER_CLASS_LOW)))
                    {
                        u64LastReportSlot     = u64UpSlot;
                        u32LoRaWanDeadlineUtc = (uint32_t)(u64BoundaryUtc - LORAWAN_UPLINK_DEADLINE_LEAD_S);
                        /* Released by the AppTask once the session ends. */
                        SYSTEM_vSleepLockAcquire();
                        osEventFlagsSet(xDiscoveryEventFlags, DISCOVERY_LORAWAN_UPLINK_BIT);
                    }
                    else if (u32ToBoundaryS <= LORAWAN_UPLINK_DEADLINE_LEAD_S)
                    {
                        u64LastReportSlot = u64UpSlot;
                        DBG_LOG("DeviceDiscovery: LoRaWAN report slot missed - device busy\r\n");
                    }
                }
            }
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
        /* Not in LoRaWAN mode: its wakes carry no position (no beacons), and
         * the reports trigger their own fix above. The latch is left alone
         * while skipped; its forward-jump resync re-arms it on the way back
         * to advanced mode. */
        if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
            MESHNETWORK_eGetDiscoveryMode() != DISCOVERY_MODE_LORAWAN &&
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
            /* GPS_vRequestFix() owns its deep-sleep lock now (taken on the
             * cold-start transition, released in GPS_vPowerOff()), so no
             * lock is taken here and an overlap with an in-flight GPS
             * session — a still-running ProductionSleep-exit fix, say —
             * simply attaches instead of leaking a second lock. The
             * POWER_CLASS_NORMAL guard is kept as an optimisation: it
             * skips the UART/DEBUG re-arm when GPS_vRequestFix() would
             * refuse for want of power anyway. */
            HAL_UART_vInit();
            DEBUG_vInit();

            DBG("DeviceDiscovery: GPS pre-trigger\r\n");
            /* auto-shutdown on completion, bounded to GPS_PRETRIGGER_S so it
             * can never run forever. bForce=false: respects the user's
             * fr9-side GPS-disable setting (unlike the ProductionSleep-exit
             * paths, which need RTC correction regardless). */
            GPS_vRequestFix(true, 120, false);
        }
    }
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vSendTS
 * -------------------------------------------------------------------------- */
/* Both call sites used to EVTLOG(LOG_TX_TS, 1) themselves. The event now
 * comes from MESHNETWORK_vSendTimeSync, which is the only place that knows
 * whether the packet actually reached the TX queue and which of its two
 * airings went out. */
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
    {
        /* Only advertise what we can actually deliver. bValid alone says the
         * metadata is intact, not that the image still verifies — a primary
         * whose PRE-SEND check keeps failing would otherwise keep telling the
         * whole fleet "newer fw available", arming every secondary for an
         * update it can never send. Suppress the advertisement until a good
         * verify clears the suspicion; the image itself is kept. */
        if (FOTA_bStagedImageTrusted())
        {
            u32StagedVer = tMeta.u32Version;
        }
        else
        {
            DBG_LOG("Fota: staged v%lu not advertised - last PRE-SEND verify failed\r\n",
                    (unsigned long)tMeta.u32Version);
        }
    }
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
    /* Shake is the documented — and, on a beaconing secondary, the only —
     * way out of the R&D radio-test mode: once beaconing it drops every
     * inbound packet, so no over-the-air command can reach it.
     *
     * Left first, before any of the normal wakeup work below, so what follows
     * is exactly the kernel window a shake would have produced had the mode
     * never been entered. This function is only ever reached from the
     * secondary-only shake sequence in Movement.c (solar activation sets
     * DISCOVERY_KERNEL_BIT directly), so a listening primary is unaffected —
     * it exits on "tag <ID> radio stop" instead. */
    if (RADIOTESTMODE_bActive())
        RADIOTESTMODE_vExit("shake");

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
        bool bWasSolarSleep = bSolarWakeDisabled;

        eProductionState   = PRODUCTION_ACTIVE;
        /* Shake is the ONLY exit from SolarSleep — clear the flag here or the
         * next ordinary "tag prodsleep" would inherit it and ignore solar. */
        bSolarWakeDisabled = false;
        /* Swap the STOP2-indicator LED back to red now that we're no
         * longer in ProductionSleep — see HAL_SYSTEM_vSetSleepIndicatorLed. */
        HAL_SYSTEM_vSetSleepIndicatorLed(HAL_SYSTEM_SLEEP_LED_RED);
        DBG_LOG("DeviceDiscovery: Kernel wakeup — exiting %s\r\n",
                bWasSolarSleep ? "SolarSleep" : "ProductionSleep");

        /* Same rationale as the solar wake path: get the RTC corrected as
         * early as possible after an unbounded ProductionSleep stretch,
         * not just at the next scheduled pre-trigger. GPS_vRequestFix()
         * owns its own sleep lock (released inside GPS_vPowerOff()). */
        if (POWER_tGetState() & POWER_CLASS_NORMAL)
        {
            DBG_LOG("DeviceDiscovery: GPS acquire on ProductionSleep wake (5 min timeout)\r\n");
            /* bForce=true: same rationale as the solar wake path -
             * RTC correction after unbounded ProductionSleep. */
            GPS_vRequestFix(true, 300, true);
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
 * DEVICE_DISCOVERY_vEnterSleepMode — shared body for the two super-deep-sleep
 * commands. Identical in every respect (state, indicator LED, suppressed
 * discovery/acc logging) except whether a solar-level rise may wake us:
 *   bAllowSolarWake == true  -> "tag prodsleep"  (solar OR shake)
 *   bAllowSolarWake == false -> "tag solarsleep" (shake only; the panel gets
 *                               to charge the battery undisturbed)
 * -------------------------------------------------------------------------- */
static void DEVICE_DISCOVERY_vEnterSleepMode(bool bAllowSolarWake)
{
    const char *pacName = bAllowSolarWake ? "ProductionSleep" : "SolarSleep";

    if (eDeviceRole != DEVICE_ROLE_SECONDARY)
    {
        DBG_LOG("DeviceDiscovery: %s not applicable on primary device\r\n", pacName);
        return;
    }

    bSolarWakeDisabled = !bAllowSolarWake;
    /* Set the state FIRST: the schedule task gates on it, so from here on it
     * starts nothing new (no scheduled wake, no GPS pre-trigger) while we
     * tear down what is already running. */
    eProductionState   = PRODUCTION_SLEEP;

    /* Kill anything still running so the device goes down clean instead of
     * sitting awake for minutes after being told to sleep.
     *
     * GPS is the one that both holds a sleep lock AND leaves a device
     * powered: the solar/kernel wake paths kick off a 5-minute
     * GPS_vRequestFix(), so without this the module would keep running (and
     * blocking STOP2) long after the command. GPS_vShutdown() powers the
     * receiver down and releases that lock through the normal
     * GPS_vPowerOff() path, so the accounting stays exact — no leak, and no
     * double release (it no-ops when already idle).
     *
     * The other in-flight work needs no forced teardown, it just has to be
     * allowed to fall through its own exit: the AppTask's campaign / kernel
     * windows now break out on PRODUCTION_SLEEP and release their sleep lock
     * at the loop tail as always, and the FrKernel handler closes the
     * session right after acking. A FOTA receive is deliberately left alone
     * — it owns flash state mid-write and unwinds safely on its own silence
     * timeout; forcing it here would risk a worse mess than the short wait. */
    GPS_vShutdown();

    /* Swap the STOP2-indicator LED to yellow so the bench can tell
     * "asleep in ProductionSleep/SolarSleep, waiting for solar/shake" apart
     * from the normal red "asleep between scheduled wakes" at a glance. Any
     * exit path (solar activation or Kernel wakeup) swaps it back to red. */
    HAL_SYSTEM_vSetSleepIndicatorLed(HAL_SYSTEM_SLEEP_LED_YELLOW);
    DBG_LOG("DeviceDiscovery: Entering %s\r\n", pacName);
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vEnterProductionSleep
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vEnterProductionSleep(void)
{
    DEVICE_DISCOVERY_vEnterSleepMode(true);
}

/* --------------------------------------------------------------------------
 * DEVICE_DISCOVERY_vEnterSolarSleep
 * -------------------------------------------------------------------------- */
void DEVICE_DISCOVERY_vEnterSolarSleep(void)
{
    DEVICE_DISCOVERY_vEnterSleepMode(false);
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
