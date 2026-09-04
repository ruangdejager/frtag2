/*
 * DeviceDiscovery.h
 *
 * Application task that orchestrates the LoRa mesh discovery campaign,
 * GPS fix acquisition, and data upload to the logger.
 *
 * Uses CMSIS-RTOS v2 throughout.
 * The wake-up event uses an osEventFlags object (DISCOVERY_WAKEUP_BIT).
 * Time-sync notifications from MeshNetwork arrive as thread flags
 * (DEVICE_DISCOVERY_NOTIFY_TIMESYNC) on the AppTask thread.
 */

#ifndef TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_
#define TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_

#include "cmsis_os2.h"
#include "MeshNetwork.h"

/* ---- Discovery timing ---- */
#define APP_WAKEUP_BUFFER_MS                (5  * 1000)   /* buffer after sync wake-up   */
#define APP_DISCOVERY_WINDOW_TIMEOUT_MS     (205 * 1000)  /* hard cap on a campaign      */

/* Secondary campaign-end (R3): once the mesh has been silent (no discovery
 * packet from any primary) for this long AND the node is not beaconing, end the
 * campaign instead of waiting out the 205 s hard cap. Polled at this cadence. */
#define APP_SECONDARY_SILENCE_MS           (10 * 1000)   /* radio-silence end window    */
#define APP_SECONDARY_POLL_MS              250U          /* secondary wait poll cadence */

/* Ceiling on DReq waves per campaign. Each wave advances the discovery frontier
 * by one ring (only acked nodes relay, so wave N reaches depth N), which makes
 * this the maximum discoverable herd depth.
 *
 * Stays at 6, now with a budget that can actually run all six. The wave-listen
 * floor scales with proven depth (MESH_DISCOVERY_WAVE_ALLOWANCE_MS: 8 s at
 * ring 1 climbing 4 s/ring to the 24 s cap by ring 4), because the round trip
 * out to the frontier and back is superlinear (measured: 2, 3, 4, 8, 21 s to
 * rings 1-5). Summing the worst-case per-wave cost (scaled floor + one idle
 * tail):
 *
 *     wave   1    2    3    4    5    6      -> cumulative
 *     ms   13k  17k  21k  25k  29k  29k        134k
 *
 * All six now fit under APP_PRIMARY_CAMPAIGN_MAX_MS (135 s) with ~1 s to spare;
 * that budget - and the secondary window it sits under - was raised so wave 6
 * is reachable for the deep herd instead of the campaign ending mid wave-5 the
 * way 110 s + an 18 s floor cap did (241F, ring 5, missed its wave by 5 s and
 * the barren wave ended the campaign). A 7-ring herd still needs more than
 * this; 6 is the ceiling the current budget honestly funds. The
 * MESH_WAVE_BUDGET_MS assert below makes any future disagreement between the
 * wave count and the budget a build error instead of a wave that silently never
 * runs. Costs little on a tight herd - the primary still ends a campaign once
 * two waves in a row turn up no new beacon (APP_PRIMARY_MIN_WAVES plus the
 * two-consecutive-barren rule in DeviceDiscovery.c), and the field herd
 * (4 rings) finishes by wave 4 in ~76 s with margin. */
#define APP_PRIMARY_MAX_WAVES              6U

/* Floor on DReq waves per campaign. A barren wave used to end the campaign
 * outright, which reads as "nobody is out there" but in the field also meant
 * "nobody answered in time": one campaign ended after ONE wave and 3 seconds
 * with 0 neighbours, while a 1-hop secondary was mid-cadence and two others
 * were relaying that very DReq (the primary's own stats line for it says
 * "DReq heard=3 beacons heard=0"). A DReq is one packet on a channel this
 * change set exists to decongest; three attempts at it cost little and remove
 * a whole-campaign failure mode.
 *
 * Scope this claim honestly: waves 2+ are relayed only by nodes that have been
 * ACKED (see MESHNETWORK_vHandleDReq), and a wave that heard no beacon
 * produced no acks and therefore no forwarders - so the forced waves are a
 * retry of the DIRECT earshot ring, not a way to reach the deep herd on a
 * silent wave 1. That is exactly what the 3 s campaign lost.
 *
 * MIN_WAVES is only the floor below which no barren wave may end the campaign;
 * above it, ending now also requires TWO barren waves in a row (see
 * DeviceDiscovery.c). A single quiet wave at the frontier - a deep node whose
 * one beacon that wave was lost to a collision - no longer ends a campaign that
 * has been steadily pushing outward. */
#define APP_PRIMARY_MIN_WAVES              3U

/* Deadline for the primary's wave loop, measured from campaign start.
 *
 * The wave loop had no time bound at all - only the wave COUNT was capped, and
 * a wave ends on beacon silence, so a herd that keeps beaconing could hold the
 * primary past the point where its TimeSync is any use. It has to be a
 * deadline on the wave loop specifically, not on the whole wake, because
 * TimeSync is not sent at campaign end: the fr9 logger session runs first
 * (connect, AT+LOG with up to 3 attempts, AT+TSREQ, AT+SETREQ) and that is
 * ~55 s of blocking AT timeouts in the worst case.
 *
 * Both roles start their campaign clock after the same APP_WAKEUP_BUFFER_MS,
 * so the secondaries' windows close at campaign_start + APP_DISCOVERY_WINDOW_
 * TIMEOUT_MS (205 s). 135 s leaves ~15 s of margin on top of that worst-case
 * fr9 budget, so the TimeSync is queued while the herd is still listening even
 * on a bad wake.
 *
 * 110 -> 135 s (window 180 -> 205 s in lockstep, so the fr9/TimeSync margin is
 * unchanged): 135 s is what six waves at the deep-scaled floor now cost
 * (MESH_WAVE_BUDGET_MS = 134 s). The pair moved together on purpose - raising
 * the wave budget without moving the window would push TimeSync past the point
 * secondaries stop listening. The cost is real: an in-footprint node the
 * primary never acks stays awake up to 25 s longer per campaign (an
 * out-of-range node still bails at APP_SECONDARY_SILENCE_MS, unaffected). */
#define APP_PRIMARY_CAMPAIGN_MAX_MS        (135 * 1000)

_Static_assert(APP_PRIMARY_MIN_WAVES <= APP_PRIMARY_MAX_WAVES,
               "APP_PRIMARY_MIN_WAVES exceeds APP_PRIMARY_MAX_WAVES");
_Static_assert(APP_PRIMARY_CAMPAIGN_MAX_MS < APP_DISCOVERY_WINDOW_TIMEOUT_MS,
               "The primary must finish its waves before the secondaries' "
               "campaign window closes, with room for the fr9 session.");

/* Worst-case time to run every wave: each of APP_PRIMARY_MAX_WAVES waves can run
 * to its depth-scaled listen floor plus one idle tail before it ends, so the
 * whole loop costs SUM_{k=1..N}(floor(k) + IDLE), where
 * floor(k) = min(MIN_WAVE + (k-1)*ALLOWANCE, MIN_WAVE_CAP). The floor ramps
 * linearly until it saturates at the cap, so the sum splits into the ramp and a
 * flat tail; RAMP_STEPS is how many ALLOWANCE steps fit before the cap, so the
 * floor first reaches the cap at wave (RAMP_STEPS + 1).
 *
 * This exists so the wave count and the campaign budget cannot silently
 * disagree: raise APP_PRIMARY_MAX_WAVES past what 135 s can run (or make a wave
 * more expensive) and this becomes a build error, not a wave that never fires.
 * See the table at APP_PRIMARY_MAX_WAVES. */
#define MESH_WAVE_FLOOR_RAMP_STEPS \
    ((MESH_DISCOVERY_MIN_WAVE_CAP_MS - MESH_DISCOVERY_MIN_WAVE_MS) / \
     MESH_DISCOVERY_WAVE_ALLOWANCE_MS)

#define MESH_WAVE_BUDGET_MS ( \
    ( (APP_PRIMARY_MAX_WAVES <= (MESH_WAVE_FLOOR_RAMP_STEPS + 1U)) \
      ? ( APP_PRIMARY_MAX_WAVES * MESH_DISCOVERY_MIN_WAVE_MS \
          + MESH_DISCOVERY_WAVE_ALLOWANCE_MS \
            * (APP_PRIMARY_MAX_WAVES * (APP_PRIMARY_MAX_WAVES - 1U) / 2U) ) \
      : ( (MESH_WAVE_FLOOR_RAMP_STEPS + 1U) * MESH_DISCOVERY_MIN_WAVE_MS \
          + MESH_DISCOVERY_WAVE_ALLOWANCE_MS \
            * (MESH_WAVE_FLOOR_RAMP_STEPS * (MESH_WAVE_FLOOR_RAMP_STEPS + 1U) / 2U) \
          + (APP_PRIMARY_MAX_WAVES - (MESH_WAVE_FLOOR_RAMP_STEPS + 1U)) \
            * MESH_DISCOVERY_MIN_WAVE_CAP_MS ) ) \
    + APP_PRIMARY_MAX_WAVES * MESH_DISCOVERY_IDLE_MS )

_Static_assert(MESH_WAVE_BUDGET_MS <= APP_PRIMARY_CAMPAIGN_MAX_MS,
               "APP_PRIMARY_MAX_WAVES cannot all run inside "
               "APP_PRIMARY_CAMPAIGN_MAX_MS at the depth-scaled wave floor + "
               "idle tail. Lower APP_PRIMARY_MAX_WAVES or the per-wave cost, or "
               "(last resort, costs fr9/TimeSync margin) raise the campaign "
               "budget.");

/* Secondary, flash backend only: once armed (see MESHNETWORK_vHandleTimeSync
 * auto-arm off the staged-fw version carried in TimeSync), how long to keep
 * listening for the OtaPrep the primary sends right after TimeSync in the
 * SAME wake (Fota's OTA_LORA_PREP_REPEATS * OTA_LORA_PREP_GAP_MS = 5 s of
 * repeats) before giving up and proceeding to sleep like normal. Bounded
 * deliberately short (not the whole distribute session) so an unarmed-
 * this-wake secondary never pays for it, and an armed one that doesn't hear
 * a prep this wake just tries again next wake. */
#define APP_OTA_PREP_WAIT_MS               15000U

/* GPS pre-trigger lead time: how many seconds before each scheduled wake the
 * wake-schedule task asks the GPS module for a fresh fix. The dispatcher runs
 * asynchronously; the AppTask never blocks waiting for it. */
/* 180 -> 150 s. A beacon may only carry a fix younger than
 * MESH_GPS_FIX_MAX_AGE_S (300 s), and the fix is taken by this pre-trigger, so
 * a 180 s lead left beacons GPS-valid only until about wake+120 s - the field
 * logs catch a tag flipping to gps=0 at wake+147 s mid-campaign, and its row
 * in the primary's union then carries Lat:0 Lon:0. Since this change set
 * deliberately lets a campaign run longer to find the deep tags, the fix has
 * to stay valid longer, and making it FRESHER at wake is the way to do that
 * without widening the staleness gate. 150 s still clears the 120 s TTFF
 * timeout the GPS session runs with (observed TTFF here: 8-34 s). */
#define DEVICE_DISCOVERY_GPS_PRETRIGGER_S   150U          /* 2.5 minutes            */

/* Basic-mode primary passive-listen cadence + duration. Independent of
 * WakeupInterval: the primary opens a 60 s RX window every 15 min and
 * accumulates heard BasicBeacons into a RAM store; the accumulated
 * store is then flushed to fr9 at each WakeupInterval boundary and
 * cleared. If WakeupInterval == 15 min the two coincide on every wake;
 * for larger intervals (30/60/120) the primary listens 2/4/8 times
 * per flush. See the basic-mode branch of DEVICE_DISCOVERY_vAppTask. */
#define DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S    (15U * 60U)
/* Primary listen window on the 15-min cadence. Halved (60 s -> 30 s)
 * alongside the secondary's 10 s -> 5 s TX cadence: with beacons coming
 * twice as fast, the primary still gets several per device inside a
 * shorter window, at half the radio-on time. */
#define DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_S    30U
#define DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_MS   (DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_S * 1000U)

/* Basic-mode primary high-water mark: when the RAM store hits this many
 * unique devices, flush to fr9 mid-cycle (log-only — no timestamp sync,
 * no settings fetch, no TimeSync) so the table can keep accepting
 * beacons from further devices in the same 60 s listen. Set below
 * MESH_MAX_BASIC_NEIGHBORS (32) so a handful of extra beacons arriving
 * DURING the ~1 s flush still fit without being silently dropped. */
#define DEVICE_DISCOVERY_BASIC_HWM                28U

/* Secondary basic-mode TX efficiency: the secondary confines its jittered
 * beacon TX to a window straddling the primary's listen — starting
 * DEVICE_DISCOVERY_BASIC_TX_GUARD_S before the 15 min boundary and ending
 * the same guard after the (30 s) listen. The guard absorbs the primary's
 * ~5 s wake buffer plus RTC drift between nodes. Phase is measured against
 * BASIC_LISTEN_PERIOD_S (the primary's fixed listen cadence), NOT the
 * secondary's own WakeupInterval. Outside this window the radio stays
 * asleep. Guard halved (10 s -> 5 s) alongside the listen window and
 * the 5 s TX cadence to keep the ratio of on-to-off time consistent. */
#define DEVICE_DISCOVERY_BASIC_TX_GUARD_S         5U

/* Kernel wakeup (shake-sequence): how long to wait for the user to start a
 * FrKernel session (send any "tag ..." command) before giving up and letting
 * the device return to deep sleep. 3 min so an operator has time to type
 * after the ~6 s shake sequence, accounting for the ~5 s -devicereq reply
 * jitter (see the LoRa spread in FrKernel.c). Once a session is actually
 * open FrKernel's own FRKERNEL_INACTIVITY_TIMEOUT_MS (5 min) is the real
 * "close this session" gate. */
#define DEVICE_DISCOVERY_KERNEL_WAKEUP_WINDOW_MS  (180 * 1000)  /* 3 min */

#define LOST_PRIMARY_TIMEOUT_MIN            480           /* ~8 hours before recovery    */

/* ---- Tiered recovery configuration ----
 *
 * Define ENABLE_LOW_POWER_RECOVERY (fr_app/inc/config/build_config.h) to
 * enable the recovery-mode check and DEVICE_DISCOVERY_vRecoveryMode() state
 * machine below. Leave undefined while time-sync is being driven from
 * another source (e.g. GPS) where a missed-primary recovery cycle is not
 * needed. */

/* Silence thresholds (seconds without a TimeSync) that select each tier.
 * Tier 1 (Sniff) starts at LOST_PRIMARY_TIMEOUT_MIN (8 h). */
#define RECOVER_SILENCE_SOFT_S       ((uint64_t)(24UL * 3600UL))   /* 24 h → soft tier    */
#define RECOVER_SILENCE_SPARSE_S     ((uint64_t)(72UL * 3600UL))   /* 3 d  → sparse tier  */
#define RECOVER_SILENCE_DEEP_S       ((uint64_t)(7UL * 24UL * 3600UL)) /* 7 d → deep tier */

/* Tier 1 – Sniff: 10 % duty cycle (2 s on / 18 s off), passive RX only.
 * Any discovery packet (DReq/DBeacon/DAck/TimeSync) heard in the 2-s window
 * escalates to 100 % duty cycle.  In 100 % mode the radio stays on as long
 * as mesh activity continues; 20 s of silence drops back to sniff.
 * TimeSync received in either phase exits recovery immediately.
 * RECOVER_SNIFF_CYCLES caps the per-wake sniff budget (30 × 20 s ≈ 10 min). */
#define RECOVER_SNIFF_RX_MS          2000U              /* sniff window duration        */
/* Off-period is jittered to spread collisions and de-correlate fleet timing.
 * 13–23 s of sleep + the 2 s RX window → 15–25 s total cycle (avg 20 s, 10 %). */
#define RECOVER_SNIFF_SLEEP_MIN_MS   13000U             /* min off period between sniffs */
#define RECOVER_SNIFF_SLEEP_MAX_MS   23000U             /* max off period between sniffs */
#define RECOVER_SNIFF_CYCLES         30U                /* max sniff cycles per wake    */
#define RECOVER_ESCALATE_POLL_MS     1000U              /* activity poll interval in 100% mode */
#define RECOVER_ESCALATE_IDLE_MS     20000U             /* 20 s idle → drop back to sniff     */

/* Tier 2 – Soft: 5 × 30 s probes separated by interval/5 gaps. */
#define RECOVER_SOFT_WALK_N          5U                 /* linear offset walk steps    */
#define RECOVER_SOFT_RX_MS           30000U             /* RX window per probe         */

/* Tier 3 – Sparse: 30 s scan; every other wake is skipped (≈ 2× interval). */
#define RECOVER_SPARSE_RX_MS         30000U             /* RX window per active wake   */

/* Tier 4 – Deep probe: 60 s scan every 6 h; all other wakes skipped. */
#define RECOVER_DEEP_RX_MS           60000U             /* RX window per probe         */
#define RECOVER_DEEP_PERIOD_S        ((uint64_t)(6UL * 3600UL)) /* probe period         */

/* osEventFlags bit — set by wakeup-schedule task to start a campaign */
#define DISCOVERY_WAKEUP_BIT                (1UL << 0)

/* osEventFlags bit — set by Movement to bypass deep sleep into FrKernel */
#define DISCOVERY_KERNEL_BIT                (1UL << 1)

/* osEventFlags bit — set by wakeup-schedule task in basic mode when the
 * ~10s jittered beacon-TX cadence fires. The AppTask handles this with a
 * short TX-only path (send one MeshPktType_BasicBeacon, radio back to
 * sleep, release lock) — no discovery campaign, no RX. See the basic-
 * mode branch in DEVICE_DISCOVERY_vAppTask and its scheduling in
 * DEVICE_DISCOVERY_vCheckWakeupScheduleTask. */
#define DISCOVERY_BASIC_BEACON_BIT          (1UL << 2)

/* Thread flag bit — set by MeshNetwork when a TimeSync packet is received */
#define DEVICE_DISCOVERY_NOTIFY_TIMESYNC    (1UL << 0)

/* Thread flag bit — set by OtaUpdate when OTA session traffic needs the
 * AppTask (an OtaPrep announcement, or a chunk in the receive mailbox) */
#define DEVICE_DISCOVERY_NOTIFY_OTA         (1UL << 1)

/* ---- Logger driver macros ---- */
#define DEVICE_DISCOVERY_DRIVER_bConnectLogger()             FARMRANGER_bDeviceOn()
#define DEVICE_DISCOVERY_DRIVER_vDisconnectLogger()          FARMRANGER_vDeviceOff()
#define DEVICE_DISCOVERY_DRIVER_u64RequestTS()               FARMRANGER_u64RequestTimestamp()
#define DEVICE_DISCOVERY_DRIVER_bRequestSettings(pi, pm, pg) FARMRANGER_bRequestSettings((pi), (pm), (pg))
#define DEVICE_DISCOVERY_bSendDiscoveryData(items, size)     FARMRANGER_bLogData(items, size)

/* ---- Production sleep state ---- */
typedef enum {
    PRODUCTION_READY  = 0,  /* initial state — normal operation (dev/test) */
    PRODUCTION_SLEEP  = 1,  /* super deep sleep, waiting for solar activation */
    PRODUCTION_ACTIVE = 2,  /* solar-activated, full discovery running */
} ProductionState_e;

/* ---- Public API ---- */
void               DEVICE_DISCOVERY_vInit(void);
void               DEVICE_DISCOVERY_vAppTask(void *pvParameters);
void               DEVICE_DISCOVERY_vConfigDeviceRole(void);
DeviceRole_e       DEVICE_DISCOVERY_eGetDeviceRole(void);
osThreadId_t       DEVICE_DISCOVERY_xGetTaskHandle(void);
void               DEVICE_DISCOVERY_vTriggerKernelWakeup(void);

/* Super-deep sleep (secondary only): discovery, acc logging and scheduled
 * wakes all stop; the STOP2 indicator goes yellow. The two variants differ
 * only in what may wake the device again:
 *   vEnterProductionSleep — a rising panel level OR the shake sequence.
 *   vEnterSolarSleep      — the shake sequence ONLY, so a flat unit can take
 *                           a full charge off the panel with the whole system
 *                           disabled instead of waking as soon as the sun
 *                           hits it.
 * Both report PRODUCTION_SLEEP via eGetProductionState(). */
void               DEVICE_DISCOVERY_vEnterProductionSleep(void);
void               DEVICE_DISCOVERY_vEnterSolarSleep(void);
ProductionState_e  DEVICE_DISCOVERY_eGetProductionState(void);

#endif /* TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_ */
