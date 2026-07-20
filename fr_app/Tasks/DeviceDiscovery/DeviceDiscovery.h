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
#define APP_DISCOVERY_WINDOW_TIMEOUT_MS     (180 * 1000)  /* hard cap on a campaign      */

/* Secondary campaign-end (R3): once the mesh has been silent (no discovery
 * packet from any primary) for this long AND the node is not beaconing, end the
 * campaign instead of waiting out the 180 s hard cap. Polled at this cadence. */
#define APP_SECONDARY_SILENCE_MS           (10 * 1000)   /* radio-silence end window    */
#define APP_SECONDARY_POLL_MS              250U          /* secondary wait poll cadence */

/* Primary issues at most this many DReq waves per campaign (R8). */
#define APP_PRIMARY_MAX_WAVES              5U

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
#define DEVICE_DISCOVERY_GPS_PRETRIGGER_S   180U          /* 3 minutes              */

/* Basic-mode primary passive-listen cadence + duration. Independent of
 * WakeupInterval: the primary opens a 60 s RX window every 15 min and
 * accumulates heard BasicBeacons into a RAM store; the accumulated
 * store is then flushed to fr9 at each WakeupInterval boundary and
 * cleared. If WakeupInterval == 15 min the two coincide on every wake;
 * for larger intervals (30/60/120) the primary listens 2/4/8 times
 * per flush. See the basic-mode branch of DEVICE_DISCOVERY_vAppTask. */
#define DEVICE_DISCOVERY_BASIC_LISTEN_PERIOD_S    (15U * 60U)
#define DEVICE_DISCOVERY_BASIC_LISTEN_WINDOW_MS   (60U * 1000U)

/* Kernel wakeup (shake-sequence): how long to wait for the user to start a
 * FrKernel session (send any "tag ..." command) before giving up and letting
 * the device return to deep sleep. */
#define DEVICE_DISCOVERY_KERNEL_WAKEUP_WINDOW_MS  (60 * 1000)  /* 60 seconds */

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

void               DEVICE_DISCOVERY_vEnterProductionSleep(void);
ProductionState_e  DEVICE_DISCOVERY_eGetProductionState(void);

#endif /* TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_ */
