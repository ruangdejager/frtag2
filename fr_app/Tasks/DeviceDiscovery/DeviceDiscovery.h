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
#define APP_DISCOVERY_WINDOW_TIMEOUT_MS     (180 * 1000)  /* max discovery window        */

#define LOST_PRIMARY_TIMEOUT_MIN            480           /* ~8 hours before recovery    */

/* ---- Tiered recovery configuration ---- */

/* Silence thresholds (seconds without a TimeSync) that select each tier.
 * Tier 1 (Sniff) starts at LOST_PRIMARY_TIMEOUT_MIN (8 h). */
#define RECOVER_SILENCE_SOFT_S       ((uint64_t)(24UL * 3600UL))   /* 24 h → soft tier    */
#define RECOVER_SILENCE_SPARSE_S     ((uint64_t)(72UL * 3600UL))   /* 3 d  → sparse tier  */
#define RECOVER_SILENCE_DEEP_S       ((uint64_t)(7UL * 24UL * 3600UL)) /* 7 d → deep tier */

/* Tier 1 – Sniff: 2 s radio-on / 18 s radio-off duty cycle.
 * Any DBeacon heard in the 2-s window escalates to a 3-min full-RX window.
 * RECOVER_SNIFF_CYCLES caps the per-wake budget (30 × 20 s ≈ 10 min). */
#define RECOVER_SNIFF_RX_MS          2000U              /* sniff window duration       */
#define RECOVER_SNIFF_SLEEP_MS       18000U             /* off period between sniffs   */
#define RECOVER_SNIFF_CYCLES         30U                /* max sniff cycles per wake   */
#define RECOVER_ESCALATE_MS          (3U * 60U * 1000U) /* escalation RX window (3 min)*/

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

/* Thread flag bit — set by MeshNetwork when a TimeSync packet is received */
#define DEVICE_DISCOVERY_NOTIFY_TIMESYNC    (1UL << 0)

/* ---- Logger driver macros ---- */
#define DEVICE_DISCOVERY_DRIVER_bConnectLogger()             FARMRANGER_bDeviceOn()
#define DEVICE_DISCOVERY_DRIVER_vDisconnectLogger()          FARMRANGER_vDeviceOff()
#define DEVICE_DISCOVERY_DRIVER_u64RequestTS()               FARMRANGER_u64RequestTimestamp()
#define DEVICE_DISCOVERY_DRIVER_u8RequestInterval()          FARMRANGER_u8RequestInterval()
#define DEVICE_DISCOVERY_bSendDiscoveryData(items, size)     FARMRANGER_bLogData(items, size)

/* ---- Public API ---- */
void          DEVICE_DISCOVERY_vInit(void);
void          DEVICE_DISCOVERY_vAppTask(void *pvParameters);
void          DEVICE_DISCOVERY_vConfigDeviceRole(void);
DeviceRole_e  DEVICE_DISCOVERY_eGetDeviceRole(void);
osThreadId_t  DEVICE_DISCOVERY_xGetTaskHandle(void);

#endif /* TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_ */
