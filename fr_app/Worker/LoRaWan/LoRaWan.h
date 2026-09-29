/*
 * LoRaWan.h
 *
 * LoRaWAN uplink for DISCOVERY_MODE_LORAWAN (see MeshNetwork.h): each
 * secondary sends its own position/battery report to TTN over LoRaWAN
 * (EU868, 1.0.4, Class A, OTAA) instead of beaconing it to a primary.
 *
 * One radio, two owners. The mesh and LoRaWAN cannot share the chip - they
 * use different bandwidths, sync words and frequencies - so LoRaWAN only
 * ever holds it for one short, blocking session between two mesh wakes:
 *
 *   LORAWAN_bSendReport()  attach the radio to the LoRaMac driver (radio.c),
 *                          join if not yet joined, otherwise send one
 *                          unconfirmed report, run the RX1/RX2 windows, then
 *                          halt the MAC and hand the radio back to the mesh
 *                          driver, asleep, exactly as the mesh left it.
 *
 * It must be called from the DeviceDiscovery AppTask (the only task that
 * owns the radio between wakes) and only outside the mesh wake window - the
 * wake-schedule task guarantees both (see DEVICE_DISCOVERY_vCheckWakeup
 * ScheduleTask). Nothing of LoRaWAN runs between sessions: the MAC is
 * halted, its timers stopped.
 *
 * The session is RAM-only (no context save), so every reboot re-joins. The
 * LoRaWAN 1.0.4 DevNonce, which the join server requires to keep
 * increasing across those re-joins, is persisted in the last sector of the
 * external NOR (LORAWAN_NVM_FLASH_ADDR, carved off the text log - see
 * Log.h).
 */

#ifndef WORKER_LORAWAN_LORAWAN_H_
#define WORKER_LORAWAN_LORAWAN_H_

#include <stdint.h>
#include <stdbool.h>

/* ---- Schedule (used by DeviceDiscovery's wake-schedule task) ----
 *
 * One report every LORAWAN_UPLINK_PERIOD_S, aligned to UTC multiples of the
 * period so it lands just BEFORE a mesh wake boundary, never inside one:
 *
 *   B-120 s  GPS fix requested (LORAWAN_GPS_LEAD_S)
 *   ...      uplink as soon as the GPS session ends (fix or timeout)
 *   B-20 s   latest an uplink may START (LORAWAN_UPLINK_DEADLINE_LEAD_S)
 *   B        mesh wake (every WakeupInterval, a multiple of 15 min)
 *
 * A session is capped at LORAWAN_SESSION_MAX_MS, so the radio is always back
 * with the mesh before B. The GPS TTFF budget is what is left in between. */
#define LORAWAN_UPLINK_PERIOD_S             (15U * 60U)
#define LORAWAN_GPS_LEAD_S                  120U
#define LORAWAN_UPLINK_DEADLINE_LEAD_S      20U
#define LORAWAN_GPS_TTFF_TIMEOUT_S          (LORAWAN_GPS_LEAD_S - LORAWAN_UPLINK_DEADLINE_LEAD_S)

/* Hard cap on one session. Both kinds end with RX2 opening 6 s after the
 * uplink: a join because JoinAccept windows are fixed at +5/+6 s, a data
 * uplink because TTN sets RX1 to 5 s (so RX2 at +6 s) in the JoinAccept. At
 * SF12 that is ~1.5 s on air + 6 s + the RX2 window, ~8 s in all. */
#define LORAWAN_SESSION_MAX_MS              12000U

_Static_assert((LORAWAN_SESSION_MAX_MS / 1000U) + 5U <= LORAWAN_UPLINK_DEADLINE_LEAD_S,
               "A session started at the deadline must end with 5 s to spare "
               "before the mesh wake reclaims the radio.");

/* Thread flag the session waits on (radio IRQ / MAC process notify). Must
 * not collide with the AppTask's DEVICE_DISCOVERY_NOTIFY_* bits. */
#define LORAWAN_THREAD_FLAG                 (1UL << 3)

/* ---- Report payload, FPort 1, big endian ----
 *
 * Same fields the basic-mode beacon carried, minus its packet type and
 * message id (LoRaWAN's FPort and FCnt do those jobs):
 *
 *   [0..3]   DeviceId      mesh device id (LORARADIO_u32GetUniqueId), so
 *                          TTN data joins the mesh tables on the server
 *   [4..5]   BatMv
 *   [6]      flags         bit0 STILL (1 = still), bit1 GPS_VALID
 *   [7]      FwPatch       VERSION_SW_PATCH
 *   [8..11]  LatUDeg       int32 microdegrees   } only if GPS_VALID
 *   [12..15] LonUDeg       int32 microdegrees   }
 *   [16..19] GpsAgeS       age of that fix, s   }
 *
 * 8 or 20 bytes. docs: MESH_TECHNICAL_HANDOVER.md, "LoRaWAN mode". */
#define LORAWAN_REPORT_FPORT                1U
#define LORAWAN_REPORT_FLAG_STILL           0x01U
#define LORAWAN_REPORT_FLAG_GPS_VALID       0x02U
#define LORAWAN_REPORT_LEN_BASE             8U
#define LORAWAN_REPORT_LEN_GPS              20U

/* Run one LoRaWAN session: join if needed, else send one report. Blocking,
 * bounded by min(u32BudgetMs, LORAWAN_SESSION_MAX_MS). Returns true when the
 * report went out (or, on a join session, when the join succeeded). Must be
 * called with the radio otherwise idle (between mesh wakes). */
bool LORAWAN_bSendReport(uint32_t u32BudgetMs);

/* True once an OTAA join has succeeded this boot. */
bool LORAWAN_bIsJoined(void);

/* The DevEUI this tag joins with (8 bytes, big endian) - its STM32WL
 * factory UID64. What to type into TTN when registering the tag. */
void LORAWAN_vGetDevEui(uint8_t au8DevEui[8]);

/* Hook for downlinks (Class A: only ever right after one of our uplinks).
 * Weak no-op by default; nothing consumes downlinks yet. Runs in the
 * session's task context, inside LORAWAN_bSendReport. */
void LORAWAN_vOnDownlink(uint8_t u8Port, const uint8_t *pu8Data, uint8_t u8Len);

#endif /* WORKER_LORAWAN_LORAWAN_H_ */
