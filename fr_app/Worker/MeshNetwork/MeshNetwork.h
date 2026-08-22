/*
 * MeshNetwork.h
 *
 * LoRa mesh network layer — types, packet definitions and public API.
 *
 * Uses CMSIS-RTOS v2 throughout.
 * All TickType_t references replaced with uint32_t (1 tick == 1 ms).
 */

#ifndef WORKER_MESHNETWORK_MESHNETWORK_H_
#define WORKER_MESHNETWORK_MESHNETWORK_H_

#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Timing constants ---- */
#define MESH_BEACON_INTERVAL_MS       3500U   /* beacon retry period while awaiting ACK */
#define MESH_PRIMARY_ACK_INTERVAL_MS  2000U
/* Beacon silence that ends one DReq wave on the primary. Was 7000: with the
 * frontier advancing one ring per wave (see MESHNETWORK_vHandleDReq), a spread
 * herd needs several waves and 7 s of dead air each was the bulk of a 100 s
 * campaign. Must stay above MESH_TX_JITTER_MAX_MS (a beacon is queued with up
 * to that much jitter, so a shorter idle could end the wave before the only
 * node in a sparse outer ring has transmitted) — asserted in MeshNetwork.c. */
#define MESH_DISCOVERY_IDLE_MS        3000U
/* Dedup window. R5 (meshOptimise) raises this to 64 to cut re-forward storms in
 * large fleets, but +128 B does not fit the current RAM budget, so it stays
 * small. Revisit with a RAM reclaim before scaling the fleet. (uint8
 * head/count allow up to 255.)
 *
 * 32 -> 24, and now BEACON/ACK ids only: DReq ids moved out to their own store
 * (MESH_DREQ_DEDUPE_SIZE below), so this ring no longer has to hold them and
 * the 8 slots they used to occupy pay for that store exactly. Net zero RAM on
 * a part whose .bss is byte-exact full, and strictly better behaviour — the
 * two id classes can no longer evict one another. */
#define FORWARD_RING_SIZE             24

/* DReq ids get their own dedup store rather than sharing the ring above. The
 * ring carries beacon and ack ids too, and one campaign pushes far more of
 * those through it than it has slots (28 nodes x up to 6 beacons per wave),
 * so a DReq id is evicted within a fraction of a second of busy traffic. That
 * is survivable while only forwarders relay DReqs, but the wave-1 flood has
 * every node relaying — an evicted id would let the same DReq be re-forwarded
 * on each lap around the mesh. 8 entries covers APP_PRIMARY_MAX_WAVES with
 * margin and cannot be displaced by beacon/ack churn. */
#define MESH_DREQ_DEDUPE_SIZE         8U
/* 120 (was 128): freed 160 B of .bss for the superOptimise fixes on a part
 * whose RAM was byte-exact full. Still far beyond a realistic per-primary
 * fleet — D-Acks carry 8 ids per 2 s, so even 120 nodes need ~30 s of ack
 * airtime per campaign. */
#define MESH_MAX_NEIGHBORS            120

/* A beaconing node stops (becomes forwarder) after this many beacons — bounds
 * airtime if its D-Ack is silently lost. This is a PER-WAVE budget: the
 * primary's next DReq wave re-arms a node that was never acked, so hitting it
 * is a backoff, not a giveup for the whole campaign.
 * (A parallel 30 s duration cap used to sit alongside this; at the 3.5 s
 * beacon interval 6 beacons take ~21 s, so the count always won and the timer
 * never once decided anything. Removed rather than left as dead config.) */
#define MESH_MAX_BEACONS_PER_CAMPAIGN 6U

/* TX jitter window — wide enough to de-correlate many nodes answering one DReq.
 * Max stays well under MESH_BEACON_INTERVAL_MS so a jittered beacon never slips
 * past the next interval. */
#define MESH_TX_JITTER_MIN_MS         20U
#define MESH_TX_JITTER_MAX_MS         1500U

/*
 * Per-packet verbose text logging. During a campaign every node hears every
 * neighbour's (re)transmissions, so the per-packet plumbing lines — the
 * "Queued TX"/"TX (len=)" duplicates of "Transmitting ..." and the
 * "... seen before" dedup hits — multiply O(N) across the fleet and are the
 * dominant flash-log/ring-buffer noise. Off by default: the event narrative
 * (received/forwarded/sent, beacon start/stop, timesync applied) and a single
 * "Transmitting <type> len=" per TX are still logged. Define this for bench
 * debugging of the TX queue / dedup behaviour.
 */
// #define MESH_LOG_VERBOSE

/* ---- Packet types (wire, first byte) ---- */
typedef enum {
    MeshPktType_Reserved  = 0,
    MeshPktType_DReq      = 1,
    MeshPktType_DBeacon   = 2,
    MeshPktType_DAck      = 3,
    MeshPktType_TimeSync  = 4,
    MeshPktType_FrKernel  = 5,   /* FrKernel command / response */
    /* OTA firmware distribution (DIRECT LoRa, never mesh-forwarded) —
     * formats and state machines in fr_app/Worker/OtaUpdate. */
    MeshPktType_OtaPrep    = 6,  /* primary bcast: session announcement    */
    MeshPktType_OtaPrepAck = 7,  /* secondary: joins the session           */
    MeshPktType_OtaChunk   = 8,  /* primary bcast: one image chunk         */
    MeshPktType_OtaPoll    = 9,  /* primary: request one target's report   */
    MeshPktType_OtaReport  = 10, /* secondary: missing bitmap / verdict    */

    MeshPktType_BasicBeacon = 11 /* secondary in basic mode: TX-only beacon
                                   * at ~10 s (± jitter). Carries cached
                                   * last-known GPS + age (no live fix),
                                   * no dreq/hops/wave. See
                                   * MeshPktBasicBeacon_t below. */
} MeshPktType_e;

/* ---- Wake-up interval enum ---- */
typedef enum {
    WAKEUP_INTERVAL_15_MIN  = 1,
    WAKEUP_INTERVAL_30_MIN  = 2,
    WAKEUP_INTERVAL_60_MIN  = 3,
    WAKEUP_INTERVAL_120_MIN = 4,
    WAKEUP_INTERVAL_240_MIN = 5,
    WAKEUP_INTERVAL_MAX_COUNT = 5
} WakeupInterval;

/* ---- Discovery mode enum ----
 *   ADVANCED: full mesh campaign — DReq waves, per-node beacon-on-DReq,
 *             forwarding. Original behavior.
 *   BASIC   : each secondary independently TX-beacons at ~10 s (± jitter);
 *             the primary passively listens for 60 s every 15 min and
 *             accumulates a RAM store, flushed to the fr9 at each
 *             WakeupInterval boundary. No DReq campaign, no forwarding.
 *
 * The mode is a system-wide setting owned by fr9 (movementAlarm.
 * nightZoneLevels.holdSecond.value), fetched by the primary via
 * AT+SETREQ, and distributed to secondaries in every TimeSync — a
 * secondary applies the received mode before its next scheduled wake.
 * Cold-boot default on every node is ADVANCED (safe fallback with full
 * mesh info) until the first TimeSync flips it. */
typedef enum {
    DISCOVERY_MODE_ADVANCED = 0,
    DISCOVERY_MODE_BASIC    = 1
} DiscoveryMode_e;

/* ---- On-wire packet structs ---- */
typedef struct {
    uint32_t u32DreqId;
    uint32_t u32DeviceId;
    uint16_t u16BatMv;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint32_t u32BeaconMsgId;
    uint8_t  dreqWaveDisc;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* sender's VERSION_SW_PATCH — logged per-neighbor
                              * so "did unit X pick up the latest OTA" is
                              * answerable from the fr9 flash log without
                              * touching each device with `tag <ID> fwver`. */
    bool     bGpsValid;      /* true if i32Lat/Lon hold a real fix */
    int32_t  i32LatUDeg;     /* latitude  in microdegrees (10^-6 deg) */
    int32_t  i32LonUDeg;     /* longitude in microdegrees (10^-6 deg) */
} MeshPktDBeacon_t;

#define MESH_MAX_ACK_IDS_PER_PACKET 8
typedef struct {
    uint32_t u32AckMsgId;
    uint32_t u32DreqId;
    uint32_t u32SenderId;
    uint8_t  u8AckCount;
    uint32_t u32AckedIds[MESH_MAX_ACK_IDS_PER_PACKET];
} MeshPktDAck_t;

/* Basic-mode beacon (see MeshPktType_BasicBeacon). Populated on the
 * secondary from cached last-known GPS + its age (no live fix); no dreq,
 * no hops, no wave. u32BeaconMsgId is monotonic per-boot per-device and
 * lets the primary's RAM store discard stale re-arrivals (newer-msgid
 * wins). */
typedef struct {
    uint32_t u32DeviceId;
    uint32_t u32BeaconMsgId;
    uint16_t u16BatMv;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* sender's VERSION_SW_PATCH */
    bool     bGpsValid;      /* true if i32Lat/Lon hold a cached fix */
    int32_t  i32LatUDeg;     /* latitude  in microdegrees (10^-6 deg) */
    int32_t  i32LonUDeg;     /* longitude in microdegrees (10^-6 deg) */
    uint32_t u32GpsAgeS;     /* age in seconds of that cached fix (only
                              * meaningful when bGpsValid) */
    int16_t  i16Rssi;        /* only set on receive */
} MeshPktBasicBeacon_t;

typedef struct {
    uint32_t     u32UtcTimestamp;
    WakeupInterval tWakeupInterval;
    uint32_t     u32StagedFwVersion;  /* MMmmpp of the image in the primary's
                                        * ext-flash scratchpad, 0 if none
                                        * valid — see FOTA_bGetMeta(). Lets
                                        * every secondary that hears the
                                        * end-of-campaign TimeSync learn
                                        * whether an update is available
                                        * without a dedicated OtaPrep round
                                        * trip. */
    DiscoveryMode_e eMode;            /* system-wide discovery mode — see
                                        * DiscoveryMode_e above */
    bool         bGpsEnabled;         /* system-wide GPS-active flag. false
                                        * turns every GPS_vRequestFix() into
                                        * a no-op (cached last-known fix
                                        * keeps aging). */
} MeshPktTimeSync_t;

/* ---- Forward ring ---- */
typedef struct {
    uint32_t u32Ring[FORWARD_RING_SIZE];
    uint8_t  u8Head;
    uint8_t  u8Count;
} ForwardRing_t;

/* ---- Neighbor entry (internal) ----
 * Field order and the u8MoveState/bGpsValid bitfields keep this at 20 bytes
 * (vs. 24 with naive ordering) — tNeighborTable[MESH_MAX_NEIGHBORS] makes
 * every byte here cost 128x in a 64K-RAM part. */
typedef struct {
    uint32_t u32DeviceId;
    int32_t  i32LatUDeg;     /* latitude  in microdegrees */
    int32_t  i32LonUDeg;     /* longitude in microdegrees */
    int16_t  i16Rssi;
    uint16_t u16BatMv;
    uint8_t  u8HopCount;
    uint8_t  u8DreqWaveDiscovered;
    uint8_t  u8FwPatch;        /* sender's VERSION_SW_PATCH from beacon */
    uint8_t  u8MoveState : 1;  /* 0 = moving, 1 = still */
    uint8_t  bGpsValid   : 1;  /* 1 = i32Lat/LonUDeg hold a fix */
    bool     bAcked;
} NeighborEntry_t;

/* ---- Node role ---- */
typedef enum {
    NODE_ROLE_UNKNOWN   = 0,
    NODE_ROLE_BEACONING = 1,
    NODE_ROLE_FORWARDER = 2
} NodeRole_e;

/* ---- Device role ---- */
typedef enum {
    DEVICE_ROLE_UNKNOWN   = 0,
    DEVICE_ROLE_PRIMARY   = 1,
    DEVICE_ROLE_SECONDARY = 2
} DeviceRole_e;

/* ---- Discovered neighbor (application interface) ---- */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint16_t u16BatMv;
    uint8_t  u8Wave;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* neighbor's VERSION_SW_PATCH from beacon */
    bool     bGpsValid;
    int32_t  i32LatUDeg;     /* latitude  in microdegrees */
    int32_t  i32LonUDeg;     /* longitude in microdegrees */
} MeshDiscoveredNeighbor_t;

/* ---- Public API ---- */
void MESHNETWORK_vInit(void);
void MESHNETWORK_vParserTask(void *pvParameters);

bool MESHNETWORK_bStartDiscoveryRound(uint32_t u32DreqId);
void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval,
                               uint32_t u32StagedFwVersion,
                               DiscoveryMode_e eMode,
                               bool bGpsEnabled);

/* Discovery-mode + GPS-enable state. Set by the primary from the fr9's
 * AT+SETREQ response and by every secondary from the last received
 * TimeSync. Getters return the currently-applied value; cold-boot default
 * is ADVANCED + GPS enabled. */
void            MESHNETWORK_vSetDiscoveryMode(DiscoveryMode_e eMode);
DiscoveryMode_e MESHNETWORK_eGetDiscoveryMode(void);
void            MESHNETWORK_vSetGpsEnabled(bool bEnabled);
bool            MESHNETWORK_bGetGpsEnabled(void);

/* Stop whatever dreq this node is currently beaconing (secondary campaign end —
 * the caller doesn't know the node's internal beacon dreq id). */
void MESHNETWORK_vStopBeaconingSelf(void);
bool MESHNETWORK_bIsBeaconing(void);     /* true while this node is beaconing */

/* True once this node has heard a DReq this campaign — directly or relayed,
 * any wave. Positive proof that a campaign is running and that this node is
 * inside its footprint, which is what the wave-1 flood exists to deliver to
 * nodes the frontier has not reached yet. DeviceDiscovery uses it to keep the
 * radio on instead of timing out on APP_SECONDARY_SILENCE_MS while it waits
 * for its ring's turn. A node that has heard nothing keeps the old behaviour
 * exactly, so out-of-range units cost no extra power. Cleared by
 * MESHNETWORK_vResetNodeRole() at campaign end. */
bool MESHNETWORK_bCampaignHeard(void);
void MESHNETWORK_vFlushTxQueue(void);    /* drop any pending TX (call on campaign wake) */
bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *pBuffer,
                                         uint16_t u16MaxEntries,
                                         uint16_t *pu16ActualEntries);
void MESHNETWORK_vClearDiscoveredNeighbors(void);

/* Basic-mode beacon TX (secondary). Reads local BAT / MOVE / VERSION_SW_PATCH
 * / GPS_bGetLastKnownFix and TX-only broadcasts a MeshPktType_BasicBeacon.
 * No RX, no wait — sends and returns. Called from the basic-mode 10 s
 * wake in DeviceDiscovery. */
void MESHNETWORK_vSendBasicBeacon(void);

/* Basic-mode primary RAM store: one entry per unique DeviceId, updated
 * on each MeshPktType_BasicBeacon received during the 15-min listen
 * window (newer BeaconMsgId wins, older is ignored). Flushed to the fr9
 * at each WakeupInterval boundary via FARMRANGER_bLogBasicData and then
 * cleared. Column set differs from the advanced-mode neighbor table (no
 * hops / wave / RSSI, has GPS-age) so it uses its own struct + row
 * format on the Farmranger side. */
/* Field order + the u8MoveState/bGpsValid bitfields keep this at 24 bytes
 * (vs. 28 with naive ordering) — same tight-packing this file already
 * applies to NeighborEntry_t, needed because this part's RAM is byte-exact
 * full (see FORWARD_RING_SIZE's comment above). tBasicNeighborTable
 * [MESH_MAX_BASIC_NEIGHBORS] makes every byte here cost 32x. Adding
 * i16Rssi bumped this from 24 to 28 bytes (worst-case pad). */
typedef struct {
    uint32_t u32DeviceId;
    uint32_t u32BeaconMsgId;  /* update key: incoming replaces stored only
                                * when incoming > stored */
    int32_t  i32LatUDeg;
    int32_t  i32LonUDeg;
    uint32_t u32GpsAgeS;
    uint16_t u16BatMv;
    int16_t  i16Rssi;         /* best (least-negative) RSSI heard across
                                * all beacons from this device this cycle */
    uint8_t  u8FwPatch;
    uint8_t  u8MoveState : 1;
    uint8_t  bGpsValid   : 1;
} MeshBasicNeighbor_t;

/* Cap on the basic-mode RAM store (small deliberately — basic mode is
 * for small remote groups). Callers stack-allocate a copy at this size
 * before calling MESHNETWORK_bGetBasicNeighbors, so it lives in the
 * public header. */
#define MESH_MAX_BASIC_NEIGHBORS    32U

bool MESHNETWORK_bGetBasicNeighbors(MeshBasicNeighbor_t *pBuffer,
                                    uint16_t u16MaxEntries,
                                    uint16_t *pu16ActualEntries);
void MESHNETWORK_vClearBasicNeighbors(void);

/* Cheap count of unique nodes currently in the basic-mode RAM store —
 * avoids stack-copying the whole table just to log a count. */
uint16_t MESHNETWORK_u16GetBasicNeighborCount(void);

uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void);

void          MESHNETWORK_vSetWakeupInterval(WakeupInterval tNewInterval);
WakeupInterval MESHNETWORK_tGetWakeupInterval(void);
uint8_t       MESHNETWORK_u8GetWakeupInterval(void);

/* Called by the parser when a FrKernel packet arrives.
 * Override in FrKernel.c when FRKERNEL_INTERFACE_LORA is defined. */
void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len);

/* Enqueue a small OTA response (PrepAck/Report, <= the mesh TX item size)
 * through the jittered mesh TX queue. Parser-context safe — used by the
 * OtaUpdate packet handlers to answer the primary without a new TX path. */
bool MESHNETWORK_bSendOtaResponse(const uint8_t *buf, uint16_t len);

uint32_t MESHNETWORK_u32GetLastBeaconHeardTick(void);
uint32_t MESHNETWORK_u32GetLastDiscoveryPktTick(void);  /* any DReq/DBeacon/DAck/TimeSync */

uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void);
void     MESHNETWORK_vUpdatePrimaryLastSeen(void);

void MESHNETWORK_vStartPrimaryAck(void);
void MESHNETWORK_vStopPrimaryAck(void);

/* Reset per-wake "first TimeSync only" gate.
 * Call at the top of each wake cycle from DeviceDiscovery. */
void MESHNETWORK_vResetTimeSyncAccepted(void);

void MESHNETWORK_vResetNodeRole(void);
void MESHNETWORK_vIncrDreqWaveCnt(void);
void MESHNETWORK_vResetDreqWaveCnt(void);

/* One-line traffic summary (DReq/beacons/acks heard, messages forwarded)
 * since the last MESHNETWORK_vResetDreqWaveCnt() — call at campaign end. */
void MESHNETWORK_vLogCampaignStats(const char *pcTag);

#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */
