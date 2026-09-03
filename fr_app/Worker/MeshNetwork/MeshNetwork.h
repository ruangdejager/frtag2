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
/* Beacon retry cadence while awaiting a D-Ack. Was one fixed 3500 ms period.
 *
 * A fixed period made an UNACKED node the loudest thing on the mesh for the
 * whole 180 s window: field logs show one hop-3 tag emitting 48 beacons in
 * 171 s, and the forwarder in front of it relaying 89 packets in the same
 * campaign. Every one of those transmissions costs the relay up to
 * MESH_TX_JITTER_MAX_MS of queue jitter plus up to LORA_TX_CARRIER_WAIT_MS of
 * carrier sense, and carrier sense runs the chip in CAD, not RX - so the relay
 * was deaf for the majority of the window and dropped both the D-Ack meant for
 * the node behind it and its own TimeSync. That is congestion collapse, and
 * the node beaconing hardest is what drives it.
 *
 * The cadence now backs off within one beaconing episode:
 *   interval(n) = min(BASE + n * STEP, MAX),  n = beacons already sent
 * so the first answers to a DReq are still prompt (that is when the primary is
 * listening and an ack is most likely) and the tail of an unheard node decays
 * to MAX instead of hammering. Same 171 s window: ~17 beacons instead of 48.
 * n resets to 0 every time beaconing (re-)starts - a new wave, a re-anchor
 * onto a newer dreq, or a re-arm - so each wave gets a fast first beacon; see
 * MESHNETWORK_vStartBeaconing.
 *
 * BASE is deliberately above both MESH_TX_JITTER_MAX_MS and
 * MESH_DREQ_FWD2_DELAY_MAX_MS (asserted in MeshNetwork.c) so a queued packet
 * still cannot slip past the NEXT beacon - the invariant the old single
 * constant carried. MAX is NOT bounded by MESH_DISCOVERY_IDLE_MS and cannot
 * be: the gap the primary observes is the period plus both ends' jitter and
 * carrier sense, up to ~6.5 s more, so no affordable idle window covers a 12 s
 * cadence. Wave end therefore no longer relies on beacon silence alone - see
 * MESH_DISCOVERY_IDLE_MS below. */
#define MESH_BEACON_BASE_MS           5000U
#define MESH_BEACON_STEP_MS           2000U
#define MESH_BEACON_MAX_MS            12000U

/* Primary D-Ack cadence. 2000 -> 4000: every tick emits one D-Ack carrying up
 * to MESH_MAX_ACK_IDS_PER_PACKET ids, but the sniffer logs show the packet was
 * never the constraint - across five campaigns not one D-Ack carried more than
 * 2 ids (81 with one id, 35 with two), because at 2 s the tick keeps outrunning
 * the arrival rate. Each of those near-empty acks is then re-flooded ~3x by the
 * forwarders. Holding twice as long roughly halves the ack packet count and
 * doubles the ids per packet for the same information.
 *
 * The extra hold is free only because MESH_BEACON_BASE_MS (5000) now exceeds
 * it: a node waiting one more ack period was going to be silent for that
 * period anyway, so no additional beacon is provoked. Do not raise this above
 * MESH_BEACON_BASE_MS - asserted in MeshNetwork.c. */
#define MESH_PRIMARY_ACK_INTERVAL_MS  4000U

/* Beacon silence that ends one DReq wave on the primary. 7000 -> 3000 -> 5000:
 * the history matters, because both previous values were wrong for the same
 * reason - beacon silence was the ONLY wave-end signal, so this one constant
 * had to serve as both "the herd has finished answering" and "nobody is still
 * mid-cadence", and no value does both.
 *
 * 7000 made 7 s of dead air the bulk of a ~100 s campaign. 3000 was below the
 * old 3500 ms beacon period, so a lone node's own inter-beacon gap
 * (3500 +/- jitter = 2.0..5.0 s) routinely read as silence and its wave was
 * declared over while it was still beaconing - visible in the sniffer logs as
 * 4 of 8 waves starting while beacons for the previous dreq were still
 * arriving, and at its worst as a campaign that ended after ONE wave and 3 s
 * with 0 neighbours while a 1-hop secondary was mid-cadence.
 *
 * Three changes remove the overload, and only then is a value pickable:
 *   - a wave cannot end before MESH_DISCOVERY_MIN_WAVE_MS (the DReq has to
 *     reach the air and a reply has to get back before silence means
 *     anything);
 *   - a wave does not end while the primary still has un-acked neighbours
 *     (MESHNETWORK_bHasUnackedNeighbors) - those are by definition nodes that
 *     are still beaconing, which is the condition this constant used to have
 *     to infer from timing and could not;
 *   - APP_PRIMARY_MIN_WAVES keeps the campaign alive through a barren wave.
 * So this is now only the tail timer: how long to keep a wave open after the
 * answers stop and everyone heard has been acked. 5000 covers one relay hop's
 * jitter at each end with margin.
 *
 * Must stay above MESH_TX_JITTER_MAX_MS (a beacon is queued with up to that
 * much jitter) - asserted in MeshNetwork.c. It deliberately does NOT try to
 * cover MESH_BEACON_MAX_MS; see there. */
#define MESH_DISCOVERY_IDLE_MS        5000U

/* Floor on one DReq wave. The wave clock starts when the DReq is ENQUEUED, and
 * between enqueue and air a packet waits out its own jitter (up to
 * MESH_TX_JITTER_MAX_MS), anything already queued ahead of it, and the radio
 * layer's carrier sense (up to LORA_TX_CARRIER_WAIT_MS, 5 s). The reply then
 * pays the same on the way back. With no floor, MESH_DISCOVERY_IDLE_MS could
 * elapse before the DReq had even been transmitted - which is exactly the 3 s
 * / 0 neighbour campaign in the field logs. 8000 covers both ends' jitter plus
 * a realistic carrier-sense wait; the absolute worst case is not covered by
 * any affordable floor, which is what the un-acked and min-wave rules above
 * are for. */
#define MESH_DISCOVERY_MIN_WAVE_MS    8000U

/* Added to the floor above for every ring of herd depth the primary has ALREADY
 * proven this campaign (MESHNETWORK_u8GetMaxDiscoveredWave).
 *
 * A flat floor is wrong because what it has to cover is not fixed: it is the
 * round trip out to the frontier and back, and that grows with depth. One
 * campaign, measured at a sniffer beside the primary:
 *
 *     wave  DReq sent   first beacon back          round trip
 *     1     03:32:45    03:32:47 (1316, ring 1)    2 s
 *     2     03:32:59    03:33:02 (3E1E, ring 2)    3 s
 *     3     03:33:11    03:33:15 (241F, ring 3)    4 s
 *     4     03:33:28    03:33:36 (2D94, ring 4)    8 s
 *
 * The 8 s floor expired on wave 4 in the same second 2D94's beacon arrived, so
 * that one tag missed the union by under a second while every shallower ring
 * made it. The outermost ring is precisely what the extra waves exist to
 * reach, so a flat floor fails the tags it is there for. Scaling means the
 * wave that goes looking for ring N+1 waits in proportion to how far out ring
 * N already is: with ring 3 on the table before wave 4 the floor becomes
 * 8000 + 3*2000 = 14000 ms, and that same beacon lands with 6 s to spare.
 *
 * 2000 rather than the ~1000 ms per ring the first three waves alone suggest,
 * because the round trip goes superlinear at the edge (2, 3, 4, then 8 s): the
 * outer rings are both further out and contending with every relay between
 * them and the primary.
 *
 * A tight herd pays almost nothing - one ring proven leaves the floor at
 * 10000 ms - which is the whole point of scaling it rather than just raising
 * the flat value: the campaigns that would need a 14 s floor are exactly the
 * ones that never see it. APP_PRIMARY_CAMPAIGN_MAX_MS bounds the total either
 * way. */
#define MESH_DISCOVERY_WAVE_ALLOWANCE_MS 2000U

/* Cap on the scaled floor, so no ring count can push a single wave past
 * MESH_DISCOVERY_WAVE_MAX_MS and leave no room for the idle tail underneath
 * it. Asserted against both in MeshNetwork.c. */
#define MESH_DISCOVERY_MIN_WAVE_CAP_MS  18000U

/* Ceiling on one DReq wave, so the un-acked extension cannot run a wave
 * forever: a node whose acks never reach it re-beacons (NEIGHBOR_vAddOrUpdate
 * clears bAcked on every re-beacon), so "un-acked neighbours exist" can stay
 * true for as long as that node keeps trying. Holding the wave open IS the
 * right response for a while - the primary keeps re-acking, and one ack that
 * lands stops 20+ beacons - but the campaign has a deadline to meet
 * (APP_PRIMARY_CAMPAIGN_MAX_MS), so bound it per wave and move on to the next
 * wave, which re-solicits anyway. */
#define MESH_DISCOVERY_WAVE_MAX_MS    25000U
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

/* Forwards permitted per DReq id. A DReq is the one packet the whole campaign
 * hangs on: lose it and a node never learns a campaign is running, never
 * beacons, and is not counted for the rest of the window. One relay per node
 * gave that packet a single chance to survive a collision, so each id now gets
 * two — the dedupe store counts forwards instead of just remembering the id.
 *
 * The two copies are driven by two separate RECEPTIONS of the same id (a flood
 * arrives from several neighbours), so the spacing is whatever the mesh gives,
 * plus MESH_DREQ_FWD2_EXTRA_*_MS below to keep the pair off one another. A node
 * that only ever hears an id once still forwards once, exactly as before.
 *
 * This applies to both relay paths — the wave-1 flood and the forwarder relay
 * of waves 2+. What stays unchanged is WHO relays: wave 1 by every node, waves
 * 2+ only by forwarders. */
#define MESH_DREQ_MAX_FORWARDS        2U
/* 120 (was 128): freed 160 B of .bss for the superOptimise fixes on a part
 * whose RAM was byte-exact full. Still far beyond a realistic per-primary
 * fleet — D-Acks carry 8 ids per 2 s, so even 120 nodes need ~30 s of ack
 * airtime per campaign. */
#define MESH_MAX_NEIGHBORS            120

/* A beaconing node used to stop (become forwarder) after MESH_MAX_BEACONS_PER_
 * CAMPAIGN (6) beacons, on the theory that it bounded airtime when a D-Ack was
 * silently lost. What it actually bounded was the node's chance of being found:
 * six beacons is ~17.5 s of a 180 s window, after which a node the primary was
 * still hunting for went mute and was not counted. A per-wave re-arm was bolted
 * on to claw some of that back, which only made the giving-up harder to follow.
 *
 * A secondary now beacons until it is ACKED, or until the campaign's 180 s hard
 * cap (APP_DISCOVERY_WINDOW_TIMEOUT_MS in DeviceDiscovery.h) ends the window —
 * that cap was always the real bound and is now the only one. The ack path
 * (MESHNETWORK_vStopBeaconingByOrigin) is unchanged, so a node that IS heard
 * still stops on the first ack and costs no more airtime than before.
 *
 * (A parallel 30 s duration cap once sat alongside the count; at the 3.5 s
 * beacon interval 6 beacons take ~21 s, so the count always won and the timer
 * never once decided anything. It was already removed.)
 *
 * What DID need bounding was the RATE, not the count - see the backoff at
 * MESH_BEACON_BASE_MS above. An unheard node still beacons to the end of the
 * window, but at a decaying cadence, so it keeps its chance of being found
 * without deafening the relay in front of it. */

/* TX jitter window - wide enough to de-correlate many nodes answering one DReq.
 * Max stays well under MESH_BEACON_BASE_MS (the SHORTEST beacon interval, and
 * therefore the binding one) so a jittered beacon never slips past the next
 * interval. Asserted in MeshNetwork.c. */
#define MESH_TX_JITTER_MIN_MS         20U
#define MESH_TX_JITTER_MAX_MS         1500U

/* Send delay for the SECOND forward of a DReq id. REPLACES the normal jitter
 * window above for that one packet (it is not added to it), so the two copies
 * cannot land inside one congestion window — a collision that killed copy 1 is
 * then unlikely to kill copy 2, which is the entire point of sending two.
 *
 * The window deliberately does not overlap [MIN, MAX] above, so the pair is
 * always separated by at least (1600 - 1500) = 100 ms and typically ~1 s. The
 * ceiling matters: the mesh TX queue is drained in order and each item blocks
 * on its own ready-tick, so a deferred forward holds up whatever is queued
 * behind it. 2600 ms worst case stays under MESH_BEACON_BASE_MS, keeping the
 * existing guarantee that a queued beacon never slips past its next interval.
 * Asserted in MeshNetwork.c. */
#define MESH_DREQ_FWD2_DELAY_MIN_MS   1600U
#define MESH_DREQ_FWD2_DELAY_MAX_MS   2600U

/* TimeSync is aired TWICE per node - once by the primary that originates it and
 * once by every node that relays it - using the same two-copy scheme as a DReq:
 * the ordinary jitter for copy 1 and the non-overlapping window above for
 * copy 2. Same reasoning, and a stronger case for it: TimeSync is the one
 * packet that ends a secondary's campaign, sets its clock and tells it a
 * firmware image is waiting, and it goes out exactly once per campaign, at the
 * moment the mesh is at its noisiest. Field logs: 5 of 21 device-wakes never
 * saw it, and in the worst campaign none of the three secondaries did.
 *
 * The receive-side dedup is untouched - it is keyed on the UTC value, so a node
 * decodes one TimeSync per campaign no matter how many copies reach it, and
 * still relays exactly two. No fabricated timestamp, no second origination
 * path, no extra state: total airings are bounded at 2 per node per UTC, the
 * same budget MESH_DREQ_MAX_FORWARDS already grants a DReq. */
#define MESH_TIMESYNC_AIRINGS         2U

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
    /* Which node's DReq produced i16Rssi above. i16Rssi is the reading of the
     * ONE DReq that triggered this beaconing episode - not the strongest heard
     * during it (that used to drift as later, unrelated receptions came in;
     * it is now latched at the trigger and frozen — see bBestDreqLatched in
     * MeshNetwork.c) - and the DReq that gave it may have come from the
     * primary directly or from any relaying peer, so the reading alone does
     * not say where this node's good path actually is. Carried as the 16-bit
     * device id of the immediate sender of that DReq (see MESH_DREQ_LEN_SRC).
     *
     * bValid is a separate flag rather than "u16 != 0" because 0x0000 is a
     * reachable device id, and because a beacon relayed by an older forwarder
     * arrives with the field genuinely absent — which must not be reported as
     * a real id of zero. */
    bool     bBestRssiSrcValid;
    uint16_t u16BestRssiSrcId;
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
 * Field order is load-bearing. The declared fields below occupy 21 bytes and
 * the struct is 24 (verified against .bss.tNeighborTable = 0xb40 = 120 x 24 in
 * the linked map), so there are 3 bytes of tail padding the compiler inserts
 * whatever we do. u16BestRssiSrcId is placed last deliberately: it lands at
 * offset 22 inside that existing padding and sizeof stays 24, which is the only
 * reason this field is affordable at all.
 *
 * That headroom is now spent. .bss is byte-exact full on this part — the linked
 * image leaves 8 bytes of the 64K RAM region — so a SECOND 2-byte field here,
 * or widening this one to uint32_t, pushes sizeof to 28 and costs 480 B, which
 * will not link. The static assert in MeshNetwork.c is what catches that. */
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
    uint16_t u16BestRssiSrcId; /* node whose DReq gave i16Rssi; 0 = not reported
                                * (older peer, or relayed via an older node) */
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

/* ---- Discovered neighbor (application interface) ----
 * Callers stack-allocate MESH_MAX_NEIGHBORS of these (DeviceDiscovery.c), so
 * sizeof is a 120x stack cost, not just a RAM one. u16BestRssiSrcId sits after
 * bGpsValid to fall in the 2-byte hole already there ahead of i32LatUDeg;
 * sizeof stays 24. */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint16_t u16BatMv;
    uint8_t  u8Wave;
    uint8_t  u8MoveState;    /* 0 = moving, 1 = still */
    uint8_t  u8FwPatch;      /* neighbor's VERSION_SW_PATCH from beacon */
    bool     bGpsValid;
    uint16_t u16BestRssiSrcId; /* node whose DReq gave i16Rssi; 0 = not reported */
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
/* Drop pending TX. Called on campaign wake and again at campaign end.
 * bKeepTimeSync re-queues a pending TimeSync instead of dropping it: pass true
 * at campaign end (a secondary's campaign ENDS on the TimeSync it must still
 * relay), false on the wake flush (a relay that survived deep sleep would air
 * a 15-minute-old timestamp). See the implementation. */
void MESHNETWORK_vFlushTxQueue(bool bKeepTimeSync);

/* Primary: does the neighbour table hold anyone this campaign has not put in a
 * D-Ack yet? True means at least one node is still beaconing at us and has no
 * reason to stop, so the wave it answered must not be declared over - that is
 * the direct signal MESH_DISCOVERY_IDLE_MS used to have to guess at from
 * timing. NEIGHBOR_vAddOrUpdate clears bAcked on every re-beacon, so this also
 * goes true again for a node whose acks are not reaching it. */
bool MESHNETWORK_bHasUnackedNeighbors(void);

/* Primary: the highest wave number that has discovered anything this campaign,
 * i.e. how many rings deep the herd has PROVEN to be; 0 before anything
 * answers. The wave loop scales its listen floor by this - see
 * MESH_DISCOVERY_WAVE_ALLOWANCE_MS.
 *
 * Reads u8DreqWaveDiscovered, which the table keeps at the FIRST wave that
 * found each node, so this is a true ring count. Deliberately NOT the hop
 * count: a beacon's hop field is the depth of whichever relayed COPY arrived
 * last, which routinely overshoots the ring - a 1-ring node's beacon reaches
 * the primary at hop 2, a 4-ring node's at hop 6-8 - and scaling a timer by an
 * inflated number would make every tight herd pay for a spread one. */
uint8_t MESHNETWORK_u8GetMaxDiscoveredWave(void);

/* TX-worker hooks for the runtime radio range test (RadioTestMode.c), whose
 * beaconing secondary has no task of its own and runs on the TX worker.
 * bTxWorkerReady lets it refuse to enter where there is no worker to run on
 * (an ENABLE_RADIO_TEST build skips MESHNETWORK_vInit); vWakeTxWorker breaks
 * the worker out of the indefinite wait it is in by the time the test's own
 * gates have silenced every other path to it. */
bool MESHNETWORK_bTxWorkerReady(void);
void MESHNETWORK_vWakeTxWorker(void);
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
