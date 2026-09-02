/*
 * MeshNetwork.c
 *
 * LoRa mesh network layer — discovery protocol, packet encoding/decoding,
 * neighbor table management and TX jitter queue.
 *
 * Uses CMSIS-RTOS v2 throughout:
 *   osMutex         — protects forward ring and neighbor table
 *   osMessageQueue  — TX jitter queue
 *   osTimer         — periodic beacon and primary-ACK timers
 *   osThreadNew     — parser and TX worker tasks
 *   osKernelGetTickCount instead of xTaskGetTickCount
 *   uint32_t instead of TickType_t (1 tick == 1 ms)
 *
 * The TimeSync handler notifies the DeviceDiscovery AppTask via
 * osThreadFlagsSet (DEVICE_DISCOVERY_NOTIFY_TIMESYNC).
 */

#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE */
#include "task.h"

#include <string.h>

#include "dbg_log.h"
#include "DeviceDiscovery.h"
#include "hal_rtc.h"
#include "Battery.h"
#include "flashLog.h"

#include "build_config.h"
#ifdef STORAGE_BACKEND_FLASH
#include "Fota.h"
#include "version_config.h"
#endif

#include "GPS.h"
#include "Movement.h"
#include "RadioTestMode.h"   /* R&D radio link/range test gates */

/* ---- DBeacon flags byte (byte 15) ---- */
#define MESH_BEACON_FLAG_STILL      0x01U   /* bit0: 1 = still, 0 = moving */
#define MESH_BEACON_FLAG_GPS_VALID  0x02U   /* bit1: 1 = lat/lon present   */
#define MESH_BEACON_FLAG_RSSI_SRC   0x04U   /* bit2: 1 = best-RSSI src id present */

/* ---- BasicBeacon flags byte (byte 11) ---- */
#define MESH_BBEACON_FLAG_STILL      0x01U   /* bit0: 1 = still, 0 = moving */
#define MESH_BBEACON_FLAG_GPS_VALID  0x02U   /* bit1: 1 = lat/lon+age valid */

/* BasicBeacon on-wire sizes.
 *
 * Layout:
 *   [0]      type
 *   [1..4]   DeviceId
 *   [5..8]   BeaconMsgId
 *   [9..10]  BatMv
 *   [11]     flags (STILL, GPS_VALID)
 *   [12]     u8FwPatch
 *   [13..16] i32LatUDeg    (only if GPS_VALID)
 *   [17..20] i32LonUDeg    (only if GPS_VALID)
 *   [21..24] u32GpsAgeS    (only if GPS_VALID)
 */
#define MESH_BBEACON_LEN_BASE       13U
#define MESH_BBEACON_LEN_GPS        25U

/* Basic-mode primary RAM store — one entry per unique DeviceId, replaced
 * on receive only when the incoming BeaconMsgId is strictly newer. Cap
 * (MESH_MAX_BASIC_NEIGHBORS) lives in MeshNetwork.h since callers stack-
 * allocate a copy for MESHNETWORK_bGetBasicNeighbors. */
static MeshBasicNeighbor_t tBasicNeighborTable[MESH_MAX_BASIC_NEIGHBORS];
static uint16_t            u16BasicNeighborCount = 0U;

/* DBeacon on-wire sizes.
 *
 * Layout:
 *   [0]       type
 *   [1..4]    DreqId
 *   [5..6]    BatMv
 *   [7]       HopCount
 *   [8..9]    Rssi
 *   [10..13]  BeaconMsgId
 *   [14]      dreqWaveDisc
 *   [15]      flags (STILL, GPS_VALID)
 *   [16]      u8FwPatch      (sender's VERSION_SW_PATCH — sent on every beacon)
 *   [17..20]  i32LatUDeg     (only if GPS_VALID)
 *   [21..24]  i32LonUDeg     (only if GPS_VALID)
 *   [17..18]  u16BestRssiSrcId  (only if RSSI_SRC and NOT GPS_VALID)
 *   [25..26]  u16BestRssiSrcId  (only if RSSI_SRC and GPS_VALID)
 *
 * A decoder that sees fewer than MESH_BEACON_LEN_BASE bytes (an older peer
 * that hasn't been updated yet) still parses cleanly — see the length-guarded
 * reads in MESHNETWORK_vHandleDBeacon.
 *
 * The src id is APPENDED after the GPS block rather than inserted at [17],
 * which is why its offset depends on GPS_VALID. Inserting it at [17] and
 * shifting lat/lon to [19..26] would be read by an un-updated peer at the old
 * fixed offsets — those are hardcoded literals in both encoder and decoder —
 * and yield a plausible but wrong position. A field that only ever lands past
 * everything an older decoder reads can be ignored by it instead. */
#define MESH_BEACON_LEN_BASE        17U     /* through the FwPatch byte    */
#define MESH_BEACON_LEN_GPS         25U     /* with lat/lon appended       */
#define MESH_BEACON_LEN_BASE_SRC    19U     /* base + src id, no GPS       */
#define MESH_BEACON_LEN_GPS_SRC     27U     /* GPS + src id                */

/* DReq on-wire sizes.
 *
 * Layout:
 *   [0]      type
 *   [1..4]   DreqId          (origin primary's 16-bit id in the top half)
 *   [5]      SenderHopCount
 *   [6]      WaveCnt
 *   [7..8]   u16SenderId     (this hop's own device id — see below)
 *
 * The sender id is what makes a beacon's best-RSSI source reportable. DreqId
 * names the primary that STARTED the campaign, and every relay re-emits it
 * unchanged, so before this field a receiver could not tell which of its
 * neighbours had actually transmitted the frame it just heard — the strongest
 * DReq of a wave is very often a peer's relay, not the primary. Each hop now
 * stamps its own id here, so the field is per-hop by construction.
 *
 * Length-gated like every other field in this protocol: an older peer's 7-byte
 * DReq is still accepted (the floor is MESH_DREQ_LEN_BASE) and reports a sender
 * of 0 = unknown, and an older peer accepts a 9-byte DReq and ignores [7..8]. */
#define MESH_DREQ_LEN_BASE          7U
#define MESH_DREQ_LEN_SRC           9U

/* Maximum age of a GPS fix that may be stamped into an originated beacon.
 * The GPS pre-trigger fires DEVICE_DISCOVERY_GPS_PRETRIGGER_S (180 s) before
 * each scheduled wake, so a fix taken during it is ~120-180 s old by the time
 * the beacon goes out. A fix older than 5 minutes therefore came from an
 * earlier cycle (the last pre-trigger failed) and is stale — drop it and send
 * the beacon with no GPS rather than a position that no longer reflects where
 * the animal is. */
#define MESH_GPS_FIX_MAX_AGE_S      300U

/* ---- TX queue item ----
 * Item buffer 64 (was 128): the largest mesh frame is a full D-Ack at
 * 10 + 8*4 = 42 B (beacon 27, DReq 9, TimeSync 6); FrKernel responses bypass
 * this queue entirely. 24 x 64 B saves ~1.5 KB of heap vs 128. */
#define MESH_TX_QUEUE_LEN        24
#define MESH_TX_MAX_PACKET_SIZE  64

/* ---- MeshTx task thread flags ----
 * The periodic beacon/ack software-timer callbacks run in the FreeRTOS Timer
 * Service ("Tmr Svc") task, which has only a small stack. Building a packet
 * there (struct + encode + verbose DBG_LOG -> vsnprintf/localtime/strftime,
 * which carry hundreds of bytes of library stack frames) overflows it. So the
 * callbacks do nothing but set one of these flags; the MeshTx task — which has
 * a full-size stack — does the actual construction and enqueue. */
#define MESH_TX_FLAG_QUEUE       0x0001U   /* an item was put on xMeshTxQueue   */
#define MESH_TX_FLAG_BEACON      0x0002U   /* build + enqueue this node's beacon*/
#define MESH_TX_FLAG_ACK         0x0004U   /* build + enqueue a primary D-Ack   */
#define MESH_TX_FLAG_ANY         (MESH_TX_FLAG_QUEUE | MESH_TX_FLAG_BEACON | \
                                  MESH_TX_FLAG_ACK)

typedef struct {
    uint8_t  u8Buf[MESH_TX_MAX_PACKET_SIZE];
    uint16_t u16Len;
    uint32_t u32ReadyTick;   /* transmit not before this tick */
} MeshTxItem_t;

/* ---- CMSIS-RTOS v2 objects ---- */
static osMessageQueueId_t xMeshTxQueue         = NULL;
static osThreadId_t       xMeshTxTaskHandle    = NULL;
static osThreadId_t       xParserTaskHandle    = NULL;
static osTimerId_t        xBeaconTimer         = NULL;
static osTimerId_t        xPrimaryAckTimer     = NULL;
static osMutexId_t        xForwardRingMutex    = NULL;
static osMutexId_t        xNeighborTableMutex  = NULL;
/* Guards the beacon/role scalar state (bNodeBeaconing, eNodeRole,
 * u32NodeBeaconDreqId, u8NodeHopCount, R2 beacon counters). Timer/flag calls
 * are always done OUTSIDE this lock (they take kernel locks of their own). */
static osMutexId_t        xRoleMutex           = NULL;

/* ---- Protocol state ---- */
static ForwardRing_t    tForwardRing;
static NeighborEntry_t  tNeighborTable[MESH_MAX_NEIGHBORS];
static uint16_t         u16NeighborCount      = 0;
static uint32_t         tLastBeaconHeardTick  = 0;

/* RAM tripwires. The linked image leaves 8 bytes of the 64K RAM region, so a
 * field added to either of these structs that spills past the padding it was
 * meant to fit in costs 120x and fails the link with "region RAM overflowed" —
 * an error that points at the linker, not at the struct that caused it. These
 * name the cause instead. Update the expected size ONLY together with a
 * measured .bss figure showing the growth fits. */
_Static_assert(sizeof(NeighborEntry_t) == 24,
               "NeighborEntry_t grew: 120 entries x each added byte comes out of "
               "8 bytes of free RAM. Fit new fields in the tail padding.");
_Static_assert(sizeof(MeshDiscoveredNeighbor_t) == 24,
               "MeshDiscoveredNeighbor_t grew: DeviceDiscovery stack-allocates "
               "120 of these.");

static bool       bNodeBeaconing      = false;
static uint32_t   u32NodeBeaconDreqId = 0;
static uint8_t    u8NodeHopCount      = 0;
static NodeRole_e eNodeRole           = NODE_ROLE_UNKNOWN;

/* Survives the stop that bStopBeaconingLocked performs (which zeroes
 * u32NodeBeaconDreqId), so after we stop we still know which primary we were
 * talking to — needed to match a D-Ack that arrives after the campaign window
 * already closed this node's beaconing. */
static uint32_t   u32LastBeaconDreqId    = 0;

/* True once a D-Ack for this campaign has been matched. Suppresses the
 * per-wave re-arm: being acked means the primary has us, so further beacons
 * would be pure wasted air time. Cleared per campaign in vResetNodeRole. */
static bool       bNodeAckedThisCampaign = false;

/* Multi-primary: "first TimeSync per wake" gate (secondaries only).
 * Reset by DeviceDiscovery at the start of each wake cycle. */
static bool       bTimeSyncAcceptedThisWake = false;

/* I2: TimeSync dedup. Previously the raw UTC timestamp was inserted into the
 * shared forward ring alongside msg-ids of the form (deviceId16<<16)|ctr —
 * epoch seconds numerically collide with msg-ids from device ids in the
 * 0x69xx range (2026+), silently dropping that node's packets as "seen".
 * Track TimeSync separately: only a strictly newer UTC propagates, which also
 * drops late echoes of older TimeSyncs. */
static uint32_t   u32LastTimeSyncUtc  = 0;
static bool       bTimeSyncUtcValid   = false;

/* ---- Wakeup interval ---- */
static WakeupInterval tCurrentWakeupInterval = WAKEUP_INTERVAL_15_MIN;
static const uint8_t u8CurrentWakeupIntervalMin[] = {
    [WAKEUP_INTERVAL_15_MIN]  = 15,
    [WAKEUP_INTERVAL_30_MIN]  = 30,
    [WAKEUP_INTERVAL_60_MIN]  = 60,
    [WAKEUP_INTERVAL_120_MIN] = 120,
    [WAKEUP_INTERVAL_240_MIN] = 240
};

/* ---- Discovery mode + GPS-enable state ----
 * Safe cold-boot defaults: full mesh campaign and GPS active — matches
 * behaviour before the basic-mode feature. Overwritten by the first
 * incoming TimeSync (secondary) or the first AT+SETREQ response (primary). */
static DiscoveryMode_e eCurrentDiscoveryMode = DISCOVERY_MODE_ADVANCED;
static bool            bCurrentGpsEnabled    = true;

static const char * const MeshPktTypeStr[] = {
    [MeshPktType_Reserved]   = "Reserved",
    [MeshPktType_DReq]       = "DReq",
    [MeshPktType_DBeacon]    = "DBeacon",
    [MeshPktType_DAck]       = "DAck",
    [MeshPktType_TimeSync]   = "TimeSync",
    [MeshPktType_FrKernel]   = "FrKernel",
    [MeshPktType_OtaPrep]    = "OtaPrep",
    [MeshPktType_OtaPrepAck] = "OtaPrepAck",
    [MeshPktType_OtaChunk]   = "OtaChunk",
    [MeshPktType_OtaPoll]    = "OtaPoll",
    [MeshPktType_OtaReport]  = "OtaReport",
    [MeshPktType_BasicBeacon] = "BasicBeacon"
};

/* ---- Misc state ---- */
static uint16_t u16MsgCounter        = 0;
static uint64_t u64LastPrimaryHeardTick = 0;
static uint8_t  u8PrimaryDreqWaveCnt = 0;

/* Best DReq RSSI of the current wave, together with the device id of the node
 * that sent that DReq, in ONE 32-bit word: [31:16] = sender id, [15:0] = RSSI
 * as int16.
 *
 * Packed rather than kept as two statics because the pair is written from the
 * parser task (every DReq received) and read from the MeshTx task (when a
 * beacon is built), with no lock on either side — the surrounding role scalars
 * take xRoleMutex, but this sits on the DReq hot path and always has. As two
 * separate variables a beacon built between the two stores would report the
 * RSSI of one sender beside the id of another, which is worse than useless: it
 * names the wrong neighbour as the good path. A naturally-aligned 32-bit access
 * is single-copy atomic on Cortex-M, so packing removes the tear outright
 * instead of papering over it with a mutex.
 *
 * Always go through BEST_* below; nothing should touch the raw word. */
#define BEST_DREQ_RSSI_NONE   (-256)
static volatile uint32_t u32BestDreqPacked =
    ((uint32_t)0U << 16) | (uint32_t)(uint16_t)(int16_t)BEST_DREQ_RSSI_NONE;

static int16_t BEST_i16Rssi(void)
{
    return (int16_t)(uint16_t)(u32BestDreqPacked & 0xFFFFU);
}
/* Both halves from ONE read — the whole reason the pair is packed. Never fetch
 * the RSSI and the id in two separate accesses. */
static void BEST_vGet(int16_t *pi16Rssi, uint16_t *pu16SrcId)
{
    uint32_t u32P = u32BestDreqPacked;
    *pi16Rssi  = (int16_t)(uint16_t)(u32P & 0xFFFFU);
    *pu16SrcId = (uint16_t)(u32P >> 16);
}
static void BEST_vSet(int16_t i16Rssi, uint16_t u16SrcId)
{
    u32BestDreqPacked = ((uint32_t)u16SrcId << 16) |
                        (uint32_t)(uint16_t)i16Rssi;
}
static void BEST_vReset(void)
{
    BEST_vSet((int16_t)BEST_DREQ_RSSI_NONE, 0U);
}

/* Campaign-level traffic counters — a one-line DBG_LOG summary in place of
 * a DBG_LOG per packet. Reset at MESHNETWORK_vResetDreqWaveCnt() (already
 * called at every campaign start); read/logged via
 * MESHNETWORK_vLogCampaignStats() at campaign end. */
static uint16_t u16StatDReqHeard;
static uint16_t u16StatBeaconsHeard;
static uint16_t u16StatAcksHeard;
static uint16_t u16StatMsgsForwarded;

/* Mesh-layer packets that never made it onto the air: dropped because the
 * mesh TX queue was full, or discarded by MESHNETWORK_vFlushTxQueue(). Summed
 * with the radio layer's own tally into the campaign stats "txDrop" figure.
 * Back-pressure skips are deliberately NOT counted here — refusing to queue a
 * relay is the mechanism that PREVENTS these drops, so folding it in would
 * make the number climb precisely when things are working. */
static uint16_t u16StatTxDropped;

/* Tick of the most recent received discovery packet (any type).
 * Updated in every handler so DeviceDiscovery can detect mesh activity
 * without needing to track individual packet-type ticks. */
static uint32_t u32LastDiscoveryPktTick = 0;

/* Set the moment any DReq is heard this campaign — see
 * MESHNETWORK_bCampaignHeard() in the header for what it is for. Written from
 * the parser task, read from the AppTask; a bool write is atomic on this core
 * and a one-pass-late read only costs one more 250 ms poll. */
static volatile bool bCampaignHeard = false;

/* The primary ends a wave after MESH_DISCOVERY_IDLE_MS of beacon silence, and
 * every beacon is queued with up to MESH_TX_JITTER_MAX_MS of TX jitter. If the
 * idle ever drops to or below the jitter, a lone node in a sparse outer ring
 * could still be holding its jittered beacon when its wave is declared over —
 * losing exactly the deep tags this change exists to catch. */
#if (MESH_DISCOVERY_IDLE_MS <= MESH_TX_JITTER_MAX_MS)
#  error "MESH_DISCOVERY_IDLE_MS must exceed MESH_TX_JITTER_MAX_MS"
#endif

/* ---- Forward declarations ---- */
static void MESHNETWORK_vTxTask(void *pvParameters);
static bool MESHNETWORK_bSendPacket(const uint8_t *pBuf, size_t u32Len);
static void MESHNETWORK_vStartBeaconing(uint32_t u32DreqId, uint8_t u8HopCount);
static NodeRole_e MESHNETWORK_eGetRole(void);
static bool MESHNETWORK_bStopBeaconingLocked(uint32_t u32DreqId);
static void MESHNETWORK_vStopBeaconingByOrigin(uint32_t u32DreqId);

/* --------------------------------------------------------------------------
 * Endian helpers (on-wire big-endian)
 * -------------------------------------------------------------------------- */
static void write_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF; p[1] = (v >> 16) & 0xFF;
    p[2] = (v >>  8) & 0xFF; p[3] = v & 0xFF;
}
static uint32_t read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) | (uint32_t)p[3];
}
static void write_u16_be(uint8_t *p, uint16_t v)
    { p[0] = (v >> 8) & 0xFF; p[1] = v & 0xFF; }
static uint16_t read_u16_be(const uint8_t *p)
    { return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]); }
static void write_s16_be(uint8_t *p, int16_t v)
    { p[0] = (uint8_t)((v >> 8) & 0xFF); p[1] = (uint8_t)(v & 0xFF); }
static int16_t read_s16_be(const uint8_t *p)
    { return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]); }

/* --------------------------------------------------------------------------
 * Forward ring
 * -------------------------------------------------------------------------- */
static bool FORWARD_bHasSeen(uint32_t u32MsgId)
{
    bool bFound = false;
    if (osMutexAcquire(xForwardRingMutex, 100) == osOK)
    {
        for (uint8_t i = 0; i < tForwardRing.u8Count; i++)
        {
            uint8_t idx = (tForwardRing.u8Head + FORWARD_RING_SIZE -
                           tForwardRing.u8Count + i) % FORWARD_RING_SIZE;
            if (tForwardRing.u32Ring[idx] == u32MsgId) { bFound = true; break; }
        }
        osMutexRelease(xForwardRingMutex);
    }
    return bFound;
}
static void FORWARD_vAdd(uint32_t u32MsgId)
{
    if (osMutexAcquire(xForwardRingMutex, 100) == osOK)
    {
        tForwardRing.u32Ring[tForwardRing.u8Head] = u32MsgId;
        tForwardRing.u8Head = (tForwardRing.u8Head + 1) % FORWARD_RING_SIZE;
        if (tForwardRing.u8Count < FORWARD_RING_SIZE) tForwardRing.u8Count++;
        osMutexRelease(xForwardRingMutex);
    }
}

/* --------------------------------------------------------------------------
 * DReq dedup — separate from the forward ring on purpose, see
 * MESH_DREQ_DEDUPE_SIZE. Shares xForwardRingMutex (same access pattern, never
 * nested, and a second mutex would buy nothing).
 * -------------------------------------------------------------------------- */
static uint32_t au32DreqSeen[MESH_DREQ_DEDUPE_SIZE];
static uint8_t  u8DreqSeenHead;
static uint8_t  u8DreqSeenCount;

/* How many times each slot's id has been forwarded, 2 bits per slot, indexed by
 * the same ring index as au32DreqSeen. A packed word rather than the obvious
 * uint8_t[MESH_DREQ_DEDUPE_SIZE] or an array-of-struct: .bss on this part has
 * 8 bytes spare in the whole image, a {uint32_t; uint8_t;} entry would be 8 B
 * after padding (+32 B here), and a parallel byte array is +8 B. Two bits hold
 * 0..3 where the cap is 2, and 2 x 8 slots is exactly one uint16_t which lands
 * in padding that already existed between these statics. */
static uint16_t u16DreqFwdCnt;

_Static_assert(MESH_DREQ_DEDUPE_SIZE * 2U <= 16U,
               "u16DreqFwdCnt holds 2 bits per dedupe slot — widen it or shrink "
               "MESH_DREQ_DEDUPE_SIZE");
_Static_assert(MESH_DREQ_MAX_FORWARDS <= 3U,
               "a 2-bit per-slot counter cannot represent this many forwards");

/* Claim the right to forward u32DreqId once, and record it. Returns true if
 * this node may transmit a relay now, false once MESH_DREQ_MAX_FORWARDS copies
 * of that id have already gone out. *pu8Ordinal receives which forward this is
 * (1-based) so the caller can space the second copy away from the first.
 *
 * Check and record happen under ONE mutex acquisition. The old
 * DREQ_bHasSeen()/DREQ_vAdd() pair took the lock twice with the encode step in
 * between, so two receptions of the same id could both pass the test and both
 * forward — which is how a "forward once" store could already emit two. Now the
 * count is the only thing that decides, and it is incremented before the lock
 * is dropped.
 *
 * A mutex timeout returns false (do not forward). The old code's timeout
 * defaulted the other way, toward re-forwarding; under the contention that
 * causes a timeout in the first place, adding more relay traffic is the wrong
 * bias. */
static bool DREQ_bClaimForward(uint32_t u32DreqId, uint8_t *pu8Ordinal)
{
    bool bAllowed = false;

    if (osMutexAcquire(xForwardRingMutex, 100) != osOK) return false;

    for (uint8_t i = 0; i < u8DreqSeenCount; i++)
    {
        uint8_t idx = (uint8_t)((u8DreqSeenHead + MESH_DREQ_DEDUPE_SIZE -
                                 u8DreqSeenCount + i) % MESH_DREQ_DEDUPE_SIZE);
        if (au32DreqSeen[idx] != u32DreqId) continue;

        uint8_t u8Done = (uint8_t)((u16DreqFwdCnt >> (idx * 2U)) & 0x3U);
        if (u8Done < MESH_DREQ_MAX_FORWARDS)
        {
            u8Done++;
            u16DreqFwdCnt = (uint16_t)((u16DreqFwdCnt & ~(0x3U << (idx * 2U))) |
                                       ((uint32_t)u8Done << (idx * 2U)));
            *pu8Ordinal = u8Done;
            bAllowed    = true;
        }
        osMutexRelease(xForwardRingMutex);
        return bAllowed;
    }

    /* Not in the store — take a slot, evicting the oldest id if full. The
     * incoming slot's count must be SET to 1, not incremented: it may still
     * carry the evicted id's count. */
    uint8_t idx = u8DreqSeenHead;
    au32DreqSeen[idx] = u32DreqId;
    u16DreqFwdCnt = (uint16_t)((u16DreqFwdCnt & ~(0x3U << (idx * 2U))) |
                               ((uint32_t)1U << (idx * 2U)));
    u8DreqSeenHead = (uint8_t)((idx + 1U) % MESH_DREQ_DEDUPE_SIZE);
    if (u8DreqSeenCount < MESH_DREQ_DEDUPE_SIZE) u8DreqSeenCount++;

    osMutexRelease(xForwardRingMutex);
    *pu8Ordinal = 1U;
    return true;
}


/* --------------------------------------------------------------------------
 * Neighbor table
 * -------------------------------------------------------------------------- */
/* Takes the decoded beacon rather than a field-per-parameter list: at eleven
 * scalars the old signature was one positional mistake away from silently
 * swapping two same-typed fields, and BASIC_vAddOrUpdate below already works
 * this way. */
static void NEIGHBOR_vAddOrUpdate(const MeshPktDBeacon_t *ptBeacon)
{
    if (osMutexAcquire(xNeighborTableMutex, 100) == osOK)
    {
        for (uint16_t i = 0; i < u16NeighborCount; i++)
        {
            if (tNeighborTable[i].u32DeviceId == ptBeacon->u32DeviceId)
            {
                tNeighborTable[i].u8HopCount           = ptBeacon->u8HopCount;
                tNeighborTable[i].u16BatMv             = ptBeacon->u16BatMv;
                tNeighborTable[i].i16Rssi              = ptBeacon->i16Rssi;
                /* Src id moves with i16Rssi and only with it: the two are one
                 * reading ("this much signal, from that node"), so keeping a
                 * previous beacon's id beside a new RSSI would credit the wrong
                 * neighbour. A beacon that reports no id clears it rather than
                 * leaving a stale one attached to the fresh RSSI. */
                tNeighborTable[i].u16BestRssiSrcId     = ptBeacon->u16BestRssiSrcId;
                /* R7: keep the FIRST wave that discovered this node — it marks
                 * the earliest (typically closest) contact; do not overwrite
                 * with later waves. (dreqWaveDisc unused in the update path.) */
                tNeighborTable[i].u8MoveState          = ptBeacon->u8MoveState;
                tNeighborTable[i].u8FwPatch            = ptBeacon->u8FwPatch;
                /* Keep the last known fix if this beacon carries none. */
                if (ptBeacon->bGpsValid)
                {
                    tNeighborTable[i].bGpsValid  = true;
                    tNeighborTable[i].i32LatUDeg = ptBeacon->i32LatUDeg;
                    tNeighborTable[i].i32LonUDeg = ptBeacon->i32LonUDeg;
                }
                if (tNeighborTable[i].bAcked) tNeighborTable[i].bAcked = false;
                osMutexRelease(xNeighborTableMutex);
                tLastBeaconHeardTick = osKernelGetTickCount();
                return;
            }
        }
        if (u16NeighborCount < MESH_MAX_NEIGHBORS)
        {
            NeighborEntry_t *ptNew = &tNeighborTable[u16NeighborCount];
            ptNew->u32DeviceId          = ptBeacon->u32DeviceId;
            ptNew->u8HopCount           = ptBeacon->u8HopCount;
            ptNew->u16BatMv             = ptBeacon->u16BatMv;
            ptNew->i16Rssi              = ptBeacon->i16Rssi;
            ptNew->u16BestRssiSrcId     = ptBeacon->u16BestRssiSrcId;
            ptNew->u8DreqWaveDiscovered = ptBeacon->dreqWaveDisc;
            ptNew->u8MoveState          = ptBeacon->u8MoveState;
            ptNew->u8FwPatch            = ptBeacon->u8FwPatch;
            ptNew->bGpsValid            = ptBeacon->bGpsValid;
            ptNew->i32LatUDeg           = ptBeacon->i32LatUDeg;
            ptNew->i32LonUDeg           = ptBeacon->i32LonUDeg;
            ptNew->bAcked               = false;
            u16NeighborCount++;
            tLastBeaconHeardTick = osKernelGetTickCount();
        }
        osMutexRelease(xNeighborTableMutex);
    }
}

static void NEIGHBOR_vClearAll(void)
{
    if (osMutexAcquire(xNeighborTableMutex, 100) == osOK)
    {
        memset(tNeighborTable, 0, sizeof(tNeighborTable));
        u16NeighborCount = 0;
        osMutexRelease(xNeighborTableMutex);
    }
}

/* --------------------------------------------------------------------------
 * Packet encoders
 * -------------------------------------------------------------------------- */
/* u16SenderId is THIS hop's own device id, not the campaign originator's —
 * every caller passes LORARADIO_u32GetUniqueId(). See MESH_DREQ_LEN_SRC. */
static bool MESHNETWORK_bEncodeDReq(uint32_t u32DreqId,
                                     uint8_t u8SenderHopCount,
                                     uint8_t u8WaveCnt,
                                     uint16_t u16SenderId,
                                     uint8_t *pBuf,
                                     size_t u32BufLen,
                                     size_t *pu32Written)
{
    if (u32BufLen < MESH_DREQ_LEN_SRC) return false;
    pBuf[0] = (uint8_t)MeshPktType_DReq;
    write_u32_be(&pBuf[1], u32DreqId);
    pBuf[5] = u8SenderHopCount;
    pBuf[6] = u8WaveCnt;
    write_u16_be(&pBuf[7], u16SenderId);
    *pu32Written = MESH_DREQ_LEN_SRC;
    return true;
}

static bool MESHNETWORK_bEncodeDBeacon(const MeshPktDBeacon_t *ptBeacon,
                                        uint8_t *pBuf,
                                        size_t u32BufLen,
                                        size_t *pu32Written)
{
    size_t u32Needed = ptBeacon->bGpsValid ? MESH_BEACON_LEN_GPS
                                            : MESH_BEACON_LEN_BASE;
    if (ptBeacon->bBestRssiSrcValid) u32Needed += 2U;
    if (u32BufLen < u32Needed) return false;

    pBuf[0] = (uint8_t)MeshPktType_DBeacon;
    write_u32_be(&pBuf[1],  ptBeacon->u32DreqId);
    write_u16_be(&pBuf[5],  ptBeacon->u16BatMv);
    pBuf[7] = ptBeacon->u8HopCount;
    write_s16_be(&pBuf[8],  ptBeacon->i16Rssi);
    write_u32_be(&pBuf[10], ptBeacon->u32BeaconMsgId);
    pBuf[14] = ptBeacon->dreqWaveDisc;

    /* Flags byte + optional GPS payload (omitted when no fix, for airtime). */
    uint8_t u8Flags = 0U;
    if (ptBeacon->u8MoveState != 0U)  u8Flags |= MESH_BEACON_FLAG_STILL;
    if (ptBeacon->bGpsValid)          u8Flags |= MESH_BEACON_FLAG_GPS_VALID;
    if (ptBeacon->bBestRssiSrcValid)  u8Flags |= MESH_BEACON_FLAG_RSSI_SRC;
    pBuf[15] = u8Flags;
    pBuf[16] = ptBeacon->u8FwPatch;

    /* Src id goes AFTER the GPS block, so its offset depends on bGpsValid.
     * Written before the GPS payload below only because the offset arithmetic
     * reads better here; the two never overlap. */
    if (ptBeacon->bBestRssiSrcValid)
        write_u16_be(&pBuf[ptBeacon->bGpsValid ? 25U : 17U],
                     ptBeacon->u16BestRssiSrcId);

    if (ptBeacon->bGpsValid)
    {
        write_u32_be(&pBuf[17], (uint32_t)ptBeacon->i32LatUDeg);
        write_u32_be(&pBuf[21], (uint32_t)ptBeacon->i32LonUDeg);
    }

    *pu32Written = u32Needed;
    return true;
}

static bool MESHNETWORK_bEncodeBasicBeacon(const MeshPktBasicBeacon_t *ptBB,
                                            uint8_t *pBuf,
                                            size_t u32BufLen,
                                            size_t *pu32Written)
{
    size_t u32Needed = ptBB->bGpsValid ? MESH_BBEACON_LEN_GPS
                                        : MESH_BBEACON_LEN_BASE;
    if (u32BufLen < u32Needed) return false;

    pBuf[0] = (uint8_t)MeshPktType_BasicBeacon;
    write_u32_be(&pBuf[1], ptBB->u32DeviceId);
    write_u32_be(&pBuf[5], ptBB->u32BeaconMsgId);
    write_u16_be(&pBuf[9], ptBB->u16BatMv);

    uint8_t u8Flags = 0U;
    if (ptBB->u8MoveState != 0U) u8Flags |= MESH_BBEACON_FLAG_STILL;
    if (ptBB->bGpsValid)         u8Flags |= MESH_BBEACON_FLAG_GPS_VALID;
    pBuf[11] = u8Flags;
    pBuf[12] = ptBB->u8FwPatch;

    if (ptBB->bGpsValid)
    {
        write_u32_be(&pBuf[13], (uint32_t)ptBB->i32LatUDeg);
        write_u32_be(&pBuf[17], (uint32_t)ptBB->i32LonUDeg);
        write_u32_be(&pBuf[21], ptBB->u32GpsAgeS);
    }

    *pu32Written = u32Needed;
    return true;
}

static bool MESHNETWORK_bEncodeDAck(const MeshPktDAck_t *ptAck,
                                     uint8_t *pBuf,
                                     size_t u32BufLen,
                                     size_t *pu32Written)
{
    size_t u32Needed = 1 + 4 + 4 + 1 + (4 * ptAck->u8AckCount);
    if (u32BufLen < u32Needed) return false;
    pBuf[0] = (uint8_t)MeshPktType_DAck;
    write_u32_be(&pBuf[1], ptAck->u32AckMsgId);
    write_u32_be(&pBuf[5], ptAck->u32DreqId);
    pBuf[9] = ptAck->u8AckCount;
    for (uint8_t i = 0; i < ptAck->u8AckCount; i++)
        write_u32_be(&pBuf[10 + 4 * i], ptAck->u32AckedIds[i]);
    *pu32Written = u32Needed;
    return true;
}

/* TimeSync flags byte (offset 10). Old peers that emitted a 10-byte
 * TimeSync are decoded as advanced-mode + gps-enabled by the receiver
 * (see MESHNETWORK_vHandleTimeSync), which matches how they behaved
 * before the flag existed — so bumping the packet from 10 -> 11 bytes
 * is backward-compatible in both directions. */
#define MESH_TIMESYNC_FLAG_GPS_ENABLED   0x01U
#define MESH_TIMESYNC_FLAG_BASIC_MODE    0x02U
#define MESH_TIMESYNC_LEN                11U

static bool MESHNETWORK_bEncodeTimeSync(const MeshPktTimeSync_t *ptTS,
                                         uint8_t *pBuf,
                                         size_t u32BufLen,
                                         size_t *pu32Written)
{
    if (u32BufLen < MESH_TIMESYNC_LEN) return false;
    pBuf[0] = (uint8_t)MeshPktType_TimeSync;
    write_u32_be(&pBuf[1], ptTS->u32UtcTimestamp);
    pBuf[5] = (uint8_t)ptTS->tWakeupInterval;
    write_u32_be(&pBuf[6], ptTS->u32StagedFwVersion);
    uint8_t u8Flags = 0U;
    if (ptTS->bGpsEnabled)                       u8Flags |= MESH_TIMESYNC_FLAG_GPS_ENABLED;
    if (ptTS->eMode == DISCOVERY_MODE_BASIC)     u8Flags |= MESH_TIMESYNC_FLAG_BASIC_MODE;
    pBuf[10] = u8Flags;
    *pu32Written = MESH_TIMESYNC_LEN;
    return true;
}

/* --------------------------------------------------------------------------
 * TX helpers
 * -------------------------------------------------------------------------- */
static bool MESHNETWORK_bTxSendRaw(const uint8_t *pBuf, size_t u32Len)
{
    LoraRadio_Packet_t tTx;
    memset(&tTx, 0, sizeof(tTx));
    if (u32Len > sizeof(tTx.buffer)) return false;
    memcpy(tTx.buffer, pBuf, u32Len);
    tTx.length = (uint16_t)u32Len;
    DBG("MeshNetwork: Transmitting %s len=%u\r\n",
        MeshPktTypeStr[pBuf[0]], (unsigned)u32Len);
    return LORARADIO_bTxPacket(&tTx);
}

/* Fleet-decorrelated PRNG for TX jitter. libc rand() was never seeded, so
 * every node produced the IDENTICAL jitter sequence — the jitter meant to
 * de-correlate many nodes answering one DReq instead synchronised their
 * collisions. xorshift32 seeded per-node (radio RNG ^ unique ID) at init;
 * cheap enough for the parser/TX task paths and needs no radio access
 * (SUBGRF_GetRandom would touch radio state from a non-radio task). */
static uint32_t u32JitterRngState = 1U;

static void MESHNETWORK_vSeedJitterRng(uint32_t u32Seed)
{
    if (u32Seed == 0U) u32Seed = 0xA5A5A5A5U;   /* xorshift must not be 0 */
    u32JitterRngState = u32Seed;
}

static uint32_t MESHNETWORK_u32JitterRand(void)
{
    uint32_t x = u32JitterRngState;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    u32JitterRngState = x;
    return x;
}

static uint32_t MESHNETWORK_u32GetTxJitterMs(void)
{
    uint32_t u32Range = MESH_TX_JITTER_MAX_MS - MESH_TX_JITTER_MIN_MS;
    return MESH_TX_JITTER_MIN_MS + (MESHNETWORK_u32JitterRand() % (u32Range + 1));
}

/* Send delay for forward number u8Ordinal of a DReq (1-based, from
 * DREQ_bClaimForward). The first copy takes the ordinary jitter; the second
 * takes the non-overlapping later window so the two cannot land inside one
 * congestion window. See MESH_DREQ_FWD2_DELAY_MIN_MS. */
static uint32_t MESHNETWORK_u32DreqFwdDelayMs(uint8_t u8Ordinal)
{
    if (u8Ordinal < 2U) return MESHNETWORK_u32GetTxJitterMs();

    uint32_t u32Range = MESH_DREQ_FWD2_DELAY_MAX_MS - MESH_DREQ_FWD2_DELAY_MIN_MS;
    return MESH_DREQ_FWD2_DELAY_MIN_MS +
           (MESHNETWORK_u32JitterRand() % (u32Range + 1U));
}

/* --------------------------------------------------------------------------
 * Timer callbacks (CMSIS-RTOS v2 signature: void cb(void *arg))
 * -------------------------------------------------------------------------- */
/* Build this node's discovery beacon and hand it to the TX queue. Runs in the
 * MeshTx task context (NOT the timer callback) so the heavy locals and the
 * verbose DBG_LOG live on a full-size stack. */
static void MESHNETWORK_vBuildAndQueueBeacon(void)
{
    if (!MESHNETWORK_bIsBeaconing()) return;

    MeshPktDBeacon_t tBeacon;
    tBeacon.u32DreqId      = u32NodeBeaconDreqId;
    tBeacon.u32DeviceId    = 0;   /* derived from BeaconMsgId on receive */
    tBeacon.u16BatMv       = BAT_u16GetVoltage();
    tBeacon.u8HopCount     = u8NodeHopCount;
    tBeacon.u32BeaconMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tBeacon.dreqWaveDisc   = u8PrimaryDreqWaveCnt;
    tBeacon.u8FwPatch      = (uint8_t)VERSION_SW_PATCH;

    /* One read of the packed word, so the RSSI reported and the sender credited
     * for it are guaranteed to be the same DReq even if another arrives while
     * this beacon is being built. A src id of 0 means the DReq that set this
     * reading came from a peer too old to stamp its id, so send the RSSI alone
     * rather than crediting device 0x0000. */
    BEST_vGet(&tBeacon.i16Rssi, &tBeacon.u16BestRssiSrcId);
    tBeacon.bBestRssiSrcValid = (tBeacon.u16BestRssiSrcId != 0U);

    /* Stamp this node's own movement state and last-known GPS fix. */
    tBeacon.u8MoveState = 0U;     /* default: moving */
    tBeacon.bGpsValid   = false;
    tBeacon.i32LatUDeg  = 0;
    tBeacon.i32LonUDeg  = 0;
    uint32_t u32GpsAgeS = UINT32_MAX;   /* age of the fix we evaluated (for log) */
    tBeacon.u8MoveState = (MOVE_eGetState() == MOVE_STATE_STILL) ? 1U : 0U;
    {
        gnss_coord_deg_t tLat, tLon;
        /* Only stamp a fix from the most recent pre-trigger: it must exist AND
         * be within MESH_GPS_FIX_MAX_AGE_S. An older fix is from a previous
         * cycle and no longer valid, so the beacon goes out with no GPS. */
        if (GPS_bGetLastKnownFix(&tLat, &tLon, &u32GpsAgeS) &&
            u32GpsAgeS <= MESH_GPS_FIX_MAX_AGE_S)
        {
            tBeacon.bGpsValid  = true;
            tBeacon.i32LatUDeg = tLat.i32MicroDeg;
            tBeacon.i32LonUDeg = tLon.i32MicroDeg;
        }
    }

    DBG_LOG("MeshNetwork: Sending Beacon %08X move=%u gps=%u age=%lus\r\n",
        tBeacon.u32BeaconMsgId, tBeacon.u8MoveState, tBeacon.bGpsValid,
        (unsigned long)u32GpsAgeS);
    EVTLOG(LOG_TX_BEACON, 1);

    uint8_t u8Buf[64];
    size_t  u32Len = 0;
    if (!MESHNETWORK_bEncodeDBeacon(&tBeacon, u8Buf, sizeof(u8Buf), &u32Len)) return;
    FORWARD_vAdd(tBeacon.u32BeaconMsgId);
    MESHNETWORK_bSendPacket(u8Buf, u32Len);

    /* No beacon count or duration cap here any more: a secondary keeps beaconing
     * on xBeaconTimer until a D-Ack stops it (MESHNETWORK_vStopBeaconingByOrigin)
     * or the campaign's 180 s window closes and DeviceDiscovery calls
     * MESHNETWORK_vStopBeaconingSelf. See MeshNetwork.h above
     * MESH_TX_JITTER_MIN_MS for why the old 6-beacon cap went. */
}

/* Timer callback (Tmr Svc context): stay tiny — just wake the MeshTx task.
 * Reads bNodeBeaconing unlocked (atomic bool); a race at a transition only
 * costs one spurious/missed flag, and vBuildAndQueueBeacon re-checks the role
 * under xRoleMutex. Avoids taking a mutex from the timer daemon. */
static void MESHNETWORK_vBeaconTimerCallback(void *arg)
{
    (void)arg;
    if (bNodeBeaconing && xMeshTxTaskHandle != NULL)
        osThreadFlagsSet(xMeshTxTaskHandle, MESH_TX_FLAG_BEACON);
}

/* Build a primary D-Ack from the neighbor table and hand it to the TX queue.
 * Runs in the MeshTx task context (NOT the timer callback). */
static void MESHNETWORK_vBuildAndQueueAck(void)
{
    MeshPktDAck_t tAck;
    memset(&tAck, 0, sizeof(tAck));
    tAck.u32AckMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tAck.u32DreqId   = u32NodeBeaconDreqId;
    tAck.u32SenderId = LORARADIO_u32GetUniqueId();

    if (osMutexAcquire(xNeighborTableMutex, 100) == osOK)
    {
        /* Collect un-acked ids WITHOUT marking them yet — if the encode or
         * enqueue below fails (e.g. TX queue full mid-campaign), a premature
         * bAcked would mean those nodes are never acked on-air and beacon
         * until their cap. Mark only after the packet is safely queued. */
        uint8_t u8Added = 0;
        for (uint16_t i = 0;
             i < u16NeighborCount && u8Added < MESH_MAX_ACK_IDS_PER_PACKET;
             i++)
        {
            if (!tNeighborTable[i].bAcked)
                tAck.u32AckedIds[u8Added++] = tNeighborTable[i].u32DeviceId;
        }
        tAck.u8AckCount = u8Added;
        osMutexRelease(xNeighborTableMutex);

        if (tAck.u8AckCount > 0)
        {
            uint8_t u8Buf[128];
            size_t  u32Len = 0;
            bool    bQueued = false;
            if (MESHNETWORK_bEncodeDAck(&tAck, u8Buf, sizeof(u8Buf), &u32Len))
            {
                FORWARD_vAdd(tAck.u32AckMsgId);
                bQueued = MESHNETWORK_bSendPacket(u8Buf, u32Len);
                if (bQueued)
                    EVTLOG(LOG_TX_ACK, 1);
            }

            if (bQueued && osMutexAcquire(xNeighborTableMutex, 100) == osOK)
            {
                for (uint8_t a = 0; a < tAck.u8AckCount; a++)
                {
                    for (uint16_t i = 0; i < u16NeighborCount; i++)
                    {
                        if (tNeighborTable[i].u32DeviceId == tAck.u32AckedIds[a])
                        {
                            tNeighborTable[i].bAcked = true;
                            break;
                        }
                    }
                }
                osMutexRelease(xNeighborTableMutex);
            }
        }
    }
}

/* Timer callback (Tmr Svc context): stay tiny — just wake the MeshTx task. */
static void MESHNETWORK_vPrimaryAckTimerCallback(void *arg)
{
    (void)arg;
    if (xMeshTxTaskHandle != NULL)
        osThreadFlagsSet(xMeshTxTaskHandle, MESH_TX_FLAG_ACK);
}

/* --------------------------------------------------------------------------
 * MESHNETWORK_bTxBacklogHigh — is the TX queue too deep to take more relay
 * traffic?
 *
 * Back-pressure for FORWARDED packets only. A node that hears a lot (a
 * well-placed relay can hear 300+ beacons a campaign) will otherwise queue
 * relays faster than a congested channel drains them, fill the queue, and
 * then drop at the tail anyway — having already committed the airtime and
 * latency of everything ahead. Refusing early keeps the queue shallow, which
 * is what keeps the radio cycling back to RX promptly.
 *
 * Applied to beacon/DReq/ack relays. NOT applied to TimeSync (rare, and the
 * one packet the whole mesh depends on for time + firmware notification) or
 * to this node's own transmissions.
 * -------------------------------------------------------------------------- */
static bool MESHNETWORK_bTxBacklogHigh(void)
{
    if (xMeshTxQueue == NULL) return false;
    return (osMessageQueueGetCount(xMeshTxQueue) >= (MESH_TX_QUEUE_LEN / 2));
}

/* --------------------------------------------------------------------------
 * MESHNETWORK_bSendPacketDelayed — enqueue, holding TX for u32DelayMs
 *
 * u32DelayMs replaces the usual jitter draw rather than adding to it, so a
 * caller that has already chosen a delay window gets exactly that window.
 * MESHNETWORK_bSendPacket below is the ordinary-jitter entry point and is what
 * almost everything uses; only the second copy of a DReq needs its own delay.
 * -------------------------------------------------------------------------- */
static bool MESHNETWORK_bSendPacketDelayed(const uint8_t *pBuf, size_t u32Len,
                                            uint32_t u32DelayMs)
{
    if (pBuf == NULL || u32Len == 0 || u32Len > MESH_TX_MAX_PACKET_SIZE)
        return false;

    /* Radio-test mode owns the air for the duration. This is the single
     * chokepoint every mesh transmission passes through — own beacons, acks,
     * forwards, TimeSync and OTA responses — so one return here silences the
     * mesh without having to find and gate each caller.
     *
     * It deliberately does NOT silence the test's own beacons or FrKernel
     * replies: both build their packets and call LORARADIO_bTxPacket
     * directly, below this layer. */
    if (RADIOTESTMODE_bActive())
        return false;

    MeshTxItem_t tItem;
    memcpy(tItem.u8Buf, pBuf, u32Len);
    tItem.u16Len = (uint16_t)u32Len;

    tItem.u32ReadyTick     = osKernelGetTickCount() + u32DelayMs;

    if (osMessageQueuePut(xMeshTxQueue, &tItem, 0, 50) != osOK)
    {
        if (u16StatTxDropped < UINT16_MAX) u16StatTxDropped++;
        DBG_LOG("MeshNetwork: TX queue full, dropping packet\r\n");
        return false;
    }

    /* Wake the TX worker to drain the queue. */
    if (xMeshTxTaskHandle != NULL)
        osThreadFlagsSet(xMeshTxTaskHandle, MESH_TX_FLAG_QUEUE);

#ifdef MESH_LOG_VERBOSE
    DBG("MeshNetwork: Queued TX (len=%u, delay=%lu ms)\r\n",
        (unsigned)u32Len, (unsigned long)u32DelayMs);
#endif
    return true;
}

static bool MESHNETWORK_bSendPacket(const uint8_t *pBuf, size_t u32Len)
{
    return MESHNETWORK_bSendPacketDelayed(pBuf, u32Len,
                                          MESHNETWORK_u32GetTxJitterMs());
}

/* --------------------------------------------------------------------------
 * Incoming packet handlers
 * -------------------------------------------------------------------------- */
static void MESHNETWORK_vHandleDReq(const uint8_t *pBuf,
                                     size_t u32Len,
                                     int16_t s16Rssi)
{
    if (u32Len < MESH_DREQ_LEN_BASE) return;
    uint32_t u32DreqId        = read_u32_be(&pBuf[1]);
    uint32_t u32OriginId      = u32DreqId >> 16;
    uint8_t  u8SenderHopCount = pBuf[5];
    uint8_t  u8WaveCnt        = pBuf[6];

    /* Who actually transmitted the frame we just heard — a peer's relay as
     * often as the primary itself. 0 when an older peer sent it without the
     * field, which is reported as "unknown" rather than guessed at. */
    uint16_t u16ImmSenderId   = (u32Len >= MESH_DREQ_LEN_SRC)
                                    ? read_u16_be(&pBuf[7]) : 0U;

    DBG("MeshNetwork: DReq: dreq=%08X origin=%04X hop=%u from=%04X rssi=%d\r\n",
        u32DreqId, u32OriginId, u8SenderHopCount, u16ImmSenderId, s16Rssi);
    u16StatDReqHeard++;

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)u32OriginId, s16Rssi, u8WaveCnt);
    EVTLOG(LOG_RX_DREQ, u32LogValue);

    if (u32OriginId == LORARADIO_u32GetUniqueId()) return;

    /* Hop 0 is the sole marker for "heard the primary directly" (see the wave-1
     * block below), so a uint8_t that wrapped 255->0 would present a deep relay
     * as a direct hearer and pull the whole herd into wave 1. Nothing observed
     * comes near this — the longest chain is bounded by fleet size — but the
     * cost of the guarantee is one comparison, and relaying each id twice
     * doubles the number of relay laps that could get there. */
    if (u8SenderHopCount >= 0xFEU) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    /* Multi-primary: primaries never beacon and never forward.
     * They observe other primaries' DReqs (e.g. for RSSI tracking
     * below) but take no action on them. */
    if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY)
    {
        NodeRole_e eRole = MESHNETWORK_eGetRole();

        /* Heard a campaign, whoever relayed it and whatever wave. Keeps the
         * radio on past APP_SECONDARY_SILENCE_MS while the frontier works
         * outward — see MESHNETWORK_bCampaignHeard(). */
        bCampaignHeard = true;

        /* ---- Wave 1: flood, but only direct hearers beacon ----
         *
         * The frontier otherwise advances exactly one ring per wave: the
         * primary sends each DReq once, only FORWARDERs relay it, and a node
         * becomes a FORWARDER only once acked. So wave N reaches depth N, and
         * discoverable depth is capped at APP_PRIMARY_MAX_WAVES — a spread
         * herd loses its outermost tags off the end of that budget.
         *
         * Wave 1 is therefore relayed by EVERY node, deduped, purely as a
         * "a campaign is running, stay awake" signal. It deliberately does
         * NOT recruit beaconers: only nodes that heard the primary itself
         * (hop 0) beacon, so the per-wave roll-call keeps its existing shape
         * and the channel does not fill with the whole herd answering at
         * once. Waves 2+ fall through to the unchanged logic below.
         *
         * The relay carries hop+1 like any other, so a node that hears only
         * this flood still records a truthful hop count. That re-encode is also
         * what protects the "only direct hearers beacon" rule: a hop-0 frame
         * re-sent VERBATIM would tell every second-hop node it had heard the
         * primary itself, and the whole herd would answer on wave 1. Always
         * re-encode, never re-transmit pBuf.
         *
         * Each id may be relayed MESH_DREQ_MAX_FORWARDS times, once per
         * reception, so a collision that eats one copy need not cost the node
         * behind us its whole campaign. */
        if (u8WaveCnt <= 1U)
        {
            uint8_t u8FwdOrdinal = 0U;
            if (!MESHNETWORK_bTxBacklogHigh() &&
                DREQ_bClaimForward(u32DreqId, &u8FwdOrdinal))
            {
                uint8_t u8Out[32];
                size_t  u32OutLen = 0;
                if (MESHNETWORK_bEncodeDReq(u32DreqId,
                                            (uint8_t)(u8SenderHopCount + 1),
                                            u8WaveCnt,
                                            (uint16_t)LORARADIO_u32GetUniqueId(),
                                            u8Out, sizeof(u8Out), &u32OutLen))
                {
                    MESHNETWORK_bSendPacketDelayed(u8Out, u32OutLen,
                        MESHNETWORK_u32DreqFwdDelayMs(u8FwdOrdinal));
                    DBG("MeshNetwork: DReq wave-1 flood relayed (hop %u, copy %u)\r\n",
                        (unsigned)(u8SenderHopCount + 1), (unsigned)u8FwdOrdinal);
                    u16StatMsgsForwarded++;
                    EVTLOG(LOG_TX_DREQ, 2);
                }
            }

            /* Relayed copy: stay awake, but do not join the roll-call yet —
             * this node's own wave will come. */
            if (u8SenderHopCount != 0U)
            {
                if ((u8PrimaryDreqWaveCnt == u8WaveCnt) && (s16Rssi > BEST_i16Rssi()))
                    BEST_vSet(s16Rssi, u16ImmSenderId);
                return;
            }
            /* hop == 0: heard the primary directly — fall through and beacon. */
        }

        /* Re-arm a forwarder that was never acked, on a genuinely new wave from
         * the same primary. With the beacon cap gone this is no longer the
         * common path it was built for — a node that is not acked now simply
         * keeps beaconing, so it never becomes a FORWARDER in the first place.
         * What can still reach here is a node whose previous campaign window
         * closed (MESHNETWORK_vStopBeaconingSelf) and which then hears a fresh
         * wave before its role is reset. Kept for that case: the cost is one
         * comparison, and the failure mode without it is a node sitting mute
         * while the primary is still asking for it.
         *
         * Guards: same primary (via the id preserved at stop time), a genuinely
         * different dreq (so relayed copies of a wave we already answered don't
         * retrigger), and not already acked. Re-arming skips forwarding this
         * DReq, matching what a node beaconing from the start does. */
        if (eRole == NODE_ROLE_FORWARDER &&
            !bNodeAckedThisCampaign &&
            u32LastBeaconDreqId != 0U &&
            u32OriginId == (u32LastBeaconDreqId >> 16) &&
            u32DreqId   != u32LastBeaconDreqId)
        {
            DBG_LOG("MeshNetwork: New wave %u while unacked - re-arming beacon\r\n",
                    (unsigned)u8WaveCnt);
            u8PrimaryDreqWaveCnt = u8WaveCnt;
            BEST_vSet(s16Rssi, u16ImmSenderId);
            MESHNETWORK_vStartBeaconing(u32DreqId, (uint8_t)(u8SenderHopCount + 1));
        }
        else if (eRole == NODE_ROLE_FORWARDER)
        {
            uint8_t u8FwdOrdinal = 0U;
            if (!MESHNETWORK_bTxBacklogHigh() &&
                DREQ_bClaimForward(u32DreqId, &u8FwdOrdinal))
            {
                uint8_t u8Out[32];
                size_t  u32OutLen = 0;
                if (MESHNETWORK_bEncodeDReq(u32DreqId,
                                            (uint8_t)(u8SenderHopCount + 1),
                                            u8WaveCnt,
                                            (uint16_t)LORARADIO_u32GetUniqueId(),
                                            u8Out, sizeof(u8Out), &u32OutLen))
                {
                    MESHNETWORK_bSendPacketDelayed(u8Out, u32OutLen,
                        MESHNETWORK_u32DreqFwdDelayMs(u8FwdOrdinal));
                    DBG("MeshNetwork: DReq forwarded (copy %u)\r\n",
                        (unsigned)u8FwdOrdinal);
                    u16StatMsgsForwarded++;
                    EVTLOG(LOG_TX_DREQ, 2);
                }
            }
#ifdef MESH_LOG_VERBOSE
            else
            {
                DBG("MeshNetwork: DReq forward budget spent\r\n");
            }
#endif
        }
        else
        {
            /* Not a forwarder: start beaconing, or re-anchor onto a newer DReq
             * from the SAME primary. Re-anchoring is essential - a node still
             * beaconing an earlier dreq (a primary's pre-reboot campaign, or an
             * earlier wave) would otherwise keep emitting that stale dreq, and
             * the current campaign's ACKs (carrying the new dreq) would never
             * match u32NodeBeaconDreqId, so it could never be acked out. The
             * same-primary guard keeps multi-primary behaviour intact.
             * vStartBeaconing re-checks the transition under xRoleMutex (R4). */
            bool bSamePrimary = (u32OriginId == (u32NodeBeaconDreqId >> 16));
            if (eRole != NODE_ROLE_BEACONING ||
                (u32DreqId != u32NodeBeaconDreqId && bSamePrimary))
            {
                /* Seed the wave and best-RSSI baseline BEFORE starting to beacon.
                 * R1 fires the first beacon immediately from the higher-priority
                 * MeshTx task, which stamps the best RSSI into the beacon; if we
                 * left the RSSI to the max-update below (which runs after
                 * vStartBeaconing) that first beacon would carry the -256 reset
                 * value - exactly the -256 RSSI seen at the primary. */
                u8PrimaryDreqWaveCnt = u8WaveCnt;
                BEST_vSet(s16Rssi, u16ImmSenderId);
                MESHNETWORK_vStartBeaconing(u32DreqId, (uint8_t)(u8SenderHopCount + 1));
            }
        }
    }

    if ((u8PrimaryDreqWaveCnt == u8WaveCnt) && (s16Rssi > BEST_i16Rssi()))
        BEST_vSet(s16Rssi, u16ImmSenderId);
}

static void MESHNETWORK_vHandleDBeacon(const uint8_t *pBuf,
                                        size_t u32Len,
                                        int16_t s16Rssi)
{
    /* 15, not 14: pBuf[14] (dreqWaveDisc) is read unconditionally below, so a
     * 14-byte frame would be a one-byte overread and a garbage wave number.
     * Nothing on this mesh has ever sent a 14-byte beacon — the shortest that
     * ever existed is the 15-byte pre-FwPatch flags-only form — so this rejects
     * nothing real. */
    if (u32Len < 15) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    MeshPktDBeacon_t tBeacon;
    tBeacon.u32DreqId      = read_u32_be(&pBuf[1]);
    tBeacon.u16BatMv       = read_u16_be(&pBuf[5]);
    tBeacon.u8HopCount     = pBuf[7];
    tBeacon.i16Rssi        = read_s16_be(&pBuf[8]);
    tBeacon.u32BeaconMsgId = read_u32_be(&pBuf[10]);
    tBeacon.u32DeviceId    = tBeacon.u32BeaconMsgId >> 16;
    tBeacon.dreqWaveDisc   = pBuf[14];

    /* Flags + FwPatch + optional GPS payload. Length-guarded per field so a
     * pre-FwPatch peer (15-byte flags-only or 16-byte flags+GPS-header) still
     * parses cleanly with FwPatch defaulted to 0. */
    tBeacon.u8MoveState = 0U;     /* default: moving */
    tBeacon.u8FwPatch   = 0U;
    tBeacon.bGpsValid   = false;
    tBeacon.i32LatUDeg  = 0;
    tBeacon.i32LonUDeg  = 0;
    tBeacon.bBestRssiSrcValid = false;
    tBeacon.u16BestRssiSrcId  = 0U;
    if (u32Len >= 16U)
    {
        uint8_t u8Flags = pBuf[15];
        tBeacon.u8MoveState = (u8Flags & MESH_BEACON_FLAG_STILL) ? 1U : 0U;
        if (u32Len >= MESH_BEACON_LEN_BASE)
            tBeacon.u8FwPatch = pBuf[16];
        if ((u8Flags & MESH_BEACON_FLAG_GPS_VALID) &&
            (u32Len >= MESH_BEACON_LEN_GPS))
        {
            tBeacon.bGpsValid  = true;
            tBeacon.i32LatUDeg = (int32_t)read_u32_be(&pBuf[17]);
            tBeacon.i32LonUDeg = (int32_t)read_u32_be(&pBuf[21]);
        }

        /* Best-RSSI source id, appended after the GPS block. The offset comes
         * from the GPS FLAG rather than tBeacon.bGpsValid: bGpsValid is also
         * cleared above when a frame claims GPS but is too short to carry it,
         * and reading at 17 in that case would land on the truncated lat/lon
         * instead of a src id that isn't there. */
        size_t u32SrcOff = (u8Flags & MESH_BEACON_FLAG_GPS_VALID) ? 25U : 17U;
        if ((u8Flags & MESH_BEACON_FLAG_RSSI_SRC) &&
            (u32Len >= u32SrcOff + 2U))
        {
            tBeacon.u16BestRssiSrcId  = read_u16_be(&pBuf[u32SrcOff]);
            tBeacon.bBestRssiSrcValid = true;
        }
    }

    DBG("MeshNetwork: Beacon: dev=%04X dreq=%08X hop=%u wave=%X bat=%u rssi=%d rsrc=%04X move=%u gps=%u fwp=%u\r\n",
        tBeacon.u32DeviceId, tBeacon.u32DreqId, tBeacon.u8HopCount,
        tBeacon.dreqWaveDisc, tBeacon.u16BatMv, tBeacon.i16Rssi,
        tBeacon.u16BestRssiSrcId,
        tBeacon.u8MoveState, tBeacon.bGpsValid, tBeacon.u8FwPatch);
    u16StatBeaconsHeard++;

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)tBeacon.u32DeviceId,
                               tBeacon.i16Rssi, 0);
    EVTLOG(LOG_RX_BEACON, u32LogValue);

    /* 1. Deduplicate */
    if (FORWARD_bHasSeen(tBeacon.u32BeaconMsgId))
    {
#ifdef MESH_LOG_VERBOSE
        DBG("MeshNetwork: Beacon seen before\r\n");
#endif
        tLastBeaconHeardTick = osKernelGetTickCount();
        return;
    }

    /* 2. Mark as seen */
    FORWARD_vAdd(tBeacon.u32BeaconMsgId);

    /* 3. Primary: record neighbor */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY)
    {
        NEIGHBOR_vAddOrUpdate(&tBeacon);
        tLastBeaconHeardTick = osKernelGetTickCount();
        return;
    }

    /* 4. Secondary forwarder: relay beacon.
     * Skipped while the TX queue is already backed up — beacon relays are by
     * far the highest-volume traffic here, and queueing more of them is what
     * drives the backlog that keeps the radio in CAD instead of RX. */
    if (MESHNETWORK_eGetRole() == NODE_ROLE_FORWARDER &&
        !MESHNETWORK_bTxBacklogHigh())
    {
        tBeacon.u8HopCount++;
        uint8_t u8Buf[64];
        size_t  u32TempLen = 0;
        if (MESHNETWORK_bEncodeDBeacon(&tBeacon, u8Buf, sizeof(u8Buf), &u32TempLen))
        {
            MESHNETWORK_bSendPacket(u8Buf, u32TempLen);
            DBG("MeshNetwork: Forwarding Beacon\r\n");
            u16StatMsgsForwarded++;
            EVTLOG(LOG_TX_BEACON, 2);
        }
    }
}

/* Basic-mode primary RAM store update: newer-BeaconMsgId-wins. Called
 * on every MeshPktType_BasicBeacon received during a listen window. */
static void BASIC_vAddOrUpdate(const MeshPktBasicBeacon_t *ptBB)
{
    if (osMutexAcquire(xNeighborTableMutex, 100) != osOK) return;

    for (uint16_t i = 0U; i < u16BasicNeighborCount; i++)
    {
        if (tBasicNeighborTable[i].u32DeviceId == ptBB->u32DeviceId)
        {
            /* Wrap-safe compare: only accept a strictly-newer msgid. A
             * repeated same-msgid is a duplicate we heard on the air a
             * second time - drop silently. */
            if ((int32_t)(ptBB->u32BeaconMsgId - tBasicNeighborTable[i].u32BeaconMsgId) <= 0)
            {
                osMutexRelease(xNeighborTableMutex);
                return;
            }
            tBasicNeighborTable[i].u32BeaconMsgId = ptBB->u32BeaconMsgId;
            tBasicNeighborTable[i].u16BatMv       = ptBB->u16BatMv;
            tBasicNeighborTable[i].u8MoveState    = ptBB->u8MoveState;
            tBasicNeighborTable[i].u8FwPatch      = ptBB->u8FwPatch;
            /* Best-RSSI-wins: keep the strongest (least-negative) reading
             * heard across all beacons from this device — a single weak
             * reception due to obstruction or a distant antenna angle
             * shouldn't overwrite a known-good one. Multi-beacon RX in
             * one cycle only helps when we retain the best sample. */
            if (ptBB->i16Rssi > tBasicNeighborTable[i].i16Rssi)
                tBasicNeighborTable[i].i16Rssi = ptBB->i16Rssi;
            if (ptBB->bGpsValid)
            {
                tBasicNeighborTable[i].bGpsValid  = true;
                tBasicNeighborTable[i].i32LatUDeg = ptBB->i32LatUDeg;
                tBasicNeighborTable[i].i32LonUDeg = ptBB->i32LonUDeg;
                tBasicNeighborTable[i].u32GpsAgeS = ptBB->u32GpsAgeS;
            }
            osMutexRelease(xNeighborTableMutex);
            return;
        }
    }

    if (u16BasicNeighborCount < MESH_MAX_BASIC_NEIGHBORS)
    {
        MeshBasicNeighbor_t *e = &tBasicNeighborTable[u16BasicNeighborCount++];
        e->u32DeviceId    = ptBB->u32DeviceId;
        e->u32BeaconMsgId = ptBB->u32BeaconMsgId;
        e->u16BatMv       = ptBB->u16BatMv;
        e->i16Rssi        = ptBB->i16Rssi;
        e->u8MoveState    = ptBB->u8MoveState;
        e->u8FwPatch      = ptBB->u8FwPatch;
        e->bGpsValid      = ptBB->bGpsValid;
        e->i32LatUDeg     = ptBB->i32LatUDeg;
        e->i32LonUDeg     = ptBB->i32LonUDeg;
        e->u32GpsAgeS     = ptBB->u32GpsAgeS;
    }
    /* Table full — drop silently. Small cap deliberate; bump the
     * MESH_MAX_BASIC_NEIGHBORS #define if the deployment grows. */
    osMutexRelease(xNeighborTableMutex);
}

static void MESHNETWORK_vHandleBasicBeacon(const uint8_t *pBuf,
                                            size_t u32Len,
                                            int16_t s16Rssi)
{
    if (u32Len < MESH_BBEACON_LEN_BASE) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    MeshPktBasicBeacon_t tBB;
    tBB.u32DeviceId    = read_u32_be(&pBuf[1]);
    tBB.u32BeaconMsgId = read_u32_be(&pBuf[5]);
    tBB.u16BatMv       = read_u16_be(&pBuf[9]);
    uint8_t u8Flags    = pBuf[11];
    tBB.u8MoveState    = (u8Flags & MESH_BBEACON_FLAG_STILL) ? 1U : 0U;
    tBB.u8FwPatch      = pBuf[12];
    tBB.bGpsValid      = false;
    tBB.i32LatUDeg     = 0;
    tBB.i32LonUDeg     = 0;
    tBB.u32GpsAgeS     = 0U;
    tBB.i16Rssi        = s16Rssi;

    if ((u8Flags & MESH_BBEACON_FLAG_GPS_VALID) &&
        (u32Len >= MESH_BBEACON_LEN_GPS))
    {
        tBB.bGpsValid  = true;
        tBB.i32LatUDeg = (int32_t)read_u32_be(&pBuf[13]);
        tBB.i32LonUDeg = (int32_t)read_u32_be(&pBuf[17]);
        tBB.u32GpsAgeS = read_u32_be(&pBuf[21]);
    }

    DBG_LOG("MeshNetwork: BasicBeacon: dev=%08X msgid=%08X bat=%u rssi=%d move=%u gps=%u ageS=%lu fwp=%u\r\n",
        tBB.u32DeviceId, tBB.u32BeaconMsgId, tBB.u16BatMv, tBB.i16Rssi,
        tBB.u8MoveState, tBB.bGpsValid, (unsigned long)tBB.u32GpsAgeS, tBB.u8FwPatch);
    u16StatBeaconsHeard++;

    /* Only the primary keeps a RAM store — secondaries do nothing with a
     * peer basic beacon (no forwarding in basic mode by design). */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY)
    {
        BASIC_vAddOrUpdate(&tBB);
        tLastBeaconHeardTick = osKernelGetTickCount();
    }
}

void MESHNETWORK_vSendBasicBeacon(void)
{
    MeshPktBasicBeacon_t tBB;
    tBB.u32DeviceId    = LORARADIO_u32GetUniqueId();
    tBB.u32BeaconMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tBB.u16BatMv       = BAT_u16GetVoltage();
    tBB.u8MoveState    = (MOVE_eGetState() == MOVE_STATE_STILL) ? 1U : 0U;
    tBB.u8FwPatch      = (uint8_t)VERSION_SW_PATCH;
    tBB.bGpsValid      = false;
    tBB.i32LatUDeg     = 0;
    tBB.i32LonUDeg     = 0;
    tBB.u32GpsAgeS     = 0U;
    tBB.i16Rssi        = 0;

    /* Cached last-known fix has no max-age gate here (unlike DBeacon's
     * MESH_GPS_FIX_MAX_AGE_S): basic mode explicitly ships whatever age
     * we've got - the primary logs the age alongside so the operator can
     * judge freshness themselves. If the fix cache is empty (cold boot,
     * never fixed), skip GPS altogether. */
    gnss_coord_deg_t tLat, tLon;
    uint32_t u32AgeS = UINT32_MAX;
    if (GPS_bGetLastKnownFix(&tLat, &tLon, &u32AgeS))
    {
        tBB.bGpsValid  = true;
        tBB.i32LatUDeg = tLat.i32MicroDeg;
        tBB.i32LonUDeg = tLon.i32MicroDeg;
        tBB.u32GpsAgeS = u32AgeS;
    }

    uint8_t u8Buf[MESH_BBEACON_LEN_GPS];
    size_t  u32Len = 0;
    if (!MESHNETWORK_bEncodeBasicBeacon(&tBB, u8Buf, sizeof(u8Buf), &u32Len))
        return;

    DBG_LOG("MeshNetwork: Sending BasicBeacon msgid=%08X bat=%u move=%u fwp=%u gps=%u ageS=%lu\r\n",
        tBB.u32BeaconMsgId, tBB.u16BatMv, tBB.u8MoveState, tBB.u8FwPatch,
        tBB.bGpsValid, (unsigned long)tBB.u32GpsAgeS);
    EVTLOG(LOG_TX_BEACON, 3);
    MESHNETWORK_bSendPacket(u8Buf, u32Len);
}

bool MESHNETWORK_bGetBasicNeighbors(MeshBasicNeighbor_t *pBuffer,
                                     uint16_t u16MaxEntries,
                                     uint16_t *pu16ActualEntries)
{
    if (osMutexAcquire(xNeighborTableMutex, 200) != osOK)
    {
        *pu16ActualEntries = 0U;
        return false;
    }
    uint16_t u16Count = (u16BasicNeighborCount < u16MaxEntries)
                        ? u16BasicNeighborCount : u16MaxEntries;
    for (uint16_t i = 0U; i < u16Count; i++)
        pBuffer[i] = tBasicNeighborTable[i];
    *pu16ActualEntries = u16Count;
    osMutexRelease(xNeighborTableMutex);
    return true;
}

void MESHNETWORK_vClearBasicNeighbors(void)
{
    if (osMutexAcquire(xNeighborTableMutex, 200) != osOK) return;
    memset(tBasicNeighborTable, 0, sizeof(tBasicNeighborTable));
    u16BasicNeighborCount = 0U;
    osMutexRelease(xNeighborTableMutex);
}

uint16_t MESHNETWORK_u16GetBasicNeighborCount(void)
{
    /* Plain read of a uint16_t — atomic on Cortex-M4; no mutex needed just
     * to log a count. */
    return u16BasicNeighborCount;
}

static void MESHNETWORK_vHandleDAck(const uint8_t *pBuf,
                                     size_t u32Len,
                                     int16_t s16Rssi)
{
    if (u32Len < 10) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    uint32_t u32AckMsgId = read_u32_be(&pBuf[1]);
    uint32_t u32DreqId   = read_u32_be(&pBuf[5]);
    uint8_t  u8AckCount  = pBuf[9];

    /* The count byte is attacker/corruption-controlled. A conforming sender
     * never emits more than MESH_MAX_ACK_IDS_PER_PACKET ids, but the RX buffer
     * is 256 B, so a garbled frame whose count byte lands in 9..61 would still
     * satisfy the length check below and overrun u32Ids[] on the stack (the
     * weak XOR-8 packet CRC lets ~1/256 of corrupted frames through). Reject
     * any over-range count before it can smash the stack. */
    if (u8AckCount > MESH_MAX_ACK_IDS_PER_PACKET)
    {
        DBG_LOG("MeshNetwork: DAck dropped, bad count=%u\r\n", u8AckCount);
        return;
    }
    if (u32Len < (size_t)(10 + 4 * u8AckCount)) return;

    uint32_t u32Ids[MESH_MAX_ACK_IDS_PER_PACKET];
    for (uint8_t i = 0; i < u8AckCount; i++)
        u32Ids[i] = read_u32_be(&pBuf[10 + 4 * i]);

    DBG("MeshNetwork: DAck: ackId=%08X dreq=%08X count=%u\r\n",
        u32AckMsgId, u32DreqId, u8AckCount);
    u16StatAcksHeard++;

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)(u32DreqId >> 16),
                               s16Rssi, u8AckCount);
    EVTLOG(LOG_RX_ACK, u32LogValue);

    if (FORWARD_bHasSeen(u32AckMsgId))
    {
#ifdef MESH_LOG_VERBOSE
        DBG("MeshNetwork: Ack seen before\r\n");
#endif
        return;
    }
    FORWARD_vAdd(u32AckMsgId);

    if (MESHNETWORK_eGetRole() == NODE_ROLE_FORWARDER &&
        !MESHNETWORK_bTxBacklogHigh())
    {
        MESHNETWORK_bSendPacket(pBuf, u32Len);
        DBG("MeshNetwork: Ack forwarded\r\n");
        u16StatMsgsForwarded++;
        EVTLOG(LOG_TX_ACK, 2);
    }

    /* Multi-primary: primaries never beacon, so they never need to
     * stop-on-ACK. Guard explicitly so any future primary code path
     * cannot accidentally trip the secondary state machine. */
    if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY)
    {
        uint32_t u32MyId = LORARADIO_u32GetUniqueId();
        for (uint8_t i = 0; i < u8AckCount; i++)
        {
            if (u32Ids[i] == u32MyId)
            {
                /* I1: match on the ack's origin primary, not the exact dreq —
                 * the primary issues a new dreq per wave, and an ack carrying
                 * a newer wave's dreq must still stop a wave-1 beaconer. */
                MESHNETWORK_vStopBeaconingByOrigin(u32DreqId);
                break;
            }
        }
    }
}

static void MESHNETWORK_vHandleTimeSync(const uint8_t *pBuf,
                                         size_t u32Len,
                                         int16_t s16Rssi)
{
    if (u32Len < 10) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    uint32_t       u32Utc       = read_u32_be(&pBuf[1]);
    WakeupInterval tInterval    = (WakeupInterval)pBuf[5];
    uint32_t       u32StagedVer = read_u32_be(&pBuf[6]);

    /* Optional flags byte at [10] — absent from pre-basicDiscovery peers
     * (10-byte TimeSync). Missing flags decode as advanced + gps enabled,
     * matching the behaviour of nodes that never had this feature. */
    DiscoveryMode_e eMode       = DISCOVERY_MODE_ADVANCED;
    bool            bGpsEnabled = true;
    if (u32Len >= MESH_TIMESYNC_LEN)
    {
        uint8_t u8Flags = pBuf[10];
        eMode       = (u8Flags & MESH_TIMESYNC_FLAG_BASIC_MODE)  ? DISCOVERY_MODE_BASIC : DISCOVERY_MODE_ADVANCED;
        bGpsEnabled = (u8Flags & MESH_TIMESYNC_FLAG_GPS_ENABLED) ? true                 : false;
    }

    DBG("MeshNetwork: TimeSync: utc=%u interval=%u fwVer=%lu mode=%s gps=%u\r\n",
            u32Utc, tInterval, (unsigned long)u32StagedVer,
            (eMode == DISCOVERY_MODE_BASIC) ? "basic" : "advanced",
            (unsigned)bGpsEnabled);

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, 0, s16Rssi, 0);
    EVTLOG(LOG_RX_TS, u32LogValue);

    /* I2: dedicated dedup — only a strictly newer UTC propagates (also kills
     * late echoes of older TimeSyncs). Not the shared msg-id ring: raw UTC
     * values alias msg-ids of nodes with device ids in the current-epoch
     * numeric range, which would silently drop their packets. */
    if (bTimeSyncUtcValid && (int32_t)(u32Utc - u32LastTimeSyncUtc) <= 0)
    {
#ifdef MESH_LOG_VERBOSE
        DBG("MeshNetwork: TimeSync seen before\r\n");
#endif
        return;
    }
    u32LastTimeSyncUtc = u32Utc;
    bTimeSyncUtcValid  = true;

    /* Multi-primary: primaries have authoritative time from the logger
     * and must not accept TimeSync from a peer primary. Drop silently
     * (no RTC update, no interval change, no forward, no AppTask
     * notify — which would prematurely end this primary's campaign). */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY)
    {
        DBG("MeshNetwork: TimeSync ignored (primary)\r\n");
        return;
    }

    /* Secondary: accept only the first TimeSync this wake cycle.
     * Subsequent TimeSyncs are still forwarded (the mesh keeps
     * propagating during the few seconds the node stays awake) but
     * do not re-apply RTC/interval and do not re-notify AppTask. */
    if (!bTimeSyncAcceptedThisWake)
    {
        RTC_vSetUTC(u32Utc);
        MESHNETWORK_vSetWakeupInterval(tInterval);
        MESHNETWORK_vSetDiscoveryMode(eMode);
        MESHNETWORK_vSetGpsEnabled(bGpsEnabled);
        MESHNETWORK_vUpdatePrimaryLastSeen();
        bTimeSyncAcceptedThisWake = true;
        DBG_LOG("MeshNetwork: TimeSync applied: %u interval=%u mode=%s gps=%u\r\n",
                u32Utc, tInterval,
                (eMode == DISCOVERY_MODE_BASIC) ? "basic" : "advanced",
                (unsigned)bGpsEnabled);

#ifdef STORAGE_BACKEND_FLASH
        /* Auto-arm firmware acceptance straight off the version the
         * primary just announced — no "tag <ID> fwaccept" needed. Each
         * campaign re-evaluates this, so a secondary that missed the
         * actual distribution wake (asleep, out of range, etc.) simply
         * re-arms on the next TimeSync it hears until it catches up.
         * Flash-backend only: Fota's OTA storage lives on ext-NOR, which
         * a MicroSD-backend build doesn't have. */
        if (u32StagedVer > VERSION_u32Get() && !FOTA_bAcceptanceArmed())
        {
            DBG_LOG("MeshNetwork: TimeSync offers newer fw v%lu (running v%lu) - auto-arming acceptance\r\n",
                    (unsigned long)u32StagedVer, (unsigned long)VERSION_u32Get());
            FOTA_vArmAcceptance();
        }
#endif

        osThreadId_t xAppTask = DEVICE_DISCOVERY_xGetTaskHandle();
        if (xAppTask != NULL)
            osThreadFlagsSet(xAppTask, DEVICE_DISCOVERY_NOTIFY_TIMESYNC);
    }
#ifdef MESH_LOG_VERBOSE
    else
    {
        DBG("MeshNetwork: TimeSync seen (already accepted this wake)\r\n");
    }
#endif

    MESHNETWORK_bSendPacket(pBuf, u32Len);
    DBG("MeshNetwork: TimeSync forwarded\r\n");
    u16StatMsgsForwarded++;
    EVTLOG(LOG_TX_TS, 2);
}

/* --------------------------------------------------------------------------
 * MESHNETWORK_vParserTask — receive loop
 * -------------------------------------------------------------------------- */
void MESHNETWORK_vParserTask(void *pvParameters)
{
    (void)pvParameters;
    LoraRadio_Packet_t tRx;

    for (;;)
    {
        if (!LORARADIO_bRxPacket(&tRx))
        {
            osDelay(5);
            continue;
        }

        if (tRx.length < 1) continue;

        MeshPktType_e eType = (MeshPktType_e)tRx.buffer[0];

#ifdef STORAGE_BACKEND_FLASH
        /* FOTA takes priority: while a distribute or receive session is
         * active, ordinary mesh traffic (beacons, DReq/DAck, TimeSync,
         * FrKernel) adds nothing but radio/CPU contention right when a
         * chunk read/write is most vulnerable to it (a nearby TX/RX's
         * supply-rail sag corrupting a flash access — see the findings in
         * FOTA_bSendChunk). Drop it outright rather than processing or
         * forwarding it; the OTA packet types below are never affected by
         * this check. */
        if (eType != MeshPktType_OtaPrep && eType != MeshPktType_OtaPrepAck &&
            eType != MeshPktType_OtaChunk && eType != MeshPktType_OtaPoll &&
            eType != MeshPktType_OtaReport && FOTA_bSessionActive())
        {
            continue;
        }
#endif

        /* R&D radio link/range test. Same shape as the FOTA filter above:
         * placed after the RX read so the LoRa queue still drains, and before
         * the dispatch so a dropped packet has no side effects at all — no
         * forward-ring insert, no neighbour update, no TimeSync notify.
         *
         * A beaconing secondary is deliberately deaf to everything: that is
         * what makes the test a clean one-way measurement, and it is why the
         * only way out is a shake. A listening primary keeps exactly two
         * doors open — the beacons it is here to measure, and FrKernel, so
         * "tag <ID> radio stop" can still reach it. */
        if (RADIOTESTMODE_bActive())
        {
            if (!RADIOTESTMODE_bIsListener())
                continue;

            if (eType == MeshPktType_Reserved)
            {
                RADIOTESTMODE_vOnBeacon(tRx.buffer + 1,
                                        (uint8_t)(tRx.length - 1),
                                        tRx.rssi, tRx.snr, tRx.i16NoiseFloor);
                continue;
            }

            if (eType != MeshPktType_FrKernel)
                continue;
        }

        switch (eType)
        {
            case MeshPktType_DReq:
                MESHNETWORK_vHandleDReq(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_DBeacon:
                MESHNETWORK_vHandleDBeacon(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_BasicBeacon:
                MESHNETWORK_vHandleBasicBeacon(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_DAck:
                MESHNETWORK_vHandleDAck(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_TimeSync:
                MESHNETWORK_vHandleTimeSync(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_FrKernel:
                MESHNETWORK_vOnFrKernelPacket(tRx.buffer + 1, (uint8_t)(tRx.length - 1));
                break;
            case MeshPktType_Reserved:
                /* A radio-test beacon. Only reached when this node is NOT in
                 * the test itself — the gate above intercepts these when it
                 * is — so this is a bystander hearing someone else's test.
                 *
                 * The bridge is a passive listener on the whole mesh and is
                 * the obvious rig to watch a range test from, so it decodes
                 * and prints. Every other build drops it silently: type 0 is
                 * now a type we legitimately transmit, so falling through to
                 * default's "Unknown pkt type" would have every production
                 * unit within earshot of a test log a line every 5 s about a
                 * packet that is not addressed to it and is not a fault. */
#ifdef FRKERNEL_INTERFACE_LORA_BRIDGE
                (void)RADIOTESTMODE_bLogBeacon(tRx.buffer + 1,
                                               (uint8_t)(tRx.length - 1),
                                               tRx.rssi, tRx.snr,
                                               tRx.i16NoiseFloor);
#endif
                break;
#ifdef STORAGE_BACKEND_FLASH
            case MeshPktType_OtaPrep:
            case MeshPktType_OtaPrepAck:
            case MeshPktType_OtaChunk:
            case MeshPktType_OtaPoll:
            case MeshPktType_OtaReport:
                /* Direct (non-mesh) OTA traffic: dispatched to the Fota
                 * worker, never entered into the forward ring, never
                 * re-forwarded. */
                FOTA_vOnLoraPacket(tRx.buffer, tRx.length);
                break;
#endif
            default:
                DBG_LOG("MeshNetwork: Unknown pkt type %u\r\n", tRx.buffer[0]);
                break;
        }
    }
}

/* --------------------------------------------------------------------------
 * MESHNETWORK_vTxTask — dequeue and transmit with jitter
 * -------------------------------------------------------------------------- */
static void MESHNETWORK_vTxTask(void *pvParameters)
{
    (void)pvParameters;
    MeshTxItem_t tItem;

    for (;;)
    {
        /* R&D radio range test, beaconing secondary only: send this node's
         * test beacon if one is due, and take its answer as how long we may
         * block. Returns osWaitForever - one volatile read - in every other
         * case, which is every case in normal operation.
         *
         * The test's beacon rides this task rather than owning one because on
         * a secondary the FreeRTOS heap has no 2 KB block left to give it, and
         * this is the right task to borrow: MESHNETWORK_bSendPacket returns
         * false for the whole duration of a test, so nothing else can reach
         * here while one runs, and the beacon it builds is no deeper on this
         * stack than MESHNETWORK_vBuildAndQueueBeacon below already is. */
        uint32_t u32Wait = RADIOTESTMODE_u32ServiceBeacon();

        /* Block until something needs doing: a periodic timer asked us to build
         * a beacon/ack, or a packet was queued for transmission. Blocking on
         * flags (not a busy poll) keeps the device asleep between events. */
        uint32_t u32Flags = osThreadFlagsWait(MESH_TX_FLAG_ANY,
                                              osFlagsWaitAny, u32Wait);
        if (u32Flags & osFlagsError) continue;   /* also the beacon timeout */

        /* Heavy packet construction runs here, on this task's full-size stack —
         * never on the tiny Tmr Svc stack the timer callbacks run in. */
        if (u32Flags & MESH_TX_FLAG_BEACON) MESHNETWORK_vBuildAndQueueBeacon();
        if (u32Flags & MESH_TX_FLAG_ACK)    MESHNETWORK_vBuildAndQueueAck();

        /* Drain everything currently queued (including anything the builders
         * above just enqueued), honouring each item's jitter delay. */
        while (osMessageQueueGet(xMeshTxQueue, &tItem, NULL, 0) == osOK)
        {
            uint32_t now = osKernelGetTickCount();
            if (tItem.u32ReadyTick > now)
                osDelay(tItem.u32ReadyTick - now);   /* wait out the TX jitter */

#ifdef MESH_LOG_VERBOSE
            DBG("MeshNetwork: TX (len=%u)\r\n", tItem.u16Len);
#endif
            MESHNETWORK_bTxSendRaw(tItem.u8Buf, tItem.u16Len);
        }
    }
}

/* --------------------------------------------------------------------------
 * MESHNETWORK_vInit
 * -------------------------------------------------------------------------- */
void MESHNETWORK_vInit(void)
{
    xForwardRingMutex   = osMutexNew(NULL);  configASSERT(xForwardRingMutex   != NULL);
    xNeighborTableMutex = osMutexNew(NULL);  configASSERT(xNeighborTableMutex != NULL);
    xRoleMutex          = osMutexNew(NULL);  configASSERT(xRoleMutex          != NULL);

    /* E11: seed the per-message counter from the RNG instead of 0, so a node
     * that reboots mid-deployment doesn't re-emit msg IDs that peers still hold
     * in their dedup rings (which would silently drop its packets). */
    u16MsgCounter = (uint16_t)LORARADIO_u32GetRandomNumber(0xFFFF);

    /* Seed the TX-jitter PRNG per node (see MESHNETWORK_u32JitterRand). */
    MESHNETWORK_vSeedJitterRng(LORARADIO_u32GetRandomNumber(0xFFFFFFFEU) ^
                               (LORARADIO_u32GetUniqueId() * 2654435761U));

    memset(&tForwardRing,   0, sizeof(tForwardRing));
    memset(tNeighborTable,  0, sizeof(tNeighborTable));

    u16NeighborCount     = 0;
    tLastBeaconHeardTick = 0;
    bNodeBeaconing       = false;
    eNodeRole            = NODE_ROLE_UNKNOWN;

    xMeshTxQueue = osMessageQueueNew(MESH_TX_QUEUE_LEN, sizeof(MeshTxItem_t), NULL);
    configASSERT(xMeshTxQueue != NULL);

    /* Timers: period is passed to osTimerStart, not at creation */
    xBeaconTimer     = osTimerNew(MESHNETWORK_vBeaconTimerCallback,     osTimerPeriodic, NULL, NULL);
    xPrimaryAckTimer = osTimerNew(MESHNETWORK_vPrimaryAckTimerCallback, osTimerPeriodic, NULL, NULL);
    configASSERT(xBeaconTimer     != NULL);
    configASSERT(xPrimaryAckTimer != NULL);

    static const osThreadAttr_t parser_attr = {
        .name       = "MeshParser",
        .stack_size = configMINIMAL_STACK_SIZE * 5 * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    static const osThreadAttr_t tx_attr = {
        .name       = "MeshTx",
        .stack_size = configMINIMAL_STACK_SIZE * 4 * sizeof(StackType_t),
        .priority   = osPriorityAboveNormal,
    };

    xParserTaskHandle = osThreadNew(MESHNETWORK_vParserTask, NULL, &parser_attr);
    xMeshTxTaskHandle = osThreadNew(MESHNETWORK_vTxTask,     NULL, &tx_attr);
    configASSERT(xParserTaskHandle != NULL);
    configASSERT(xMeshTxTaskHandle != NULL);

    DBG("MeshNetwork initialized\r\n");
}

/* --------------------------------------------------------------------------
 * Public API implementations
 * -------------------------------------------------------------------------- */
uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void)
{
    u16MsgCounter++;
    uint32_t u32Hi = (LORARADIO_u32GetUniqueId() & 0xFFFF);
    return (u32Hi << 16) | (uint32_t)u16MsgCounter;
}

bool MESHNETWORK_bStartDiscoveryRound(uint32_t u32DreqId)
{
    tLastBeaconHeardTick = osKernelGetTickCount();
    u32NodeBeaconDreqId  = u32DreqId;

    uint8_t u8Out[32];
    size_t  u32Len = 0;
    /* hop 0 and our own id: for a DReq straight off the primary the immediate
     * sender IS the origin, so the two agree by construction. */
    if (!MESHNETWORK_bEncodeDReq(u32DreqId, 0, u8PrimaryDreqWaveCnt,
                                  (uint16_t)LORARADIO_u32GetUniqueId(),
                                  u8Out, sizeof(u8Out), &u32Len))
        return false;

    if (!MESHNETWORK_bSendPacket(u8Out, u32Len))
        return false;

    MESHNETWORK_vStartPrimaryAck();
    DBG_LOG("MeshNetwork: DReq %08X sent\r\n", u32DreqId);
    EVTLOG(LOG_TX_DREQ, 1);
    return true;
}

void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval,
                               uint32_t u32StagedFwVersion,
                               DiscoveryMode_e eMode,
                               bool bGpsEnabled)
{
    MeshPktTimeSync_t tTs = {
        .u32UtcTimestamp     = u32UtcTimestamp,
        .tWakeupInterval     = tWakeupInterval,
        .u32StagedFwVersion  = u32StagedFwVersion,
        .eMode               = eMode,
        .bGpsEnabled         = bGpsEnabled,
    };
    uint8_t u8Buf[16];
    size_t  u32Len = 0;
    if (!MESHNETWORK_bEncodeTimeSync(&tTs, u8Buf, sizeof(u8Buf), &u32Len)) return;
    /* I2: record in the dedicated TimeSync tracker (not the msg-id ring) so
     * echoes of our own TimeSync are not re-forwarded. */
    u32LastTimeSyncUtc = u32UtcTimestamp;
    bTimeSyncUtcValid  = true;
    MESHNETWORK_bSendPacket(u8Buf, u32Len);
    DBG_LOG("MeshNetwork: TimeSync sent %u interval=%u fwVer=%lu mode=%s gps=%u\r\n",
        u32UtcTimestamp, tWakeupInterval, (unsigned long)u32StagedFwVersion,
        (eMode == DISCOVERY_MODE_BASIC) ? "basic" : "advanced",
        (unsigned)bGpsEnabled);
}

/* Snapshot the node role under the mutex (parser task uses this instead of
 * reading eNodeRole directly). */
static NodeRole_e MESHNETWORK_eGetRole(void)
{
    NodeRole_e e = NODE_ROLE_UNKNOWN;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        e = eNodeRole;
        osMutexRelease(xRoleMutex);
    }
    return e;
}

bool MESHNETWORK_bIsBeaconing(void)
{
    bool b = false;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        b = bNodeBeaconing;
        osMutexRelease(xRoleMutex);
    }
    return b;
}

/* Core stop transition — assumes xRoleMutex is HELD. Returns true if it
 * actually stopped (caller must then osTimerStop OUTSIDE the lock). */
static bool MESHNETWORK_bStopBeaconingLocked(uint32_t u32DreqId)
{
    if (!bNodeBeaconing)                  return false;
    if (u32NodeBeaconDreqId != u32DreqId) return false;
    bNodeBeaconing      = false;
    BEST_vReset();
    /* Remember who we were beaconing to before clearing the live id: a D-Ack
     * delayed past the window's end, or that primary's next wave, both still
     * need to be attributable to this primary. */
    u32LastBeaconDreqId = u32NodeBeaconDreqId;
    u32NodeBeaconDreqId = 0;
    eNodeRole           = NODE_ROLE_FORWARDER;
    return true;
}

/* I1: stop when the D-Ack came from the SAME primary, even if its dreq id is
 * a newer wave than the one this node anchored to. The primary generates a
 * new dreq per wave, so an exact-dreq match leaves a wave-1 beaconer acked
 * during wave 2 beaconing until its cap. Origin (dreq >> 16) is the primary's
 * 16-bit id, so this stays multi-primary safe. */
static void MESHNETWORK_vStopBeaconingByOrigin(uint32_t u32DreqId)
{
    bool bDoStop  = false;
    bool bLateAck = false;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        if (bNodeBeaconing &&
            (u32NodeBeaconDreqId >> 16) == (u32DreqId >> 16))
        {
            bNodeAckedThisCampaign = true;
            bDoStop = MESHNETWORK_bStopBeaconingLocked(u32NodeBeaconDreqId);
        }
        else if (!bNodeBeaconing && u32LastBeaconDreqId != 0U &&
                 (u32LastBeaconDreqId >> 16) == (u32DreqId >> 16))
        {
            /* Ack from the primary we beaconed to, arriving after we already
             * hit the beacon cap. Congestion is exactly what both delays acks
             * and burns the beacon budget, so this is the likely case rather
             * than a rare one — and it used to be dropped on the floor,
             * leaving the node convinced it was never heard. It WAS heard:
             * record that so the next wave doesn't re-beacon for nothing. */
            bNodeAckedThisCampaign = true;
            bLateAck               = true;
        }
        osMutexRelease(xRoleMutex);
    }
    if (bDoStop)
    {
        osTimerStop(xBeaconTimer);
        DBG_LOG("MeshNetwork: Stop beaconing (acked), become forwarder\r\n");
    }
    else if (bLateAck)
    {
        DBG_LOG("MeshNetwork: Late D-Ack after beacon cap - counted, no re-arm\r\n");
    }
}

/* B4: stop whatever dreq this node is currently beaconing (campaign end on a
 * secondary — the caller doesn't know the node's internal beacon dreq). */
void MESHNETWORK_vStopBeaconingSelf(void)
{
    bool bDoStop = false;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        bDoStop = MESHNETWORK_bStopBeaconingLocked(u32NodeBeaconDreqId);
        osMutexRelease(xRoleMutex);
    }
    if (bDoStop)
    {
        osTimerStop(xBeaconTimer);
        DBG_LOG("MeshNetwork: Stop beaconing (campaign end), become forwarder\r\n");
    }
}

static void MESHNETWORK_vStartBeaconing(uint32_t u32DreqId, uint8_t u8HopCount)
{
    bool bDoStart = false;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        if (!(bNodeBeaconing && u32NodeBeaconDreqId == u32DreqId))
        {
            bNodeBeaconing         = true;
            u32NodeBeaconDreqId    = u32DreqId;
            u8NodeHopCount         = u8HopCount;
            eNodeRole              = NODE_ROLE_BEACONING;
            bDoStart               = true;
        }
        osMutexRelease(xRoleMutex);
    }
    if (bDoStart)
    {
        /* Periodic retries... */
        osTimerStart(xBeaconTimer, MESH_BEACON_INTERVAL_MS);
        /* ...plus R1: fire the FIRST beacon immediately (built on the MeshTx
         * full stack), not after a full interval. */
        if (xMeshTxTaskHandle != NULL)
            osThreadFlagsSet(xMeshTxTaskHandle, MESH_TX_FLAG_BEACON);
        DBG_LOG("MeshNetwork: Start beaconing dreq=%08X\r\n", u32DreqId);
    }
}

bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *pBuffer,
                                          uint16_t u16MaxEntries,
                                          uint16_t *pu16ActualEntries)
{
    if (osMutexAcquire(xNeighborTableMutex, 200) != osOK)
    {
        *pu16ActualEntries = 0;
        return false;
    }
    uint16_t u16Count = (u16NeighborCount < u16MaxEntries)
                        ? u16NeighborCount : u16MaxEntries;
    for (uint16_t i = 0; i < u16Count; i++)
    {
        pBuffer[i].u32DeviceId = tNeighborTable[i].u32DeviceId;
        pBuffer[i].u8HopCount  = tNeighborTable[i].u8HopCount;
        pBuffer[i].i16Rssi     = tNeighborTable[i].i16Rssi;
        pBuffer[i].u16BatMv    = tNeighborTable[i].u16BatMv;
        pBuffer[i].u8Wave      = tNeighborTable[i].u8DreqWaveDiscovered;
        pBuffer[i].u8MoveState = tNeighborTable[i].u8MoveState;
        pBuffer[i].u8FwPatch   = tNeighborTable[i].u8FwPatch;
        pBuffer[i].bGpsValid   = tNeighborTable[i].bGpsValid;
        pBuffer[i].i32LatUDeg  = tNeighborTable[i].i32LatUDeg;
        pBuffer[i].i32LonUDeg  = tNeighborTable[i].i32LonUDeg;
        pBuffer[i].u16BestRssiSrcId = tNeighborTable[i].u16BestRssiSrcId;
    }
    *pu16ActualEntries = u16Count;
    osMutexRelease(xNeighborTableMutex);
    return true;
}

void MESHNETWORK_vClearDiscoveredNeighbors(void) { NEIGHBOR_vClearAll(); }

void MESHNETWORK_vSetWakeupInterval(WakeupInterval tNewInterval)
{
    if (tNewInterval <= WAKEUP_INTERVAL_MAX_COUNT)
        tCurrentWakeupInterval = tNewInterval;
}
WakeupInterval MESHNETWORK_tGetWakeupInterval(void) { return tCurrentWakeupInterval; }
uint8_t        MESHNETWORK_u8GetWakeupInterval(void) { return u8CurrentWakeupIntervalMin[tCurrentWakeupInterval]; }

void MESHNETWORK_vSetDiscoveryMode(DiscoveryMode_e eMode)
{
    if (eMode == DISCOVERY_MODE_ADVANCED || eMode == DISCOVERY_MODE_BASIC)
        eCurrentDiscoveryMode = eMode;
}
DiscoveryMode_e MESHNETWORK_eGetDiscoveryMode(void) { return eCurrentDiscoveryMode; }

void MESHNETWORK_vSetGpsEnabled(bool bEnabled) { bCurrentGpsEnabled = bEnabled; }
bool MESHNETWORK_bGetGpsEnabled(void)          { return bCurrentGpsEnabled; }

uint32_t MESHNETWORK_u32GetLastBeaconHeardTick(void)    { return tLastBeaconHeardTick;       }
uint32_t MESHNETWORK_u32GetLastDiscoveryPktTick(void)   { return u32LastDiscoveryPktTick;    }

void MESHNETWORK_vUpdatePrimaryLastSeen(void)   { u64LastPrimaryHeardTick = HAL_RTC_u64GetValue(); }
uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void) { return u64LastPrimaryHeardTick; }

void MESHNETWORK_vStartPrimaryAck(void)
{
    if (xPrimaryAckTimer != NULL)
        osTimerStart(xPrimaryAckTimer, MESH_PRIMARY_ACK_INTERVAL_MS);
}
void MESHNETWORK_vStopPrimaryAck(void)
{
    if (xPrimaryAckTimer != NULL)
        osTimerStop(xPrimaryAckTimer);
}

void MESHNETWORK_vResetTimeSyncAccepted(void)
{
    bTimeSyncAcceptedThisWake = false;
}

void MESHNETWORK_vResetNodeRole(void)
{
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        bNodeBeaconing         = false;
        u32NodeBeaconDreqId    = 0;
        eNodeRole              = NODE_ROLE_UNKNOWN;
        /* Fresh campaign: forget who we beaconed to and whether we were
         * acked, so the per-wave re-arm starts from a clean slate. */
        u32LastBeaconDreqId    = 0;
        bNodeAckedThisCampaign = false;
        osMutexRelease(xRoleMutex);
    }

    /* Drop the stay-awake latch and the DReq dedup history together: both are
     * per-campaign. Keeping either would make the NEXT campaign either skip
     * its silence timeout with no evidence, or refuse to relay a wave-1 flood
     * whose id happened to repeat. */
    bCampaignHeard = false;
    if (osMutexAcquire(xForwardRingMutex, 100) == osOK)
    {
        u8DreqSeenHead  = 0U;
        u8DreqSeenCount = 0U;
        u16DreqFwdCnt   = 0U;   /* forward budgets are per-campaign too */
        osMutexRelease(xForwardRingMutex);
    }

    /* A node that heard a campaign but never beaconed is not covered by the
     * reset in bStopBeaconingLocked, so without this it carries the previous
     * campaign's best RSSI — and now a stale sender id with it — into the next
     * one, and would report a neighbour it has not heard from this wake. */
    BEST_vReset();

    osTimerStop(xBeaconTimer);   /* outside the lock */
}

bool MESHNETWORK_bCampaignHeard(void)
{
    return bCampaignHeard;
}

/* Radio-test support. The worker exists in every build that calls
 * MESHNETWORK_vInit, which is every production build; an ENABLE_RADIO_TEST
 * build skips MeshNetwork entirely, and the runtime test checks this before
 * entering rather than beaconing into a task that is not there. */
bool MESHNETWORK_bTxWorkerReady(void)
{
    return (xMeshTxTaskHandle != NULL);
}

/* Nudge the TX worker so it goes round its loop once and re-evaluates how long
 * it should block. Used on radio-test entry: by then every path that would
 * normally wake this task is gated off, so without a poke it would sit on
 * osWaitForever and the test's beacon deadline would never be picked up. */
void MESHNETWORK_vWakeTxWorker(void)
{
    if (xMeshTxTaskHandle != NULL)
        osThreadFlagsSet(xMeshTxTaskHandle, MESH_TX_FLAG_QUEUE);
}

/* R6: drop any TX still queued (e.g. a late-jittered forward from the previous
 * campaign) so it can't fire at the start of the next one. Call on wake. */
void MESHNETWORK_vFlushTxQueue(void)
{
    if (xMeshTxQueue == NULL) return;
    MeshTxItem_t tItem;
    while (osMessageQueueGet(xMeshTxQueue, &tItem, NULL, 0) == osOK)
    {
        /* Counted, not silent: a flush that routinely discards a lot is the
         * signature of a node queueing relays faster than the channel drains
         * them — the condition that used to leave it deaf for minutes. */
        if (u16StatTxDropped < UINT16_MAX) u16StatTxDropped++;
    }
}

void MESHNETWORK_vIncrDreqWaveCnt(void) { u8PrimaryDreqWaveCnt++; }
void MESHNETWORK_vResetDreqWaveCnt(void)
{
    u8PrimaryDreqWaveCnt = 0;
    u16StatDReqHeard      = 0U;
    u16StatBeaconsHeard   = 0U;
    u16StatAcksHeard      = 0U;
    u16StatMsgsForwarded  = 0U;
    u16StatTxDropped      = 0U;
}

/* One-line campaign traffic summary in place of a DBG_LOG per packet —
 * call at campaign end (or wave end, if a per-wave breakdown is wanted)
 * from DeviceDiscovery.c. Does not reset the counters itself; call
 * MESHNETWORK_vResetDreqWaveCnt() to start a fresh campaign's tally. */
void MESHNETWORK_vLogCampaignStats(const char *pcTag)
{
    /* cadTmo: CAD attempts that timed out this campaign — the congestion
     * indicator that used to be one DBG_LOG line per occurrence. Read-and-
     * clear, so each campaign reports only its own. */
    DBG_LOG("MeshNetwork: %s stats - DReq heard=%u beacons heard=%u acks heard=%u forwarded=%u cadTmo=%u txDrop=%u\r\n",
            pcTag, (unsigned)u16StatDReqHeard, (unsigned)u16StatBeaconsHeard,
            (unsigned)u16StatAcksHeard, (unsigned)u16StatMsgsForwarded,
            (unsigned)LORARADIO_u16GetAndClearCadTimeouts(),
            /* Both layers' discards in one figure: mesh-queue drops/flushes
             * plus the radio layer's aged-out and flushed packets. */
            (unsigned)(u16StatTxDropped + LORARADIO_u16GetAndClearTxStaleDrops()));
}

/* Small OTA responses (PrepAck/Report) ride the normal jittered mesh TX
 * queue — the jitter de-correlates many secondaries answering one OtaPrep,
 * exactly as it does for discovery beacons. */
bool MESHNETWORK_bSendOtaResponse(const uint8_t *buf, uint16_t len)
{
    return MESHNETWORK_bSendPacket(buf, len);
}

__attribute__((weak)) void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len)
{
    (void)buf; (void)len;
}
