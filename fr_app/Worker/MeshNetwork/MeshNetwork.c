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

/* ---- DBeacon flags byte (byte 15) ---- */
#define MESH_BEACON_FLAG_STILL      0x01U   /* bit0: 1 = still, 0 = moving */
#define MESH_BEACON_FLAG_GPS_VALID  0x02U   /* bit1: 1 = lat/lon present   */

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
 *
 * A decoder that sees fewer than MESH_BEACON_LEN_BASE bytes (an older peer
 * that hasn't been updated yet) still parses cleanly — see the length-guarded
 * reads in MESHNETWORK_vHandleDBeacon. */
#define MESH_BEACON_LEN_BASE        17U     /* through the FwPatch byte    */
#define MESH_BEACON_LEN_GPS         25U     /* with lat/lon appended       */

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
 * 10 + 8*4 = 42 B (beacon 24, DReq 7, TimeSync 6); FrKernel responses bypass
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

static bool       bNodeBeaconing      = false;
static uint32_t   u32NodeBeaconDreqId = 0;
static uint8_t    u8NodeHopCount      = 0;
static NodeRole_e eNodeRole           = NODE_ROLE_UNKNOWN;

/* R2: bound beaconing so a lost D-Ack can't keep a node beaconing all campaign. */
static uint8_t    u8NodeBeaconCount      = 0;
static uint32_t   u32NodeBeaconStartTick = 0;

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
static int16_t  i16BestDreqRssi      = -256;
static uint8_t  u8PrimaryDreqWaveCnt = 0;

/* Campaign-level traffic counters — a one-line DBG_LOG summary in place of
 * a DBG_LOG per packet. Reset at MESHNETWORK_vResetDreqWaveCnt() (already
 * called at every campaign start); read/logged via
 * MESHNETWORK_vLogCampaignStats() at campaign end. */
static uint16_t u16StatDReqHeard;
static uint16_t u16StatBeaconsHeard;
static uint16_t u16StatAcksHeard;
static uint16_t u16StatMsgsForwarded;

/* Tick of the most recent received discovery packet (any type).
 * Updated in every handler so DeviceDiscovery can detect mesh activity
 * without needing to track individual packet-type ticks. */
static uint32_t u32LastDiscoveryPktTick = 0;

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
 * Neighbor table
 * -------------------------------------------------------------------------- */
static void NEIGHBOR_vAddOrUpdate(uint32_t u32DeviceId, uint8_t u8HopCount,
                                   int16_t i16Rssi, uint16_t u16BatMv,
                                   uint8_t u8DreqWaveDisc,
                                   uint8_t u8MoveState, uint8_t u8FwPatch,
                                   bool bGpsValid,
                                   int32_t i32LatUDeg, int32_t i32LonUDeg)
{
    if (osMutexAcquire(xNeighborTableMutex, 100) == osOK)
    {
        for (uint16_t i = 0; i < u16NeighborCount; i++)
        {
            if (tNeighborTable[i].u32DeviceId == u32DeviceId)
            {
                tNeighborTable[i].u8HopCount          = u8HopCount;
                tNeighborTable[i].u16BatMv             = u16BatMv;
                tNeighborTable[i].i16Rssi              = i16Rssi;
                /* R7: keep the FIRST wave that discovered this node — it marks
                 * the earliest (typically closest) contact; do not overwrite
                 * with later waves. (u8DreqWaveDisc unused in the update path.) */
                tNeighborTable[i].u8MoveState          = u8MoveState;
                tNeighborTable[i].u8FwPatch            = u8FwPatch;
                /* Keep the last known fix if this beacon carries none. */
                if (bGpsValid)
                {
                    tNeighborTable[i].bGpsValid  = true;
                    tNeighborTable[i].i32LatUDeg = i32LatUDeg;
                    tNeighborTable[i].i32LonUDeg = i32LonUDeg;
                }
                if (tNeighborTable[i].bAcked) tNeighborTable[i].bAcked = false;
                osMutexRelease(xNeighborTableMutex);
                tLastBeaconHeardTick = osKernelGetTickCount();
                return;
            }
        }
        if (u16NeighborCount < MESH_MAX_NEIGHBORS)
        {
            tNeighborTable[u16NeighborCount].u32DeviceId          = u32DeviceId;
            tNeighborTable[u16NeighborCount].u8HopCount           = u8HopCount;
            tNeighborTable[u16NeighborCount].u16BatMv             = u16BatMv;
            tNeighborTable[u16NeighborCount].i16Rssi              = i16Rssi;
            tNeighborTable[u16NeighborCount].u8DreqWaveDiscovered = u8DreqWaveDisc;
            tNeighborTable[u16NeighborCount].u8MoveState          = u8MoveState;
            tNeighborTable[u16NeighborCount].u8FwPatch            = u8FwPatch;
            tNeighborTable[u16NeighborCount].bGpsValid            = bGpsValid;
            tNeighborTable[u16NeighborCount].i32LatUDeg           = i32LatUDeg;
            tNeighborTable[u16NeighborCount].i32LonUDeg           = i32LonUDeg;
            tNeighborTable[u16NeighborCount].bAcked               = false;
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
static bool MESHNETWORK_bEncodeDReq(uint32_t u32DreqId,
                                     uint8_t u8SenderHopCount,
                                     uint8_t u8WaveCnt,
                                     uint8_t *pBuf,
                                     size_t u32BufLen,
                                     size_t *pu32Written)
{
    if (u32BufLen < 7) return false;
    pBuf[0] = (uint8_t)MeshPktType_DReq;
    write_u32_be(&pBuf[1], u32DreqId);
    pBuf[5] = u8SenderHopCount;
    pBuf[6] = u8WaveCnt;
    *pu32Written = 7;
    return true;
}

static bool MESHNETWORK_bEncodeDBeacon(const MeshPktDBeacon_t *ptBeacon,
                                        uint8_t *pBuf,
                                        size_t u32BufLen,
                                        size_t *pu32Written)
{
    size_t u32Needed = ptBeacon->bGpsValid ? MESH_BEACON_LEN_GPS
                                            : MESH_BEACON_LEN_BASE;
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
    if (ptBeacon->u8MoveState != 0U) u8Flags |= MESH_BEACON_FLAG_STILL;
    if (ptBeacon->bGpsValid)         u8Flags |= MESH_BEACON_FLAG_GPS_VALID;
    pBuf[15] = u8Flags;
    pBuf[16] = ptBeacon->u8FwPatch;

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
    tBeacon.i16Rssi        = i16BestDreqRssi;
    tBeacon.u32BeaconMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tBeacon.dreqWaveDisc   = u8PrimaryDreqWaveCnt;
    tBeacon.u8FwPatch      = (uint8_t)VERSION_SW_PATCH;

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

    /* R2: bound beaconing — count this beacon; once the count or time cap is
     * hit, stop beaconing and become a forwarder even if no D-Ack arrived (lost
     * D-Ack safety). The campaign then ends via the secondary silence rule. */
    bool bDoStop = false;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        if (bNodeBeaconing)
        {
            u8NodeBeaconCount++;
            uint32_t u32Now = osKernelGetTickCount();
            if (u8NodeBeaconCount >= MESH_MAX_BEACONS_PER_CAMPAIGN ||
                (u32Now - u32NodeBeaconStartTick) >= MESH_MAX_BEACON_DURATION_MS)
            {
                bDoStop = MESHNETWORK_bStopBeaconingLocked(u32NodeBeaconDreqId);
            }
        }
        osMutexRelease(xRoleMutex);
    }
    if (bDoStop)
    {
        osTimerStop(xBeaconTimer);
        DBG_LOG("MeshNetwork: Beacon cap reached, become forwarder\r\n");
    }
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
 * MESHNETWORK_bSendPacket — enqueue with TX jitter
 * -------------------------------------------------------------------------- */
static bool MESHNETWORK_bSendPacket(const uint8_t *pBuf, size_t u32Len)
{
    if (pBuf == NULL || u32Len == 0 || u32Len > MESH_TX_MAX_PACKET_SIZE)
        return false;

    MeshTxItem_t tItem;
    memcpy(tItem.u8Buf, pBuf, u32Len);
    tItem.u16Len = (uint16_t)u32Len;

    uint32_t jitterMs      = MESHNETWORK_u32GetTxJitterMs();
    tItem.u32ReadyTick     = osKernelGetTickCount() + jitterMs;

    if (osMessageQueuePut(xMeshTxQueue, &tItem, 0, 50) != osOK)
    {
        DBG_LOG("MeshNetwork: TX queue full, dropping packet\r\n");
        return false;
    }

    /* Wake the TX worker to drain the queue. */
    if (xMeshTxTaskHandle != NULL)
        osThreadFlagsSet(xMeshTxTaskHandle, MESH_TX_FLAG_QUEUE);

#ifdef MESH_LOG_VERBOSE
    DBG("MeshNetwork: Queued TX (len=%u, jitter=%lu ms)\r\n",
        (unsigned)u32Len, jitterMs);
#endif
    return true;
}

/* --------------------------------------------------------------------------
 * Incoming packet handlers
 * -------------------------------------------------------------------------- */
static void MESHNETWORK_vHandleDReq(const uint8_t *pBuf,
                                     size_t u32Len,
                                     int16_t s16Rssi)
{
    if (u32Len < 7) return;
    uint32_t u32DreqId        = read_u32_be(&pBuf[1]);
    uint32_t u32OriginId      = u32DreqId >> 16;
    uint8_t  u8SenderHopCount = pBuf[5];
    uint8_t  u8WaveCnt        = pBuf[6];

    DBG("MeshNetwork: DReq: dreq=%08X origin=%04X hop=%u rssi=%d\r\n",
        u32DreqId, u32OriginId, u8SenderHopCount, s16Rssi);
    u16StatDReqHeard++;

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)u32OriginId, s16Rssi, u8WaveCnt);
    EVTLOG(LOG_RX_DREQ, u32LogValue);

    if (u32OriginId == LORARADIO_u32GetUniqueId()) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    /* Multi-primary: primaries never beacon and never forward.
     * They observe other primaries' DReqs (e.g. for RSSI tracking
     * below) but take no action on them. */
    if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY)
    {
        NodeRole_e eRole = MESHNETWORK_eGetRole();
        if (eRole == NODE_ROLE_FORWARDER)
        {
            if (!FORWARD_bHasSeen(u32DreqId))
            {
                uint8_t u8Out[32];
                size_t  u32OutLen = 0;
                if (MESHNETWORK_bEncodeDReq(u32DreqId,
                                            (uint8_t)(u8SenderHopCount + 1),
                                            u8WaveCnt,
                                            u8Out, sizeof(u8Out), &u32OutLen))
                {
                    FORWARD_vAdd(u32DreqId);
                    MESHNETWORK_bSendPacket(u8Out, u32OutLen);
                    DBG("MeshNetwork: DReq forwarded\r\n");
                    u16StatMsgsForwarded++;
                    EVTLOG(LOG_TX_DREQ, 2);
                }
            }
#ifdef MESH_LOG_VERBOSE
            else
            {
                DBG("MeshNetwork: DReq seen before\r\n");
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
                 * MeshTx task, which stamps i16BestDreqRssi into the beacon; if we
                 * left the RSSI to the max-update below (which runs after
                 * vStartBeaconing) that first beacon would carry the -256 reset
                 * value - exactly the -256 RSSI seen at the primary. */
                u8PrimaryDreqWaveCnt = u8WaveCnt;
                i16BestDreqRssi      = s16Rssi;
                MESHNETWORK_vStartBeaconing(u32DreqId, (uint8_t)(u8SenderHopCount + 1));
            }
        }
    }

    if ((u8PrimaryDreqWaveCnt == u8WaveCnt) && (s16Rssi > i16BestDreqRssi))
        i16BestDreqRssi = s16Rssi;
}

static void MESHNETWORK_vHandleDBeacon(const uint8_t *pBuf,
                                        size_t u32Len,
                                        int16_t s16Rssi)
{
    if (u32Len < 14) return;

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
    }

    DBG("MeshNetwork: Beacon: dev=%04X dreq=%08X hop=%u wave=%X bat=%u rssi=%d move=%u gps=%u fwp=%u\r\n",
        tBeacon.u32DeviceId, tBeacon.u32DreqId, tBeacon.u8HopCount,
        tBeacon.dreqWaveDisc, tBeacon.u16BatMv, tBeacon.i16Rssi,
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
        NEIGHBOR_vAddOrUpdate(tBeacon.u32DeviceId, tBeacon.u8HopCount,
                              tBeacon.i16Rssi, tBeacon.u16BatMv,
                              tBeacon.dreqWaveDisc, tBeacon.u8MoveState,
                              tBeacon.u8FwPatch,
                              tBeacon.bGpsValid, tBeacon.i32LatUDeg,
                              tBeacon.i32LonUDeg);
        tLastBeaconHeardTick = osKernelGetTickCount();
        return;
    }

    /* 4. Secondary forwarder: relay beacon */
    if (MESHNETWORK_eGetRole() == NODE_ROLE_FORWARDER)
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

    if (MESHNETWORK_eGetRole() == NODE_ROLE_FORWARDER)
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
        /* Block until something needs doing: a periodic timer asked us to build
         * a beacon/ack, or a packet was queued for transmission. Blocking on
         * flags (not a busy poll) keeps the device asleep between events. */
        uint32_t u32Flags = osThreadFlagsWait(MESH_TX_FLAG_ANY,
                                              osFlagsWaitAny, osWaitForever);
        if (u32Flags & osFlagsError) continue;

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
    if (!MESHNETWORK_bEncodeDReq(u32DreqId, 0, u8PrimaryDreqWaveCnt,
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
    i16BestDreqRssi     = -256;
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
    bool bDoStop = false;
    if (osMutexAcquire(xRoleMutex, 100) == osOK)
    {
        if (bNodeBeaconing &&
            (u32NodeBeaconDreqId >> 16) == (u32DreqId >> 16))
        {
            bDoStop = MESHNETWORK_bStopBeaconingLocked(u32NodeBeaconDreqId);
        }
        osMutexRelease(xRoleMutex);
    }
    if (bDoStop)
    {
        osTimerStop(xBeaconTimer);
        DBG_LOG("MeshNetwork: Stop beaconing (acked), become forwarder\r\n");
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
            u8NodeBeaconCount      = 0;
            u32NodeBeaconStartTick = osKernelGetTickCount();
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
        bNodeBeaconing      = false;
        u32NodeBeaconDreqId = 0;
        eNodeRole           = NODE_ROLE_UNKNOWN;
        osMutexRelease(xRoleMutex);
    }
    osTimerStop(xBeaconTimer);   /* outside the lock */
}

/* R6: drop any TX still queued (e.g. a late-jittered forward from the previous
 * campaign) so it can't fire at the start of the next one. Call on wake. */
void MESHNETWORK_vFlushTxQueue(void)
{
    if (xMeshTxQueue == NULL) return;
    MeshTxItem_t tItem;
    while (osMessageQueueGet(xMeshTxQueue, &tItem, NULL, 0) == osOK)
        ; /* discard */
}

void MESHNETWORK_vIncrDreqWaveCnt(void) { u8PrimaryDreqWaveCnt++; }
void MESHNETWORK_vResetDreqWaveCnt(void)
{
    u8PrimaryDreqWaveCnt = 0;
    u16StatDReqHeard      = 0U;
    u16StatBeaconsHeard   = 0U;
    u16StatAcksHeard      = 0U;
    u16StatMsgsForwarded  = 0U;
}

/* One-line campaign traffic summary in place of a DBG_LOG per packet —
 * call at campaign end (or wave end, if a per-wave breakdown is wanted)
 * from DeviceDiscovery.c. Does not reset the counters itself; call
 * MESHNETWORK_vResetDreqWaveCnt() to start a fresh campaign's tally. */
void MESHNETWORK_vLogCampaignStats(const char *pcTag)
{
    DBG_LOG("MeshNetwork: %s stats - DReq heard=%u beacons heard=%u acks heard=%u forwarded=%u\r\n",
            pcTag, (unsigned)u16StatDReqHeard, (unsigned)u16StatBeaconsHeard,
            (unsigned)u16StatAcksHeard, (unsigned)u16StatMsgsForwarded);
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
