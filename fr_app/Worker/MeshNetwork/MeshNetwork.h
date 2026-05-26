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
#define MESH_BEACON_INTERVAL_MS       5000U
#define MESH_PRIMARY_ACK_INTERVAL_MS  2000U
#define MESH_DISCOVERY_IDLE_MS        7000U
#define FORWARD_RING_SIZE             32
#define MESH_MAX_NEIGHBORS            128

#define MESH_TX_JITTER_MIN_MS         0U
#define MESH_TX_JITTER_MAX_MS         500U

/* ---- Packet types (wire, first byte) ---- */
typedef enum {
    MeshPktType_Reserved = 0,
    MeshPktType_DReq     = 1,
    MeshPktType_DBeacon  = 2,
    MeshPktType_DAck     = 3,
    MeshPktType_TimeSync = 4
} MeshPktType_e;

/* ---- Wake-up interval enum ---- */
typedef enum {
    WAKEUP_INTERVAL_15_MIN  = 1,
    WAKEUP_INTERVAL_30_MIN  = 2,
    WAKEUP_INTERVAL_60_MIN  = 3,
    WAKEUP_INTERVAL_120_MIN = 4,
    WAKEUP_INTERVAL_MAX_COUNT = 4
} WakeupInterval;

/* ---- On-wire packet structs ---- */
typedef struct {
    uint32_t u32DreqId;
    uint32_t u32OriginId;
    uint32_t u32SenderId;
    uint8_t  u8SenderHopCount;
} MeshPktDReq_t;

typedef struct {
    uint32_t u32DreqId;
    uint32_t u32DeviceId;
    uint16_t u16BatMv;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint32_t u32BeaconMsgId;
    uint8_t  dreqWaveDisc;
} MeshPktDBeacon_t;

#define MESH_MAX_ACK_IDS_PER_PACKET 8
typedef struct {
    uint32_t u32AckMsgId;
    uint32_t u32DreqId;
    uint32_t u32SenderId;
    uint8_t  u8AckCount;
    uint32_t u32AckedIds[MESH_MAX_ACK_IDS_PER_PACKET];
} MeshPktDAck_t;

typedef struct {
    uint32_t     u32UtcTimestamp;
    WakeupInterval tWakeupInterval;
} MeshPktTimeSync_t;

/* ---- Forward ring ---- */
typedef struct {
    uint32_t u32Ring[FORWARD_RING_SIZE];
    uint8_t  u8Head;
    uint8_t  u8Count;
} ForwardRing_t;

/* ---- Neighbor entry (internal) ---- */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8HopCount;
    int16_t  i16Rssi;
    uint16_t u16BatMv;
    uint8_t  u8DreqWaveDiscovered;
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
} MeshDiscoveredNeighbor_t;

/* ---- Public API ---- */
void MESHNETWORK_vInit(void);
void MESHNETWORK_vParserTask(void *pvParameters);

bool MESHNETWORK_bStartDiscoveryRound(uint32_t u32DreqId);
void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval);

void MESHNETWORK_vStopBeaconing(uint32_t u32DreqId);
bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *pBuffer,
                                         uint16_t u16MaxEntries,
                                         uint16_t *pu16ActualEntries);
void MESHNETWORK_vClearDiscoveredNeighbors(void);

uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void);

void          MESHNETWORK_vSetWakeupInterval(WakeupInterval tNewInterval);
WakeupInterval MESHNETWORK_tGetWakeupInterval(void);
uint8_t       MESHNETWORK_u8GetWakeupInterval(void);

uint32_t MESHNETWORK_u32GetLastBeaconHeardTick(void);

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

#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */
