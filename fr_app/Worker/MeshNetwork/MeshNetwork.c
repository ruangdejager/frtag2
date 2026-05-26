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
#include "MeshNetwork_Port.h"
#include "LoraRadio.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE */
#include "task.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "dbg_log.h"
#include "DeviceDiscovery.h"
#include "hal_rtc.h"
#include "Battery.h"
#include "flashLog.h"

/* ---- Local config aliases ---- */
#define MESH_BEACON_INTERVAL_MS_CFG      MESH_BEACON_INTERVAL_MS
#define MESH_PRIMARY_ACK_INTERVAL_MS_CFG MESH_PRIMARY_ACK_INTERVAL_MS
#define MESH_DISCOVERY_IDLE_MS_CFG       MESH_DISCOVERY_IDLE_MS

/* ---- TX queue item ---- */
#define MESH_TX_QUEUE_LEN        24
#define MESH_TX_MAX_PACKET_SIZE  128

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

/* ---- Protocol state ---- */
static ForwardRing_t    tForwardRing;
static NeighborEntry_t  tNeighborTable[MESH_MAX_NEIGHBORS];
static uint16_t         u16NeighborCount      = 0;
static uint32_t         tLastBeaconHeardTick  = 0;

static bool       bNodeBeaconing      = false;
static uint32_t   u32NodeBeaconDreqId = 0;
static uint8_t    u8NodeHopCount      = 0;
static NodeRole_e eNodeRole           = NODE_ROLE_UNKNOWN;

/* ---- Wakeup interval ---- */
static WakeupInterval tCurrentWakeupInterval = WAKEUP_INTERVAL_15_MIN;
static const uint8_t u8CurrentWakeupIntervalMin[] = {
    [WAKEUP_INTERVAL_15_MIN]  = 15,
    [WAKEUP_INTERVAL_30_MIN]  = 30,
    [WAKEUP_INTERVAL_60_MIN]  = 60,
    [WAKEUP_INTERVAL_120_MIN] = 120
};

static const char * const MeshPktTypeStr[] = {
    [MeshPktType_Reserved] = "Reserved",
    [MeshPktType_DReq]     = "DReq",
    [MeshPktType_DBeacon]  = "DBeacon",
    [MeshPktType_DAck]     = "DAck",
    [MeshPktType_TimeSync] = "TimeSync"
};

/* ---- Misc state ---- */
static uint16_t u16MsgCounter        = 0;
static uint64_t u64LastPrimaryHeardTick = 0;
static int16_t  i16BestDreqRssi      = -256;
static uint8_t  u8PrimaryDreqWaveCnt = 0;

/* Tick of the most recent received discovery packet (any type).
 * Updated in every handler so DeviceDiscovery can detect mesh activity
 * without needing to track individual packet-type ticks. */
static uint32_t u32LastDiscoveryPktTick = 0;

/* ---- Forward declarations ---- */
static void MESHNETWORK_vTxTask(void *pvParameters);
static bool MESHNETWORK_bSendPacket(const uint8_t *pBuf, size_t u32Len);
static void MESHNETWORK_vStartBeaconing(uint32_t u32DreqId, uint8_t u8HopCount);

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
                                   uint8_t u8DreqWaveDisc)
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
                tNeighborTable[i].u8DreqWaveDiscovered = u8DreqWaveDisc;
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
    if (u32BufLen < 15) return false;
    pBuf[0] = (uint8_t)MeshPktType_DBeacon;
    write_u32_be(&pBuf[1],  ptBeacon->u32DreqId);
    write_u16_be(&pBuf[5],  ptBeacon->u16BatMv);
    pBuf[7] = ptBeacon->u8HopCount;
    write_s16_be(&pBuf[8],  ptBeacon->i16Rssi);
    write_u32_be(&pBuf[10], ptBeacon->u32BeaconMsgId);
    pBuf[14] = ptBeacon->dreqWaveDisc;
    *pu32Written = 15;
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

static bool MESHNETWORK_bEncodeTimeSync(const MeshPktTimeSync_t *ptTS,
                                         uint8_t *pBuf,
                                         size_t u32BufLen,
                                         size_t *pu32Written)
{
    if (u32BufLen < 6) return false;
    pBuf[0] = (uint8_t)MeshPktType_TimeSync;
    write_u32_be(&pBuf[1], ptTS->u32UtcTimestamp);
    pBuf[5] = (uint8_t)ptTS->tWakeupInterval;
    *pu32Written = 6;
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

static uint32_t MESHNETWORK_u32GetTxJitterMs(void)
{
    uint32_t u32Range = MESH_TX_JITTER_MAX_MS - MESH_TX_JITTER_MIN_MS;
    return MESH_TX_JITTER_MIN_MS + (rand() % (u32Range + 1));
}

/* --------------------------------------------------------------------------
 * Timer callbacks (CMSIS-RTOS v2 signature: void cb(void *arg))
 * -------------------------------------------------------------------------- */
static void MESHNETWORK_vBeaconTimerCallback(void *arg)
{
    (void)arg;
    if (!bNodeBeaconing) return;

    MeshPktDBeacon_t tBeacon;
    tBeacon.u32DreqId      = u32NodeBeaconDreqId;
    tBeacon.u32DeviceId    = 0;   /* derived from BeaconMsgId on receive */
    tBeacon.u16BatMv       = BAT_u16GetVoltage();
    tBeacon.u8HopCount     = u8NodeHopCount;
    tBeacon.i16Rssi        = i16BestDreqRssi;
    tBeacon.u32BeaconMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tBeacon.dreqWaveDisc   = u8PrimaryDreqWaveCnt;

    DBG("MeshNetwork: Sending Beacon %08X\r\n", tBeacon.u32BeaconMsgId);
    LOG(LOG_TX_BEACON, 1);

    uint8_t u8Buf[64];
    size_t  u32Len = 0;
    if (!MESHNETWORK_bEncodeDBeacon(&tBeacon, u8Buf, sizeof(u8Buf), &u32Len)) return;
    FORWARD_vAdd(tBeacon.u32BeaconMsgId);
    MESHNETWORK_bSendPacket(u8Buf, u32Len);
}

static void MESHNETWORK_vPrimaryAckTimerCallback(void *arg)
{
    (void)arg;
    MeshPktDAck_t tAck;
    memset(&tAck, 0, sizeof(tAck));
    tAck.u32AckMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tAck.u32DreqId   = u32NodeBeaconDreqId;
    tAck.u32SenderId = LORARADIO_u32GetUniqueId();

    if (osMutexAcquire(xNeighborTableMutex, 100) == osOK)
    {
        uint8_t u8Added = 0;
        for (uint16_t i = 0;
             i < u16NeighborCount && u8Added < MESH_MAX_ACK_IDS_PER_PACKET;
             i++)
        {
            if (!tNeighborTable[i].bAcked)
            {
                tAck.u32AckedIds[u8Added++] = tNeighborTable[i].u32DeviceId;
                tNeighborTable[i].bAcked    = true;
            }
        }
        tAck.u8AckCount = u8Added;
        osMutexRelease(xNeighborTableMutex);

        if (tAck.u8AckCount > 0)
        {
            uint8_t u8Buf[128];
            size_t  u32Len = 0;
            if (MESHNETWORK_bEncodeDAck(&tAck, u8Buf, sizeof(u8Buf), &u32Len))
            {
                FORWARD_vAdd(tAck.u32AckMsgId);
                MESHNETWORK_bSendPacket(u8Buf, u32Len);
                LOG(LOG_TX_ACK, 1);
            }
        }
    }
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
        DBG("MeshNetwork: TX queue full, dropping packet\r\n");
        return false;
    }

    DBG("MeshNetwork: Queued TX (len=%u, jitter=%lu ms)\r\n",
        (unsigned)u32Len, jitterMs);
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

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)u32OriginId, s16Rssi, u8WaveCnt);
    LOG(LOG_RX_DREQ, u32LogValue);

    if (u32OriginId == LORARADIO_u32GetUniqueId()) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

#ifndef LISTENER_MODE
    if (eNodeRole == NODE_ROLE_FORWARDER)
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
                LOG(LOG_TX_DREQ, 2);
            }
        }
        else
        {
            DBG("MeshNetwork: DReq seen before\r\n");
        }
    }
    else if (eNodeRole != NODE_ROLE_BEACONING)
    {
        MESHNETWORK_vStartBeaconing(u32DreqId, (uint8_t)(u8SenderHopCount + 1));
        u8PrimaryDreqWaveCnt = u8WaveCnt;
    }
#endif

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

    DBG("MeshNetwork: Beacon: dev=%04X dreq=%08X hop=%u wave=%X bat=%u rssi=%d\r\n",
        tBeacon.u32DeviceId, tBeacon.u32DreqId, tBeacon.u8HopCount,
        tBeacon.dreqWaveDisc, tBeacon.u16BatMv, tBeacon.i16Rssi);

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)tBeacon.u32DeviceId,
                               tBeacon.i16Rssi, 0);
    LOG(LOG_RX_BEACON, u32LogValue);

    /* 1. Deduplicate */
    if (FORWARD_bHasSeen(tBeacon.u32BeaconMsgId))
    {
        DBG("MeshNetwork: Beacon seen before\r\n");
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
                              tBeacon.dreqWaveDisc);
        tLastBeaconHeardTick = osKernelGetTickCount();
        return;
    }

#ifndef LISTENER_MODE
    /* 4. Secondary forwarder: relay beacon */
    if (eNodeRole == NODE_ROLE_FORWARDER)
    {
        tBeacon.u8HopCount++;
        uint8_t u8Buf[64];
        size_t  u32TempLen = 0;
        if (MESHNETWORK_bEncodeDBeacon(&tBeacon, u8Buf, sizeof(u8Buf), &u32TempLen))
        {
            MESHNETWORK_bSendPacket(u8Buf, u32TempLen);
            DBG("MeshNetwork: Forwarding Beacon\r\n");
            LOG(LOG_TX_BEACON, 2);
        }
    }
#endif
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
    if (u32Len < (size_t)(10 + 4 * u8AckCount)) return;

    uint32_t u32Ids[MESH_MAX_ACK_IDS_PER_PACKET];
    for (uint8_t i = 0; i < u8AckCount; i++)
        u32Ids[i] = read_u32_be(&pBuf[10 + 4 * i]);

    DBG("MeshNetwork: DAck: ackId=%08X dreq=%08X count=%u\r\n",
        u32AckMsgId, u32DreqId, u8AckCount);

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, (uint16_t)(u32DreqId >> 16),
                               s16Rssi, u8AckCount);
    LOG(LOG_RX_ACK, u32LogValue);

    if (FORWARD_bHasSeen(u32AckMsgId))
    {
        DBG("MeshNetwork: Ack seen before\r\n");
        return;
    }
    FORWARD_vAdd(u32AckMsgId);

#ifndef LISTENER_MODE
    if (eNodeRole == NODE_ROLE_FORWARDER)
    {
        MESHNETWORK_bSendPacket(pBuf, u32Len);
        DBG("MeshNetwork: Ack forwarded\r\n");
        LOG(LOG_TX_ACK, 2);
    }

    uint32_t u32MyId = LORARADIO_u32GetUniqueId();
    for (uint8_t i = 0; i < u8AckCount; i++)
    {
        if (u32Ids[i] == u32MyId)
        {
            MESHNETWORK_vStopBeaconing(u32DreqId);
            break;
        }
    }
#endif
}

static void MESHNETWORK_vHandleTimeSync(const uint8_t *pBuf,
                                         size_t u32Len,
                                         int16_t s16Rssi)
{
    if (u32Len < 6) return;

    u32LastDiscoveryPktTick = osKernelGetTickCount();

    uint32_t       u32Utc    = read_u32_be(&pBuf[1]);
    WakeupInterval tInterval = (WakeupInterval)pBuf[5];

    DBG("MeshNetwork: TimeSync: utc=%u interval=%u\r\n", u32Utc, tInterval);

    uint32_t u32LogValue;
    FLASHLOG_vEncodeRXLogValue(&u32LogValue, 0, s16Rssi, 0);
    LOG(LOG_RX_TS, u32LogValue);

    if (FORWARD_bHasSeen(u32Utc))
    {
        DBG("MeshNetwork: TimeSync seen before\r\n");
        return;
    }
    FORWARD_vAdd(u32Utc);

    RTC_vSetUTC(u32Utc);
    MESHNETWORK_vSetWakeupInterval(tInterval);
    MESHNETWORK_vUpdatePrimaryLastSeen();
    DBG("MeshNetwork: TimeSync applied: %u interval=%u\r\n", u32Utc, tInterval);

#ifndef LISTENER_MODE
    MESHNETWORK_bSendPacket(pBuf, u32Len);
    DBG("MeshNetwork: TimeSync forwarded\r\n");
    LOG(LOG_TX_TS, 2);
#endif

    /* Notify DeviceDiscovery task that the round is complete */
    osThreadId_t xAppTask = DEVICE_DISCOVERY_xGetTaskHandle();
    if (xAppTask != NULL)
        osThreadFlagsSet(xAppTask, DEVICE_DISCOVERY_NOTIFY_TIMESYNC);
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
        switch (eType)
        {
            case MeshPktType_DReq:
                MESHNETWORK_vHandleDReq(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_DBeacon:
                MESHNETWORK_vHandleDBeacon(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_DAck:
                MESHNETWORK_vHandleDAck(tRx.buffer, tRx.length, tRx.rssi);
                break;
            case MeshPktType_TimeSync:
                MESHNETWORK_vHandleTimeSync(tRx.buffer, tRx.length, tRx.rssi);
                break;
            default:
                DBG("MeshNetwork: Unknown pkt type %u\r\n", tRx.buffer[0]);
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
        if (osMessageQueueGet(xMeshTxQueue, &tItem, NULL, osWaitForever) == osOK)
        {
            uint32_t now = osKernelGetTickCount();
            if (tItem.u32ReadyTick > now)
            {
                /* Not ready yet — requeue and wait */
                osMessageQueuePut(xMeshTxQueue, &tItem, 0, 0);
                osDelay(10);
                continue;
            }
            DBG("MeshNetwork: TX (len=%u)\r\n", tItem.u16Len);
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
    DBG("MeshNetwork: DReq %08X sent\r\n", u32DreqId);
    LOG(LOG_TX_DREQ, 1);
    return true;
}

void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval)
{
    MeshPktTimeSync_t tTs = {
        .u32UtcTimestamp = u32UtcTimestamp,
        .tWakeupInterval = tWakeupInterval
    };
    uint8_t u8Buf[16];
    size_t  u32Len = 0;
    if (!MESHNETWORK_bEncodeTimeSync(&tTs, u8Buf, sizeof(u8Buf), &u32Len)) return;
    FORWARD_vAdd(u32UtcTimestamp);
    MESHNETWORK_bSendPacket(u8Buf, u32Len);
    DBG("MeshNetwork: TimeSync sent %u interval=%u\r\n",
        u32UtcTimestamp, tWakeupInterval);
}

static void MESHNETWORK_vStartBeaconing(uint32_t u32DreqId, uint8_t u8HopCount)
{
    if (bNodeBeaconing && u32NodeBeaconDreqId == u32DreqId) return;
    bNodeBeaconing      = true;
    u32NodeBeaconDreqId = u32DreqId;
    u8NodeHopCount      = u8HopCount;
    eNodeRole           = NODE_ROLE_BEACONING;
    osTimerStart(xBeaconTimer, MESH_BEACON_INTERVAL_MS_CFG);
    DBG("MeshNetwork: Start beaconing dreq=%08X\r\n", u32DreqId);
}

void MESHNETWORK_vStopBeaconing(uint32_t u32DreqId)
{
    if (!bNodeBeaconing)              return;
    if (u32NodeBeaconDreqId != u32DreqId) return;
    bNodeBeaconing      = false;
    i16BestDreqRssi     = -256;
    u32NodeBeaconDreqId = 0;
    osTimerStop(xBeaconTimer);
    eNodeRole = NODE_ROLE_FORWARDER;
    DBG("MeshNetwork: Stop beaconing, become forwarder\r\n");
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

uint32_t MESHNETWORK_u32GetLastBeaconHeardTick(void)    { return tLastBeaconHeardTick;       }
uint32_t MESHNETWORK_u32GetLastDiscoveryPktTick(void)   { return u32LastDiscoveryPktTick;    }

void MESHNETWORK_vUpdatePrimaryLastSeen(void)   { u64LastPrimaryHeardTick = HAL_RTC_u64GetValue(); }
uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void) { return u64LastPrimaryHeardTick; }

void MESHNETWORK_vStartPrimaryAck(void)
{
    if (xPrimaryAckTimer != NULL)
        osTimerStart(xPrimaryAckTimer, MESH_PRIMARY_ACK_INTERVAL_MS_CFG);
}
void MESHNETWORK_vStopPrimaryAck(void)
{
    if (xPrimaryAckTimer != NULL)
        osTimerStop(xPrimaryAckTimer);
}

void MESHNETWORK_vResetNodeRole(void)
{
    bNodeBeaconing      = false;
    u32NodeBeaconDreqId = 0;
    osTimerStop(xBeaconTimer);
    eNodeRole = NODE_ROLE_UNKNOWN;
}

void MESHNETWORK_vIncrDreqWaveCnt(void) { u8PrimaryDreqWaveCnt++; }
void MESHNETWORK_vResetDreqWaveCnt(void) { u8PrimaryDreqWaveCnt = 0; }
