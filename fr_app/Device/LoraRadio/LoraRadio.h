/*
 * LoraRadio.h
 *
 * LoRa radio device layer — application-level API.
 *
 * This layer sits above LoraRadio_Driver (SX126x/SUBGRF) and manages the
 * radio task, TX/RX queues and carrier-sense back-off using CMSIS-RTOS v2.
 */

#ifndef DEVICE_LORARADIO_LORARADIO_H_
#define DEVICE_LORARADIO_LORARADIO_H_

#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

#include "LoraRadio_Config.h"

/* Radio event bits — used with osThreadFlagsSet / osThreadFlagsWait */
#define RADIO_EVT_RX_DONE       (1UL << 0)
#define RADIO_EVT_TX_DONE       (1UL << 1)
#define RADIO_EVT_CRC_ERROR     (1UL << 2)
#define RADIO_EVT_HEADER_ERROR  (1UL << 3)
#define RADIO_EVT_TIMEOUT       (1UL << 4)
#define RADIO_EVT_CAD_CLEAR     (1UL << 5)
#define RADIO_EVT_CAD_BUSY      (1UL << 6)
#define RADIO_EVT_TX_PENDING    (1UL << 7)

typedef struct {
    uint8_t  buffer[LORA_MAX_PACKET_SIZE];
    uint8_t  length;
    int16_t  rssi;
    int8_t   snr;
    /* Tick at which this packet was queued for TX. Set by the radio layer on
     * enqueue (callers leave it alone); used to discard packets that sat in
     * the queue so long that sending them is pointless — see
     * LORA_TX_MAX_AGE_MS. Unused on the RX path. */
    uint32_t u32QueuedTick;
} LoraRadio_Packet_t;

void     LORARADIO_vInit(void);
bool     LORARADIO_bRxPacket(LoraRadio_Packet_t *packet);
bool     LORARADIO_bTxPacket(LoraRadio_Packet_t *packet);

/* Same as LORARADIO_bTxPacket, but blocks the calling task (instead of
 * dropping the packet) when the TX queue is momentarily full, up to
 * timeoutMs. The queue only frees a slot once the radio task has actually
 * finished with the packet ahead of it (transmitted or dropped after
 * carrier-sense/back-off), so this lets a bulk sender (e.g. streaming the
 * ext-flash log over LoRa) pace itself off real radio progress instead of a
 * guessed fixed delay between sends. Not for latency-sensitive single
 * responses -- those should keep using the non-blocking LORARADIO_bTxPacket. */
bool     LORARADIO_bTxPacketWait(LoraRadio_Packet_t *packet, uint32_t timeoutMs);

void     LORARADIO_vRadioTask(void *arg);
uint32_t LORARADIO_u32GetUniqueId(void);

void LORARADIO_vEventRxDone(void);
void LORARADIO_vEventTxDone(void);
void LORARADIO_vEventCrcError(void);
void LORARADIO_vEventHeaderError(void);
void LORARADIO_vEventTimeout(void);
void LORARADIO_vEventCADDetected(void);
void LORARADIO_vEventCADClear(void);

uint32_t LORARADIO_u32GetRandomNumber(uint32_t max_value);
bool     LORARADIO_bCarrierSense(void);
bool     LORARADIO_bCarrierSenseAndWait(uint32_t maxWaitMs);

/* Number of CAD attempts that timed out (no CAD result inside the 300 ms
 * window) since the last call; reading clears the tally. Reported once per
 * campaign in the mesh stats line rather than logged per occurrence, which
 * under congestion ran to hundreds of lines and buried everything else. */
uint16_t LORARADIO_u16GetAndClearCadTimeouts(void);

/* Packets discarded unsent since the last call — aged out of the TX queue or
 * dropped by LORARADIO_vFlushTxQueue(). Reading clears the tally. Reported
 * alongside the CAD count in the per-campaign mesh stats line. */
uint16_t LORARADIO_u16GetAndClearTxStaleDrops(void);

/* osKernelGetTickCount() at the moment the PA was last actually fired, or 0
 * if it has not transmitted since boot. Does NOT clear on read.
 *
 * Diagnostic: this is the only way to correlate a flash-read fault with the
 * TX current spike that OTA_LORA_CHUNK_GAP_MS was measured against, because
 * queue time and PA time are decoupled by the mesh jitter (20-1500 ms) plus
 * carrier sense (up to LORA_TX_CARRIER_WAIT_MS). Compare against a read
 * window with signed tick arithmetic so wrap is handled. */
uint32_t LORARADIO_u32GetLastTxTick(void);

/* True when no PA pulse is pending or recent: nothing queued, nothing already
 * dequeued and awaiting carrier sense, and at least u32SettleMs has passed
 * since the last transmission actually fired.
 *
 * Exists for callers whose work is disturbed by the TX current spike rather
 * than by radio airtime — specifically the whole-image flash verify passes in
 * Fota.c, where a TX landing mid-pass corrupts the bytes read back (see
 * OTA_VERIFY_TX_SETTLE_MS and OTA_LORA_CHUNK_GAP_MS). Cheap: three reads of
 * volatile state, no locking, safe to poll. */
bool     LORARADIO_bTxQuiet(uint32_t u32SettleMs);

/* Discard everything still queued for TX. Call when the work that queued it
 * is over (campaign end / before sleep): otherwise the radio task keeps
 * carrier-sensing its way through a stale backlog for minutes afterwards,
 * burning power and — because carrier sense runs the chip in CAD rather than
 * RX — staying deaf well into the next event it should have heard. */
void LORARADIO_vFlushTxQueue(void);

void LORARADIO_vEnterDeepSleep(void);
void LORARADIO_vWakeUp(void);

#endif /* DEVICE_LORARADIO_LORARADIO_H_ */
