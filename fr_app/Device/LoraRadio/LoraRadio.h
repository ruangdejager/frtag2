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

/* Included here, not left to callers: ENABLE_RADIO_TEST adds a field to
 * LoraRadio_Packet_t below, and that struct is the RX/TX queue element. Every
 * translation unit must agree on its size, so the flag has to be visible
 * wherever this header is. */
#include "build_config.h"
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
#ifdef ENABLE_RADIO_TEST
    /* Channel noise floor in dBm, sampled by the radio task on the last idle
     * pass before this packet arrived — i.e. with the channel quiet, which is
     * what makes it a noise floor rather than just another RSSI. Compare
     * against rssi for the link margin. RX path only; LORA_NOISE_FLOOR_INVALID
     * until the first sample lands. Bench-only, so production builds keep
     * their queue-element size unchanged. */
    int16_t  i16NoiseFloor;
#endif
} LoraRadio_Packet_t;

#ifdef ENABLE_RADIO_TEST
/* No sample taken yet (the radio task has not reached an idle pass). */
#define LORA_NOISE_FLOOR_INVALID   INT16_MIN
#endif

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

/* Discard everything still queued for TX. Call when the work that queued it
 * is over (campaign end / before sleep): otherwise the radio task keeps
 * carrier-sensing its way through a stale backlog for minutes afterwards,
 * burning power and — because carrier sense runs the chip in CAD rather than
 * RX — staying deaf well into the next event it should have heard. */
void LORARADIO_vFlushTxQueue(void);

void LORARADIO_vEnterDeepSleep(void);
void LORARADIO_vWakeUp(void);

#endif /* DEVICE_LORARADIO_LORARADIO_H_ */
