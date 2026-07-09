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

void LORARADIO_vEnterDeepSleep(void);
void LORARADIO_vWakeUp(void);

#endif /* DEVICE_LORARADIO_LORARADIO_H_ */
