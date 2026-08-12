/*
 * LoraRadio.c
 *
 * LoRa radio device layer — CMSIS-RTOS v2 task, queues and
 * carrier-sense back-off.
 *
 * IMPORTANT — osThreadFlagsWait mask must be 0x7FFFFFFFU
 * --------------------------------------------------------
 * osThreadFlagsWait() wraps xTaskNotifyWait() but only clears the bits
 * that are IN the supplied mask.  If any other bits are pending in the
 * notification word when the call is made, xTaskNotifyWait returns
 * immediately, the non-masked bits remain, and the next call does the
 * same — producing an infinite spin loop that burns the entire timeout
 * without ever seeing a CAD result.
 *
 * Using 0x7FFFFFFFU as the mask guarantees all pending bits are
 * consumed on every wait; non-CAD events that arrive during carrier-
 * sense are stashed and replayed in the main loop.
 */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "LoraRadio.h"
#include "LoraRadio_Driver.h"
#include "dbg_log.h"
#include "radio_driver.h"

#include <string.h>
#include <stdlib.h>

/* ---- Private defines ---- */
/* TX queue: 8 (was 24). Each slot is a full ~264 B LoraRadio_Packet_t from the
 * 48 KB heap, and the mesh layer already paces its TX through its own 24-deep
 * jitter queue upstream — this queue only buffers what the radio task is about
 * to drain. 8 still absorbs a multi-line FrKernel LoRa response burst;
 * overflow is handled gracefully (logged drop). Frees ~4.2 KB of heap. */
#define LORA_TX_QUEUE_SIZE      (8)
#define LORA_RX_QUEUE_SIZE      (8)
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 10)

#define CAD_BASE_BACKOFF_MS     100
#define CAD_MAX_BACKOFF_MS      2000
#define CAD_MAX_EXPONENT        4

/* Per-packet carrier-sense budget in the TX drain. Was 5000 ms inline; named
 * here because it multiplies with the drain batch size to bound how long the
 * radio can be away from RX. */
#define LORA_TX_CARRIER_WAIT_MS 5000U

/* Max packets sent per pass through the radio task's event loop.
 *
 * This used to drain the ENTIRE queue in one `while`, never returning to the
 * event loop until empty. With an 8-deep queue and a 5 s carrier-sense budget
 * each, a congested node could sit up to ~40 s inside that loop — and carrier
 * sense runs the chip in CAD, not RX, so the device is deaf for the whole
 * time. A heavy-forwarding unit in the field missed the primary's TimeSync in
 * 39 of 47 campaigns that way, which meant FOTA acceptance was never armed and
 * it silently refused every firmware update. Draining a couple of packets per
 * pass keeps the radio cycling back through RX. */
#define LORA_TX_DRAIN_PER_PASS  2U

/* Packets older than this are dropped instead of transmitted. A mesh forward
 * that has been stuck behind a congested queue for this long is stale — the
 * campaign that cared about it has moved on — and sending it only adds to the
 * congestion that delayed it. */
#define LORA_TX_MAX_AGE_MS      10000U

/* Mask covering all 31 usable CMSIS thread-flag bits */
#define ALL_FLAGS               (0x7FFFFFFFU)

/* ---- CMSIS-RTOS v2 objects ---- */
static osMessageQueueId_t xLoRaTxQueue;
static osMessageQueueId_t xLoRaRxQueue;
static osThreadId_t       LORARADIO_vRadioTask_handle;

/* CAD-timeout tally. A CAD that never reports a result within the 300 ms
 * window is normal-ish under congestion but used to emit a DBG_LOG line every
 * single time — hundreds per campaign, drowning the flash log. Counted here
 * and reported once per campaign via the mesh stats line. */
static volatile uint16_t u16CadTimeoutCount = 0;

/* Packets discarded unsent for exceeding LORA_TX_MAX_AGE_MS, plus those
 * dropped by LORARADIO_vFlushTxQueue(). Reported alongside the CAD tally. */
static volatile uint16_t u16TxStaleDropCount = 0;

/* Pending radio events captured during carrier-sense / back-off */
static volatile uint32_t gRadioPendingEvents = 0;

static uint8_t u8DevEUI[8];

/* -------------------------------------------------------------------------- */

static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len);
static void    LORARADIO_vNotifyFromISR(uint32_t evt);

static inline void LORARADIO_vStashPendingEvents(uint32_t evt)
{
    taskENTER_CRITICAL();
    gRadioPendingEvents |= evt;
    taskEXIT_CRITICAL();
}

static inline uint32_t LORARADIO_u32ConsumePendingEvents(void)
{
    uint32_t v;
    taskENTER_CRITICAL();
    v = gRadioPendingEvents;
    gRadioPendingEvents = 0;
    taskEXIT_CRITICAL();
    return v;
}

/* --------------------------------------------------------------------------
 * LORARADIO_vInit
 * -------------------------------------------------------------------------- */
void LORARADIO_vInit(void)
{
    xLoRaTxQueue = osMessageQueueNew(LORA_TX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t), NULL);
    xLoRaRxQueue = osMessageQueueNew(LORA_RX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t), NULL);
    configASSERT(xLoRaTxQueue != NULL && xLoRaRxQueue != NULL);  /* && not ||: both must succeed */

    /*
     * Initialise hardware and enter deep sleep BEFORE creating the task.
     * The task calls vWakeUp() as its first action; creating it first risks
     * the scheduler running it before we reach vEnterDeepSleep() here,
     * leaving the radio awake while the task has already passed vWakeUp().
     */
    LORARADIO_DRIVER_vInit(u8DevEUI);
    LORARADIO_vEnterDeepSleep();

    DBG("\r\nLoraradio: Device ID %lX\r\n", LORARADIO_u32GetUniqueId());

    static const osThreadAttr_t radioTask_attr = {
        .name       = "LoRaRadioTask",
        .stack_size = LORA_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    LORARADIO_vRadioTask_handle = osThreadNew(LORARADIO_vRadioTask, NULL, &radioTask_attr);
    configASSERT(LORARADIO_vRadioTask_handle != NULL);
}

/* --------------------------------------------------------------------------
 * LORARADIO_bRxPacket — blocks until a verified packet is available
 * -------------------------------------------------------------------------- */
bool LORARADIO_bRxPacket(LoraRadio_Packet_t *packet)
{
    return (osMessageQueueGet(xLoRaRxQueue, packet, NULL, osWaitForever) == osOK);
}

/* --------------------------------------------------------------------------
 * LORARADIO_bTxPacket — enqueue a packet for transmission
 * -------------------------------------------------------------------------- */
bool LORARADIO_bTxPacket(LoraRadio_Packet_t *packet)
{
    /* Reserve room for the appended CRC byte: the length field is uint8_t, so
     * type+payload+CRC must fit in 255. A 255-byte payload would wrap
     * pkt.length to 0 on the CRC append in the radio task (corrupt frame). */
    if (packet->length > (LORA_MAX_PACKET_SIZE - 2))
        return false;

    packet->u32QueuedTick = osKernelGetTickCount();   /* for the stale-drop check */

    if (osMessageQueuePut(xLoRaTxQueue, packet, 0, 0) != osOK)
    {
        DBG_LOG("Loraradio: TX PKT queue full\r\n");
        return false;
    }

    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_TX_PENDING);
    return true;
}

/* --------------------------------------------------------------------------
 * LORARADIO_bTxPacketWait — enqueue, blocking while the queue is full
 * -------------------------------------------------------------------------- */
bool LORARADIO_bTxPacketWait(LoraRadio_Packet_t *packet, uint32_t timeoutMs)
{
    if (packet->length > (LORA_MAX_PACKET_SIZE - 2))
        return false;

    /* Blocking put: FreeRTOS wakes this caller the moment the radio task's
     * osMessageQueueGet() pulls the head item, i.e. as soon as it has
     * actually finished with the previous packet -- no polling, no guessed
     * delay. */
    packet->u32QueuedTick = osKernelGetTickCount();   /* for the stale-drop check */

    if (osMessageQueuePut(xLoRaTxQueue, packet, 0, timeoutMs) != osOK)
        return false;

    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_TX_PENDING);
    return true;
}

/* --------------------------------------------------------------------------
 * LORARADIO_vRadioTask — main radio state machine
 * -------------------------------------------------------------------------- */
void LORARADIO_vRadioTask(void *arg)
{
    (void)arg;
    LoraRadio_Packet_t pkt;

    /*
     * LORARADIO_vInit() puts the radio into deep sleep (WarmStart=0) so the
     * chip draws minimal current during boot.  A plain vEnterRxMode() after
     * WarmStart=0 sleep would leave modulation/frequency/TX-power registers
     * in an undefined state.  vWakeUp() performs a full re-initialisation
     * (LORARADIO_DRIVER_vInit + vEnterRxMode) before the task loop begins.
     */
    LORARADIO_vWakeUp();

    for (;;)
    {
        /* Wait up to 50 ms for any radio event — ALL_FLAGS clears every bit */
        uint32_t events = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, 50);
        if (events & osFlagsError) events = 0;

        /* Merge any stashed events from a carrier-sense interruption */
        events |= LORARADIO_u32ConsumePendingEvents();

        /* ---------- RX DONE ---------- */
        if (events & RADIO_EVT_RX_DONE)
        {
            memset(&pkt, 0, sizeof(pkt));
            LORARADIO_DRIVER_bReceivePayload(&pkt);

            /* Shortest valid frame is type byte + CRC byte. A 0/1-byte frame
             * (corrupt header that passed the radio CRC) would otherwise
             * underflow pkt.length - 1 below: buffer[-1] read and a CRC pass
             * over (uint16_t)-1 = 65535 bytes, far past the 256-byte buffer. */
            if (pkt.length < 2)
            {
                DBG_LOG("Loraradio: runt frame dropped (len=%u)\r\n", pkt.length);
            }
            else
            {
                uint8_t crc_rx = pkt.buffer[pkt.length - 1];
                uint8_t crc    = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length - 1);
                if (crc == crc_rx)
                {
                    pkt.length--;
                    osMessageQueuePut(xLoRaRxQueue, &pkt, 0, 0);
                }
                else
                {
                    DBG_LOG("CRC mismatch\r\n");
                }
            }
            SUBGRF_ClearIrqStatus(IRQ_RX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- ERRORS ----------
         * Log every hardware-detected packet fault. These paths used to be
         * completely silent, which made it impossible to distinguish "no
         * packet was on the air" from "packet arrived corrupted and got
         * dropped by the SX126x's own LoRa CRC check". During OTA
         * distribution debugging especially, these lines are what tells
         * you whether an intermittent 1-chunk-missing pattern is a real
         * RF corruption event vs. some upstream logic bug. */
        if (events & RADIO_EVT_CRC_ERROR)
        {
            DBG_LOG("LoraRadio: RX CRC error (packet dropped by hardware)\r\n");
            SUBGRF_ClearIrqStatus(IRQ_CRC_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }
        if (events & RADIO_EVT_HEADER_ERROR)
        {
            DBG_LOG("LoraRadio: RX header error (packet dropped by hardware)\r\n");
            SUBGRF_ClearIrqStatus(IRQ_HEADER_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }
        if (events & RADIO_EVT_TIMEOUT)
        {
            SUBGRF_ClearIrqStatus(IRQ_RX_TX_TIMEOUT);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX DONE ---------- */
        if (events & RADIO_EVT_TX_DONE)
        {
            DBG("LoraRadio: TX done IRQ received\r\n");
            SUBGRF_ClearIrqStatus(IRQ_TX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX REQUEST ----------
         * Bounded drain: at most LORA_TX_DRAIN_PER_PASS packets before falling
         * back to the event loop. Carrier sense puts the chip in CAD, not RX,
         * so time spent here is time spent deaf; emptying the whole queue in
         * one go could hold that for tens of seconds and make the node miss
         * the very TimeSync/OtaPrep it needs (see LORA_TX_DRAIN_PER_PASS).
         * Anything still queued is picked up next pass — the queue-count test
         * above re-enters this block without needing a fresh TX_PENDING. */
        if ((events & RADIO_EVT_TX_PENDING) ||
            (osMessageQueueGetCount(xLoRaTxQueue) > 0))
        {
            uint8_t u8Sent = 0U;

            while (u8Sent < LORA_TX_DRAIN_PER_PASS &&
                   osMessageQueueGet(xLoRaTxQueue, &pkt, NULL, 0) == osOK)
            {
                /* Drop rather than send a packet that has been stuck behind a
                 * congested queue: it is a mesh forward whose moment has
                 * passed, and transmitting it now only feeds the congestion
                 * that delayed it. Doesn't count against the send budget. */
                if ((osKernelGetTickCount() - pkt.u32QueuedTick) > LORA_TX_MAX_AGE_MS)
                {
                    if (u16TxStaleDropCount < UINT16_MAX) u16TxStaleDropCount++;
                    continue;
                }

                u8Sent++;

                uint8_t crc = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length);
                pkt.buffer[pkt.length++] = crc;

                if (!LORARADIO_bCarrierSenseAndWait(LORA_TX_CARRIER_WAIT_MS))
                {
                    uint32_t stashed = LORARADIO_u32ConsumePendingEvents();
                    if (stashed)
                    {
                        events |= stashed;
                        DBG_LOG("Loraradio: TX REQ interrupted by events 0x%08lX\r\n", stashed);
                        LORARADIO_DRIVER_vEnterRxMode(0);
                        continue;
                    }
                    DBG_LOG("Loraradio: TX REQ abort — carrier busy\r\n");
                    LORARADIO_DRIVER_vEnterRxMode(0);
                    continue;
                }

                LORARADIO_DRIVER_bTransmitPayload(pkt.buffer, pkt.length);
                LORARADIO_DRIVER_vEnterRxMode(0x00);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */

uint32_t LORARADIO_u32GetUniqueId(void)
{
    return (((uint32_t)u8DevEUI[1] << 24) | ((uint32_t)u8DevEUI[3] << 16)) >> 16;
}

void LORARADIO_vEventRxDone(void)      { LORARADIO_vNotifyFromISR(RADIO_EVT_RX_DONE);      }
void LORARADIO_vEventTxDone(void)      { LORARADIO_vNotifyFromISR(RADIO_EVT_TX_DONE);      }
void LORARADIO_vEventCrcError(void)    { LORARADIO_vNotifyFromISR(RADIO_EVT_CRC_ERROR);    }
void LORARADIO_vEventHeaderError(void) { LORARADIO_vNotifyFromISR(RADIO_EVT_HEADER_ERROR); }
void LORARADIO_vEventTimeout(void)     { LORARADIO_vNotifyFromISR(RADIO_EVT_TIMEOUT);      }
void LORARADIO_vEventCADDetected(void) { LORARADIO_vNotifyFromISR(RADIO_EVT_CAD_BUSY);     }
void LORARADIO_vEventCADClear(void)    { LORARADIO_vNotifyFromISR(RADIO_EVT_CAD_CLEAR);    }

/* Byte-wise XOR fold — a weak checksum, but kept as such deliberately for
 * over-the-air compatibility with older firmware in the field. A real CRC
 * upgrade is a hard protocol break (mixed old/new mesh drops every packet
 * between mismatched pairs, including the OTA packets that would deliver
 * the upgrade) so we rely on the SX126x's own hardware LoRa CRC (enabled
 * via LORA_CRC_ON in the packet params, and its failure reported through
 * IRQ_CRC_ERROR / RADIO_EVT_CRC_ERROR) as the primary line of defence
 * against RF corruption; this app-layer check is a secondary sanity check
 * only. Do NOT change this to a real CRC without a coordinated flash of
 * every device on the mesh via SWD. */
static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

uint32_t LORARADIO_u32GetRandomNumber(uint32_t max_value)
{
    return LORARADIO_DRIVER_u32GetRandomNumber(max_value);
}

/* --------------------------------------------------------------------------
 * LORARADIO_bCarrierSense — single CAD measurement
 *
 * Uses ALL_FLAGS (0x7FFFFFFF) so that any event that arrives during the
 * 300 ms window — including RX_DONE, CRC_ERROR etc. — is consumed and
 * the wait does not spin.  Non-CAD events are stashed for the main loop.
 * -------------------------------------------------------------------------- */
bool LORARADIO_bCarrierSense(void)
{
    LORARADIO_DRIVER_vEnterCAD();

    uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, 300);

    if (r & osFlagsError)
    {
        /* Counted, not logged per occurrence: under congestion this fires
         * hundreds of times per campaign and used to bury everything else in
         * the flash log. The tally is reported once per campaign instead —
         * see LORARADIO_u16GetAndClearCadTimeouts(). */
        if (u16CadTimeoutCount < UINT16_MAX) u16CadTimeoutCount++;
        return false;
    }

    /* TX_PENDING is self-noise here: we're already inside TX processing for
     * a packet already dequeued off xLoRaTxQueue (that's why CAD is running
     * at all). LORARADIO_bTxPacket() sets this flag unconditionally, and it
     * can still be pending the first time we wait on it if the flag was set
     * after the radio task's top-of-loop flags-wait already snapshotted (a
     * fast responder, e.g. FrKernel answering a just-received request, wins
     * that race essentially every time). Treating our own trigger as a
     * foreign interrupting event caused every first-attempt TX to abort and
     * drop the packet with no retry. */
    r &= ~RADIO_EVT_TX_PENDING;

    if (r & RADIO_EVT_CAD_BUSY)
    {
        DBG("Loraradio: CAD busy\r\n");
        return false;
    }
    if (r & RADIO_EVT_CAD_CLEAR)
    {
        DBG("Loraradio: CAD clear\r\n");
        return true;
    }

    if (r)
    {
        /* Another, genuinely foreign radio event arrived during CAD — stash
         * it, report busy. */
        LORARADIO_vStashPendingEvents(r);
    }
    return false;
}

uint16_t LORARADIO_u16GetAndClearCadTimeouts(void)
{
    uint16_t u16Count = u16CadTimeoutCount;
    u16CadTimeoutCount = 0U;
    return u16Count;
}

uint16_t LORARADIO_u16GetAndClearTxStaleDrops(void)
{
    uint16_t u16Count = u16TxStaleDropCount;
    u16TxStaleDropCount = 0U;
    return u16Count;
}

void LORARADIO_vFlushTxQueue(void)
{
    LoraRadio_Packet_t tDiscard;
    uint16_t           u16Dropped = 0U;

    if (xLoRaTxQueue == NULL) return;

    while (osMessageQueueGet(xLoRaTxQueue, &tDiscard, NULL, 0) == osOK)
        u16Dropped++;

    if (u16Dropped > 0U)
    {
        if ((uint32_t)u16TxStaleDropCount + u16Dropped > UINT16_MAX)
            u16TxStaleDropCount = UINT16_MAX;
        else
            u16TxStaleDropCount = (uint16_t)(u16TxStaleDropCount + u16Dropped);
    }
}

/* --------------------------------------------------------------------------
 * LORARADIO_bCarrierSenseAndWait — adaptive back-off loop
 * -------------------------------------------------------------------------- */
bool LORARADIO_bCarrierSenseAndWait(uint32_t maxWaitMs)
{
    uint32_t startMs  = osKernelGetTickCount();
    uint32_t failCount = 0;

    while ((osKernelGetTickCount() - startMs) < maxWaitMs)
    {
        if (LORARADIO_bCarrierSense()) return true;

        if (gRadioPendingEvents) return false;

        uint32_t exponent = (failCount > CAD_MAX_EXPONENT) ? CAD_MAX_EXPONENT : failCount;
        uint32_t window   = CAD_BASE_BACKOFF_MS << exponent;
        if (window > CAD_MAX_BACKOFF_MS) window = CAD_MAX_BACKOFF_MS;

        uint32_t backoffMs = CAD_BASE_BACKOFF_MS
            + LORARADIO_u32GetRandomNumber(window - CAD_BASE_BACKOFF_MS + 1);

        DBG("Loraradio: CAD back-off %lu ms (fail=%lu)\r\n", backoffMs, failCount);

        /* Back-off sleep — also uses ALL_FLAGS so any event wakes us cleanly */
        uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, backoffMs);
        if (!(r & osFlagsError))
        {
            /* Same self-noise as LORARADIO_bCarrierSense() — see comment
             * there. Strip it before deciding whether something genuinely
             * foreign woke us. */
            r &= ~RADIO_EVT_TX_PENDING;

            if (r & RADIO_EVT_CAD_CLEAR) return true;
            if (r & RADIO_EVT_CAD_BUSY)  return false;
            if (r)
            {
                LORARADIO_vStashPendingEvents(r);
                return false;
            }
            /* Only our own TX_PENDING fired — spurious wake, retry CAD. */
        }

        failCount++;
    }

    DBG_LOG("Loraradio: carrier-sense timed out after %lu ms\r\n", maxWaitMs);
    return false;
}

void LORARADIO_vEnterDeepSleep(void) { LORARADIO_DRIVER_vEnterDeepSleep(); }
void LORARADIO_vWakeUp(void)         { LORARADIO_DRIVER_vWakeUp(); }

/* --------------------------------------------------------------------------
 * LORARADIO_vNotifyFromISR — set thread flags from ISR context
 * osThreadFlagsSet handles IS_IRQ() internally and calls portYIELD_FROM_ISR.
 * -------------------------------------------------------------------------- */
static void LORARADIO_vNotifyFromISR(uint32_t evt)
{
    if (LORARADIO_vRadioTask_handle != NULL)
        osThreadFlagsSet(LORARADIO_vRadioTask_handle, evt);
}
