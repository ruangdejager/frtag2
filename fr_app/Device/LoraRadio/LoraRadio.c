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
#define LORA_TX_QUEUE_SIZE      (24)
#define LORA_RX_QUEUE_SIZE      (8)
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 10)

#define CAD_BASE_BACKOFF_MS     100
#define CAD_MAX_BACKOFF_MS      2000
#define CAD_MAX_EXPONENT        4

/* Mask covering all 31 usable CMSIS thread-flag bits */
#define ALL_FLAGS               (0x7FFFFFFFU)

/* ---- CMSIS-RTOS v2 objects ---- */
static osMessageQueueId_t xLoRaTxQueue;
static osMessageQueueId_t xLoRaRxQueue;
static osThreadId_t       LORARADIO_vRadioTask_handle;

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

    DBG_LOG("\r\nLoraradio: Device ID %lX\r\n", LORARADIO_u32GetUniqueId());

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
    if (packet->length > LORA_MAX_PACKET_SIZE)
        return false;

    if (osMessageQueuePut(xLoRaTxQueue, packet, 0, 0) != osOK)
    {
        DBG_LOG("Loraradio: TX PKT queue full\r\n");
        return false;
    }

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
            SUBGRF_ClearIrqStatus(IRQ_RX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- ERRORS ---------- */
        if (events & RADIO_EVT_CRC_ERROR)
        {
            SUBGRF_ClearIrqStatus(IRQ_CRC_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }
        if (events & RADIO_EVT_HEADER_ERROR)
        {
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
            DBG_LOG("LoraRadio: TX done IRQ received\r\n");
            SUBGRF_ClearIrqStatus(IRQ_TX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX REQUEST ---------- */
        if ((events & RADIO_EVT_TX_PENDING) ||
            (osMessageQueueGetCount(xLoRaTxQueue) > 0))
        {
            while (osMessageQueueGet(xLoRaTxQueue, &pkt, NULL, 0) == osOK)
            {
                uint8_t crc = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length);
                pkt.buffer[pkt.length++] = crc;

                if (!LORARADIO_bCarrierSenseAndWait(5000))
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
        DBG_LOG("Loraradio: CAD timeout, assuming busy\r\n");
        return false;
    }

    if (r & RADIO_EVT_CAD_BUSY)
    {
        DBG_LOG("Loraradio: CAD busy\r\n");
        return false;
    }
    if (r & RADIO_EVT_CAD_CLEAR)
    {
        DBG_LOG("Loraradio: CAD clear\r\n");
        return true;
    }

    /* Another radio event arrived during CAD — stash it, report busy */
    LORARADIO_vStashPendingEvents(r);
    return false;
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

        DBG_LOG("Loraradio: CAD back-off %lu ms (fail=%lu)\r\n", backoffMs, failCount);

        /* Back-off sleep — also uses ALL_FLAGS so any event wakes us cleanly */
        uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, backoffMs);
        if (!(r & osFlagsError))
        {
            if (r & RADIO_EVT_CAD_CLEAR) return true;
            if (r & RADIO_EVT_CAD_BUSY)  return false;
            LORARADIO_vStashPendingEvents(r);
            return false;
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
