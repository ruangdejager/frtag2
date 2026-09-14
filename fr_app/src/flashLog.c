/*
 * flashLog.c
 *
 * Persistent flash event logger.
 *
 * Events are pushed into an osMessageQueue from any context (task or ISR)
 * and drained by a dedicated low-priority background task that programs
 * them into internal flash as packed 8-byte double-words.
 *
 * Flash layout: FLASHLOG_START_ADDR … FLASHLOG_END_ADDR (defined in flashLog.h)
 * Record format (8 bytes, little-endian):
 *   [31:0]  UNIX timestamp (seconds)
 *   [36:32] event id (5 bits)
 *   [63:37] value   (27 bits)
 */

#include "flashLog.h"

#ifdef ENABLE_FLASH_LOG

#include "FreeRTOS.h"
#include "task.h"
#include "stm32wlxx_hal.h"
#include <string.h>
#include <stdio.h>

#include "hal_rtc.h"
#include "dbg_log.h"
#include "cmsis_os2.h"
#include "platform_rtc.h"

#include <time.h>
#include <limits.h>

#define FLASH_EMPTY_64      0xFFFFFFFFFFFFFFFFULL
#define FLASHLOG_QUEUE_LENGTH  16
#define FLASHLOG_ITEM_SIZE     sizeof(FlashLogEntry)

static volatile uint32_t wr_addr;

static osMessageQueueId_t logQueue      = NULL;
static osThreadId_t       logTaskHandle = NULL;

/* TX_REPEAT: a SECOND airing of a packet this node originated - the primary
 * airs each TimeSync twice (see MESH_TIMESYNC_AIRINGS), and without a distinct
 * value the log could not show whether the repeat went out. Every value used
 * with LOG_TX_* must have an entry here: TxTypeStr is indexed directly. */
typedef enum { TX_ORIGIN = 1, TX_FORWARDER, TX_REPEAT } DreqType;

static const char *TxTypeStr[] = {
    [TX_ORIGIN]    = "ORIGIN",
    [TX_FORWARDER] = "FORWARDER",
    [TX_REPEAT]    = "REPEAT"
};

/* Packed log record — 8 bytes total */
typedef struct __attribute__((packed))
{
    uint32_t timestamp;
    uint32_t event : 5;   /* bits [4:0]  */
    uint32_t value : 27;  /* bits [31:5] */
} FlashLogEntry;

/* -------------------------------------------------------------------------- */

static uint32_t FLASHLOG_u32FindWriteAddr(void)
{
    uint32_t addr = FLASHLOG_START_ADDR;
    while (addr < FLASHLOG_END_ADDR)
    {
        if (*(uint64_t *)addr == FLASH_EMPTY_64)
            return addr;
        addr += 8;
    }
    return FLASHLOG_END_ADDR;
}

/* -------------------------------------------------------------------------- */

static void FLASHLOG_vTask(void *argument)
{
    (void)argument;
    FlashLogEntry entry;
    uint64_t      packed_data;

    for (;;)
    {
        /* Block until a log event arrives */
        if (osMessageQueueGet(logQueue, &entry, NULL, osWaitForever) == osOK)
        {
            if (wr_addr >= FLASHLOG_END_ADDR)
                continue;

            memcpy(&packed_data, &entry, sizeof(packed_data));

            HAL_StatusTypeDef status;
            HAL_FLASH_Unlock();
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, wr_addr, packed_data);
            if (status != HAL_OK)
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, wr_addr, packed_data);

            if (status == HAL_OK)
                wr_addr += 8;

            HAL_FLASH_Lock();
        }
    }
}

/* -------------------------------------------------------------------------- */

void FLASHLOG_vInit(void)
{
    wr_addr = FLASHLOG_u32FindWriteAddr();

    if (wr_addr >= FLASHLOG_END_ADDR)
        return;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    logQueue = osMessageQueueNew(FLASHLOG_QUEUE_LENGTH, FLASHLOG_ITEM_SIZE, NULL);
    configASSERT(logQueue != NULL);

    static const osThreadAttr_t logTask_attr = {
        .name       = "LogTask",
        .stack_size = configMINIMAL_STACK_SIZE * 8, /* bytes */
        .priority   = osPriorityLow1,
    };
    logTaskHandle = osThreadNew(FLASHLOG_vTask, NULL, &logTask_attr);
    configASSERT(logTaskHandle != NULL);
}

/* -------------------------------------------------------------------------- */

void FLASHLOG_vWrite(uint16_t event, uint32_t value)
{
    if (logQueue == NULL) return;

    FlashLogEntry e;
    e.timestamp = (uint32_t)RTC_u64GetUTC();
    e.event     = event & 0x1F;
    e.value     = value & 0x07FFFFFF;

    osMessageQueuePut(logQueue, &e, 0, 10); /* 10 ms timeout */
}

/* -------------------------------------------------------------------------- */

void FLASHLOG_vWriteFromISR(uint16_t event, int16_t value,
                             BaseType_t *pxHigherPriorityTaskWoken)
{
    if (logQueue == NULL) return;

    /* Signal to the caller that no manual yield is required — CMSIS v2
     * osMessageQueuePut handles context switching internally. */
    if (pxHigherPriorityTaskWoken)
        *pxHigherPriorityTaskWoken = pdFALSE;

    FlashLogEntry e;
    e.timestamp = (uint32_t)RTC_u64GetUTC();
    e.event     = event & 0x1F;
    e.value     = (uint32_t)value & 0x07FFFFFF;

    osMessageQueuePut(logQueue, &e, 0, 0); /* non-blocking from ISR */
}

/* -------------------------------------------------------------------------- */

void FLASHLOG_vEncodeRXLogValue(uint32_t *pBuf,
                                uint16_t id, int16_t rssi, uint8_t res)
{
    uint32_t rssi_abs;
    if (rssi == INT16_MIN) rssi_abs = 127;
    else rssi_abs = (rssi < 0) ? (uint32_t)(-rssi) : (uint32_t)rssi;
    if (rssi_abs > 127) rssi_abs = 127;

    *pBuf  = (rssi_abs & 0x7F);
    *pBuf |= ((uint32_t)id  << 7);
    *pBuf |= ((uint32_t)res << 23);
}

void FLASHLOG_vDecodeRXLogValue(uint32_t value,
                                uint16_t *id, int16_t *rssi, uint8_t *res)
{
    *rssi = (int16_t)(value & 0x7F) * (-1);
    *id   = (value >> 7) & 0xFFFF;
    *res  = (value >> 23) & 0x0F;
}

/* -------------------------------------------------------------------------- */

#ifdef DEBUG_OUTPUT_UART
void FLASHLOG_vDump(void)
{
    uint32_t addr = FLASHLOG_START_ADDR;

    while (addr < FLASHLOG_END_ADDR)
    {
        uint64_t raw = *(uint64_t *)addr;
        if (raw == FLASH_EMPTY_64) break;

        uint32_t timestamp = (uint32_t)(raw & 0xFFFFFFFF);
        uint32_t packed    = (uint32_t)(raw >> 32);
        uint32_t event     = packed & 0x1F;
        uint32_t value     = (packed >> 5) & 0x07FFFFFF;

        uint16_t devId  = 0;
        int16_t  rssi   = 0;
        uint8_t  resVal = 0;
        if (event >= LOG_RX_DREQ && event < LOG_TX_DREQ)
            FLASHLOG_vDecodeRXLogValue(value, &devId, &rssi, &resVal);

        time_t    rawtime = timestamp;
        struct tm ts;
        char      timebuf[20];
        ts = *localtime(&rawtime);
        strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &ts);

        char buf[96];
        switch (event)
        {
            case LOG_DISCOVERY_START:
                snprintf(buf, sizeof(buf), "[%s] Discovery start [%s]\r\n",
                         timebuf, (value == 1) ? "PRIMARY" : "SECONDARY");
                break;
            case LOG_DISCOVERY_INIT:
                snprintf(buf, sizeof(buf), "[%s] Discovery init [%s]\r\n",
                         timebuf, (value == 1) ? "PRIMARY" : "SECONDARY");
                break;
            case LOG_DISCOVERY_RECOVER:
                snprintf(buf, sizeof(buf), "[%s] Discovery recovery [%04lX]\r\n",
                         timebuf, value);
                break;
            case LOG_RX_DREQ:
                snprintf(buf, sizeof(buf), "[%s] Dreq received Wave %d [%04X %ddBm]\r\n",
                         timebuf, resVal, devId, rssi);
                break;
            case LOG_RX_BEACON:
                snprintf(buf, sizeof(buf), "[%s] Beacon received [%04X %ddBm]\r\n",
                         timebuf, devId, rssi);
                break;
            case LOG_RX_ACK:
                snprintf(buf, sizeof(buf), "[%s] Acks received %d [%04X %ddBm]\r\n",
                         timebuf, resVal, devId, rssi);
                break;
            case LOG_RX_TS:
                snprintf(buf, sizeof(buf), "[%s] Timestamp received [%ddBm]\r\n",
                         timebuf, rssi);
                break;
            case LOG_TX_DREQ:
                snprintf(buf, sizeof(buf), "[%s] Dreq sent [%s]\r\n",
                         timebuf, TxTypeStr[value]);
                break;
            case LOG_TX_BEACON:
                snprintf(buf, sizeof(buf), "[%s] Beacon sent [%s]\r\n",
                         timebuf, TxTypeStr[value]);
                break;
            case LOG_TX_ACK:
                snprintf(buf, sizeof(buf), "[%s] Ack sent [%s]\r\n",
                         timebuf, TxTypeStr[value]);
                break;
            case LOG_TX_TS:
                snprintf(buf, sizeof(buf), "[%s] Timestamp sent [%s]\r\n",
                         timebuf, TxTypeStr[value]);
                break;
            case LOG_RESET_CAUSE:
            {
                uint32_t csrUpdated = value << 5;
                snprintf(buf, sizeof(buf), "[%s] Reset Cause: csr=[%08lX]\r\n",
                         timebuf, csrUpdated);
                break;
            }
            case LOG_DISCOVERY_CMPLT:
                snprintf(buf, sizeof(buf), "[%s] Discovery complete [%s]\r\n",
                         timebuf, (value == 1) ? "PRIMARY" : "SECONDARY");
                break;
            case LOG_DEVICE_ENTERING_SLEEP:
                snprintf(buf, sizeof(buf), "[%s] Going to sleep [%s]\r\n",
                         timebuf, (value == 1) ? "PRIMARY" : "SECONDARY");
                break;
            case LOG_FRLOG_ERROR:
                snprintf(buf, sizeof(buf), "[%s] FrLog ERROR [%lu]\r\n",
                         timebuf, value);
                break;
            case LOG_DISCOVERY_COUNT:
                snprintf(buf, sizeof(buf), "[%s] Discovery Count [%lu]\r\n",
                         timebuf, value);
                break;
            default:
                snprintf(buf, sizeof(buf), "[%s] Unknown event %lu val=%lu\r\n",
                         timebuf, event, value);
                break;
        }
        DBG_LOG("%s", buf);
        addr += 8;
    }
}
#endif /* DEBUG_OUTPUT_UART */

#endif /* ENABLE_FLASH_LOG */
