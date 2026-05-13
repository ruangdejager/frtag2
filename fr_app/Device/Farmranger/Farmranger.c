/*
 * Farmranger.c
 *
 * Farmranger device layer — UART AT-command interface to the companion
 * logger board.
 *
 * All task / queue / semaphore management uses CMSIS-RTOS v2.
 * taskENTER_CRITICAL / taskEXIT_CRITICAL are kept (valid under the
 * CMSIS overlay) for short critical sections that protect shared buffers.
 *
 * AT-command notification protocol
 * ---------------------------------
 * ATSend() waits on two dedicated thread-flag bits:
 *   FR_AT_NOTIFY_SUCCESS  — ATHandlerTask found a matching response
 *   FR_AT_NOTIFY_TIMEOUT  — ATHandlerTask timed out
 *
 * The bits are set on the CALLER thread (osThreadGetId() at enqueue time)
 * so that ATSend() is safely re-entrant from different calling tasks.
 */

#include "Farmranger.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE, taskENTER/EXIT_CRITICAL */
#include "task.h"

#include "dbg_log.h"
#include "flashLog.h"
#include "str.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* ---- Private defines ---- */
#define FR_RX_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE)
#define FR_AT_HANDLER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define FR_DBG_TX_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)

#define FR_DBG_TX_STR_LEN     128
#define FR_DBG_TX_QUEUE_DEPTH 8

/* Thread flag used by UART ISR to wake the RX task */
#define FR_RX_FLAG              (1UL << 0)

/* Thread flags set on the CALLER thread by ATHandlerTask */
#define FR_AT_NOTIFY_SUCCESS    (1UL << 8)
#define FR_AT_NOTIFY_TIMEOUT    (1UL << 9)

/* ---- RX buffer ---- */
#define FR_RX_BUF_LEN 128
static char     acFrRxBuf[FR_RX_BUF_LEN];
static uint8_t  u8FrRxBufIdx = 0;
static char     acFrLineBuf[FR_RX_BUF_LEN];

/* ---- CMSIS-RTOS v2 objects ---- */
static osThreadId_t  Farmranger_vRxTask_handle;
static osThreadId_t  Farmranger_vATHandlerTask_handle;
static osThreadId_t  Farmranger_vDbgTxTask_handle;

static osSemaphoreId_t xLineReadySem;
static osSemaphoreId_t xUartTxDoneSem;
static osMessageQueueId_t xATQueue;

#ifdef LISTENER_MODE
typedef struct {
    char     str[FR_DBG_TX_STR_LEN];
    uint16_t len;
} DbgTxItem_t;

static osMessageQueueId_t xDbgTxQueue;
#endif

/* ---- Device state ---- */
bool bFRDeviceOn;

/* ---- AT parser type ---- */
typedef bool (*ATParserFn)(const char *line, void *context);

typedef struct {
    const char    *cmd;
    ATParserFn     parser;
    void          *context;
    char          *out;
    size_t         outLen;
    osThreadId_t   caller;
    uint32_t       timeout;     /* ms — 1 tick == 1 ms */
} ATReq_t;

/* ---- UART handle ---- */
struct _farmranger_s {
    hal_uart_t UartHandle;
    uint8_t    byte;
} farmranger;

/* ---- Forward declarations ---- */
static bool FARMRANGER_bATSend(const char *cmd,
                               ATParserFn parser,
                               char *out,
                               size_t outLen,
                               void *context,
                               uint32_t timeout);
static void FARMRANGER_vATHandlerTask(void *args);
#ifdef LISTENER_MODE
static void FARMRANGER_vDbgTxTask(void *args);
#endif
static bool FARMRANGER_bParseTimestamp(const char *line, void *ctx);
static bool FARMRANGER_bParseInterval(const char *line, void *ctx);
static bool FARMRANGER_bParseLoggerReady(const char *line, void *ctx);
static bool FARMRANGER_bParseOK(const char *line, void *ctx);
static bool FARMRANGER_bParseRDY(const char *line, void *ctx);

/* --------------------------------------------------------------------------
 * FARMRANGER_vInit
 * -------------------------------------------------------------------------- */
void FARMRANGER_vInit(void)
{
    bFRDeviceOn = false;

    FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);

#ifdef LISTENER_MODE
    FR_DRIVER_vEnableUart(&farmranger.UartHandle);
#endif

    xLineReadySem    = osSemaphoreNew(1, 0, NULL);
    xUartTxDoneSem   = osSemaphoreNew(1, 0, NULL);
    xATQueue         = osMessageQueueNew(4, sizeof(ATReq_t), NULL);

    configASSERT(xLineReadySem  != NULL);
    configASSERT(xUartTxDoneSem != NULL);
    configASSERT(xATQueue       != NULL);

#ifdef LISTENER_MODE
    xDbgTxQueue = osMessageQueueNew(FR_DBG_TX_QUEUE_DEPTH, sizeof(DbgTxItem_t), NULL);
    configASSERT(xDbgTxQueue != NULL);
#endif

    static const osThreadAttr_t rx_attr = {
        .name       = "FRRxTask",
        .stack_size = FR_RX_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityRealtime,
    };
    static const osThreadAttr_t at_attr = {
        .name       = "FRAtHandlerTask",
        .stack_size = FR_AT_HANDLER_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityHigh,
    };

    Farmranger_vRxTask_handle        = osThreadNew(FARMRANGER_vRxTask,        NULL, &rx_attr);
    Farmranger_vATHandlerTask_handle = osThreadNew(FARMRANGER_vATHandlerTask, NULL, &at_attr);

    configASSERT(Farmranger_vRxTask_handle        != NULL);
    configASSERT(Farmranger_vATHandlerTask_handle  != NULL);

#ifdef LISTENER_MODE
    static const osThreadAttr_t dbtx_attr = {
        .name       = "FRDbgTxTask",
        .stack_size = FR_DBG_TX_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityLow,
    };
    Farmranger_vDbgTxTask_handle = osThreadNew(FARMRANGER_vDbgTxTask, NULL, &dbtx_attr);
    configASSERT(Farmranger_vDbgTxTask_handle != NULL);
#endif
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vUartOnWake
 * -------------------------------------------------------------------------- */
void FARMRANGER_vUartOnWake(void)
{
    HAL_UART_vInit();
    FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);
#ifdef LISTENER_MODE
    FR_DRIVER_vEnableUart(&farmranger.UartHandle);
#endif
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vRxTask — drains UART ring buffer, signals line-ready
 * -------------------------------------------------------------------------- */
void FARMRANGER_vRxTask(void *parameters)
{
    uint8_t byte;

    for (;;)
    {
        /* Block until UART ISR notifies us that a byte arrived */
        osThreadFlagsWait(FR_RX_FLAG, osFlagsWaitAny, osWaitForever);

        /* Drain all available bytes from the ring buffer */
        while (UART_bReadByte(&farmranger.UartHandle, &byte))
        {
            if (u8FrRxBufIdx < FR_RX_BUF_LEN - 1)
                acFrRxBuf[u8FrRxBufIdx++] = byte;

            if (byte == '\n')
            {
                /* Transfer ownership of the completed line */
                memcpy(acFrLineBuf, acFrRxBuf, u8FrRxBufIdx);
                acFrLineBuf[u8FrRxBufIdx] = '\0';
                u8FrRxBufIdx = 0;
                memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
                osSemaphoreRelease(xLineReadySem);
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vNotifyOnRX — called from UART RX ISR via uart_callbacks.c
 * -------------------------------------------------------------------------- */
void FARMRANGER_vNotifyOnRX(void)
{
    if (Farmranger_vRxTask_handle != NULL)
        osThreadFlagsSet(Farmranger_vRxTask_handle, FR_RX_FLAG);
}

/* --------------------------------------------------------------------------
 * FARMRANGER_bDeviceOn — power up device and wait for RDY
 * -------------------------------------------------------------------------- */
bool FARMRANGER_bDeviceOn(void)
{
    osThreadResume(Farmranger_vRxTask_handle);

    if (bFRDeviceOn)
        return true;

    /* Clear stale state */
    taskENTER_CRITICAL();
    memset(acFrRxBuf,   0, FR_RX_BUF_LEN);
    memset(acFrLineBuf, 0, FR_RX_BUF_LEN);
    u8FrRxBufIdx = 0;
    taskEXIT_CRITICAL();
    osSemaphoreAcquire(xLineReadySem, 0);   /* drain any pending token */

    FR_DRIVER_vEnableUart(&farmranger.UartHandle);
    FR_DRIVER_vIntEnable();

    char respBuf[32] = {0};
    DBG("Wait for RDY...\r\n");
    if (!FARMRANGER_bATSend(NULL,
                            FARMRANGER_bParseRDY,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            5000))
    {
        DBG("RDY not received\r\n");
        return false;
    }

    bFRDeviceOn = true;
    DBG("Farmranger Ready.\r\n");
    return true;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vDeviceOff
 * -------------------------------------------------------------------------- */
void FARMRANGER_vDeviceOff(void)
{
    osThreadSuspend(Farmranger_vRxTask_handle);
    FR_DRIVER_vDisableUart(&farmranger.UartHandle);
    FR_DRIVER_vIntDisable();
    bFRDeviceOn = false;
    DBG("Farmranger released.\r\n");
}

/* --------------------------------------------------------------------------
 * FARMRANGER_bATSend — enqueue an AT command and block for the response
 *
 * Returns true on success, false on timeout or queue-full.
 * The ATHandlerTask notifies this thread via thread flags:
 *   FR_AT_NOTIFY_SUCCESS  → response matched
 *   FR_AT_NOTIFY_TIMEOUT  → timed out
 * -------------------------------------------------------------------------- */
static bool FARMRANGER_bATSend(const char *cmd,
                               ATParserFn parser,
                               char *out,
                               size_t outLen,
                               void *context,
                               uint32_t timeout)
{
    ATReq_t req = {
        .cmd     = cmd,
        .parser  = parser,
        .context = context,
        .out     = out,
        .outLen  = outLen,
        .caller  = osThreadGetId(),
        .timeout = timeout,
    };

    /* Clear any stale AT notification flags on this thread */
    osThreadFlagsClear(FR_AT_NOTIFY_SUCCESS | FR_AT_NOTIFY_TIMEOUT);
    osSemaphoreAcquire(xLineReadySem, 0);   /* drain */

    configASSERT(xATQueue != NULL);
    if (osMessageQueuePut(xATQueue, &req, 0, 100) != osOK)
        return false;

    uint32_t r = osThreadFlagsWait(FR_AT_NOTIFY_SUCCESS | FR_AT_NOTIFY_TIMEOUT,
                                   osFlagsWaitAny,
                                   timeout);

    if (r & osFlagsError)
        return false;

    return (r & FR_AT_NOTIFY_SUCCESS) ? true : false;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vATHandlerTask — dequeues ATReq_t and polls for a response
 * -------------------------------------------------------------------------- */
static void FARMRANGER_vATHandlerTask(void *args)
{
    ATReq_t req;

    for (;;)
    {
        if (osMessageQueueGet(xATQueue, &req, NULL, osWaitForever) != osOK)
            continue;

        /* Reset RX state before issuing the command */
        taskENTER_CRITICAL();
        u8FrRxBufIdx = 0;
        memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
        taskEXIT_CRITICAL();
        osSemaphoreAcquire(xLineReadySem, 0);

        if (req.cmd && strlen(req.cmd) > 0)
        {
            HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                                  (uint8_t *)req.cmd,
                                  strlen(req.cmd));
        }

        uint32_t start     = osKernelGetTickCount();
        bool     notified  = false;

        while ((osKernelGetTickCount() - start) < req.timeout)
        {
            if (osSemaphoreAcquire(xLineReadySem, 50) == osOK)
            {
                if (req.parser(acFrLineBuf, req.context))
                {
                    osThreadFlagsSet(req.caller, FR_AT_NOTIFY_SUCCESS);
                    notified = true;
                    break;
                }
                memset(acFrLineBuf, 0, FR_RX_BUF_LEN);
            }
        }

        if (!notified)
            osThreadFlagsSet(req.caller, FR_AT_NOTIFY_TIMEOUT);
    }
}

/* --------------------------------------------------------------------------
 * FARMRANGER_u64RequestTimestamp
 * -------------------------------------------------------------------------- */
uint64_t FARMRANGER_u64RequestTimestamp(void)
{
    char     tsStr[32]  = {0};
    uint64_t tsValue    = 0;

    if (FARMRANGER_bATSend("AT+TSREQ\r\n",
                           FARMRANGER_bParseTimestamp,
                           tsStr,
                           sizeof(tsStr),
                           tsStr,
                           2000))
    {
        tsValue = strtoull(tsStr, NULL, 10);
    }

    return tsValue;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_u8RequestInterval
 * -------------------------------------------------------------------------- */
uint8_t FARMRANGER_u8RequestInterval(void)
{
    char    intStr[32] = {0};
    uint8_t intValue   = 0;

    if (FARMRANGER_bATSend("AT+INTREQ\r\n",
                           FARMRANGER_bParseInterval,
                           intStr,
                           sizeof(intStr),
                           intStr,
                           10000))
    {
        intValue = (uint8_t)strtoull(intStr, NULL, 10);
    }

    return intValue;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_bLogData — sends neighbor table to logger
 * -------------------------------------------------------------------------- */
bool FARMRANGER_bLogData(MeshDiscoveredNeighbor_t *neighbors, uint16_t count)
{
    static char logBuffer[2048];
    size_t pos = 0;

    for (uint16_t i = 0; i < count; i++)
    {
        int n = snprintf(&logBuffer[pos],
                         sizeof(logBuffer) - pos,
                         "%X,%u,%d,%u,%u\t",
                         neighbors[i].u32DeviceId,
                         neighbors[i].u8HopCount,
                         neighbors[i].i16Rssi,
                         neighbors[i].u16BatMv,
                         neighbors[i].u8Wave);

        if (n <= 0 || n >= (int)(sizeof(logBuffer) - pos))
            return false;

        pos += n;
    }

    /* Step 1: send AT+LOG=<len> and wait for "Logger ready" */
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+LOG=%u\r\n", (unsigned)pos);

    char respBuf[32] = {0};
    if (!FARMRANGER_bATSend(cmd,
                            FARMRANGER_bParseLoggerReady,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            1000))
    {
        LOG(LOG_FRLOG_ERROR, 1);
        DBG("LogData: No 'Logger ready' received.\r\n");
        BSP_LED_On(LED_YELLOW);
    }

    if (pos > 0)
    {
        /* Step 2: stream the CSV payload */
        HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                              (uint8_t *)logBuffer,
                              pos);

        /* Wait until TX is fully drained */
        if (osSemaphoreAcquire(xUartTxDoneSem, 3500) != osOK)
        {
            LOG(LOG_FRLOG_ERROR, 2);
            DBG("UART TX timeout\r\n");
            return false;
        }
    }

    /* Step 3: wait for final OK */
    memset(respBuf, 0, sizeof(respBuf));
    if (!FARMRANGER_bATSend(NULL,
                            FARMRANGER_bParseOK,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            3500))
    {
        DBG("LogData: No final OK received.\r\n");
        LOG(LOG_FRLOG_ERROR, 3);
        return false;
    }

    return true;
}

/* --------------------------------------------------------------------------
 * HAL_UART_vTxCompleteISR — ISR callback; releases TX-done semaphore
 * -------------------------------------------------------------------------- */
void HAL_UART_vTxCompleteISR(hal_uart_t *drv)
{
    if (drv == &farmranger.UartHandle)
        osSemaphoreRelease(xUartTxDoneSem);   /* ISR-safe in CMSIS-RTOS v2 */
}

/* --------------------------------------------------------------------------
 * AT response parsers
 * -------------------------------------------------------------------------- */

static bool FARMRANGER_bParseTimestamp(const char *line, void *ctx)
{
    char *out = (char *)ctx;
    size_t len = strlen(line);

    /* Expect: 10 digits + "\r\n" */
    if (len == 12 && line[10] == '\r' && line[11] == '\n')
    {
        for (int i = 0; i < 10; i++)
            if (line[i] < '0' || line[i] > '9') return false;

        memcpy(out, line, 10);
        out[10] = '\0';
        return true;
    }
    return false;
}

static bool FARMRANGER_bParseInterval(const char *line, void *ctx)
{
    char  *out = (char *)ctx;
    size_t len = strlen(line);

    /* Expect: 1-3 digits + "\r\n" */
    if (len >= 3 && len < 6 && line[len - 2] == '\r' && line[len - 1] == '\n')
    {
        for (size_t i = 0; i < len - 2; i++)
            if (line[i] < '0' || line[i] > '9') return false;

        memcpy(out, line, len - 2);
        out[len - 2] = '\0';
        return true;
    }
    return false;
}

static bool FARMRANGER_bParseLoggerReady(const char *line, void *ctx)
{
    (void)ctx;
    return (line != NULL && strstr(line, "Logger ready") != NULL);
}

static bool FARMRANGER_bParseOK(const char *line, void *ctx)
{
    (void)ctx;
    return (line != NULL && strstr(line, "OK") != NULL);
}

static bool FARMRANGER_bParseRDY(const char *line, void *ctx)
{
    (void)ctx;
    return (line != NULL && strstr(line, "RDY") != NULL);
}

/* --------------------------------------------------------------------------
 * Listener-mode API
 * -------------------------------------------------------------------------- */
#ifdef LISTENER_MODE

void FARMRANGER_vPutString(const uint8_t *data, uint16_t len)
{
    if (xDbgTxQueue == NULL || len == 0)
        return;

    DbgTxItem_t item;
    if (len > FR_DBG_TX_STR_LEN)
        len = FR_DBG_TX_STR_LEN;

    memcpy(item.str, data, len);
    item.len = len;

    osMessageQueuePut(xDbgTxQueue, &item, 0, 0);
}

static void FARMRANGER_vDbgTxTask(void *args)
{
    DbgTxItem_t item;

    for (;;)
    {
        if (osMessageQueueGet(xDbgTxQueue, &item, NULL, osWaitForever) == osOK)
        {
            HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                                  (uint8_t *)item.str,
                                  item.len);
        }
    }
}

#endif /* LISTENER_MODE */
