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

/* Thread flag used by UART ISR to wake the RX task */
#define FR_RX_FLAG              (1UL << 0)

/* Thread flags set on the CALLER thread by ATHandlerTask */
#define FR_AT_NOTIFY_SUCCESS    (1UL << 8)
#define FR_AT_NOTIFY_TIMEOUT    (1UL << 9)

/* ---- RX buffer ----
 * The fr9 logger's AT responses are all short fixed tokens ("RDY\r\n",
 * "Logger ready\r\n", "OK\r\n", a 10-digit timestamp, a wake interval) -
 * the longest is ~14 bytes. 48 keeps a >3x margin while two of these line
 * buffers stay small (RAM here is very tight). */
#define FR_RX_BUF_LEN 48
static char     acFrRxBuf[FR_RX_BUF_LEN];
static uint8_t  u8FrRxBufIdx = 0;
static char     acFrLineBuf[FR_RX_BUF_LEN];

/* ---- CMSIS-RTOS v2 objects ---- */
static osThreadId_t  Farmranger_vRxTask_handle;
static osThreadId_t  Farmranger_vATHandlerTask_handle;

static osSemaphoreId_t xLineReadySem;
static osSemaphoreId_t xUartTxDoneSem;
static osMessageQueueId_t xATQueue;

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

    xLineReadySem    = osSemaphoreNew(1, 0, NULL);
    xUartTxDoneSem   = osSemaphoreNew(1, 0, NULL);
    xATQueue         = osMessageQueueNew(4, sizeof(ATReq_t), NULL);

    configASSERT(xLineReadySem  != NULL);
    configASSERT(xUartTxDoneSem != NULL);
    configASSERT(xATQueue       != NULL);

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
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vUartOnWake
 * -------------------------------------------------------------------------- */
void FARMRANGER_vUartOnWake(void)
{
    HAL_UART_vInit();
    FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);
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
    DBG_LOG("Wait for RDY...\r\n");
    if (!FARMRANGER_bATSend(NULL,
                            FARMRANGER_bParseRDY,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            3000))
    {
        DBG_LOG("RDY not received\r\n");
        BSP_LED_On(LED_YELLOW);
        return false;
    }

    bFRDeviceOn = true;
    DBG_LOG("Farmranger Ready.\r\n");
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
    DBG_LOG("Farmranger released.\r\n");
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
/* One CSV row: "DeviceId,Hops,Rssi,BatMv,Wave,Move,Lat,Lon\t". Worst case
 * is ~60 chars (8-hex id + signed micro-degree lat/lon); 80 leaves margin. */
#define FR_CSV_ROW_MAX 80

/* Format neighbor `i` into `row` (size FR_CSV_ROW_MAX). Returns the byte
 * count, or <=0 / >=FR_CSV_ROW_MAX on error. */
static int FARMRANGER_iFormatRow(char *row, const MeshDiscoveredNeighbor_t *n)
{
    return snprintf(row, FR_CSV_ROW_MAX,
                    "%X,%u,%d,%u,%u,%u,%ld,%ld\t",
                    (unsigned int)n->u32DeviceId,
                    n->u8HopCount,
                    n->i16Rssi,
                    n->u16BatMv,
                    n->u8Wave,
                    n->u8MoveState,
                    (long)n->i32LatUDeg,
                    (long)n->i32LonUDeg);
}

bool FARMRANGER_bLogData(MeshDiscoveredNeighbor_t *neighbors, uint16_t count)
{
    /* The AT+LOG protocol needs the total payload length up front, so build
     * it in two passes over a single small row buffer rather than one large
     * static payload buffer: pass 1 sums the byte count, pass 2 streams each
     * row. Keeps RAM use to one ~80-byte row regardless of neighbor count. */
    char   row[FR_CSV_ROW_MAX];
    size_t pos = 0;

    /* Pass 1: compute total payload length. */
    for (uint16_t i = 0; i < count; i++)
    {
        int n = FARMRANGER_iFormatRow(row, &neighbors[i]);
        if (n <= 0 || n >= (int)sizeof(row))
            return false;
        pos += (size_t)n;
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
        EVTLOG(LOG_FRLOG_ERROR, 1);
        DBG_LOG("LogData: No 'Logger ready' received.\r\n");
        BSP_LED_On(LED_YELLOW);
    }

    /* Step 2: stream the CSV payload one row at a time. */
    for (uint16_t i = 0; i < count; i++)
    {
        int n = FARMRANGER_iFormatRow(row, &neighbors[i]);
        if (n <= 0 || n >= (int)sizeof(row))
            return false;

        HAL_UART_vTxPutBuffer(&farmranger.UartHandle, (uint8_t *)row, (uint16_t)n);

        /* Wait until this row's TX is fully drained before reusing `row`. */
        if (osSemaphoreAcquire(xUartTxDoneSem, 3500) != osOK)
        {
            EVTLOG(LOG_FRLOG_ERROR, 2);
            DBG_LOG("UART TX timeout\r\n");
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
        DBG_LOG("LogData: No final OK received.\r\n");
        EVTLOG(LOG_FRLOG_ERROR, 3);
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

