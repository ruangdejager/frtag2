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

/* ---- Raw block capture (OTA firmware pull) ----
 * While armed (u16CaptureLen > 0 and not yet filled), incoming bytes bypass
 * the line path straight into the caller's buffer. The RX task only copies —
 * no parsing, no flash — so capture can't stall reception. Armed/filled/
 * cancelled from the AppTask; consumed on the RX task (volatile indices). */
static uint8_t *pu8CaptureBuf;
static volatile uint16_t u16CaptureLen;   /* capture target (0 = disarmed) */
static volatile uint16_t u16CaptureIdx;   /* bytes captured so far          */

/* ---- CMSIS-RTOS v2 objects ---- */
static osThreadId_t  Farmranger_vRxTask_handle;
static osThreadId_t  Farmranger_vATHandlerTask_handle;

static osSemaphoreId_t xLineReadySem;
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
    xATQueue         = osMessageQueueNew(4, sizeof(ATReq_t), NULL);

    configASSERT(xLineReadySem  != NULL);
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
            /* Raw block capture (OTA pull): bytes bypass the line path. */
            if (u16CaptureIdx < u16CaptureLen)
            {
                pu8CaptureBuf[u16CaptureIdx] = byte;
                u16CaptureIdx++;
                continue;
            }

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
//        BSP_LED_On(LED_YELLOW);
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

/* Inter-row pacing for the CSV upload (ms). The fr9 receives into a 128-byte
 * UART RX ring drained by a co-operative loop that echoes every byte to USB;
 * if the whole payload is streamed back-to-back at line rate that ring
 * overflows and the fr9 never counts the full length (→ "LOG TIMEOUT"). We
 * therefore wait for each row to leave the wire and pause briefly, bounding
 * the in-flight data to a single row and giving the fr9 time to drain. This is
 * the throttle the per-byte osDelay in HAL_UART_vTxPutBuffer used to provide
 * before that path was sped up for the (USART2) flash-log dump. */
#define FR_LOG_ROW_GAP_MS 10U

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
//        BSP_LED_On(LED_YELLOW);
    }

    /* Step 2: stream the CSV payload one row at a time, paced so the fr9 keeps
     * up. HAL_UART_vTxPutByte copies each byte into the TX ring, so `row` is
     * reusable as soon as vTxPutBuffer returns; after queueing a row we wait
     * for it to fully leave the wire (HAL_UART_bTxIdle) and then pause
     * FR_LOG_ROW_GAP_MS. That bounds the in-flight data to one row so the fr9's
     * RX ring can't overflow (see FR_LOG_ROW_GAP_MS). */
    for (uint16_t i = 0; i < count; i++)
    {
        int n = FARMRANGER_iFormatRow(row, &neighbors[i]);
        if (n <= 0 || n >= (int)sizeof(row))
            return false;

        HAL_UART_vTxPutBuffer(&farmranger.UartHandle, (uint8_t *)row, (uint16_t)n);

        /* Wait for this row to drain off the wire before sending the next. */
        uint32_t u32TxStart = osKernelGetTickCount();
        while (!HAL_UART_bTxIdle(&farmranger.UartHandle))
        {
            if ((osKernelGetTickCount() - u32TxStart) >= 3500)
            {
                EVTLOG(LOG_FRLOG_ERROR, 2);
                DBG_LOG("UART TX timeout\r\n");
                return false;
            }
            osDelay(1);
        }

        osDelay(FR_LOG_ROW_GAP_MS);
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
 * Firmware-file pull (OTA acquire) — see Farmranger.h for the protocol
 * -------------------------------------------------------------------------- */

/* Response timeouts. A 1 KB block at 115200 with the fr9's in-block pacing is
 * ~150 ms on the wire plus the fr9's modem-filesystem QFREAD turnaround. */
#define FR_FW_QUERY_TIMEOUT_MS  1500U
#define FR_FW_BLOCK_TIMEOUT_MS  2000U

typedef struct {
    FarmrangerFw_e eResult;
    uint32_t       u32Version;
    uint32_t       u32FileBytes;
} FwQueryCtx_t;

typedef struct {
    uint32_t u32Offset;
    uint8_t  u8Xor;
} FwTrailerCtx_t;

/* "FW,NONE" | "FW,WAIT" | "FW,<verMMmmpp>,<fileBytes>" */
static bool FARMRANGER_bParseFwInfo(const char *line, void *ctx)
{
    FwQueryCtx_t *pt = (FwQueryCtx_t *)ctx;

    const char *p = strstr(line, "FW,");
    if (p == NULL)
        return false;
    p += 3;

    if (strncmp(p, "NONE", 4) == 0) { pt->eResult = FARMRANGER_FW_NONE; return true; }
    if (strncmp(p, "WAIT", 4) == 0) { pt->eResult = FARMRANGER_FW_WAIT; return true; }

    char *pcEnd;
    uint32_t u32Ver = strtoul(p, &pcEnd, 10);
    if (pcEnd == p || *pcEnd != ',')
        return false;
    uint32_t u32Bytes = strtoul(pcEnd + 1, &pcEnd, 10);
    if (u32Bytes == 0UL)
        return false;

    pt->eResult      = FARMRANGER_FW_AVAILABLE;
    pt->u32Version   = u32Ver;
    pt->u32FileBytes = u32Bytes;
    return true;
}

/* "FB,<offset>,<xor8hex>" — sent by the logger after the raw block bytes. */
static bool FARMRANGER_bParseFwTrailer(const char *line, void *ctx)
{
    FwTrailerCtx_t *pt = (FwTrailerCtx_t *)ctx;

    const char *p = strstr(line, "FB,");
    if (p == NULL)
        return false;
    p += 3;

    char *pcEnd;
    pt->u32Offset = strtoul(p, &pcEnd, 10);
    if (pcEnd == p || *pcEnd != ',')
        return false;
    pt->u8Xor = (uint8_t)strtoul(pcEnd + 1, NULL, 16);
    return true;
}

FarmrangerFw_e FARMRANGER_eFwQuery(uint32_t *pu32Version, uint32_t *pu32FileBytes)
{
    FwQueryCtx_t tCtx = { .eResult = FARMRANGER_FW_NONE };
    char respBuf[32] = {0};

    if (!FARMRANGER_bATSend("AT+FWREQ\r\n",
                            FARMRANGER_bParseFwInfo,
                            respBuf,
                            sizeof(respBuf),
                            &tCtx,
                            FR_FW_QUERY_TIMEOUT_MS))
    {
        return FARMRANGER_FW_NONE;   /* no answer — treat as nothing to fetch */
    }

    *pu32Version   = tCtx.u32Version;
    *pu32FileBytes = tCtx.u32FileBytes;
    return tCtx.eResult;
}

bool FARMRANGER_bFwGetBlock(uint32_t u32Offset, uint16_t u16Len, uint8_t *pu8Buf)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+FWGET=%lu,%u\r\n",
             (unsigned long)u32Offset, (unsigned)u16Len);

    /* Arm the raw capture BEFORE the command goes out so the first response
     * byte can never race past the line path. Length is written last — it is
     * the enable. */
    pu8CaptureBuf = pu8Buf;
    u16CaptureIdx = 0U;
    u16CaptureLen = u16Len;

    FwTrailerCtx_t tTrailer = {0};
    char respBuf[32] = {0};
    bool bOk = FARMRANGER_bATSend(cmd,
                                  FARMRANGER_bParseFwTrailer,
                                  respBuf,
                                  sizeof(respBuf),
                                  &tTrailer,
                                  FR_FW_BLOCK_TIMEOUT_MS);

    /* Disarm capture whatever happened (lost bytes leave it partly filled —
     * the trailer line then got eaten by the capture and bOk is false). */
    uint16_t u16Got = u16CaptureIdx;
    u16CaptureLen = 0U;
    u16CaptureIdx = 0U;
    pu8CaptureBuf = NULL;

    if (!bOk || u16Got != u16Len || tTrailer.u32Offset != u32Offset)
        return false;

    /* Verify the block against the logger's XOR-8. */
    uint8_t u8Xor = 0U;
    for (uint16_t i = 0U; i < u16Len; i++)
        u8Xor ^= pu8Buf[i];

    return (u8Xor == tTrailer.u8Xor);
}

bool FARMRANGER_bFwReportDone(bool bOk)
{
    char respBuf[32] = {0};
    return FARMRANGER_bATSend(bOk ? "AT+FWDONE=OK\r\n" : "AT+FWDONE=ERR\r\n",
                              FARMRANGER_bParseOK,
                              respBuf,
                              sizeof(respBuf),
                              respBuf,
                              FR_FW_QUERY_TIMEOUT_MS);
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

