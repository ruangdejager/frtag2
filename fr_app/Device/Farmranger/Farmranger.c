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
 * the longest is ~14 bytes. 48 keeps a >3x margin. */
#define FR_RX_BUF_LEN 48
static char     acFrRxBuf[FR_RX_BUF_LEN];
static uint8_t  u8FrRxBufIdx = 0;

/* Delivered-line queue. Replaces the earlier single-slot
 * (acFrLineBuf + binary xLineReadySem) design, which lost lines whenever
 * two AT responses arrived back-to-back: for AT+LOG=0 the fr9's own state
 * machine sends "Logger ready\r\n" and "OK\r\n" on adjacent ticks (see
 * fr9's FRTAG_vLogCmdHandler), so the two land at this RX task inside a
 * single ring-drain pass. With one slot the RX task's second "\n" would
 * overwrite the still-unread "Logger ready" with "OK" and the second
 * osSemaphoreRelease would silently no-op (binary sem capped at 1). The AT
 * handler then acquired the sem and parsed "OK" against the "Logger ready"
 * parser -> mismatch, Step 1 timeout, whole log retried. A small queue
 * absorbs those bursts atomically instead. */
typedef struct { char data[FR_RX_BUF_LEN]; } FrRxLine_t;
#define FR_LINE_QUEUE_DEPTH  8

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

static osMessageQueueId_t xLineQueue;
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
static bool FARMRANGER_bParseLogVerdict(const char *line, void *ctx);
static bool FARMRANGER_bParseRDY(const char *line, void *ctx);

/* --------------------------------------------------------------------------
 * FARMRANGER_vInit
 * -------------------------------------------------------------------------- */
void FARMRANGER_vInit(void)
{
    bFRDeviceOn = false;

    FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);

    xLineQueue       = osMessageQueueNew(FR_LINE_QUEUE_DEPTH, sizeof(FrRxLine_t), NULL);
    xATQueue         = osMessageQueueNew(4, sizeof(ATReq_t), NULL);

    configASSERT(xLineQueue     != NULL);
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
                /* Publish the completed line into the queue. Non-blocking:
                 * if the queue is full (shouldn't happen with the 8-slot
                 * depth for our protocol) the newest line is dropped rather
                 * than the RX task stalling and letting the UART ring
                 * overflow. */
                FrRxLine_t line;
                uint8_t n = (u8FrRxBufIdx < (uint8_t)sizeof(line.data))
                          ? u8FrRxBufIdx
                          : (uint8_t)(sizeof(line.data) - 1U);
                memcpy(line.data, acFrRxBuf, n);
                line.data[n] = '\0';
                (void)osMessageQueuePut(xLineQueue, &line, 0U, 0U);

                u8FrRxBufIdx = 0;
                memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
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
    memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
    u8FrRxBufIdx = 0;
    taskEXIT_CRITICAL();
    osMessageQueueReset(xLineQueue);        /* drop any queued stale lines */

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

    /* Clear any stale AT notification flags on this thread. Do NOT drain
     * xLineQueue here -- the AT handler resets it after dequeueing this
     * request, and doing it here as well would race with anything already
     * in flight from the caller task's own perspective. */
    osThreadFlagsClear(FR_AT_NOTIFY_SUCCESS | FR_AT_NOTIFY_TIMEOUT);

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

        /* Reset RX state and drop any stale queued lines from before this
         * command was issued. Bytes accumulating in acFrRxBuf that don't
         * yet form a complete line stay put -- the RX task will finish
         * them on the next '\n' -- but partial garbage from a previous
         * command shouldn't hang around either. */
        taskENTER_CRITICAL();
        u8FrRxBufIdx = 0;
        memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
        taskEXIT_CRITICAL();
        osMessageQueueReset(xLineQueue);

        if (req.cmd && strlen(req.cmd) > 0)
        {
            HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                                  (uint8_t *)req.cmd,
                                  strlen(req.cmd));
        }

        uint32_t start     = osKernelGetTickCount();
        bool     notified  = false;
        FrRxLine_t line;

        while ((osKernelGetTickCount() - start) < req.timeout)
        {
            if (osMessageQueueGet(xLineQueue, &line, NULL, 50U) == osOK)
            {
                if (req.parser(line.data, req.context))
                {
                    osThreadFlagsSet(req.caller, FR_AT_NOTIFY_SUCCESS);
                    notified = true;
                    break;
                }
                /* Non-matching line -- discard and keep waiting. Queue
                 * absorbs the next line (e.g. an early "OK" that arrived
                 * back-to-back with the "Logger ready" we were looking
                 * for) so nothing is lost. */
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

/* Upload retry policy. The fr9 signals a failed transfer (bytes lost on the
 * wire, e.g. its RX ring overflowed during a GNSS acquire or a flash-log page
 * erase) with "ERR" 5 s after the last byte it received, so the verdict wait
 * must exceed that; an fr9 running older firmware answers "OK" either way,
 * which degrades this to today's single-attempt behaviour. */
#define FR_LOG_ATTEMPTS        3U
#define FR_LOG_VERDICT_MS      6500U
#define FR_LOG_RETRY_DELAY_MS  250U

/* One AT+LOG upload attempt: handshake, paced row stream, final verdict.
 * `pos` is the pre-computed total payload length. */
static bool FARMRANGER_bLogAttempt(const MeshDiscoveredNeighbor_t *neighbors,
                                   uint16_t count, size_t pos)
{
    char row[FR_CSV_ROW_MAX];

    /* Step 1: send AT+LOG=<len> and wait for "Logger ready". Without it the
     * fr9 is not in its payload state and the stream would go into the void —
     * fail the attempt rather than transmit blind. */
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+LOG=%u\r\n", (unsigned)pos);

    DBG_LOG("LogData: attempt sending AT+LOG=%u\r\n", (unsigned)pos);

    /* For an empty upload (count==0) the fr9 sends "Logger ready\r\n" and
     * "OK\r\n" back-to-back on adjacent state-machine ticks. Splitting that
     * into a separate Step-1 wait (via bATSend, which resets the line queue
     * on entry) only exists to guard the payload stream below -- there's no
     * stream to guard here, and the split is exactly what created a
     * consume-race that lost the OK and forced an unnecessary retry (the
     * fr9 then received a second AT+LOG=0, logging the empty campaign
     * twice). Skip the AT-handler round-trip: send AT+LOG=0 directly and
     * let Step 3 below discard the Logger ready and pick up the OK from
     * the same queue. For count>0 the payload's own streaming time gives
     * fr9 the gap it needs, so the split remains correct. */
    char respBuf[32] = {0};
    if (count == 0)
    {
        /* Reset the line queue so nothing stale from before this attempt
         * lands in Step 3 as a false verdict. */
        osMessageQueueReset(xLineQueue);
        HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                              (uint8_t *)cmd,
                              (uint16_t)strlen(cmd));
    }
    else
    {
        if (!FARMRANGER_bATSend(cmd,
                                FARMRANGER_bParseLoggerReady,
                                respBuf,
                                sizeof(respBuf),
                                respBuf,
                                1000))
        {
            EVTLOG(LOG_FRLOG_ERROR, 1);
            DBG_LOG("LogData: No 'Logger ready' received (Step 1 fail).\r\n");
            return false;
        }
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

    /* Step 3: wait for the fr9's verdict — "OK" (all bytes counted) or "ERR"
     * (its silence timeout hit before the count was reached: bytes were lost
     * and the upload must be retried). No line at all also fails the attempt.
     *
     * Captured INLINE off xLineQueue rather than via FARMRANGER_bATSend, and
     * without resetting the queue first. The fr9 answers AT+LOG=0 by sending
     * "Logger ready\r\n" and "OK\r\n" back-to-back on adjacent state-machine
     * ticks (see fr9's FRTAG_vLogCmdHandler), so for empty uploads the "OK"
     * regularly lands here before this code even starts waiting -- either
     * queued right behind the "Logger ready" that Step 1 consumed, or arriving
     * within microseconds of the empty Step 2 loop finishing. Going through
     * bATSend would call osMessageQueueReset on entry (via the AT handler) and
     * lose that already-queued OK; the whole upload would time out at
     * FR_LOG_VERDICT_MS and retry unnecessarily, logging the same discovery
     * twice on the fr9. */
    char       cVerdict    = '\0';
    uint32_t   u32Start    = osKernelGetTickCount();
    bool       bGotVerdict = false;
    FrRxLine_t line;

    while ((osKernelGetTickCount() - u32Start) < FR_LOG_VERDICT_MS)
    {
        if (osMessageQueueGet(xLineQueue, &line, NULL, 50U) == osOK)
        {
            if (FARMRANGER_bParseLogVerdict(line.data, &cVerdict))
            {
                bGotVerdict = true;
                break;
            }
        }
    }

    if (!bGotVerdict)
    {
        DBG_LOG("LogData: no verdict received (Step 3 timeout).\r\n");
        EVTLOG(LOG_FRLOG_ERROR, 3);
        return false;
    }

    if (cVerdict != 'O')
    {
        DBG_LOG("LogData: fr9 reported ERR (bytes lost).\r\n");
        EVTLOG(LOG_FRLOG_ERROR, 4);
        return false;
    }

    DBG_LOG("LogData: Step 3 OK (verdict='%c')\r\n", cVerdict);
    return true;
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

    /* Pass 2: upload, retrying on a failed transfer. A single field event
     * (fr9 RX overrun during its GNSS window / a flash page erase) drops a
     * chunk of bytes mid-stream; without a retry that discovery is lost for
     * good. Each attempt is self-contained — the fr9 returns to idle before
     * the next AT+LOG goes out (verdict wait > its silence timeout). */
    for (uint8_t u8Attempt = 1U; u8Attempt <= FR_LOG_ATTEMPTS; u8Attempt++)
    {
        if (FARMRANGER_bLogAttempt(neighbors, count, pos))
        {
            if (u8Attempt > 1U)
                DBG_LOG("LogData: upload OK on attempt %u\r\n", u8Attempt);
            return true;
        }

        if (u8Attempt < FR_LOG_ATTEMPTS)
        {
            DBG_LOG("LogData: attempt %u failed, retrying...\r\n", u8Attempt);
            osDelay(FR_LOG_RETRY_DELAY_MS);
        }
    }

    DBG_LOG("LogData: all %u attempts failed\r\n", FR_LOG_ATTEMPTS);
    return false;
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

/* Transfer verdict: matches "OK" (success) or "ERR" (fr9 timed out — bytes
 * were lost on the wire). Writes 'O'/'E' into ctx so the caller can tell the
 * two apart; matching ERR here (instead of only OK) lets a failed upload
 * fail fast rather than burn the whole verdict window. Note "ERR" must be
 * checked first: strstr("ERR...", "OK") can't false-match, but keeping the
 * explicit order makes the precedence obvious. */
static bool FARMRANGER_bParseLogVerdict(const char *line, void *ctx)
{
    char *pcVerdict = (char *)ctx;

    if (line == NULL)
        return false;

    if (strstr(line, "ERR") != NULL)
    {
        *pcVerdict = 'E';
        return true;
    }
    if (strstr(line, "OK") != NULL)
    {
        *pcVerdict = 'O';
        return true;
    }
    return false;
}

static bool FARMRANGER_bParseRDY(const char *line, void *ctx)
{
    (void)ctx;
    return (line != NULL && strstr(line, "RDY") != NULL);
}

