/*
 * RadioTestMode.c
 *
 * Runtime radio link/range test. See RadioTestMode.h for what the mode does
 * and how it is entered and left.
 *
 * Design notes worth knowing before editing:
 *
 *  - Almost nothing is static. This firmware links with well under 100 bytes
 *    of RAM to spare (see the "reclaim RAM so FRKERNEL_INTERFACE_UART links"
 *    fix), so every counter, buffer and handle this mode needs lives in one
 *    heap-allocated context allocated on entry and freed on exit. What remains
 *    static is three flags and two pointers — the minimum needed for the gates
 *    in MeshNetwork and DeviceDiscovery to be a cheap read on the hot path.
 *
 *  - The listener does NOT drain the LoRa RX queue. That queue belongs to
 *    MESHNETWORK_vParserTask in a production build, which is the key
 *    difference from the compile-time ENABLE_RADIO_TEST build in RadioTest.c
 *    (there, the mesh does not exist and RadioTest owns the queue outright).
 *    Beacons reach us through RADIOTESTMODE_vOnBeacon, called from the parser.
 *
 *  - The parser hands us beacons on its own task. We only parse and enqueue
 *    there; the logging and the Farmranger round-trip happen on our task,
 *    because an fr9 handshake can block for seconds and stalling the parser
 *    would back up the LoRa RX queue and drop packets.
 */

#include "RadioTestMode.h"

#include "LoraRadio.h"
#include "MeshNetwork.h"        /* MeshPktType_Reserved, DeviceRole_e */
#include "DeviceDiscovery.h"    /* DEVICE_DISCOVERY_eGetDeviceRole */
#include "Farmranger.h"
#include "hal_system.h"         /* SYSTEM_vSleepLockAcquire / Release */
#include "dbg_log.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */
#define RTM_STACK_SIZE      (configMINIMAL_STACK_SIZE * 6)
#define RTM_BEACON_MS       5000U    /* 0.2 Hz, per the test spec */

/* Beacon frame: [0] = type, [1..] = ASCII "RT <id:hex> <seq:dec>".
 *
 * Same wire format as the compile-time test in RadioTest.c, deliberately: a
 * compile-time-built beacon and a runtime-built one are interchangeable on the
 * bench. MeshPktType_Reserved keeps it inert to production units, whose parser
 * switch drops reserved types in its default arm. */
#define RTM_TAG             "RT "
#define RTM_TAG_LEN         3U
#define RTM_LINE_MAX        48U

/* fr9 upload batching. Beacons are buffered and flushed together rather than
 * one AT+RTLOG per beacon: each upload costs a full Farmranger session (GPIO
 * attention, ~1 s RDY handshake, command, stream, verdict, teardown), which
 * does not comfortably fit a 5 s cadence and would leave the link busy more
 * often than idle. Flushing on a count or an age bound also keeps every
 * session far shorter than the fr9's 600 s session cap. */
#define RTM_FR9_BATCH_MAX   8U
#define RTM_FR9_FLUSH_MS    60000U

/* How long RADIOTESTMODE_vExit waits for the task to wind itself down. Sized
 * for the worst case: a final fr9 flush is up to FR_LOG_ATTEMPTS AT+RTLOG
 * attempts, each with a handshake, a paced stream and a verdict wait. */
#define RTM_STOP_WAIT_MS    15000U

/* Thread flags */
#define RTM_FLAG_BEACON     (1UL << 0)   /* beacon timer fired           */
#define RTM_FLAG_RX         (1UL << 1)   /* a beacon landed in the queue */
#define RTM_FLAG_STOP       (1UL << 2)   /* exit requested               */

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

/* One received beacon, as passed from the parser task to ours. */
typedef struct {
    uint32_t u32Id;
    uint32_t u32Seq;
    int16_t  i16Rssi;
    int16_t  i16NoiseFloor;
    int8_t   i8Snr;
} RtmBeacon_t;

/* Everything the mode needs while it is running, in one heap block. */
typedef struct {
    osMessageQueueId_t xRxQueue;      /* listener only */
    osTimerId_t        xBeaconTimer;  /* beacon only   */

    uint32_t u32TxSeq;                /* beacon: next sequence number      */

    uint32_t u32RxCount;              /* listener bookkeeping              */
    uint32_t u32Missed;
    uint32_t u32LastId;
    uint32_t u32LastSeq;
    bool     bHaveLast;

    uint16_t u16BatchCount;           /* fr9 upload batch                  */
    uint32_t u32BatchStart;
    FarmrangerRtBeacon_t tBatch[RTM_FR9_BATCH_MAX];
} RtmCtx_t;

/* The gates in MeshNetwork's parser and TX paths read bActive on every packet,
 * so it stays a plain static rather than something behind a pointer chase.
 * volatile because those reads happen on other tasks. */
static volatile bool  bActive      = false;
static volatile bool  bIsListener  = false;

/* Set before the task starts, cleared by the task as its very last act.
 * RADIOTESTMODE_vExit waits on this before freeing anything the task uses. */
static volatile bool  bTaskRunning = false;

static osThreadId_t   xRtmTask     = NULL;
static RtmCtx_t      *pCtx         = NULL;

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void RTM_vTask(void *arg);
static void RTM_vBeaconTimerCallback(void *arg);
static void RTM_vSendBeacon(void);
static void RTM_vHandleBeacon(const RtmBeacon_t *pB);
static void RTM_vFlushToFr9(void);
static void RTM_vTeardown(void);

/* --------------------------------------------------------------------------
 * Public state
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bActive(void)     { return bActive; }
bool RADIOTESTMODE_bIsListener(void) { return bIsListener; }

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_bEnter
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bEnter(void)
{
    if (bActive || pCtx != NULL)
        return false;

    bIsListener = (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY);

    pCtx = pvPortMalloc(sizeof(RtmCtx_t));
    if (pCtx == NULL)
    {
        DBG_LOG("RadioTest: enter failed - out of heap\r\n");
        return false;
    }
    memset(pCtx, 0, sizeof(RtmCtx_t));
    pCtx->u32BatchStart = osKernelGetTickCount();

    if (bIsListener)
    {
        pCtx->xRxQueue = osMessageQueueNew(4U, sizeof(RtmBeacon_t), NULL);
        if (pCtx->xRxQueue == NULL)
        {
            DBG_LOG("RadioTest: enter failed - no RX queue\r\n");
            RTM_vTeardown();
            return false;
        }
    }
    else
    {
        pCtx->xBeaconTimer = osTimerNew(RTM_vBeaconTimerCallback,
                                        osTimerPeriodic, NULL, NULL);
        if (pCtx->xBeaconTimer == NULL)
        {
            DBG_LOG("RadioTest: enter failed - no beacon timer\r\n");
            RTM_vTeardown();
            return false;
        }
    }

    static const osThreadAttr_t attr = {
        .name       = "RadioTestMode",
        .stack_size = RTM_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    /* Set before the thread is created: the task clears it on the way out, and
     * a task that gets scheduled before osThreadNew returns must not observe a
     * stale false here. */
    bTaskRunning = true;

    xRtmTask = osThreadNew(RTM_vTask, NULL, &attr);
    if (xRtmTask == NULL)
    {
        DBG_LOG("RadioTest: enter failed - no task\r\n");
        bTaskRunning = false;
        RTM_vTeardown();
        return false;
    }

    /* Hold the system out of STOP2 for the duration. STOP2 parks the debug
     * UART pins to analog and suspends the core, so the radio task could only
     * service RX once per 1 Hz RTC heartbeat - fine for a sleeping node,
     * useless for a beacon on a 5 s cadence or for a listener. Released in
     * RADIOTESTMODE_vExit, and nowhere else, so the pairing stays obvious. */
    SYSTEM_vSleepLockAcquire();

    /* Make sure the radio is actually awake: the command may have arrived in a
     * kernel window that would otherwise have ended in a deep sleep. */
    LORARADIO_vWakeUp();

    /* Noise floor is only meaningful to the listener, and sampling costs the
     * radio task an extra SPI read per idle pass - leave it off elsewhere. */
    if (bIsListener)
        LORARADIO_vSetNoiseFloorSampling(true);

    /* Publish the state last: every gate this feature adds keys off bActive,
     * so nothing starts dropping mesh traffic until the mode is fully built
     * and can actually do something with what it keeps. */
    bActive = true;

    /* Stand the mesh down. The parser and TX gates (which now see bActive)
     * stop anything new; these calls clear what was already in flight so the
     * node does not spend the test chewing through a stale backlog. */
    MESHNETWORK_vResetNodeRole();
    MESHNETWORK_vStopPrimaryAck();
    MESHNETWORK_vFlushTxQueue();
    LORARADIO_vFlushTxQueue();

    if (!bIsListener)
        osTimerStart(pCtx->xBeaconTimer, RTM_BEACON_MS);

    DBG_LOG("RadioTest: ENTERED as %s\r\n",
            bIsListener ? "PRIMARY (listen)" : "SECONDARY (beacon)");
    return true;
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vExit
 * -------------------------------------------------------------------------- */
void RADIOTESTMODE_vExit(const char *pcReason)
{
    uint32_t u32Rx     = (pCtx != NULL) ? pCtx->u32RxCount : 0U;
    uint32_t u32Missed = (pCtx != NULL) ? pCtx->u32Missed  : 0U;

    if (!bActive)
        return;

    /* Clear the gates first so the mesh is live again the moment we stop, and
     * so nothing new is handed to a queue we are about to delete. */
    bActive = false;

    if (bIsListener)
        LORARADIO_vSetNoiseFloorSampling(false);

    /* Ask the task to wind itself down, then wait for it to say it has. We do
     * NOT terminate it out from under itself: it may be several seconds deep
     * in a Farmranger transaction, and killing it there would leave the fr9 AT
     * handler holding a thread handle it will later try to signal. The task
     * does its own final fr9 flush and clears bTaskRunning as its last act, so
     * by the time we free anything it is genuinely done with it.
     *
     * The bound is generous because that final flush is up to three AT+RTLOG
     * attempts. On a secondary there is no flush and this returns almost at
     * once, so the shake path (Movement task) never stalls. */
    if (xRtmTask != NULL)
    {
        uint32_t u32Start = osKernelGetTickCount();

        osThreadFlagsSet(xRtmTask, RTM_FLAG_STOP);

        while (bTaskRunning &&
               (osKernelGetTickCount() - u32Start) < RTM_STOP_WAIT_MS)
        {
            osDelay(20);
        }

        if (bTaskRunning)
        {
            /* Wedged somewhere we did not anticipate. Leaking the context is
             * strictly better than freeing memory a live task still writes to,
             * so say so loudly and leave it parked. */
            DBG_LOG("RadioTest: task did not stop in %lu ms - left parked\r\n",
                    (unsigned long)RTM_STOP_WAIT_MS);
            SYSTEM_vSleepLockRelease();
            return;
        }

        u32Rx     = (pCtx != NULL) ? pCtx->u32RxCount : u32Rx;
        u32Missed = (pCtx != NULL) ? pCtx->u32Missed  : u32Missed;
    }

    RTM_vTeardown();

    SYSTEM_vSleepLockRelease();

    DBG_LOG("RadioTest: EXITED (%s) - rx=%lu missed=%lu\r\n",
            (pcReason != NULL) ? pcReason : "?",
            (unsigned long)u32Rx, (unsigned long)u32Missed);
}

/* Free everything the mode allocated. The task has already exited by the time
 * this runs from RADIOTESTMODE_vExit; from RADIOTESTMODE_bEnter's failure path
 * it either never started or is terminated here. Safe with any subset built,
 * which is what makes it usable as that failure path. */
static void RTM_vTeardown(void)
{
    if (xRtmTask != NULL)
    {
        if (bTaskRunning)
        {
            osThreadTerminate(xRtmTask);
            bTaskRunning = false;
        }
        xRtmTask = NULL;
    }

    if (pCtx != NULL)
    {
        if (pCtx->xBeaconTimer != NULL)
        {
            osTimerStop(pCtx->xBeaconTimer);
            osTimerDelete(pCtx->xBeaconTimer);
        }
        if (pCtx->xRxQueue != NULL)
            osMessageQueueDelete(pCtx->xRxQueue);

        vPortFree(pCtx);
        pCtx = NULL;
    }
}

/* --------------------------------------------------------------------------
 * Beacon timer - kept tiny: it runs on the FreeRTOS timer-service task, whose
 * stack cannot carry packet building or a DBG_LOG timestamp format.
 * -------------------------------------------------------------------------- */
static void RTM_vBeaconTimerCallback(void *arg)
{
    (void)arg;
    if (bActive && xRtmTask != NULL)
        osThreadFlagsSet(xRtmTask, RTM_FLAG_BEACON);
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vOnBeacon - parser-task context, keep it cheap
 * -------------------------------------------------------------------------- */
void RADIOTESTMODE_vOnBeacon(const uint8_t *pBuf, uint8_t u8Len,
                             int16_t i16Rssi, int8_t i8Snr, int16_t i16NoiseFloor)
{
    char        line[RTM_LINE_MAX];
    char       *pEnd = NULL;
    RtmBeacon_t tB;
    uint8_t     len = u8Len;

    if (!bActive || !bIsListener || pCtx == NULL || pCtx->xRxQueue == NULL ||
        pBuf == NULL)
        return;

    if (len >= (uint8_t)RTM_LINE_MAX)
        len = (uint8_t)(RTM_LINE_MAX - 1U);

    memcpy(line, pBuf, len);
    line[len] = '\0';

    if (len <= (uint8_t)RTM_TAG_LEN ||
        strncmp(line, RTM_TAG, RTM_TAG_LEN) != 0)
        return;

    tB.u32Id = strtoul(&line[RTM_TAG_LEN], &pEnd, 16);
    if (pEnd == &line[RTM_TAG_LEN])
        return;

    tB.u32Seq        = strtoul(pEnd, NULL, 10);
    tB.i16Rssi       = i16Rssi;
    tB.i8Snr         = i8Snr;
    tB.i16NoiseFloor = i16NoiseFloor;

    /* Non-blocking: if our task is mid-fr9-handshake and the queue is full,
     * drop this beacon rather than stall the parser. The gap shows up in the
     * missed count on the next one we do log, which is the honest outcome. */
    (void)osMessageQueuePut(pCtx->xRxQueue, &tB, 0U, 0U);
    osThreadFlagsSet(xRtmTask, RTM_FLAG_RX);
}

/* --------------------------------------------------------------------------
 * RTM_vTask
 * -------------------------------------------------------------------------- */
static void RTM_vTask(void *arg)
{
    (void)arg;

    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(RTM_FLAG_BEACON | RTM_FLAG_RX | RTM_FLAG_STOP,
                                           osFlagsWaitAny, 1000U);
        if (flags & osFlagsError)
            flags = 0U;

        if (!bActive || (flags & RTM_FLAG_STOP) || pCtx == NULL)
            break;

        if (flags & RTM_FLAG_BEACON)
            RTM_vSendBeacon();

        if (bIsListener)
        {
            RtmBeacon_t tB;
            while (pCtx->xRxQueue != NULL &&
                   osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
            {
                RTM_vHandleBeacon(&tB);
            }

            /* Age-based flush, so a slow trickle of beacons still reaches the
             * fr9 without waiting for a full batch. */
            if (pCtx->u16BatchCount > 0U &&
                (osKernelGetTickCount() - pCtx->u32BatchStart) >= RTM_FR9_FLUSH_MS)
            {
                RTM_vFlushToFr9();
            }
        }
    }

    /* Winding down. Drain what is still queued and push the tail of the run to
     * the fr9 before going: the last beacons of a range walk - the ones from
     * the far end - are the interesting ones, and dropping them would lose
     * exactly the data the test was run for.
     *
     * Done here rather than in RADIOTESTMODE_vExit so it runs on this task's
     * stack and cannot race the loop above. */
    if (bIsListener && pCtx != NULL)
    {
        RtmBeacon_t tB;
        while (pCtx->xRxQueue != NULL &&
               osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
        {
            RTM_vHandleBeacon(&tB);
        }

        if (pCtx->u16BatchCount > 0U)
            RTM_vFlushToFr9();
    }

    /* Last act: tell RADIOTESTMODE_vExit it is safe to free the context.
     * Returning from a CMSIS-RTOS2 thread function exits the thread. */
    bTaskRunning = false;
}

/* --------------------------------------------------------------------------
 * Secondary: one beacon
 * -------------------------------------------------------------------------- */
static void RTM_vSendBeacon(void)
{
    LoraRadio_Packet_t pkt;
    int                n;

    if (pCtx == NULL)
        return;

    memset(&pkt, 0, sizeof(pkt));
    pkt.buffer[0] = (uint8_t)MeshPktType_Reserved;

    n = snprintf((char *)&pkt.buffer[1],
                 (size_t)(LORA_MAX_PACKET_SIZE - 2),
                 RTM_TAG "%04lX %lu",
                 (unsigned long)LORARADIO_u32GetUniqueId(),
                 (unsigned long)pCtx->u32TxSeq);

    if (n > 0)
    {
        pkt.length = (uint8_t)(n + 1);

        if (LORARADIO_bTxPacket(&pkt))
            DBG_LOG("RadioTest: TX seq=%lu\r\n", (unsigned long)pCtx->u32TxSeq);
        else
            DBG_LOG("RadioTest: TX drop seq=%lu (queue full)\r\n",
                    (unsigned long)pCtx->u32TxSeq);
    }

    /* Bumped even on a drop: to the listener a dropped beacon is identical to
     * one that never made it across, and the sequence gap should say so. */
    pCtx->u32TxSeq++;
}

/* --------------------------------------------------------------------------
 * Primary: account for and log one beacon
 * -------------------------------------------------------------------------- */
static void RTM_vHandleBeacon(const RtmBeacon_t *pB)
{
    bool bHaveNf;

    if (pCtx == NULL)
        return;

    bHaveNf = (pB->i16NoiseFloor != LORA_NOISE_FLOOR_INVALID);

    if (pCtx->bHaveLast && (pB->u32Id == pCtx->u32LastId))
    {
        if (pB->u32Seq > pCtx->u32LastSeq)
            pCtx->u32Missed += (pB->u32Seq - pCtx->u32LastSeq - 1U);
    }
    else if (pCtx->bHaveLast)
    {
        /* A different secondary took over the test - the old tally is not
         * comparable, so restart it rather than report nonsense. */
        pCtx->u32Missed = 0U;
    }

    pCtx->u32LastId  = pB->u32Id;
    pCtx->u32LastSeq = pB->u32Seq;
    pCtx->bHaveLast  = true;
    pCtx->u32RxCount++;

    if (bHaveNf)
    {
        DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                "nf=%d dBm margin=%d dB rx=%lu missed=%lu\r\n",
                (unsigned long)pB->u32Id, (unsigned long)pB->u32Seq,
                pB->i16Rssi, pB->i8Snr, pB->i16NoiseFloor,
                (int)(pB->i16Rssi - pB->i16NoiseFloor),
                (unsigned long)pCtx->u32RxCount, (unsigned long)pCtx->u32Missed);
    }
    else
    {
        DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                "nf=n/a rx=%lu missed=%lu\r\n",
                (unsigned long)pB->u32Id, (unsigned long)pB->u32Seq,
                pB->i16Rssi, pB->i8Snr,
                (unsigned long)pCtx->u32RxCount, (unsigned long)pCtx->u32Missed);
    }

    if (pCtx->u16BatchCount < RTM_FR9_BATCH_MAX)
    {
        FarmrangerRtBeacon_t *pRow = &pCtx->tBatch[pCtx->u16BatchCount++];
        pRow->u32DeviceId   = pB->u32Id;
        pRow->u32Seq        = pB->u32Seq;
        pRow->u32Missed     = pCtx->u32Missed;
        pRow->i16Rssi       = pB->i16Rssi;
        pRow->i16NoiseFloor = pB->i16NoiseFloor;
        pRow->i8Snr         = pB->i8Snr;
    }

    if (pCtx->u16BatchCount >= RTM_FR9_BATCH_MAX)
        RTM_vFlushToFr9();
}

/* --------------------------------------------------------------------------
 * Primary: push the batch to the fr9
 * -------------------------------------------------------------------------- */
static void RTM_vFlushToFr9(void)
{
    if (pCtx == NULL || pCtx->u16BatchCount == 0U)
        return;

    /* A full session per flush, same shape as DEVICE_DISCOVERY_bBasicLogAndClear:
     * raise the attention line, wait for RDY, upload, drop the line. Short
     * sessions keep us well inside the fr9's 600 s session cap over a long
     * range walk. */
    if (FARMRANGER_bDeviceOn())
    {
        (void)FARMRANGER_bLogRadioTestData(pCtx->tBatch, pCtx->u16BatchCount);
        FARMRANGER_vDeviceOff();
    }
    else
    {
        DBG_LOG("RadioTest: fr9 did not answer RDY - batch of %u dropped\r\n",
                pCtx->u16BatchCount);
    }

    /* Cleared either way. A retry queue would grow without bound on a bench rig
     * with no fr9 attached, and the terminal log already has every line. */
    pCtx->u16BatchCount = 0U;
    pCtx->u32BatchStart = osKernelGetTickCount();
}
