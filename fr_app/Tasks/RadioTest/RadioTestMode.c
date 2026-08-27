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
 *
 *  - Every beacon goes to the fr9 immediately, nothing is buffered, so the
 *    flash log tracks the walk live and a unit that loses power mid-test has
 *    already banked what it heard. The Farmranger session is held open across
 *    beacons to make that cheap - see RTM_FR9_SESSION_MS.
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

/* Unlike the compile-time ENABLE_RADIO_TEST build, this task has to fit on the
 * heap ALONGSIDE everything the mesh build already created — LoRa radio task,
 * MeshNetwork's two tasks, DeviceDiscovery's two tasks, FrKernel, GPS, and on
 * a secondary Movement/ACC/SolarPower/Battery too. Those already commit on
 * the order of 30+ KB of the 46 KB FreeRTOS heap between their stacks and
 * queues, so headroom here is genuinely tight and the two roles are sized
 * separately rather than sharing one figure:
 *
 *   - The secondary's call depth is shallow — build a packet, LORARADIO_bTxPacket,
 *     DBG_LOG — so *4 is ample with margin.
 *   - The primary's is not: RTM_vHandleBeacon runs into
 *     FARMRANGER_bLogRadioTestData -> FARMRANGER_bRtLogAttempt, which has its
 *     own row/cmd/response buffers on the stack before it even reaches
 *     FARMRANGER_bATSend and the UART layer beneath it. Keep the *6 the
 *     compile-time RadioTest.c task already uses for the same reason FrKernel
 *     does (see FrKernel.c's own *6 comment) — trimming this one and hitting
 *     a real overrun would be a much worse failure than the extra 1 KB. */
#define RTM_STACK_SIZE_BEACON   (configMINIMAL_STACK_SIZE * 4)
#define RTM_STACK_SIZE_LISTEN   (configMINIMAL_STACK_SIZE * 6)

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

/* fr9 logging. Every beacon is pushed to the fr9 the moment it is measured -
 * nothing is buffered - so the flash log tracks the walk live and a unit that
 * loses power mid-test has already banked everything it heard.
 *
 * To make that affordable the Farmranger session is opened once and HELD for
 * the duration, rather than a fresh session per beacon. A session costs a GPIO
 * attention edge and the fr9's fixed ~1 s wait before it answers RDY, and -
 * more to the point - the fr9 writes a session-start line to the very syslog
 * we are trying to read. At one session per 5 s beacon that is ~720 lines an
 * hour of noise around the data. Held open, each beacon costs only the
 * AT+RTLOG exchange, which is tens of milliseconds.
 *
 * The fr9 force-ends a session at FRTAG_SESSION_MAX_S (600 s) even with the
 * attention line still asserted, and needs a fresh rising edge to start
 * another - so recycle ours before it gets there rather than discovering it
 * mid-beacon. */
#define RTM_FR9_SESSION_MS  540000U   /* 9 min, inside the fr9's 600 s cap */

/* How long RADIOTESTMODE_vExit waits for the task to wind itself down. Sized
 * for the worst case on the way out: the task may be mid-upload for a beacon
 * it just measured, and that upload can cost two AT+RTLOG attempts (each with
 * a handshake and a ~6.5 s verdict wait) plus two fr9 session opens. */
#define RTM_STOP_WAIT_MS    20000U

/* Thread flags. No RTM_FLAG_BEACON: the beacon role has no osTimer behind it
 * (see the note on RtmCtx_t below) — RTM_vTask just times its own wait out. */
#define RTM_FLAG_RX         (1UL << 0)   /* a beacon landed in the queue */
#define RTM_FLAG_STOP       (1UL << 1)   /* exit requested               */

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

/* Everything the mode needs while it is running, in one heap block.
 *
 * No timer object for the beacon cadence, on purpose: MeshNetwork's own
 * beacon timer exists to hand off to a SEPARATE already-running TX task via a
 * tiny callback on the FreeRTOS timer-service stack. RTM_vTask is not that —
 * it is a full task with its own stack, so it can just time its own wait out
 * every RTM_BEACON_MS and send from there. A software timer would be a second
 * heap allocation (its control block) for something a plain wait already
 * does, and on a secondary this heap is tight enough that this genuinely
 * mattered — see the comment on RTM_STACK_SIZE_BEACON. */
typedef struct {
    osMessageQueueId_t xRxQueue;      /* listener only */

    uint32_t u32TxSeq;                /* beacon: next sequence number      */

    uint32_t u32RxCount;              /* listener bookkeeping              */
    uint32_t u32Missed;
    uint32_t u32LastId;
    uint32_t u32LastSeq;
    bool     bHaveLast;

    bool     bFr9Up;                  /* Farmranger session currently open  */
    uint32_t u32Fr9SessionStart;      /* tick it was opened, for the recycle */
    uint32_t u32Fr9Logged;            /* beacons the fr9 acknowledged        */
    uint32_t u32Fr9Failed;            /* beacons it did not                  */
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

/* Why the last RADIOTESTMODE_bEnter() call failed, if it did. A LoRa-addressed
 * command's ack is the ONLY thing the caller ever sees — the DBG_LOG lines
 * below land on the failing unit's own debug UART, which is exactly what a
 * remote operator cannot read. Surfaced via RADIOTESTMODE_pcLastEnterError()
 * so FrKernel can put the real reason in the ack instead of a bare
 * "could not enter" that gives a field failure nothing to go on. */
static const char *pcLastEnterError = "";

/* Backing store for a failure reason that needs to carry a number (free-heap
 * byte counts) rather than just a fixed string. Sized exactly for the longest
 * message this file builds — "out of heap (task): 65535 B free, wanted
 * 65535" is 46 chars — not rounded up: every static byte here is RAM this
 * build cannot spare (see RTM_STACK_SIZE_BEACON's comment on how tight the
 * heap already is; the same scarcity applies to .bss). */
static char acLastEnterError[47];

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void RTM_vTask(void *arg);
static void RTM_vSendBeacon(void);
static void RTM_vHandleBeacon(const RtmBeacon_t *pB);
static void RTM_vLogToFr9(const FarmrangerRtBeacon_t *pRow);
static bool RTM_bFr9SessionOpen(void);
static void RTM_vFr9SessionClose(void);
static void RTM_vTeardown(void);

/* --------------------------------------------------------------------------
 * Public state
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bActive(void)     { return bActive; }
bool RADIOTESTMODE_bIsListener(void) { return bIsListener; }

const char *RADIOTESTMODE_pcLastEnterError(void) { return pcLastEnterError; }

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_bEnter
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bEnter(void)
{
    pcLastEnterError = "";

    if (bActive || pCtx != NULL)
    {
        pcLastEnterError = "already active";
        return false;
    }

    bIsListener = (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY);

    pCtx = pvPortMalloc(sizeof(RtmCtx_t));
    if (pCtx == NULL)
    {
        snprintf(acLastEnterError, sizeof(acLastEnterError),
                "out of heap (ctx): %u B free, wanted %u",
                (unsigned)xPortGetFreeHeapSize(), (unsigned)sizeof(RtmCtx_t));
        pcLastEnterError = acLastEnterError;
        DBG_LOG("RadioTest: enter failed - %s\r\n", acLastEnterError);
        return false;
    }
    /* memset gives every counter its zero and, importantly, leaves bFr9Up
     * false: the fr9 session is opened lazily by the first beacon, so a
     * listener that never hears one never raises the attention line. */
    memset(pCtx, 0, sizeof(RtmCtx_t));

    /* Beacon role needs nothing else here — no timer object; see the note on
     * RtmCtx_t for why. */
    if (bIsListener)
    {
        pCtx->xRxQueue = osMessageQueueNew(4U, sizeof(RtmBeacon_t), NULL);
        if (pCtx->xRxQueue == NULL)
        {
            snprintf(acLastEnterError, sizeof(acLastEnterError),
                    "out of heap (rx queue): %u B free",
                    (unsigned)xPortGetFreeHeapSize());
            pcLastEnterError = acLastEnterError;
            DBG_LOG("RadioTest: enter failed - %s\r\n", acLastEnterError);
            RTM_vTeardown();
            return false;
        }
    }

    /* Two attrs, not one: see the stack-size comment above for why the roles
     * cannot share a figure on a heap this tight. */
    const osThreadAttr_t attr = {
        .name       = "RadioTestMode",
        .stack_size = (bIsListener ? RTM_STACK_SIZE_LISTEN : RTM_STACK_SIZE_BEACON)
                      * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    /* Set before the thread is created: the task clears it on the way out, and
     * a task that gets scheduled before osThreadNew returns must not observe a
     * stale false here. */
    bTaskRunning = true;

    xRtmTask = osThreadNew(RTM_vTask, NULL, &attr);
    if (xRtmTask == NULL)
    {
        /* The number, not just the category: this ack is very likely the
         * only place a remote operator ever sees this, and "out of heap" on
         * its own leaves them guessing exactly how far short it fell. */
        snprintf(acLastEnterError, sizeof(acLastEnterError),
                "out of heap (task): %u B free, wanted %u",
                (unsigned)xPortGetFreeHeapSize(), (unsigned)attr.stack_size);
        pcLastEnterError = acLastEnterError;
        DBG_LOG("RadioTest: enter failed - %s\r\n", acLastEnterError);
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
        if (pCtx->xRxQueue != NULL)
            osMessageQueueDelete(pCtx->xRxQueue);

        vPortFree(pCtx);
        pCtx = NULL;
    }
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

    /* Beacon role has no timer object (see the note on RtmCtx_t) - it just
     * times its own wait out every RTM_BEACON_MS and sends from there.
     * Listener role reacts to RTM_FLAG_RX; its timeout is only a periodic
     * liveness check, nothing is timed off it. */
    const uint32_t u32WaitMs = bIsListener ? 1000U : RTM_BEACON_MS;

    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(RTM_FLAG_RX | RTM_FLAG_STOP,
                                           osFlagsWaitAny, u32WaitMs);
        if (flags & osFlagsError)
            flags = 0U;

        if (!bActive || (flags & RTM_FLAG_STOP) || pCtx == NULL)
            break;

        if (!bIsListener)
        {
            /* Nothing else can set a flag for this role (RADIOTESTMODE_vOnBeacon
             * only signals the listener), so reaching here with STOP already
             * ruled out means the wait timed out - RTM_BEACON_MS since the
             * last beacon. */
            RTM_vSendBeacon();
            continue;
        }

        {
            RtmBeacon_t tB;
            while (pCtx->xRxQueue != NULL &&
                   osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
            {
                RTM_vHandleBeacon(&tB);
            }
        }
    }

    /* Winding down. Anything already measured is already on the fr9 - that is
     * the point of logging per beacon - but the queue may still hold beacons
     * the parser handed us in the last moments, and the last beacons of a
     * range walk are the ones from the far end. Log those, then drop the
     * attention line so the fr9 can end its session cleanly instead of sitting
     * out its 600 s cap.
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

        RTM_vFr9SessionClose();

        DBG_LOG("RadioTest: fr9 logged=%lu failed=%lu\r\n",
                (unsigned long)pCtx->u32Fr9Logged,
                (unsigned long)pCtx->u32Fr9Failed);
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

    /* Straight to the fr9, now, while the beacon is in hand. This blocks for
     * the AT+RTLOG exchange - tens of milliseconds normally - which is why it
     * runs on this task and not on the parser that handed us the beacon.
     * Anything that arrives meanwhile waits in the RX queue. */
    {
        FarmrangerRtBeacon_t tRow;
        tRow.u32DeviceId   = pB->u32Id;
        tRow.u32Seq        = pB->u32Seq;
        tRow.u32Missed     = pCtx->u32Missed;
        tRow.i16Rssi       = pB->i16Rssi;
        tRow.i16NoiseFloor = pB->i16NoiseFloor;
        tRow.i8Snr         = pB->i8Snr;

        RTM_vLogToFr9(&tRow);
    }
}

/* --------------------------------------------------------------------------
 * Primary: fr9 session handling
 * -------------------------------------------------------------------------- */

/* Raise the attention line and wait for the fr9's RDY. */
static bool RTM_bFr9SessionOpen(void)
{
    if (pCtx == NULL)
        return false;

    if (pCtx->bFr9Up)
        return true;

    if (!FARMRANGER_bDeviceOn())
    {
        DBG_LOG("RadioTest: fr9 did not answer RDY\r\n");
        return false;
    }

    pCtx->bFr9Up             = true;
    pCtx->u32Fr9SessionStart = osKernelGetTickCount();
    return true;
}

static void RTM_vFr9SessionClose(void)
{
    if (pCtx == NULL || !pCtx->bFr9Up)
        return;

    FARMRANGER_vDeviceOff();
    pCtx->bFr9Up = false;

    /* Hold the attention line low long enough for the fr9 to actually see the
     * edge. It debounces GPIO1 for 20 ms and starts a session on the RISING
     * edge, so a close immediately followed by an open - which is exactly what
     * the recycle path does - would otherwise look like no change at all and
     * the fr9 would stay in the session it had already given up on. */
    osDelay(50);
}

/* --------------------------------------------------------------------------
 * Primary: push one beacon to the fr9, as it is measured
 * -------------------------------------------------------------------------- */
static void RTM_vLogToFr9(const FarmrangerRtBeacon_t *pRow)
{
    if (pCtx == NULL || pRow == NULL)
        return;

    /* Recycle before the fr9's own 600 s cap force-ends the session under us.
     * Done here, between beacons, so the drop/raise never lands in the middle
     * of an AT+RTLOG exchange. */
    if (pCtx->bFr9Up &&
        (osKernelGetTickCount() - pCtx->u32Fr9SessionStart) >= RTM_FR9_SESSION_MS)
    {
        RTM_vFr9SessionClose();
    }

    if (!RTM_bFr9SessionOpen())
    {
        pCtx->u32Fr9Failed++;
        return;
    }

    if (FARMRANGER_bLogRadioTestData(pRow, 1U))
    {
        pCtx->u32Fr9Logged++;
        return;
    }

    /* One retry, and only after a full session recycle. A failed upload nearly
     * always means the session is gone (the fr9 timed it out, or the board was
     * reset or unplugged mid-walk); retrying into the same dead session would
     * just spend another verdict timeout to learn the same thing. Reopening
     * gives the fr9 the fresh rising edge it needs to start a new one.
     *
     * If that also fails, count it and move on rather than fight for one row:
     * the next beacon is 5 s away, the terminal log already has this one, and
     * the gap is visible in the Seq column of the fr9 log. */
    RTM_vFr9SessionClose();

    if (RTM_bFr9SessionOpen() && FARMRANGER_bLogRadioTestData(pRow, 1U))
    {
        pCtx->u32Fr9Logged++;
        DBG_LOG("RadioTest: fr9 session recycled mid-test\r\n");
        return;
    }

    pCtx->u32Fr9Failed++;
}
