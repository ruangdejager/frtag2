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
 *  - The beacon role has no task and no timer. It allocates its context and
 *    nothing else, and runs on MESHNETWORK_vTxTask via
 *    RADIOTESTMODE_u32ServiceBeacon. That is not tidiness, it is the only
 *    thing that fits: a secondary reaches this code with roughly 2.6 KB of
 *    FreeRTOS heap left, split across free blocks, and heap_4 must satisfy a
 *    request from ONE of them. Three attempts at sizing a beacon task ended
 *    in "could not enter (out of heap (task): 2640 B free, wanted 2048)" —
 *    a sum that looks sufficient right up until you remember it is a sum.
 *    Only the listener, which must block for seconds inside a Farmranger AT
 *    exchange and therefore cannot borrow anyone's task, still creates one,
 *    and only a primary — which has GPS, Movement and SolarPower absent from
 *    its heap and ~6.5 KB more of it free — ever takes that path.
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
 *    already banked what it heard. One beacon, one fr9 session - see
 *    RTM_vLogToFr9.
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

/* Listener only — the beacon role creates no task at all; see the note at the
 * top of the file and RADIOTESTMODE_u32ServiceBeacon below.
 *
 * Unlike the compile-time ENABLE_RADIO_TEST build, this task has to fit on the
 * heap ALONGSIDE everything the mesh build already created — LoRa radio task,
 * MeshNetwork's two tasks, DeviceDiscovery's two tasks, FrKernel, Farmranger's
 * two, Battery and Power. Those already commit on the order of 30 KB of the
 * 46 KB FreeRTOS heap between their stacks and queues, so headroom is tight
 * even here, where it is least tight.
 *
 * It stays at *6 regardless. -fstack-usage measures the deep path —
 * RTM_vTask 32 + RTM_vHandleBeacon 72 + FARMRANGER_bLogRadioTestData 288
 * (its own row/cmd/response/line buffers) + FARMRANGER_bATSend 40 — at ~430 B,
 * and under all of it sits DBG_LOG's localtime/strftime/vsnprintf, which is
 * library code -fstack-usage cannot see. It is that unmeasured tail, not the
 * 430 B, that the margin is for. Same figure the compile-time RadioTest.c
 * task uses, for the same reason FrKernel does (see FrKernel.c's own *6
 * comment) — and unlike the beacon role, this allocation is not the one that
 * was failing: only a primary ever reaches it. */
#define RTM_STACK_SIZE_LISTEN   (configMINIMAL_STACK_SIZE * 6)

#define RTM_BEACON_MS       5000U    /* 0.2 Hz, per the test spec */

/* Deliberately shorter than RTM_BEACON_MS: how long to wait, when a beacon
 * comes due but a PREVIOUS one is still sitting in the LoRa TX queue, before
 * checking again. See the "one at a time" note on RADIOTESTMODE_u32ServiceBeacon
 * for why that check exists at all. */
#define RTM_BEACON_RETRY_MS 500U

/* Beacon frame: [0] = type, [1..] = ASCII "RT <id:hex> <seq:dec>".
 *
 * Same wire format as the compile-time test in RadioTest.c, deliberately: a
 * compile-time-built beacon and a runtime-built one are interchangeable on the
 * bench. MeshPktType_Reserved keeps it inert to production units, whose parser
 * switch drops reserved types in its default arm. */
#define RTM_TAG             "RT "
#define RTM_TAG_LEN         3U
#define RTM_LINE_MAX        48U

/* How long RADIOTESTMODE_vExit waits for the task to wind itself down. Sized
 * for the worst case on the way out: the task may be mid-upload for a beacon
 * it just measured — an fr9 session open (~1 s to RDY) plus an AT+RTLOG whose
 * verdict wait is ~6.5 s. */
#define RTM_STOP_WAIT_MS    20000U

/* Thread flags. Listener only — there is no task on the beacon side to flag. */
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

/* Everything the mode needs while it is running, in one heap block — the only
 * allocation a beaconing secondary makes, at 44 bytes plus heap_4's 8-byte
 * block header.
 *
 * No timer object for the beacon cadence, and no task either: the beacon runs
 * on MESHNETWORK_vTxTask, which keeps its own deadline here and calls
 * RADIOTESTMODE_u32ServiceBeacon when it comes round. MeshNetwork's own beacon
 * timer exists to hand work off from the tiny FreeRTOS timer-service stack to
 * that same TX task; there is no reason to take the long way round to a task
 * we are already running on. */
typedef struct {
    osMessageQueueId_t xRxQueue;      /* listener only */

    uint32_t u32TxSeq;                /* beacon: next sequence number      */
    uint32_t u32NextBeaconTick;       /* beacon: tick the next one is due  */

    uint32_t u32RxCount;              /* listener bookkeeping              */
    uint32_t u32Missed;
    uint32_t u32LastId;
    uint32_t u32LastSeq;
    bool     bHaveLast;

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

/* Every reason above is a string literal in flash, deliberately. An earlier
 * version formatted heap byte counts into a 47-byte static buffer, which was
 * scaffolding for a heap failure the design has since removed — the beacon
 * role allocates nothing but its 44-byte context now. 47 bytes of permanently
 * resident RAM to describe a failure that no longer happens is a bad trade in
 * a build that links with well under 100 bytes to spare. The numbers still go
 * to DBG_LOG on the failing unit, which formats into the log ring and costs no
 * static at all; only the ack, which has to outlive this function, is fixed
 * text. */

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void     RTM_vTask(void *arg);
static void     RTM_vHandleBeacon(const RtmBeacon_t *pB);
static void     RTM_vLogToFr9(const FarmrangerRtBeacon_t *pRow);
static void     RTM_vTeardown(void);

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

    /* A beaconing secondary has no task of its own — it rides MeshNetwork's TX
     * worker. Refuse up front if that worker is not there rather than entering
     * a mode that would then never transmit: that is the case in an
     * ENABLE_RADIO_TEST build, where MESHNETWORK_vInit is skipped entirely and
     * the compile-time test in RadioTest.c owns the radio instead. A silent
     * no-beacon "success" would be indistinguishable, from the far end of a
     * range walk, from a link that simply does not reach. */
    if (!bIsListener && !MESHNETWORK_bTxWorkerReady())
    {
        pcLastEnterError = "no mesh TX worker";
        DBG_LOG("RadioTest: enter failed - %s\r\n", pcLastEnterError);
        return false;
    }

    pCtx = pvPortMalloc(sizeof(RtmCtx_t));
    if (pCtx == NULL)
    {
        /* Fixed string for the ack (it has to outlive this function), numbers
         * to the log. Note the free TOTAL is only indicative: heap_4 satisfies
         * a request from ONE block, so a failure here can read as plenty free
         * and still be genuine. */
        pcLastEnterError = "out of heap (ctx)";
        DBG_LOG("RadioTest: enter failed - ctx alloc of %u B, %u B free total\r\n",
                (unsigned)sizeof(RtmCtx_t), (unsigned)xPortGetFreeHeapSize());
        return false;
    }
    memset(pCtx, 0, sizeof(RtmCtx_t));

    if (bIsListener)
    {
        pCtx->xRxQueue = osMessageQueueNew(4U, sizeof(RtmBeacon_t), NULL);
        if (pCtx->xRxQueue == NULL)
        {
            pcLastEnterError = "out of heap (queue)";
            DBG_LOG("RadioTest: enter failed - RX queue, %u B free total\r\n",
                    (unsigned)xPortGetFreeHeapSize());
            RTM_vTeardown();
            return false;
        }

        const osThreadAttr_t attr = {
            .name       = "RadioTestMode",
            .stack_size = RTM_STACK_SIZE_LISTEN * sizeof(StackType_t),
            .priority   = osPriorityNormal,
        };

        /* Set before the thread is created: the task clears it on the way out,
         * and a task that gets scheduled before osThreadNew returns must not
         * observe a stale false here. */
        bTaskRunning = true;

        xRtmTask = osThreadNew(RTM_vTask, NULL, &attr);
        if (xRtmTask == NULL)
        {
            /* Log the size wanted alongside the total free. They can disagree
             * wildly and both be right — heap_4 allocates from a single block,
             * and "2640 B free, wanted 2048" failing is what sent three rounds
             * of stack-size guessing down the wrong path. Only the listener
             * (a primary) reaches this at all; the beacon role stopped needing
             * a task of its own. */
            pcLastEnterError = "out of heap (task)";
            DBG_LOG("RadioTest: enter failed - task stack %u B, %u B free total\r\n",
                    (unsigned)attr.stack_size,
                    (unsigned)xPortGetFreeHeapSize());
            bTaskRunning = false;
            RTM_vTeardown();
            return false;
        }
    }
    else
    {
        /* Due immediately, so the first beacon goes out on the TX worker's
         * very next pass rather than 5 s after the operator sees the ack. */
        pCtx->u32NextBeaconTick = osKernelGetTickCount();
    }

    /* Hold the system out of STOP2 for the duration. STOP2 parks the debug
     * UART pins to analog and suspends the core, so the radio task could only
     * service RX once per 1 Hz RTC heartbeat - fine for a sleeping node,
     * useless for a beacon on a 5 s cadence or for a listener. Released in
     * RADIOTESTMODE_vExit, and nowhere else, so the pairing stays obvious. */
    SYSTEM_vSleepLockAcquire();

    /* ...and hold the RADIO out of deep sleep, which is a separate thing from
     * the MCU sleep lock above. Both are needed and neither substitutes for
     * the other: the sleep lock keeps the core running and the debug UART
     * alive, this keeps the transceiver able to answer a CAD.
     *
     * Taken BEFORE the wake-up below, because the thing it defends against is
     * already in flight by the time we get here: entering the test from a
     * FrKernel session clears s_bConnected, which releases DeviceDiscovery's
     * AppTask from its "waiting for release" hold, and the campaign tail that
     * follows calls LORARADIO_vEnterDeepSleep(). Without this the test wakes
     * the radio, sends one or two beacons, and is then slept out from under —
     * after which every carrier sense times out with no IRQ and the terminal
     * shows an endlessly busy channel instead of a sleeping radio. */
    LORARADIO_vSetKeepAwake(true);

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

    /* The TX worker we are about to beacon from is parked on osWaitForever
     * with nothing left to wake it - the gates above just made sure of that.
     * Poke it so it goes round once more and picks up the beacon deadline as
     * its timeout. After bActive, or it would go straight back to sleep. */
    if (!bIsListener)
        MESHNETWORK_vWakeTxWorker();

    /* Free heap on the way in, not just when it runs out: this is the number
     * that decides whether the mode fits, and having it logged for a
     * successful entry is what makes the next failure a comparison rather
     * than another first data point. */
    DBG_LOG("RadioTest: ENTERED as %s, %u B heap free\r\n",
            bIsListener ? "PRIMARY (listen)" : "SECONDARY (beacon)",
            (unsigned)xPortGetFreeHeapSize());
    return true;
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_u32ServiceBeacon - runs on MESHNETWORK_vTxTask
 * -------------------------------------------------------------------------- */
uint32_t RADIOTESTMODE_u32ServiceBeacon(void)
{
    LoraRadio_Packet_t pkt;
    uint32_t           u32Now;
    uint32_t           u32Seq;
    uint32_t           u32Id;
    int                n;

    /* Cheap out first: this is on every trip round the mesh TX worker's loop
     * in normal operation, and must cost one volatile read. */
    if (!bActive || bIsListener)
        return osWaitForever;

    u32Now = osKernelGetTickCount();

    /* Read the context and claim the beacon in one atomic step, then work from
     * locals only. We are on a task this file does not own, and
     * RADIOTESTMODE_vExit can free pCtx from Movement (shake) or from FrKernel
     * at any moment. Nothing below the critical section touches pCtx, so the
     * worst a teardown landing mid-beacon can do is put one last frame on the
     * air - it can never dereference freed memory, whatever the priorities of
     * the two tasks happen to be. */
    taskENTER_CRITICAL();

    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        return osWaitForever;
    }

    /* Signed difference, so this survives the tick counter wrapping. */
    if ((int32_t)(pCtx->u32NextBeaconTick - u32Now) > 0)
    {
        uint32_t u32Left = pCtx->u32NextBeaconTick - u32Now;
        taskEXIT_CRITICAL();
        return u32Left;             /* woken early - nothing due yet */
    }

    taskEXIT_CRITICAL();

    /* One of our own beacons outstanding at a time, never two.
     *
     * RTM_BEACON_MS (5 s) and LORA_TX_CARRIER_WAIT_MS (5 s, the radio task's
     * own carrier-sense budget per packet) are numerically equal, which is not
     * a coincidence to route around so much as a fact to respect: under a busy
     * channel, a packet can still be being carrier-sensed when the NEXT
     * beacon becomes due by our own clock. Enqueuing anyway would let our
     * beacons pile up behind each other — worse, faster than they can
     * possibly be sent — for no gain.
     *
     * Both checks are needed: the radio task DEQUEUES a packet before it
     * carrier-senses it, so the queue alone reads empty for the entire
     * multi-second window a packet can be in flight — that window is exactly
     * what caused the pile-up, so it is the one that must not be missed.
     *
     * Checking here, before claiming the sequence number, means a beacon
     * delayed this way is not yet "sent" and is retried at its own number
     * rather than skipped — the count of genuinely un-transmittable beacons is
     * what RTM_BEACON_RETRY_MS is for. */
    if (LORARADIO_u16GetTxQueueDepth() > 0U || LORARADIO_bTxInProgress())
        return RTM_BEACON_RETRY_MS;

    taskENTER_CRITICAL();

    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        return osWaitForever;
    }

    /* Claimed even though the send below may drop it: to the listener a
     * dropped beacon is identical to one that never made it across, and the
     * sequence gap should say so. */
    u32Seq                  = pCtx->u32TxSeq++;
    pCtx->u32NextBeaconTick = u32Now + RTM_BEACON_MS;

    taskEXIT_CRITICAL();

    u32Id = LORARADIO_u32GetUniqueId();

    memset(&pkt, 0, sizeof(pkt));
    pkt.buffer[0] = (uint8_t)MeshPktType_Reserved;

    n = snprintf((char *)&pkt.buffer[1],
                 (size_t)(LORA_MAX_PACKET_SIZE - 2),
                 RTM_TAG "%04lX %lu",
                 (unsigned long)u32Id, (unsigned long)u32Seq);

    if (n > 0)
    {
        pkt.length = (uint8_t)(n + 1);

        if (LORARADIO_bTxPacket(&pkt))
            DBG_LOG("RadioTest: TX seq=%lu\r\n", (unsigned long)u32Seq);
        else
            DBG_LOG("RadioTest: TX drop seq=%lu (queue full)\r\n",
                    (unsigned long)u32Seq);
    }

    return RTM_BEACON_MS;
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vExit
 * -------------------------------------------------------------------------- */
void RADIOTESTMODE_vExit(const char *pcReason)
{
    uint32_t u32Rx     = 0U;
    uint32_t u32Missed = 0U;
    uint32_t u32Tx     = 0U;

    if (!bActive)
        return;

    /* Clear the gates first so the mesh is live again the moment we stop, and
     * so nothing new is handed to a queue we are about to delete. It is also
     * what stops the next RADIOTESTMODE_u32ServiceBeacon on the mesh TX worker
     * dead, and puts that worker back on osWaitForever. */
    bActive = false;

    /* Hand the radio's power state back to its normal owners. Released here,
     * with bActive, rather than down beside SYSTEM_vSleepLockRelease() so the
     * two early returns below cannot leave the radio pinned awake for the rest
     * of the unit's uptime — that would cost real battery on a secondary and
     * would be invisible until someone noticed the drain. */
    LORARADIO_vSetKeepAwake(false);

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
     * attempts. Listener only - a beaconing secondary has no task, so the
     * shake path (Movement task) skips all of this and returns at once. */
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
    }

    if (pCtx != NULL)
    {
        u32Rx     = pCtx->u32RxCount;
        u32Missed = pCtx->u32Missed;
        u32Tx     = pCtx->u32TxSeq;
    }

    RTM_vTeardown();

    SYSTEM_vSleepLockRelease();

    /* Per role, because the other role's counters are all zero and printing
     * them reads as a test that measured nothing. On a secondary this line is
     * the whole result of the run - it is deaf for the duration, so the beacon
     * count is the only thing it can report, and a shake-exit is the only
     * place it gets reported. */
    if (bIsListener)
        DBG_LOG("RadioTest: EXITED (%s) - rx=%lu missed=%lu\r\n",
                (pcReason != NULL) ? pcReason : "?",
                (unsigned long)u32Rx, (unsigned long)u32Missed);
    else
        DBG_LOG("RadioTest: EXITED (%s) - tx=%lu\r\n",
                (pcReason != NULL) ? pcReason : "?",
                (unsigned long)u32Tx);
}

/* Free everything the mode allocated. The listener's task has already exited
 * by the time this runs from RADIOTESTMODE_vExit; from RADIOTESTMODE_bEnter's
 * failure path it either never started or is terminated here. Safe with any
 * subset built, which is what makes it usable as that failure path. */
static void RTM_vTeardown(void)
{
    RtmCtx_t *pDoomed;

    if (xRtmTask != NULL)
    {
        if (bTaskRunning)
        {
            osThreadTerminate(xRtmTask);
            bTaskRunning = false;
        }
        xRtmTask = NULL;
    }

    /* Unpublish before freeing, in one atomic step. RADIOTESTMODE_u32ServiceBeacon
     * runs on the mesh TX worker and reads pCtx under the same critical
     * section, so it either gets a context that is still whole or sees NULL -
     * there is no window in which it holds a pointer we are freeing. */
    taskENTER_CRITICAL();
    pDoomed = pCtx;
    pCtx    = NULL;
    taskEXIT_CRITICAL();

    if (pDoomed != NULL)
    {
        if (pDoomed->xRxQueue != NULL)
            osMessageQueueDelete(pDoomed->xRxQueue);

        vPortFree(pDoomed);
    }
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vOnBeacon - parser-task context, keep it cheap
 *
 * "Cheap" is also what makes the pCtx reads below safe. Nothing in here
 * blocks or yields, so this function runs to completion against every task
 * that can call RADIOTESTMODE_vExit and free pCtx underneath it - FrKernel
 * and Movement are both osPriorityLow, the parser this runs on is Normal.
 * Anything added here that can block breaks that, and would need the same
 * critical section RADIOTESTMODE_u32ServiceBeacon uses.
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
 * RTM_vTask - listener only
 * -------------------------------------------------------------------------- */
static void RTM_vTask(void *arg)
{
    (void)arg;

    for (;;)
    {
        RtmBeacon_t tB;

        /* The 1 s timeout is only a periodic liveness check - nothing is timed
         * off it. Real work arrives as RTM_FLAG_RX from the parser task. */
        uint32_t flags = osThreadFlagsWait(RTM_FLAG_RX | RTM_FLAG_STOP,
                                           osFlagsWaitAny, 1000U);
        if (flags & osFlagsError)
            flags = 0U;

        if (!bActive || (flags & RTM_FLAG_STOP) || pCtx == NULL)
            break;

        while (pCtx->xRxQueue != NULL &&
               osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
        {
            RTM_vHandleBeacon(&tB);
        }
    }

    /* Winding down. Anything already measured is already on the fr9 - that is
     * the point of logging per beacon - but the queue may still hold beacons
     * the parser handed us in the last moments, and the last beacons of a
     * range walk are the ones from the far end. Log those before going.
     *
     * No session to close: each beacon opens and drops its own, so by here the
     * attention line is already low.
     *
     * Done here rather than in RADIOTESTMODE_vExit so it runs on this task's
     * stack and cannot race the loop above. */
    if (pCtx != NULL)
    {
        RtmBeacon_t tB;

        while (pCtx->xRxQueue != NULL &&
               osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
        {
            RTM_vHandleBeacon(&tB);
        }

        DBG_LOG("RadioTest: fr9 logged=%lu failed=%lu\r\n",
                (unsigned long)pCtx->u32Fr9Logged,
                (unsigned long)pCtx->u32Fr9Failed);
    }

    /* Last act: tell RADIOTESTMODE_vExit it is safe to free the context.
     * Returning from a CMSIS-RTOS2 thread function exits the thread. */
    bTaskRunning = false;
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
 * Primary: push one beacon to the fr9, as it is measured
 *
 * One beacon, one session: assert the attention line, wait for RDY, send the
 * single AT+RTLOG, de-assert. Every beacon therefore starts from a known
 * state, which is the whole point - a session held across beacons has to stay
 * healthy for the length of a range walk, and when it does not (the fr9's own
 * 600 s cap, a reset, a cable knocked loose) the failure is silent and every
 * subsequent beacon is lost. That is what the held-session version did in
 * practice: it logged the first beacon and then nothing.
 *
 * The cost is the fr9's fixed ~1 s wait before it answers RDY, once per
 * beacon. Against a 5 s cadence that is affordable, and it buys a design with
 * no session lifetime to manage, no recycle timer, and nothing to go stale.
 * -------------------------------------------------------------------------- */
static void RTM_vLogToFr9(const FarmrangerRtBeacon_t *pRow)
{
    bool bOk;

    if (pCtx == NULL || pRow == NULL)
        return;

    if (!FARMRANGER_bDeviceOn())
    {
        DBG_LOG("RadioTest: fr9 did not answer RDY\r\n");
        pCtx->u32Fr9Failed++;
        FARMRANGER_vDeviceOff();   /* drop the line again; no session to keep */
        return;
    }

    bOk = FARMRANGER_bLogRadioTestData(pRow, 1U);

    FARMRANGER_vDeviceOff();

    /* Hold the attention line low long enough for the fr9 to see the falling
     * edge before the next beacon raises it again. It debounces GPIO1 for
     * 20 ms and starts a session on the RISING edge, so back-to-back sessions
     * that never showed a clean low would look like no edge at all. There are
     * ~5 s until the next beacon, so this costs nothing. */
    osDelay(50);

    if (bOk)
        pCtx->u32Fr9Logged++;
    else
        pCtx->u32Fr9Failed++;
}
