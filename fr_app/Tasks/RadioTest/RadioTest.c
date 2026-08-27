/*
 * RadioTest.c
 *
 * Radio link/range test task — compiled only when ENABLE_RADIO_TEST is defined.
 *
 * Role is taken from the GPIO role strap, so the same image goes on both
 * boards of the pair:
 *
 *   SECONDARY — beacons once every 5 s, carrying its unique ID and a
 *               sequence number.
 *   PRIMARY   — sits in continuous RX and DBG_LOGs one line per beacon with
 *               the RSSI and SNR it measured, the channel noise floor and the
 *               resulting link margin, plus a running count of beacons missed
 *               (derived from gaps in the sequence number).
 *
 * RSSI alone is a poor range measure on two counts: it hides dropouts (hence
 * the sequence number) and it means nothing without the noise it is competing
 * with (hence the noise floor, sampled by the radio task while the channel is
 * idle — see LORARADIO_vRadioTask). Walk the secondary away from the primary
 * and read the range off the primary's terminal; DBG_LOG also lands in the
 * ext-flash log, so the walk is recoverable afterwards.
 *
 * DeviceDiscovery and MeshNetwork must NOT be initialised alongside this
 * task — see init.c for the guard. That also means nothing else is draining
 * the LoRa RX queue, so the primary loop below can own it outright.
 */

/* Before the guard, not after: build_config.h is what defines
 * ENABLE_RADIO_TEST. With the #ifdef first this file compiled to an empty
 * object and the link failed on RADIO_TEST_vInit -- the flag only ever
 * worked when it was also set as an IDE preprocessor symbol. */
#include "build_config.h"

#ifdef ENABLE_RADIO_TEST

#include "RadioTest.h"
#include "LoraRadio.h"
#include "DeviceDiscovery.h"    /* DEVICE_DISCOVERY_eGetDeviceRole, DeviceRole_e */
#include "MeshNetwork.h"        /* MeshPktType_Reserved */
#include "dbg_log.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Private defines
 * -------------------------------------------------------------------------- */
/*
 * *6 rather than the *4 this task used as a bare TX smoke test: both loops
 * now format with snprintf AND call DBG_LOG, whose timestamp formatting is
 * what overran the FrKernel task (see FrKernel.c). RAM is not tight in this
 * build — MeshNetwork's two tasks, DeviceDiscovery, GPS and Movement are all
 * absent.
 */
#define RADIO_TEST_STACK_SIZE   (configMINIMAL_STACK_SIZE * 6)
#define RADIO_TEST_BEACON_MS    5000U   /* 0.2 Hz */

/*
 * Beacon frame: [0] = type, [1..] = ASCII "RT <id:hex> <seq:dec>".
 *
 * The type byte is MeshPktType_Reserved (0) on purpose. A production unit in
 * earshot runs MESHNETWORK_vParserTask, whose type switch has a default arm,
 * so a reserved type is dropped silently there. Minting a real MeshPktType_e
 * for a bench-only mode would also oblige us to extend MeshPktTypeStr[] in
 * lockstep — it is indexed unguarded on the mesh TX path.
 */
#define RADIO_TEST_TAG          "RT "
#define RADIO_TEST_TAG_LEN      3U

/* Longest frame we build is ~16 bytes; 48 leaves room and still bounds the
 * stack. Anything longer than this is not one of ours anyway. */
#define RADIO_TEST_LINE_MAX     48U

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void RADIO_TEST_vTask(void *arg);
static void RADIO_TEST_vSecondaryLoop(void);
static void RADIO_TEST_vPrimaryLoop(void);

/* --------------------------------------------------------------------------
 * RADIO_TEST_vInit
 * Creates the test task.  Call from init.c in place of MESHNETWORK_vInit /
 * DEVICE_DISCOVERY_vInit when ENABLE_RADIO_TEST is defined.
 * -------------------------------------------------------------------------- */
void RADIO_TEST_vInit(void)
{
    static const osThreadAttr_t attr = {
        .name       = "RadioTest",
        .stack_size = RADIO_TEST_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    osThreadId_t h = osThreadNew(RADIO_TEST_vTask, NULL, &attr);
    configASSERT(h != NULL);
}

/* --------------------------------------------------------------------------
 * RADIO_TEST_vTask
 * -------------------------------------------------------------------------- */
static void RADIO_TEST_vTask(void *arg)
{
    (void)arg;

    /*
     * Allow the LoRa radio task to complete its first DRIVER_vEnterRxMode
     * call (which wakes the radio from the deep-sleep entered during
     * LORARADIO_vInit) before we attempt to enqueue a TX or expect an RX.
     */
    osDelay(500U);

    /*
     * The role strap is read once by DEVICE_DISCOVERY_vConfigDeviceRole()
     * early in INIT_vInitialization(), before the mode select that skips
     * DEVICE_DISCOVERY_vInit() — so the getter is valid here even though the
     * DeviceDiscovery task never starts.
     */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        RADIO_TEST_vSecondaryLoop();    /* never returns */
    }
    else
    {
        RADIO_TEST_vPrimaryLoop();      /* never returns */
    }
}

/* --------------------------------------------------------------------------
 * RADIO_TEST_vSecondaryLoop — beacon every RADIO_TEST_BEACON_MS
 * -------------------------------------------------------------------------- */
static void RADIO_TEST_vSecondaryLoop(void)
{
    const uint32_t u32Id  = LORARADIO_u32GetUniqueId();
    uint32_t       u32Seq = 0U;
    uint32_t       u32Next;

    DBG_LOG("\r\nRadioTest: SECONDARY — beaconing id=%04lX every %lu ms\r\n",
            u32Id, (uint32_t)RADIO_TEST_BEACON_MS);

    u32Next = osKernelGetTickCount();

    for (;;)
    {
        LoraRadio_Packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));

        pkt.buffer[0] = (uint8_t)MeshPktType_Reserved;

        int n = snprintf((char *)&pkt.buffer[1],
                         (size_t)(LORA_MAX_PACKET_SIZE - 2),
                         RADIO_TEST_TAG "%04lX %lu", u32Id, u32Seq);

        if (n > 0)
        {
            pkt.length = (uint8_t)(n + 1);

            if (LORARADIO_bTxPacket(&pkt))
                DBG_LOG("RadioTest: TX seq=%lu\r\n", u32Seq);
            else
                DBG_LOG("RadioTest: TX drop seq=%lu (queue full)\r\n", u32Seq);
        }

        /* Bumped even when the enqueue failed: the primary counts gaps as
         * beacons it never heard, and a drop here is exactly that as far as
         * the link is concerned. */
        u32Seq++;

        /* osDelayUntil, not osDelay: the TX path carrier-senses with back-off
         * (up to 5 s on a busy channel) before keying up, so a plain delay
         * would let that jitter accumulate and the sequence-to-wall-clock
         * mapping would drift over a long walk. */
        u32Next += (uint32_t)RADIO_TEST_BEACON_MS;
        osDelayUntil(u32Next);
    }
}

/* --------------------------------------------------------------------------
 * RADIO_TEST_vPrimaryLoop — receive, measure, log
 * -------------------------------------------------------------------------- */
static void RADIO_TEST_vPrimaryLoop(void)
{
    uint32_t u32Rx      = 0U;   /* beacons received */
    uint32_t u32Missed  = 0U;   /* beacons inferred lost from sequence gaps */
    uint32_t u32LastId  = 0U;
    uint32_t u32LastSeq = 0U;
    bool     bHaveLast  = false;

    /* Noise-floor sampling is off by default so normal builds pay nothing for
     * it; this build's whole purpose is the measurement, so switch it on. */
    LORARADIO_vSetNoiseFloorSampling(true);

    DBG_LOG("\r\nRadioTest: PRIMARY — listening, logging RSSI per beacon\r\n");

    for (;;)
    {
        LoraRadio_Packet_t pkt;
        char               line[RADIO_TEST_LINE_MAX];
        uint8_t            len;
        char              *pEnd = NULL;
        uint32_t           u32Id;
        uint32_t           u32Seq;

        /* Blocks forever on the RX queue. Nothing else drains it in this
         * build — MeshNetwork's parser task is not running. */
        if (!LORARADIO_bRxPacket(&pkt))
            continue;

        /* length still counts the type byte; the radio layer already
         * verified and stripped its CRC8. */
        len = (pkt.length > 0U) ? (uint8_t)(pkt.length - 1U) : 0U;
        if (len >= (uint8_t)RADIO_TEST_LINE_MAX)
            len = (uint8_t)(RADIO_TEST_LINE_MAX - 1U);

        memcpy(line, &pkt.buffer[1], len);
        line[len] = '\0';

        if (pkt.buffer[0] != (uint8_t)MeshPktType_Reserved ||
            len <= (uint8_t)RADIO_TEST_TAG_LEN ||
            strncmp(line, RADIO_TEST_TAG, RADIO_TEST_TAG_LEN) != 0)
        {
            /* Not one of ours — a mis-strapped board or stray mesh traffic.
             * Worth one terse line rather than a silent drop. */
            DBG_LOG("RadioTest: RX other type=%u len=%u rssi=%d dBm\r\n",
                    pkt.buffer[0], pkt.length, pkt.rssi);
            continue;
        }

        u32Id = strtoul(&line[RADIO_TEST_TAG_LEN], &pEnd, 16);
        if (pEnd == &line[RADIO_TEST_TAG_LEN])
        {
            DBG_LOG("RadioTest: RX malformed \"%s\"\r\n", line);
            continue;
        }
        u32Seq = strtoul(pEnd, NULL, 10);

        if (bHaveLast && (u32Id == u32LastId))
        {
            if (u32Seq > u32LastSeq)
                u32Missed += (u32Seq - u32LastSeq - 1U);
        }
        else if (bHaveLast)
        {
            /* A different secondary took the bench — the old tally is not
             * comparable, so start it over rather than report nonsense. */
            u32Missed = 0U;
        }

        u32LastId  = u32Id;
        u32LastSeq = u32Seq;
        bHaveLast  = true;
        u32Rx++;

        /* margin = how far the beacon sat above the noise the channel was
         * carrying just before it arrived. That, not RSSI on its own, is what
         * says whether there is room left to walk further out: a -100 dBm
         * beacon over a -120 dBm floor is a healthier link than a -80 dBm one
         * fighting a -85 dBm noise floor. */
        if (pkt.i16NoiseFloor == LORA_NOISE_FLOOR_INVALID)
        {
            DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                    "nf=n/a rx=%lu missed=%lu\r\n",
                    u32Id, u32Seq, pkt.rssi, pkt.snr, u32Rx, u32Missed);
        }
        else
        {
            DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                    "nf=%d dBm margin=%d dB rx=%lu missed=%lu\r\n",
                    u32Id, u32Seq, pkt.rssi, pkt.snr, pkt.i16NoiseFloor,
                    (int)(pkt.rssi - pkt.i16NoiseFloor), u32Rx, u32Missed);
        }
    }
}

#endif /* ENABLE_RADIO_TEST */
