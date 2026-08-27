/*
 * FrKernel.c
 *
 * Device command interface Worker.
 *
 * Session management:
 *   Any addressed command marks the session "connected".  DeviceDiscovery
 *   checks FRKERNEL_bIsConnected() before entering deep sleep and waits
 *   until the session is released.  The user releases explicitly with
 *   "tag release" (or "tag <ID> release" on LoRa), or the session
 *   auto-releases after FRKERNEL_INACTIVITY_TIMEOUT_MS of inactivity. One
 *   shared session/timeout, not per-transport — see FrKernel_Config.h.
 *
 * UART path:
 *   UART2_vNotifyOnRX() (overrides weak HAL callback) → thread flag →
 *   drain ring buffer → accumulate line → parse on CR/LF →
 *   DEBUG_vPutBuffer() response.
 *
 * LoRa path:
 *   MESHNETWORK_vOnFrKernelPacket() (overrides weak MeshNetwork callback) →
 *   enqueue FrKernelPkt_t → parse → LORARADIO_bTxPacket() response.
 *
 * Both may be compiled in together (FRKERNEL_INTERFACE_UART and _LORA both
 * defined): FRKERNEL_vTask services either source as it arrives (a thread
 * flag per transport wakes it), and every command carries a FrKernelXport_e
 * tag through FRKERNEL_vProcessCommand/FRKERNEL_vRespond so the reply goes
 * back out the same transport the request came in on, and so LoRa's
 * mandatory "<ID>" address prefix is only required/parsed for LoRa-sourced
 * commands.
 *
 * LoRa-bridge path (bench test rig — see FrKernel_Config.h):
 *   Same UART line accumulation as the UART path, but a completed line is
 *   forwarded verbatim as a LoRa FrKernel packet instead of being parsed
 *   locally (FRKERNEL_vForwardLine). MESHNETWORK_vOnFrKernelPacket() prints
 *   whatever comes back straight to the debug UART instead of enqueueing it
 *   for local processing. No session/inactivity tracking — there is nothing
 *   local to time out.
 *
 * Commands:
 *   tag -devicereq              broadcast; every device replies with its ID
 *   UART:  tag <cmd>
 *   LoRa:  tag <ID> <cmd>       only the addressed device replies
 *   LoRa:  tag * <cmd>          bulk: every device runs the action command
 *                               silently (DBG_LOG only, no over-air reply —
 *                               see FRKERNEL_vAck). Query commands (-help,
 *                               juice, fwver, discovery schedule, flash
 *                               stream, sd log stream) are dropped under
 *                               "*" instead of flooding the air with every
 *                               device's answer — see FRKERNEL_bQueryOnlyCmd.
 *
 *   <cmd>:
 *     -help               list all commands
 *     battery             battery voltage (mV)
 *     discovery schedule  wakeup interval (min)
 *     release             release session; device may enter deep sleep
 */

#include "FrKernel.h"
#include "FrKernel_Config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Battery.h"
#include "SolarPower.h"
#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "DeviceDiscovery.h"

#include "build_config.h"
#include "version_config.h"
#include "DbgLog.h"
#include "SelfTest.h"
#include "RadioTestMode.h"     /* R&D radio link/range test (tag radiotest) */
#include "stm32wlxx_hal.h"     /* TAMP->BKP2R for the bootloader version    */
#ifdef STORAGE_BACKEND_MICROSD
#  include "AccLog.h"
#endif
#ifdef STORAGE_BACKEND_FLASH
#  include "Fota.h"
#endif

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
#  include "Debug.h"
#endif

/* -------------------------------------------------------------------------- */

#define FRKERNEL_FLAG_UART  0x01U
#define FRKERNEL_FLAG_LORA  0x02U

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA)
/* Which transport a command arrived on — threaded through
 * FRKERNEL_vProcessCommand/FRKERNEL_vRespond so the reply goes back out the
 * same side it came in on, and so LoRa's "<ID>" address prefix is only
 * required for LoRa-sourced commands. In a solo build only one value is
 * ever produced/consumed; the runtime branches on it still compile fine
 * (the other transport's actual I/O code is compiled out separately by its
 * own #ifdef, per function below). */
typedef enum {
    FRKERNEL_XPORT_UART,
    FRKERNEL_XPORT_LORA,
} FrKernelXport_e;
#endif

static osThreadId_t      s_taskHandle   = NULL;
static volatile bool     s_bConnected   = false;
static volatile uint32_t s_u32LastCmdTick = 0U;

#ifdef FRKERNEL_INTERFACE_LORA
typedef struct {
    uint8_t data[FRKERNEL_LINE_BUF_LEN];
    uint8_t len;
} FrKernelPkt_t;

static osMessageQueueId_t s_rxQueue = NULL;
#endif

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
static char    s_lineBuf[FRKERNEL_LINE_BUF_LEN];
static uint8_t s_lineIdx = 0U;
#endif

static void FRKERNEL_vTask(void *arg);

/* -------------------------------------------------------------------------- */

bool FRKERNEL_bIsConnected(void)
{
    return s_bConnected;
}

/* --------------------------------------------------------------------------
 * FRKERNEL_vInit
 * -------------------------------------------------------------------------- */
void FRKERNEL_vInit(void)
{
#ifdef FRKERNEL_INTERFACE_LORA
    s_rxQueue = osMessageQueueNew(FRKERNEL_LORA_QUEUE_LEN, sizeof(FrKernelPkt_t), NULL);
    configASSERT(s_rxQueue != NULL);
#endif

    static const osThreadAttr_t attr = {
        .name       = "FrKernel",
        /* CMSIS-RTOS v2 stack_size is BYTES; configMINIMAL_STACK_SIZE is a
         * WORD count, so every other task in this codebase multiplies by
         * sizeof(StackType_t). This one didn't -- it was actually allocating
         * 768 B, a quarter of the intended ~3 KB, which the deeper call
         * chain behind "tag prodsleep" (-> DEVICE_DISCOVERY_vEnterProductionSleep
         * -> DBG_LOG's timestamp formatting) finally overran. */
        .stack_size = configMINIMAL_STACK_SIZE * 6U * sizeof(StackType_t),
        .priority   = osPriorityLow,
    };
    s_taskHandle = osThreadNew(FRKERNEL_vTask, NULL, &attr);
    configASSERT(s_taskHandle != NULL);
}

/* -------------------------------------------------------------------------- */

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA)
static void FRKERNEL_vRespond(FrKernelXport_e eXport, const char *msg)
{
    /* Any TX extends an active session (covers bulk/slow data scenarios) */
    if (s_bConnected)
        s_u32LastCmdTick = osKernelGetTickCount();

    uint16_t len = (uint16_t)strlen(msg);

#ifdef FRKERNEL_INTERFACE_UART
    if (eXport == FRKERNEL_XPORT_UART)
    {
        DEBUG_vPutBuffer((const uint8_t *)msg, len);
        return;
    }
#endif
#ifdef FRKERNEL_INTERFACE_LORA
    if (eXport == FRKERNEL_XPORT_LORA)
    {
        LoraRadio_Packet_t pkt = {0};
        pkt.buffer[0] = (uint8_t)MeshPktType_FrKernel;
        /* Cap so type byte + payload + radio-appended CRC fit the uint8_t
         * length field (255): payload <= LORA_MAX_PACKET_SIZE - 3. The old
         * "- 1" cap made pkt.length wrap to 0 for max-length responses. */
        if (len > (uint16_t)(LORA_MAX_PACKET_SIZE - 3))
            len = (uint16_t)(LORA_MAX_PACKET_SIZE - 3);
        memcpy(&pkt.buffer[1], msg, len);
        pkt.length = (uint8_t)(len + 1U);
        LORARADIO_bTxPacket(&pkt);
        return;
    }
#endif
}

/* Acknowledge a command's result. Under a normal single-target "tag <ID>
 * <cmd>", behaves exactly like FRKERNEL_vRespond. Under a bulk "tag *
 * <cmd>" (bBulk == true), every addressed device would otherwise reply
 * over the air at once — collisions on a shared channel, and a flood the
 * operator didn't ask to receive N times. Instead just DBG_LOG the result
 * locally so it's visible on the device's own bench UART, and send
 * nothing over the radio. */
static void FRKERNEL_vAck(FrKernelXport_e eXport, bool bBulk, const char *msg)
{
    if (bBulk)
    {
        size_t len = strlen(msg);
        while (len > 0U && (msg[len - 1U] == '\r' || msg[len - 1U] == '\n'))
            len--;
        DBG_LOG("FrKernel bulk: %.*s\r\n", (int)len, msg);
        return;
    }
    FRKERNEL_vRespond(eXport, msg);
}

/* Commands that only report state (never change it). Excluded from bulk
 * "tag * <cmd>" addressing entirely — every device replying to a query at
 * once is exactly the over-air flood bulk mode exists to avoid, and unlike
 * action commands there's no local result worth DBG_LOG-only either (the
 * whole point of a query is the answer leaving the device). */
static bool FRKERNEL_bQueryOnlyCmd(const char *p)
{
    static const char *const apacQueries[] = {
        "-help", "juice", "fwver", "discovery schedule",
        "flash stream", "sd log stream",
        "selftest", "selftest gps", "selftest acc", "selftest flash",
    };
    for (size_t i = 0U; i < (sizeof(apacQueries) / sizeof(apacQueries[0])); i++)
        if (strcmp(p, apacQueries[i]) == 0)
            return true;
    return false;
}

#if defined(FRKERNEL_INTERFACE_LORA) && defined(STORAGE_BACKEND_FLASH)
/* DbgLog's dump sink for "tag flash stream" on the LoRa interface: there's no
 * UART session to read a UART-routed dump from, so the dump goes out as
 * FrKernel LoRa packets instead (same framing as FRKERNEL_vRespond's LoRa
 * branch, but with an explicit length rather than strlen — flash-log bytes
 * aren't guaranteed to be NUL-free). Runs on the DbgLog consumer task (see
 * DBGLOG_vRequestDumpVia), not this task.
 *
 * Sliced to the full per-packet payload (LORA_MAX_PACKET_SIZE - 3) rather
 * than whatever chunk size the log backend happens to hand in — a caller
 * chunk larger than that (e.g. a 512 B MicroSD block, on that backend) is
 * split across as many packets as it takes instead of being silently
 * truncated to the first slice.
 *
 * Paced with LORARADIO_bTxPacketWait instead of a fixed delay: it blocks
 * only when the TX queue is actually full, and unblocks the instant the
 * radio task makes room by picking up the packet ahead of it — i.e. paced
 * off real radio progress, not a guessed inter-packet gap. */
static void FRKERNEL_vLoraLogSink(const uint8_t *data, uint16_t len)
{
    const uint16_t maxPayload = (uint16_t)(LORA_MAX_PACKET_SIZE - 3);

    while (len > 0U)
    {
        uint16_t n = (len > maxPayload) ? maxPayload : len;

        LoraRadio_Packet_t pkt = {0};
        pkt.buffer[0] = (uint8_t)MeshPktType_FrKernel;
        memcpy(&pkt.buffer[1], data, n);
        pkt.length = (uint8_t)(n + 1U);
        LORARADIO_bTxPacketWait(&pkt, osWaitForever);

        /* Each outbound packet counts as session activity — keeps the
         * inactivity timer from firing during a long stream where no
         * incoming commands arrive (the stream is one-way outbound). */
        if (s_bConnected)
            s_u32LastCmdTick = osKernelGetTickCount();

        data += n;
        len  = (uint16_t)(len - n);
    }
}
#endif

/* -------------------------------------------------------------------------- */

static void FRKERNEL_vProcessCommand(FrKernelXport_e eXport, const char *line)
{
    /* Must start with "tag" */
    if (strncmp(line, FRKERNEL_CMD_PREFIX, 3U) != 0) return;

    const char *p = line + 3U;
    if (*p == ' ') p++;

    char resp[FRKERNEL_RESP_BUF_LEN];
    bool bBulk = false;    /* set below for LoRa "tag * <cmd>"; always
                             * declared so FRKERNEL_vAck has a uniform call
                             * signature regardless of which transports are
                             * compiled in. */

    /* -devicereq: broadcast — every device responds, no addressing check.
     * Does not start a session over LoRa (any device may respond to a
     * scan); over UART there's exactly one device on the wire, so it's
     * effectively addressed already. */
    if (strcmp(p, "-devicereq") == 0)
    {
        snprintf(resp, sizeof(resp), "Device ID: %04" PRIX32 "\r\n",
                 LORARADIO_u32GetUniqueId());
#ifdef FRKERNEL_INTERFACE_LORA
        /* Collision avoidance: on a broadcast scan EVERY listening device
         * hears this at the same instant and would otherwise TX its reply
         * simultaneously — on a shared channel they collide and the kernel
         * decodes at most one. Spread the replies uniformly across a 5 s
         * window so they land at different times. rand() is seeded per
         * device from its unique id (DEVICE_DISCOVERY_vInit), so two
         * devices pick different offsets. Runs on the FrKernel task, so
         * blocking here is fine; a scan has nothing else time-critical.
         * UART is a single device on the wire — no spread needed there. */
        if (eXport == FRKERNEL_XPORT_LORA)
        {
            uint32_t u32JitterMs = (uint32_t)(rand() % 5000);
            osDelay(u32JitterMs);
        }
#endif
        FRKERNEL_vRespond(eXport, resp);
        /* Open a session for the responder — same for LoRa as for UART.
         * The comment above ("does not start a session over LoRa") was
         * wrong in practice: without holding the device awake past this
         * reply, the AppTask's kernel-wake window (see
         * DEVICE_DISCOVERY_KERNEL_WAKEUP_WINDOW_MS) can expire and the
         * radio drops to deep sleep before an operator's follow-up
         * "tag <ID> <cmd>" arrives. The user then sees "-devicereq
         * silently stopped working". FrKernel's own 5 min inactivity
         * timeout (FRKERNEL_INACTIVITY_TIMEOUT_MS) is the real "close
         * this session" gate on both transports. */
        s_bConnected      = true;
        s_u32LastCmdTick  = osKernelGetTickCount();
        return;
    }

#ifdef FRKERNEL_INTERFACE_LORA
    /* LoRa commands must be addressed: "tag <XXXX> <cmd>" or the bulk
     * wildcard "tag * <cmd>" (every device runs it, no over-air reply —
     * see FRKERNEL_vAck). UART commands (eXport == FRKERNEL_XPORT_UART)
     * skip this — there's only one device on the wire, no address needed. */
    if (eXport == FRKERNEL_XPORT_LORA)
    {
        if (p[0] == '*' && p[1] == ' ')
        {
            bBulk = true;
            p += 2;
        }
        else
        {
            char    *end;
            uint32_t targetId = (uint32_t)strtoul(p, &end, 16);
            if (end == p || *end != ' ') return;                /* malformed */
            if (targetId != LORARADIO_u32GetUniqueId()) return;  /* not for us */
            p = end + 1;                                         /* skip "<ID> " */
        }

        /* Queries never run under a bulk address — every device replying
         * at once is the exact flood bulk mode exists to avoid, and
         * there's no local-only result worth DBG_LOG-ing either. */
        if (bBulk && FRKERNEL_bQueryOnlyCmd(p))
            return;
    }
#endif

    /* release: close the session so the device can enter deep sleep */
    if (strcmp(p, "release") == 0)
    {
        s_bConnected = false;
        FRKERNEL_vAck(eXport, bBulk, "Released\r\n");
        return;
    }

    /* All other addressed commands: mark session active and update timer */
    s_bConnected     = true;
    s_u32LastCmdTick = osKernelGetTickCount();

    if (strcmp(p, "-help") == 0)
    {
        bool bAddressed = true;
#ifdef FRKERNEL_INTERFACE_UART
        if (eXport == FRKERNEL_XPORT_UART)
        {
            bAddressed = false;
            FRKERNEL_vRespond(eXport,
                "FrKernel commands:\r\n"
                "  tag -devicereq              this device's ID\r\n"
                "  tag -help                   list commands\r\n"
                "  tag fwver                   app + bootloader version\r\n"
                "  tag juice                   battery + solar panel voltage (mV)\r\n"
                "  tag selftest                boot-time gps/acc/flash results\r\n"
                "  tag selftest gps|acc|flash  single boot-time result\r\n"
                "  tag discovery schedule      wakeup interval (min)\r\n"
                "  tag discovery schedule <N>  set wakeup interval (15/30/60/120/240 min)\r\n"
                "  tag prodsleep               enter production sleep (secondary only)\r\n"
                "  tag solarsleep              as prodsleep, shake-to-wake only\r\n"
                "  tag radiotest               R&D range test (2nd: beacon+shake to exit,\r\n"
                "                              1st: listen+log to fr9)\r\n"
                "  tag radio stop              end the range test (primary)\r\n"
                "  tag release                 release device for sleep\r\n"
            );
        }
#endif
        if (bAddressed)
        {
            FRKERNEL_vRespond(eXport,
                "FrKernel commands:\r\n"
                "  tag -devicereq              discover all device IDs\r\n"
                "  tag <ID> -help              list commands\r\n"
                "  tag <ID> fwver              app + bootloader version\r\n"
                "  tag <ID> juice              battery + solar panel voltage (mV)\r\n"
                "  tag <ID> selftest           boot-time gps/acc/flash results\r\n"
                "  tag <ID> selftest gps|acc|flash  single boot-time result\r\n"
                "  tag <ID> discovery schedule wakeup interval (min)\r\n"
                "  tag <ID> discovery schedule <N>  set wakeup interval (15/30/60/120/240)\r\n"
                "  tag <ID> prodsleep          enter production sleep (secondary only)\r\n"
                "  tag <ID> solarsleep         as prodsleep, shake-to-wake only\r\n"
                "  tag <ID> radiotest          R&D range test (2nd: beacon+shake to exit,\r\n"
                "                              1st: listen+log to fr9)\r\n"
                "  tag <ID> radio stop         end the range test (primary)\r\n"
                "  tag <ID> release            release device for sleep\r\n"
                "  tag * <cmd>                 bulk: every device runs <cmd> silently\r\n"
                "                              (DBG_LOG only, no reply; action cmds only)\r\n"
            );
        }
#ifdef STORAGE_BACKEND_FLASH
        if (!bAddressed)
        {
            FRKERNEL_vRespond(eXport,
                "  tag flash clear             erase ext-flash log\r\n"
                "  tag flash eraseall           erase entire ext-flash (log + staged OTA image)\r\n"
                "  tag flash stream            stream ext-flash log\r\n"
                "  tag fwaccept [off]          arm/disarm firmware acceptance (secondary)\r\n"
                "  tag fwdistribute            distribute staged firmware (primary)\r\n");
        }
        else
        {
            FRKERNEL_vRespond(eXport,
                "  tag <ID> flash clear        erase ext-flash log\r\n"
                "  tag <ID> flash eraseall     erase entire ext-flash (log + staged OTA image)\r\n"
                "  tag <ID> flash stream       stream ext-flash log\r\n"
                "  tag <ID> fwaccept [off]     arm/disarm firmware acceptance (secondary)\r\n"
                "  tag <ID> fwdistribute       distribute staged firmware (primary)\r\n");
        }
#endif
#ifdef STORAGE_BACKEND_MICROSD
        if (!bAddressed)
        {
            FRKERNEL_vRespond(eXport,
                "  tag sd clear                wipe MicroSD (logs + acc)\r\n"
                "  tag sd log stream           stream MicroSD log\r\n");
        }
        else
        {
            FRKERNEL_vRespond(eXport,
                "  tag <ID> sd clear           wipe MicroSD (logs + acc)\r\n"
                "  tag <ID> sd log stream      stream MicroSD log\r\n");
        }
#endif
    }
    else if (strcmp(p, "juice") == 0)
    {
        /* Solar getter is safe to call unconditionally: SolarPower is
         * secondary-only (primaries have no panel), and its getter is a
         * plain static-variable read that defaults to 0 if never inited. */
        snprintf(resp, sizeof(resp), "Battery: %u mV  Solar: %u mV\r\n",
                 BAT_u16GetVoltage(), SOLAR_u16GetVSolarMV());
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strcmp(p, "fwver") == 0)
    {
        /* App version comes straight from version_config.h (single source
         * of truth for the build). Bootloader version comes from
         * TAMP->BKP2R (the bootloader latches it on every boot per
         * Fota_Config.h's backup-register map). */
        snprintf(resp, sizeof(resp), "App: v%u.%u.%u  Bootloader: v%lu\r\n",
                 (unsigned)VERSION_SW_MAJOR,
                 (unsigned)VERSION_SW_MINOR,
                 (unsigned)VERSION_SW_PATCH,
                 (unsigned long)TAMP->BKP2R);
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strcmp(p, "selftest") == 0)
    {
        /* Answers from the memoized boot-time run (see SELFTEST_vRunAndReport
         * called from INIT_vInitialization). "n/a" is reported for tests
         * that don't apply on this build/role (GPS on primary, flash under
         * MicroSD backend) — no re-run happens here. */
        snprintf(resp, sizeof(resp), "selftest: gps=%s acc=%s flash=%s\r\n",
                 SELFTEST_bGpsApplicable()   ? (SELFTEST_bGpsOk()   ? "OK" : "FAIL") : "n/a",
                 SELFTEST_bAccOk()           ? "OK" : "FAIL",
                 SELFTEST_bFlashApplicable() ? (SELFTEST_bFlashOk() ? "OK" : "FAIL") : "n/a");
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strcmp(p, "selftest gps") == 0)
    {
        snprintf(resp, sizeof(resp), "selftest gps: %s\r\n",
                 SELFTEST_bGpsApplicable() ? (SELFTEST_bGpsOk() ? "OK" : "FAIL") : "n/a");
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strcmp(p, "selftest acc") == 0)
    {
        snprintf(resp, sizeof(resp), "selftest acc: %s\r\n",
                 SELFTEST_bAccOk() ? "OK" : "FAIL");
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strcmp(p, "selftest flash") == 0)
    {
        snprintf(resp, sizeof(resp), "selftest flash: %s\r\n",
                 SELFTEST_bFlashApplicable() ? (SELFTEST_bFlashOk() ? "OK" : "FAIL") : "n/a");
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strncmp(p, "discovery schedule ", 19) == 0)
    {
        /* Action variant: "discovery schedule <N>" where N is minutes.
         * Bulk-capable ("tag * discovery schedule 60") — every listening
         * device applies the interval locally and DBG_LOGs the result
         * silently (no over-air reply, see FRKERNEL_vAck). Single-target
         * ("tag <ID> discovery schedule 60") also works but note the
         * value will be overwritten by the primary's next TimeSync,
         * which broadcasts whatever interval the PRIMARY holds — for a
         * persistent change, bulk the whole fleet (or set fr9's own
         * minWindow so the primary re-broadcasts it).
         *
         * Values must be from the fixed set of WakeupInterval enum
         * mappings (15/30/60/120/240 min); anything else is rejected. */
        uint32_t u32Mins = (uint32_t)strtoul(p + 19, NULL, 10);
        WakeupInterval eNew = WAKEUP_INTERVAL_MAX_COUNT;   /* sentinel */
        if      (u32Mins == 15U)  eNew = WAKEUP_INTERVAL_15_MIN;
        else if (u32Mins == 30U)  eNew = WAKEUP_INTERVAL_30_MIN;
        else if (u32Mins == 60U)  eNew = WAKEUP_INTERVAL_60_MIN;
        else if (u32Mins == 120U) eNew = WAKEUP_INTERVAL_120_MIN;
        else if (u32Mins == 240U) eNew = WAKEUP_INTERVAL_240_MIN;

        if (eNew != WAKEUP_INTERVAL_MAX_COUNT)
        {
            MESHNETWORK_vSetWakeupInterval(eNew);
            snprintf(resp, sizeof(resp), "Discovery interval set to %u min\r\n",
                     (unsigned)u32Mins);
        }
        else
        {
            snprintf(resp, sizeof(resp), "Invalid interval '%lu' - allowed: 15,30,60,120,240\r\n",
                     (unsigned long)u32Mins);
        }
        FRKERNEL_vAck(eXport, bBulk, resp);
    }
    else if (strcmp(p, "discovery schedule") == 0)
    {
        snprintf(resp, sizeof(resp), "Discovery interval: %u min\r\n",
                 MESHNETWORK_u8GetWakeupInterval());
        FRKERNEL_vRespond(eXport, resp);
    }
    else if (strcmp(p, "prodsleep") == 0)
    {
        DEVICE_DISCOVERY_vEnterProductionSleep();
        FRKERNEL_vAck(eXport, bBulk, "ProductionSleep entered — wakes on Vsolar >= 3000 mV\r\n");

        /* prodsleep is a terminal command: don't linger in an open session
         * waiting for "tag release" or the 5-min inactivity timeout, or
         * DeviceDiscovery's deep-sleep tail blocks on FRKERNEL_bIsConnected()
         * for no reason. osDelay first so the response above — written
         * directly to the UART TX ring, interrupt-drained in the background —
         * actually leaves the wire before sleep can park the UART pins mid-
         * transmission (same "let the log line drain" pattern used before
         * FOTA_vArmBootloaderAndReset's reset). Then auto-release so
         * sleep proceeds immediately instead of opening a kernel window. */
        osDelay(100);
        s_bConnected = false;
    }
    else if (strcmp(p, "solarsleep") == 0)
    {
        /* Same super-deep sleep as prodsleep, minus the solar wake: only the
         * shake sequence brings it back. For leaving a flat unit to charge
         * with everything disabled — a rising panel voltage would otherwise
         * wake it the moment the sun came up, which is exactly what we don't
         * want while trying to put charge INTO the battery. */
        DEVICE_DISCOVERY_vEnterSolarSleep();
        FRKERNEL_vAck(eXport, bBulk, "SolarSleep entered — shake-to-wake only\r\n");

        /* Terminal command, same as prodsleep above: let the response drain
         * off the wire, then auto-release so sleep proceeds immediately
         * instead of holding an open kernel window. */
        osDelay(100);
        s_bConnected = false;
    }
    else if (strcmp(p, "radiotest") == 0)
    {
        /* R&D radio link/range test. Role decides what the unit does:
         *   secondary -> beacons every 5 s and goes deaf to all radio traffic,
         *                including this kernel. Shake-to-wake is the only way
         *                back, exactly as for solarsleep.
         *   primary   -> listens, logs RSSI/SNR/noise floor per beacon and
         *                forwards them to the fr9. Stays reachable, so
         *                "tag <ID> radio stop" can end it over the air.
         *
         * Entered BEFORE the ack so the reply reflects what actually happened:
         * the entry flushes the radio TX queue, which would otherwise discard
         * an ack queued ahead of it. TX still works once the mode is up — it
         * is only the receive path a secondary closes. */

        /* Never over a broadcast. This is an action command, so without this
         * check "tag * radiotest" would put EVERY secondary in earshot into
         * beacon mode at once — each one deaf to the radio from that moment,
         * including to this kernel, and recoverable only by physically
         * shaking it. On a deployed herd that means finding and handling
         * every animal. A range test is a two-unit bench/field procedure and
         * has no broadcast form worth having; refuse rather than offer one.
         *
         * Guarded here rather than by listing it in apacQueries[]: that would
         * make bulk drop it silently, and silence is the wrong answer to a
         * command that would have been this destructive. */
        if (bBulk)
        {
            DBG_LOG("FrKernel: refusing bulk 'radiotest' — address a single device\r\n");
        }
        else if (RADIOTESTMODE_bEnter())
        {
            FRKERNEL_vAck(eXport, bBulk,
                          RADIOTESTMODE_bIsListener()
                          ? "RadioTest: listening — end with 'radio stop'\r\n"
                          : "RadioTest: beaconing every 5 s — shake to exit\r\n");
        }
        else
        {
            /* Include the reason: this ack is very likely the ONLY thing a
             * remote operator ever sees of this failure (the detail behind it
             * goes to DBG_LOG on the unit itself, over UART nobody out here
             * can read). A bare "could not enter" gives a field failure
             * nothing to act on. */
            snprintf(resp, sizeof(resp), "RadioTest: could not enter (%s)\r\n",
                    RADIOTESTMODE_pcLastEnterError());
            FRKERNEL_vAck(eXport, bBulk, resp);
        }

        /* Terminal, same reasoning as prodsleep: drain the reply, then drop
         * the session rather than hold a kernel window open for a mode that
         * manages its own sleep lock. */
        osDelay(100);
        s_bConnected = false;
    }
    else if (strcmp(p, "radio stop") == 0)
    {
        /* The listening primary's way out. A beaconing secondary never gets
         * here over LoRa (it drops all inbound packets by design), but the
         * command still works on it over the debug UART. */
        if (RADIOTESTMODE_bActive())
        {
            RADIOTESTMODE_vExit("radio stop");
            FRKERNEL_vAck(eXport, bBulk, "RadioTest: stopped\r\n");
        }
        else
        {
            FRKERNEL_vAck(eXport, bBulk, "RadioTest: not running\r\n");
        }
    }
#ifdef STORAGE_BACKEND_FLASH
    else if (strcmp(p, "fwaccept") == 0)
    {
        /* Arm firmware acceptance (secondary only). The session stays open so
         * the secondary live-listens for the primary's OtaPrep from the
         * DeviceDiscovery hold-while-connected loop. */
        if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_SECONDARY)
        {
            FRKERNEL_vAck(eXport, bBulk, "fwaccept: secondary only\r\n");
        }
        else
        {
            FOTA_vArmAcceptanceKernel();
            FRKERNEL_vAck(eXport, bBulk, "Firmware acceptance ARMED - awaiting OtaPrep\r\n");
        }
    }
    else if (strcmp(p, "fwaccept off") == 0)
    {
        FOTA_vDisarmAcceptance();
        FRKERNEL_vAck(eXport, bBulk, "Firmware acceptance disarmed\r\n");
    }
    else if (strcmp(p, "fwdistribute") == 0)
    {
        /* Request an on-demand distribution of the staged image (primary
         * only). The session keeps the primary awake past its campaign so the
         * hold-while-connected loop runs FOTA_vDistribute(). */
        if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY)
        {
            FRKERNEL_vAck(eXport, bBulk, "fwdistribute: primary only\r\n");
        }
        else if (FOTA_bRequestDistribute())
        {
            FRKERNEL_vAck(eXport, bBulk, "Firmware distribute requested\r\n");
        }
        else
        {
            FRKERNEL_vAck(eXport, bBulk, "fwdistribute: no valid image staged\r\n");
        }
    }
    else if (strcmp(p, "flash clear") == 0)
    {
        /* Routed through the DbgLog consumer so it can't race log writes. */
        DBGLOG_vRequestErase();
        FRKERNEL_vAck(eXport, bBulk, "Ext-flash log erase requested\r\n");
    }
    else if (strcmp(p, "flash eraseall") == 0)
    {
        /* Whole-chip erase: log AND the OTA image scratchpad/metadata below
         * it. Destroys any staged-but-not-yet-installed OTA image along
         * with the log — deliberate full field-unit reset, not routine
         * maintenance, hence the distinct command name from "flash clear". */
        DBGLOG_vRequestEraseAll();
        FRKERNEL_vAck(eXport, bBulk, "Full ext-flash erase requested (log + staged OTA image)\r\n");
    }
    else if (strcmp(p, "flash stream") == 0)
    {
        /* consumer streams oldest -> newest, on whichever transport THIS
         * command arrived on -- a LoRa command has no UART session to read
         * a UART-routed dump from. */
#ifdef FRKERNEL_INTERFACE_LORA
        if (eXport == FRKERNEL_XPORT_LORA)
        {
            DBGLOG_vRequestDumpVia(FRKERNEL_vLoraLogSink);
            FRKERNEL_vRespond(eXport, "Streaming ext-flash log via LoRa...\r\n");
        }
        else
#endif
        {
            DBGLOG_vRequestDump();
            FRKERNEL_vRespond(eXport, "Streaming ext-flash log...\r\n");
        }
    }
#endif
#ifdef STORAGE_BACKEND_MICROSD
    else if (strcmp(p, "sd clear") == 0)
    {
        /* Wipe the whole card: log FIFO (via the consumer) + ACC write head
         * (deferred to the movement task). Both avoid racing their owners. */
        DBGLOG_vRequestErase();
        ACCLOG_vRequestErase();
        FRKERNEL_vAck(eXport, bBulk, "MicroSD wipe requested (logs + acc data)\r\n");
    }
    else if (strcmp(p, "sd log stream") == 0)
    {
        DBGLOG_vRequestDump();   /* log region only; no acc-data stream by design */
        FRKERNEL_vRespond(eXport, "Streaming MicroSD log...\r\n");
    }
#endif
    else
    {
        snprintf(resp, sizeof(resp), "Unknown: '%s'. Try 'tag -help'\r\n", p);
        FRKERNEL_vAck(eXport, bBulk, resp);
    }
}
#endif /* FRKERNEL_INTERFACE_UART || FRKERNEL_INTERFACE_LORA */

#ifdef FRKERNEL_INTERFACE_LORA_BRIDGE
/* Forward a UART-typed line verbatim as a LoRa FrKernel command packet —
 * whatever the user types (including the "tag" prefix and, for an addressed
 * command, the target "<ID>") goes out exactly as a real UART-attached
 * secondary would need to receive it. No local parsing: this device isn't
 * the one answering, it's just the human's radio-shaped keyboard. */
static void FRKERNEL_vForwardLine(const char *line)
{
    uint16_t len = (uint16_t)strlen(line);
    if (len == 0U) return;

    LoraRadio_Packet_t pkt = {0};
    pkt.buffer[0] = (uint8_t)MeshPktType_FrKernel;
    /* Cap so type byte + payload + radio-appended CRC fit the uint8_t length
     * field (255) — same bound as FRKERNEL_vRespond's LoRa framing. */
    if (len > (uint16_t)(LORA_MAX_PACKET_SIZE - 3))
        len = (uint16_t)(LORA_MAX_PACKET_SIZE - 3);
    memcpy(&pkt.buffer[1], line, len);
    pkt.length = (uint8_t)(len + 1U);

    char echo[8 + FRKERNEL_LINE_BUF_LEN];
    int  n = snprintf(echo, sizeof(echo), "--> %s\r\n", line);
    if (n > 0)
        DEBUG_vPutBuffer((const uint8_t *)echo, (uint16_t)n);

    if (!LORARADIO_bTxPacket(&pkt))
        DEBUG_vPutBuffer((const uint8_t *)"(TX queue full, dropped)\r\n", 27U);
}
#endif /* FRKERNEL_INTERFACE_LORA_BRIDGE */

/* --------------------------------------------------------------------------
 * Transport-specific hooks
 * -------------------------------------------------------------------------- */

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
/* Override the weak HAL callback — called from USART2 ISR */
void UART2_vNotifyOnRX(void)
{
    if (s_taskHandle != NULL)
        osThreadFlagsSet(s_taskHandle, FRKERNEL_FLAG_UART);
}
#endif

#ifdef FRKERNEL_INTERFACE_LORA
/* Override the weak MeshNetwork callback — called from parser task */
void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len)
{
    if (s_rxQueue == NULL || len == 0U) return;
    FrKernelPkt_t pkt;
    pkt.len = (len < (uint8_t)sizeof(pkt.data)) ? len : (uint8_t)(sizeof(pkt.data) - 1U);
    memcpy(pkt.data, buf, pkt.len);
    pkt.data[pkt.len] = '\0';
    osMessageQueuePut(s_rxQueue, &pkt, 0U, 0U);    /* non-blocking, ISR-safe */
    if (s_taskHandle != NULL)
        osThreadFlagsSet(s_taskHandle, FRKERNEL_FLAG_LORA);
}
#elif defined(FRKERNEL_INTERFACE_LORA_BRIDGE)

/* Substring search over a non-NUL-terminated FrKernel payload. */
static bool FRKERNEL_bBufContains(const uint8_t *buf, uint8_t len, const char *needle)
{
    size_t needleLen = strlen(needle);
    if (needleLen == 0U || (size_t)len < needleLen) return false;
    size_t last = (size_t)len - needleLen;
    for (size_t i = 0U; i <= last; i++)
        if (memcmp(&buf[i], needle, needleLen) == 0) return true;
    return false;
}

static bool s_bLogStreaming = false;

/* Override the weak MeshNetwork callback — called from parser task context
 * (not an ISR), so writing straight to the debug UART here is safe. Normally
 * prints "<-- <payload>" (the plain command/response case). While a "tag
 * flash stream" dump is in flight -- bracketed by LOG_vStreamViaSink's
 * "...LOG DUMP (...)" / "...LOG DUMP END" header/footer lines, relayed here
 * like any other FrKernel packet -- switch to passing the raw payload
 * straight through with no "<-- "/"\r\n" framing, and ask the 1 Hz heartbeat
 * marker (platform.c) to hold off, so what lands on the terminal is the log
 * itself instead of the relay's usual chrome interleaved with it. Both
 * markers are matched as substrings, but in practice each always arrives
 * whole in a single small packet (LOG_vStreamViaSink emits them as one
 * sub-64-byte sink call each, well under one LoRa payload). */
void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len)
{
    if (len == 0U) return;

    if (!s_bLogStreaming && FRKERNEL_bBufContains(buf, len, "LOG DUMP ("))
    {
        s_bLogStreaming = true;
        DEBUG_vSetQuiet(true);
    }

    if (s_bLogStreaming)
    {
        DEBUG_vPutBuffer(buf, len);
    }
    else
    {
        DEBUG_vPutBuffer((const uint8_t *)"<-- ", 4U);
        DEBUG_vPutBuffer(buf, len);
        DEBUG_vPutBuffer((const uint8_t *)"\r\n", 2U);
    }

    if (s_bLogStreaming && FRKERNEL_bBufContains(buf, len, "LOG DUMP END"))
    {
        s_bLogStreaming = false;
        DEBUG_vSetQuiet(false);
    }
}
#endif

/* --------------------------------------------------------------------------
 * Main task
 * -------------------------------------------------------------------------- */

static void FRKERNEL_vTask(void *arg)
{
    (void)arg;

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA)
    for (;;)
    {
        /* Wakes on either transport's flag; timed out so the inactivity
         * check still runs during a quiet period on both. In a solo build
         * only one flag is ever set (the other transport's ISR/callback
         * that would set it isn't compiled in), so this behaves exactly
         * like the old single-transport wait. */
        osThreadFlagsWait(FRKERNEL_FLAG_UART | FRKERNEL_FLAG_LORA,
                          osFlagsWaitAny, FRKERNEL_POLL_INTERVAL_MS);

#ifdef FRKERNEL_INTERFACE_UART
        uint8_t byte;
        while (DEBUG_bReadByte(&byte))
        {
            /* Any received byte extends an active session */
            if (s_bConnected)
                s_u32LastCmdTick = osKernelGetTickCount();

            if (byte == '\r' || byte == '\n')
            {
                if (s_lineIdx > 0U)
                {
                    s_lineBuf[s_lineIdx] = '\0';
                    FRKERNEL_vProcessCommand(FRKERNEL_XPORT_UART, s_lineBuf);
                    s_lineIdx = 0U;
                }
            }
            else if (s_lineIdx < FRKERNEL_LINE_BUF_LEN - 1U)
            {
                s_lineBuf[s_lineIdx++] = (char)byte;
            }
        }
#endif

#ifdef FRKERNEL_INTERFACE_LORA
        FrKernelPkt_t pkt;
        while (osMessageQueueGet(s_rxQueue, &pkt, NULL, 0U) == osOK)
            FRKERNEL_vProcessCommand(FRKERNEL_XPORT_LORA, (const char *)pkt.data);
#endif

        /* Inactivity auto-release. Notify on every compiled-in transport —
         * this is a rare, low-frequency event (once per idle session), not
         * per-packet traffic, so announcing it on both sides in a dual
         * build is harmless and keeps either terminal informed. */
        if (s_bConnected &&
            (osKernelGetTickCount() - s_u32LastCmdTick) >= FRKERNEL_INACTIVITY_TIMEOUT_MS)
        {
            s_bConnected = false;
#ifdef FRKERNEL_INTERFACE_UART
            FRKERNEL_vRespond(FRKERNEL_XPORT_UART, "FrKernel: session timed out\r\n");
#endif
#ifdef FRKERNEL_INTERFACE_LORA
            FRKERNEL_vRespond(FRKERNEL_XPORT_LORA, "FrKernel: session timed out\r\n");
#endif
        }
    }

#elif defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
    /* Same UART line accumulation as the UART interface, but a completed
     * line is forwarded over LoRa instead of processed locally — no session/
     * inactivity tracking, there is nothing local for it to gate. */
    for (;;)
    {
        osThreadFlagsWait(FRKERNEL_FLAG_UART, osFlagsWaitAny, osWaitForever);

        uint8_t byte;
        while (DEBUG_bReadByte(&byte))
        {
            if (byte == '\r' || byte == '\n')
            {
                if (s_lineIdx > 0U)
                {
                    s_lineBuf[s_lineIdx] = '\0';
                    FRKERNEL_vForwardLine(s_lineBuf);
                    s_lineIdx = 0U;
                }
            }
            else if (s_lineIdx < FRKERNEL_LINE_BUF_LEN - 1U)
            {
                s_lineBuf[s_lineIdx++] = (char)byte;
            }
        }
    }
#endif
}
