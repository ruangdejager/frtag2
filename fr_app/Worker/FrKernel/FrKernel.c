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
 *   auto-releases after FRKERNEL_INACTIVITY_TIMEOUT_MS of inactivity.
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
#include "DbgLog.h"
#ifdef STORAGE_BACKEND_MICROSD
#  include "AccLog.h"
#endif
#ifdef STORAGE_BACKEND_FLASH
#  include "OtaUpdate.h"
#endif

#if defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
#  include "Debug.h"
#endif

/* -------------------------------------------------------------------------- */

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
static void FRKERNEL_vRespond(const char *msg)
{
    /* Any TX extends an active session (covers bulk/slow data scenarios) */
    if (s_bConnected)
        s_u32LastCmdTick = osKernelGetTickCount();

    uint16_t len = (uint16_t)strlen(msg);

#ifdef FRKERNEL_INTERFACE_UART
    DEBUG_vPutBuffer((const uint8_t *)msg, len);

#elif defined(FRKERNEL_INTERFACE_LORA)
    LoraRadio_Packet_t pkt = {0};
    pkt.buffer[0] = (uint8_t)MeshPktType_FrKernel;
    /* Cap so type byte + payload + radio-appended CRC fit the uint8_t length
     * field (255): payload <= LORA_MAX_PACKET_SIZE - 3. The old "- 1" cap
     * made pkt.length wrap to 0 for max-length responses. */
    if (len > (uint16_t)(LORA_MAX_PACKET_SIZE - 3))
        len = (uint16_t)(LORA_MAX_PACKET_SIZE - 3);
    memcpy(&pkt.buffer[1], msg, len);
    pkt.length = (uint8_t)(len + 1U);
    LORARADIO_bTxPacket(&pkt);
#endif
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

static void FRKERNEL_vProcessCommand(const char *line)
{
    /* Must start with "tag" */
    if (strncmp(line, FRKERNEL_CMD_PREFIX, 3U) != 0) return;

    const char *p = line + 3U;
    if (*p == ' ') p++;

    char resp[FRKERNEL_RESP_BUF_LEN];

    /* -devicereq: broadcast — every device responds, no addressing check.
     * Does not start a session on LoRa (any device may respond to a scan). */
    if (strcmp(p, "-devicereq") == 0)
    {
        snprintf(resp, sizeof(resp), "Device ID: %04" PRIX32 "\r\n",
                 LORARADIO_u32GetUniqueId());
        FRKERNEL_vRespond(resp);
#ifdef FRKERNEL_INTERFACE_UART
        s_bConnected      = true;
        s_u32LastCmdTick  = osKernelGetTickCount();
#endif
        return;
    }

#ifdef FRKERNEL_INTERFACE_LORA
    /* All other LoRa commands must be addressed: "tag <XXXX> <cmd>" */
    char    *end;
    uint32_t targetId = (uint32_t)strtoul(p, &end, 16);
    if (end == p || *end != ' ') return;                    /* malformed */
    if (targetId != LORARADIO_u32GetUniqueId()) return;     /* not for us */
    p = end + 1;                                            /* skip "<ID> " */
#endif

    /* release: close the session so the device can enter deep sleep */
    if (strcmp(p, "release") == 0)
    {
        s_bConnected = false;
        FRKERNEL_vRespond("Released\r\n");
        return;
    }

    /* All other addressed commands: mark session active and update timer */
    s_bConnected     = true;
    s_u32LastCmdTick = osKernelGetTickCount();

    if (strcmp(p, "-help") == 0)
    {
#ifdef FRKERNEL_INTERFACE_UART
        FRKERNEL_vRespond(
            "FrKernel commands:\r\n"
            "  tag -devicereq              this device's ID\r\n"
            "  tag -help                   list commands\r\n"
            "  tag juice                   battery + solar panel voltage (mV)\r\n"
            "  tag discovery schedule      wakeup interval (min)\r\n"
            "  tag prodsleep               enter production sleep (secondary only)\r\n"
            "  tag release                 release device for sleep\r\n"
        );
#else
        FRKERNEL_vRespond(
            "FrKernel commands:\r\n"
            "  tag -devicereq              discover all device IDs\r\n"
            "  tag <ID> -help              list commands\r\n"
            "  tag <ID> juice              battery + solar panel voltage (mV)\r\n"
            "  tag <ID> discovery schedule wakeup interval (min)\r\n"
            "  tag <ID> prodsleep          enter production sleep (secondary only)\r\n"
            "  tag <ID> release            release device for sleep\r\n"
        );
#endif
#if defined(STORAGE_BACKEND_FLASH) && defined(FRKERNEL_INTERFACE_UART)
        FRKERNEL_vRespond(
            "  tag flash clear             erase ext-flash log\r\n"
            "  tag flash stream            stream ext-flash log\r\n"
            "  tag fwaccept [off]          arm/disarm firmware acceptance (secondary)\r\n"
            "  tag fwdistribute            distribute staged firmware (primary)\r\n");
#elif defined(STORAGE_BACKEND_FLASH)
        FRKERNEL_vRespond(
            "  tag <ID> flash clear        erase ext-flash log\r\n"
            "  tag <ID> flash stream       stream ext-flash log\r\n"
            "  tag <ID> fwaccept [off]     arm/disarm firmware acceptance (secondary)\r\n"
            "  tag <ID> fwdistribute       distribute staged firmware (primary)\r\n");
#endif
#if defined(STORAGE_BACKEND_MICROSD) && defined(FRKERNEL_INTERFACE_UART)
        FRKERNEL_vRespond(
            "  tag sd clear                wipe MicroSD (logs + acc)\r\n"
            "  tag sd log stream           stream MicroSD log\r\n");
#elif defined(STORAGE_BACKEND_MICROSD)
        FRKERNEL_vRespond(
            "  tag <ID> sd clear           wipe MicroSD (logs + acc)\r\n"
            "  tag <ID> sd log stream      stream MicroSD log\r\n");
#endif
    }
    else if (strcmp(p, "juice") == 0)
    {
        /* Solar getter is safe to call unconditionally: SolarPower is
         * secondary-only (primaries have no panel), and its getter is a
         * plain static-variable read that defaults to 0 if never inited. */
        snprintf(resp, sizeof(resp), "Battery: %u mV  Solar: %u mV\r\n",
                 BAT_u16GetVoltage(), SOLAR_u16GetVSolarMV());
        FRKERNEL_vRespond(resp);
    }
    else if (strcmp(p, "discovery schedule") == 0)
    {
        snprintf(resp, sizeof(resp), "Discovery interval: %u min\r\n",
                 MESHNETWORK_u8GetWakeupInterval());
        FRKERNEL_vRespond(resp);
    }
    else if (strcmp(p, "prodsleep") == 0)
    {
        DEVICE_DISCOVERY_vEnterProductionSleep();
        FRKERNEL_vRespond("ProductionSleep entered — wakes on Vsolar >= 3000 mV\r\n");

        /* prodsleep is a terminal command: don't linger in an open session
         * waiting for "tag release" or the 5-min inactivity timeout, or
         * DeviceDiscovery's deep-sleep tail blocks on FRKERNEL_bIsConnected()
         * for no reason. osDelay first so the response above — written
         * directly to the UART TX ring, interrupt-drained in the background —
         * actually leaves the wire before sleep can park the UART pins mid-
         * transmission (same "let the log line drain" pattern used before
         * OTASTORE_vArmBootloaderAndReset's reset). Then auto-release so
         * sleep proceeds immediately instead of opening a kernel window. */
        osDelay(100);
        s_bConnected = false;
    }
#ifdef STORAGE_BACKEND_FLASH
    else if (strcmp(p, "fwaccept") == 0)
    {
        /* Arm firmware acceptance (secondary only). The session stays open so
         * the secondary live-listens for the primary's OtaPrep from the
         * DeviceDiscovery hold-while-connected loop. */
        if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_SECONDARY)
        {
            FRKERNEL_vRespond("fwaccept: secondary only\r\n");
        }
        else
        {
            OTAUPDATE_vArmAcceptance();
            FRKERNEL_vRespond("Firmware acceptance ARMED - awaiting OtaPrep\r\n");
        }
    }
    else if (strcmp(p, "fwaccept off") == 0)
    {
        OTAUPDATE_vDisarmAcceptance();
        FRKERNEL_vRespond("Firmware acceptance disarmed\r\n");
    }
    else if (strcmp(p, "fwdistribute") == 0)
    {
        /* Request an on-demand distribution of the staged image (primary
         * only). The session keeps the primary awake past its campaign so the
         * hold-while-connected loop runs OTAUPDATE_vDistribute(). */
        if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY)
        {
            FRKERNEL_vRespond("fwdistribute: primary only\r\n");
        }
        else if (OTAUPDATE_bRequestDistribute())
        {
            FRKERNEL_vRespond("Firmware distribute requested\r\n");
        }
        else
        {
            FRKERNEL_vRespond("fwdistribute: no valid image staged\r\n");
        }
    }
    else if (strcmp(p, "flash clear") == 0)
    {
        /* Routed through the DbgLog consumer so it can't race log writes. */
        DBGLOG_vRequestErase();
        FRKERNEL_vRespond("Ext-flash log erase requested\r\n");
    }
    else if (strcmp(p, "flash stream") == 0)
    {
        /* consumer streams oldest -> newest, on whichever transport this
         * session arrived on -- a LoRa session has no UART to read a
         * UART-routed dump from. */
#ifdef FRKERNEL_INTERFACE_LORA
        DBGLOG_vRequestDumpVia(FRKERNEL_vLoraLogSink);
        FRKERNEL_vRespond("Streaming ext-flash log via LoRa...\r\n");
#else
        DBGLOG_vRequestDump();
        FRKERNEL_vRespond("Streaming ext-flash log...\r\n");
#endif
    }
#endif
#ifdef STORAGE_BACKEND_MICROSD
    else if (strcmp(p, "sd clear") == 0)
    {
        /* Wipe the whole card: log FIFO (via the consumer) + ACC write head
         * (deferred to the movement task). Both avoid racing their owners. */
        DBGLOG_vRequestErase();
        ACCLOG_vRequestErase();
        FRKERNEL_vRespond("MicroSD wipe requested (logs + acc data)\r\n");
    }
    else if (strcmp(p, "sd log stream") == 0)
    {
        DBGLOG_vRequestDump();   /* log region only; no acc-data stream by design */
        FRKERNEL_vRespond("Streaming MicroSD log...\r\n");
    }
#endif
    else
    {
        snprintf(resp, sizeof(resp), "Unknown: '%s'. Try 'tag -help'\r\n", p);
        FRKERNEL_vRespond(resp);
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
        osThreadFlagsSet(s_taskHandle, 0x01U);
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

#ifdef FRKERNEL_INTERFACE_UART
    for (;;)
    {
        /* Timed wait so the inactivity check runs even with no input */
        osThreadFlagsWait(0x01U, osFlagsWaitAny, FRKERNEL_POLL_INTERVAL_MS);

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
                    FRKERNEL_vProcessCommand(s_lineBuf);
                    s_lineIdx = 0U;
                }
            }
            else if (s_lineIdx < FRKERNEL_LINE_BUF_LEN - 1U)
            {
                s_lineBuf[s_lineIdx++] = (char)byte;
            }
        }

        /* Inactivity auto-release */
        if (s_bConnected &&
            (osKernelGetTickCount() - s_u32LastCmdTick) >= FRKERNEL_INACTIVITY_TIMEOUT_MS)
        {
            s_bConnected = false;
            FRKERNEL_vRespond("FrKernel: session timed out\r\n");
        }
    }

#elif defined(FRKERNEL_INTERFACE_LORA)
    FrKernelPkt_t pkt;
    for (;;)
    {
        /* Timed wait so the inactivity check runs even with no packets */
        if (osMessageQueueGet(s_rxQueue, &pkt, NULL, FRKERNEL_POLL_INTERVAL_MS) == osOK)
            FRKERNEL_vProcessCommand((const char *)pkt.data);

        /* Inactivity auto-release */
        if (s_bConnected &&
            (osKernelGetTickCount() - s_u32LastCmdTick) >= FRKERNEL_INACTIVITY_TIMEOUT_MS)
        {
            s_bConnected = false;
            FRKERNEL_vRespond("FrKernel: session timed out\r\n");
        }
    }

#elif defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
    /* Same UART line accumulation as the UART interface, but a completed
     * line is forwarded over LoRa instead of processed locally — no session/
     * inactivity tracking, there is nothing local for it to gate. */
    for (;;)
    {
        osThreadFlagsWait(0x01U, osFlagsWaitAny, osWaitForever);

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
