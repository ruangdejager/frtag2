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
#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "DeviceDiscovery.h"

#include "storage_config.h"
#include "DbgLog.h"
#ifdef STORAGE_BACKEND_MICROSD
#  include "AccLog.h"
#endif

#ifdef FRKERNEL_INTERFACE_UART
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

#ifdef FRKERNEL_INTERFACE_UART
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
        .stack_size = configMINIMAL_STACK_SIZE * 6U,
        .priority   = osPriorityLow,
    };
    s_taskHandle = osThreadNew(FRKERNEL_vTask, NULL, &attr);
    configASSERT(s_taskHandle != NULL);
}

/* -------------------------------------------------------------------------- */

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
            "  tag battery                 battery voltage (mV)\r\n"
            "  tag discovery schedule      wakeup interval (min)\r\n"
            "  tag prodsleep               enter production sleep (secondary only)\r\n"
            "  tag release                 release device for sleep\r\n"
        );
#else
        FRKERNEL_vRespond(
            "FrKernel commands:\r\n"
            "  tag -devicereq              discover all device IDs\r\n"
            "  tag <ID> -help              list commands\r\n"
            "  tag <ID> battery            battery voltage (mV)\r\n"
            "  tag <ID> discovery schedule wakeup interval (min)\r\n"
            "  tag <ID> prodsleep          enter production sleep (secondary only)\r\n"
            "  tag <ID> release            release device for sleep\r\n"
        );
#endif
#if defined(STORAGE_BACKEND_FLASH) && defined(FRKERNEL_INTERFACE_UART)
        FRKERNEL_vRespond(
            "  tag flash clear             erase ext-flash log\r\n"
            "  tag flash stream            stream ext-flash log\r\n");
#elif defined(STORAGE_BACKEND_FLASH)
        FRKERNEL_vRespond(
            "  tag <ID> flash clear        erase ext-flash log\r\n"
            "  tag <ID> flash stream       stream ext-flash log\r\n");
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
    else if (strcmp(p, "battery") == 0)
    {
        snprintf(resp, sizeof(resp), "Battery: %u mV\r\n", BAT_u16GetVoltage());
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
    }
#ifdef STORAGE_BACKEND_FLASH
    else if (strcmp(p, "flash clear") == 0)
    {
        /* Routed through the DbgLog consumer so it can't race log writes. */
        DBGLOG_vRequestErase();
        FRKERNEL_vRespond("Ext-flash log erase requested\r\n");
    }
    else if (strcmp(p, "flash stream") == 0)
    {
        DBGLOG_vRequestDump();   /* consumer streams oldest -> newest */
        FRKERNEL_vRespond("Streaming ext-flash log...\r\n");
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

/* --------------------------------------------------------------------------
 * Transport-specific hooks
 * -------------------------------------------------------------------------- */

#ifdef FRKERNEL_INTERFACE_UART
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
#endif
}
