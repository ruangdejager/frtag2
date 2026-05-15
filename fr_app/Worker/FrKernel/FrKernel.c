/*
 * FrKernel.c
 *
 * Device command interface Worker.
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
 *   tag -devicereq              — broadcast; every device replies with its ID
 *   UART:  tag <cmd>
 *   LoRa:  tag <ID> <cmd>       — only the addressed device replies
 *
 *   Supported <cmd>:
 *     -help               list all commands
 *     battery             battery voltage (mV)
 *     discovery schedule  wakeup interval (min)
 */

#include "FrKernel.h"
#include "FrKernel_Config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "cmsis_os2.h"
#include "Battery.h"
#include "MeshNetwork.h"
#include "LoraRadio.h"

#ifdef FRKERNEL_INTERFACE_UART
#  include "Debug.h"
#endif

/* -------------------------------------------------------------------------- */

static osThreadId_t s_taskHandle = NULL;

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

/* -------------------------------------------------------------------------- */

static void FRKERNEL_vRespond(const char *msg)
{
    uint16_t len = (uint16_t)strlen(msg);

#ifdef FRKERNEL_INTERFACE_UART
    DEBUG_vPutBuffer((const uint8_t *)msg, len);

#elif defined(FRKERNEL_INTERFACE_LORA)
    LoraRadio_Packet_t pkt = {0};
    pkt.buffer[0] = (uint8_t)MeshPktType_FrKernel;
    if (len > (uint16_t)(LORA_MAX_PACKET_SIZE - 1))
        len = (uint16_t)(LORA_MAX_PACKET_SIZE - 1);
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

    /* -devicereq: broadcast — every device responds regardless of addressing */
    if (strcmp(p, "-devicereq") == 0)
    {
        snprintf(resp, sizeof(resp), "Device ID: %04" PRIX32 "\r\n",
                 LORARADIO_u32GetUniqueId());
        FRKERNEL_vRespond(resp);
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

    if (strcmp(p, "-help") == 0)
    {
#ifdef FRKERNEL_INTERFACE_UART
        FRKERNEL_vRespond(
            "FrKernel commands:\r\n"
            "  tag -devicereq              this device's ID\r\n"
            "  tag -help                   list commands\r\n"
            "  tag battery                 battery voltage (mV)\r\n"
            "  tag discovery schedule      wakeup interval (min)\r\n"
        );
#else
        FRKERNEL_vRespond(
            "FrKernel commands:\r\n"
            "  tag -devicereq              discover all device IDs\r\n"
            "  tag <ID> -help              list commands\r\n"
            "  tag <ID> battery            battery voltage (mV)\r\n"
            "  tag <ID> discovery schedule wakeup interval (min)\r\n"
        );
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
        osThreadFlagsWait(0x01U, osFlagsWaitAny, osWaitForever);
        uint8_t byte;
        while (DEBUG_bReadByte(&byte))
        {
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
    }

#elif defined(FRKERNEL_INTERFACE_LORA)
    FrKernelPkt_t pkt;
    for (;;)
    {
        if (osMessageQueueGet(s_rxQueue, &pkt, NULL, osWaitForever) == osOK)
            FRKERNEL_vProcessCommand((const char *)pkt.data);
    }
#endif
}

/* -------------------------------------------------------------------------- */

void FRKERNEL_vInit(void)
{
#ifdef FRKERNEL_INTERFACE_LORA
    s_rxQueue = osMessageQueueNew(FRKERNEL_LORA_QUEUE_LEN, sizeof(FrKernelPkt_t), NULL);
    configASSERT(s_rxQueue != NULL);
#endif

    static const osThreadAttr_t attr = {
        .name       = "FrKernel",
        .stack_size = configMINIMAL_STACK_SIZE * 4U,
        .priority   = osPriorityLow,
    };
    s_taskHandle = osThreadNew(FRKERNEL_vTask, NULL, &attr);
    configASSERT(s_taskHandle != NULL);
}
