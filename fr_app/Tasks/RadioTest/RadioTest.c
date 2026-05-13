/*
 * RadioTest.c
 *
 * Radio smoke-test task — compiled only when ENABLE_RADIO_TEST is defined.
 *
 * Transmits the string "Blink!\r\n" over LoRa at 0.5 Hz (every 2 seconds).
 * The LoRa radio task logs "LoraRadio: TX done IRQ" whenever the TX-done
 * interrupt fires, confirming that the radio hardware is alive.
 *
 * DeviceDiscovery and MeshNetwork must NOT be initialised alongside this
 * task — see init.c for the guard.
 */

#ifdef ENABLE_RADIO_TEST

#include "RadioTest.h"
#include "LoraRadio.h"
#include "dbg_log.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* --------------------------------------------------------------------------
 * Private defines
 * -------------------------------------------------------------------------- */
#define RADIO_TEST_STACK_SIZE   (configMINIMAL_STACK_SIZE * 4)
#define RADIO_TEST_PERIOD_MS    2000U   /* 0.5 Hz */

/* --------------------------------------------------------------------------
 * Forward declaration
 * -------------------------------------------------------------------------- */
static void RADIO_TEST_vTask(void *arg);

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
     * LORARADIO_vInit) before we attempt to enqueue a TX.
     */
    osDelay(500U);

    DBG("\r\nRadioTest: started — transmitting \"Blink!\\r\\n\" at 0.5 Hz\r\n");

    for (;;)
    {
        LoraRadio_Packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));

        const char *msg = "Blink!\r\n";
        pkt.length = (uint8_t)strlen(msg);
        memcpy(pkt.buffer, msg, pkt.length);

        if (LORARADIO_bTxPacket(&pkt))
            DBG("RadioTest: TX enqueued\r\n");
        else
            DBG("RadioTest: TX queue full!\r\n");

        osDelay(RADIO_TEST_PERIOD_MS);
    }
}

#endif /* ENABLE_RADIO_TEST */
