/*
 * app_freertos.c
 *
 * FreeRTOS application hooks.
 *
 * STOP2 sleep is handled entirely by the custom vPortSuppressTicksAndSleep()
 * in Hal/src/hal_system.c, which overrides the weak ARM_CM3 port routine.
 * The CubeMX configPRE_SLEEP_PROCESSING / configPOST_SLEEP_PROCESSING hooks
 * are therefore not used and intentionally left unmapped.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "dbg_log.h"
#include "Debug.h"
#include "hal_bsp.h"

#include <stdio.h>
#include <string.h>

/* Last task to overflow its stack — readable from a debugger if the UART
 * output itself doesn't make it out. */
volatile char *pcLastStackOverflowTask = NULL;

#if defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
/* Scratch buffer for the overflow message. Static (not on-stack): the
 * stack we're reporting on is the one that just overflowed, so this hook
 * must not add its own stack usage on top of it. */
static char acOverflowMsg[80];
#endif

/* Stack-overflow flash code: the yellow LED flashes (50ms on / 50ms off) N
 * times, holds off for 2s, then repeats — N is this table's 1-based index
 * of pcTaskName. A task name not in this table holds the LED solid ON
 * instead, which is visually distinct from any numbered code.
 *
 *  1 = InitTask          9 = CheckWakeupSched    17 = FRAtHandlerTask
 *  2 = Heartbeat        10 = SolarSampleTask     18 = FRDbgTxTask
 *  3 = DbgLog           11 = SolarSchedTask      19 = BatSampleTask
 *  4 = LogTask          12 = Movement            20 = BatSchedTask
 *  5 = FrKernel         13 = GPSRxTask           21 = BatPurgeTask
 *  6 = MeshParser       14 = GPSDispatcher       22 = PowerStateMgr
 *  7 = MeshTx           15 = LoRaRadioTask       23 = RadioTest
 *  8 = DevDiscoveryApp  16 = FRRxTask
 */
static const char * const apcOverflowTaskNames[] = {
    "InitTask", "Heartbeat", "DbgLog", "LogTask", "FrKernel",
    "MeshParser", "MeshTx", "DevDiscoveryApp", "CheckWakeupSched",
    "SolarSampleTask", "SolarSchedTask", "Movement", "GPSRxTask",
    "GPSDispatcher", "LoRaRadioTask", "FRRxTask", "FRAtHandlerTask",
    "FRDbgTxTask", "BatSampleTask", "BatSchedTask", "BatPurgeTask",
    "PowerStateMgr", "RadioTest"
};

/* Approximate busy-wait, calibrated off SystemCoreClock. Good enough for a
 * visual flash code — does not depend on SysTick/interrupts or the
 * scheduler, both of which are dead by the time this hook runs.
 * (HAL_vDelayUs() is unsafe here: it wraps incorrectly for delays beyond a
 * single SysTick period.) */
static void LED_vBlockingDelayMs(uint32_t u32Ms)
{
    uint32_t u32Loops = u32Ms * (SystemCoreClock / 8000U);
    for (volatile uint32_t i = 0; i < u32Loops; i++) { __NOP(); }
}

/* Returns the 1-based flash count for pcTaskName, or 0 if not found in
 * apcOverflowTaskNames[]. */
static uint8_t u8GetOverflowFlashCode(const char *pcTaskName)
{
    for (uint8_t i = 0; i < (sizeof(apcOverflowTaskNames) / sizeof(apcOverflowTaskNames[0])); i++)
    {
        if (strcmp(pcTaskName, apcOverflowTaskNames[i]) == 0)
            return (uint8_t)(i + 1U);
    }
    return 0U;
}

/* Called by the kernel (configCHECK_FOR_STACK_OVERFLOW == 2) when a task
 * overruns its stack. Record the name, write it out, then halt so the
 * debugger stops here with the culprit identified.
 *
 * DBG()/DBGLOG_vPut() do NOT work here: this hook runs from vTaskSwitchContext
 * inside PendSV (i.e. IPSR != 0), so DBGLOG_vPut's ISR guard drops the message
 * silently, and even if it didn't, the message would just sit in the ring
 * buffer for the DbgLog consumer task — which never runs because interrupts
 * are disabled immediately afterwards and the scheduler is dead. Instead,
 * write directly to the UART with a polling transmit that needs neither
 * interrupts nor the scheduler. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    pcLastStackOverflowTask = pcTaskName;

    taskDISABLE_INTERRUPTS();

#if defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
    /* DEBUG_vPutBufferBlocking() drives the debug UART (USART2) directly via
     * Debug.c's sDriverData. In LISTENER_MODE that UART is owned by the
     * Farmranger driver instead and Debug.c's handle is never set up, so
     * skip the write there — pcLastStackOverflowTask remains debugger-readable. */
    int len = snprintf(acOverflowMsg, sizeof(acOverflowMsg),
                        "\r\n*** STACK OVERFLOW: %s ***\r\n", pcTaskName);
    if (len > 0)
        DEBUG_vPutBufferBlocking((const uint8_t *)acOverflowMsg, (uint16_t)len);
#endif

    /* Visual flash code on the yellow LED — see apcOverflowTaskNames[] above
     * for the task/flash-count mapping. Runs forever; no debugger or UART
     * required. */
    uint8_t u8FlashCount = u8GetOverflowFlashCode(pcTaskName);

    if (u8FlashCount == 0U)
    {
        /* Unrecognized task name: solid-on is visually distinct from any
         * numbered code. */
        BSP_LED_On(LED_YELLOW);
        for (;;) { }
    }

    for (;;)
    {
        for (uint8_t i = 0; i < u8FlashCount; i++)
        {
            BSP_LED_On(LED_YELLOW);
            LED_vBlockingDelayMs(50);
            BSP_LED_Off(LED_YELLOW);
            LED_vBlockingDelayMs(50);
        }
        LED_vBlockingDelayMs(2000);
    }
}
