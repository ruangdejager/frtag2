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

#include <stdio.h>

/* Last task to overflow its stack — readable from a debugger if the UART
 * output itself doesn't make it out. */
volatile char *pcLastStackOverflowTask = NULL;

#if defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
/* Scratch buffer for the overflow message. Static (not on-stack): the
 * stack we're reporting on is the one that just overflowed, so this hook
 * must not add its own stack usage on top of it. */
static char acOverflowMsg[80];
#endif

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
#else
    (void)pcTaskName;
#endif

    for (;;) { }
}
