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

/* Last task to overflow its stack — readable from a debugger if the DBG
 * output itself doesn't make it out. */
volatile char *pcLastStackOverflowTask = NULL;

/* Called by the kernel (configCHECK_FOR_STACK_OVERFLOW == 2) when a task
 * overruns its stack. Record the name, log it, then halt so the debugger
 * stops here with the culprit identified. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    pcLastStackOverflowTask = pcTaskName;
    DBG("\r\n*** STACK OVERFLOW: %s ***\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
