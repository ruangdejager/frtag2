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
