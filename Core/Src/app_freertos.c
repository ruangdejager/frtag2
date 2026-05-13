/*
 * app_freertos.c
 *
 * FreeRTOS application hooks.
 * PreSleepProcessing / PostSleepProcessing delegate to the HAL system layer.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "hal_system.h"

void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    HAL_SYSTEM_vOnPreSleep();
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    HAL_SYSTEM_vOnPostWake();
}
