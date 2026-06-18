/*
 * main.c  (fr_app/src/main.c)
 *
 * Application entry point for the frtag firmware.
 *
 * Responsibilities:
 *   - Enable configurable fault handlers (BusFault, MemManage)
 *   - Capture and clear the RCC reset-cause register (CSR) before any
 *     HAL call that might clear it
 *   - Call HAL_vInit() — expands to HAL_Init() + SystemClock_Config()
 *     (SystemClock_Config implementation is in Hal/src/hal_system.c)
 *   - Initialise the CMSIS-RTOS v2 kernel
 *   - Create the single boot-sequence task (INIT_vInitialization)
 *   - Start the scheduler
 *
 * All remaining subsystem initialisation happens inside
 * INIT_vInitialization() once the scheduler is running.
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "tag_hal.h"
#include "platform.h"
#include "init.h"
#include "cmsis_os2.h"

#include "hal_system.h"

static uint32_t csr;

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */
int main(void)
{
    /* Enable configurable fault handlers for easier debug */
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    /* Snapshot RCC reset-cause flags before HAL_Init() clears them */
    csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* HAL_Init() + SystemClock_Config() (48 MHz PLL via hal_system.c) */
    HAL_vInit();

    /* ---- CMSIS-RTOS v2 / FreeRTOS ------------------------------------- */
    osKernelInitialize();

    static const osThreadAttr_t initTask_attr = {
        .name       = "InitTask",
        .stack_size = configMINIMAL_STACK_SIZE * 8,  /* 1024 bytes */
        .priority   = osPriorityNormal,
    };
    osThreadNew(INIT_vInitialization, NULL, &initTask_attr);

    osKernelStart();

    /* Should never be reached — scheduler never returns */
    while (1) { }
}

/* --------------------------------------------------------------------------
 * u32GetCSR — returns the RCC reset-cause register captured at startup
 * Callers (e.g. FLASHLOG in init.c) use this to log the reset reason.
 * -------------------------------------------------------------------------- */
uint32_t u32GetCSR(void)
{
    return csr;
}
