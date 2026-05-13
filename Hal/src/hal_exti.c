/*
 * hal_exti.c
 *
 * EXTI callback dispatch for GPIO interrupt lines.
 * Register application callbacks via HAL_EXTI_vRegisterCallback().
 */

#include <stddef.h>
#include "hal_exti.h"
#include "stm32wlxx.h"
#include "hal_bsp.h"

static callback_function_t nPgPinCallback = NULL;

void HAL_EXTI_vRegisterCallback(hal_exti_pin_t pin, callback_function_t callback_function)
{
    switch (pin)
    {
        case CHG_nPG_PIN:
            nPgPinCallback = callback_function;
            break;
        default:
            break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* No charger EXTI in frtag2; extend here for ACC_INT (PC13) if needed */
    (void)GPIO_Pin;
}
