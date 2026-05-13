/*
 * stm32wlxx_hal_msp.c
 *
 * MSP Initialization for frtag2.
 * HAL_SUBGHZ_MspInit / HAL_SUBGHZ_MspDeInit are defined in
 * Hal/src/hal_subghz.c alongside the rest of the SUBGHZ HAL layer.
 */

#include "main.h"

void HAL_MspInit(void)
{
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}
