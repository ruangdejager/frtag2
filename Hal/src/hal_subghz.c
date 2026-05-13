/*
 * hal_subghz.c
 *
 * Sub-GHz radio SPI peripheral initialisation and unique-ID helper.
 */

#include "hal_subghz.h"
#include "hal_system.h"

SUBGHZ_HandleTypeDef hsubghz;

void HAL_vSUBGHZ_Init(void)
{
    hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_4;
    if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
        Error_Handler();
}

/* --------------------------------------------------------------------------
 * HAL_SUBGHZ_MspInit / HAL_SUBGHZ_MspDeInit
 * Called by HAL_SUBGHZ_Init / HAL_SUBGHZ_DeInit.
 * These override the __weak defaults in stm32wlxx_hal_subghz.c.
 * stm32wlxx_hal_msp.c must NOT define these functions.
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef *subghzHandle)
{
    (void)subghzHandle;
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

void HAL_SUBGHZ_MspDeInit(SUBGHZ_HandleTypeDef *subghzHandle)
{
    (void)subghzHandle;
    __HAL_RCC_SUBGHZSPI_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
}

/* --------------------------------------------------------------------------
 * HAL_SUBGHZ_vSetUniqueId
 * Writes the first 8 bytes of the 96-bit device unique ID into id[].
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_vSetUniqueId(uint8_t *id)
{
    uint32_t uid[3];
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();

    id[0] = (uid[0] >> 24) & 0xFF;
    id[1] = (uid[0] >> 16) & 0xFF;
    id[2] = (uid[0] >>  8) & 0xFF;
    id[3] = (uid[0]      ) & 0xFF;
    id[4] = (uid[1] >> 24) & 0xFF;
    id[5] = (uid[1] >> 16) & 0xFF;
    id[6] = (uid[1] >>  8) & 0xFF;
    id[7] = (uid[1]      ) & 0xFF;
}
