/*
 * hal_crc.c
 *
 * CRC peripheral driver — default polynomial, byte-wise input.
 */

#include "hal_crc.h"
#include "stm32wlxx.h"

CRC_HandleTypeDef hcrc;

void HAL_CRC_vInit(void)
{
    hcrc.Instance                          = CRC;
    hcrc.Init.DefaultPolynomialUse         = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse          = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode       = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode      = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat                   = CRC_INPUTDATA_FORMAT_BYTES;
    HAL_CRC_Init(&hcrc);
}

void HAL_CRC_MspInit(CRC_HandleTypeDef *crcHandle)
{
    if (crcHandle->Instance == CRC)
        __HAL_RCC_CRC_CLK_ENABLE();
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef *crcHandle)
{
    if (crcHandle->Instance == CRC)
        __HAL_RCC_CRC_CLK_DISABLE();
}
