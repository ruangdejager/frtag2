/*
 * hal_spi.c
 *
 * SPI driver for the accelerometer (LIS2DH or compatible) on SPI1.
 */

#include "hal_spi.h"
#include "hal_bsp.h"
#include "stm32wle5xx.h"

SPI_HandleTypeDef hAccSpi;

/* --------------------------------------------------------------------------
 * HAL_SPI_vInit
 * Initialises SPI1 in master mode for the accelerometer.
 * -------------------------------------------------------------------------- */
void HAL_SPI_vInit(void)
{
    hAccSpi.Instance               = ACC_SPI;
    hAccSpi.Init.Mode              = SPI_MODE_MASTER;
    hAccSpi.Init.Direction         = SPI_DIRECTION_2LINES;
    hAccSpi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hAccSpi.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hAccSpi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hAccSpi.Init.NSS               = SPI_NSS_SOFT;
    hAccSpi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hAccSpi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hAccSpi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hAccSpi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hAccSpi.Init.CRCPolynomial     = 7;
    hAccSpi.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hAccSpi.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    HAL_SPI_Init(&hAccSpi);
}

/* --------------------------------------------------------------------------
 * HAL_SPI_MspInit — called from HAL_SPI_Init()
 * -------------------------------------------------------------------------- */
void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (spiHandle->Instance == ACC_SPI)
    {
        ACC_SPI_CLK_ENABLE();
        ACC_PORT_CLK_ENABLE();

        GPIO_InitStruct.Pin       = BSP_ACC_MISO_PIN | BSP_ACC_MOSI_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = BSP_ACC_SPI_AF;
        HAL_GPIO_Init(BSP_ACC_MOSI_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = BSP_ACC_SCK_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = BSP_ACC_SPI_AF;
        HAL_GPIO_Init(BSP_ACC_SCK_PORT, &GPIO_InitStruct);
    }
}

/* --------------------------------------------------------------------------
 * HAL_SPI_OnWake
 * Re-initialises the SPI GPIO pins after returning from STOP2 sleep.
 * -------------------------------------------------------------------------- */
void HAL_SPI_OnWake(void)
{
    ACC_SPI_CLK_ENABLE();
    ACC_PORT_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin       = BSP_ACC_MISO_PIN | BSP_ACC_MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_ACC_SPI_AF;
    HAL_GPIO_Init(BSP_ACC_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = BSP_ACC_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_ACC_SPI_AF;
    HAL_GPIO_Init(BSP_ACC_SCK_PORT, &GPIO_InitStruct);
}

void HAL_SPI_vDeInit(void)
{
    HAL_SPI_DeInit(&hAccSpi);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
    if (spiHandle->Instance == ACC_SPI)
    {
        ACC_SPI_CLK_DISABLE();
        HAL_GPIO_DeInit(BSP_ACC_SCK_PORT,  BSP_ACC_SCK_PIN);
        HAL_GPIO_DeInit(BSP_ACC_MOSI_PORT, BSP_ACC_MISO_PIN | BSP_ACC_MOSI_PIN);
    }
}
