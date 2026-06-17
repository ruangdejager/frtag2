/*
 * hal_spi.c
 *
 * SPI drivers for:
 *   SPI1 — accelerometer (LIS2DH or compatible), PA1/PA6/PA7/PA11
 *   SPI2 — external NOR flash (AT25EU0041A),     PA5/PA8/PA10/PA15
 *
 * HAL_SPI_vInit() initialises both peripherals in one call.
 * Call it once at boot (before Flash_vInit) and once after every STOP2 wake.
 */

#include "hal_spi.h"
#include "hal_bsp.h"
#include "stm32wle5xx.h"

#include <string.h>

SPI_HandleTypeDef hAccSpi;
SPI_HandleTypeDef hFlashSpi;

/* --------------------------------------------------------------------------
 * HAL_SPI_FLASH_vReadPacket
 * Reads 'len' bytes from the flash by clocking out 0xFF dummy bytes via
 * full-duplex TransmitReceive (HAL_SPI_Receive does not clock in 2-line
 * master mode). Chunked so the dummy buffer stays small.
 *
 * Returns HAL_OK only if every chunk transferred cleanly; on the first
 * non-OK chunk the transfer is aborted immediately so the caller can
 * deselect without clocking further (possibly stale) bytes into rx.
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef HAL_SPI_FLASH_vReadPacket(uint8_t *rx, uint16_t len)
{
    uint8_t  au8Dummy[64];
    memset(au8Dummy, 0xFF, sizeof(au8Dummy));

    uint16_t u16Off = 0U;
    while (u16Off < len)
    {
        uint16_t u16N = (uint16_t)((len - u16Off) > sizeof(au8Dummy)
                                   ? sizeof(au8Dummy) : (len - u16Off));
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hFlashSpi, au8Dummy, rx + u16Off, u16N, SPI_TIMEOUT);
        if (status != HAL_OK)
            return status;
        u16Off = (uint16_t)(u16Off + u16N);
    }
    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * HAL_SPI_vInit
 * Initialises both SPI peripherals:
 *   SPI1 — accelerometer (PA1 SCK / PA6 MISO / PA7 MOSI, AF5)
 *   SPI2 — external NOR flash (PA5 MISO AF3 / PA8 SCK AF5 / PA10 MOSI AF5)
 *
 * Safe to call multiple times (e.g. after STOP2 wake — HAL_SPI_Init is
 * idempotent when the handle is already initialised).
 * -------------------------------------------------------------------------- */
void HAL_SPI_vInit(void)
{
    /* --- SPI1: accelerometer --- */
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

    /* --- SPI2: external NOR flash --- */
    hFlashSpi.Instance               = FLASH_SPI;
    hFlashSpi.Init.Mode              = SPI_MODE_MASTER;
    hFlashSpi.Init.Direction         = SPI_DIRECTION_2LINES;
    hFlashSpi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hFlashSpi.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hFlashSpi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hFlashSpi.Init.NSS               = SPI_NSS_SOFT;
    hFlashSpi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  /* ~0.5 MHz at 32 MHz PCLK */
    hFlashSpi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hFlashSpi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hFlashSpi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hFlashSpi.Init.CRCPolynomial     = 7;
    hFlashSpi.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hFlashSpi.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    HAL_SPI_Init(&hFlashSpi);
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

        /* MISO + MOSI — AF5 (SPI1 on port A) */
        GPIO_InitStruct.Pin       = BSP_ACC_MISO_PIN | BSP_ACC_MOSI_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = BSP_ACC_SPI_AF;
        HAL_GPIO_Init(BSP_ACC_MOSI_PORT, &GPIO_InitStruct);

        /* SCK — AF5 */
        GPIO_InitStruct.Pin       = BSP_ACC_SCK_PIN;
        GPIO_InitStruct.Alternate = BSP_ACC_SPI_AF;
        HAL_GPIO_Init(BSP_ACC_SCK_PORT, &GPIO_InitStruct);
    }
    else if (spiHandle->Instance == FLASH_SPI)
    {
        FLASH_SPI_CLK_ENABLE();
        FLASH_PORT_CLK_ENABLE();

        /* CS — push-pull output, idle HIGH */
        GPIO_InitStruct.Pin   = BSP_FLASH_CS_PIN;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(BSP_FLASH_CS_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(BSP_FLASH_CS_PORT, BSP_FLASH_CS_PIN, GPIO_PIN_SET);

        /* PA5 (MISO) — AF3 (SPI2_MISO on port A) */
        GPIO_InitStruct.Pin       = BSP_FLASH_MISO_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_SPI2;
        HAL_GPIO_Init(BSP_FLASH_MISO_PORT, &GPIO_InitStruct);

        /* PA8 (SCK) + PA10 (MOSI) — AF5 (SPI2 SCK/MOSI on port A) */
        GPIO_InitStruct.Pin       = BSP_FLASH_SCK_PIN | BSP_FLASH_MOSI_PIN;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(BSP_FLASH_MOSI_PORT, &GPIO_InitStruct);
    }
}

void HAL_SPI_vDeInit(void)
{
    HAL_SPI_DeInit(&hAccSpi);
}

void HAL_SPI_FLASH_vDeInit(void)
{
    HAL_SPI_DeInit(&hFlashSpi);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
    if (spiHandle->Instance == ACC_SPI)
    {
        ACC_SPI_CLK_DISABLE();
        HAL_GPIO_DeInit(BSP_ACC_SCK_PORT,  BSP_ACC_SCK_PIN);
        HAL_GPIO_DeInit(BSP_ACC_MOSI_PORT, BSP_ACC_MISO_PIN | BSP_ACC_MOSI_PIN);
    }
    else if (spiHandle->Instance == FLASH_SPI)
    {
        FLASH_SPI_CLK_DISABLE();
        HAL_GPIO_DeInit(BSP_FLASH_CS_PORT,   BSP_FLASH_CS_PIN);
        HAL_GPIO_DeInit(BSP_FLASH_MISO_PORT, BSP_FLASH_MISO_PIN);
        HAL_GPIO_DeInit(BSP_FLASH_MOSI_PORT, BSP_FLASH_SCK_PIN | BSP_FLASH_MOSI_PIN);
    }
}
