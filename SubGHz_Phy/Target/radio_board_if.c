/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    radio_board_if.c
  * @brief   Radio board interface for frtag2.
  *          RF switch: PA9 (RF_SW_CTRL) — HIGH=TX(LP), LOW=RX/OFF
  ******************************************************************************
  */
/* USER CODE END Header */

#include "radio_board_if.h"
#include "main.h"
#include "stm32wlxx_hal.h"

int32_t RBI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port, RF_SW_CTRL_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = RF_SW_CTRL_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RF_SW_CTRL_GPIO_Port, &GPIO_InitStruct);

    return 0;
}

int32_t RBI_DeInit(void)
{
    HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port, RF_SW_CTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_DeInit(RF_SW_CTRL_GPIO_Port, RF_SW_CTRL_Pin);
    return 0;
}

int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
    switch (Config)
    {
        case RBI_SWITCH_OFF:
        case RBI_SWITCH_RX:
            HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port, RF_SW_CTRL_Pin, GPIO_PIN_RESET);
            break;
        case RBI_SWITCH_RFO_LP:
        case RBI_SWITCH_RFO_HP:
            HAL_GPIO_WritePin(RF_SW_CTRL_GPIO_Port, RF_SW_CTRL_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
    return 0;
}

int32_t RBI_GetTxConfig(void)
{
    return RBI_CONF_RFO_LP;
}

int32_t RBI_IsTCXO(void)
{
    return IS_TCXO_SUPPORTED;
}

int32_t RBI_IsDCDC(void)
{
    return IS_DCDC_SUPPORTED;
}

int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef Config)
{
    if (Config == RBI_RFO_LP_MAXPOWER)
        return 15;
    else
        return 22;
}
