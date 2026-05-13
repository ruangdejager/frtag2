/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t u32GetCSR(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VSOLAR_MEAS_VOL_Pin GPIO_PIN_3
#define VSOLAR_MEAS_VOL_GPIO_Port GPIOB
#define RSENSE_MEAS_VOLT_Pin GPIO_PIN_4
#define RSENSE_MEAS_VOLT_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOB
#define GNSS_RX_Pin GPIO_PIN_6
#define GNSS_RX_GPIO_Port GPIOB
#define GNSS_TX_Pin GPIO_PIN_7
#define GNSS_TX_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_8
#define LED_YELLOW_GPIO_Port GPIOB
#define GNSS_ON_Pin GPIO_PIN_0
#define GNSS_ON_GPIO_Port GPIOA
#define ACC_SCLK_Pin GPIO_PIN_1
#define ACC_SCLK_GPIO_Port GPIOA
#define DBG_UART_TX_Pin GPIO_PIN_2
#define DBG_UART_TX_GPIO_Port GPIOA
#define DBG_UART_RX_Pin GPIO_PIN_3
#define DBG_UART_RX_GPIO_Port GPIOA
#define BAT_MEAS_BIAS_Pin GPIO_PIN_4
#define BAT_MEAS_BIAS_GPIO_Port GPIOA
#define SD_SO_Pin GPIO_PIN_5
#define SD_SO_GPIO_Port GPIOA
#define ACC_SO_Pin GPIO_PIN_6
#define ACC_SO_GPIO_Port GPIOA
#define ACC_SI_Pin GPIO_PIN_7
#define ACC_SI_GPIO_Port GPIOA
#define SD_SCLK_Pin GPIO_PIN_8
#define SD_SCLK_GPIO_Port GPIOA
#define RF_SW_CTRL_Pin GPIO_PIN_9
#define RF_SW_CTRL_GPIO_Port GPIOA
#define ROLE_BIT0_Pin GPIO_PIN_12
#define ROLE_BIT0_GPIO_Port GPIOB
#define SD_SI_Pin GPIO_PIN_10
#define SD_SI_GPIO_Port GPIOA
#define ACC_CS_Pin GPIO_PIN_11
#define ACC_CS_GPIO_Port GPIOA
#define MEAS_BAT_VOLT_Pin GPIO_PIN_12
#define MEAS_BAT_VOLT_GPIO_Port GPIOA
#define ACC_INT_Pin GPIO_PIN_13
#define ACC_INT_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
