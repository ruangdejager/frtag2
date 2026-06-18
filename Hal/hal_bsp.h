/*
 * hal_bsp.h
 *
 * Board support definitions for the frtag2 board.
 */

#ifndef HAL_BSP_H_
#define HAL_BSP_H_

#include "stm32wlxx_hal.h"

/* -----------------------------------------------------------------------
 * LED
 * ----------------------------------------------------------------------- */
#define BSP_LED_RED_PORT        GPIOB
#define BSP_LED_RED_PIN         GPIO_PIN_5
#define BSP_LED_YELLOW_PORT     GPIOB
#define BSP_LED_YELLOW_PIN      GPIO_PIN_8

typedef enum {
    LED_RED    = 0,
    LED_YELLOW = 1,
} Led_TypeDef;

/* Define to drive the status LEDs. Comment out to TERMINATE them: both pins are
 * left in their analog reset state (Hi-Z, undriven, zero current) and every LED
 * call becomes a no-op. Use the terminated build for production / low-power
 * current measurement. */
#define LEDS_ENABLED

static inline void BSP_LED_Init(Led_TypeDef led)
{
#ifdef LEDS_ENABLED
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    if (led == LED_RED)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = BSP_LED_RED_PIN;
        HAL_GPIO_Init(BSP_LED_RED_PORT, &gpio);
        HAL_GPIO_WritePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN, GPIO_PIN_SET);
    }
    else
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = BSP_LED_YELLOW_PIN;
        HAL_GPIO_Init(BSP_LED_YELLOW_PORT, &gpio);
        HAL_GPIO_WritePin(BSP_LED_YELLOW_PORT, BSP_LED_YELLOW_PIN, GPIO_PIN_SET);
    }
#else
    /* LEDs terminated: park both pins analog (reset state) regardless of which
     * LED was requested, so they draw nothing and are never driven. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = BSP_LED_RED_PIN | BSP_LED_YELLOW_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
    (void)led;
#endif
}

static inline void BSP_LED_On(Led_TypeDef led)
{
#ifdef LEDS_ENABLED
    if (led == LED_RED)
        HAL_GPIO_WritePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(BSP_LED_YELLOW_PORT, BSP_LED_YELLOW_PIN, GPIO_PIN_SET);
#else
    (void)led;
#endif
}

static inline void BSP_LED_Off(Led_TypeDef led)
{
#ifdef LEDS_ENABLED
    if (led == LED_RED)
        HAL_GPIO_WritePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(BSP_LED_YELLOW_PORT, BSP_LED_YELLOW_PIN, GPIO_PIN_RESET);
#else
    (void)led;
#endif
}

static inline void BSP_LED_Toggle(Led_TypeDef led)
{
#ifdef LEDS_ENABLED
    if (led == LED_RED)
        HAL_GPIO_TogglePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN);
    else
        HAL_GPIO_TogglePin(BSP_LED_YELLOW_PORT, BSP_LED_YELLOW_PIN);
#else
    (void)led;
#endif
}

/* -----------------------------------------------------------------------
 * Debug UART (USART2) — PA2/PA3
 * ----------------------------------------------------------------------- */
#define BSP_DEBUG_USART_INSTANCE    USART2
#define BSP_DEBUG_USART_IRQn        USART2_IRQn
#define BSP_DEBUG_UART_TX_PORT      GPIOA
#define BSP_DEBUG_UART_TX_PIN       GPIO_PIN_2
#define BSP_DEBUG_UART_RX_PORT      GPIOA
#define BSP_DEBUG_UART_RX_PIN       GPIO_PIN_3
#define BSP_DEBUG_UART_AF           GPIO_AF7_USART2
#define BSP_DEBUG_UART_BAUD         115200

/* -----------------------------------------------------------------------
 * GPS / Farmranger UART (USART1) — PB6/PB7
 * Primary role: Farmranger device.
 * Secondary role: GNSS module.
 * These two devices are mutually exclusive based on ROLE_BIT0.
 * GNSS_RX (PB6) = MCU USART1_TX; GNSS_TX (PB7) = MCU USART1_RX.
 * ----------------------------------------------------------------------- */
#define BSP_GPS_USART_INSTANCE      USART1
#define BSP_GPS_USART_IRQn          USART1_IRQn
#define BSP_GPS_UART_TX_PORT        GPIOB
#define BSP_GPS_UART_TX_PIN         GPIO_PIN_6
#define BSP_GPS_UART_RX_PORT        GPIOB
#define BSP_GPS_UART_RX_PIN         GPIO_PIN_7
#define BSP_GPS_UART_AF             GPIO_AF7_USART1
#define BSP_GPS_UART_BAUD           9600    /* MAX-M10S factory default; switch to 115200 via UBX-CFG-VALSET once comms established */

/* On a PRIMARY, USART1 is repurposed as the Farmranger link to the fr9 board.
 * That board's frtag UART runs at a fixed 115200, so the link must match -
 * the GPS default (9600) above only applies to a SECONDARY running GNSS. */
#define BSP_FARMRANGER_UART_BAUD    115200

/* GNSS power enable — single pin (replaces 3-pin GPS power from frtag) */
#define BSP_GNSS_ON_PORT            GPIOA
#define BSP_GNSS_ON_PIN             GPIO_PIN_0

/* -----------------------------------------------------------------------
 * Accelerometer SPI (SPI1) — PA1/PA6/PA7/PA11
 * ----------------------------------------------------------------------- */
#define BSP_ACC_CS_PIN              GPIO_PIN_11
#define BSP_ACC_CS_PORT             GPIOA
#define BSP_ACC_SCK_PIN             GPIO_PIN_1
#define BSP_ACC_SCK_PORT            GPIOA
#define BSP_ACC_MISO_PIN            GPIO_PIN_6
#define BSP_ACC_MISO_PORT           GPIOA
#define BSP_ACC_MOSI_PIN            GPIO_PIN_7
#define BSP_ACC_MOSI_PORT           GPIOA
#define BSP_ACC_INT_PIN            	GPIO_PIN_13
#define BSP_ACC_INT_PORT           	GPIOC

/* -----------------------------------------------------------------------
 * Battery measurement (ADC)
 * MEAS_BAT_VOLT = PA12 = ADC_IN8, 10k/33k divider, 1.8 V ref
 * ----------------------------------------------------------------------- */
#define BSP_BAT_BIAS_ENABLE_PORT        GPIOA
#define BSP_BAT_BIAS_ENABLE_PIN         GPIO_PIN_4
#define BSP_BAT_MEAS_VOLT_PORT          GPIOA
#define BSP_BAT_MEAS_VOLT_PIN           GPIO_PIN_12
#define BSP_BAT_ADC_VOLTAGE_CHANNEL     ADC_CHANNEL_8

/* -----------------------------------------------------------------------
 * External Flash SPI (SPI2) — PA5/PA8/PA10/PA15
 * AT25EU0041A-SSHN-T, 512 KB NOR flash
 * ----------------------------------------------------------------------- */
#define BSP_FLASH_MISO_PORT         GPIOA
#define BSP_FLASH_MISO_PIN          GPIO_PIN_5
#define BSP_FLASH_SCK_PORT          GPIOA
#define BSP_FLASH_SCK_PIN           GPIO_PIN_8
#define BSP_FLASH_MOSI_PORT         GPIOA
#define BSP_FLASH_MOSI_PIN          GPIO_PIN_10
#define BSP_FLASH_CS_PORT           GPIOA
#define BSP_FLASH_CS_PIN            GPIO_PIN_15

/* -----------------------------------------------------------------------
 * Solar power measurement (ADC)
 * VSOLAR_MEAS  = PB3 = ADC_IN2, 10k/33k divider, 1.8 V ref
 * RSENSE_MEAS  = PB4 = ADC_IN3, no divider,       1.8 V ref
 * ----------------------------------------------------------------------- */
#define BSP_VSOLAR_MEAS_PORT            GPIOB
#define BSP_VSOLAR_MEAS_PIN             GPIO_PIN_3
#define BSP_VSOLAR_ADC_CHANNEL          ADC_CHANNEL_2

#define BSP_RSENSE_MEAS_PORT            GPIOB
#define BSP_RSENSE_MEAS_PIN             GPIO_PIN_4
#define BSP_RSENSE_ADC_CHANNEL          ADC_CHANNEL_3

/* -----------------------------------------------------------------------
 * RF switch control — PA9 (HIGH = TX/LP, LOW = RX/OFF)
 * ----------------------------------------------------------------------- */
#define BSP_RF_SW_CTRL_PORT         GPIOA
#define BSP_RF_SW_CTRL_PIN          GPIO_PIN_9

/* -----------------------------------------------------------------------
 * Device role sense — single GPIO (primary=HIGH / secondary=LOW)
 * Only ROLE_BIT0 exists; VER_BIT1/VER_BIT2 removed in frtag2.
 * ----------------------------------------------------------------------- */
#define BSP_ROLE_BIT0_PORT          GPIOB
#define BSP_ROLE_BIT0_PIN           GPIO_PIN_12

/* -----------------------------------------------------------------------
 * External Farmranger interface GPIO interrupt
 * Shared with GNSS_ON: used as wake-up signal to the Farmranger device
 * on primary; used as GNSS power enable on secondary.
 * ----------------------------------------------------------------------- */
#define BSP_FR_GPIO_INT_PIN         GPIO_PIN_0
#define BSP_FR_GPIO_INT_PORT        GPIOA

#endif /* HAL_BSP_H_ */
