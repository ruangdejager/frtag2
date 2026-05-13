/*
 * hal_gpio.c
 *
 * GPIO initialisation for the frtag2 custom board.
 * All GPIOs are brought to a known low-power state at boot.
 * Peripheral-specific pins (UART TX/RX, SPI SCK/MISO/MOSI) are
 * initialised by their respective HAL drivers.
 */

#include "hal_gpio.h"
#include "hal_bsp.h"

#include "stm32wlxx.h"

static bool bLedsAllowed = true;

void HAL_GPIO_vInit(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LEDs */
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_YELLOW);

    /* GNSS power enable — single pin replaces 3-pin GPS power from frtag */
    HAL_GPIO_vInitOutput(BSP_GNSS_ON_PORT, BSP_GNSS_ON_PIN, GPIO_PIN_RESET);

    /* Accelerometer chip-select */
#ifdef ENABLE_MOVE
    HAL_GPIO_vInitOutput(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_vInitOutput(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, GPIO_PIN_RESET);
#endif
    /* ACC SPI SCK/MISO/MOSI are initialised by hal_spi */

    /* Battery measurement */
    HAL_GPIO_vInitOutput(BSP_BAT_BIAS_ENABLE_PORT, BSP_BAT_BIAS_ENABLE_PIN, GPIO_PIN_RESET);
    /* BAT_MEAS_VOLT is configured as analog by hal_adc */

    /* Farmranger interrupt output */
    HAL_GPIO_vInitOutput(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_RESET);

    /* RF switch control — start LOW (RX/OFF) */
    HAL_GPIO_vInitOutput(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_RESET);

    /* Device role sense */
    HAL_GPIO_vInitInput(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN, GPIO_PULLDOWN);
}

void HAL_GPIO_vOnSleep(void)
{
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
}

void HAL_GPIO_OnWake(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

bool HAL_GPIO_bLedsAllowed(void)
{
    return bLedsAllowed;
}

void HAL_GPIO_vAllowLeds(bool allow)
{
    bLedsAllowed = allow;
}

void HAL_GPIO_vInitIntPullup(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitOutput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, GPIO_PinState gpio_pinstate)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
    HAL_GPIO_WritePin(GPIOx, gpio_pin, gpio_pinstate);
}

void HAL_GPIO_vInitInput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = gpio_pull;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitAnalogNoPull(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}
