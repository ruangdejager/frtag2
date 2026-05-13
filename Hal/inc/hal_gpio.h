/*
 * hal_gpio.h
 *
 * GPIO initialisation, sleep/wake helpers, and LED permission control.
 */

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32wlxx.h"
#include "hal_bsp.h"

typedef enum _leds_active_t
{
    LEDS_ACTIVE_RESERVED = 0,
    LEDS_ACTIVE_ENABLE,
    LEDS_ACTIVE_DISABLE,
} leds_active_t;

void HAL_GPIO_vInit(void);
void HAL_GPIO_vOnSleep(void);
void HAL_GPIO_OnWake(void);

void HAL_GPIO_vInitInput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pull);
void HAL_GPIO_vInitIntPullup(GPIO_TypeDef *GPIOx, uint32_t gpio_pin);
void HAL_GPIO_vInitOutput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, GPIO_PinState gpio_pinstate);
void HAL_GPIO_vInitAnalogNoPull(GPIO_TypeDef *GPIOx, uint32_t gpio_pin);

bool HAL_GPIO_bLedsAllowed(void);
void HAL_GPIO_vAllowLeds(bool allow);

#endif /* HAL_GPIO_H_ */
