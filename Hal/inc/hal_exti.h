/*
 * hal_exti.h
 *
 * EXTI callback registration for GPIO interrupt sources.
 */

#ifndef INC_HAL_EXTI_H_
#define INC_HAL_EXTI_H_

#include <stdint.h>

typedef enum {
    CHG_nPG_PIN = 0,
} hal_exti_pin_t;

typedef void (*callback_function_t)(void);

void HAL_EXTI_vRegisterCallback(hal_exti_pin_t pin, callback_function_t callback_function);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_HAL_EXTI_H_ */
