/*
 * hal_adc.h
 *
 * ADC driver — battery voltage measurement on BSP_BAT_MEAS_VOLT_PIN (PB4).
 */

#ifndef INC_HAL_ADC_H_
#define INC_HAL_ADC_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32wlxx_hal.h"

typedef enum {
    BAT_VOLTAGE_CHANNEL = 0,
} hal_adc_channel_t;

extern ADC_HandleTypeDef hadc;

void     HAL_ADC_vInit(void);
void     HAL_ADC_vEnable(void);
bool     HAL_ADC_bIsEnabled(void);
void     HAL_ADC_vDisable(void);
bool     HAL_ADC_bGetInterruptFlag(void);
void     HAL_ADC_vClearInterruptFlag(void);
void     HAL_ADC_vSelectChannel(hal_adc_channel_t channel);
void     HAL_ADC_vStartConversion(hal_adc_channel_t channel);
uint16_t HAL_ADC_u16GetResult(void);

#endif /* INC_HAL_ADC_H_ */
