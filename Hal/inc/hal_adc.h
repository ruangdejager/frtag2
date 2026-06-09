/*
 * hal_adc.h
 *
 * ADC driver — battery, VSOLAR, and RSENSE voltage measurement.
 *
 * The ADC peripheral is shared across multiple workers. Callers must
 * acquire the ADC mutex (HAL_ADC_vLock / HAL_ADC_vUnlock) around the
 * full enable → convert → disable sequence to prevent concurrent access.
 */

#ifndef INC_HAL_ADC_H_
#define INC_HAL_ADC_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32wlxx_hal.h"
#include "cmsis_os2.h"

typedef enum {
    BAT_VOLTAGE_CHANNEL    = 0,
    VSOLAR_VOLTAGE_CHANNEL = 1,
    RSENSE_VOLTAGE_CHANNEL = 2,
    VREFINT_CHANNEL        = 3,
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

/* Serialize access to the shared ADC across workers (battery, solar). Acquire
 * around the full enable → convert → disable sequence. */
void     HAL_ADC_vLock(void);
void     HAL_ADC_vUnlock(void);

/* Convert a raw VREFINT conversion (12-bit) into the actual analog supply
 * voltage VDDA in mV, using the chip's factory VREFINT calibration. Returns
 * 0 if the raw value is 0 (conversion failed). */
uint16_t HAL_ADC_u16VddaFromVrefint(uint16_t u16VrefintRaw);

#endif /* INC_HAL_ADC_H_ */
