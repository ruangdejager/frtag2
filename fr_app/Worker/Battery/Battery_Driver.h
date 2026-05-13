/*
 * Battery_Driver.h
 *
 * Board-level macros for battery ADC measurement circuit.
 */

#ifndef WORKER_BATTERY_BATTERY_DRIVER_H_
#define WORKER_BATTERY_BATTERY_DRIVER_H_

#include "hal_bsp.h"
#include "hal_gpio.h"
#include "hal_adc.h"

#define BAT_DRIVER_vEnableBiasCircuit()     HAL_GPIO_WritePin(BSP_BAT_BIAS_ENABLE_PORT, BSP_BAT_BIAS_ENABLE_PIN, RESET)
#define BAT_DRIVER_vDisableBiasCircuit()    HAL_GPIO_WritePin(BSP_BAT_BIAS_ENABLE_PORT, BSP_BAT_BIAS_ENABLE_PIN, SET)

#define BAT_DRIVER_vEnable()                HAL_ADC_vEnable()
#define BAT_DRIVER_bIsEnabled()             HAL_ADC_bIsEnabled()
#define BAT_DRIVER_vDisable()               HAL_ADC_vDisable()

#define BAT_DRIVER_bGetInterruptFlag()      HAL_ADC_bGetInterruptFlag()
#define BAT_DRIVER_vCleanInterrupt()        HAL_ADC_vClearInterruptFlag()

#define BAT_DRIVER_vStartConversion()       HAL_ADC_vStartConversion(BAT_VOLTAGE_CHANNEL)
#define BAT_DRIVER_u16GetResult()           HAL_ADC_u16GetResult()

#endif /* WORKER_BATTERY_BATTERY_DRIVER_H_ */
