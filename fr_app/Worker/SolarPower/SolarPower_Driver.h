/*
 * SolarPower_Driver.h
 *
 * Board-level macros for solar power ADC measurement.
 * No bias circuit — VSOLAR and RSENSE are measured directly.
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_DRIVER_H_
#define WORKER_SOLARPOWER_SOLARPOWER_DRIVER_H_

#include "hal_adc.h"

#define SOLAR_DRIVER_vLock()             HAL_ADC_vLock()
#define SOLAR_DRIVER_vUnlock()           HAL_ADC_vUnlock()
#define SOLAR_DRIVER_vEnable()           HAL_ADC_vEnable()
#define SOLAR_DRIVER_vDisable()          HAL_ADC_vDisable()
#define SOLAR_DRIVER_bGetInterruptFlag() HAL_ADC_bGetInterruptFlag()
#define SOLAR_DRIVER_vCleanInterrupt()   HAL_ADC_vClearInterruptFlag()
#define SOLAR_DRIVER_vStartVsolar()      HAL_ADC_vStartConversion(VSOLAR_VOLTAGE_CHANNEL)
#define SOLAR_DRIVER_vStartRsense()      HAL_ADC_vStartConversion(RSENSE_VOLTAGE_CHANNEL)
#define SOLAR_DRIVER_u16GetResult()      HAL_ADC_u16GetResult()

#endif /* WORKER_SOLARPOWER_SOLARPOWER_DRIVER_H_ */
