/*
 * Battery_Config.h
 *
 * Battery measurement parameters.
 */

#ifndef WORKER_BATTERY_BATTERY_CONFIG_H_
#define WORKER_BATTERY_BATTERY_CONFIG_H_

/* Settling time (ms) after bias enable before measurement */
#define BAT_MEAS_BIAS_SETTLE_TIME   20

/* Averaging factor */
#define BAT_AVG_FACTOR              8

/* Sample rate (s) */
#define BAT_SAMPLE_INTERVAL         10

/* Max voltage delta between samples (mV) */
#define BAT_SAMPLE_VALUE_DELTA_MAX  20.f

/* ADC full-scale code (12-bit) */
#define BAT_ADC_FULL_SCALE          4095U

/* Resistor-divider ratio on the battery sense input, expressed as a fraction.
 * The legacy full-scale constant (7510 mV at a 1.8 V reference) factors into
 * the pure divider ratio 7510/1800. Battery voltage is reconstructed as:
 *   Vpin  = adc_bat * VDDA_measured / BAT_ADC_FULL_SCALE
 *   Vbatt = Vpin * BAT_DIVIDER_NUM / BAT_DIVIDER_DEN
 * VDDA_measured comes from the VREFINT channel, so the result no longer
 * depends on the rail being exactly 1.8 V. */
#define BAT_DIVIDER_NUM             7510U
#define BAT_DIVIDER_DEN             1800U

#endif /* WORKER_BATTERY_BATTERY_CONFIG_H_ */
