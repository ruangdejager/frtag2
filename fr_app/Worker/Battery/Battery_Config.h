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

/* Resistor-divider ratio on the battery sense input: (R_top + R_bottom)/R_bottom
 * = (330k + 100k)/100k = 4.30. Battery voltage is reconstructed as:
 *   Vbatt = adc_bat * VDDA_measured / BAT_ADC_FULL_SCALE * BAT_DIVIDER_NUM/DEN
 * VDDA_measured comes from the calibrated VREFINT channel, so the result no
 * longer depends on the rail being exactly 1.8 V. The ratio was confirmed
 * against a calibrated reference (true 4050 mV / measured pin 941.9 mV = 4.300). */
#define BAT_DIVIDER_NUM             4300U
#define BAT_DIVIDER_DEN             1000U

#endif /* WORKER_BATTERY_BATTERY_CONFIG_H_ */
