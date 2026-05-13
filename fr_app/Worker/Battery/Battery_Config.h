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

/* ADC → mV linear conversion: mV = (adc * M_NUM / M_DEN) + C */
#define BAT_MEAS_ADC_M_NUM          7510
#define BAT_MEAS_ADC_M_DEN          4095
#define BAT_MEAS_ADC_C              0

#endif /* WORKER_BATTERY_BATTERY_CONFIG_H_ */
