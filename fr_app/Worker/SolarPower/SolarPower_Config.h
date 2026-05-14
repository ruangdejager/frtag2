/*
 * SolarPower_Config.h
 *
 * Solar power measurement parameters.
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_
#define WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_

/* Sample rate (s) — one measurement per heartbeat cycle */
#define SOLAR_SAMPLE_INTERVAL           10

/* Sense resistor: 1.5 Ω, 1% — stored ×100 for integer arithmetic */
#define SOLAR_RSENSE_OHM_X100           150

/* ADC → mV for VSOLAR: 10k/33k divider, 1.8 V ref
 * Theoretical: count × (1800 × 43) / (4095 × 10) = count × 7740 / 4095
 * Calibrate against a known source on hardware; battery was tuned to 7510. */
#define SOLAR_VSOLAR_ADC_M_NUM          7740
#define SOLAR_VSOLAR_ADC_M_DEN          4095
#define SOLAR_VSOLAR_ADC_C              0

/* ADC → mV for RSENSE: no divider, 1.8 V ref
 * count × 1800 / 4095  (~0.44 mV/count; ~0.29 mA/count at 1.5 Ω) */
#define SOLAR_RSENSE_ADC_M_NUM          1800
#define SOLAR_RSENSE_ADC_M_DEN          4095
#define SOLAR_RSENSE_ADC_C              0

/* Minimum solar power (mW) required to exit ProductionSleep */
#define SOLAR_ACTIVATION_POWER_MW       50

#endif /* WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_ */
