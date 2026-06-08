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

/* ADC full-scale code (12-bit) */
#define SOLAR_ADC_FULL_SCALE            4095U

/* VSOLAR resistor divider: 33k top / 10k bottom → (33+10)/10 = 4.3.
 *   Vpin   = adc_vsolar * VDDA_measured / SOLAR_ADC_FULL_SCALE
 *   Vsolar = Vpin * SOLAR_VSOLAR_DIV_NUM / SOLAR_VSOLAR_DIV_DEN
 * VDDA comes from the calibrated VREFINT channel (same as the battery worker),
 * so the result no longer assumes an exact 1.8 V rail. */
#define SOLAR_VSOLAR_DIV_NUM            43U
#define SOLAR_VSOLAR_DIV_DEN            10U

/* RSENSE has no divider — the pin voltage is the sense voltage directly:
 *   Vrsense = adc_rsense * VDDA_measured / SOLAR_ADC_FULL_SCALE */

/* Minimum solar power (mW) required to exit ProductionSleep */
#define SOLAR_ACTIVATION_POWER_MW       50

#endif /* WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_ */
