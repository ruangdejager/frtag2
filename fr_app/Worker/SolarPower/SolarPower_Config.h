/*
 * SolarPower_Config.h
 *
 * Solar power measurement parameters.
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_
#define WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_

/*
 * Panel power sensing (RSENSE shunt -> current / power / coulomb counting) is
 * DISABLED on this board revision. The low-side shunt sits in the solar-return
 * path (Vsolar- -> Rsense -> GND), so charge current drives the sense node
 * (PB4 / ADC_IN3) NEGATIVE with respect to the MCU's only ground reference, and
 * the single-ended ADC clamps it to 0 — the current/power/coulomb figures are
 * therefore meaningless. Rsense is fitted as a 0 Ω link and PB4 is parked as an
 * unused analog input (see hal_adc.c HAL_ADC_MspInit).
 *
 * Vsolar (panel voltage) is unaffected by this and remains enabled.
 *
 * Define ENABLE_SOLAR_POWER_SENSE on a future board revision that corrects the
 * front-end (e.g. a ground-referenced current-sense amplifier across the shunt)
 * to restore the RSENSE measurement and all current/power/coulomb code below.
 */
// #define ENABLE_SOLAR_POWER_SENSE

/* Sample rate (s) — one measurement per heartbeat cycle */
#define SOLAR_SAMPLE_INTERVAL           10

/*
 * The 10 s sample task can also write a "solar: ..." line to the flash log on
 * every sample. On a PRIMARY that line lands in the middle of every discovery
 * campaign and fills the log with periodic noise. Off by default — a single
 * solar reading is logged once per production wake instead (see
 * DeviceDiscovery.c). Define this to restore the old per-sample logging for
 * bench debugging.
 */
// #define LOG_SOLAR_PERIODIC

#ifdef ENABLE_SOLAR_POWER_SENSE
/* Sense resistor: 1.5 Ω, 1% — stored ×100 for integer arithmetic */
#define SOLAR_RSENSE_OHM_X100           150
#endif

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

#ifdef ENABLE_SOLAR_POWER_SENSE
/* Minimum solar power (mW) required to exit ProductionSleep */
#define SOLAR_ACTIVATION_POWER_MW       50
#else
/* Power sensing disabled — exit ProductionSleep on panel VOLTAGE instead.
 * Minimum Vsolar (mV) that counts as "panel illuminated / charging".
 * NOTE: tune to this board's behaviour — Vsolar tracks the charge rail, so it
 * sits near the battery voltage and rises when the panel actively drives it. */
#define SOLAR_ACTIVATION_VSOLAR_MV      4000U
#endif

#endif /* WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_ */
