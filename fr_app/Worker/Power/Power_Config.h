/*
 * Power_Config.h
 *
 * Battery voltage thresholds for normal / recovery power modes.
 */

#ifndef WORKER_POWER_POWER_CONFIG_H_
#define WORKER_POWER_POWER_CONFIG_H_

/* RECOVERY ↔ LOW boundary (existing — 50 mV hysteresis) */
#define ENTER_RECOVERY_MV   3400
#define EXIT_RECOVERY_MV    3450

/* LOW ↔ NORMAL boundary (new — 20 mV hysteresis).
 * GPS and other heavy operations gate strictly on POWER_CLASS_NORMAL
 * so they idle out in LOW; scheduled wakes still fire in LOW. */
#define ENTER_LOW_MV        3500
#define EXIT_LOW_MV         3520

#endif /* WORKER_POWER_POWER_CONFIG_H_ */
