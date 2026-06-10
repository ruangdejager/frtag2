/*
 * Power.h
 *
 * Power class management — battery-voltage driven mode switching.
 *
 * Power classes are stored in an osEventFlags object.
 * Callers can block on POWER_vWaitForClass() until the required class is set.
 *
 * Uses CMSIS-RTOS v2 throughout.
 */

#ifndef WORKER_POWER_POWER_H_
#define WORKER_POWER_POWER_H_

#include <stdint.h>
#include "platform.h"

/* ---- Power class bit flags ----
 *
 *   NORMAL   : fully operational (default)               V_bat ≥ EXIT_LOW_MV
 *   LOW      : degraded — heavy ops (GPS) disabled       3400 < V_bat < 3500 (hyst)
 *   RECOVERY : critical — only baseline activity         V_bat ≤ ENTER_RECOVERY_MV
 *   ALWAYS   : always-on baseline bit, set alongside the active class
 *
 * Class precedence: NORMAL > LOW > RECOVERY. Exactly one of
 * {NORMAL, LOW, RECOVERY} is set at any time. ALWAYS is set in all three.
 */
#define POWER_CLASS_NORMAL    (1UL << 0)
#define POWER_CLASS_RECOVERY  (1UL << 1)
#define POWER_CLASS_ALWAYS    (1UL << 2)
#define POWER_CLASS_LOW       (1UL << 3)

void     POWER_vInit(void);
void     POWER_vSetModeNormal(void);
void     POWER_vSetModeLow(void);
void     POWER_vSetModeRecovery(void);
void     POWER_vWaitForClass(uint32_t classMask);
uint32_t POWER_tGetState(void);

#endif /* WORKER_POWER_POWER_H_ */
