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

/* ---- Power class bit flags ---- */
#define POWER_CLASS_NORMAL    (1UL << 0)
#define POWER_CLASS_RECOVERY  (1UL << 1)
#define POWER_CLASS_ALWAYS    (1UL << 2)

void     POWER_vInit(void);
void     POWER_vSetModeNormal(void);
void     POWER_vSetModeRecovery(void);
void     POWER_vWaitForClass(uint32_t classMask);
uint32_t POWER_tGetState(void);

#endif /* WORKER_POWER_POWER_H_ */
