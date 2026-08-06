/*
 * Movement_Driver.h
 *
 * Board-level driver macros for the movement algorithm.
 * Maps abstract accelerometer operations to the ACC device layer.
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_DRIVER_H_
#define WORKER_MOVEMENT_MOVEMENT_DRIVER_H_

#include "hal_bsp.h"
#include "hal_system.h"
#include <stdbool.h>
#include "Acc.h"

#define MOVE_DRIVER_acclSampleTypedef       acc_t

#define MOVE_DRIVER_u8GetAccelDeviceId()    ACC_u8GetDeviceId()
#define MOVE_DRIVER_u8NumSamplesInFifo()    ACC_u8NumSamplesInFifo()
#define MOVE_DRIVER_vGetAccSample(x)        ACC_vGetAccSample(x)
#define MOVE_DRIVER_vAccelInit()            ACC_vInit()

#define MOVE_DRIVER_vUSBPutValues           HAL_USB_vPutMoveMessage

/* Shake-sequence step/complete confirmation flash. Always blinks whichever
 * LED is NOT the current sleep indicator (HAL_SYSTEM_eGetSleepIndicatorLed):
 * yellow normally (indicator is red), but red instead during
 * ProductionSleep/SolarSleep (indicator is yellow there) — otherwise the
 * STOP2 entry/exit and 2 Hz lock-held pulsing that also drive yellow in
 * that mode would stomp this flash and the operator shaking the device
 * awake would never see step confirmation. */
static inline void MOVE_DRIVER_vLedOn(void)
{
    if (HAL_SYSTEM_eGetSleepIndicatorLed() == HAL_SYSTEM_SLEEP_LED_RED)
        BSP_LED_On(LED_YELLOW);
    else
        BSP_LED_On(LED_RED);
}

static inline void MOVE_DRIVER_vLedOff(void)
{
    if (HAL_SYSTEM_eGetSleepIndicatorLed() == HAL_SYSTEM_SLEEP_LED_RED)
        BSP_LED_Off(LED_YELLOW);
    else
        BSP_LED_Off(LED_RED);
}

#endif /* WORKER_MOVEMENT_MOVEMENT_DRIVER_H_ */
