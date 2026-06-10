/*
 * Movement.h
 *
 * Movement worker — public interface.
 * Runs as an RTOS task subscribed to the 1 Hz platform heartbeat.
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_H_
#define WORKER_MOVEMENT_MOVEMENT_H_

#include <stdint.h>
#include <stdbool.h>
#include "Movement_Config.h"

/* ---- Motion state ---- */
typedef enum {
    MOVE_STATE_MOVING = 0,
    MOVE_STATE_STILL  = 1,
} MoveState_e;

/* ---- Public API ---- */
void        MOVE_vInit(void);
MoveState_e MOVE_eGetState(void);

#ifdef SWITCH_MOVE_SENSOR_DATA
int8_t MOVE_i8SensorDataX(void);
int8_t MOVE_i8SensorDataY(void);
int8_t MOVE_i8SensorDataZ(void);
#endif

#endif /* WORKER_MOVEMENT_MOVEMENT_H_ */
