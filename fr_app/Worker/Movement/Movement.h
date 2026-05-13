/*
 * Movement.h
 *
 * Movement alarm algorithm — public interface.
 * Pure algorithm layer — no RTOS dependencies.
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_H_
#define WORKER_MOVEMENT_MOVEMENT_H_

#include "Movement_Config.h"

/* ---- Alarm setting indices ---- */
typedef enum {
    MOVE_ALARM_SETTING_MAX_LEVEL   = 0,
    MOVE_ALARM_SETTING_MAX_WINDOW,
    MOVE_ALARM_SETTING_FIRST_HOLD,
    MOVE_ALARM_SETTING_SECOND_HOLD,
    MOVE_ALARM_SETTING_MIN_LEVEL,
    MOVE_ALARM_SETTING_MIN_WINDOW
} move_alarm_setting_t;

/* ---- Per-daytimezone alarm settings ---- */
typedef struct move_settings_dtz_t {
    uint8_t u8MaxLevel;
    uint8_t u8MaxWindow;
    uint8_t u8HoldFirstCall;
    uint8_t u8HoldSecondCall;
    uint8_t u8MinLevel;
    uint8_t u8MinWindow;
} move_settings_dtz_t;

/* ---- No-movement alarm state ---- */
typedef enum _move_not_alarm_state_t {
    MOVE_NOT_ALARM_STATE_IDLE      = 0,
    MOVE_NOT_ALARM_STATE_TRIGGERED = 1,
    MOVE_NOT_ALARM_STATE_CANCELLED = 2
} move_not_alarm_state_t;

typedef struct _move_not_alarm_t {
    move_not_alarm_state_t tState;
} move_not_alarm_t;

/* ---- Hold-back event ---- */
typedef struct _move_holdback_event_t {
    uint32_t u32Ts;
    uint8_t  u8TrigCnt;
    uint8_t  u8HoldCnt;
    uint8_t  u8WinHwm;
    uint8_t  u8WinLen;
} move_holdback_event_t;

/* ---- Index macro ---- */
#define MOVE_ALARM_SETTING_IDX(TimeOfDay, Setting)  ((6 * (TimeOfDay)) + (Setting))

/* ---- Public API ---- */
void    MOVE_vInit(void);
bool    MOVE_bTick(void);
bool    MOVE_bIsActive(void);
bool    MOVE_bIsAlarmActiveNow(void);
uint8_t MOVE_u8GetAlarmSettingNow(move_alarm_setting_t tMoveAlarmSetting);
bool    MOVE_bSetSensorAxis(uint8_t u8AxisMask);

bool              MOVE_bNoActAlertPush(move_not_alarm_state_t tState);
move_not_alarm_t *MOVE_pNoActAlertPeek(void);
bool              MOVE_bNoActAlertPull(move_not_alarm_t *pMoveNotAlarm);

bool MOVE_bSensorHealthAlertPeek(void);
bool MOVE_bSensorHealthAlertPull(void);
bool MOVE_bSensorHealthOk(void);

uint8_t              MOVE_u8HighActivityAlarmCnt(void);
uint8_t              MOVE_u8HighActAlarmSuccessCnt(void);
void                 MOVE_vHighActAlarmSuccessIncr(void);
uint8_t              MOVE_u8NotActiveAlarmCnt(void);

uint8_t              MOVE_u8HoldbackBufLen(void);
move_holdback_event_t *MOVE_pGetHoldbackBuf(void);
void                 MOVE_vClearHoldbackBuf(void);

#ifdef SWITCH_MOVE_SENSOR_DATA
int8_t MOVE_i8SensorDataX(void);
int8_t MOVE_i8SensorDataY(void);
int8_t MOVE_i8SensorDataZ(void);
#endif

#endif /* WORKER_MOVEMENT_MOVEMENT_H_ */
