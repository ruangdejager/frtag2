/*
 * Movement.c
 *
 * Movement alarm algorithm — sensor sampling, HPF, alarm windows.
 * Pure algorithm layer — no RTOS dependencies.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hal_bsp.h"
#include "Movement.h"
#include "Movement_Driver.h"

#include "platform.h"

#include "dbg_log.h"
#include "math_func.h"


//! Movement algorithm axis info
typedef struct move_alg_axis_t {
	int8_t ai8DcBuf[MOVE_ALG_DC_BUF_SIZE];
	uint8_t u8DcBufIdx;
	int8_t i8DcLevel;
} move_alg_axis_t;

typedef struct acc_t
{
	move_alg_axis_t X;
	move_alg_axis_t Y;
	move_alg_axis_t Z;
	uint8_t au8EnvBuf[MOVE_ALG_ENV_BUF_SIZE];
	uint8_t u8EnvBufIdx;
	uint8_t u8EnvLevel;
} move_alg_t;
move_alg_t MoveAlg;

//! Scratchpad variables
uint8_t u8MoveTemp;
int16_t i16MoveTemp;

typedef enum _move_alarm_state_t {
	MOVE_ALARM_STATE_IDLE     = 0,
	MOVE_ALARM_STATE_WINDOW_A = 1,
	MOVE_ALARM_STATE_WINDOW_B = 2
} move_alarm_state_t;

static uint8_t u8MoveSensorHealthErrorCnt;
static uint8_t u8MoveSensorHealthAlertCnt;

typedef struct {
	uint16_t u16TmrHwm;
	uint8_t  u8HoldBackCnt;
	uint8_t  u8TriggerCnt;
} move_window_a_t;
move_window_a_t MoveWinA;

#define MOVE_HOLDBACK_BUF_SIZE 16
typedef struct _move_holdback {
	uint8_t u8Idx;
	move_holdback_event_t Events[MOVE_HOLDBACK_BUF_SIZE];
} move_holdback_t;
move_holdback_t MoveHoldback;

uint16_t u16MoveMaxLevelWindowTmr;
uint32_t u32MoveMinLevelWindowTmr;

uint8_t u8MoveHighActivityAlarmCnt;
uint8_t u8MoveHighActAlarmSuccessCnt;
uint8_t u8MoveNotActiveAlarmCnt;

#define MOVE_ALARM_TIME_OF_DAY_ACT_IDX(DayOfWeek, TimeOfDay)  ((4*(DayOfWeek))+(TimeOfDay))

time_of_day_t tTimeOfDayPrev;

#define MOVE_NOT_ALARM_FIFO_SIZE 4
typedef struct _move_not_alarm_fifo_t {
	move_not_alarm_t Buf[MOVE_NOT_ALARM_FIFO_SIZE];
	uint8_t u8Head;
	uint8_t u8Tail;
} move_not_alarm_fifo_t;

move_not_alarm_fifo_t MoveNotAlarmFifo;

uint8_t  u8MoveTemp;
uint32_t u32MoveTemp;

ccrContext ccrMoveTestTask;

typedef struct _acc_health_t {
	uint8_t       u8DevIdErrorSeqCnt;
	uint16_t      u16SampleDeltaErrorCnt;
	uint8_t       u8NoSampleErrorCnt;
	acc_t         AccPrevSample;
	bool          bErrorFlag;
	uint8_t       u8AlertCnt;
	time_of_day_t tAlertTimeOfDayPrev;
	bool          bAlertFlag;
} acc_health_t;
acc_health_t AccHealth;

const char acMoveNoActAlarmStateIdle []      = "IDLE";
const char acMoveNoActAlarmStateTriggered [] = "TRIGGERED";
const char acMoveNoActAlarmStateCancelled [] = "CLEARED";
const char * const acMoveNoActAlarmState [] = {
    acMoveNoActAlarmStateIdle,
    acMoveNoActAlarmStateTriggered,
    acMoveNoActAlarmStateCancelled
};

bool MOVE_bUpdateMovementLevel(uint8_t *pu8Level);
void MOVE_vEvalMovementLevel(void);
void MOVE_vWindowATick(void);

/* --------------------------------------------------------------------------
 * MOVE_vInit
 * -------------------------------------------------------------------------- */
void MOVE_vInit(void)
{
    MoveAlg.X.u8DcBufIdx = 0;
    MoveAlg.Y.u8DcBufIdx = 0;
    MoveAlg.Z.u8DcBufIdx = 0;
    MoveAlg.u8EnvBufIdx  = 0;

    ccrMoveTestTask = 0;

    MoveNotAlarmFifo.u8Head = 0;
    MoveNotAlarmFifo.u8Tail = 0;

    u16MoveMaxLevelWindowTmr = 0;
#ifndef SWITCH_POSITION_LOGGER
    u32MoveMinLevelWindowTmr = 60 * 25 * (uint32_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_WINDOW);
#else
    u32MoveMinLevelWindowTmr = 60 * 25 * 60;
#endif

    u8MoveHighActivityAlarmCnt   = 0;
    u8MoveHighActAlarmSuccessCnt = 0;
    u8MoveNotActiveAlarmCnt      = 0;

    memset(&MoveWinA, 0, sizeof(move_window_a_t));

    tTimeOfDayPrev = TIME_tGetDayTimeZoneNow();

    u8MoveSensorHealthErrorCnt = 0;
    u8MoveSensorHealthAlertCnt = 0;

    memset(&AccHealth, 0, sizeof(acc_health_t));
    AccHealth.tAlertTimeOfDayPrev = TIME_tGetDayTimeZoneNow();
}

/* --------------------------------------------------------------------------
 * MOVE_bTick — main module tick
 * -------------------------------------------------------------------------- */
bool MOVE_bTick(void)
{
    bool bBusy = false;

    MOVE_vEvalMovementLevel();
    MOVE_vWindowATick();

    /* Sensor health check #1: device ID */
    if (AccHealth.u8DevIdErrorSeqCnt < MOVE_SENSOR_ID_ERR_CNT_ALERT)
    {
        if (MOVE_DRIVER_u8GetAccelDeviceId() != ACC_WHO_AM_I_VALUE)
        {
            AccHealth.u8DevIdErrorSeqCnt++;
            if (AccHealth.u8DevIdErrorSeqCnt == MOVE_SENSOR_ID_ERR_CNT_ALERT &&
                !AccHealth.bErrorFlag)
            {
                AccHealth.bErrorFlag = true;
                DBG_LOG("+++ MOVE SENSOR ERROR (DEVICE_ID) - alert +++");
            }
        }
        else
        {
            AccHealth.u8DevIdErrorSeqCnt = 0;
        }
    }

    /* Sensor health check #2: no sample */
    if (AccHealth.u8NoSampleErrorCnt < MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT)
    {
        AccHealth.u8NoSampleErrorCnt++;
        if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_RESET)
        {
            ACC_vInit();
            DBG_LOG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - reset +++");
        }
        if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT &&
            !AccHealth.bErrorFlag)
        {
            AccHealth.bErrorFlag = true;
            DBG_LOG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - alert +++");
        }
    }

    /* Sensor health alerts */
    if ((AccHealth.bErrorFlag && AccHealth.u8AlertCnt == 0) ||
        (AccHealth.u8AlertCnt > 0 &&
         TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_MIDDAY &&
         AccHealth.tAlertTimeOfDayPrev == TIME_OF_DAY_MORNING &&
         AccHealth.u8AlertCnt < MOVE_SENSOR_ALERT_COUNT_MAX))
    {
        AccHealth.u8AlertCnt++;
        AccHealth.bAlertFlag = true;
        DBG_LOG("+++ MOVE SENSOR HEALTH ALARM (%u of %u) +++",
            AccHealth.u8AlertCnt, MOVE_SENSOR_ALERT_COUNT_MAX);
    }

    AccHealth.tAlertTimeOfDayPrev = TIME_tGetDayTimeZoneNow();

    /* Check for time-of-day change → reset no-movement window */
    if (tTimeOfDayPrev != TIME_tGetDayTimeZoneNow())
    {
        if (u32MoveMinLevelWindowTmr)
        {
#ifndef SWITCH_POSITION_LOGGER
            u32MoveMinLevelWindowTmr = 60 * 25 *
                (uint32_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_WINDOW);
#else
            u32MoveMinLevelWindowTmr = 60 * 25 * 60;
#endif
        }
        tTimeOfDayPrev = TIME_tGetDayTimeZoneNow();
    }

    return bBusy;
}

/* --------------------------------------------------------------------------
 * Sensor health peek / pull
 * -------------------------------------------------------------------------- */
bool MOVE_bSensorHealthAlertPeek(void)  { return AccHealth.bAlertFlag; }

bool MOVE_bSensorHealthAlertPull(void)
{
    bool bTemp = AccHealth.bAlertFlag;
    AccHealth.bAlertFlag = false;
    return bTemp;
}

bool MOVE_bSensorHealthOk(void) { return !AccHealth.bErrorFlag; }

/* --------------------------------------------------------------------------
 * MOVE_bUpdateMovementLevel — sample ACC FIFO, run HPF
 * -------------------------------------------------------------------------- */
bool MOVE_bUpdateMovementLevel(uint8_t *pu8Level)
{
    uint8_t u8;
    acc_t   AccMoveDev;

    if (ACC_u8NumSamplesInFifo() > 0)
    {
        AccHealth.u8NoSampleErrorCnt = 0;

        ACC_vGetAccSample(&AccMoveDev);

        AccMoveDev.i16.i16OutX /= MOVE_ACC_RAW_SCALE_VALUE;
        AccMoveDev.i16.i16OutY /= MOVE_ACC_RAW_SCALE_VALUE;
        AccMoveDev.i16.i16OutZ /= MOVE_ACC_RAW_SCALE_VALUE;

        AccMoveDev.i16.i16OutX = min(AccMoveDev.i16.i16OutX,  127);
        AccMoveDev.i16.i16OutX = max(AccMoveDev.i16.i16OutX, -128);
        AccMoveDev.i16.i16OutY = min(AccMoveDev.i16.i16OutY,  127);
        AccMoveDev.i16.i16OutY = max(AccMoveDev.i16.i16OutY, -128);
        AccMoveDev.i16.i16OutZ = min(AccMoveDev.i16.i16OutZ,  127);
        AccMoveDev.i16.i16OutZ = max(AccMoveDev.i16.i16OutZ, -128);

        u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
        MoveAlg.X.ai8DcBuf[MoveAlg.X.u8DcBufIdx++] = ((u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0);
        MoveAlg.X.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
        MoveAlg.Y.ai8DcBuf[MoveAlg.Y.u8DcBufIdx++] = ((u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0);
        MoveAlg.Y.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
        MoveAlg.Z.ai8DcBuf[MoveAlg.Z.u8DcBufIdx++] = ((u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0);
        MoveAlg.Z.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.X.ai8DcBuf[u8MoveTemp];
        MoveAlg.X.i8DcLevel = (i16MoveTemp / MOVE_ALG_DC_BUF_SIZE);

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.Y.ai8DcBuf[u8MoveTemp];
        MoveAlg.Y.i8DcLevel = (i16MoveTemp / MOVE_ALG_DC_BUF_SIZE);

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.Z.ai8DcBuf[u8MoveTemp];
        MoveAlg.Z.i8DcLevel = (i16MoveTemp / MOVE_ALG_DC_BUF_SIZE);

        u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx]  = (uint8_t)abs((int8_t)((u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0) - MoveAlg.X.i8DcLevel);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t)abs((int8_t)((u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0) - MoveAlg.Y.i8DcLevel);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t)abs((int8_t)((u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0) - MoveAlg.Z.i8DcLevel);
        MoveAlg.u8EnvBufIdx = (MoveAlg.u8EnvBufIdx + 1) % MOVE_ALG_ENV_BUF_SIZE;

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_ENV_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.au8EnvBuf[u8MoveTemp];
        MoveAlg.u8EnvLevel = (uint8_t)(i16MoveTemp / MOVE_ALG_ENV_BUF_SIZE);

        *pu8Level = MoveAlg.u8EnvLevel;

        /* Health check #3: no delta */
        for (u8 = 0; u8 < sizeof(acc_t); u8++)
        {
            if (AccHealth.AccPrevSample.au8Data[u8] != AccMoveDev.au8Data[u8])
            {
                memcpy(&AccHealth.AccPrevSample, &AccMoveDev, sizeof(acc_t));
                AccHealth.u16SampleDeltaErrorCnt = 0;
                break;
            }
        }
        if (u8 == sizeof(acc_t) &&
            AccHealth.u16SampleDeltaErrorCnt < MOVE_SENSOR_DELTA_ERR_CNT_ALERT)
        {
            AccHealth.u16SampleDeltaErrorCnt++;
            if (AccHealth.u16SampleDeltaErrorCnt == MOVE_SENSOR_DELTA_ERR_CNT_RESET)
            {
                ACC_vInit();
                DBG_LOG("+++ MOVE SENSOR ERROR (NO_DELTA) - reset +++");
            }
            else if (AccHealth.u16SampleDeltaErrorCnt == MOVE_SENSOR_DELTA_ERR_CNT_ALERT &&
                     !AccHealth.bErrorFlag)
            {
                AccHealth.bErrorFlag = true;
                DBG_LOG("+++ MOVE SENSOR ERROR (NO_DELTA) - alert +++");
            }
        }

        return true;
    }
    return false;
}

#ifdef SWITCH_MOVE_SENSOR_DATA
int8_t MOVE_i8SensorDataX(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutX; }
int8_t MOVE_i8SensorDataY(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutY; }
int8_t MOVE_i8SensorDataZ(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutZ; }
#endif

bool MOVE_bSetSensorAxis(uint8_t u8AxisMask)
{
    if (u8AxisMask & 0x07)
    {
        SETTINGS_vSetByte(movementAlarm.accelerometerAxes, u8AxisMask);
        return true;
    }
    return false;
}

uint8_t MOVE_u8HighActivityAlarmCnt(void)     { return u8MoveHighActivityAlarmCnt; }
uint8_t MOVE_u8HighActAlarmSuccessCnt(void)   { return u8MoveHighActAlarmSuccessCnt; }
void    MOVE_vHighActAlarmSuccessIncr(void)   { u8MoveHighActAlarmSuccessCnt++; }
uint8_t MOVE_u8NotActiveAlarmCnt(void)        { return u8MoveNotActiveAlarmCnt; }

/* --------------------------------------------------------------------------
 * MOVE_vEvalMovementLevel — sample & process movement, queue alarms
 * -------------------------------------------------------------------------- */
void MOVE_vEvalMovementLevel(void)
{
    uint8_t u8Level;

    while (MOVE_bUpdateMovementLevel(&u8Level))
    {
        if (u8Level >= MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_LEVEL))
        {
            if (MOVE_bIsAlarmActiveNow())
            {
                u16MoveMaxLevelWindowTmr++;
                MoveWinA.u16TmrHwm = max(MoveWinA.u16TmrHwm, u16MoveMaxLevelWindowTmr);

                if (u16MoveMaxLevelWindowTmr >=
                    (25 * (uint16_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW)))
                {
                    u16MoveMaxLevelWindowTmr = 0;
                    MoveWinA.u16TmrHwm       = 0;
                    MoveWinA.u8TriggerCnt++;
                    MoveWinA.u8HoldBackCnt = (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_FIRST_HOLD) +
                                             (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_SECOND_HOLD);

                    if (((MoveWinA.u8TriggerCnt == 1) &&
                         (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_FIRST_HOLD)) ||
                        ((MoveWinA.u8TriggerCnt == 2) &&
                         (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_SECOND_HOLD)))
                    {
                        DBG_LOG("--- high act alarm | hold (%u/%u) ---",
                            MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt);

                        if (MoveHoldback.u8Idx < MOVE_HOLDBACK_BUF_SIZE &&
                            MoveWinA.u8TriggerCnt == 1)
                        {
                            MoveHoldback.Events[MoveHoldback.u8Idx].u32Ts = RTC_u64GetUTC();
                        }
                    }
                    else
                    {
                        u8MoveHighActivityAlarmCnt++;
                    }
                }
            }
        }
        else
        {
            MOVE_DRIVER_vLedOff();
            if (u16MoveMaxLevelWindowTmr) u16MoveMaxLevelWindowTmr--;
        }

        if (u8Level < MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_LEVEL))
        {
            if (u32MoveMinLevelWindowTmr)
            {
                u32MoveMinLevelWindowTmr--;
                if (u32MoveMinLevelWindowTmr == 0)
                {
                    MOVE_bNoActAlertPush(MOVE_NOT_ALARM_STATE_TRIGGERED);
                    u8MoveNotActiveAlarmCnt++;
                }
            }
        }
        else
        {
            if (u32MoveMinLevelWindowTmr == 0 && u8Level >= 5)
                MOVE_bNoActAlertPush(MOVE_NOT_ALARM_STATE_CANCELLED);

            if (u32MoveMinLevelWindowTmr || (u32MoveMinLevelWindowTmr == 0 && u8Level >= 5))
            {
#ifndef SWITCH_POSITION_LOGGER
                u32MoveMinLevelWindowTmr = 60 * 25 *
                    (uint32_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_WINDOW);
#else
                u32MoveMinLevelWindowTmr = 60 * 25 * 60;
#endif
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * MOVE_vWindowATick — window A state machine
 * -------------------------------------------------------------------------- */
void MOVE_vWindowATick(void)
{
    if (MoveWinA.u8TriggerCnt)
    {
        if (MoveWinA.u8TriggerCnt <= MoveWinA.u8HoldBackCnt)
        {
            DBG_LOG("--- high act alarm | cancel (hold=%u/%u, hwm=%u/%us) ---",
                MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt,
                MoveWinA.u16TmrHwm / 25,
                MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW));

            if (MoveHoldback.u8Idx < MOVE_HOLDBACK_BUF_SIZE)
            {
                MoveHoldback.Events[MoveHoldback.u8Idx].u8TrigCnt = MoveWinA.u8TriggerCnt;
                MoveHoldback.Events[MoveHoldback.u8Idx].u8HoldCnt = MoveWinA.u8HoldBackCnt;
                MoveHoldback.Events[MoveHoldback.u8Idx].u8WinHwm  = MoveWinA.u16TmrHwm / 25;
                MoveHoldback.Events[MoveHoldback.u8Idx].u8WinLen  =
                    MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW);
                MoveHoldback.u8Idx++;
            }
        }
        else
        {
            DBG_LOG("--- high act alarm | done (trig=%u, hold=%u) ---",
                MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt);
        }
        memset(&MoveWinA, 0, sizeof(move_window_a_t));
    }
}

/* --------------------------------------------------------------------------
 * MOVE_u8GetAlarmSettingNow
 * -------------------------------------------------------------------------- */
uint8_t MOVE_u8GetAlarmSettingNow(move_alarm_setting_t tMoveAlarmSetting)
{
    if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_MORNING)
    {
        switch (tMoveAlarmSetting)
        {
            case MOVE_ALARM_SETTING_MAX_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.maxLevel);
            case MOVE_ALARM_SETTING_MAX_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.maxWindow);
            case MOVE_ALARM_SETTING_FIRST_HOLD:  return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.holdFirst);
            case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.holdSecond);
            case MOVE_ALARM_SETTING_MIN_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.minLevel);
            case MOVE_ALARM_SETTING_MIN_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.minWindow);
        }
    }
    if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_MIDDAY)
    {
        switch (tMoveAlarmSetting)
        {
            case MOVE_ALARM_SETTING_MAX_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.maxLevel);
            case MOVE_ALARM_SETTING_MAX_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.maxWindow);
            case MOVE_ALARM_SETTING_FIRST_HOLD:  return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.holdFirst);
            case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.holdSecond);
            case MOVE_ALARM_SETTING_MIN_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.minLevel);
            case MOVE_ALARM_SETTING_MIN_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.minWindow);
        }
    }
    if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_AFTERNOON)
    {
        switch (tMoveAlarmSetting)
        {
            case MOVE_ALARM_SETTING_MAX_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.maxLevel);
            case MOVE_ALARM_SETTING_MAX_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.maxWindow);
            case MOVE_ALARM_SETTING_FIRST_HOLD:  return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.holdFirst);
            case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.holdSecond);
            case MOVE_ALARM_SETTING_MIN_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.minLevel);
            case MOVE_ALARM_SETTING_MIN_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.minWindow);
        }
    }
    if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_NIGHT)
    {
        switch (tMoveAlarmSetting)
        {
            case MOVE_ALARM_SETTING_MAX_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.maxLevel);
            case MOVE_ALARM_SETTING_MAX_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.maxWindow);
            case MOVE_ALARM_SETTING_FIRST_HOLD:  return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.holdFirst);
            case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.holdSecond);
            case MOVE_ALARM_SETTING_MIN_LEVEL:   return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.minLevel);
            case MOVE_ALARM_SETTING_MIN_WINDOW:  return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.minWindow);
        }
    }
    return 0;
}

bool MOVE_bIsAlarmActiveNow(void)
{
    return (bool)(SETTINGS_u32GetWord(movementAlarm.dayTimeActive) &
                  ((uint32_t)1 << (TIME_u8GetDayOfWeek() * 4 + TIME_tGetDayTimeZoneNow())));
}

bool MOVE_bIsActive(void) { return u32MoveMinLevelWindowTmr != 0; }

/* --------------------------------------------------------------------------
 * No-movement alarm FIFO
 * -------------------------------------------------------------------------- */
bool MOVE_bNoActAlertPush(move_not_alarm_state_t tState)
{
    uint8_t u8 = ((MoveNotAlarmFifo.u8Head + 1) % MOVE_NOT_ALARM_FIFO_SIZE);
    if (u8 != MoveNotAlarmFifo.u8Tail)
    {
        MoveNotAlarmFifo.Buf[MoveNotAlarmFifo.u8Head].tState = tState;
        MoveNotAlarmFifo.u8Head = u8;
        DBG_LOG("+++ NO ACTIVITY ALARM (%s) +++", (char *)acMoveNoActAlarmState[tState]);
        return true;
    }
    DBG_LOG("*** no activity alarm buffer ERROR ***");
    return false;
}

move_not_alarm_t *MOVE_pNoActAlertPeek(void)
{
    if (MoveNotAlarmFifo.u8Head != MoveNotAlarmFifo.u8Tail)
        return &MoveNotAlarmFifo.Buf[MoveNotAlarmFifo.u8Tail];
    return NULL;
}

bool MOVE_bNoActAlertPull(move_not_alarm_t *pMoveNotAlarm)
{
    if (MoveNotAlarmFifo.u8Head != MoveNotAlarmFifo.u8Tail)
    {
        if (pMoveNotAlarm)
            memcpy(pMoveNotAlarm, &MoveNotAlarmFifo.Buf[MoveNotAlarmFifo.u8Tail],
                   sizeof(move_not_alarm_t));
        MoveNotAlarmFifo.u8Tail = (MoveNotAlarmFifo.u8Tail + 1) % MOVE_NOT_ALARM_FIFO_SIZE;
        return true;
    }
    return false;
}

/* --------------------------------------------------------------------------
 * Hold-back buffer
 * -------------------------------------------------------------------------- */
uint8_t               MOVE_u8HoldbackBufLen(void)      { return MoveHoldback.u8Idx; }
move_holdback_event_t *MOVE_pGetHoldbackBuf(void)      { return MoveHoldback.Events; }
void                  MOVE_vClearHoldbackBuf(void)     { memset(&MoveHoldback, 0, sizeof(move_holdback_t)); }
