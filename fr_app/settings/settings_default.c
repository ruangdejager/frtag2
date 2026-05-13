/*
 * settings_default.c
 *
 * Compile-time default values for all persistent settings.
 * settings.c copies this struct into RAM on init; any field not explicitly
 * set here will be zero / false.
 */

#include "settings_default.h"
#include "Movement_Config.h"
#include "time_config.h"

Settings_EE settings_def =
{
    .utcTime      = 1,
    .serialNumber = "700000",

    .time.timezoneAdjust  = TIME_TZ_GMT_ADJ_DEF,
    .time.summerStartMonth = SUMMER_START_MON_DEF,
    .time.winterStartMonth = WINTER_START_MON_DEF,

    .dayTimeZone.summerMorningStartTime   = MORNING_START_TIME_DEF_SUMMER,
    .dayTimeZone.summerMiddayStartTime    = MIDDAY_START_TIME_DEF_SUMMER,
    .dayTimeZone.summerAfternoonStartTime = AFTERNOON_START_TIME_DEF_SUMMER,
    .dayTimeZone.summerNightStartTime     = NIGHT_START_TIME_DEF_SUMMER,
    .dayTimeZone.winterMorningStartTime   = MORNING_START_TIME_DEF_WINTER,
    .dayTimeZone.winterMiddayStartTime    = MIDDAY_START_TIME_DEF_WINTER,
    .dayTimeZone.winterAfternoonStartTime = AFTERNOON_START_TIME_DEF_WINTER,
    .dayTimeZone.winterNightStartTime     = NIGHT_START_TIME_DEF_WINTER,

    .movementAlarm.dayTimeActive  = MOVE_ALARM_TIME_OF_DAY_ACT_DEF,
    .movementAlarm.windowSensitive = MOVE_ALARM_WINDOW_A_TIME_DEF,
    .movementAlarm.windowTrack    = MOVE_ALARM_WINDOW_B_TIME_DEF,
    .movementAlarm.maxCount       = MOVE_HIGH_ACT_ALERTS_MAX_DEF,

    .movementAlarm.morningZoneLevels.maxLevel   = MOVE_MAX_LEVEL_DTZ0_DEF,
    .movementAlarm.morningZoneLevels.maxWindow  = MOVE_MAX_WINDOW_DTZ0_DEF,
    .movementAlarm.morningZoneLevels.holdFirst  = MOVE_MAX_HOLD_FIRST_DTZ0_DEF,
    .movementAlarm.morningZoneLevels.holdSecond = MOVE_MAX_HOLD_SECOND_DTZ0_DEF,
    .movementAlarm.morningZoneLevels.minLevel   = MOVE_MIN_LEVEL_DTZ0_DEF,
    .movementAlarm.morningZoneLevels.minWindow  = MOVE_MIN_WINDOW_DTZ0_DEF,

    .movementAlarm.middayZoneLevels.maxLevel    = MOVE_MAX_LEVEL_DTZ1_DEF,
    .movementAlarm.middayZoneLevels.maxWindow   = MOVE_MAX_WINDOW_DTZ1_DEF,
    .movementAlarm.middayZoneLevels.holdFirst   = MOVE_MAX_HOLD_FIRST_DTZ1_DEF,
    .movementAlarm.middayZoneLevels.holdSecond  = MOVE_MAX_HOLD_SECOND_DTZ1_DEF,
    .movementAlarm.middayZoneLevels.minLevel    = MOVE_MIN_LEVEL_DTZ1_DEF,
    .movementAlarm.middayZoneLevels.minWindow   = MOVE_MIN_WINDOW_DTZ1_DEF,

    .movementAlarm.afternoonZoneLevels.maxLevel   = MOVE_MAX_LEVEL_DTZ2_DEF,
    .movementAlarm.afternoonZoneLevels.maxWindow  = MOVE_MAX_WINDOW_DTZ2_DEF,
    .movementAlarm.afternoonZoneLevels.holdFirst  = MOVE_MAX_HOLD_FIRST_DTZ2_DEF,
    .movementAlarm.afternoonZoneLevels.holdSecond = MOVE_MAX_HOLD_SECOND_DTZ2_DEF,
    .movementAlarm.afternoonZoneLevels.minLevel   = MOVE_MIN_LEVEL_DTZ2_DEF,
    .movementAlarm.afternoonZoneLevels.minWindow  = MOVE_MIN_WINDOW_DTZ2_DEF,

    .movementAlarm.nightZoneLevels.maxLevel    = MOVE_MAX_LEVEL_DTZ3_DEF,
    .movementAlarm.nightZoneLevels.maxWindow   = MOVE_MAX_WINDOW_DTZ3_DEF,
    .movementAlarm.nightZoneLevels.holdFirst   = MOVE_MAX_HOLD_FIRST_DTZ3_DEF,
    .movementAlarm.nightZoneLevels.holdSecond  = MOVE_MAX_HOLD_SECOND_DTZ3_DEF,
    .movementAlarm.nightZoneLevels.minLevel    = MOVE_MIN_LEVEL_DTZ3_DEF,
    .movementAlarm.nightZoneLevels.minWindow   = MOVE_MIN_WINDOW_DTZ3_DEF,

    .movementAlarm.accelerometerAxes           = MOVE_SENSOR_AXIS_DEF,
    .movementAlarm.slowMoveAlarmTimeOfDayActive = UINT32_C(0x08888888),
    .movementAlarm.slowMoveMaxLevel            = 8,
    .movementAlarm.slowMoveMaxWindow           = 15,
};
