/*
 * settings_default.h
 *
 * Compile-time default settings structure definitions.
 * Used by settings.c to initialise RAM settings on first boot or after a
 * settings corruption check fails.
 */

#ifndef SETTINGS_SETTINGS_DEFAULT_H_
#define SETTINGS_SETTINGS_DEFAULT_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct _Settings_EE_Time {
    bool    has_timezoneAdjust;
    int32_t timezoneAdjust;
    bool    has_summerStartMonth;
    uint32_t summerStartMonth;
    bool    has_winterStartMonth;
    uint32_t winterStartMonth;
} Settings_EE_Time;

typedef struct _Settings_EE_DayTimeZone {
    bool    has_summerMorningStartTime;
    int32_t summerMorningStartTime;
    bool    has_summerMiddayStartTime;
    int32_t summerMiddayStartTime;
    bool    has_summerAfternoonStartTime;
    int32_t summerAfternoonStartTime;
    bool    has_summerNightStartTime;
    int32_t summerNightStartTime;
    bool    has_winterMorningStartTime;
    int32_t winterMorningStartTime;
    bool    has_winterMiddayStartTime;
    int32_t winterMiddayStartTime;
    bool    has_winterAfternoonStartTime;
    int32_t winterAfternoonStartTime;
    bool    has_winterNightStartTime;
    int32_t winterNightStartTime;
} Settings_EE_DayTimeZone;

typedef struct _Settings_EE_MovementAlarm_ZoneLevels {
    bool     has_maxLevel;
    uint32_t maxLevel;
    bool     has_maxWindow;
    uint32_t maxWindow;
    bool     has_holdFirst;
    uint32_t holdFirst;
    bool     has_holdSecond;
    bool     holdSecond;
    bool     has_minLevel;
    uint32_t minLevel;
    bool     has_minWindow;
    uint32_t minWindow;
} Settings_EE_MovementAlarm_ZoneLevels;

typedef struct _Settings_EE_MovementAlarm {
    bool     has_type;
    bool     has_gps;
    bool     gps;
    bool     has_dayTimeActive;
    uint32_t dayTimeActive;
    bool     has_windowSensitive;
    uint32_t windowSensitive;
    bool     has_windowTrack;
    uint32_t windowTrack;
    bool     has_maxCount;
    uint32_t maxCount;
    bool     has_morningZoneLevels;
    Settings_EE_MovementAlarm_ZoneLevels morningZoneLevels;
    bool     has_middayZoneLevels;
    Settings_EE_MovementAlarm_ZoneLevels middayZoneLevels;
    bool     has_afternoonZoneLevels;
    Settings_EE_MovementAlarm_ZoneLevels afternoonZoneLevels;
    bool     has_nightZoneLevels;
    Settings_EE_MovementAlarm_ZoneLevels nightZoneLevels;
    bool     has_noMovementAlarmCount;
    uint32_t noMovementAlarmCount;
    bool     has_accelerometerAxes;
    uint8_t  accelerometerAxes;
    bool     has_slowMoveAlarmTimeOfDayActive;
    uint32_t slowMoveAlarmTimeOfDayActive;
    bool     has_slowMoveMaxLevel;
    uint8_t  slowMoveMaxLevel;
    bool     has_slowMoveMaxWindow;
    uint8_t  slowMoveMaxWindow;
} Settings_EE_MovementAlarm;

typedef struct _Settings_EE {
    uint32_t utcTime;
    char     serialNumber[9];
    bool     has_time;
    Settings_EE_Time time;
    bool     has_dayTimeZone;
    Settings_EE_DayTimeZone dayTimeZone;
    bool     has_movementAlarm;
    Settings_EE_MovementAlarm movementAlarm;
} Settings_EE;

typedef struct _DayTimeZoneSchedule {
    uint32_t dtzActiveBitMask;
} DayTimeZoneSchedule;

typedef enum _MoMessage_Event_MovementAlarmParameter_MovementAlarmState {
    MoMessage_Event_MovementAlarmParameter_MovementAlarmState_RESERVED  = 0,
    MoMessage_Event_MovementAlarmParameter_MovementAlarmState_TRIGGERED = 1,
    MoMessage_Event_MovementAlarmParameter_MovementAlarmState_TRACKING  = 2,
    MoMessage_Event_MovementAlarmParameter_MovementAlarmState_EXPIRED   = 3,
    MoMessage_Event_MovementAlarmParameter_MovementAlarmState_SLOW_MOVE = 4,
} MoMessage_Event_MovementAlarmParameter_MovementAlarmState;

#endif /* SETTINGS_SETTINGS_DEFAULT_H_ */
