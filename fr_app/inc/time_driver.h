/*
 * time_driver.h
 *
 * Time, season, and day-time-zone (DTZ) management.
 */

#ifndef TIME_DRIVER_H_
#define TIME_DRIVER_H_

#include "time_config.h"
#include "settings_default.h"
#include "platform.h"
#include "datetime.h"

/* Season */
typedef enum {
    TIME_SEASON_SUMMER = 0,
    TIME_SEASON_WINTER
} time_season_t;

/* Day of week */
typedef enum {
    TIME_DAY_OF_WEEK_SUN = 0,
    TIME_DAY_OF_WEEK_MON,
    TIME_DAY_OF_WEEK_TUE,
    TIME_DAY_OF_WEEK_WED,
    TIME_DAY_OF_WEEK_THU,
    TIME_DAY_OF_WEEK_FRI,
    TIME_DAY_OF_WEEK_SAT
} time_day_of_week_t;

/* Time-of-day zone */
typedef enum {
    TIME_OF_DAY_MORNING = 0,
    TIME_OF_DAY_MIDDAY,
    TIME_OF_DAY_AFTERNOON,
    TIME_OF_DAY_NIGHT
} time_of_day_t;

#define TIME_SEASON_START_MONTH_IDX(Season)          (Season)
#define TIME_OF_DAY_START_IDX(Season, TimeOfDay)     ((4 * (Season)) + (TimeOfDay))

/* Standardised timestamp string: "01 Jan 1970 (THU) 02:00:00(+02:00)\0" */
typedef char acTimestampTimeStr_t[sizeof("01 Jan 1970 (THU) 02:00:00(+02:00)\0")];

/* Init / tick */
void     TIME_vInit(void);
void     TIME_vTick(void);
bool     TIME_bGetClearDtzChangeFlag(void);
uint16_t TIME_u16GetUpTimeDays(void);

/* RTC validity */
bool TIME_bIsRtcValid(void);
bool TIME_bIsTimestampValid(uint32_t u32Timestamp);

/* Time-span conversions */
int8_t   TIME_i8TimeSpanGetHour(int32_t i32TimeSpan);
int8_t   TIME_i8TimeSpanGetMin(int32_t i32TimeSpan);
uint32_t TIME_u32ConvertHourMinToTimeSpan(uint16_t u16Hour, uint8_t u8Minute);
uint32_t TIME_u32GetTimespanToNow(uint32_t u32Timestamp);

/* Timestamp → string */
char *       TIME_pacConvertTimestampToStr(acTimestampTimeStr_t acTimestampTimeStr,
                                           uint32_t u32Timestamp, bool bTimeAndDate);
const char * TIME_pacGetDayOfWeek(time_day_of_week_t tDayOfWeek);
const char * TIME_pacGetMonthSeasonStart(time_season_t tSeason);

/* Current local time — struct */
datetime_t *  TIME_pGetTimeDateLocalNow(void);
time_of_day_t TIME_tGetDayTimeZoneNow(void);
time_season_t TIME_tGetSeasonNow(void);

/* Current local time — numeric */
uint8_t TIME_u8GetHourLocal(void);
uint8_t TIME_u8GetMinuteLocal(void);
uint8_t TIME_u8GetSecondLocal(void);
uint8_t TIME_u8GetDayOfWeek(void);
uint8_t TIME_u8GetDayOfMonthLocal(void);

/* Schedule helpers */
uint32_t TIME_u32GetCurrentSchedule(void);
bool     TIME_bIsScheduleActive(DayTimeZoneSchedule *schedule);

/* Current local time — strings */
const char * TIME_pacGetMonthLocal(void);
const char * TIME_pacGetSeasonNow(void);
const char * TIME_pacGetDayTimeZoneNow(void);

#endif /* TIME_DRIVER_H_ */
