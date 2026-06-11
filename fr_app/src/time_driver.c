/*
 * time_driver.c
 *
 * Time, season and day-time-zone (DTZ) management.
 * Called every second from the platform heartbeat dispatcher (TIME_vTick).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time_driver.h>

#include "platform.h"
#include "dbg_log.h"

/* Day-of-week strings */
static const char acTimeDayOfWeekSun[] = "Sun";
static const char acTimeDayOfWeekMon[] = "Mon";
static const char acTimeDayOfWeekTue[] = "Tue";
static const char acTimeDayOfWeekWed[] = "Wed";
static const char acTimeDayOfWeekThu[] = "Thu";
static const char acTimeDayOfWeekFri[] = "Fri";
static const char acTimeDayOfWeekSat[] = "Sat";
static const char * const pacTimeDayOfWeek[] = {
    acTimeDayOfWeekSun, acTimeDayOfWeekMon, acTimeDayOfWeekTue,
    acTimeDayOfWeekWed, acTimeDayOfWeekThu, acTimeDayOfWeekFri,
    acTimeDayOfWeekSat
};

/* Month strings */
static const char acTimeMonthJan[] = "Jan";
static const char acTimeMonthFeb[] = "Feb";
static const char acTimeMonthMar[] = "Mar";
static const char acTimeMonthApr[] = "Apr";
static const char acTimeMonthMay[] = "May";
static const char acTimeMonthJun[] = "Jun";
static const char acTimeMonthJul[] = "Jul";
static const char acTimeMonthAug[] = "Aug";
static const char acTimeMonthSep[] = "Sep";
static const char acTimeMonthOct[] = "Oct";
static const char acTimeMonthNov[] = "Nov";
static const char acTimeMonthDec[] = "Dec";
static const char *pacTimeMonth[] = {
    acTimeMonthJan, acTimeMonthFeb, acTimeMonthMar, acTimeMonthApr,
    acTimeMonthMay, acTimeMonthJun, acTimeMonthJul, acTimeMonthAug,
    acTimeMonthSep, acTimeMonthOct, acTimeMonthNov, acTimeMonthDec
};

/* Season strings */
static const char acTimeSeasonSummer[] = "SUMMER";
static const char acTimeSeasonWinter[] = "WINTER";
static const char * const pacTimeSeason[] = { acTimeSeasonSummer, acTimeSeasonWinter };

/* Time-of-day strings */
static const char acTimeOfDayMorning[]   = "MORNING";
static const char acTimeOfDayMidday[]    = "MIDDAY";
static const char acTimeOfDayAfternoon[] = "AFTERNOON";
static const char acTimeOfDayNight[]     = "NIGHT";
static const char * const pacTimeOfDay[] = {
    acTimeOfDayMorning, acTimeOfDayMidday, acTimeOfDayAfternoon, acTimeOfDayNight
};

static bool           bTimeDtzChange;
static uint32_t       u32TimeTemp;
static acTimestampTimeStr_t acTimeTemp;
static datetime_t     TimeCalendarDateTemp;
static datetime_t     TimeCalendarDateLocalNow;
static time_season_t  tTimeSeasonNow, tTimeSeasonPrev;
static time_of_day_t  tTimeDayTimeZoneNow, tTimeDayTimeZonePrev;

volatile uint32_t u32TimeRtc;

/* --------------------------------------------------------------------------
 * TIME_vInit
 * -------------------------------------------------------------------------- */
void TIME_vInit(void)
{
    tTimeSeasonPrev      = TIME_SEASON_SUMMER;
    tTimeDayTimeZonePrev = TIME_OF_DAY_NIGHT;

    uint32_t tz = SETTINGS_u32GetWord(time.timezoneAdjust);
    DATETIME_vTimestampToDateTZ((uint32_t)RTC_u64GetUTC(),
                                TIME_i8TimeSpanGetHour(tz),
                                TIME_i8TimeSpanGetMin(tz),
                                &TimeCalendarDateLocalNow);
}

/* --------------------------------------------------------------------------
 * TIME_vTick — called every second from the heartbeat dispatcher
 * -------------------------------------------------------------------------- */
void TIME_vTick(void)
{
    uint32_t u32New;
    uint32_t u32Morning, u32Midday, u32Afternoon, u32Night;

    uint32_t tz = SETTINGS_u32GetWord(time.timezoneAdjust);
    DATETIME_vTimestampToDateTZ((uint32_t)RTC_u64GetUTC(),
                                TIME_i8TimeSpanGetHour(tz),
                                TIME_i8TimeSpanGetMin(tz),
                                &TimeCalendarDateLocalNow);

    /* Update season */
    if ((TimeCalendarDateLocalNow.month >= SETTINGS_u32GetWord(time.winterStartMonth)) &&
        (TimeCalendarDateLocalNow.month <  SETTINGS_u32GetWord(time.summerStartMonth)))
        tTimeSeasonNow = TIME_SEASON_WINTER;
    else
        tTimeSeasonNow = TIME_SEASON_SUMMER;

    /* Update DTZ */
    u32New = TIME_u32ConvertHourMinToTimeSpan(TimeCalendarDateLocalNow.hour,
                                              TimeCalendarDateLocalNow.minute);
    if (tTimeSeasonNow == TIME_SEASON_WINTER)
    {
        u32Morning   = SETTINGS_u32GetWord(dayTimeZone.winterMorningStartTime);
        u32Midday    = SETTINGS_u32GetWord(dayTimeZone.winterMiddayStartTime);
        u32Afternoon = SETTINGS_u32GetWord(dayTimeZone.winterAfternoonStartTime);
        u32Night     = SETTINGS_u32GetWord(dayTimeZone.winterNightStartTime);
    }
    else
    {
        u32Morning   = SETTINGS_u32GetWord(dayTimeZone.summerMorningStartTime);
        u32Midday    = SETTINGS_u32GetWord(dayTimeZone.summerMiddayStartTime);
        u32Afternoon = SETTINGS_u32GetWord(dayTimeZone.summerAfternoonStartTime);
        u32Night     = SETTINGS_u32GetWord(dayTimeZone.summerNightStartTime);
    }

    if      (u32New < u32Morning)   tTimeDayTimeZoneNow = TIME_OF_DAY_NIGHT;
    else if (u32New < u32Midday)    tTimeDayTimeZoneNow = TIME_OF_DAY_MORNING;
    else if (u32New < u32Afternoon) tTimeDayTimeZoneNow = TIME_OF_DAY_MIDDAY;
    else if (u32New < u32Night)     tTimeDayTimeZoneNow = TIME_OF_DAY_AFTERNOON;
    else                            tTimeDayTimeZoneNow = TIME_OF_DAY_NIGHT;

    /* Detect DTZ change */
    if (tTimeDayTimeZonePrev != tTimeDayTimeZoneNow)
    {
        DBG_LOG("DTZ change: %s\r\n", TIME_pacGetDayTimeZoneNow());

        bTimeDtzChange = true;
    }

    if (tTimeSeasonPrev != tTimeSeasonNow)
        DBG_LOG("Season changed; now=%s\r\n", TIME_pacGetSeasonNow());

    tTimeSeasonPrev      = tTimeSeasonNow;
    tTimeDayTimeZonePrev = tTimeDayTimeZoneNow;
}

bool TIME_bGetClearDtzChangeFlag(void)
{
    bool b = bTimeDtzChange;
    bTimeDtzChange = false;
    return b;
}

uint16_t TIME_u16GetUpTimeDays(void)
{
    return (uint16_t)(RTC_u64GetTicks() / 60 / 60 / 24);
}

bool TIME_bIsRtcValid(void)
{
    return TIME_bIsTimestampValid((uint32_t)RTC_u64GetUTC());
}

bool TIME_bIsTimestampValid(uint32_t u32Timestamp)
{
    return (u32Timestamp < TIME_UTC_VALID_MAX) && (u32Timestamp > TIME_UTC_VALID_MIN);
}

int8_t TIME_i8TimeSpanGetHour(int32_t i32TimeSpan)
{
    return (int8_t)(i32TimeSpan / (60 * 60));
}

int8_t TIME_i8TimeSpanGetMin(int32_t i32TimeSpan)
{
    return (int8_t)((i32TimeSpan / 60) % 60);
}

uint32_t TIME_u32ConvertHourMinToTimeSpan(uint16_t u16Hour, uint8_t u8Minute)
{
    return ((uint32_t)u16Hour * 60 * 60) + ((uint32_t)u8Minute * 60);
}

uint32_t TIME_u32GetTimespanToNow(uint32_t u32Timestamp)
{
    return (uint32_t)RTC_u64GetUTC() - u32Timestamp;
}

char *TIME_pacConvertTimestampToStr(acTimestampTimeStr_t acTimestampTimeStr,
                                    uint32_t u32Timestamp, bool bTimeAndDate)
{
    uint32_t u32Tz = SETTINGS_u32GetWord(time.timezoneAdjust);
    DATETIME_vTimestampToDateTZ(u32Timestamp, TIME_i8TimeSpanGetHour(u32Tz),
                                TIME_i8TimeSpanGetMin(u32Tz), &TimeCalendarDateTemp);

    uint8_t i = (uint8_t)snprintf(acTimeTemp, sizeof(acTimeTemp),
        "%02u:%02u:%02u(%c%02u:%02u)",
        TimeCalendarDateTemp.hour,
        TimeCalendarDateTemp.minute,
        TimeCalendarDateTemp.second,
        (TIME_i8TimeSpanGetHour(u32Tz) >= 0) ? '+' : '-',
        (uint8_t)abs(TIME_i8TimeSpanGetHour(u32Tz)),
        (uint8_t)TIME_i8TimeSpanGetMin(u32Tz));

    if (bTimeAndDate)
        snprintf(&acTimeTemp[i], sizeof(acTimeTemp) - i, " %s %02u-%s-%04u",
            pacTimeDayOfWeek[TimeCalendarDateTemp.dayofweek],
            TimeCalendarDateTemp.date + 1,
            pacTimeMonth[TimeCalendarDateTemp.month],
            TimeCalendarDateTemp.year);

    if (acTimestampTimeStr)
        strcpy(acTimestampTimeStr, acTimeTemp);

    return acTimeTemp;
}

const char *TIME_pacGetDayOfWeek(time_day_of_week_t tDayOfWeek)
{
    return pacTimeDayOfWeek[tDayOfWeek];
}

const char *TIME_pacGetMonthSeasonStart(time_season_t tSeason)
{
    if (tSeason == TIME_SEASON_SUMMER)
        return pacTimeMonth[SETTINGS_u32GetWord(time.summerStartMonth)];
    else
        return pacTimeMonth[SETTINGS_u32GetWord(time.winterStartMonth)];
}

datetime_t   *TIME_pGetTimeDateLocalNow(void)  { return &TimeCalendarDateLocalNow; }
time_of_day_t TIME_tGetDayTimeZoneNow(void)    { return tTimeDayTimeZoneNow;       }
time_season_t TIME_tGetSeasonNow(void)         { return tTimeSeasonNow;            }
uint8_t       TIME_u8GetHourLocal(void)        { return TimeCalendarDateLocalNow.hour;      }
uint8_t       TIME_u8GetMinuteLocal(void)      { return TimeCalendarDateLocalNow.minute;    }
uint8_t       TIME_u8GetSecondLocal(void)      { return TimeCalendarDateLocalNow.second;    }
uint8_t       TIME_u8GetDayOfWeek(void)        { return TimeCalendarDateLocalNow.dayofweek; }
uint8_t       TIME_u8GetDayOfMonthLocal(void)  { return TimeCalendarDateLocalNow.date + 1;  }

uint32_t TIME_u32GetCurrentSchedule(void)
{
    return ((uint32_t)0x00000001 << (4 * TimeCalendarDateLocalNow.dayofweek + tTimeDayTimeZoneNow));
}

bool TIME_bIsScheduleActive(DayTimeZoneSchedule *schedule)
{
    return (schedule->dtzActiveBitMask &
            ((uint32_t)0x00000001 << (4 * TimeCalendarDateLocalNow.dayofweek + tTimeDayTimeZoneNow)))
           ? true : false;
}

const char *TIME_pacGetMonthLocal(void)    { return pacTimeMonth[TimeCalendarDateLocalNow.month]; }
const char *TIME_pacGetSeasonNow(void)     { return pacTimeSeason[TIME_tGetSeasonNow()];          }
const char *TIME_pacGetDayTimeZoneNow(void){ return pacTimeOfDay[TIME_tGetDayTimeZoneNow()];      }
