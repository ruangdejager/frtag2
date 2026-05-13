/*
 * datetime.c
 *
 * UNIX timestamp ↔ calendar date/time conversion.
 */

#include "datetime.h"

#define EPOCH_YEAR      1970
#define SECS_PER_DAY    86400UL
#define SECS_PER_HOUR   3600UL
#define SECS_PER_MINUTE 60UL

/* Days per month: index [0] = normal year, [1] = leap year */
static const uint8_t month[2][12] = {
    { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
    { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

static bool     DATETIME_bIsCalendarLeapYear(uint16_t year);
static uint16_t DATETIME_u16CalendarYearSize(uint16_t year);

bool DATETIME_bIsDateValid(datetime_t *date)
{
    if ((date->second >= 60) || (date->minute >= 60) || (date->hour >= 24))
        return false;
    if ((date->month >= 12) || (date->date >= 31))
        return false;
    if (date->date >= month[DATETIME_bIsCalendarLeapYear(date->year)][date->month])
        return false;
    if ((date->year < EPOCH_YEAR) || (date->year >= 2106))
        return false;
    return true;
}

void DATETIME_vTimestampToDate(uint32_t timestamp, datetime_t *date_out)
{
    uint32_t day_number;
    uint32_t day_clock;

    date_out->year  = EPOCH_YEAR;
    date_out->month = 0;

    day_clock  = timestamp % SECS_PER_DAY;
    day_number = timestamp / SECS_PER_DAY;

    date_out->second    = day_clock % SECS_PER_MINUTE;
    date_out->minute    = (day_clock % SECS_PER_HOUR) / SECS_PER_MINUTE;
    date_out->hour      = day_clock / SECS_PER_HOUR;
    date_out->dayofweek = (day_number + 4) % 7;

    while (day_number >= DATETIME_u16CalendarYearSize(date_out->year))
    {
        day_number -= DATETIME_u16CalendarYearSize(date_out->year);
        date_out->year++;
    }
    while (day_number >= month[DATETIME_bIsCalendarLeapYear(date_out->year)][date_out->month])
    {
        day_number -= month[DATETIME_bIsCalendarLeapYear(date_out->year)][date_out->month];
        date_out->month++;
    }
    date_out->date = day_number;
}

void DATETIME_vTimestampToDateTZ(uint32_t timestamp, int8_t hour, uint8_t min, datetime_t *date_out)
{
    if (hour >= 0)
        DATETIME_vTimestampToDate(timestamp + (SECS_PER_HOUR * (uint32_t)hour) + (SECS_PER_MINUTE * min), date_out);
    else
        DATETIME_vTimestampToDate(timestamp + (SECS_PER_HOUR * (int32_t)hour) - (SECS_PER_MINUTE * min), date_out);
}

uint32_t DATETIME_u32DateTimetoTimestamp(datetime_t *date)
{
    if (!DATETIME_bIsDateValid(date))
        return 0;

    uint32_t timestamp = 0;
    uint8_t  date_month = date->month;
    uint16_t date_year  = date->year;

    timestamp += (date->date   * SECS_PER_DAY)
               + (date->hour   * SECS_PER_HOUR)
               + (date->minute * SECS_PER_MINUTE)
               +  date->second;

    while (date_month != 0)
    {
        date_month--;
        timestamp += month[DATETIME_bIsCalendarLeapYear(date_year)][date_month] * SECS_PER_DAY;
    }
    while (date_year > EPOCH_YEAR)
    {
        date_year--;
        timestamp += DATETIME_u16CalendarYearSize(date_year) * SECS_PER_DAY;
    }
    return timestamp;
}

uint32_t DATETIME_u32DateTimetoTimestampTZ(datetime_t *date, int8_t hour, uint8_t min)
{
    uint32_t timestamp = DATETIME_u32DateTimetoTimestamp(date);
    if (timestamp == 0)
        return 0;
    if (hour >= 0)
        return timestamp - (SECS_PER_HOUR * (uint32_t)hour + SECS_PER_MINUTE * min);
    else
        return timestamp - (SECS_PER_HOUR * (int32_t)hour  - SECS_PER_MINUTE * min);
}

static bool DATETIME_bIsCalendarLeapYear(uint16_t year)
{
    return (!((year) % 4) && (((year) % 100) || !((year) % 400)));
}

static uint16_t DATETIME_u16CalendarYearSize(uint16_t year)
{
    return DATETIME_bIsCalendarLeapYear(year) ? 366 : 365;
}
