/*
 * time_config.h
 *
 * Compile-time defaults for the time/season/DTZ module.
 * All time values are in seconds from midnight (UTC).
 */

#ifndef TIME_CONFIG_H_
#define TIME_CONFIG_H_

/* --------------------------------------------------------------------------
 * Timezone adjustment from UTC (seconds)
 * -------------------------------------------------------------------------- */
#define TIME_TZ_GMT_ADJ_MIN     INT32_C(-43200)  /* UTC-12:00 */
#ifdef POC_TZ
/* Brisbane, Australia — UTC+10 */
#  define TIME_TZ_GMT_ADJ_DEF   INT32_C(36000)
#else
/* South Africa — UTC+02 */
#  define TIME_TZ_GMT_ADJ_DEF   INT32_C(7200)
#endif
#define TIME_TZ_GMT_ADJ_MAX     INT32_C(43200)   /* UTC+12:00 */

/* --------------------------------------------------------------------------
 * Season start months (0 = Jan … 11 = Dec)
 * -------------------------------------------------------------------------- */
#define SUMMER_START_MON_MIN    8    /* Sep */
#define SUMMER_START_MON_DEF    8    /* Sep */
#define SUMMER_START_MON_MAX    11   /* Dec */

#define WINTER_START_MON_MIN    2    /* Mar */
#define WINTER_START_MON_DEF    3    /* Apr */
#define WINTER_START_MON_MAX    5    /* Jun */

/* --------------------------------------------------------------------------
 * Day-time-zone start times
 * -------------------------------------------------------------------------- */
#define NIGHT_START_TIME_MIN            UINT32_C(0  * 60 * 60)
#define NIGHT_START_TIME_DEF_SUMMER     UINT32_C(20 * 60 * 60)
#define NIGHT_START_TIME_DEF_WINTER     UINT32_C(19 * 60 * 60)
#define NIGHT_START_TIME_MAX            UINT32_C(23 * 60 * 60)

#define MORNING_START_TIME_MIN          UINT32_C(0  * 60 * 60)
#define MORNING_START_TIME_DEF_SUMMER   UINT32_C(5  * 60 * 60)
#define MORNING_START_TIME_DEF_WINTER   UINT32_C(6  * 60 * 60)
#define MORNING_START_TIME_MAX          UINT32_C(23 * 60 * 60)

#define MIDDAY_START_TIME_MIN           UINT32_C(0  * 60 * 60)
#define MIDDAY_START_TIME_DEF_SUMMER    UINT32_C(10 * 60 * 60)
#define MIDDAY_START_TIME_DEF_WINTER    UINT32_C(10 * 60 * 60)
#define MIDDAY_START_TIME_MAX           UINT32_C(23 * 60 * 60)

#define AFTERNOON_START_TIME_MIN        UINT32_C(0  * 60 * 60)
#define AFTERNOON_START_TIME_DEF_SUMMER UINT32_C(19 * 60 * 60)
#define AFTERNOON_START_TIME_DEF_WINTER UINT32_C(18 * 60 * 60)
#define AFTERNOON_START_TIME_MAX        UINT32_C(23 * 60 * 60)

/* --------------------------------------------------------------------------
 * UTC validity window
 * -------------------------------------------------------------------------- */
#define TIME_UTC_VALID_MIN  UINT32_C(1514764800)   /* 2018-01-01 00:00:00 UTC */
#define TIME_UTC_VALID_DEF  UINT32_C(0xFFFFFFFF)   /* sentinel — outside range */
#define TIME_UTC_VALID_MAX  UINT32_C(2145916799)   /* 2037-12-31 23:59:59 UTC */

#endif /* TIME_CONFIG_H_ */
