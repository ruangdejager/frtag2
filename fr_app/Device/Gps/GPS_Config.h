/*
 * GPS_Config.h
 *
 * GNSS session parameters and timing constants.
 */

#ifndef DEVICE_GPS_GPS_CONFIG_H_
#define DEVICE_GPS_GPS_CONFIG_H_

/*
 * Per-sample GNSS telemetry CSV (see GPS.c). One flash-log line per second of
 * every fix session — useful for bench TTFF/convergence tuning, but pure noise
 * in production (the per-session FIX OK / FIX TIMEOUT lines already capture the
 * outcome). Off by default; define to restore it.
 */
// #define LOG_GPS_PERIODIC

/* Default TTFF timeout [s] — aided start */
#define GNSS_TTFF_TIMEOUT_1_AID_ASSIST      UINT16_C(60)

/* Extended TTFF timeout [s] — cold start */
#define GNSS_TTFF_TIMEOUT_2_COLD_START      UINT16_C(180)

/*
 * Sentinel TTFF timeout meaning "no timeout" — used when GPS_vRequestFix()
 * is called with bAutoShutdown == false, so the session is allowed to run
 * indefinitely until a stable fix is declared or GPS_vShutdown() is called.
 */
#define GNSS_TTFF_NO_TIMEOUT                UINT16_C(0xFFFF)

/*
 * Minimum sequential valid fixes before declaring a stable solution.
 * Testing shows solution deltas can take up to 7-8 fixes to converge;
 * choose a margin above that.
 */
#define GNSS_SESSION_VALID_FIX_CNT_MIN      5

/*
 * Nominal sequential fix count: extra time beyond the minimum for the
 * position error to settle before applying GNSS_SESSION_VALID_EST_POS_ERROR_MAX.
 */
#define GNSS_SESSION_VALID_FIX_CNT_NOM      30

/* Maximum lat/long variation between consecutive fixes for a stable session */
#define GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX   3

/* Primary position error threshold [m] (EPE_2D, tight) */
#define GNSS_SESSION_VALID_EST_POS_ERROR_NOM    5

/* Secondary position error threshold [m] (EPE_2D, relaxed) */
#define GNSS_SESSION_VALID_EST_POS_ERROR_MAX    10

/* GNSS assist data filenames / URLs */
#define GNSS_ASSIST_LOCAL_FILENAME          "UFS:xtra2.bin"
#define GNSS_ASSIST_SERVER_RESOURCE_1       "http://xtrapath1.izatcloud.net/xtra2.bin"
#define GNSS_ASSIST_SERVER_RESOURCE_2       "http://xtrapath2.izatcloud.net/xtra2.bin"

/*
 * Compile-time option for NMEA GGA (altitude) — currently only used in
 * experimental logging applications.
 */
#if defined SWITCH_GNSS_LOGGER
#define GNSS_NMEA_GGA   true
#endif

/*
 * Timeout for external GNSS module detection [ms].
 * Chosen for >50% safety margin over worst-case production startup time.
 */
#define GNSS_EXT_MODULE_DETECT_TIMEOUT      2000

/*
 * Minimum RTC error [s] required before a GPS-derived time is applied.
 * Differences smaller than this threshold are treated as within acceptable
 * oscillator drift and the RTC is left unchanged.
 */
#define GPS_RTC_SYNC_MIN_ERROR_S            5U

#endif /* DEVICE_GPS_GPS_CONFIG_H_ */
