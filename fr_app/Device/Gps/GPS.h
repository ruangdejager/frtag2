/*
 * GPS.h
 *
 * GNSS module public interface — types, session control and data access.
 *
 * The GNSS subsystem is a self-contained worker. A dispatcher task owns
 * power and session lifecycle; an RX task parses NMEA from the UART ISR.
 *
 * Any module can call GPS_vRequestFix() to start (or attach to) a fix
 * acquisition. Multiple callers may concurrently call GPS_eWaitForFix()
 * to block until the session completes. Sessions can be auto-shutdown
 * (GPS powers itself off after fix or TTFF timeout) or held open until
 * GPS_vShutdown() is called.
 *
 * The most recent successful fix is retained across sessions and can be
 * retrieved together with its age via GPS_bGetLastKnownFix().
 *
 * If the board is not in POWER_CLASS_NORMAL at request time, the GPS
 * refuses to power up and any waiter is released with GPS_RESULT_NO_POWER.
 */

#ifndef DEVICE_GPS_GPS_H_
#define DEVICE_GPS_GPS_H_

#include "GPS_Config.h"
#include "GPS_driver.h"
#include "platform.h"
#include "cmsis_os2.h"
#include "dbg_log.h"

#include <stdbool.h>
#include <string.h>
#include <ctype.h>

/* ---- Satellite info ---- */
typedef struct {
    uint8_t u8SvInView;
    uint8_t u8SvTracking;
    uint8_t u8SnrAvgDbHz;
    uint8_t u8SnrAvgPercent;
} gnss_sol_sv_t;

/* ---- Coordinate types ---- */
typedef struct {
    int16_t  i16Deg;
    uint32_t u32DeciMicroDeg;
    float    fDegrees;
    int32_t  i32MicroDeg;
} gnss_coord_deg_t;

typedef struct {
    int16_t  i16Degrees;
    uint8_t  u8Minutes;
    uint8_t  u8Seconds;
    uint16_t u16MilliSeconds;
} gnss_coord_dms_t;

typedef struct {
    int16_t  i16Degrees;
    uint8_t  u8Minutes;
    uint32_t u32MicroMinutes;
} gnss_coord_ddm_t;

typedef struct {
    gnss_coord_ddm_t Ddm;
    gnss_coord_deg_t Deg;
    gnss_coord_dms_t Dms;
} gnss_coordinate_t;

/* ---- Position error ---- */
typedef struct {
    bool     bHasValue;
    uint16_t u16ValueInt;
    uint8_t  u8ValueDecimal;
} gnss_position_error_t;

/* ---- Altitude ---- */
typedef struct {
    int16_t i16Value;
    uint8_t u8DecimalValue;
} gnss_altitude_t;

/* ---- NMEA message flags ---- */
typedef enum {
    GNSS_NMEA_MSG_RMC_bm     = (1 << 0),
    GNSS_NMEA_MSG_GSV_bm     = (1 << 1),
    GNSS_NMEA_MSG_GGA_bm     = (1 << 2),
    GNSS_NMEA_MSG_PQTMEPE_bm = (1 << 3),
    GNSS_NMEA_MSG_PUBX00_bm  = (1 << 4),
} gnss_nmea_msg_bm_t;

/* ---- Session fix state ---- */
typedef enum {
    GNSS_SESSION_FIX_NONE  = 0,
    GNSS_SESSION_FIX_BUSY,
    GNSS_SESSION_FIX_OK,
    GNSS_SESSION_FIX_ERROR,
} gnss_session_fix_t;

/* ---- Complete GNSS solution ---- */
typedef struct {
    gnss_nmea_msg_bm_t  tNmeaMsgFlags;
    bool                bStatusValid;
    gnss_sol_sv_t       Sv;
    gnss_coordinate_t   Lat;
    gnss_coordinate_t   Long;
    gnss_altitude_t     Altitude;
    datetime_t          TimeDateUtc;
    uint32_t            u32TimeUnix;
    gnss_position_error_t PositionError;
} gnss_sol_t;

/* ---- Session state ---- */
typedef struct {
    bool              bDbgStartFlag;
    bool              bDbgStopFlag;
    uint16_t          u16TimeToFirstFix;
    uint32_t          u32StartTime;
    gnss_coordinate_t LastValidLat;
    gnss_coordinate_t LastValidLong;
    uint8_t           u8ValidFixCount;
    uint16_t          u16TtffTimeout;
    uint16_t          u16TtffTmr;
    bool              bHasFirstStableFix;
    bool              bHasStableFixNow;
    gnss_nmea_msg_bm_t tNmeaMsgFlagsBm;
    gnss_session_fix_t Fix;
} gnss_session_t;

/* ---- Result of a fix session (returned by GPS_eWaitForFix) ---- */
typedef enum {
    GPS_RESULT_FIX_OK      = 0,   /* Stable fix acquired                       */
    GPS_RESULT_FIX_TIMEOUT = 1,   /* TTFF timeout — best-effort fix or none    */
    GPS_RESULT_NO_POWER    = 2,   /* Refused — board not in POWER_CLASS_NORMAL */
} gps_result_e;

/* ---- Public API ---- */

/* One-time initialization — call from main init path. Creates the dispatcher
 * and RX tasks, mutex, and result-event-flags group. */
void GPS_vInit(void);

/* Kick off (or attach to) a fix acquisition.
 *   bAutoShutdown == true  → GPS powers off after a stable fix or after
 *                            u32TtffTimeoutS seconds, whichever comes first.
 *                            u32TtffTimeoutS is applied to GnssSession.u16TtffTimeout
 *                            (clamped to uint16_t; 0 falls back to
 *                            GNSS_TTFF_TIMEOUT_1_AID_ASSIST).
 *   bAutoShutdown == false → GPS stays powered until GPS_vShutdown(); no TTFF
 *                            timeout is enforced (GnssSession.u16TtffTimeout is
 *                            set to GNSS_TTFF_NO_TIMEOUT) and u32TtffTimeoutS
 *                            is ignored.
 *
 * Idempotent: if a session is already in progress, returns immediately
 * and updates bAutoShutdown (and the TTFF timeout, if applicable) to the
 * last-caller values. If the board is not in POWER_CLASS_NORMAL the call
 * refuses to power the GPS and sets the NO_POWER result so any concurrent
 * waiter is released.
 *
 * Deep-sleep lock: this function owns it. A single SYSTEM_vSleepLock is
 * taken on the cold-start transition (when a new powered session actually
 * begins) and released in GPS_vPowerOff() when the receiver powers down.
 * Callers must NOT bracket the call with their own acquire/release —
 * doing so would double-count on overlapping requests (each request
 * acquires, but only one power-down releases) and wedge the device out of
 * STOP2. Just call it; the lock is handled internally.
 *
 * bForce: when true, bypass the system-wide GPS-enable gate (fr9's
 * movementAlarm.nightZoneLevels.holdFirst flag distributed via TimeSync).
 * Reserved for scenarios that MUST have a fix regardless of user
 * preference — currently only ProductionSleep-exit paths, which need
 * RTC correction after an unbounded stretch of sleep. Routine callers
 * (the pre-trigger before every scheduled wake) should pass false so
 * the user's disable is respected. */
void GPS_vRequestFix(bool bAutoShutdown, uint32_t u32TtffTimeoutS, bool bForce);

/* Block until the current (or next) session signals a terminal result.
 * Returns the result enum, or GPS_RESULT_FIX_TIMEOUT if u32TimeoutMs
 * expires before any GPS event. Multiple callers may wait concurrently. */
gps_result_e GPS_eWaitForFix(uint32_t u32TimeoutMs);

/* Manually power down the GPS (no-op if already off). Use only when the
 * matching GPS_vRequestFix() was called with bAutoShutdown == false. */
void GPS_vShutdown(void);

/* Live coordinates from the current/last session — returns false if no
 * stable fix has been declared in the current session. */
bool GPS_bGetCoordinates(gnss_coord_deg_t *pLat, gnss_coord_deg_t *pLong);
bool GPS_bHasStableFix(void);

/* Last-known fix retention. Retained across sessions and power cycles;
 * cleared only on cold boot. Returns true and copies lat/lon if any fix
 * has ever been acquired since boot. *pu32AgeSeconds is set to the wall-
 * clock age of the stored fix (RTC-based). */
bool GPS_bGetLastKnownFix(gnss_coord_deg_t *pLat,
                          gnss_coord_deg_t *pLon,
                          uint32_t         *pu32AgeSeconds);

/* Age in seconds of the most recent fix; UINT32_MAX if none yet. */
uint32_t GPS_u32GetLastFixAgeSeconds(void);

/* Called from USART1 ISR via uart_callbacks.c — do not call directly. */
void GPS_vNotifyOnRX(void);

#endif /* DEVICE_GPS_GPS_H_ */
