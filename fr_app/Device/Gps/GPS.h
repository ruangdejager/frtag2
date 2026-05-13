/*
 * GPS.h
 *
 * GNSS module public interface — types, session control and data access.
 *
 * The GNSS subsystem runs a dedicated CMSIS-RTOS v2 RX task that is
 * woken by the UART ISR via thread flags.  Callers start a session with
 * GPS_vStart(), passing their own thread ID so the GPS layer can notify
 * them via osThreadFlagsSet() when a stable fix is acquired or the TTFF
 * timeout expires.
 */

#ifndef DEVICE_GPS_GPS_H_
#define DEVICE_GPS_GPS_H_

#include "GPS_Config.h"
#include "GPS_driver.h"
#include "platform.h"
#include "cmsis_os2.h"
#include "dbg_log.h"

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

/* Notification flags sent to the caller via osThreadFlagsSet() */
#define GPS_NOTIFY_FIX_OK       (1UL << 0)   /* Stable fix acquired         */
#define GPS_NOTIFY_FIX_TIMEOUT  (1UL << 1)   /* TTFF timeout — best fix used */

/* ---- Public API ---- */
void GPS_vInit(void);
void GPS_vRxTask(void *parameters);
void GPS_vStart(osThreadId_t callerTask);   /* Power on; notifies caller on fix/timeout */
void GPS_vStop(void);
bool GPS_bGetCoordinates(gnss_coord_deg_t *pLat, gnss_coord_deg_t *pLong);
bool GPS_bHasStableFix(void);
void GPS_vNotifyOnRX(void);                 /* Called from UART ISR — do not call directly */

#endif /* DEVICE_GPS_GPS_H_ */
