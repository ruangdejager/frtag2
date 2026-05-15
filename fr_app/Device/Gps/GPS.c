/*
 * GPS.c
 *
 * GNSS module driver — NMEA stream parsing and session management.
 *
 * The RX task blocks on thread flags set by the UART ISR.  When a
 * complete, valid NMEA sentence is received GNSS_vOnSolution() is
 * called to evaluate the fix quality and notify the registered caller
 * task when a stable fix is declared or the TTFF timeout expires.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "GPS.h"
#include "dbg_log.h"
#include "Debug.h"

/* Task parameters */
#define GPS_RX_TASK_PRIORITY    osPriorityRealtime7
#define GPS_RX_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE)

/* Thread flag used by ISR → RX task notification */
#define GPS_THREAD_FLAG_RX      0x0001U

/* CMSIS-RTOS v2 task handle */
static osThreadId_t GPS_vRxTask_handle;

gnss_session_t GnssSession;

gnss_sol_t GnssSolDraft;
gnss_sol_t GnssSol;
gnss_sol_t GnssSolTs;

uint32_t u32GnssAssistDataExpiryTs;

char acGnssStrBuf[16];

#define GPS_RX_BUF_LEN  128
char    acGpsRxBuf[GPS_RX_BUF_LEN];
uint8_t u8GpsRxBufIdx;
uint32_t u32GnssRxLastTs;

uint16_t u16GpsSvSnrAvg;
uint8_t  u8GpsSvTrackingCnt;

bool GnssSyslogFlag = true;

static osThreadId_t  GnssCallerTask  = NULL;
static bool          bSessionActive  = false;
static bool          bCallerNotified = false;
static gnss_coord_deg_t GnssStableLat;
static gnss_coord_deg_t GnssStableLong;

struct _gps_s {
    hal_uart_t UartHandle;
    uint8_t    byte;
    uint32_t   upTimeCounter;
} gps;

/* Private prototypes */
uint8_t u8HexCharToInt(char cHexChar);
bool    __bFindNext_P(char **p1, const char *p2, bool bAdvance);
uint8_t u8CalcNmeaChecksum(char *pacStart, char *pacStop);
bool    GPS_bOnRxByte(char pcRxByte);
bool    GPS_bCheckNmeaMsg(char *pacNmeaMsg);
bool    GPS_bParseNmeaMsgGsv(char *pNmeaMsg, gnss_sol_t *pSol);
bool    GPS_bParseNmeaMsgRmc(char *pNmeaMsg, gnss_sol_t *pSol);
bool    GPS_bParseNmeaMsgGga(char *pGgaMsg, gnss_sol_t *pSol);
bool    GPS_bParseNmeaMsgPQTMEPE(char *pNmeaMsg, gnss_sol_t *pSol);
bool    GPS_bParseNmeaMsgPubx00(char *pNmeaMsg, gnss_sol_t *pSol);
void    GPS_vCoordConvertDdmToDeg(gnss_coord_deg_t *pDeg, const gnss_coord_ddm_t *pDdm);
void    GPS_vCoordConvertDdmToDms(gnss_coord_dms_t *pDms, const gnss_coord_ddm_t *pDdm);
uint8_t u8SnrDbHzToPercent(uint8_t u8DbHz);
void    GNSS_vOnSolution(void);
bool    bAddUbxChecksum(uint8_t *pUbxFrame, uint16_t u16Len);

/* -------------------------------------------------------------------------- */

uint8_t u8SnrDbHzToPercent(uint8_t u8DbHz)
{
    uint16_t u16Temp = ((u8DbHz > 44) ? 44 : u8DbHz);
    return (uint8_t)((100 * u16Temp) / 44);
}

bool __bFindNext_P(char **p1, const char *p2, bool bAdvance)
{
    char *s = strstr(*p1, p2);
    if (s == NULL) return false;
    if (bAdvance) s += strlen(p2);
    *p1 = s;
    return true;
}

/* --------------------------------------------------------------------------
 * GPS_vInit — set up UART handle and create the RX task
 * -------------------------------------------------------------------------- */
void GPS_vInit(void)
{
    GPS_DRIVER_vInitGPS(&gps.UartHandle);

    static const osThreadAttr_t rx_attr = {
        .name       = "GPSRxTask",
        .stack_size = GPS_RX_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = GPS_RX_TASK_PRIORITY,
    };
    GPS_vRxTask_handle = osThreadNew(GPS_vRxTask, NULL, &rx_attr);
    configASSERT(GPS_vRxTask_handle != NULL);

    DBG("gps: init\r\n");
}

/* --------------------------------------------------------------------------
 * GPS_vStart — power on the GNSS module and begin a new session
 * -------------------------------------------------------------------------- */
void GPS_vStart(osThreadId_t callerTask)
{
    GnssCallerTask   = callerTask;
    bCallerNotified  = false;
    bSessionActive   = true;

    memset(&GnssSession, 0, sizeof(gnss_session_t));
    GnssSession.u32StartTime     = (uint32_t)RTC_u64GetTicks();
    GnssSession.u16TtffTimeout   = GNSS_TTFF_TIMEOUT_1_AID_ASSIST;
    GnssSession.tNmeaMsgFlagsBm  = GNSS_NMEA_MSG_RMC_bm | GNSS_NMEA_MSG_PQTMEPE_bm;
    GnssSession.Fix              = GNSS_SESSION_FIX_BUSY;
    memset(&GnssSolDraft, 0, sizeof(gnss_sol_t));

    GPS_DRIVER_vEnableUart(&gps.UartHandle);
    HAL_UART_vClearBuffer(&gps.UartHandle);
    GPS_DRIVER_vPowerEnHigh();

    DBG("gps: start, ttff timeout=%us\r\n", GnssSession.u16TtffTimeout);
}

/* --------------------------------------------------------------------------
 * GPS_vStop — power down and disable UART
 * -------------------------------------------------------------------------- */
void GPS_vStop(void)
{
    bSessionActive = false;
    GnssCallerTask = NULL;
    GPS_DRIVER_vPowerEnLow();
    GPS_DRIVER_vDisableUart(&gps.UartHandle);
    DBG("gps: stop\r\n");
}

/* --------------------------------------------------------------------------
 * GPS_vRxTask — CMSIS-RTOS v2 task; blocks on thread flag from ISR
 * -------------------------------------------------------------------------- */
void GPS_vRxTask(void *parameters)
{
    (void)parameters;

    for (;;)
    {
        osThreadFlagsWait(GPS_THREAD_FLAG_RX, osFlagsWaitAny, osWaitForever);

        if (!bSessionActive) continue;

        while (UART_bReadByte(&gps.UartHandle, &gps.byte))
        {
            u32GnssRxLastTs = RTC_u64GetTicks();
            GPS_bOnRxByte(gps.byte);
        }
    }
}

/* --------------------------------------------------------------------------
 * GPS_vNotifyOnRX — called from USART1 ISR via uart_callbacks.c; ISR-safe
 * -------------------------------------------------------------------------- */
void GPS_vNotifyOnRX(void)
{
    if (GPS_vRxTask_handle != NULL)
        osThreadFlagsSet(GPS_vRxTask_handle, GPS_THREAD_FLAG_RX);
}

/* --------------------------------------------------------------------------
 * GPS_bOnRxByte — accumulate NMEA bytes and trigger parsing on LF
 * -------------------------------------------------------------------------- */
bool GPS_bOnRxByte(char pcRxByte)
{
    if (u8GpsRxBufIdx == 0 && pcRxByte != '$') return true;

    uint8_t u8NextIdx = u8GpsRxBufIdx + 1;
    if (u8NextIdx < GPS_RX_BUF_LEN)
    {
        acGpsRxBuf[u8GpsRxBufIdx] = pcRxByte;
        acGpsRxBuf[u8NextIdx]     = 0;
        DEBUG_vPutByte((uint8_t)pcRxByte);
    }
    else
    {
        DBG("gps: rx buf overflow\r\n");
        u8NextIdx = 0;
        memset(acGpsRxBuf, 0, sizeof(acGpsRxBuf));
    }
    u8GpsRxBufIdx = u8NextIdx;

    if (strstr(acGpsRxBuf, "\r\n"))
    {
        if (GPS_bCheckNmeaMsg(acGpsRxBuf))
        {
            if      (GPS_bParseNmeaMsgGsv(acGpsRxBuf, &GnssSolDraft))
                GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_GSV_bm;
            else if (GPS_bParseNmeaMsgRmc(acGpsRxBuf, &GnssSolDraft))
                GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_RMC_bm;
#ifdef GNSS_NMEA_GGA
            else if (GPS_bParseNmeaMsgGga(acGpsRxBuf, &GnssSolDraft))
                GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_GGA_bm;
#endif
            else if (GPS_bParseNmeaMsgPQTMEPE(acGpsRxBuf, &GnssSolDraft))
                GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_PQTMEPE_bm;
            else if (GPS_bParseNmeaMsgPubx00(acGpsRxBuf, &GnssSolDraft))
                GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_PUBX00_bm;

            if (GnssSolDraft.tNmeaMsgFlags == GnssSession.tNmeaMsgFlagsBm)
                GNSS_vOnSolution();
        }

        u8GpsRxBufIdx = 0;
        memset(acGpsRxBuf, 0, sizeof(acGpsRxBuf));
    }

    return true;
}

/* --------------------------------------------------------------------------
 * GNSS_vOnSolution — evaluate fix quality; notify caller when appropriate
 * -------------------------------------------------------------------------- */
void GNSS_vOnSolution(void)
{
    uint16_t u16SessionTmr;
    int32_t  i32LatDelta = 0, i32LongDelta = 0;
    bool     bGnssEstPosErrorOk;

    memcpy(&GnssSol, &GnssSolDraft, sizeof(gnss_sol_t));
    memset(&GnssSolDraft, 0, sizeof(gnss_sol_t));

    u16SessionTmr = (uint16_t)((uint32_t)RTC_u64GetTicks() - GnssSession.u32StartTime);

    if (GnssSession.u16TimeToFirstFix == 0)
    {
        GnssSession.u16TtffTmr = u16SessionTmr;
        if (GnssSession.u16TtffTmr > GnssSession.u16TtffTimeout)
            GnssSession.u16TtffTmr = GnssSession.u16TtffTimeout;
    }

    if (GnssSol.bStatusValid)
    {
        if (!GnssSession.bHasStableFixNow)
            GnssSession.u8ValidFixCount++;

        i32LatDelta  = GnssSession.LastValidLat.Deg.i32MicroDeg  - GnssSol.Lat.Deg.i32MicroDeg;
        i32LongDelta = GnssSession.LastValidLong.Deg.i32MicroDeg - GnssSol.Long.Deg.i32MicroDeg;
        i32LatDelta  = abs((int16_t)i32LatDelta);
        i32LongDelta = abs((int16_t)i32LongDelta);

        if (GnssSol.PositionError.bHasValue)
        {
            bGnssEstPosErrorOk = (GnssSession.u8ValidFixCount <= GNSS_SESSION_VALID_FIX_CNT_NOM)
                ? (GnssSol.PositionError.u16ValueInt < GNSS_SESSION_VALID_EST_POS_ERROR_NOM)
                : (GnssSol.PositionError.u16ValueInt < GNSS_SESSION_VALID_EST_POS_ERROR_MAX);
        }
        else
        {
            bGnssEstPosErrorOk = (((int16_t)i32LatDelta  <= GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX)
                               && ((int16_t)i32LongDelta <= GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX));
        }

        memcpy(&GnssSession.LastValidLat,  &GnssSol.Lat,  sizeof(gnss_coordinate_t));
        memcpy(&GnssSession.LastValidLong, &GnssSol.Long, sizeof(gnss_coordinate_t));

        if ((GnssSession.u8ValidFixCount >= GNSS_SESSION_VALID_FIX_CNT_MIN)
            && (bGnssEstPosErrorOk || (GnssSession.u16TtffTmr >= GnssSession.u16TtffTimeout)))
        {
            if (GnssSession.u16TimeToFirstFix == 0)
                GnssSession.u16TimeToFirstFix = u16SessionTmr;
            GnssSession.bHasFirstStableFix = true;
            GnssSession.bHasStableFixNow   = true;
        }
    }
    else
    {
        if (GnssSession.u8ValidFixCount != 0)
            DBG("gps: fix LOST at t=%us (had %u valid fixes)\r\n",
                u16SessionTmr, GnssSession.u8ValidFixCount);
        GnssSession.u8ValidFixCount  = 0;
        GnssSession.bHasStableFixNow = false;
    }

    GnssSession.Fix = GnssSol.bStatusValid ? GNSS_SESSION_FIX_OK
                    : (GnssSession.u16TtffTmr < GnssSession.u16TtffTimeout)
                        ? GNSS_SESSION_FIX_BUSY : GNSS_SESSION_FIX_ERROR;

    /* Notify caller on first stable fix */
    if (GnssSession.bHasFirstStableFix && !bCallerNotified && GnssCallerTask != NULL)
    {
        taskENTER_CRITICAL();
        GnssStableLat  = GnssSol.Lat.Deg;
        GnssStableLong = GnssSol.Long.Deg;
        taskEXIT_CRITICAL();
        bCallerNotified = true;
        DBG("gps: FIX OK t=%us lat=%i.%06lu lon=%i.%06lu\r\n",
            GnssSession.u16TimeToFirstFix,
            GnssSol.Lat.Deg.i16Deg,  GnssSol.Lat.Deg.u32DeciMicroDeg,
            GnssSol.Long.Deg.i16Deg, GnssSol.Long.Deg.u32DeciMicroDeg);
        osThreadFlagsSet(GnssCallerTask, GPS_NOTIFY_FIX_OK);
    }
    /* Notify caller on TTFF timeout */
    else if (!bCallerNotified && GnssCallerTask != NULL
             && GnssSession.u16TtffTmr >= GnssSession.u16TtffTimeout)
    {
        if (GnssSol.bStatusValid)
        {
            taskENTER_CRITICAL();
            GnssStableLat  = GnssSol.Lat.Deg;
            GnssStableLong = GnssSol.Long.Deg;
            taskEXIT_CRITICAL();
            DBG("gps: FIX TIMEOUT t=%us, using best fix: lat=%i.%06lu lon=%i.%06lu\r\n",
                GnssSession.u16TtffTmr,
                GnssSol.Lat.Deg.i16Deg,  GnssSol.Lat.Deg.u32DeciMicroDeg,
                GnssSol.Long.Deg.i16Deg, GnssSol.Long.Deg.u32DeciMicroDeg);
        }
        else
        {
            DBG("gps: FIX TIMEOUT t=%us, no valid fix\r\n", GnssSession.u16TtffTmr);
        }
        bCallerNotified = true;
        osThreadFlagsSet(GnssCallerTask, GPS_NOTIFY_FIX_TIMEOUT);
    }

#ifndef SWITCH_GNSS_LOGGER
    if (!GnssSession.bDbgStartFlag)
    {
        if (GnssSol.PositionError.bHasValue)
            DBG("\t\t\t\t\t\tgps ttff, sv, snr, fix_cnt, pos_err, [d_lat, d_long]\r\n");
        else
            DBG("\t\t\t\t\t\tgps ttff, sv, snr, fix_cnt, [d_lat, d_long]\r\n");
    }
    GnssSession.bDbgStartFlag = true;

    if (GnssSyslogFlag && !GnssSession.bDbgStopFlag)
    {
        if (GnssSol.PositionError.bHasValue)
            DBG("gps %02u/%02u, %02u/%02u, %u, %u, %u, [%i,%i]",
                GnssSession.u16TtffTmr, GnssSession.u16TtffTimeout,
                GnssSol.Sv.u8SvTracking, GnssSol.Sv.u8SvInView,
                GnssSol.Sv.u8SnrAvgDbHz, GnssSession.u8ValidFixCount,
                GnssSol.PositionError.u16ValueInt,
                (int16_t)i32LatDelta, (int16_t)i32LongDelta);
        else
            DBG("gps %02u/%02u, %02u/%02u, %u, %u, [%i,%i]",
                GnssSession.u16TtffTmr, GnssSession.u16TtffTimeout,
                GnssSol.Sv.u8SvTracking, GnssSol.Sv.u8SvInView,
                GnssSol.Sv.u8SnrAvgDbHz, GnssSession.u8ValidFixCount,
                (int16_t)i32LatDelta, (int16_t)i32LongDelta);
    }

    GnssSession.bDbgStopFlag = (bool)GnssSession.u16TimeToFirstFix
                               || (GnssSession.u16TtffTmr == GnssSession.u16TtffTimeout);
#endif
}

/* --------------------------------------------------------------------------
 * GPS_bGetCoordinates — thread-safe read of the last stable fix
 * -------------------------------------------------------------------------- */
bool GPS_bGetCoordinates(gnss_coord_deg_t *pLat, gnss_coord_deg_t *pLong)
{
    if (!bCallerNotified) return false;
    taskENTER_CRITICAL();
    *pLat  = GnssStableLat;
    *pLong = GnssStableLong;
    taskEXIT_CRITICAL();
    return true;
}

bool GPS_bHasStableFix(void)
{
    return GnssSession.bHasFirstStableFix;
}

/* ===================================================================
 * NMEA parsing helpers
 * =================================================================== */

bool GPS_bParseNmeaMsgGsv(char *pNmeaMsg, gnss_sol_t *pSol)
{
    char *p;
    uint8_t u8totalNumberOfMsgs, u8msgNumber, i, u8SvFieldCnt, u8SvInView;

    if (pSol == NULL) return false;

    p = pNmeaMsg;
    if (!__bFindNext_P(&p, "GPGSV,", true))           return false;
    if (!isdigit((unsigned char)*p))                   return false;
    u8totalNumberOfMsgs = *p - 48;
    if (!__bFindNext_P(&p, ",", true))                 return false;
    if (!isdigit((unsigned char)*p))                   return false;
    u8msgNumber = *p - 48;
    if (!__bFindNext_P(&p, ",", true))                 return false;
    if (!isdigit((unsigned char)*p))                   return false;
    if (!isdigit((unsigned char)*(p + 1)))             return false;
    memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
    strncpy(acGnssStrBuf, p, 2);
    u8SvInView = atoi(acGnssStrBuf);

    if (u8msgNumber == 1) { u16GpsSvSnrAvg = 0; u8GpsSvTrackingCnt = 0; }

    u8SvFieldCnt = (u8totalNumberOfMsgs == u8msgNumber)
                 ? (u8SvInView - ((u8msgNumber - 1) * 4)) : 4;

    for (i = 0; i < u8SvFieldCnt; i++)
    {
        if (!__bFindNext_P(&p, ",", true)) break;
        if (!__bFindNext_P(&p, ",", true)) break;
        if (!__bFindNext_P(&p, ",", true)) break;
        if (!__bFindNext_P(&p, ",", true)) break;
        if (*p != ',')
        {
            if (!isdigit((unsigned char)*p))     return false;
            if (!isdigit((unsigned char)*(p+1))) return false;
            memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
            strncpy(acGnssStrBuf, p, 2);
            u16GpsSvSnrAvg += atoi(acGnssStrBuf);
            u8GpsSvTrackingCnt++;
        }
    }

    if (!__bFindNext_P(&p, ",", true)) return false;
    if (*p != '1') return false;

    if (u8totalNumberOfMsgs == u8msgNumber)
    {
        pSol->Sv.u8SvInView = u8SvInView;
        if (u8GpsSvTrackingCnt)
            pSol->Sv.u8SnrAvgDbHz = (uint8_t)(u16GpsSvSnrAvg / (uint16_t)u8GpsSvTrackingCnt);
        else
            pSol->Sv.u8SnrAvgDbHz = 0;
        pSol->Sv.u8SnrAvgPercent = u8SnrDbHzToPercent(pSol->Sv.u8SnrAvgDbHz);
        pSol->Sv.u8SvTracking    = u8GpsSvTrackingCnt;
        return true;
    }
    return false;
}

bool GPS_bParseNmeaMsgRmc(char *pNmeaMsg, gnss_sol_t *pSol)
{
    char *p;
    uint8_t u8LetLongDecimalCnt, i;

    if (pSol == NULL) return false;
    p = pNmeaMsg;

    if (!__bFindNext_P(&p, "RMC,", true)) return false;

    if (*p != ',')
    {
        for (i = 0; i < 6; i++)
            if (!isdigit((unsigned char)*(p+i))) return false;
        memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
        strncpy(acGnssStrBuf, (p+0), 2); pSol->TimeDateUtc.hour   = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+2), 2); pSol->TimeDateUtc.minute = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+4), 2); pSol->TimeDateUtc.second = atoi(acGnssStrBuf);
    }
    if (!__bFindNext_P(&p, ",", true)) return false;
    if      (*p == 'A') pSol->bStatusValid = true;
    else if (*p == 'V') pSol->bStatusValid = false;
    else if (*p != ',') return false;

    if (!__bFindNext_P(&p, ",", true)) return false;
    if (*p != ',')
    {
        for (i = 0; i < 4; i++)
            if (!isdigit((unsigned char)*(p+i))) return false;
        if (*(p+4) != '.') return false;
        u8LetLongDecimalCnt = 0;
        for (i = 0; i < 6; i++)
            if (isdigit((unsigned char)*(p+5+i))) u8LetLongDecimalCnt++;
        memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
        strncpy(acGnssStrBuf, (p+0), 2); pSol->Lat.Ddm.i16Degrees = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+2), 2); pSol->Lat.Ddm.u8Minutes  = atoi(acGnssStrBuf);
        memset(acGnssStrBuf, '0', 6);
        strncpy(acGnssStrBuf, (p+5), u8LetLongDecimalCnt);
        pSol->Lat.Ddm.u32MicroMinutes = atol(acGnssStrBuf);
    }
    if (!__bFindNext_P(&p, ",", true)) return false;
    if      (*p == 'S') pSol->Lat.Ddm.i16Degrees = -pSol->Lat.Ddm.i16Degrees;
    else if (*p != 'N' && *p != ',') return false;
    GPS_vCoordConvertDdmToDms(&pSol->Lat.Dms, &pSol->Lat.Ddm);
    GPS_vCoordConvertDdmToDeg(&pSol->Lat.Deg, &pSol->Lat.Ddm);

    if (!__bFindNext_P(&p, ",", true)) return false;
    if (*p != ',')
    {
        for (i = 0; i < 5; i++)
            if (!isdigit((unsigned char)*(p+i))) return false;
        if (*(p+5) != '.') return false;
        u8LetLongDecimalCnt = 0;
        for (i = 0; i < 6; i++)
            if (isdigit((unsigned char)*(p+6+i))) u8LetLongDecimalCnt++;
        memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
        strncpy(acGnssStrBuf, (p+0), 3); pSol->Long.Ddm.i16Degrees = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+3), 2); pSol->Long.Ddm.u8Minutes  = atoi(acGnssStrBuf);
        memset(acGnssStrBuf, '0', 6);
        strncpy(acGnssStrBuf, (p+6), u8LetLongDecimalCnt);
        pSol->Long.Ddm.u32MicroMinutes = atol(acGnssStrBuf);
    }
    if (!__bFindNext_P(&p, ",", true)) return false;
    if      (*p == 'W') pSol->Long.Ddm.i16Degrees = -pSol->Long.Ddm.i16Degrees;
    else if (*p != 'E' && *p != ',') return false;
    GPS_vCoordConvertDdmToDms(&pSol->Long.Dms, &pSol->Long.Ddm);
    GPS_vCoordConvertDdmToDeg(&pSol->Long.Deg, &pSol->Long.Ddm);

    if (!__bFindNext_P(&p, ",", true)) return false;
    if (!__bFindNext_P(&p, ",", true)) return false;
    if (!__bFindNext_P(&p, ",", true)) return false;
    if (*p != ',')
    {
        for (i = 0; i < 6; i++)
            if (!isdigit((unsigned char)*(p+i))) return false;
        memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
        strncpy(acGnssStrBuf, (p+0), 2); pSol->TimeDateUtc.date  = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+2), 2); pSol->TimeDateUtc.month = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+4), 2); pSol->TimeDateUtc.year  = atoi(acGnssStrBuf);
        if (pSol->TimeDateUtc.date)  pSol->TimeDateUtc.date--;
        if (pSol->TimeDateUtc.month) pSol->TimeDateUtc.month--;
        pSol->TimeDateUtc.year += 2000;
        pSol->u32TimeUnix = DATETIME_u32DateTimetoTimestamp(&pSol->TimeDateUtc);
    }
    return true;
}

bool GPS_bParseNmeaMsgGga(char *pNmeaMsg, gnss_sol_t *pSol)
{
    char *p;
    uint8_t u8LetLongDecimalCnt, i;

    if (pSol == NULL) return false;
    p = pNmeaMsg;

    if (!__bFindNext_P(&p, "GGA,", true))  return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;
    if (!__bFindNext_P(&p, ",", true))     return false;

    if (*p != ',')
    {
        u8LetLongDecimalCnt = 0;
        for (i = 0; i < 5; i++)
        {
            if (isdigit((unsigned char)*(p+i))) u8LetLongDecimalCnt++;
            else break;
        }
        if (*(p + u8LetLongDecimalCnt) != '.') return false;
        if (!isdigit((unsigned char)*(p + u8LetLongDecimalCnt + 1))) return false;
        memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
        strncpy(acGnssStrBuf, p, u8LetLongDecimalCnt);
        pSol->Altitude.i16Value      = atoi(acGnssStrBuf);
        pSol->Altitude.u8DecimalValue = *p - 48;
    }
    return true;
}

bool GPS_bParseNmeaMsgPQTMEPE(char *pNmeaMsg, gnss_sol_t *pSol)
{
    char *s1, *s2;

    if (!__bFindNext_P(&pNmeaMsg, "PQTMEPE,", true)) return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;

    s1 = pNmeaMsg;
    if (!__bFindNext_P(&pNmeaMsg, ".", true)) return false;
    s2 = pNmeaMsg - 1;

    if (s2 - s1 <= 3)
    {
        memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
        strncpy(acGnssStrBuf, s1, (s2 - s1));
        pSol->PositionError.u16ValueInt = atoi(acGnssStrBuf);
        s1 = pNmeaMsg;
        if (!__bFindNext_P(&pNmeaMsg, ",", true)) return false;
        if (!isdigit((unsigned char)*s1)) return false;
        pSol->PositionError.u8ValueDecimal = *s1 - 48;
    }
    else
    {
        pSol->PositionError.u16ValueInt    = 999;
        pSol->PositionError.u8ValueDecimal = 9;
    }
    pSol->PositionError.bHasValue = true;
    return true;
}

bool GPS_bParseNmeaMsgPubx00(char *pNmeaMsg, gnss_sol_t *pSol)
{
    char *s1, *s2;

    if (pSol == NULL) return false;
    if (!__bFindNext_P(&pNmeaMsg, "PUBX,00,", true)) return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;
    if (!__bFindNext_P(&pNmeaMsg, ",", true))         return false;

    s1 = pNmeaMsg;
    if (!__bFindNext_P(&pNmeaMsg, ",", true)) return false;
    s2 = pNmeaMsg - 1;
    if (s2 - s1 > 5) return false;

    memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
    pSol->PositionError.u8ValueDecimal = 0;
    uint8_t i = 0;
    while (s1 <= s2)
    {
        if (*s1 == '.') { pSol->PositionError.u8ValueDecimal = *(s1+1) - 48; break; }
        else            { acGnssStrBuf[i++] = *s1++; }
    }
    pSol->PositionError.u16ValueInt = atoi(acGnssStrBuf);
    pSol->PositionError.bHasValue   = true;
    return true;
}

/* ---- Coordinate conversion ---- */

void GPS_vCoordConvertDdmToDeg(gnss_coord_deg_t *pDeg, const gnss_coord_ddm_t *pDdm)
{
    int32_t i32MicroDegrees;

    pDeg->i16Deg         = pDdm->i16Degrees;
    pDeg->u32DeciMicroDeg = (uint32_t)pDdm->u8Minutes * 10000;
    pDeg->u32DeciMicroDeg += (pDdm->u32MicroMinutes / 100);
    pDeg->u32DeciMicroDeg *= 100;
    pDeg->u32DeciMicroDeg /= 60;

    i32MicroDegrees = (pDeg->i16Deg > 0) ? (int32_t)pDeg->u32DeciMicroDeg
                                          : -(int32_t)pDeg->u32DeciMicroDeg;
    pDeg->fDegrees  = ((float)pDeg->i16Deg) + ((float)i32MicroDegrees) / 1000000.0f;
    pDeg->i32MicroDeg = 1000000 * ((int32_t)pDeg->i16Deg) + i32MicroDegrees;
}

void GPS_vCoordConvertDdmToDms(gnss_coord_dms_t *pDms, const gnss_coord_ddm_t *pDdm)
{
    uint16_t u16;
    pDms->i16Degrees = pDdm->i16Degrees;
    pDms->u8Minutes  = pDdm->u8Minutes;
    u16 = (60 * pDdm->u32MicroMinutes) / 1000000;
    pDms->u8Seconds  = (uint8_t)u16;
    u16 = (60 * pDdm->u32MicroMinutes) % 1000000;
    pDms->u16MilliSeconds = u16;
}

/* ---- NMEA integrity ---- */

bool GPS_bCheckNmeaMsg(char *pacNmeaMsg)
{
    char *s1, *s2;
    uint8_t u8CkCalc, u8CkMsg;

    s1 = pacNmeaMsg;
    if (!__bFindNext_P(&s1, "$", true))    return false;
    s2 = s1;
    if (!__bFindNext_P(&s2, "\r\n", false)) return false;
    s2 = s1;
    if (!__bFindNext_P(&s2, "*", false))   return false;
    s2--;

    u8CkCalc = u8CalcNmeaChecksum(s1, s2);
    if (!__bFindNext_P(&s1, "*", true))    return false;
    s2 = s1;
    if (!__bFindNext_P(&s2, "\r\n", false)) return false;
    s2--;
    if (s2 - s1 != 1) return false;
    u8CkMsg = (u8HexCharToInt(*s1) << 4) | u8HexCharToInt(*s2);
    return (u8CkCalc == u8CkMsg);
}

uint8_t u8HexCharToInt(char cHexChar)
{
    if (cHexChar >= 48 && cHexChar <= 57) return cHexChar - 48;
    if (cHexChar >= 65 && cHexChar <= 70) return cHexChar - 65 + 10;
    return 0;
}

uint8_t u8CalcNmeaChecksum(char *pacStart, char *pacStop)
{
    uint8_t u8Ck = 0;
    char *p = pacStart;
    while (p <= pacStop) { u8Ck ^= *p; p++; }
    return u8Ck;
}

bool bAddUbxChecksum(uint8_t *pUbxFrame, uint16_t u16Len)
{
    if (*pUbxFrame++ != 0xB5) return false;
    if (*pUbxFrame++ != 0x62) return false;
    uint8_t ck_a = 0, ck_b = 0;
    u16Len -= 4;
    while (u16Len--) { ck_a += *pUbxFrame++; ck_b += ck_a; }
    *pUbxFrame++ = ck_a;
    *pUbxFrame   = ck_b;
    return true;
}
