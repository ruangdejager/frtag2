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
#include "Power.h"
#include "platform_rtc.h"
#include "dbg_log.h"
#include "Debug.h"
#include "hal_system.h"

#include <limits.h>

/* ---- Task parameters ---- */
#define GPS_RX_TASK_PRIORITY      osPriorityNormal
#define GPS_RX_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 8)

#define GPS_DISP_TASK_PRIORITY    osPriorityNormal
#define GPS_DISP_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE * 2)

/* ---- Thread flags ---- */
#define GPS_THREAD_FLAG_RX        0x0001U   /* UART ISR  → RX task   */

#define GPS_DISP_TRIGGER_BIT      (1UL << 0)   /* RequestFix → dispatcher */
#define GPS_DISP_FIX_OK_BIT       (1UL << 1)   /* RX task    → dispatcher */
#define GPS_DISP_FIX_TIMEOUT_BIT  (1UL << 2)   /* RX task    → dispatcher */
#define GPS_DISP_SHUTDOWN_BIT     (1UL << 3)   /* Shutdown   → dispatcher */

/* ---- Result event-flags bits (fan-out to waiters in GPS_eWaitForFix) ---- */
#define GPS_RESULT_FIX_OK_BIT       (1UL << 0)
#define GPS_RESULT_FIX_TIMEOUT_BIT  (1UL << 1)
#define GPS_RESULT_NO_POWER_BIT     (1UL << 2)
#define GPS_RESULT_ALL_BITS \
    (GPS_RESULT_FIX_OK_BIT | GPS_RESULT_FIX_TIMEOUT_BIT | GPS_RESULT_NO_POWER_BIT)

/* ---- Dispatcher state machine ---- */
typedef enum {
    GPS_STATE_IDLE      = 0,   /* GPS off, waiting for trigger              */
    GPS_STATE_ACQUIRING = 1,   /* GPS powered, RX task searching for fix    */
    GPS_STATE_FIX_HELD  = 2,   /* Fix held; GPS still powered (auto=false)  */
} gps_state_e;

/* ---- Task handles ---- */
static osThreadId_t  GPS_vRxTask_handle;
static osThreadId_t  GPS_vDispatcherTask_handle;

/* ---- Solution/session state (NMEA parser ownership) ---- */
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

/* ---- UBX-ACK-ACK/NAK capture (diagnostics for UBX-CFG-VALSET) ----
 * Fixed-size frame: sync(2) B5 62 + class(1) + id(1) + len(2)=02 00
 * + payload(2)=ackedClass,ackedId + ck(2) = 10 bytes total. */
#define GPS_UBX_ACK_FRAME_LEN  10
static bool    bUbxFrameActive;
static uint8_t u8UbxRxBufIdx;
static uint8_t acUbxRxBuf[GPS_UBX_ACK_FRAME_LEN];

uint16_t u16GpsSvSnrAvg;
uint8_t  u8GpsSvTrackingCnt;

bool GnssSyslogFlag = true;

/* ---- Dispatcher state (mutex-guarded except where atomic) ---- */
static volatile gps_state_e eState         = GPS_STATE_IDLE;
static volatile bool        bAutoShutdown  = true;
static volatile uint16_t    u16PendingTtffTimeout = GNSS_TTFF_TIMEOUT_1_AID_ASSIST;
static bool                 bSessionActive = false;   /* drives RX-task gating      */
static bool                 bCallerNotified= false;   /* GNSS_vOnSolution edge       */
static bool                 bGpsPowered    = false;   /* hardware rail + UART state  */
static osEventFlagsId_t     xGpsResultFlags = NULL;
static osMutexId_t          xGpsLock        = NULL;

/* ---- Live session stable coordinates (current session only) ---- */
static gnss_coord_deg_t GnssStableLat;
static gnss_coord_deg_t GnssStableLong;

/* ---- Last-known-fix retention (persists across sessions) ---- */
static bool             bHasEverFixed  = false;
static gnss_coord_deg_t tLastFixLat;
static gnss_coord_deg_t tLastFixLon;
static uint64_t         u64LastFixUtc  = 0;  /* RTC_u64GetUTC() at acquisition */

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

static void GPS_vRxTask(void *parameters);
static void GPS_vDispatcherTask(void *parameters);
static void GPS_vSessionArm(void);
static void GPS_vPowerOff(void);
static void GPS_vMaybeSyncRtc(uint32_t u32GpsUnixTime);

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
 * UBX-CFG-VALSET cold-start configuration (MAX-M10S, PROTVER 34.10)
 *
 * Sent once per cold start (GPS_vSessionArm) to configure the receiver for:
 *   - 1 Hz measurement/navigation rate (explicit, belt-and-braces)
 *   - GPS-only (L1C/A) reception for lowest power draw. A single-constellation
 *     receiver runs fewer correlator channels, which is the dominant RX power
 *     cost on the M10. The capture from this unit shows 12-15 GPS SVs in view
 *     with good C/N0 (open-sky), so dropping GLONASS/Galileo/BeiDou/QZSS/SBAS
 *     should not meaningfully hurt TTFF here while cutting acquisition/tracking
 *     power. (SBAS is dropped too — it adds power for a DGNSS correction this
 *     application doesn't need.)
 *   - NMEA output trimmed to RMC + GGA + GSV (GSA/VTG/GLL disabled). GSV is
 *     forced to 1 report/epoch (CFG-MSGOUT-NMEA_ID_GSV_UART1=1) so it arrives
 *     every second along with RMC — required because GnssSession.tNmeaMsgFlagsBm
 *     is RMC_bm | GSV_bm, i.e. GNSS_vOnSolution() needs both per cycle. With
 *     GPS-only there's a single GPGSV sequence per epoch, so this now lines up
 *     1:1 with RMC instead of the ~5 s GSV burst period seen with multi-GNSS.
 *   - CFG-PM-OPERATEMODE=FULL — the receiver's own power-save/cyclic-tracking
 *     stays off; our own duty-cycling (GPS_vRequestFix/GPS_vPowerOff) already
 *     handles power, and PSM would re-introduce a slow/irregular output rate.
 *   - CFG-NAVSPG-DYNMODEL=Pedestrian — appropriate for a grazing-animal tag
 *     (low velocity/acceleration), helps the navigation filter without
 *     affecting TTFF or power.
 *
 * Layers = RAM | BBR (0x03). The M10 has no config flash; BBR is retained via
 * V_BCKP across power-down so this only needs to be (re-)applied on cold start.
 *
 * Frames are stored with placeholder checksum bytes; GPS_vSendUbxValset() copies
 * each to a scratch buffer and finalizes the checksum via bAddUbxChecksum()
 * before transmitting on gps.UartHandle.
 * -------------------------------------------------------------------------- */

/* CFG-RATE: MEAS=1000 ms, NAV=1 -> 1 Hz */
static const uint8_t aUbxCfgRate[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x10, 0x00,            /* hdr: CFG-VALSET, len=16 */
    0x00, 0x03, 0x00, 0x00,                        /* version, layers=RAM|BBR, reserved */
    0x01, 0x00, 0x21, 0x30, 0xE8, 0x03,            /* CFG-RATE-MEAS (U2) = 1000 */
    0x02, 0x00, 0x21, 0x30, 0x01, 0x00,            /* CFG-RATE-NAV  (U2) = 1    */
    0x00, 0x00,                                    /* checksum placeholder */
};

/* CFG-SIGNAL: GPS L1C/A only — disable GLONASS/Galileo/BeiDou/QZSS/SBAS */
static const uint8_t aUbxCfgSignalGpsOnly[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x40, 0x00,            /* hdr: CFG-VALSET, len=64 */
    0x00, 0x03, 0x00, 0x00,                        /* version, layers=RAM|BBR, reserved */
    0x1F, 0x00, 0x31, 0x10, 0x01,                  /* CFG-SIGNAL-GPS_ENA       = 1 */
    0x01, 0x00, 0x31, 0x10, 0x01,                  /* CFG-SIGNAL-GPS_L1CA_ENA  = 1 */
    0x20, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-SBAS_ENA      = 0 */
    0x05, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-SBAS_L1CA_ENA = 0 */
    0x21, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-GAL_ENA       = 0 */
    0x07, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-GAL_E1_ENA    = 0 */
    0x22, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-BDS_ENA       = 0 */
    0x0D, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-BDS_B1_ENA    = 0 */
    0x24, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-QZSS_ENA      = 0 */
    0x12, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-QZSS_L1CA_ENA = 0 */
    0x25, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-GLO_ENA       = 0 */
    0x18, 0x00, 0x31, 0x10, 0x00,                  /* CFG-SIGNAL-GLO_L1_ENA    = 0 */
    0x00, 0x00,                                    /* checksum placeholder */
};

/* CFG-MSGOUT: NMEA on UART1 — keep RMC/GGA, force GSV to every epoch, drop GSA/VTG/GLL */
static const uint8_t aUbxCfgMsgOut[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x22, 0x00,            /* hdr: CFG-VALSET, len=34 */
    0x00, 0x03, 0x00, 0x00,                        /* version, layers=RAM|BBR, reserved */
    0xC5, 0x00, 0x91, 0x20, 0x01,                  /* CFG-MSGOUT-NMEA_ID_GSV_UART1 = 1 */
    0xC0, 0x00, 0x91, 0x20, 0x00,                  /* CFG-MSGOUT-NMEA_ID_GSA_UART1 = 0 */
    0xCA, 0x00, 0x91, 0x20, 0x00,                  /* CFG-MSGOUT-NMEA_ID_GLL_UART1 = 0 */
    0xB1, 0x00, 0x91, 0x20, 0x00,                  /* CFG-MSGOUT-NMEA_ID_VTG_UART1 = 0 */
    0xBB, 0x00, 0x91, 0x20, 0x01,                  /* CFG-MSGOUT-NMEA_ID_GGA_UART1 = 1 */
    0xAB, 0x00, 0x91, 0x20, 0x01,                  /* CFG-MSGOUT-NMEA_ID_RMC_UART1 = 1 */
    0x00, 0x00,                                    /* checksum placeholder */
};

/* CFG-PM / CFG-NAVSPG: full-power (no internal PSM), pedestrian dynamic model */
static const uint8_t aUbxCfgPmNav[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x0E, 0x00,            /* hdr: CFG-VALSET, len=14 */
    0x00, 0x03, 0x00, 0x00,                        /* version, layers=RAM|BBR, reserved */
    0x01, 0x00, 0xD0, 0x20, 0x00,                  /* CFG-PM-OPERATEMODE  (E1) = 0 (FULL) */
    0x21, 0x00, 0x11, 0x20, 0x03,                  /* CFG-NAVSPG-DYNMODEL (E1) = 3 (Pedestrian) */
    0x00, 0x00,                                    /* checksum placeholder */
};

/* --------------------------------------------------------------------------
 * GPS_vSendUbxValset — finalize checksum and transmit one UBX-CFG-VALSET frame.
 * Copies the const template to a scratch buffer (bAddUbxChecksum writes the
 * last two bytes in place) and pushes it out via the TX ring buffer.
 * -------------------------------------------------------------------------- */
static void GPS_vSendUbxValset(const uint8_t *pTemplate, uint16_t u16Len)
{
    uint8_t acFrame[80];

    if (u16Len > sizeof(acFrame))
        return;

    memcpy(acFrame, pTemplate, u16Len);
    bAddUbxChecksum(acFrame, u16Len);
    HAL_UART_vTxPutBuffer(&gps.UartHandle, acFrame, u16Len);
}

/* --------------------------------------------------------------------------
 * GPS_vConfigureModule — send the cold-start UBX-CFG-VALSET sequence.
 * See the frame definitions above for the rationale of each setting.
 * -------------------------------------------------------------------------- */
static void GPS_vConfigureModule(void)
{
    GPS_vSendUbxValset(aUbxCfgRate,          sizeof(aUbxCfgRate));
    GPS_vSendUbxValset(aUbxCfgSignalGpsOnly, sizeof(aUbxCfgSignalGpsOnly));
    GPS_vSendUbxValset(aUbxCfgMsgOut,        sizeof(aUbxCfgMsgOut));
    GPS_vSendUbxValset(aUbxCfgPmNav,         sizeof(aUbxCfgPmNav));

    DBG_LOG("gps: ubx config sent (1Hz, GPS-only, full power, pedestrian)\r\n");
}

/* --------------------------------------------------------------------------
 * GPS_vInit — set up UART handle, mutex, event flags, and worker tasks
 * -------------------------------------------------------------------------- */
void GPS_vInit(void)
{

    xGpsLock = osMutexNew(NULL);
    configASSERT(xGpsLock != NULL);

    xGpsResultFlags = osEventFlagsNew(NULL);
    configASSERT(xGpsResultFlags != NULL);

    static const osThreadAttr_t rx_attr = {
        .name       = "GPSRxTask",
        .stack_size = GPS_RX_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = GPS_RX_TASK_PRIORITY,
    };
    GPS_vRxTask_handle = osThreadNew(GPS_vRxTask, NULL, &rx_attr);
    configASSERT(GPS_vRxTask_handle != NULL);

    static const osThreadAttr_t disp_attr = {
        .name       = "GPSDispatcher",
        .stack_size = GPS_DISP_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = GPS_DISP_TASK_PRIORITY,
    };
    GPS_vDispatcherTask_handle = osThreadNew(GPS_vDispatcherTask, NULL, &disp_attr);
    configASSERT(GPS_vDispatcherTask_handle != NULL);

    DBG_LOG("gps: init\r\n");
}

/* --------------------------------------------------------------------------
 * GPS_vSessionArm — internal: reset session state and power up GPS
 * Caller must hold xGpsLock.
 * -------------------------------------------------------------------------- */
static void GPS_vSessionArm(void)
{
    memset(&GnssSession, 0, sizeof(gnss_session_t));
    GnssSession.u32StartTime    = (uint32_t)RTC_u64GetTicks();
    GnssSession.u16TtffTimeout  = u16PendingTtffTimeout;
    GnssSession.tNmeaMsgFlagsBm = GNSS_NMEA_MSG_RMC_bm | GNSS_NMEA_MSG_GSV_bm;// | GNSS_NMEA_MSG_PQTMEPE_bm;
    GnssSession.Fix             = GNSS_SESSION_FIX_BUSY;
    memset(&GnssSolDraft, 0, sizeof(gnss_sol_t));

    bCallerNotified = false;
    bSessionActive  = true;

    if (!bGpsPowered)
    {
        /* Cold start — power the hardware and enable the UART */
    	GPS_DRIVER_vInitGPS(&gps.UartHandle);
        GPS_DRIVER_vEnableUart(&gps.UartHandle);
        HAL_UART_vClearBuffer(&gps.UartHandle);
        GPS_DRIVER_vPowerEnHigh();
        bGpsPowered = true;

        /* Apply 1Hz/GPS-only/full-power config. The MAX-M10S emits its first
         * $GNTXT boot messages almost immediately, but its UART *receiver*
         * is not guaranteed to be live yet — frames sent at t=0 are silently
         * lost. Wait briefly, send the VALSET sequence, then send it again
         * after a further delay as a hedge against a slow boot. Idempotent
         * (same keys both times); any UBX-ACK/NAK is logged by
         * GPS_bOnRxByte for diagnostics. */
        osDelay(300);
        GPS_vConfigureModule();
        osDelay(700);
        GPS_vConfigureModule();

        DBG_LOG("gps: session armed (cold start), ttff timeout=%us\r\n",
            GnssSession.u16TtffTimeout);
    }
    else
    {
        /* Re-arm from FIX_HELD — GPS already powered, just reset session state.
         * Flush the UART buffer so stale NMEA from the held session is discarded. */
        HAL_UART_vClearBuffer(&gps.UartHandle);
        DBG_LOG("gps: session re-armed (GPS already on), ttff timeout=%us\r\n",
            GnssSession.u16TtffTimeout);
    }

    eState = GPS_STATE_ACQUIRING;
    DBG_LOG("gps: start, ttff timeout=%us\r\n", GnssSession.u16TtffTimeout);
}

/* --------------------------------------------------------------------------
 * GPS_vPowerOff — internal: power down GPS and disable UART
 * Caller must hold xGpsLock.
 * -------------------------------------------------------------------------- */
static void GPS_vPowerOff(void)
{
    bSessionActive = false;
    bGpsPowered    = false;
    GPS_DRIVER_vPowerEnLow();
    GPS_DRIVER_vDisableUart(&gps.UartHandle);
    eState = GPS_STATE_IDLE;
    DBG_LOG("gps: powered off\r\n");

    /* GPS is fully off — release the sleep lock taken by whoever requested
     * this session (e.g. the DeviceDiscovery GPS pre-trigger), allowing
     * deep sleep again once any other outstanding lock is also released. */
    SYSTEM_vSleepLockRelease();
}

/* --------------------------------------------------------------------------
 * GPS_vMaybeSyncRtc — apply GPS-derived UTC to the RTC if the error exceeds
 * GPS_RTC_SYNC_MIN_ERROR_S.  Called at most once per session (FIX_OK or
 * TTFF timeout).  Time can be decoded from the NMEA RMC sentence before a
 * full position fix is available, so this is attempted on both terminal paths.
 * -------------------------------------------------------------------------- */
static void GPS_vMaybeSyncRtc(uint32_t u32GpsUnixTime)
{
    if (u32GpsUnixTime == 0U) return;   /* RMC time fields not yet parsed */

    uint64_t u64RtcNow = RTC_u64GetUTC();
    int64_t  i64Delta  = (int64_t)u32GpsUnixTime - (int64_t)u64RtcNow;
    if (i64Delta < 0) i64Delta = -i64Delta;

    if ((uint64_t)i64Delta > (uint64_t)GPS_RTC_SYNC_MIN_ERROR_S)
    {
//        RTC_vSetUTC((uint64_t)u32GpsUnixTime);
        DBG("gps: RTC synced from GPS (delta=%lds)\r\n", (long)i64Delta);
    }
    else
    {
        DBG("gps: RTC within tolerance (delta=%lds), no sync\r\n", (long)i64Delta);
    }
}

/* --------------------------------------------------------------------------
 * GPS_vRequestFix — public: trigger (or attach to) a fix acquisition.
 * Idempotent; updates bAutoShutdown (and the TTFF timeout) to the
 * last-caller values.
 * Refuses if board is not in POWER_CLASS_NORMAL — sets NO_POWER result.
 * -------------------------------------------------------------------------- */
void GPS_vRequestFix(bool bAutoShutdownIn, uint32_t u32TtffTimeoutS)
{
    /* Safe no-op if GPS_vInit() was never called (e.g. PRIMARY device or
     * ENABLE_GPS not defined). The wake-schedule task fires this
     * unconditionally on every cycle, so being defensive here is required. */
    if (xGpsLock == NULL || xGpsResultFlags == NULL) return;

    osMutexAcquire(xGpsLock, osWaitForever);

    /* ---- Power-class gate ---- */
    if ((POWER_tGetState() & POWER_CLASS_NORMAL) == 0U)
    {
        DBG("gps: request refused — power class not NORMAL\r\n");
        osEventFlagsClear(xGpsResultFlags, GPS_RESULT_ALL_BITS);
        osEventFlagsSet(xGpsResultFlags,   GPS_RESULT_NO_POWER_BIT);
        osMutexRelease(xGpsLock);
        return;
    }

    bAutoShutdown = bAutoShutdownIn;

    /* ---- TTFF timeout: only enforced when auto-shutdown is requested ----
     * Without auto-shutdown the session is held open (FIX_HELD) until
     * GPS_vShutdown() is called, so it is allowed to run "forever". */
    if (bAutoShutdownIn)
    {
        if (u32TtffTimeoutS == 0U)
            u16PendingTtffTimeout = GNSS_TTFF_TIMEOUT_1_AID_ASSIST;
        else if (u32TtffTimeoutS > (uint32_t)UINT16_MAX)
            u16PendingTtffTimeout = UINT16_MAX;
        else
            u16PendingTtffTimeout = (uint16_t)u32TtffTimeoutS;
    }
    else
    {
        u16PendingTtffTimeout = GNSS_TTFF_NO_TIMEOUT;
    }

    switch (eState)
    {
    case GPS_STATE_IDLE:
        /* Cold start — clear stale result flags and kick the dispatcher */
        osEventFlagsClear(xGpsResultFlags, GPS_RESULT_ALL_BITS);
        osThreadFlagsSet(GPS_vDispatcherTask_handle, GPS_DISP_TRIGGER_BIT);
        DBG("gps: request — dispatcher kicked (auto=%u)\r\n", (unsigned)bAutoShutdownIn);
        break;

    case GPS_STATE_ACQUIRING:
        /* Already searching — last-caller bAutoShutdown wins */
        DBG("gps: request attached (auto=%u)\r\n", (unsigned)bAutoShutdownIn);
        break;

    case GPS_STATE_FIX_HELD:
        if (GnssSession.bHasFirstStableFix)
        {
            /* A live fix is available (either from the original acquisition or
             * acquired in the interim after a timeout).  Signal FIX_OK immediately
             * so any concurrent GPS_eWaitForFix() caller is released at once. */
            DBG("gps: request — live fix available, signalling FIX_OK immediately\r\n");
            osEventFlagsClear(xGpsResultFlags, GPS_RESULT_ALL_BITS);
            osEventFlagsSet(xGpsResultFlags, GPS_RESULT_FIX_OK_BIT);
            if (bAutoShutdownIn)
            {
                DBG("gps: auto-shutdown after immediate FIX_OK\r\n");
                GPS_vPowerOff();
            }
            /* else: GPS stays powered, continuous position updates continue */
        }
        else
        {
            /* GPS is on but no stable fix yet (TTFF timed out, still searching).
             * Re-arm the session with a fresh TTFF budget.  The dispatcher is
             * waiting at its outer loop; sending TRIGGER_BIT causes it to call
             * GPS_vSessionArm(), which skips the power-on (bGpsPowered=true) and
             * only resets the session counters. */
            DBG("gps: request — no fix in FIX_HELD, re-arming session\r\n");
            osEventFlagsClear(xGpsResultFlags, GPS_RESULT_ALL_BITS);
            osThreadFlagsSet(GPS_vDispatcherTask_handle, GPS_DISP_TRIGGER_BIT);
        }
        break;
    }

    osMutexRelease(xGpsLock);
}

/* --------------------------------------------------------------------------
 * GPS_eWaitForFix — public: block until session signals a terminal result.
 * Multiple callers may wait concurrently; osFlagsNoClear keeps the flag
 * set across all waiters until the next GPS_vRequestFix() clears them.
 * -------------------------------------------------------------------------- */
gps_result_e GPS_eWaitForFix(uint32_t u32TimeoutMs)
{
    if (xGpsResultFlags == NULL) return GPS_RESULT_FIX_TIMEOUT;

    uint32_t flags = osEventFlagsWait(xGpsResultFlags,
                                       GPS_RESULT_ALL_BITS,
                                       osFlagsWaitAny | osFlagsNoClear,
                                       u32TimeoutMs);

    if (flags & osFlagsError)
        return GPS_RESULT_FIX_TIMEOUT;   /* caller's own wait timed out */

    if (flags & GPS_RESULT_NO_POWER_BIT)    return GPS_RESULT_NO_POWER;
    if (flags & GPS_RESULT_FIX_OK_BIT)      return GPS_RESULT_FIX_OK;
    return GPS_RESULT_FIX_TIMEOUT;
}

/* --------------------------------------------------------------------------
 * GPS_vShutdown — public: manual power-down (no-op if already off)
 * -------------------------------------------------------------------------- */
void GPS_vShutdown(void)
{
    if (xGpsLock == NULL) return;

    osMutexAcquire(xGpsLock, osWaitForever);
    if (eState != GPS_STATE_IDLE)
    {
        DBG("gps: manual shutdown\r\n");
        GPS_vPowerOff();
        /* Signal dispatcher in case it's mid-wait — it observes eState==IDLE
         * and returns to its outer trigger-wait. */
        osThreadFlagsSet(GPS_vDispatcherTask_handle, GPS_DISP_SHUTDOWN_BIT);
    }
    osMutexRelease(xGpsLock);
}

/* --------------------------------------------------------------------------
 * GPS_vRxTask — CMSIS-RTOS v2 task; blocks on thread flag from ISR.
 * Internal — only the dispatcher gates sessions on/off via bSessionActive.
 * -------------------------------------------------------------------------- */
static void GPS_vRxTask(void *parameters)
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
    uint8_t u8Byte = (uint8_t)pcRxByte;

    /* UBX-ACK-ACK/ACK-NAK capture — diagnostics for the UBX-CFG-VALSET
     * frames sent at cold start. These are binary (start with 0xB5 0x62),
     * so they're handled separately from the '$'-delimited NMEA path below
     * and never enter acGpsRxBuf. */
    if (u8GpsRxBufIdx == 0 && !bUbxFrameActive && u8Byte == 0xB5)
    {
        bUbxFrameActive = true;
        u8UbxRxBufIdx   = 0;
    }

    if (bUbxFrameActive)
    {
        if (u8UbxRxBufIdx < GPS_UBX_ACK_FRAME_LEN)
            acUbxRxBuf[u8UbxRxBufIdx] = u8Byte;
        u8UbxRxBufIdx++;

        if (u8UbxRxBufIdx >= GPS_UBX_ACK_FRAME_LEN)
        {
            /* sync 0xB5 0x62 already matched at index 0; verify class==ACK
             * (0x05) and checksum over class..payload (indices 2..7). */
            if (acUbxRxBuf[1] == 0x62 && acUbxRxBuf[2] == 0x05)
            {
                uint8_t ck_a = 0, ck_b = 0;
                for (uint8_t i = 2; i < 8; i++)
                {
                    ck_a += acUbxRxBuf[i];
                    ck_b += ck_a;
                }
                if (ck_a == acUbxRxBuf[8] && ck_b == acUbxRxBuf[9])
                {
                    DBG_LOG("gps: ubx %s %02X/%02X\r\n",
                        (acUbxRxBuf[3] == 0x01) ? "ACK" : "NAK",
                        acUbxRxBuf[6], acUbxRxBuf[7]);
                }
            }
            bUbxFrameActive = false;
            u8UbxRxBufIdx   = 0;
        }
        return true;
    }

    if (u8GpsRxBufIdx == 0 && pcRxByte != '$') return true;

    uint8_t u8NextIdx = u8GpsRxBufIdx + 1;
    if (u8NextIdx < GPS_RX_BUF_LEN)
    {
        acGpsRxBuf[u8GpsRxBufIdx] = pcRxByte;
        acGpsRxBuf[u8NextIdx]     = 0;
//        DEBUG_vPutByte((uint8_t)pcRxByte);
    }
    else
    {
        DBG_LOG("gps: rx buf overflow\r\n");
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
            DBG_LOG("gps: fix LOST at t=%us (had %u valid fixes)\r\n",
                u16SessionTmr, GnssSession.u8ValidFixCount);
        GnssSession.u8ValidFixCount  = 0;
        GnssSession.bHasStableFixNow = false;
    }

    GnssSession.Fix = GnssSol.bStatusValid ? GNSS_SESSION_FIX_OK
                    : (GnssSession.u16TtffTmr < GnssSession.u16TtffTimeout)
                        ? GNSS_SESSION_FIX_BUSY : GNSS_SESSION_FIX_ERROR;

    /* Continuously update position whenever a stable fix is available.
     *
     * The position update is intentionally NOT gated by bCallerNotified — when
     * the GPS is held on (bAutoShutdown=false, GPS_STATE_FIX_HELD), the RX task
     * keeps running and we want GnssStableLat/Long and the last-known-fix to
     * track the freshest data on every NMEA cycle.
     *
     * The dispatcher signal (osThreadFlagsSet) IS gated by !bCallerNotified so
     * the dispatcher wakes only once per session acquisition.
     */
    if (GnssSession.bHasFirstStableFix)
    {
        taskENTER_CRITICAL();
        GnssStableLat  = GnssSol.Lat.Deg;
        GnssStableLong = GnssSol.Long.Deg;

        /* Last-known-fix retention — persists across sessions */
        tLastFixLat   = GnssSol.Lat.Deg;
        tLastFixLon   = GnssSol.Long.Deg;
        u64LastFixUtc = RTC_u64GetUTC();
        bHasEverFixed = true;
        taskEXIT_CRITICAL();

        if (!bCallerNotified)
        {
            /* First stable fix of this session — sync RTC then wake dispatcher */
            bCallerNotified = true;
            GPS_vMaybeSyncRtc(GnssSol.u32TimeUnix);
            DBG_LOG("gps: FIX OK t=%us/%us lat=%i.%06lu lon=%i.%06lu\r\n",
                GnssSession.u16TimeToFirstFix, GnssSession.u16TtffTimeout,
                GnssSol.Lat.Deg.i16Deg,  GnssSol.Lat.Deg.u32DeciMicroDeg,
                GnssSol.Long.Deg.i16Deg, GnssSol.Long.Deg.u32DeciMicroDeg);
            osThreadFlagsSet(GPS_vDispatcherTask_handle, GPS_DISP_FIX_OK_BIT);
        }
    }
    /* Notify dispatcher on TTFF timeout (position update only if a best-effort
     * fix is available; no continuous update path — timeout is a terminal event
     * for the current acquisition, though the GPS may stay on in FIX_HELD). */
    else if (!bCallerNotified
             && GnssSession.u16TtffTmr >= GnssSession.u16TtffTimeout)
    {
        if (GnssSol.bStatusValid)
        {
            taskENTER_CRITICAL();
            GnssStableLat  = GnssSol.Lat.Deg;
            GnssStableLong = GnssSol.Long.Deg;

            /* Best-effort fix at timeout — still retain as last-known */
            tLastFixLat   = GnssSol.Lat.Deg;
            tLastFixLon   = GnssSol.Long.Deg;
            u64LastFixUtc = RTC_u64GetUTC();
            bHasEverFixed = true;
            taskEXIT_CRITICAL();
            DBG_LOG("gps: FIX TIMEOUT t=%us/%us, using best fix: lat=%i.%06lu lon=%i.%06lu\r\n",
                GnssSession.u16TtffTmr, GnssSession.u16TtffTimeout,
                GnssSol.Lat.Deg.i16Deg,  GnssSol.Lat.Deg.u32DeciMicroDeg,
                GnssSol.Long.Deg.i16Deg, GnssSol.Long.Deg.u32DeciMicroDeg);
        }
        else
        {
            DBG_LOG("gps: FIX TIMEOUT t=%us/%us, no valid fix\r\n",
                GnssSession.u16TtffTmr, GnssSession.u16TtffTimeout);
        }
        /* Attempt RTC sync regardless of position validity — NMEA time fields
         * are typically decoded well before a position fix is declared. */
        GPS_vMaybeSyncRtc(GnssSol.u32TimeUnix);
        bCallerNotified = true;
        osThreadFlagsSet(GPS_vDispatcherTask_handle, GPS_DISP_FIX_TIMEOUT_BIT);
    }

/* Per-sample GNSS telemetry CSV — one line per second of every fix session.
 * Off by default: it floods the flash log during a campaign (alongside the mesh
 * burst) yet duplicates information already summarised by the per-session
 * "FIX OK"/"FIX TIMEOUT" lines. Define LOG_GPS_PERIODIC for bench TTFF tuning.
 * Still suppressed entirely in the dedicated SWITCH_GNSS_LOGGER build. */
#if !defined(SWITCH_GNSS_LOGGER) && defined(LOG_GPS_PERIODIC)
    if (!GnssSession.bDbgStartFlag)
    {
        if (GnssSol.PositionError.bHasValue)
            DBG_LOG("gps ttff, sv, snr, fix_cnt, pos_err, [d_lat, d_long]\r\n");
        else
            DBG_LOG("gps ttff, sv, snr, fix_cnt, [d_lat, d_long]\r\n");
    }
    GnssSession.bDbgStartFlag = true;

    if ( !GnssSession.bDbgStopFlag && (GnssSession.u8ValidFixCount || (u16SessionTmr % 10 == 0) ) )
    {
        if (GnssSol.PositionError.bHasValue)
            DBG_LOG("gps %02u/%02u, %02u/%02u, %u, %u, %u, [%i,%i]\r\n",
                GnssSession.u16TtffTmr, GnssSession.u16TtffTimeout,
                GnssSol.Sv.u8SvTracking, GnssSol.Sv.u8SvInView,
                GnssSol.Sv.u8SnrAvgDbHz, GnssSession.u8ValidFixCount,
                GnssSol.PositionError.u16ValueInt,
                (int16_t)i32LatDelta, (int16_t)i32LongDelta);
        else
            DBG_LOG("gps %02u/%02u, %02u/%02u, %u, %u, [%i,%i]\r\n",
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

/* --------------------------------------------------------------------------
 * GPS_bGetLastKnownFix — retained across sessions; returns false if no fix
 * has ever been acquired since boot. Age is in seconds (RTC UTC delta).
 * -------------------------------------------------------------------------- */
bool GPS_bGetLastKnownFix(gnss_coord_deg_t *pLat,
                          gnss_coord_deg_t *pLon,
                          uint32_t         *pu32AgeSeconds)
{
    if (!bHasEverFixed)
    {
        if (pu32AgeSeconds != NULL) *pu32AgeSeconds = UINT32_MAX;
        return false;
    }

    taskENTER_CRITICAL();
    if (pLat != NULL) *pLat = tLastFixLat;
    if (pLon != NULL) *pLon = tLastFixLon;
    uint64_t u64Fixed = u64LastFixUtc;
    taskEXIT_CRITICAL();

    if (pu32AgeSeconds != NULL)
    {
        uint64_t u64Now = RTC_u64GetUTC();
        *pu32AgeSeconds = (u64Now > u64Fixed) ? (uint32_t)(u64Now - u64Fixed) : 0U;
    }
    return true;
}

uint32_t GPS_u32GetLastFixAgeSeconds(void)
{
    if (!bHasEverFixed) return UINT32_MAX;
    uint64_t u64Now = RTC_u64GetUTC();
    return (u64Now > u64LastFixUtc) ? (uint32_t)(u64Now - u64LastFixUtc) : 0U;
}

/* --------------------------------------------------------------------------
 * GPS_vDispatcherTask — owns session lifecycle and power control.
 *
 * Idles waiting for GPS_DISP_TRIGGER_BIT. Once triggered, arms a session
 * and waits for the RX task to report FIX_OK or FIX_TIMEOUT (via thread
 * flags). The wait polls every 5 s so it can re-check the power class:
 * a drop out of NORMAL aborts the session with NO_POWER.
 *
 * After a terminal event the dispatcher either powers the GPS off (auto-
 * shutdown) or transitions to FIX_HELD (caller must call GPS_vShutdown).
 * -------------------------------------------------------------------------- */
#define GPS_DISP_POLL_MS    5000U

static void GPS_vDispatcherTask(void *parameters)
{
    (void)parameters;

    for (;;)
    {
        /* ---- Idle: wait for a request ---- */
        osThreadFlagsWait(GPS_DISP_TRIGGER_BIT, osFlagsWaitAny, osWaitForever);

        osMutexAcquire(xGpsLock, osWaitForever);
        GPS_vSessionArm();
        osMutexRelease(xGpsLock);

        /* Drop any FIX_OK/FIX_TIMEOUT/SHUTDOWN bits left pending from a prior
         * session (e.g. a wall-clock timeout below racing a late NMEA-driven
         * notification) so the poll loop below can't return immediately on a
         * stale flag. */
        osThreadFlagsClear(GPS_DISP_FIX_OK_BIT | GPS_DISP_FIX_TIMEOUT_BIT | GPS_DISP_SHUTDOWN_BIT);

        /* ---- Acquiring: poll until terminal event or power loss ---- */
        gps_result_e eResult = GPS_RESULT_FIX_TIMEOUT;

        for (;;)
        {
            uint32_t flags = osThreadFlagsWait(GPS_DISP_FIX_OK_BIT
                                              | GPS_DISP_FIX_TIMEOUT_BIT
                                              | GPS_DISP_SHUTDOWN_BIT,
                                              osFlagsWaitAny,
                                              GPS_DISP_POLL_MS);

            if (!(flags & osFlagsError))
            {
                if (flags & GPS_DISP_FIX_OK_BIT)      eResult = GPS_RESULT_FIX_OK;
                else if (flags & GPS_DISP_FIX_TIMEOUT_BIT) eResult = GPS_RESULT_FIX_TIMEOUT;
                else if (flags & GPS_DISP_SHUTDOWN_BIT)
                {
                    /* GPS_vShutdown already powered things off; just fan out
                     * a TIMEOUT result so any blocked waiter is released. */
                    eResult = GPS_RESULT_FIX_TIMEOUT;
                }
                break;
            }

            /* 5 s poll tick — re-check power class for mid-session loss */
            if ((POWER_tGetState() & POWER_CLASS_NORMAL) == 0U)
            {
                DBG("gps: power class dropped mid-session — aborting\r\n");
                osMutexAcquire(xGpsLock, osWaitForever);
                if (eState != GPS_STATE_IDLE) GPS_vPowerOff();
                osMutexRelease(xGpsLock);
                eResult = GPS_RESULT_NO_POWER;
                break;
            }

            /* 5 s poll tick — wall-clock TTFF fallback.
             *
             * GNSS_vOnSolution() (the only place that normally raises
             * FIX_OK/FIX_TIMEOUT) is only invoked once a full RMC+GSV epoch
             * has been parsed (GPS_bOnRxByte). Under poor signal the module
             * can stop emitting one of those sentences (commonly GSV when 0
             * SVs are tracked) entirely — GnssSession.u16TtffTmr then never
             * advances and the TTFF check inside GNSS_vOnSolution() is never
             * reached, so FIX_TIMEOUT_BIT is never set and this loop would
             * otherwise spin forever (while POWER_CLASS_NORMAL holds).
             *
             * Enforce the TTFF deadline here too, purely from the RTC tick
             * count, independent of NMEA reception. GNSS_TTFF_NO_TIMEOUT
             * (FIX_HELD / bAutoShutdown==false sessions) is exempt — those
             * are allowed to run indefinitely until GPS_vShutdown(). */
            if (GnssSession.u16TtffTimeout != GNSS_TTFF_NO_TIMEOUT)
            {
                osMutexAcquire(xGpsLock, osWaitForever);
                if (!bCallerNotified)
                {
                    uint16_t u16SessionTmr =
                        (uint16_t)((uint32_t)RTC_u64GetTicks() - GnssSession.u32StartTime);

                    if (u16SessionTmr >= GnssSession.u16TtffTimeout)
                    {
                        bCallerNotified = true;
                        DBG_LOG("gps: FIX TIMEOUT (wall-clock) t=%us/%us, no NMEA epoch completed\r\n",
                            u16SessionTmr, GnssSession.u16TtffTimeout);
                    }
                }
                osMutexRelease(xGpsLock);

                if (bCallerNotified)
                {
                    eResult = GPS_RESULT_FIX_TIMEOUT;
                    break;
                }
            }
        }

        /* ---- Terminal: fan out result and decide power state ---- */
        osMutexAcquire(xGpsLock, osWaitForever);

        if (eResult == GPS_RESULT_NO_POWER)
        {
            osEventFlagsSet(xGpsResultFlags, GPS_RESULT_NO_POWER_BIT);
        }
        else if (eResult == GPS_RESULT_FIX_OK)
        {
            osEventFlagsSet(xGpsResultFlags, GPS_RESULT_FIX_OK_BIT);
            if (bAutoShutdown)
            {
                GPS_vPowerOff();
            }
            else
            {
                eState = GPS_STATE_FIX_HELD;
                DBG("gps: fix held (auto-shutdown disabled)\r\n");
            }
        }
        else  /* GPS_RESULT_FIX_TIMEOUT */
        {
            osEventFlagsSet(xGpsResultFlags, GPS_RESULT_FIX_TIMEOUT_BIT);
            if (bAutoShutdown && eState != GPS_STATE_IDLE)
                GPS_vPowerOff();
            else if (eState != GPS_STATE_IDLE)
                eState = GPS_STATE_FIX_HELD;
        }

        osMutexRelease(xGpsLock);
    }
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
        strncpy(acGnssStrBuf, (p+0), 2); acGnssStrBuf[2] = '\0'; pSol->Lat.Ddm.i16Degrees = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+2), 2); acGnssStrBuf[2] = '\0'; pSol->Lat.Ddm.u8Minutes  = atoi(acGnssStrBuf);
        memset(acGnssStrBuf, '0', 6); acGnssStrBuf[6] = '\0';
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
        strncpy(acGnssStrBuf, (p+0), 3); acGnssStrBuf[3] = '\0'; pSol->Long.Ddm.i16Degrees = atoi(acGnssStrBuf);
        strncpy(acGnssStrBuf, (p+3), 2); acGnssStrBuf[2] = '\0'; pSol->Long.Ddm.u8Minutes  = atoi(acGnssStrBuf);
        memset(acGnssStrBuf, '0', 6); acGnssStrBuf[6] = '\0';
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
