/*
 * LoRaWan.c
 *
 * LoRaWAN uplink session for DISCOVERY_MODE_LORAWAN. See LoRaWan.h for the
 * contract; the notes here are about how the two radio drivers are made to
 * take turns.
 *
 * Radio hand-over
 * ---------------
 * The chip has one DIO-IRQ callback slot (radio_driver.c's SUBGRF_Init()).
 * The mesh driver takes it on every mesh wake (LORARADIO_vWakeUp ->
 * LORARADIO_DRIVER_vInit), LoRaMac's driver (radio.c) takes it on
 * Radio.Init(). So a session starts with Radio.Init(NULL) - "re-attach,
 * keep LoRaMac's events" (the NULL case is a small frtag2 change in radio.c)
 * - and ends by giving the mesh driver its callback back and deep-sleeping
 * the chip, which is the state the mesh left it in. A cold deep sleep keeps
 * no registers, so neither driver ever inherits the other's configuration;
 * each wake/attach configures the chip from scratch.
 *
 * Radio.Init() resets radio.c's idea of the sync word to "private", so the
 * public LoRaWAN sync word is re-applied explicitly after every attach.
 *
 * Where things run
 * ----------------
 *   radio IRQ    SUBGHZ ISR -> radio.c RadioOnDioIrq -> RADIO_IRQ_PROCESS()
 *                = LORAWAN_vRadioIrqFromIsr (radio_conf.h): latch + wake
 *                the session loop, which runs RadioIrqProcess() in task
 *                context. LoRaMac arms its RX-window timers from there, and
 *                those are FreeRTOS timers that must not be touched from an
 *                ISR.
 *   MAC timers   FreeRTOS timer task (Core/Src/timer_if.c).
 *   MAC process  the session loop below, in the AppTask, whenever
 *                LORAWAN_THREAD_FLAG is raised (MacProcessNotify) or on a
 *                short poll.
 *
 * The confirm/indication callbacks are only acted on while a request of
 * ours is outstanding: a session cut short by its cap can leave a radio.c
 * timeout timer armed, whose late event LoRaMac would otherwise report as
 * the completion of the NEXT session's request. The first thing each
 * session does is let LoRaMac swallow any such leftover with no request
 * open.
 */

#include "LoRaWan.h"

#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "secure-element.h"
#include "radio.h"
#include "timer.h"

#include "cmsis_os2.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_ll_system.h"

#include "build_config.h"
#include "dbg_log.h"
#include "LoraRadio.h"
#include "Battery.h"
#include "Movement.h"
#include "GPS.h"
#include "version_config.h"

#ifdef STORAGE_BACKEND_FLASH
#include "Flash.h"
#include "Flash_Config.h"
#include "Log.h"          /* LORAWAN_NVM_FLASH_ADDR */
#endif

#include <string.h>

/* ---- MAC configuration ---- */

/* First join at DR0 (SF12): the farms are rural and the gateways are other
 * people's, so the join uses the longest-reach rate. ADR then walks the data
 * rate up (and the airtime down) once TTN has heard enough uplinks. */
#define LORAWAN_JOIN_DATARATE          DR_0

/* How far off its nominal start an RX window may open. Wider than ST's 20 ms
 * default because the window is opened by the FreeRTOS timer task (priority
 * 2), which a busy higher-priority task can hold off for a few ms, and the
 * TX-done that starts the RX delay is itself processed from a task, not the
 * ISR. A wider window costs a few ms of RX per uplink. */
#define LORAWAN_MAX_RX_ERROR_MS        30U

/* Poll period of the session loop when nothing has signalled it. */
#define LORAWAN_POLL_MS                20U

typedef enum
{
    LW_RESULT_PENDING = 0,
    LW_RESULT_OK,
    LW_RESULT_FAILED,
} LoRaWanResult_e;

/* ---- State (kept small: every byte here is .bss on a full part) ---- */
static LoRaMacPrimitives_t tMacPrimitives;
static LoRaMacCallback_t   tMacCallbacks;

static void           (*pfnRadioIrqProcess)(void) = NULL;
static volatile bool    bRadioIrqPending = false;
static osThreadId_t     volatile xSessionTask = NULL;

static bool             bMacReady      = false;
static bool             bRequestOpen   = false;   /* one of ours is outstanding */
static volatile uint8_t eResult        = LW_RESULT_PENDING;
#ifdef STORAGE_BACKEND_FLASH
static uint16_t         u16NvmNextRec  = 0U;      /* next free record slot */
#endif

/* --------------------------------------------------------------------------
 * Radio IRQ deferral (radio_conf.h hooks)
 * -------------------------------------------------------------------------- */
void LORAWAN_vRegisterRadioIrqProcess(void (*pfnProcess)(void))
{
    pfnRadioIrqProcess = pfnProcess;
}

void LORAWAN_vRadioIrqFromIsr(void)
{
    bRadioIrqPending = true;
    osThreadId_t xTask = xSessionTask;
    if (xTask != NULL)
        osThreadFlagsSet(xTask, LORAWAN_THREAD_FLAG);
}

static void LORAWAN_vOnMacProcessNotify(void)
{
    osThreadId_t xTask = xSessionTask;
    if (xTask != NULL)
        osThreadFlagsSet(xTask, LORAWAN_THREAD_FLAG);
}

/* --------------------------------------------------------------------------
 * Identity
 * -------------------------------------------------------------------------- */

/* STM32WL UID64 as the DevEUI - same derivation as ST's reference apps:
 * the 64-bit UID is an IEEE EUI-64 in ST's OUI, unique per die. */
static void LORAWAN_vGetUniqueId(uint8_t *id)
{
    uint32_t val = LL_FLASH_GetUDN();
    if (val == 0xFFFFFFFFUL)   /* not expected on a production part */
    {
        uint32_t ID_1_3_val = HAL_GetUIDw0() + HAL_GetUIDw2();
        uint32_t ID_2_val   = HAL_GetUIDw1();

        id[7] = (uint8_t)(ID_1_3_val >> 24);
        id[6] = (uint8_t)(ID_1_3_val >> 16);
        id[5] = (uint8_t)(ID_1_3_val >> 8);
        id[4] = (uint8_t)(ID_1_3_val);
        id[3] = (uint8_t)(ID_2_val >> 24);
        id[2] = (uint8_t)(ID_2_val >> 16);
        id[1] = (uint8_t)(ID_2_val >> 8);
        id[0] = (uint8_t)(ID_2_val);
    }
    else
    {
        id[7] = (uint8_t)(val & 0xFFU);
        id[6] = (uint8_t)((val >> 8) & 0xFFU);
        id[5] = (uint8_t)((val >> 16) & 0xFFU);
        id[4] = (uint8_t)((val >> 24) & 0xFFU);
        val = LL_FLASH_GetDeviceID();
        id[3] = (uint8_t)(val & 0xFFU);
        val = LL_FLASH_GetSTCompanyID();
        id[2] = (uint8_t)(val & 0xFFU);
        id[1] = (uint8_t)((val >> 8) & 0xFFU);
        id[0] = (uint8_t)((val >> 16) & 0xFFU);
    }
}

/* Only consulted for ABP; OTAA gets its DevAddr from the JoinAccept. */
static void LORAWAN_vGetDevAddr(uint32_t *devAddr)
{
    *devAddr = LL_FLASH_GetUDN();
    if (*devAddr == 0xFFFFFFFFUL)
        *devAddr = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
}

void LORAWAN_vGetDevEui(uint8_t au8DevEui[8])
{
    LORAWAN_vGetUniqueId(au8DevEui);
}

/* 255 = "not able to measure", per the DevStatusAns definition. The battery
 * voltage itself is in every report payload. */
static uint8_t LORAWAN_u8GetBatteryLevel(void)
{
    return 255U;
}

/* --------------------------------------------------------------------------
 * DevNonce persistence
 * -------------------------------------------------------------------------- */
#ifdef STORAGE_BACKEND_FLASH
/* External NOR, one sector (LORAWAN_NVM_FLASH_ADDR). Append-only 8-byte
 * records - magic, nonce, ~nonce, 0x0000 - and the last valid one wins, so a
 * record is only ever programmed onto blank bytes. 512 fit before the sector
 * is erased and restarted, i.e. one erase per 512 join attempts. Records are
 * 8-byte aligned, so none straddles a 256-byte program page. */
#define LORAWAN_NVM_MAGIC              0x4C57U     /* "LW" */
#define LORAWAN_NVM_REC_BYTES          8U
#define LORAWAN_NVM_RECS_PER_SECTOR    (FLASH_SECTOR_SIZE_BYTES / LORAWAN_NVM_REC_BYTES)

static bool LORAWAN_bNvmRecValid(const uint8_t *r, uint16_t *pu16Nonce)
{
    uint16_t u16Magic = (uint16_t)((r[0] << 8) | r[1]);
    uint16_t u16Nonce = (uint16_t)((r[2] << 8) | r[3]);
    uint16_t u16Inv   = (uint16_t)((r[4] << 8) | r[5]);
    if (u16Magic != LORAWAN_NVM_MAGIC || (uint16_t)~u16Nonce != u16Inv ||
        r[6] != 0x00U || r[7] != 0x00U)
        return false;
    *pu16Nonce = u16Nonce;
    return true;
}

/* Scan the sector for the newest record. Also leaves u16NvmNextRec at the
 * first blank slot - or at LORAWAN_NVM_RECS_PER_SECTOR, which makes the next
 * write erase first, when the sector is full or holds anything that is not
 * ours (on a unit upgraded from v2.4.x it is the tail of the old text log). */
static uint16_t LORAWAN_u16NvmRestore(void)
{
    uint8_t  au8Buf[64];
    uint16_t u16Last  = 0U;
    uint16_t u16Nonce = 0U;

    u16NvmNextRec = LORAWAN_NVM_RECS_PER_SECTOR;

    for (uint16_t u16Rec = 0U; u16Rec < LORAWAN_NVM_RECS_PER_SECTOR; )
    {
        uint32_t u32Addr = LORAWAN_NVM_FLASH_ADDR + (uint32_t)u16Rec * LORAWAN_NVM_REC_BYTES;
        if (!FLASH_vRead(u32Addr, au8Buf, sizeof(au8Buf)))
            return u16Last;   /* unreadable: keep the erase-first default */

        for (uint16_t i = 0U; i < sizeof(au8Buf); i += LORAWAN_NVM_REC_BYTES, u16Rec++)
        {
            const uint8_t *r = &au8Buf[i];
            bool bBlank = true;
            for (uint8_t k = 0U; k < LORAWAN_NVM_REC_BYTES; k++)
                if (r[k] != 0xFFU) { bBlank = false; break; }

            if (bBlank)
            {
                u16NvmNextRec = u16Rec;
                return u16Last;
            }
            if (!LORAWAN_bNvmRecValid(r, &u16Nonce))
                return u16Last;   /* foreign data: erase before next write */
            u16Last = u16Nonce;
        }
    }
    return u16Last;   /* full: erase before next write */
}

static bool LORAWAN_bNvmStore(uint16_t u16Nonce)
{
    if (u16NvmNextRec >= LORAWAN_NVM_RECS_PER_SECTOR)
    {
        if (!FLASH_vSectorErase(LORAWAN_NVM_FLASH_ADDR))
            return false;
        u16NvmNextRec = 0U;
    }

    uint8_t r[LORAWAN_NVM_REC_BYTES] = {
        (uint8_t)(LORAWAN_NVM_MAGIC >> 8), (uint8_t)LORAWAN_NVM_MAGIC,
        (uint8_t)(u16Nonce >> 8),          (uint8_t)u16Nonce,
        (uint8_t)((uint16_t)~u16Nonce >> 8), (uint8_t)~u16Nonce,
        0x00U, 0x00U,
    };
    uint32_t u32Addr = LORAWAN_NVM_FLASH_ADDR + (uint32_t)u16NvmNextRec * LORAWAN_NVM_REC_BYTES;
    if (!FLASH_vPageWrite(u32Addr, r, sizeof(r)))
        return false;
    u16NvmNextRec++;
    return true;
}
#else
static uint16_t LORAWAN_u16NvmRestore(void)       { return 0U; }
static bool     LORAWAN_bNvmStore(uint16_t u16Nonce) { (void)u16Nonce; return false; }
#endif /* STORAGE_BACKEND_FLASH */

static LoRaMacNvmData_t *LORAWAN_ptNvm(void)
{
    MibRequestConfirm_t mib;
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    return mib.Param.Contexts;
}

/* --------------------------------------------------------------------------
 * MAC primitives
 * -------------------------------------------------------------------------- */
static void LORAWAN_vMcpsConfirm(McpsConfirm_t *ptConfirm)
{
    if (!bRequestOpen)
        return;
    bRequestOpen = false;

    DBG_LOG("lorawan: uplink fcnt=%lu status=%u dr=%u txpwr=%u toa=%lums\r\n",
            (unsigned long)ptConfirm->UpLinkCounter, (unsigned)ptConfirm->Status,
            (unsigned)ptConfirm->Datarate, (unsigned)ptConfirm->TxPower,
            (unsigned long)ptConfirm->TxTimeOnAir);

    /* Unconfirmed uplink: OK means it went out and both RX windows closed. */
    eResult = (ptConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            ? LW_RESULT_OK : LW_RESULT_FAILED;
}

static void LORAWAN_vMcpsIndication(McpsIndication_t *ptInd, LoRaMacRxStatus_t *ptRx)
{
    if (ptInd->Status != LORAMAC_EVENT_INFO_STATUS_OK)
        return;

    if (ptInd->RxData)
    {
        DBG_LOG("lorawan: downlink port=%u len=%u rssi=%d snr=%d fcntdown=%lu\r\n",
                (unsigned)ptInd->Port, (unsigned)ptInd->BufferSize,
                (int)ptRx->Rssi, (int)ptRx->Snr,
                (unsigned long)ptInd->DownLinkCounter);
        if (ptInd->Port != 0U)
            LORAWAN_vOnDownlink(ptInd->Port, ptInd->Buffer, ptInd->BufferSize);
    }
}

static void LORAWAN_vMlmeConfirm(MlmeConfirm_t *ptConfirm)
{
    if (ptConfirm->MlmeRequest != MLME_JOIN || !bRequestOpen)
        return;
    bRequestOpen = false;

    if (ptConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
    {
        MibRequestConfirm_t mib;
        mib.Type = MIB_DEV_ADDR;
        LoRaMacMibGetRequestConfirm(&mib);
        DBG_LOG("lorawan: JOINED devaddr=%08lX\r\n", (unsigned long)mib.Param.DevAddr);
        eResult = LW_RESULT_OK;
    }
    else
    {
        DBG_LOG("lorawan: join failed status=%u\r\n", (unsigned)ptConfirm->Status);
        eResult = LW_RESULT_FAILED;
    }
}

static void LORAWAN_vMlmeIndication(MlmeIndication_t *ptInd, LoRaMacRxStatus_t *ptRx)
{
    (void)ptRx;
    /* MLME_SCHEDULE_UPLINK (the network has MAC answers waiting) needs no
     * special handling: the next report, 15 min later, carries them. */
    (void)ptInd;
}

__attribute__((weak)) void LORAWAN_vOnDownlink(uint8_t u8Port, const uint8_t *pu8Data, uint8_t u8Len)
{
    (void)u8Port; (void)pu8Data; (void)u8Len;
}

/* --------------------------------------------------------------------------
 * One-time MAC bring-up (first session after boot, radio already awake)
 * -------------------------------------------------------------------------- */
static bool LORAWAN_bMacInit(void)
{
    MibRequestConfirm_t mib;

    if (UTIL_TIMER_Init() != UTIL_TIMER_OK)
    {
        DBG_LOG("lorawan: timer init FAILED\r\n");
        return false;
    }

    tMacPrimitives.MacMcpsConfirm    = LORAWAN_vMcpsConfirm;
    tMacPrimitives.MacMcpsIndication = LORAWAN_vMcpsIndication;
    tMacPrimitives.MacMlmeConfirm    = LORAWAN_vMlmeConfirm;
    tMacPrimitives.MacMlmeIndication = LORAWAN_vMlmeIndication;

    tMacCallbacks.GetBatteryLevel     = LORAWAN_u8GetBatteryLevel;
    tMacCallbacks.GetTemperatureLevel = NULL;
    tMacCallbacks.GetUniqueId         = LORAWAN_vGetUniqueId;
    tMacCallbacks.GetDevAddress       = LORAWAN_vGetDevAddr;
    tMacCallbacks.NvmDataChange       = NULL;
    tMacCallbacks.MacProcessNotify    = LORAWAN_vOnMacProcessNotify;

    /* Calls Radio.Init(&its own events), which attaches radio.c. */
    LoRaMacStatus_t eStatus = LoRaMacInitialization(&tMacPrimitives, &tMacCallbacks,
                                                    LORAMAC_REGION_EU868);
    if (eStatus != LORAMAC_STATUS_OK)
    {
        DBG_LOG("lorawan: MAC init FAILED status=%u\r\n", (unsigned)eStatus);
        return false;
    }

    if (SecureElementInitMcuID(LORAWAN_vGetUniqueId, LORAWAN_vGetDevAddr) != SECURE_ELEMENT_SUCCESS)
    {
        DBG_LOG("lorawan: SE id init FAILED\r\n");
        return false;
    }

    mib.Type = MIB_PUBLIC_NETWORK;
    mib.Param.EnablePublicNetwork = true;
    LoRaMacMibSetRequestConfirm(&mib);

    mib.Type = MIB_ADR;
    mib.Param.AdrEnable = true;
    LoRaMacMibSetRequestConfirm(&mib);

    mib.Type = MIB_SYSTEM_MAX_RX_ERROR;
    mib.Param.SystemMaxRxError = LORAWAN_MAX_RX_ERROR_MS;
    LoRaMacMibSetRequestConfirm(&mib);

    /* EU868 duty cycle is on by default (and must stay on); LoRaMac also
     * applies the join back-off from it, which is what keeps a tag with no
     * coverage from joining every 15 min forever. */

    /* Continue the DevNonce count from before this boot. */
    uint16_t u16Nonce = LORAWAN_u16NvmRestore();
    LORAWAN_ptNvm()->Crypto.DevNonce = u16Nonce;

    uint8_t au8Eui[8];
    LORAWAN_vGetUniqueId(au8Eui);
    DBG_LOG("lorawan: DevEUI=%02X%02X%02X%02X%02X%02X%02X%02X devNonce=%u\r\n",
            au8Eui[0], au8Eui[1], au8Eui[2], au8Eui[3],
            au8Eui[4], au8Eui[5], au8Eui[6], au8Eui[7], (unsigned)u16Nonce);

    bMacReady = true;
    return true;
}

bool LORAWAN_bIsJoined(void)
{
    if (!bMacReady)
        return false;
    MibRequestConfirm_t mib;
    mib.Type = MIB_NETWORK_ACTIVATION;
    if (LoRaMacMibGetRequestConfirm(&mib) != LORAMAC_STATUS_OK)
        return false;
    return mib.Param.NetworkActivation != ACTIVATION_TYPE_NONE;
}

/* --------------------------------------------------------------------------
 * Requests
 * -------------------------------------------------------------------------- */
static bool LORAWAN_bStartJoin(void)
{
    /* Persist the nonce this request will use BEFORE sending it: LoRaMac
     * increments DevNonce and then transmits, so storing (current + 1)
     * first means a reset at any point after this - mid-join included -
     * restarts above every nonce that has gone on air. */
    LoRaMacNvmData_t *ptNvm = LORAWAN_ptNvm();
    uint16_t u16Next = (uint16_t)(ptNvm->Crypto.DevNonce + 1U);
    if (!LORAWAN_bNvmStore(u16Next))
        DBG_LOG("lorawan: WARNING devNonce %u not persisted - a reboot may reuse it\r\n",
                (unsigned)u16Next);

    MlmeReq_t tReq;
    memset(&tReq, 0, sizeof(tReq));
    tReq.Type = MLME_JOIN;
    tReq.Req.Join.NetworkActivation = ACTIVATION_TYPE_OTAA;
    tReq.Req.Join.Datarate          = LORAWAN_JOIN_DATARATE;

    bRequestOpen = true;
    LoRaMacStatus_t eStatus = LoRaMacMlmeRequest(&tReq);
    if (eStatus != LORAMAC_STATUS_OK)
    {
        bRequestOpen = false;
        DBG_LOG("lorawan: join not sent status=%u wait=%lums\r\n",
                (unsigned)eStatus, (unsigned long)tReq.ReqReturn.DutyCycleWaitTime);
        return false;
    }
    DBG_LOG("lorawan: JoinRequest devNonce=%u dr=%u\r\n",
            (unsigned)u16Next, (unsigned)LORAWAN_JOIN_DATARATE);
    return true;
}

static uint8_t LORAWAN_u8BuildReport(uint8_t *p)
{
    uint32_t u32Id  = LORARADIO_u32GetUniqueId();
    uint16_t u16Bat = BAT_u16GetVoltage();
    uint8_t  u8Flags = (MOVE_eGetState() == MOVE_STATE_STILL) ? LORAWAN_REPORT_FLAG_STILL : 0U;

    /* Last known fix with its age, however old - same policy as the
     * basic-mode beacon this replaces: the server judges freshness. */
    gnss_coord_deg_t tLat, tLon;
    uint32_t u32AgeS = UINT32_MAX;
    bool bGps = GPS_bGetLastKnownFix(&tLat, &tLon, &u32AgeS);
    if (bGps)
        u8Flags |= LORAWAN_REPORT_FLAG_GPS_VALID;

    p[0] = (uint8_t)(u32Id >> 24);  p[1] = (uint8_t)(u32Id >> 16);
    p[2] = (uint8_t)(u32Id >> 8);   p[3] = (uint8_t)u32Id;
    p[4] = (uint8_t)(u16Bat >> 8);  p[5] = (uint8_t)u16Bat;
    p[6] = u8Flags;
    p[7] = (uint8_t)VERSION_SW_PATCH;

    if (!bGps)
        return LORAWAN_REPORT_LEN_BASE;

    uint32_t u32Lat = (uint32_t)tLat.i32MicroDeg;
    uint32_t u32Lon = (uint32_t)tLon.i32MicroDeg;
    p[8]  = (uint8_t)(u32Lat >> 24);  p[9]  = (uint8_t)(u32Lat >> 16);
    p[10] = (uint8_t)(u32Lat >> 8);   p[11] = (uint8_t)u32Lat;
    p[12] = (uint8_t)(u32Lon >> 24);  p[13] = (uint8_t)(u32Lon >> 16);
    p[14] = (uint8_t)(u32Lon >> 8);   p[15] = (uint8_t)u32Lon;
    p[16] = (uint8_t)(u32AgeS >> 24); p[17] = (uint8_t)(u32AgeS >> 16);
    p[18] = (uint8_t)(u32AgeS >> 8);  p[19] = (uint8_t)u32AgeS;
    return LORAWAN_REPORT_LEN_GPS;
}

static bool LORAWAN_bStartReport(void)
{
    /* Static: LoRaMac keeps a pointer to the buffer until the frame is
     * serialised, which is after this function returns. */
    static uint8_t au8Payload[LORAWAN_REPORT_LEN_GPS];
    uint8_t u8Len = LORAWAN_u8BuildReport(au8Payload);

    McpsReq_t tReq;
    memset(&tReq, 0, sizeof(tReq));

    LoRaMacTxInfo_t tTxInfo;
    if (LoRaMacQueryTxPossible(u8Len, &tTxInfo) != LORAMAC_STATUS_OK)
    {
        /* Pending MAC answers leave no room at this data rate: send them on
         * an empty frame now, the report goes next time. */
        tReq.Type = MCPS_UNCONFIRMED;
        tReq.Req.Unconfirmed.fBuffer     = NULL;
        tReq.Req.Unconfirmed.fBufferSize = 0U;
        DBG_LOG("lorawan: report does not fit (max %u) - flushing MAC commands\r\n",
                (unsigned)tTxInfo.MaxPossibleApplicationDataSize);
    }
    else
    {
        tReq.Type = MCPS_UNCONFIRMED;
        tReq.Req.Unconfirmed.fPort       = LORAWAN_REPORT_FPORT;
        tReq.Req.Unconfirmed.fBuffer     = au8Payload;
        tReq.Req.Unconfirmed.fBufferSize = u8Len;
    }
    tReq.Req.Unconfirmed.Datarate = LORAWAN_JOIN_DATARATE;   /* ignored while ADR is on */

    bRequestOpen = true;
    LoRaMacStatus_t eStatus = LoRaMacMcpsRequest(&tReq, false);
    if (eStatus != LORAMAC_STATUS_OK)
    {
        bRequestOpen = false;
        DBG_LOG("lorawan: report not sent status=%u wait=%lums\r\n",
                (unsigned)eStatus, (unsigned long)tReq.ReqReturn.DutyCycleWaitTime);
        return false;
    }
    DBG_LOG("lorawan: report queued len=%u gps=%u\r\n", (unsigned)u8Len,
            (unsigned)((au8Payload[6] & LORAWAN_REPORT_FLAG_GPS_VALID) != 0U));
    return true;
}

/* --------------------------------------------------------------------------
 * Session loop
 * -------------------------------------------------------------------------- */
static void LORAWAN_vProcess(void)
{
    if (bRadioIrqPending)
    {
        bRadioIrqPending = false;
        if (pfnRadioIrqProcess != NULL)
            pfnRadioIrqProcess();
    }
    LoRaMacProcess();
}

/* Run the MAC until the open request completes or the budget runs out. */
static bool LORAWAN_bRunUntilDone(uint32_t u32StartTick, uint32_t u32BudgetMs)
{
    for (;;)
    {
        LORAWAN_vProcess();

        if (eResult != LW_RESULT_PENDING)
            return eResult == LW_RESULT_OK;

        if ((osKernelGetTickCount() - u32StartTick) >= u32BudgetMs)
        {
            DBG_LOG("lorawan: session cap (%lu ms) hit - abandoning request\r\n",
                    (unsigned long)u32BudgetMs);
            bRequestOpen = false;
            return false;
        }

        (void)osThreadFlagsWait(LORAWAN_THREAD_FLAG, osFlagsWaitAny, LORAWAN_POLL_MS);
    }
}

bool LORAWAN_bSendReport(uint32_t u32BudgetMs)
{
    uint32_t u32Start = osKernelGetTickCount();
    bool     bOk      = false;

    if (u32BudgetMs > LORAWAN_SESSION_MAX_MS)
        u32BudgetMs = LORAWAN_SESSION_MAX_MS;

    /* Any IRQ latched since the last session belongs to that session. */
    bRadioIrqPending = false;
    (void)osThreadFlagsClear(LORAWAN_THREAD_FLAG);
    xSessionTask = osThreadGetId();

    if (!bMacReady)
    {
        if (!LORAWAN_bMacInit())
            goto done;
    }
    else
    {
        Radio.Init(NULL);   /* re-attach radio.c, keep LoRaMac's events */
    }
    Radio.SetPublicNetwork(true);

    /* Drain anything a previous, capped session left behind (see the file
     * header) before a request of ours is open to be mistaken for it. */
    bRequestOpen = false;
    LoRaMacProcess();

    LoRaMacStart();
    eResult = LW_RESULT_PENDING;

    if (!LORAWAN_bIsJoined())
    {
        /* A join session carries no report: EU868 duty cycle would hold the
         * data frame off for longer than the session is allowed to run. The
         * first report goes one period later. */
        if (LORAWAN_bStartJoin())
            bOk = LORAWAN_bRunUntilDone(u32Start, u32BudgetMs);
    }
    else
    {
        if (LORAWAN_bStartReport())
            bOk = LORAWAN_bRunUntilDone(u32Start, u32BudgetMs);
    }

done:
    if (bMacReady)
        (void)LoRaMacHalt();   /* stops the MAC timers, radio to sleep */

    xSessionTask = NULL;

    /* Back to the mesh driver: its DIO callback, its configuration, then a
     * cold deep sleep - the state a mesh wake expects to find. */
    LORARADIO_vWakeUp();
    LORARADIO_vEnterDeepSleep();

    DBG_LOG("lorawan: session end ok=%u joined=%u %lu ms\r\n",
            (unsigned)bOk, (unsigned)LORAWAN_bIsJoined(),
            (unsigned long)(osKernelGetTickCount() - u32Start));
    return bOk;
}
