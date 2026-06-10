/*
 * Movement.c
 *
 * Movement worker — sensor sampling, HPF, MOVING/STILL state, gravity
 * sequence detector.  Runs as an RTOS task subscribed to the 1 Hz
 * platform heartbeat.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hal_bsp.h"
#include "Movement.h"
#include "Movement_Driver.h"

#include "platform.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "dbg_log.h"
#include "math_func.h"
#include "Acc.h"
#include "DeviceDiscovery.h"

/* --------------------------------------------------------------------------
 * Internal types
 * -------------------------------------------------------------------------- */
typedef struct {
    int8_t  ai8DcBuf[MOVE_ALG_DC_BUF_SIZE];
    uint8_t u8DcBufIdx;
    int8_t  i8DcLevel;
} move_alg_axis_t;

typedef struct {
    move_alg_axis_t X;
    move_alg_axis_t Y;
    move_alg_axis_t Z;
    uint8_t au8EnvBuf[MOVE_ALG_ENV_BUF_SIZE];
    uint8_t u8EnvBufIdx;
    uint8_t u8EnvLevel;
} move_alg_t;

typedef struct {
    uint8_t  u8DevIdErrorSeqCnt;
    uint16_t u16SampleDeltaErrorCnt;
    uint8_t  u8NoSampleErrorCnt;
    acc_t    AccPrevSample;
    bool     bErrorFlag;
} acc_health_t;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */
static move_alg_t  MoveAlg;
static acc_health_t AccHealth;

static uint8_t  u8MoveTemp;
static int16_t  i16MoveTemp;

static MoveState_e eState        = MOVE_STATE_MOVING;
static uint32_t    u32StillTicks = 0U;

/* Gravity sequence detector */
static uint8_t u8SeqStep         = 0U;
static uint8_t u8HoldCounter     = 0U;
static uint8_t u8TransitionTicks = 0U;

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static bool MOVE_bUpdateMovementLevel(uint8_t *pu8Level);
static void MOVE_vEvalMovementLevel(void);
static void MOVE_vUpdateState(void);
static bool MOVE_bCheckPosition(uint8_t step);
static void MOVE_vSequenceTick(void);
static void MOVE_vTask(void *arg);

/* --------------------------------------------------------------------------
 * MOVE_vInit
 * -------------------------------------------------------------------------- */
void MOVE_vInit(void)
{
    MoveAlg.X.u8DcBufIdx = 0;
    MoveAlg.Y.u8DcBufIdx = 0;
    MoveAlg.Z.u8DcBufIdx = 0;
    MoveAlg.u8EnvBufIdx  = 0;

    memset(&AccHealth, 0, sizeof(acc_health_t));

    eState            = MOVE_STATE_MOVING;
    u32StillTicks     = 0U;
    u8SeqStep         = 0U;
    u8HoldCounter     = 0U;
    u8TransitionTicks = 0U;

    static const osThreadAttr_t attr = {
        .name       = "Movement",
        .stack_size = configMINIMAL_STACK_SIZE * 5U,
        .priority   = osPriorityLow,
    };
    osThreadNew(MOVE_vTask, NULL, &attr);
}

/* --------------------------------------------------------------------------
 * MOVE_eGetState
 * -------------------------------------------------------------------------- */
MoveState_e MOVE_eGetState(void) { return eState; }

/* --------------------------------------------------------------------------
 * MOVE_vTask — 1 Hz heartbeat-driven task
 * -------------------------------------------------------------------------- */
static void MOVE_vTask(void *arg)
{
    (void)arg;
    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
        osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
        MOVE_vEvalMovementLevel();
        MOVE_vUpdateState();
        MOVE_vSequenceTick();

        /* Sensor health check #1: device ID */
        if (AccHealth.u8DevIdErrorSeqCnt < MOVE_SENSOR_ID_ERR_CNT_ALERT)
        {
            if (MOVE_DRIVER_u8GetAccelDeviceId() != ACC_WHO_AM_I_VALUE)
            {
                AccHealth.u8DevIdErrorSeqCnt++;
                if (AccHealth.u8DevIdErrorSeqCnt == MOVE_SENSOR_ID_ERR_CNT_ALERT &&
                    !AccHealth.bErrorFlag)
                {
                    AccHealth.bErrorFlag = true;
                    DBG("+++ MOVE SENSOR ERROR (DEVICE_ID) +++");
                }
            }
            else
            {
                AccHealth.u8DevIdErrorSeqCnt = 0;
            }
        }

        /* Sensor health check #2: no sample */
        if (AccHealth.u8NoSampleErrorCnt < MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT)
        {
            AccHealth.u8NoSampleErrorCnt++;
            if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_RESET)
            {
                ACC_vInit();
                DBG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - reset +++");
            }
            if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT &&
                !AccHealth.bErrorFlag)
            {
                AccHealth.bErrorFlag = true;
                DBG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - alert +++");
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * MOVE_vUpdateState — MOVING / STILL bookkeeping
 * -------------------------------------------------------------------------- */
static void MOVE_vUpdateState(void)
{
    if (MoveAlg.u8EnvLevel > MOVE_STILL_ENV_THRESHOLD)
    {
        if (eState != MOVE_STATE_MOVING)
            DBG_LOG("Movement: state -> MOVING (env=%u)\r\n", MoveAlg.u8EnvLevel);

        eState        = MOVE_STATE_MOVING;
        u32StillTicks = 0U;
    }
    else
    {
        if (u32StillTicks < MOVE_STILL_TIMEOUT_S)
            u32StillTicks++;

        if (u32StillTicks >= MOVE_STILL_TIMEOUT_S && eState != MOVE_STATE_STILL)
        {
            eState = MOVE_STATE_STILL;
            DBG_LOG("Movement: state -> STILL (env=%u)\r\n", MoveAlg.u8EnvLevel);
        }
    }
}

/* --------------------------------------------------------------------------
 * MOVE_bCheckPosition — is the device in the given sequence position?
 * -------------------------------------------------------------------------- */
static bool MOVE_bCheckPosition(uint8_t step)
{
    int8_t x = MoveAlg.X.i8DcLevel;
    int8_t y = MoveAlg.Y.i8DcLevel;
    int8_t z = MoveAlg.Z.i8DcLevel;

    switch (step)
    {
        case 0: return (z < -MOVE_SEQ_GRAV_TH) &&
                       (x > -MOVE_SEQ_GRAV_NULL && x < MOVE_SEQ_GRAV_NULL) &&
                       (y > -MOVE_SEQ_GRAV_NULL && y < MOVE_SEQ_GRAV_NULL);
        case 1: return (z > +MOVE_SEQ_GRAV_TH);
        case 2: return ((x > MOVE_SEQ_GRAV_TH || x < -MOVE_SEQ_GRAV_TH)) &&
                       (z > -MOVE_SEQ_GRAV_NULL && z < MOVE_SEQ_GRAV_NULL);
        case 3: return (z > +MOVE_SEQ_GRAV_TH);
        case 4: return ((y > MOVE_SEQ_GRAV_TH || y < -MOVE_SEQ_GRAV_TH)) &&
                       (z > -MOVE_SEQ_GRAV_NULL && z < MOVE_SEQ_GRAV_NULL);
        case 5: return (z > +MOVE_SEQ_GRAV_TH);
        default: return false;
    }
}

/* --------------------------------------------------------------------------
 * MOVE_vSequenceTick — gravity orientation sequence detector
 * -------------------------------------------------------------------------- */
static void MOVE_vSequenceTick(void)
{
    if (MOVE_bCheckPosition(u8SeqStep))
    {
        u8HoldCounter++;
        u8TransitionTicks = 0U;

        if (u8HoldCounter >= MOVE_SEQ_HOLD_TICKS)
        {
            MOVE_DRIVER_vLedOn();
            osDelay(100);
            MOVE_DRIVER_vLedOff();

            u8SeqStep++;
            u8HoldCounter     = 0U;
            u8TransitionTicks = 0U;

            DBG_LOG("Movement: shake-sequence step %u/6 accepted\r\n", u8SeqStep);

            if (u8SeqStep >= 6U)
            {
                /* Sequence complete — flash 5 times then trigger kernel wakeup */
                DBG_LOG("Movement: shake-sequence COMPLETE -> kernel wakeup\r\n");

                for (uint8_t i = 0U; i < 20U; i++)
                {
                    MOVE_DRIVER_vLedOn();
                    osDelay(25);
                    MOVE_DRIVER_vLedOff();
                    osDelay(75);
                }
                u8SeqStep = 0U;
                DEVICE_DISCOVERY_vTriggerKernelWakeup();
            }
        }
    }
    else if (u8SeqStep != 0U)
    {
        /* Mid-sequence: not yet in the next required position. Allow a few
         * seconds to physically move into place before giving up. */
        u8HoldCounter = 0U;
        u8TransitionTicks++;

        if (u8TransitionTicks >= MOVE_SEQ_TRANSITION_TICKS)
        {
            DBG_LOG("Movement: shake-sequence reset (was step %u/6)\r\n", u8SeqStep);
            u8SeqStep         = 0U;
            u8TransitionTicks = 0U;
        }
    }
    else
    {
        u8HoldCounter     = 0U;
        u8TransitionTicks = 0U;
    }
}

/* --------------------------------------------------------------------------
 * MOVE_bUpdateMovementLevel — drain one sample from ACC FIFO, run HPF
 * -------------------------------------------------------------------------- */
static bool MOVE_bUpdateMovementLevel(uint8_t *pu8Level)
{
    uint8_t u8;
    acc_t   AccMoveDev;

    if (ACC_u8NumSamplesInFifo() > 0)
    {
        AccHealth.u8NoSampleErrorCnt = 0;

        ACC_vGetAccSample(&AccMoveDev);

        AccMoveDev.i16.i16OutX /= MOVE_ACC_RAW_SCALE_VALUE;
        AccMoveDev.i16.i16OutY /= MOVE_ACC_RAW_SCALE_VALUE;
        AccMoveDev.i16.i16OutZ /= MOVE_ACC_RAW_SCALE_VALUE;

        AccMoveDev.i16.i16OutX = min(AccMoveDev.i16.i16OutX,  127);
        AccMoveDev.i16.i16OutX = max(AccMoveDev.i16.i16OutX, -128);
        AccMoveDev.i16.i16OutY = min(AccMoveDev.i16.i16OutY,  127);
        AccMoveDev.i16.i16OutY = max(AccMoveDev.i16.i16OutY, -128);
        AccMoveDev.i16.i16OutZ = min(AccMoveDev.i16.i16OutZ,  127);
        AccMoveDev.i16.i16OutZ = max(AccMoveDev.i16.i16OutZ, -128);

        u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
        MoveAlg.X.ai8DcBuf[MoveAlg.X.u8DcBufIdx++] = ((u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0);
        MoveAlg.X.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
        MoveAlg.Y.ai8DcBuf[MoveAlg.Y.u8DcBufIdx++] = ((u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0);
        MoveAlg.Y.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
        MoveAlg.Z.ai8DcBuf[MoveAlg.Z.u8DcBufIdx++] = ((u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0);
        MoveAlg.Z.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.X.ai8DcBuf[u8MoveTemp];
        MoveAlg.X.i8DcLevel = (i16MoveTemp / MOVE_ALG_DC_BUF_SIZE);

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.Y.ai8DcBuf[u8MoveTemp];
        MoveAlg.Y.i8DcLevel = (i16MoveTemp / MOVE_ALG_DC_BUF_SIZE);

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.Z.ai8DcBuf[u8MoveTemp];
        MoveAlg.Z.i8DcLevel = (i16MoveTemp / MOVE_ALG_DC_BUF_SIZE);

        u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx]  = (uint8_t)abs((int8_t)((u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0) - MoveAlg.X.i8DcLevel);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t)abs((int8_t)((u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0) - MoveAlg.Y.i8DcLevel);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t)abs((int8_t)((u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0) - MoveAlg.Z.i8DcLevel);
        MoveAlg.u8EnvBufIdx = (MoveAlg.u8EnvBufIdx + 1) % MOVE_ALG_ENV_BUF_SIZE;

        i16MoveTemp = 0;
        for (u8MoveTemp = 0; u8MoveTemp < MOVE_ALG_ENV_BUF_SIZE; u8MoveTemp++)
            i16MoveTemp += MoveAlg.au8EnvBuf[u8MoveTemp];
        MoveAlg.u8EnvLevel = (uint8_t)(i16MoveTemp / MOVE_ALG_ENV_BUF_SIZE);

        *pu8Level = MoveAlg.u8EnvLevel;

        /* Health check #3: no delta */
        for (u8 = 0; u8 < sizeof(acc_t); u8++)
        {
            if (AccHealth.AccPrevSample.au8Data[u8] != AccMoveDev.au8Data[u8])
            {
                memcpy(&AccHealth.AccPrevSample, &AccMoveDev, sizeof(acc_t));
                AccHealth.u16SampleDeltaErrorCnt = 0;
                break;
            }
        }
        if (u8 == sizeof(acc_t) &&
            AccHealth.u16SampleDeltaErrorCnt < MOVE_SENSOR_DELTA_ERR_CNT_ALERT)
        {
            AccHealth.u16SampleDeltaErrorCnt++;
            if (AccHealth.u16SampleDeltaErrorCnt == MOVE_SENSOR_DELTA_ERR_CNT_RESET)
            {
                ACC_vInit();
                DBG("+++ MOVE SENSOR ERROR (NO_DELTA) - reset +++");
            }
            else if (AccHealth.u16SampleDeltaErrorCnt == MOVE_SENSOR_DELTA_ERR_CNT_ALERT &&
                     !AccHealth.bErrorFlag)
            {
                AccHealth.bErrorFlag = true;
                DBG("+++ MOVE SENSOR ERROR (NO_DELTA) - alert +++");
            }
        }

        return true;
    }
    return false;
}

/* --------------------------------------------------------------------------
 * MOVE_vEvalMovementLevel — drain full ACC FIFO
 * -------------------------------------------------------------------------- */
static void MOVE_vEvalMovementLevel(void)
{
    uint8_t u8Level;
    while (MOVE_bUpdateMovementLevel(&u8Level))
        ;
}

#ifdef SWITCH_MOVE_SENSOR_DATA
int8_t MOVE_i8SensorDataX(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutX; }
int8_t MOVE_i8SensorDataY(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutY; }
int8_t MOVE_i8SensorDataZ(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutZ; }
#endif
