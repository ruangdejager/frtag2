/*
 * LoraRadio_Driver.c
 *
 * Low-level LoRa radio driver — thin wrapper around the STM32WL SUBGHZ
 * peripheral and the SX126x radio driver library.
 *
 * Uses CMSIS-RTOS v2 (osDelay, osThreadFlagsWait) to stay consistent with
 * the rest of the application layer.  The ALL_FLAGS mask (0x7FFFFFFFU) is
 * used in bTransmitPayload so that any event that arrives while waiting for
 * TX_DONE is consumed rather than left pending in the notification word.
 */

#include "cmsis_os2.h"
#include "LoraRadio_Driver.h"
#include "radio_driver.h"
#include "dbg_log.h"

#include "FreeRTOS.h"
#include "task.h"

#define ALL_FLAGS   (0x7FFFFFFFU)

/* Radio chip mode values from the SX126x GetStatus response (bits 3:1) */
#define RADIO_CHIP_MODE_STDBY_RC    0x02
#define RADIO_CHIP_MODE_STDBY_XOSC  0x03
#define RADIO_CHIP_MODE_FS          0x04
#define RADIO_CHIP_MODE_RX          0x05
#define RADIO_CHIP_MODE_TX          0x06
#define RADIO_CHIP_MODE_CAD         0x07

static void LORARADIO_DRIVER_vRadioOnDioIrq(RadioIrqMasks_t radioIrq);

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_vInit
 * -------------------------------------------------------------------------- */
void LORARADIO_DRIVER_vInit(uint8_t *pUniqueID)
{
    PacketParams_t packetParams;

    HAL_vSUBGHZ_Init();

    SUBGRF_Init(LORARADIO_DRIVER_vRadioOnDioIrq);

    /* Enable SMPS clock detection before enabling SMPS (RM0453 §6.1) */
    SUBGRF_WriteRegister(SUBGHZ_SMPSC0R,
                         SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE);
    SUBGRF_SetRegulatorMode();

    SUBGRF_SetBufferBaseAddress(0x00, 0x00);
    SUBGRF_SetRfFrequency(RF_FREQUENCY);
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
    SUBGRF_SetStopRxTimerOnPreambleDetect(false);
    SUBGRF_SetPacketType(PACKET_TYPE_LORA);

    SUBGRF_WriteRegister(REG_LR_SYNCWORD,     (LORA_SYNCWORD >> 8) & 0xFF);
    SUBGRF_WriteRegister(REG_LR_SYNCWORD + 1,  LORA_SYNCWORD       & 0xFF);

    ModulationParams_t modulationParams;
    modulationParams.PacketType                         = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.Bandwidth              = LORA_BANDWIDTH;
    modulationParams.Params.LoRa.CodingRate             = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
    modulationParams.Params.LoRa.LowDatarateOptimize    = 0x00;
    modulationParams.Params.LoRa.SpreadingFactor        = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
    SUBGRF_SetModulationParams(&modulationParams);

    packetParams.PacketType                          = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.CrcMode                 = LORA_CRC_ON;
    packetParams.Params.LoRa.HeaderType              = LORA_PACKET_VARIABLE_LENGTH;
    packetParams.Params.LoRa.InvertIQ                = LORA_IQ_NORMAL;
    packetParams.Params.LoRa.PayloadLength           = 0xFF;
    packetParams.Params.LoRa.PreambleLength          = LORA_PREAMBLE_LENGTH;
    SUBGRF_SetPacketParams(&packetParams);

    /* Workaround: Optimising Inverted IQ (DS_SX1261-2_V1.2 §15.4) */
    SUBGRF_WriteRegister(0x0736, SUBGRF_ReadRegister(0x0736) | (1 << 2));

    /* Enable CRC + header error IRQs from init too, not just from
     * vEnterRxMode later. The SX126x's own LoRa hardware CRC check
     * (LORA_CRC_ON above) is our primary integrity guarantee against RF
     * corruption; without these IRQ bits, a corrupted packet is silently
     * dropped by the chip with zero indication to firmware, which makes
     * intermittent packet loss during OTA distribution look like radio
     * silence. Routed through LoraRadio.c's RX_DONE-mode-restart handling
     * so we can also log the event for correlation with OTA activity. */
    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RX_DONE
                                       | IRQ_CRC_ERROR    | IRQ_HEADER_ERROR,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RX_DONE | IRQ_CRC_ERROR    | IRQ_HEADER_ERROR,
                           IRQ_RADIO_NONE);

    if (pUniqueID != NULL)
        HAL_SUBGHZ_vSetUniqueId(pUniqueID);
}

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_bTransmitPayload
 * Returns true if the TX_DONE notification was received within 1 s.
 * -------------------------------------------------------------------------- */
bool LORARADIO_DRIVER_bTransmitPayload(uint8_t *payload, uint8_t payload_length)
{
    RadioPhyStatus_t tRadioStatus = SUBGRF_GetStatus();
    uint8_t u8ChipMode = tRadioStatus.Fields.ChipMode;

    if (u8ChipMode != RADIO_CHIP_MODE_STDBY_RC && u8ChipMode != RADIO_CHIP_MODE_STDBY_XOSC)
    {
        DBG_LOG("LoraRadio_Driver: Radio not in standby (mode: 0x%02X). Forcing STDBY_RC.\r\n",
            u8ChipMode);
        SUBGRF_SetStandby(STDBY_RC);
        osDelay(5);

        tRadioStatus = SUBGRF_GetStatus();
        u8ChipMode   = tRadioStatus.Fields.ChipMode;
        if (u8ChipMode != RADIO_CHIP_MODE_STDBY_RC && u8ChipMode != RADIO_CHIP_MODE_STDBY_XOSC)
        {
            DBG_LOG("LoraRadio_Driver: Failed to enter standby. TX aborted.\r\n");
            return false;
        }
    }

    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE);
    SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);

    /* Workaround 5.1 in DS.SX1261-2.W.APP */
    SUBGRF_WriteRegister(0x0889, SUBGRF_ReadRegister(0x0889) | 0x04);

    PacketParams_t packetParams = {
        .PacketType = PACKET_TYPE_LORA,
        .Params.LoRa = {
            .PreambleLength = LORA_PREAMBLE_LENGTH,
            .HeaderType     = LORA_PACKET_VARIABLE_LENGTH,
            .PayloadLength  = payload_length,
            .CrcMode        = LORA_CRC_ON,
            .InvertIQ       = LORA_IQ_NORMAL
        }
    };
    SUBGRF_SetPacketParams(&packetParams);
    SUBGRF_SendPayload(payload, payload_length, 0x00);

    /* Wait for TX_DONE — must be called from the radio task only (waits on
     * its own thread flags). Loop instead of a single wait: RADIO_EVT_
     * TX_PENDING is self-noise here — LORARADIO_bTxPacket() sets it
     * unconditionally on every enqueue, including the NEXT packet's, which
     * routinely happens while THIS packet's transmission is still in
     * flight during a burst (e.g. the OTA chunk blast, which queues up to
     * 8 packets back to back). A single ALL_FLAGS wait treats that as
     * "something happened, not TX_DONE" and reports a false failure before
     * the real TX_DONE IRQ — whose airtime hasn't even elapsed yet — ever
     * has a chance to arrive. Same hazard LORARADIO_bCarrierSense() already
     * strips for CAD (see its comment); it just wasn't applied here too.
     * Strip the self-noise and keep waiting on the remaining budget; only
     * report failure on a genuine timeout or an explicit RX_TX_TIMEOUT. */
    uint32_t u32Start = osKernelGetTickCount();
    bool bSuccess = false;
    for (;;)
    {
        uint32_t u32Elapsed = osKernelGetTickCount() - u32Start;
        if (u32Elapsed >= 1000U)
            break;   /* genuine timeout */

        uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, 1000U - u32Elapsed);
        if (r & osFlagsError)
            break;   /* genuine timeout */

        r &= ~RADIO_EVT_TX_PENDING;   /* self-noise, see comment above */

        if (r & RADIO_EVT_TX_DONE)
        {
            bSuccess = true;
            break;
        }
        if (r & RADIO_EVT_TIMEOUT)
            break;   /* genuine radio-reported failure */

        /* r == 0: only our own TX_PENDING fired — spurious wake, keep
         * waiting. Any other, genuinely foreign bit (e.g. RX_DONE from
         * ambient mesh traffic) is dropped here rather than stashed —
         * LoraRadio.c's stash is private to that file — same as this
         * function's prior behavior for anything that wasn't TX_DONE. */
    }

    if (bSuccess)
        DBG("LoraRadio: TX done\r\n");
    else
        DBG_LOG("LoraRadio: TX failed (timeout or error)\r\n");
    return bSuccess;
}

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_bReceivePayload
 * -------------------------------------------------------------------------- */
bool LORARADIO_DRIVER_bReceivePayload(LoraRadio_Packet_t *rxParams)
{
    PacketStatus_t packetStatus;

    /* Workaround 15.3 in DS.SX1261-2.W.APP */
    SUBGRF_WriteRegister(0x0920, 0x00);
    SUBGRF_WriteRegister(0x0944, SUBGRF_ReadRegister(0x0944) | 0x02);

    SUBGRF_GetPayload((uint8_t *)rxParams->buffer, &rxParams->length, 0xFF);
    SUBGRF_GetPacketStatus(&packetStatus);

    rxParams->rssi = packetStatus.Params.LoRa.RssiPkt;
    rxParams->snr  = packetStatus.Params.LoRa.SnrPkt;
    return true;
}

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_vEnterRxMode
 * -------------------------------------------------------------------------- */
void LORARADIO_DRIVER_vEnterRxMode(uint32_t u32RxTimeout)
{
    SUBGRF_SetStandby(STDBY_RC);
    osDelay(5);

    SUBGRF_SetDioIrqParams(
        IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
        IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
        IRQ_RADIO_NONE, IRQ_RADIO_NONE);

    SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);

    PacketParams_t packetParams = {
        .PacketType = PACKET_TYPE_LORA,
        .Params.LoRa = {
            .PreambleLength = LORA_PREAMBLE_LENGTH,
            .HeaderType     = LORA_PACKET_VARIABLE_LENGTH,
            .PayloadLength  = 0xFF,
            .CrcMode        = LORA_CRC_ON,
            .InvertIQ       = LORA_IQ_NORMAL
        }
    };
    SUBGRF_SetPacketParams(&packetParams);
    SUBGRF_SetRx(u32RxTimeout << 6);  /* ms → radio ticks (×64, 15.625 µs/tick) */
}

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_u32GetRandomNumber
 * -------------------------------------------------------------------------- */
uint32_t LORARADIO_DRIVER_u32GetRandomNumber(uint32_t max_value)
{
    if (max_value == 0) return 0;
    return SUBGRF_GetRandom() % (max_value + 1);
}

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_vEnterCAD
 * -------------------------------------------------------------------------- */
void LORARADIO_DRIVER_vEnterCAD(void)
{
    SUBGRF_SetStandby(STDBY_RC);
    osDelay(5);
    SUBGRF_ClearIrqStatus(IRQ_RADIO_ALL);

    SUBGRF_SetCadParams(LORA_CAD_04_SYMBOL, 22, 10, LORA_CAD_ONLY, 0xFF);
    SUBGRF_SetDioIrqParams(
        IRQ_CAD_CLEAR | IRQ_CAD_DETECTED,
        IRQ_CAD_CLEAR | IRQ_CAD_DETECTED,
        IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    SUBGRF_SetCad();
}

/* --------------------------------------------------------------------------
 * LORARADIO_DRIVER_vEnterDeepSleep / vWakeUp
 * -------------------------------------------------------------------------- */
void LORARADIO_DRIVER_vEnterDeepSleep(void)
{
    SleepParams_t sleepConfig;
    sleepConfig.Fields.WarmStart  = 0;  /* cold start: registers not retained   */
    sleepConfig.Fields.Reset      = 0;  /* no reset on wakeup                   */
    sleepConfig.Fields.WakeUpRTC  = 0;  /* wakeup by NSS pulse only             */
    SUBGRF_SetSleep(sleepConfig);
}

void LORARADIO_DRIVER_vWakeUp(void)
{
    LORARADIO_DRIVER_vInit(NULL);
    LORARADIO_DRIVER_vEnterRxMode(0x00);
}

/* --------------------------------------------------------------------------
 * Radio DIO IRQ dispatch — called from SUBGHZ_Radio_IRQHandler
 * -------------------------------------------------------------------------- */
static void LORARADIO_DRIVER_vRadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    if (radioIrq & IRQ_TX_DONE)        LORARADIO_vEventTxDone();
    if (radioIrq & IRQ_RX_DONE)        LORARADIO_vEventRxDone();
    if (radioIrq & IRQ_HEADER_ERROR)   LORARADIO_vEventHeaderError();
    if (radioIrq & IRQ_CRC_ERROR)      LORARADIO_vEventCrcError();
    if (radioIrq & IRQ_RX_TX_TIMEOUT)  LORARADIO_vEventTimeout();
    if (radioIrq & IRQ_CAD_CLEAR)      LORARADIO_vEventCADClear();
    if (radioIrq & IRQ_CAD_DETECTED)   LORARADIO_vEventCADDetected();
}
