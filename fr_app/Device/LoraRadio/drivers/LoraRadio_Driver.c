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

    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RX_DONE,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RX_DONE,
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
     * its own thread flags).  ALL_FLAGS clears every pending bit so no stale
     * event is left in the notification word after this returns. */
    uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, 1000);
    bool bSuccess = (!(r & osFlagsError) && (r & RADIO_EVT_TX_DONE));
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
