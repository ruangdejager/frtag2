/*
 * Farmranger.h
 *
 * Farmranger device layer — UART-based AT command interface to the
 * companion logger board.
 *
 * Uses CMSIS-RTOS v2 throughout.  The data-send path streams all rows into
 * the interrupt-driven TX ring (which copies each byte) and then polls
 * HAL_UART_bTxIdle() once for the line to drain before tearing down the UART.
 */

#ifndef DEVICE_FARMRANGER_FARMRANGER_H_
#define DEVICE_FARMRANGER_FARMRANGER_H_

#include <stdbool.h>
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_bsp.h"
#include "MeshNetwork.h"

/* ---- Driver macros ---- */
#define FR_DRIVER_vInitFRDevice(drv)        HAL_UART_vSetup(drv, GPS_UART, FLOWCONTROL_NONE)
#define FR_DRIVER_vEnableUart(drv)          HAL_UART_vEnable(drv)
#define FR_DRIVER_vDisableUart(drv)         HAL_UART_vDisable(drv)
#define FR_DRIVER_vUartPutByte(drv, byte)   HAL_UART_vTxPutByte(drv, byte)
#define FR_DRIVER_vIntEnable()              HAL_GPIO_WritePin(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_SET)
#define FR_DRIVER_vIntDisable()             HAL_GPIO_WritePin(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_RESET)

/* ---- Public API ---- */
void     FARMRANGER_vInit(void);
void     FARMRANGER_vUartOnWake(void);
void     FARMRANGER_vRxTask(void *parameters);
void     FARMRANGER_vNotifyOnRX(void);         /* Called from UART ISR — do not call directly */
bool     FARMRANGER_bDeviceOn(void);
void     FARMRANGER_vDeviceOff(void);
uint64_t FARMRANGER_u64RequestTimestamp(void);
/* Fetch the primary's per-system settings from the logger in one AT round
 * trip: wake interval (min), discovery mode (false=advanced, true=basic),
 * GPS enable (false=disabled/battery-save, true=normal). Returns true on
 * a fully-parsed response; on false the caller keeps its previous
 * defaults. Wire format is documented in FARMRANGER_bRequestSettings's
 * definition and the fr9-side FRTAG_vSetReqHandler. */
bool     FARMRANGER_bRequestSettings(uint8_t *pu8Interval,
                                     bool    *pbBasicMode,
                                     bool    *pbGpsEnabled);
bool     FARMRANGER_bLogData(MeshDiscoveredNeighbor_t *neighbors, uint16_t count);

/* Basic-mode variant: uploads the primary's accumulated MeshBasicNeighbor_t
 * RAM store to fr9 (same AT+LOG framing, different per-row column set —
 * see FARMRANGER_iFormatBasicRow). Called at each WakeupInterval boundary
 * in basic mode. fr9 side treats the payload as opaque bytes so no
 * parser change is needed there. */
bool     FARMRANGER_bLogBasicData(MeshBasicNeighbor_t *neighbors, uint16_t count);

/* ---- Firmware-file pull (OTA acquire, see Worker/OtaUpdate) ----
 * Protocol (tag is master): AT+FWREQ queries what the logger holds, answered
 * with "FW,<verMMmmpp>,<fileBytes>,<xor8hex>" — the xor8 is the logger's own
 * manifest-verified whole-image XOR-8 (see tag_fota.h on the fr9 side),
 * independent of the per-block trailer below; the tag compares its own
 * final stored-image XOR-8 against it once the whole file has arrived, one
 * more link in the chain (build -> fr9 download -> tag storage -> bootloader
 * flash) that catches corruption the per-block check alone would miss.
 * AT+FWGET=<offset>,<len> pulls one block — the logger answers with exactly
 * <len> raw file bytes followed by a "FB,<offset>,<xor8hex>" trailer line;
 * AT+FWDONE=OK/ERR reports the outcome. Raw block bytes are captured into a
 * caller buffer via the block-capture hook in the RX task (bypassing the
 * 48-byte line path). */

/* FW query outcome. */
typedef enum {
    FARMRANGER_FW_NONE = 0,     /* logger holds no tag firmware            */
    FARMRANGER_FW_WAIT,         /* logger is warming its modem — poll again */
    FARMRANGER_FW_AVAILABLE     /* version + file size returned            */
} FarmrangerFw_e;

/* Ask the logger to check its GitHub Pages OTA host for a newer image
 * (AT+FWCHECK=<currentVerMMmmpp>,<hasStaged>). Returns true once fr9 acks
 * the request (+FWCHECK: OK) — the actual manifest+download work then runs
 * on fr9's own side, asynchronously; poll FARMRANGER_eFwQuery() afterward
 * the same as always (it answers FW,WAIT while fr9's check is in flight).
 * Returns false on no response, or if fr9 reports +FWCHECK: BUSY (a check
 * from a previous wake is still running) — the caller can still poll FWREQ
 * regardless.
 *
 * bHasStaged tells fr9 whether we already hold a valid staged copy of our
 * own running version or newer in ext-flash scratch. When false, fr9 will
 * also offer the CURRENT version's binary (not just strictly newer), so we
 * can refill an empty/erased scratchpad after a PRE-SEND xor mismatch —
 * without this the primary would be running vN with no staged copy of vN
 * and no way to distribute it to secondaries. */
bool FARMRANGER_bFwCheckRequest(uint32_t u32CurrentVer, bool bHasStaged);

FarmrangerFw_e FARMRANGER_eFwQuery(uint32_t *pu32Version, uint32_t *pu32FileBytes,
                                   uint8_t *pu8ExpectedXor);

/* Pull one raw file block into pu8Buf. Returns true when exactly u16Len
 * bytes arrived AND the trailer's offset and XOR-8 match the captured data. */
bool FARMRANGER_bFwGetBlock(uint32_t u32Offset, uint16_t u16Len, uint8_t *pu8Buf);

/* Report the transfer outcome to the logger. */
bool FARMRANGER_bFwReportDone(bool bOk);

/* --------------------------------------------------------------------------
 * R&D radio-test upload (AT+RTLOG)
 *
 * Used only by the runtime radio link/range test (see
 * fr_app/Tasks/RadioTest/RadioTestMode.h). Deliberately a separate AT command
 * from AT+LOG rather than an extra field on it: the discovery upload is the
 * primary's core job and must keep behaving byte-for-byte as it does today,
 * on both ends of the wire.
 * -------------------------------------------------------------------------- */

/* One measured beacon, as handed to the logger. */
typedef struct {
    uint32_t u32DeviceId;    /* beaconing secondary's LoRa unique ID        */
    uint32_t u32Seq;         /* beacon sequence number                      */
    uint32_t u32Missed;      /* running total missed, from sequence gaps    */
    int16_t  i16Rssi;        /* dBm                                         */
    int16_t  i16NoiseFloor;  /* dBm, or LORA_NOISE_FLOOR_INVALID            */
    int8_t   i8Snr;          /* dB                                          */
} FarmrangerRtBeacon_t;

/* Upload measured beacons to the fr9 for logging to its flash — in practice
 * one at a time, as each beacon is heard. Same handshake/pacing/verdict
 * contract as FARMRANGER_bLogData against the fr9's AT+RTLOG handler, but a
 * SINGLE attempt: the caller is expected to be keeping up with a live 5 s
 * beacon stream, so recovery is its decision, not ours (see the note on the
 * implementation). The caller owns the connect/disconnect
 * (FARMRANGER_bDeviceOn / FARMRANGER_vDeviceOff), exactly as for AT+LOG; the
 * radio test opens and drops a session per beacon. */
bool FARMRANGER_bLogRadioTestData(const FarmrangerRtBeacon_t *pBeacons,
                                  uint16_t u16Count);

#endif /* DEVICE_FARMRANGER_FARMRANGER_H_ */
