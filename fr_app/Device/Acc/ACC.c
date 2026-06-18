/*
 * ACC.c
 *
 * LIS2DH accelerometer driver.
 *
 * Initialises the device via a register configuration table and exposes
 * the public API for ID verification, FIFO sample count and data read.
 */

#include "ACC.h"
#include "platform.h"

/* SPI frame bit positions */
#define SPI_READ_NWRITE_BP      7
#define SPI_MULTI_NSINGLE_BP    6

/* Nominal register configuration (startup sequence per AN3308) */
static const acc_reg_config_t AccRegConfig_P[] =
{
    { ACC_CTRL_REG0,        0b00010000 },   /* default */
    { ACC_TEMP_CFG_REG,     0b00000000 },   /* default */
    { ACC_CTRL_REG1,        0b00110111 },   /* high-res, 25 Hz ODR, XYZ enabled */
    { ACC_CTRL_REG2,        0b00000000 },   /* default */
    { ACC_CTRL_REG3,        0b00000010 },   /* FIFO overrun on INT1 */
    { ACC_CTRL_REG4,        0b00000000 },   /* default */
    { ACC_CTRL_REG5,        0b01000000 },   /* FIFO enabled */
    { ACC_CTRL_REG6,        0b00000000 },   /* default */
    { ACC_REFERENCE,        0b00000000 },   /* default */
    { ACC_FIFO_CTRL_REG,    0b10000000 },   /* stream mode, event trigger INT1 */
    { ACC_INT1_CFG,         0b00000000 },   /* default */
    { ACC_INT1_THS,         0b00000000 },   /* default */
    { ACC_INT1_DUR,         0b00000000 },   /* default */
    { ACC_INT2_CFG,         0b00000000 },   /* default */
    { ACC_INT2_THS,         0b00000000 },   /* default */
    { ACC_INT2_DUR,         0b00000000 },   /* default */
    { ACC_CLICK_CFG,        0b00000000 },   /* default */
    { ACC_CLICK_THS,        0b00000000 },   /* default */
    { ACC_TIME_LIMIT,       0b00000000 },   /* default */
    { ACC_TIME_LATENCY,     0b00000000 },   /* default */
    { ACC_TIME_WINDOW,      0b00000000 },   /* default */
    { ACC_ACT_THS,          0b00000000 },   /* default */
    { ACC_ACT_DUR,          0b00000000 },   /* default */
    { ACC_CTRL_REG5,        0b01000000 },   /* repeat per AN3308 startup */
};

/*
 * Known-bad register configuration captured from a field-returned unit
 * (May 2022) where CTRL_REG5 had FIFO disabled — kept for test/recovery use.
 */
static const acc_reg_config_t AccRegConfigErr_P[] =
{
    { ACC_CTRL_REG0,        0x10 },
    { ACC_TEMP_CFG_REG,     0x00 },
    { ACC_CTRL_REG1,        0x37 },
    { ACC_CTRL_REG2,        0x00 },
    { ACC_CTRL_REG3,        0x02 },
    { ACC_CTRL_REG4,        0x00 },
    { ACC_CTRL_REG5,        0x00 },   /* ERROR — should be 0x40 (FIFO enabled) */
    { ACC_CTRL_REG6,        0x00 },
    { ACC_REFERENCE,        0x00 },
    { ACC_FIFO_CTRL_REG,    0x80 },
    { ACC_INT1_CFG,         0x00 },
    { ACC_INT1_THS,         0x00 },
    { ACC_INT1_DUR,         0x00 },
    { ACC_INT2_CFG,         0x00 },
    { ACC_INT2_THS,         0x00 },
    { ACC_INT2_DUR,         0x00 },
    { ACC_CLICK_CFG,        0x00 },
    { ACC_CLICK_THS,        0x00 },
    { ACC_TIME_LIMIT,       0x00 },
    { ACC_TIME_LATENCY,     0x00 },
    { ACC_TIME_WINDOW,      0x00 },
    { ACC_ACT_THS,          0x00 },
    { ACC_ACT_DUR,          0x00 },
    { ACC_CTRL_REG5,        0x00 },   /* ERROR (repeat) */
};

/* Private prototypes */
void    _vDeviceReadRegArray(uint8_t u8RegAddrStart, uint8_t *pau8Buf, uint8_t u8NumBytes);
void    _vDeviceWriteReg(uint8_t u8RegAddr, uint8_t u8RegData);
void    _vDeviceRegsConfig(const acc_reg_config_t *pRegConfig_P, uint8_t u8RegConfigSize);

/* --------------------------------------------------------------------------
 * ACC_vInit
 * -------------------------------------------------------------------------- */
void ACC_vInit(void)
{
    _vDeviceRegsConfig(AccRegConfig_P, sizeof(AccRegConfig_P) / sizeof(acc_reg_config_t));
}

/* --------------------------------------------------------------------------
 * ACC_vConfigIdle — low-power idle config for roles that don't track movement.
 *
 * The primary doesn't use movement. Three states were measured on its rail:
 *   - unconfigured power-on default  ~115 uA (undefined I/O)
 *   - configured active 25 Hz + FIFO  ~45 uA (FIFO fills and overruns, never
 *     drained because there's no movement task)
 *   - full power-down (CTRL_REG1=0)  WORSE (floating SPI inputs into the now-
 *     idle I/O crowbar)
 * So keep the I/O drivers active but remove the overrun source: low-power mode,
 * low ODR, and NO FIFO (bypass) with no INT routing. CTRL_REG4=0 selects 4-wire
 * SPI so the interface is in a defined state.
 * -------------------------------------------------------------------------- */
static const acc_reg_config_t AccRegConfigIdle_P[] =
{
    { ACC_CTRL_REG0,        0b00010000 },   /* default (SDO pull-up state)        */
    { ACC_CTRL_REG4,        0b00000000 },   /* 4-wire SPI                          */
    { ACC_CTRL_REG3,        0b00000000 },   /* no INT routing                      */
    { ACC_CTRL_REG5,        0b00000000 },   /* FIFO disabled                       */
    { ACC_FIFO_CTRL_REG,    0b00000000 },   /* bypass mode (nothing to overrun)    */
    { ACC_CTRL_REG1,        0b00101111 },   /* low-power mode, 10 Hz, XYZ on       */
};

void ACC_vConfigIdle(void)
{
    _vDeviceRegsConfig(AccRegConfigIdle_P,
                       sizeof(AccRegConfigIdle_P) / sizeof(acc_reg_config_t));
}

/* --------------------------------------------------------------------------
 * _vDeviceReadRegArray — read N sequential registers via SPI
 * -------------------------------------------------------------------------- */
void _vDeviceReadRegArray(uint8_t u8RegAddrStart, uint8_t *pau8Buf, uint8_t u8NumBytes)
{
    uint8_t u8Data;

    u8RegAddrStart &= ACC_SPI_ADDR_MASK;
    u8Data = (1 << SPI_READ_NWRITE_BP) | (1 << SPI_MULTI_NSINGLE_BP) | u8RegAddrStart;
    HAL_SPI_ACC_vSelect();
    HAL_SPI_ACC_vSpiWritePacket(&u8Data, 1);
    HAL_SPI_ACC_vSpiReadPacket(pau8Buf, u8NumBytes);
    HAL_SPI_ACC_vDeselect();
}

/* --------------------------------------------------------------------------
 * _u8DeviceReadReg — read one register via SPI
 * -------------------------------------------------------------------------- */
uint8_t _u8DeviceReadReg(uint8_t u8RegAddr)
{
    uint8_t u8Data;
    _vDeviceReadRegArray(u8RegAddr, &u8Data, 1);
    return u8Data;
}

/* --------------------------------------------------------------------------
 * _vDeviceWriteReg — write one register via SPI
 * -------------------------------------------------------------------------- */
void _vDeviceWriteReg(uint8_t u8RegAddr, uint8_t u8RegData)
{
    uint8_t u8Data;

    u8RegAddr &= ACC_SPI_ADDR_MASK;
    u8Data = (0 << SPI_READ_NWRITE_BP) | (0 << SPI_MULTI_NSINGLE_BP) | u8RegAddr;
    HAL_SPI_ACC_vSelect();
    HAL_SPI_ACC_vSpiWritePacket(&u8Data, 1);
    HAL_SPI_ACC_vSpiWritePacket(&u8RegData, 1);
    HAL_SPI_ACC_vDeselect();
}

/* --------------------------------------------------------------------------
 * _vDeviceRegsConfig — write an arbitrary register configuration table
 * -------------------------------------------------------------------------- */
void _vDeviceRegsConfig(const acc_reg_config_t *pRegConfig_P, uint8_t u8RegConfigSize)
{
    for (uint8_t i = 0; i < u8RegConfigSize; i++)
    {
        _vDeviceWriteReg(pRegConfig_P[i].u8Name, pRegConfig_P[i].u8Value);
    }
}

/* --------------------------------------------------------------------------
 * ACC_u8GetDeviceId
 * -------------------------------------------------------------------------- */
uint8_t ACC_u8GetDeviceId(void)
{
    return _u8DeviceReadReg(ACC_WHO_AM_I);
}

/* --------------------------------------------------------------------------
 * ACC_bDeviceIdOk — returns true when WHO_AM_I matches expected value
 * -------------------------------------------------------------------------- */
bool ACC_bDeviceIdOk(void)
{
    return (ACC_u8GetDeviceId() == ACC_WHO_AM_I_VALUE);
}

/* --------------------------------------------------------------------------
 * ACC_u8NumSamplesInFifo
 * -------------------------------------------------------------------------- */
uint8_t ACC_u8NumSamplesInFifo(void)
{
    return (_u8DeviceReadReg(ACC_FIFO_SRC_REG) & 0x1F);
}

/* --------------------------------------------------------------------------
 * ACC_vGetAccSample — read one XYZ sample (6 bytes) from the FIFO
 * -------------------------------------------------------------------------- */
void ACC_vGetAccSample(acc_t *pAcc)
{
    _vDeviceReadRegArray(0x28, (uint8_t *)pAcc, 6);
}

/* --------------------------------------------------------------------------
 * ACC_vTestRegsConfigError — load the known-bad config for recovery testing
 * -------------------------------------------------------------------------- */
void ACC_vTestRegsConfigError(void)
{
    _vDeviceRegsConfig(AccRegConfigErr_P, sizeof(AccRegConfigErr_P) / sizeof(acc_reg_config_t));
}
