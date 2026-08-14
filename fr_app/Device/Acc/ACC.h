/*
 * ACC.h
 *
 * Accelerometer (LIS2DH) public interface.
 */

#ifndef DEVICE_ACC_ACC_H_
#define DEVICE_ACC_ACC_H_

#include <stdbool.h>
#include "ACC_Config.h"
#include "ACC_Driver.h"
#include "hal_bsp.h"

#define ACC_u8ReadReg   _u8DeviceReadReg

typedef struct {
    uint8_t u8OutXL;
    uint8_t u8OutXH;
    uint8_t u8OutYL;
    uint8_t u8OutYH;
    uint8_t u8OutZL;
    uint8_t u8OutZH;
} acc_xyz_byte_t;

typedef struct {
    int16_t i16OutX;
    int16_t i16OutY;
    int16_t i16OutZ;
} acc_xyz_word_t;

typedef union {
    uint8_t        au8Data[6];
    acc_xyz_byte_t u8;
    acc_xyz_word_t i16;
} acc_t;

typedef struct _acc_reg_config_t {
    uint8_t u8Name;
    uint8_t u8Value;
} acc_reg_config_t;

void    ACC_vInit(void);
void    ACC_vConfigIdle(void);
uint8_t ACC_u8GetDeviceId(void);
bool    ACC_bDeviceIdOk(void);

/* WHO_AM_I check with retries. Hands back the last raw byte read and how many
 * attempts it took, so a self-test can log what actually came off the wire
 * instead of just "FAIL". pu8Id / pu8Attempts may be NULL.
 * ACC_bDeviceIdOk() is this with both discarded. */
bool    ACC_bDeviceIdOkEx(uint8_t *pu8Id, uint8_t *pu8Attempts);
uint8_t ACC_u8NumSamplesInFifo(void);
void    ACC_vGetAccSample(acc_t *pAcc);
void    ACC_vTestRegsConfigError(void);

uint8_t _u8DeviceReadReg(uint8_t u8RegAddr);

#endif /* DEVICE_ACC_ACC_H_ */
