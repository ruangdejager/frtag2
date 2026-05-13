/*
 * ACC_Driver.h
 *
 * LIS2DH register map and SPI address definitions.
 */

#ifndef DEVICE_ACC_ACC_DRIVER_H_
#define DEVICE_ACC_ACC_DRIVER_H_

#include "hal_spi.h"

/* Device ID */
#define LIS2DH_WHO_AM_I_VALUE   0x33

/* LIS2DH register definitions (selected) */
#define LIS2DH_WHO_AM_I         0x0F    /* r   */
#define LIS2DH_CTRL_REG0        0x1E    /* rw  */
#define LIS2DH_TEMP_CFG_REG     0x1F    /* rw  */
#define LIS2DH_CTRL_REG1        0x20    /* rw  */
#define LIS2DH_CTRL_REG2        0x21    /* rw  */
#define LIS2DH_CTRL_REG3        0x22    /* rw  */
#define LIS2DH_CTRL_REG4        0x23    /* rw  */
#define LIS2DH_CTRL_REG5        0x24    /* rw  */
#define LIS2DH_CTRL_REG6        0x25    /* rw  */
#define LIS2DH_REFERENCE        0x26    /* rw  */
#define LIS2DH_FIFO_CTRL_REG    0x2E    /* rw  */
#define LIS2DH_FIFO_SRC_REG     0x2F    /* r   */
#define LIS2DH_INT1_CFG         0x30    /* rw  */
#define LIS2DH_INT1_THS         0x32    /* rw  */
#define LIS2DH_INT1_DUR         0x33    /* rw  */
#define LIS2DH_INT2_CFG         0x34    /* rw  */
#define LIS2DH_INT2_THS         0x36    /* rw  */
#define LIS2DH_INT2_DUR         0x37    /* rw  */
#define LIS2DH_CLICK_CFG        0x38    /* rw  */
#define LIS2DH_CLICK_THS        0x3A    /* rw  */
#define LIS2DH_TIME_LIMIT       0x3B    /* rw  */
#define LIS2DH_TIME_LATENCY     0x3C    /* rw  */
#define LIS2DH_TIME_WINDOW      0x3D    /* rw  */
#define LIS2DH_ACT_THS          0x3E    /* rw  */
#define LIS2DH_ACT_DUR          0x3F    /* rw  */

#define LIS2DH_SPI_ADDR_MASK    0x3F

#endif /* DEVICE_ACC_ACC_DRIVER_H_ */
