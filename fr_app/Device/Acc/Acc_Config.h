/*
 * ACC_Config.h
 *
 * Accelerometer device configuration — maps generic ACC aliases to the
 * LIS2DH register and SPI definitions.
 */

#ifndef DEVICE_ACC_ACC_CONFIG_H_
#define DEVICE_ACC_ACC_CONFIG_H_

#define ACC_WHO_AM_I_VALUE          LIS2DH_WHO_AM_I_VALUE

#define ACC_WHO_AM_I                LIS2DH_WHO_AM_I
#define ACC_CTRL_REG0               LIS2DH_CTRL_REG0
#define ACC_TEMP_CFG_REG            LIS2DH_TEMP_CFG_REG
#define ACC_CTRL_REG1               LIS2DH_CTRL_REG1
#define ACC_CTRL_REG2               LIS2DH_CTRL_REG2
#define ACC_CTRL_REG3               LIS2DH_CTRL_REG3
#define ACC_CTRL_REG4               LIS2DH_CTRL_REG4
#define ACC_CTRL_REG5               LIS2DH_CTRL_REG5
#define ACC_CTRL_REG6               LIS2DH_CTRL_REG6
#define ACC_REFERENCE               LIS2DH_REFERENCE
#define ACC_FIFO_CTRL_REG           LIS2DH_FIFO_CTRL_REG
#define ACC_FIFO_SRC_REG            LIS2DH_FIFO_SRC_REG
#define ACC_INT1_CFG                LIS2DH_INT1_CFG
#define ACC_INT1_THS                LIS2DH_INT1_THS
#define ACC_INT1_DUR                LIS2DH_INT1_DUR
#define ACC_INT2_CFG                LIS2DH_INT2_CFG
#define ACC_INT2_THS                LIS2DH_INT2_THS
#define ACC_INT2_DUR                LIS2DH_INT2_DUR
#define ACC_CLICK_CFG               LIS2DH_CLICK_CFG
#define ACC_CLICK_THS               LIS2DH_CLICK_THS
#define ACC_TIME_LIMIT              LIS2DH_TIME_LIMIT
#define ACC_TIME_LATENCY            LIS2DH_TIME_LATENCY
#define ACC_TIME_WINDOW             LIS2DH_TIME_WINDOW
#define ACC_ACT_THS                 LIS2DH_ACT_THS
#define ACC_ACT_DUR                 LIS2DH_ACT_DUR

#define ACC_SPI_ADDR_MASK           LIS2DH_SPI_ADDR_MASK

#define ACC_SPI                     SPI1
#define BSP_ACC_SPI_AF              GPIO_AF5_SPI1
#define ACC_SPI_CLK_ENABLE()        __HAL_RCC_SPI1_CLK_ENABLE()
#define ACC_SPI_CLK_DISABLE()       __HAL_RCC_SPI1_CLK_DISABLE()

#define ACC_PORT_CLK_ENABLE() \
{ \
    __HAL_RCC_GPIOA_CLK_ENABLE(); \
}

#endif /* DEVICE_ACC_ACC_CONFIG_H_ */
