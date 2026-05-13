/*
 * tag_hal.h
 *
 * Top-level HAL include aggregator for the frtag board.
 */

#ifndef INC_TAG_HAL_H_
#define INC_TAG_HAL_H_

#include <stdint.h>

#include "stm32wlxx.h"
#include "hal_gpio.h"
#include "hal_crc.h"
#include "hal_delay.h"
#include "hal_rtc.h"
#include "hal_timer.h"
#include "hal_uart.h"
#include "hal_system.h"
#include "hal_wdt.h"

#define HAL_vInit()  { \
    HAL_Init(); \
    SystemClock_Config(); \
}

#define HAL_vResetMCU()  NVIC_SystemReset()

#endif /* INC_TAG_HAL_H_ */
