/*
 * hal_subghz.h
 *
 * Sub-GHz radio SPI peripheral driver.
 */

#ifndef INC_HAL_SUBGHZ_H_
#define INC_HAL_SUBGHZ_H_

#include "stm32wlxx_hal.h"

extern SUBGHZ_HandleTypeDef hsubghz;

void HAL_vSUBGHZ_Init(void);
void HAL_SUBGHZ_vSetUniqueId(uint8_t *id);

#endif /* INC_HAL_SUBGHZ_H_ */
