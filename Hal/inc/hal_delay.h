/*
 * hal_delay.h
 *
 * Busy-wait delay utilities using SysTick and HAL tick.
 */

#ifndef INC_HAL_DELAY_H_
#define INC_HAL_DELAY_H_

#include <stdint.h>

void HAL_vDelayMs(uint32_t count);
void HAL_vDelayUs(uint32_t count);

#endif /* INC_HAL_DELAY_H_ */
