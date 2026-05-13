/*
 * platform.h
 *
 * Platform layer: 1 Hz heartbeat dispatcher and subscriber registry.
 * Tasks subscribe to receive a thread-flag notification every second,
 * driven by the RTC wakeup interrupt via CMSIS-RTOS v2.
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

#include "coroutine.h"
#include "platform_rtc.h"
#include "datetime.h"
#include "settings.h"
#include "time_driver.h"

/* --------------------------------------------------------------------------
 * Utility macros
 * -------------------------------------------------------------------------- */
#define Min(a, b)   (((a) < (b)) ? (a) : (b))
#define min(a, b)   Min(a, b)
#define Max(a, b)   (((a) > (b)) ? (a) : (b))
#define max(a, b)   Max(a, b)

#define MSB(u16)    (((uint8_t *)&u16)[1])
#define LSB(u16)    (((uint8_t *)&u16)[0])

#define convert_byte_array_to_16_bit(data)  (*(uint16_t *)(data))

/* --------------------------------------------------------------------------
 * Heartbeat subscriber flags
 * -------------------------------------------------------------------------- */
#define MAX_SUBSCRIBERS         12
#define HB_ALLOW_IN_RECOVERY    (1 << 0)

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */
void          PLATFORM_vInit(void);
void          PLATFORM_vHeartbeatDispatchTask(void *parameters);
osThreadId_t  PLATFORM_tGetHeartbeatDispatchTaskHandle(void);

bool PLATFORM_bSubscribeToHeartbeat(osThreadId_t task, uint32_t flags);
void PLATFORM_vEnableHeartbeat(osThreadId_t task);
void PLATFORM_vDisableHeartbeat(osThreadId_t task);

#endif /* PLATFORM_H_ */
