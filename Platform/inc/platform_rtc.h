/*
 * platform_rtc.h
 *
 * Platform-level RTC abstraction: UTC offset management on top of the
 * raw HAL tick counter.
 */

#ifndef PLATFORM_RTC_H_
#define PLATFORM_RTC_H_

#include <stdbool.h>
#include <stdint.h>

uint64_t RTC_u64GetUTC(void);
uint64_t RTC_u64GetTicks(void);
void     RTC_vSetUTC(uint64_t time);
void     RTC_vSetUTCToSync(uint64_t time);
bool     RTC_bSyncNow(void);
bool     RTC_bIsRtcValid(void);

#endif /* PLATFORM_RTC_H_ */
