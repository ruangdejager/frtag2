/*
 * platform_rtc.c
 *
 * UTC offset management on top of the raw HAL RTC tick counter.
 *
 * The HAL tick is a monotonic counter that starts at 0 on reset.
 * An application-supplied UTC base time is stored as an offset and added
 * on every read, converting the monotonic tick to wall-clock UTC.
 */

#include "platform_rtc.h"
#include "hal_rtc.h"

static uint64_t offset      = 0;
static bool     isValid     = false;
static bool     has_new_value = false;
static uint64_t new_offset  = 0;

uint64_t RTC_u64GetUTC(void)
{
    return HAL_RTC_u64GetValue() + offset;
}

uint64_t RTC_u64GetTicks(void)
{
    return HAL_RTC_u64GetValue();
}

void RTC_vSetUTC(uint64_t time)
{
    RTC_vSetUTCToSync(time);
    RTC_bSyncNow();
}

void RTC_vSetUTCToSync(uint64_t time)
{
    has_new_value = true;
    new_offset    = time - RTC_u64GetTicks();
}

bool RTC_bSyncNow(void)
{
    if (has_new_value)
    {
        offset        = new_offset;
        isValid       = true;
        has_new_value = false;
        return true;
    }
    return false;
}

bool RTC_bIsRtcValid(void)
{
    return isValid;
}
