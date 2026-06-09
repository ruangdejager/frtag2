/*
 * init.c
 *
 * Boot-sequence task.
 * Runs once immediately after the scheduler starts, initialises all
 * hardware and software subsystems in the correct order, then exits.
 *
 * Order matters:
 *   1. GPIO (board direction, charger enable)
 *   2. Device-role detection (primary vs secondary)
 *   3. UART (required by DBG macros)
 *   4. DbgLog, ext-flash, text log
 *   5. ADC, TIM2 tick counter
 *   6. Platform (creates heartbeat task — must come before HAL_RTC_vInit)
 *   7. Settings
 *   8. RTC  (starts 1 Hz wakeup interrupt — heartbeat task must exist)
 *   9. LoRa radio stack
 *  10. Role-specific subsystems
 *  11. Time, battery, power management
 */

#include "init.h"
#include "hal_rtc.h"
#include "hal_timer.h"
#include "hal_uart.h"
#include "hal_spi.h"
#include "platform.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_wdt.h"
#include "settings.h"
#include "main.h"

#include "Gps.h"
#include "Acc.h"

#include "Movement.h"
#include "LoraRadio.h"
#include "MeshNetwork.h"
#include "DeviceDiscovery.h"
#include "Farmranger.h"
#ifdef ENABLE_RADIO_TEST
#  include "RadioTest.h"
#endif
#include "Battery.h"
#include "SolarPower.h"
#include "Power.h"

#include "flashLog.h"
#include "Flash.h"
#include "Log.h"
#include "DbgLog.h"

#include "cmsis_os2.h"

volatile bool systemReadyForSleep = false;

void INIT_vInitialization(void *parameters)
{
    (void)parameters;

    FLASHLOG_vInit();

    /* Board-level GPIO */
    HAL_GPIO_vInit();

    /* Determine primary / secondary role before UART init */
    DEVICE_DISCOVERY_vConfigDeviceRole();

    HAL_UART_vInit();

#ifdef LISTENER_MODE
    FARMRANGER_vInit();   /* must be first — DBG_LOG() routes through this */
#endif
    DBGLOG_vInit();

    FLASH_vInit();
    LOG_vInit();

    /* Stream the external-flash text log over the debug UART. Runs before any
     * operational task is created, so there is no concurrent writer. After a
     * reset this shows everything DBG_LOG()/LOG() persisted in the previous run —
     * the readback test for the external flash. The markers use the UART-only
     * path so they are not themselves written back into the flash log. */
    DBGLOG_vPutDebug("\r\n==== EXT-FLASH LOG DUMP (%lu bytes) ====\r\n",
                     (unsigned long)LOG_u32GetUsedBytes());
    LOG_vStreamToDebug();
    DBGLOG_vPutDebug("\r\n==== EXT-FLASH LOG DUMP END ====\r\n");

    FLASHLOG_vDump();     /* no-op unless ENABLE_FLASH_LOG + DEBUG_OUTPUT_UART both defined */

    HAL_WDT_vReset();

    HAL_ADC_vInit();
    HAL_TIMER_vInit();

    /* Platform must be created before RTC so the heartbeat task handle
     * exists when the first RTC wakeup ISR fires. */
    PLATFORM_vInit();
    SETTINGS_vInit();

    HAL_RTC_vInit();

    osDelay(100);

    LORARADIO_vInit();

#ifdef ENABLE_RADIO_TEST
    /*
     * Radio smoke-test mode: transmit "Blink!\r\n" at 0.5 Hz and confirm
     * the TX-done IRQ fires.  MeshNetwork and DeviceDiscovery are skipped
     * because they also drive the radio and would interfere.
     * Enable DEBUG_OUTPUT_UART (or LISTENER_MODE) alongside this define to
     * see the output on the debug UART.
     */
    RADIO_TEST_vInit();
#else
    MESHNETWORK_vInit();
    DEVICE_DISCOVERY_vInit();

    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        SOLAR_vInit();
#ifdef ENABLE_MOVE
        HAL_SPI_vInit();
        ACC_vInit();
        MOVE_vInit();
#endif
#ifdef ENABLE_GPS
        GPS_vInit();
#endif
    }
    else
    {
#if !defined(ENABLE_DBG_UART) && !defined(LISTENER_MODE)
        FARMRANGER_vInit();
#endif
    }
#endif /* ENABLE_RADIO_TEST */

    TIME_vInit();
    BAT_vInit();
    POWER_vInit();

    BSP_LED_Off(LED_YELLOW);

    systemReadyForSleep = true;

    osThreadExit();
}

bool INIT_bIsSleepReady(void)
{
    return systemReadyForSleep;
}
