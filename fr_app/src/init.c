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
 *   4. ADC, TIM2 tick counter
 *   5. Platform (creates heartbeat task — must come before HAL_RTC_vInit)
 *   6. Settings
 *   7. RTC  (starts 1 Hz wakeup interrupt — heartbeat task must exist)
 *   8. LoRa radio stack
 *   9. Role-specific subsystems
 *  10. Time, battery, power management
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
#include "debug_uart_output.h"
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
    FARMRANGER_vInit();   /* must be first — DBG() routes through this */
#elif defined(ENABLE_DBG_UART)
    DBG_UART_vInit();
    FLASHLOG_vDump();
#endif

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
     * Enable ENABLE_DBG_UART (or LISTENER_MODE) alongside this define to
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
