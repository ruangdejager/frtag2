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
#include "hal_system.h"
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
#include "FrKernel_Config.h"
#include "Battery.h"
#include "SolarPower.h"
#include "Power.h"
#include "FrKernel.h"

#include "flashLog.h"
#include "Flash.h"
#include "Log.h"
#include "DbgLog.h"

#include "storage_config.h"
#ifdef STORAGE_BACKEND_MICROSD
#  include "MicroSD.h"
#  include "AccLog.h"
#endif

#include "cmsis_os2.h"

volatile bool systemReadyForSleep = false;

void INIT_vInitialization(void *parameters)
{
    (void)parameters;

    FLASHLOG_vInit();

    /* Board-level GPIO */
    HAL_GPIO_vInit();

    /* Determine primary / secondary role before UART init. This is the only
     * read of the role strap (PB12); it tristates the pin immediately after, so
     * latch the role into the UART layer here for the USART1 baud/swap choice
     * rather than having HAL_UART_vInit() re-read the strap on every (re-)init. */
    DEVICE_DISCOVERY_vConfigDeviceRole();
    HAL_UART_vSetRole(DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY);

    HAL_UART_vInit();

    /* Both SPI peripherals (ACC + flash) must be up before FLASH_vInit and any
     * DBG_LOG call that writes to ext-flash.  Single call covers both. */
    HAL_SPI_vInit();

    DBGLOG_vInit();

    /* Bring up the selected storage backend (mutually exclusive HW, same SPI2
     * bus). LOG_vInit() then recovers the text-log FIFO on whichever is fitted. */
#ifdef STORAGE_BACKEND_MICROSD
    MICROSD_vInit();
#else
    FLASH_vInit();
#endif
    LOG_vInit();

    /* Ask the DbgLog consumer to stream the external-flash log over the debug
     * UART once it runs — this replays everything DBG_LOG()/LOG() persisted in
     * the previous run (the ext-flash readback test). Routed through the
     * consumer so the dump never interleaves with live log output. */
    DBGLOG_vRequestDump();

    FLASHLOG_vDump();     /* no-op unless ENABLE_FLASH_LOG + DEBUG_OUTPUT_UART both defined */

    // Enable WDT
    HAL_WDT_vInit();
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
    FRKERNEL_vInit();

#ifdef ENABLE_RADIO_TEST
    /*
     * Radio smoke-test mode: transmit "Blink!\r\n" at 0.5 Hz and confirm
     * the TX-done IRQ fires.  MeshNetwork and DeviceDiscovery are skipped
     * because they also drive the radio and would interfere.
     * DBG output is visible on the debug UART (always enabled).
     */
    RADIO_TEST_vInit();
#elif defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
    /*
     * UART<->LoRa FrKernel bridge (bench test rig): this primary's only job
     * is relaying whatever the user types over the debug UART out as a LoRa
     * FrKernel command, and printing whatever answer comes back. MeshNetwork
     * supplies the radio TX/RX plumbing (its parser task is what delivers
     * inbound FrKernel packets to FRKERNEL.c's callback); DeviceDiscovery,
     * Farmranger, GPS and Movement are all skipped, same rationale as
     * ENABLE_RADIO_TEST -- a scheduled campaign or logger session would
     * fight this device for the radio/UART instead of just relaying.
     */
    MESHNETWORK_vInit();

    /* This rig has to stay reachable at any moment: a sleep lock acquired
     * and never released holds the whole system on the tickless-idle LIGHT
     * WFI path forever, so it never drops into STOP2. That matters for two
     * things at once -- STOP2 parks the debug UART pins to analog (the
     * terminal would go dead), and it suspends the CPU core entirely, so
     * the LoRa radio task could only service RX/TX once per 1 Hz RTC
     * heartbeat instead of promptly. Without this the bridge looks "asleep"
     * within moments of boot completing. */
    SYSTEM_vSleepLockAcquire();
#else
    MESHNETWORK_vInit();
    DEVICE_DISCOVERY_vInit();

    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        SOLAR_vInit();
#ifdef ENABLE_MOVE
        ACC_vInit();
#  ifdef STORAGE_BACKEND_MICROSD
        /* Recover the ACC-data write head before the movement task (which feeds
         * the logger from its 1 Hz FIFO drain) starts ticking. */
        ACCLOG_vInit();
#  endif
        MOVE_vInit();
#endif
#ifdef ENABLE_GPS
        GPS_vInit();
#endif
    }
    else
    {
#ifdef ENABLE_MOVE
        /* The accelerometer is fitted and powered on primaries too, but the
         * primary doesn't track movement. Left in its undefined power-on state
         * its I/O drew ~115 uA; configured-but-active (25 Hz, un-drained FIFO)
         * ~45 uA; full power-down was worse (floating SPI inputs crowbar). So
         * use a low-power, no-FIFO idle config - I/O active, nothing to overrun. */
        ACC_vConfigIdle();
#endif
        /* Primary: bring up the Farmranger UART link to the fr9 logger board.
         * (Debug UART is a separate peripheral, so the two no longer conflict.) */
        FARMRANGER_vInit();
    }
#endif /* ENABLE_RADIO_TEST / FRKERNEL_INTERFACE_LORA_BRIDGE */

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
