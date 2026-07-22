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
#include "build_config.h"
#include "dbg_log.h"
#include "platform_rtc.h"
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
#include "Fota.h"
#include "DbgLog.h"

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

    /* Record the active build_config.h feature set at the top of every boot
     * log, so "what did I actually flash" is recoverable from the terminal
     * (or, once persisted, the flash/SD log) instead of only from whatever
     * .cproject or build_config.h happened to say at build time. Only lists
     * options this TU actually saw defined -- the string is assembled by the
     * preprocessor, not read back at runtime. */
    DBG_LOG("\r\n\r\nBuild config:\r\n"
#ifdef FRKERNEL_INTERFACE_UART
        " FRKERNEL_INTERFACE_UART\r\n"
#endif
#ifdef FRKERNEL_INTERFACE_LORA
        " FRKERNEL_INTERFACE_LORA\r\n"
#endif
#ifdef FRKERNEL_INTERFACE_LORA_BRIDGE
        " FRKERNEL_INTERFACE_LORA_BRIDGE\r\n"
#endif
#ifdef STORAGE_BACKEND_FLASH
        " STORAGE_BACKEND_FLASH\r\n"
#endif
#ifdef STORAGE_BACKEND_MICROSD
        " STORAGE_BACKEND_MICROSD\r\n"
#endif
#ifdef DEBUG_OUTPUT_UART
        " DEBUG_OUTPUT_UART\r\n"
#endif
#ifdef DEBUG_OUTPUT_USB
        " DEBUG_OUTPUT_USB\r\n"
#endif
#ifdef LEDS_ALLOWED
        " LEDS_ALLOWED\r\n"
#endif
#ifdef ENABLE_RADIO_TEST
        " ENABLE_RADIO_TEST\r\n"
#endif
#ifdef ENABLE_LOW_POWER_RECOVERY
        " ENABLE_LOW_POWER_RECOVERY\r\n"
#endif
        "\r\n");

    /* Bring up the selected storage backend (mutually exclusive HW, same SPI2
     * bus). LOG_vInit() then recovers the text-log FIFO on whichever is fitted. */
#ifdef STORAGE_BACKEND_MICROSD
    MICROSD_vInit();
#else
    FLASH_vInit();
    /* One-time OTA partition migration + latch installed-version into
     * TAMP->BKP3R for the bootloader to compare against (see Fota_Config.h). */
    FOTA_vInit();
#endif
    LOG_vInit();

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

    /* Restore UTC from TAMP backup registers if it survived the reset.
     * OTA-triggered NVIC_SystemReset() keeps VBAT alive, so the value
     * persisted by the heartbeat task before the reset is still in
     * BKP4R/BKP5R — apply it now so log timestamps don't jump back to
     * 1970 while we wait for the next TimeSync. */
    {
        uint64_t u64SavedUtc = 0;
        if (HAL_RTC_bLoadPersistedUtc(&u64SavedUtc))
        {
            RTC_vSetUTC(u64SavedUtc);
            DBG_LOG("RTC: restored UTC=%lu from backup registers\r\n",
                    (unsigned long)u64SavedUtc);
        }
    }

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
        ACC_vInit();
#ifdef STORAGE_BACKEND_MICROSD
        /* Recover the ACC-data write head before the movement task (which feeds
         * the logger from its 1 Hz FIFO drain) starts ticking. */
        ACCLOG_vInit();
#endif
        MOVE_vInit();
        GPS_vInit();
    }
    else
    {
        /* The accelerometer is fitted and powered on primaries too, but the
         * primary doesn't track movement. Left in its undefined power-on state
         * its I/O drew ~115 uA; configured-but-active (25 Hz, un-drained FIFO)
         * ~45 uA; full power-down was worse (floating SPI inputs crowbar). So
         * use a low-power, no-FIFO idle config - I/O active, nothing to overrun. */
        ACC_vConfigIdle();
        /* Primary: bring up the Farmranger UART link to the fr9 logger board.
         * (Debug UART is a separate peripheral, so the two no longer conflict.) */
        FARMRANGER_vInit();
    }
#endif /* ENABLE_RADIO_TEST / FRKERNEL_INTERFACE_LORA_BRIDGE */

    TIME_vInit();
    BAT_vInit();
    POWER_vInit();

    BSP_LED_Off(LED_YELLOW);

    /* Let the whole boot-time DBG_LOG output -- including the build-config
     * line above -- actually drain out of DbgLog's ring and off the wire
     * before this task exits. systemReadyForSleep is what gates the first
     * STOP2 entry (INIT_bIsSleepReady(), checked by
     * vPortSuppressTicksAndSleep()), so delaying it here is what makes the
     * boot log's completion a precondition of the first sleep, not a race
     * against it (same "let the log line drain" pattern used before
     * FrKernel's prodsleep and FOTA_vArmBootloaderAndReset's reset). */
    osDelay(100);

    systemReadyForSleep = true;

    osThreadExit();
}

bool INIT_bIsSleepReady(void)
{
    return systemReadyForSleep;
}
