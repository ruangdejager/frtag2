/*
 * build_config.h
 *
 * Single place to select this build's compile-time feature set. Every
 * option below used to be a project preprocessor symbol (STM32CubeIDE:
 * Project > Properties > C/C++ Build > Settings > Preprocessor), toggled by
 * hand per flash. That made "what did I actually just flash" hard to
 * recover once the device was on the bench, and easy to lose track of
 * between primary / secondary / bridge builds. Now: comment the options you
 * want in below and rebuild — INIT_vInitialization() also logs the active
 * set over DBG_LOG at boot (see init.c), so it's recoverable from the
 * terminal or the persisted flash/SD log after the fact too.
 *
 * DEBUG, and the toolchain-required CORE_CM4 / USE_HAL_DRIVER / STM32WLE5xx /
 * USE_FULL_LL_DRIVER, remain project preprocessor symbols — they're IDE/
 * toolchain plumbing (ST-generated), not application features, and some are
 * needed by CMSIS/HAL source files compiled before any of our own headers
 * would be visible.
 */

#ifndef CONFIG_BUILD_CONFIG_H_
#define CONFIG_BUILD_CONFIG_H_

/* --------------------------------------------------------------------------
 * FrKernel command-interface transport — select exactly one.
 * See fr_app/Worker/FrKernel/FrKernel_Config.h.
 * -------------------------------------------------------------------------- */
// #define FRKERNEL_INTERFACE_UART        /* debug UART (bench use) */
 #define FRKERNEL_INTERFACE_LORA        /* LoRa radio (deployed secondary) */
//#define FRKERNEL_INTERFACE_LORA_BRIDGE    /* UART<->LoRa relay (bench test rig) */

/* --------------------------------------------------------------------------
 * Persistent-storage backend — select exactly one. Mutually exclusive
 * hardware: never fitted together, share the same SPI2 pins. Drives the
 * text log sink, the accelerometer data store (MicroSD only), and the
 * FrKernel storage commands. See fr_app/Services/Log, fr_app/Services/AccLog.
 * -------------------------------------------------------------------------- */
#define STORAGE_BACKEND_FLASH
// #define STORAGE_BACKEND_MICROSD

/* --------------------------------------------------------------------------
 * Debug transport — select at most one. Neither defined compiles Debug.c to
 * stubs (no output produced). See fr_app/Services/Debug/Debug.h.
 * -------------------------------------------------------------------------- */
#define DEBUG_OUTPUT_UART
// #define DEBUG_OUTPUT_USB               /* not yet implemented -- stub bodies only */

/* --------------------------------------------------------------------------
 * Feature toggles
 *
 * Movement (shake-to-wake / movement sequencing) and GPS are always built
 * in -- not configurable here, no ENABLE_MOVE / ENABLE_GPS guards remain in
 * the code.
 * -------------------------------------------------------------------------- */
#define LEDS_ALLOWED                      /* currently unreferenced in code */
// #define ENABLE_RADIO_TEST              /* radio smoke-test task, see fr_app/Tasks/RadioTest */
// #define ENABLE_LOW_POWER_RECOVERY      /* see fr_app/Tasks/DeviceDiscovery/DeviceDiscovery.h */

/* --------------------------------------------------------------------------
 * Compile-time guards
 * -------------------------------------------------------------------------- */
#if (defined(FRKERNEL_INTERFACE_UART) + defined(FRKERNEL_INTERFACE_LORA) + \
     defined(FRKERNEL_INTERFACE_LORA_BRIDGE)) != 1
#  error "build_config.h: define exactly one of FRKERNEL_INTERFACE_UART, FRKERNEL_INTERFACE_LORA, FRKERNEL_INTERFACE_LORA_BRIDGE"
#endif

#if defined(STORAGE_BACKEND_FLASH) == defined(STORAGE_BACKEND_MICROSD)
#  error "build_config.h: define exactly one of STORAGE_BACKEND_FLASH / STORAGE_BACKEND_MICROSD"
#endif

#endif /* CONFIG_BUILD_CONFIG_H_ */
