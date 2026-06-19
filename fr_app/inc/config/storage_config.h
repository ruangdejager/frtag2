/*
 * storage_config.h
 *
 * Selects the persistent-storage backend for the board.
 *
 * The external NOR flash (fr_app/Device/Flash) and the MicroSD card
 * (fr_app/Device/MicroSD) are MUTUALLY EXCLUSIVE hardware: they are never
 * fitted together and share the exact same SPI2 pins (PA5/PA8/PA10/PA15,
 * see Hal/hal_bsp.h). Exactly one of the two backends below must be defined.
 *
 * Selected backend drives:
 *   - the text log sink (fr_app/Services/Log)         — circular, wraps
 *   - the accelerometer data store (MicroSD only)     — linear, never wraps
 *   - the FrKernel storage commands (clear / stream)
 *
 * To switch hardware, change the single active #define below (or override it
 * with a -D in the build) and rebuild.
 */

#ifndef CONFIG_STORAGE_CONFIG_H_
#define CONFIG_STORAGE_CONFIG_H_

/* ---- Select exactly one ---- */
#define STORAGE_BACKEND_FLASH
//#define STORAGE_BACKEND_MICROSD

/* Compile-time guard: exactly one backend, never both, never neither. */
#if defined(STORAGE_BACKEND_FLASH) == defined(STORAGE_BACKEND_MICROSD)
#  error "storage_config.h: define exactly one of STORAGE_BACKEND_FLASH / STORAGE_BACKEND_MICROSD"
#endif

#endif /* CONFIG_STORAGE_CONFIG_H_ */
