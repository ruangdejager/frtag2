/*
 * RadioTest.h
 *
 * Standalone radio smoke-test task.
 *
 * Enabled by defining ENABLE_RADIO_TEST in fr_app/inc/config/build_config.h.
 *
 * When ENABLE_RADIO_TEST is defined:
 *   - This task transmits "Blink!\r\n" at 0.5 Hz over LoRa.
 *   - The LoRa radio task logs "LoraRadio: TX done IRQ" on every successful TX.
 *   - DeviceDiscovery and MeshNetwork are NOT initialised (they also use the radio).
 *
 * DBG output is visible on the debug UART (always enabled).
 */

#ifndef FR_APP_TASKS_RADIOTEST_RADIOTEST_H_
#define FR_APP_TASKS_RADIOTEST_RADIOTEST_H_

#include "build_config.h"

#ifdef ENABLE_RADIO_TEST

void RADIO_TEST_vInit(void);

#endif /* ENABLE_RADIO_TEST */

#endif /* FR_APP_TASKS_RADIOTEST_RADIOTEST_H_ */
