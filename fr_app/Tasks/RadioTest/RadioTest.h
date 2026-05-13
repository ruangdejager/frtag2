/*
 * RadioTest.h
 *
 * Standalone radio smoke-test task.
 *
 * Enabled by defining ENABLE_RADIO_TEST in the project preprocessor settings
 * (STM32CubeIDE: Project > Properties > C/C++ Build > Settings > Preprocessor).
 *
 * When ENABLE_RADIO_TEST is defined:
 *   - This task transmits "Blink!\r\n" at 0.5 Hz over LoRa.
 *   - The LoRa radio task logs "LoraRadio: TX done IRQ" on every successful TX.
 *   - DeviceDiscovery and MeshNetwork are NOT initialised (they also use the radio).
 *
 * Pair with ENABLE_DBG_UART (or LISTENER_MODE) so DBG output is visible.
 */

#ifndef FR_APP_TASKS_RADIOTEST_RADIOTEST_H_
#define FR_APP_TASKS_RADIOTEST_RADIOTEST_H_

#ifdef ENABLE_RADIO_TEST

void RADIO_TEST_vInit(void);

#endif /* ENABLE_RADIO_TEST */

#endif /* FR_APP_TASKS_RADIOTEST_RADIOTEST_H_ */
