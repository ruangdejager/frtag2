/*
 * RadioTest.h
 *
 * Standalone radio link/range test task.
 *
 * Enabled by defining ENABLE_RADIO_TEST in fr_app/inc/config/build_config.h.
 *
 * When ENABLE_RADIO_TEST is defined:
 *   - The GPIO role strap picks what this unit does, so one image serves both
 *     boards of the pair:
 *       SECONDARY — beacons its unique ID + a sequence number every 5 s.
 *       PRIMARY   — listens and logs the RSSI/SNR of each beacon received,
 *                   the channel noise floor and link margin, plus a running
 *                   count of beacons missed (sequence gaps).
 *   - DeviceDiscovery and MeshNetwork are NOT initialised (they also use the
 *     radio), so this task drains the LoRa RX queue itself.
 *   - init.c holds a sleep lock for the life of the build so the unit stays
 *     out of STOP2 and never goes deaf or mute mid-test.
 *
 * DBG_LOG output goes to the debug UART and the ext-flash log, so a range
 * walk is recoverable from flash afterwards.
 */

#ifndef FR_APP_TASKS_RADIOTEST_RADIOTEST_H_
#define FR_APP_TASKS_RADIOTEST_RADIOTEST_H_

#include "build_config.h"

#ifdef ENABLE_RADIO_TEST

void RADIO_TEST_vInit(void);

#endif /* ENABLE_RADIO_TEST */

#endif /* FR_APP_TASKS_RADIOTEST_RADIOTEST_H_ */
