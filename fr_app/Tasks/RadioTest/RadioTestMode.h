/*
 * RadioTestMode.h
 *
 * Runtime radio link/range test — R&D bench + field tool, built into the
 * PRODUCTION firmware (unlike ENABLE_RADIO_TEST in RadioTest.h, which is a
 * compile-time build that replaces the mesh entirely).
 *
 * Nothing here runs until a FrKernel command explicitly enters the mode, and
 * everything it touches is behind RADIOTESTMODE_bActive(). A unit that is
 * never given the command behaves exactly as it did before this file existed:
 * no task is created, no queue is allocated, no sleep lock is held, and every
 * gate in MeshNetwork / DeviceDiscovery is a single already-false read.
 *
 * Entering (both roles):   tag <ID> radiotest
 *
 *   SECONDARY -> beacons "RT <id> <seq>" every 5 s and goes deliberately deaf:
 *                every inbound radio packet is dropped, including FrKernel, so
 *                it cannot be commanded off the air. Shake-to-wake is the only
 *                way out, matching how solarsleep behaves.
 *
 *   PRIMARY   -> listens. Logs RSSI/SNR/noise-floor/link-margin and missed-
 *                beacon count for every beacon heard, and pushes each one to
 *                the fr9 logger board over the Farmranger UART as it is
 *                measured, so the run is on fr9 flash live rather than at the
 *                end. Keeps the FrKernel door open so "tag <ID> radio stop"
 *                can end the mode over the air.
 *
 * The mode is RAM-only and deliberately not persisted: a reboot returns the
 * unit to normal operation. A mode that survived a power cycle would be an
 * easy way to strand a unit in the field with no mesh participation and, on a
 * secondary, no over-the-air way back.
 */

#ifndef FR_APP_TASKS_RADIOTEST_RADIOTESTMODE_H_
#define FR_APP_TASKS_RADIOTEST_RADIOTESTMODE_H_

#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

/* True while the R&D radio test is running. Every hook this feature adds to
 * the core (MeshNetwork parser + TX, DeviceDiscovery scheduler) is guarded by
 * this and does nothing when it is false. Safe to call from any task. */
bool RADIOTESTMODE_bActive(void);

/* True when this unit is the listening primary rather than the beaconing
 * secondary. Only meaningful while RADIOTESTMODE_bActive(). */
bool RADIOTESTMODE_bIsListener(void);

/* --------------------------------------------------------------------------
 * Entry / exit
 * -------------------------------------------------------------------------- */

/* Enter the mode. Role is taken from the GPIO strap. Returns false if already
 * active or if the task/queue could not be created (in which case nothing is
 * left half-started). Call from a task context, not an ISR — it allocates.
 *
 * The caller should have already sent its command acknowledgement and let it
 * drain: on a secondary this call is the point of no return for the radio. */
bool RADIOTESTMODE_bEnter(void);

/* Leave the mode and restore normal operation: stops the beacon, tears down
 * the task and queue, releases the sleep lock and re-opens the mesh gates.
 * Safe to call when not active (no-op), and safe to call from the Movement
 * task via the shake path. pcReason is logged. */
void RADIOTESTMODE_vExit(const char *pcReason);

/* --------------------------------------------------------------------------
 * Radio hook
 * -------------------------------------------------------------------------- */

/* Hand a received beacon frame to the listener. Called from
 * MESHNETWORK_vParserTask for MeshPktType_Reserved frames while the mode is
 * active; pBuf/u8Len are the payload with the type byte already stripped.
 *
 * Deliberately cheap: it parses and enqueues, nothing more. The logging and
 * the Farmranger round-trip happen on the RadioTest task so a slow fr9
 * handshake can never stall the parser and back up the LoRa RX queue. */
void RADIOTESTMODE_vOnBeacon(const uint8_t *pBuf, uint8_t u8Len,
                             int16_t i16Rssi, int8_t i8Snr, int16_t i16NoiseFloor);

#endif /* FR_APP_TASKS_RADIOTEST_RADIOTESTMODE_H_ */
