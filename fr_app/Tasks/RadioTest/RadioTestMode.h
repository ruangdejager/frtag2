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
 *                It creates NOTHING to do this. The beacon runs on
 *                MESHNETWORK_vTxTask, which is the mesh's packet builder and
 *                sits idle for the duration of a test — see
 *                RADIOTESTMODE_u32ServiceBeacon() below and the comment on
 *                RTM_STACK_SIZE_LISTEN in the .c for why a secondary cannot
 *                afford a task of its own.
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

/* Why the most recent RADIOTESTMODE_bEnter() call failed, or "" if the last
 * call succeeded (or none has been made). A command that arrives over LoRa
 * only ever gets its ack back to the caller — the reason otherwise lands
 * solely on the failing unit's own debug UART, which a remote operator
 * cannot read. FrKernel puts this string in the "could not enter" ack so a
 * field failure is diagnosable without physical access to the unit. */
const char *RADIOTESTMODE_pcLastEnterError(void);

/* --------------------------------------------------------------------------
 * Entry / exit
 * -------------------------------------------------------------------------- */

/* Enter the mode. Role is taken from the GPIO strap. Returns false if already
 * active, if the context/queue/task could not be allocated, or if the beacon
 * role has no MeshNetwork TX worker to run on (in which case nothing is left
 * half-started). Call from a task context, not an ISR — it allocates.
 *
 * The caller should have already sent its command acknowledgement and let it
 * drain: on a secondary this call is the point of no return for the radio. */
bool RADIOTESTMODE_bEnter(void);

/* Leave the mode and restore normal operation: stops the beacon, tears down
 * the listener's task and queue, releases the sleep lock and re-opens the
 * mesh gates.
 * Safe to call when not active (no-op), and safe to call from the Movement
 * task via the shake path. pcReason is logged. */
void RADIOTESTMODE_vExit(const char *pcReason);

/* --------------------------------------------------------------------------
 * Beacon service — secondary only, called from MESHNETWORK_vTxTask
 * -------------------------------------------------------------------------- */

/* Send this secondary's next beacon if one is due, and return how long the
 * caller may block before calling again. Returns osWaitForever whenever this
 * node is not a beaconing radio-test secondary, which is the normal case and
 * costs one volatile read.
 *
 * Called from MESHNETWORK_vTxTask at the top of its wait loop, and nowhere
 * else. That task builds and transmits the mesh's own beacons on a stack
 * sized for exactly this work, and MESHNETWORK_bSendPacket returns false for
 * the whole duration of a test, so it has nothing else to do while one runs.
 * Borrowing it is what lets the beacon role allocate nothing but its context:
 * on a secondary the FreeRTOS heap has no 2 KB block left for a task stack,
 * which is what "could not enter (out of heap (task))" was.
 *
 * Self-timing rather than cadence-driven: it compares against its own
 * deadline, so a caller that wakes early (a stray mesh TX flag) does not
 * bring the next beacon forward. RTM_BEACON_MS spacing is kept by the
 * deadline, not by the caller.
 *
 * A due beacon is still held back if the radio has not finished with the
 * previous one — RTM_BEACON_MS and the radio's own carrier-sense budget are
 * numerically equal, so under a busy channel a naive deadline check alone
 * would enqueue a second beacon on top of a first that is still being
 * carrier-sensed. See the check against LORARADIO_u16GetTxQueueDepth() /
 * LORARADIO_bTxInProgress() in the .c. */
uint32_t RADIOTESTMODE_u32ServiceBeacon(void);

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
