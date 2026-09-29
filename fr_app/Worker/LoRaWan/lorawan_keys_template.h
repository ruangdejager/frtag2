/*
 * lorawan_keys_template.h
 *
 * Template for lorawan_keys.h, which holds the OTAA join credentials and is
 * git-ignored. To set up a build:
 *
 *   1. Copy this file to lorawan_keys.h in the same directory.
 *   2. In the TTN console, open the application the tags are registered in
 *      and copy the AppKey (MSB order) into FRTAG_LORAWAN_APP_KEY below,
 *      one byte per comma-separated pair, no 0x prefix.
 *   3. Leave the JoinEUI at all zeros unless the TTN end devices were
 *      registered with a different one.
 *
 * Pilot fleet: ONE AppKey shared by every tag (agreed 2026-09-29). Each tag
 * is still registered in TTN separately, by its own DevEUI, all with this
 * same AppKey. The DevEUI is not configured here: every tag derives it from
 * the STM32WL's factory UID64 (LORAWAN_vGetDevEui) and prints it at boot as
 * "lorawan: DevEUI=...". Move to per-device keys before the fleet grows -
 * anyone who reads this key out of one tag can impersonate every tag.
 *
 * TTN registration settings for these tags:
 *   Frequency plan     Europe 863-870 MHz (SF9 for RX2 - recommended)
 *   LoRaWAN version    LoRaWAN Specification 1.0.4
 *   Regional params    RP002 Regional Parameters 1.0.1
 *   Activation         Over the air activation (OTAA)
 *   Class              A only
 */

#ifndef WORKER_LORAWAN_LORAWAN_KEYS_H_
#define WORKER_LORAWAN_LORAWAN_KEYS_H_

/* JoinEUI (a.k.a. AppEUI), big endian */
#define FRTAG_LORAWAN_JOIN_EUI    00,00,00,00,00,00,00,00

/* AppKey (LoRaWAN 1.0.x root key), big endian - REPLACE with the TTN key */
#define FRTAG_LORAWAN_APP_KEY     00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00

#endif /* WORKER_LORAWAN_LORAWAN_KEYS_H_ */
