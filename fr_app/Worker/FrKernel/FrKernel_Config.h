/*
 * FrKernel_Config.h
 *
 * Compile-time configuration for the FrKernel command interface.
 * Transport is selected in fr_app/inc/config/build_config.h — either or
 * both of:
 *   FRKERNEL_INTERFACE_UART        — debug UART (bench use)
 *   FRKERNEL_INTERFACE_LORA        — LoRa radio (deployed device)
 * Defining both lets the same build answer "tag <cmd>" over UART and
 * "tag <ID> <cmd>" over the mesh at the same time, sharing one session
 * (FRKERNEL_bIsConnected()/inactivity timer aren't per-transport — a
 * command on either side keeps the device awake and extends the timeout).
 *
 * Or, mutually exclusive with both of the above:
 *   FRKERNEL_INTERFACE_LORA_BRIDGE — UART<->LoRa relay (bench test rig: a
 *                                    primary that forwards whatever's typed
 *                                    over UART out as a LoRa FrKernel command
 *                                    and prints whatever comes back. Also
 *                                    skips DeviceDiscovery/Farmranger/GPS —
 *                                    see init.c)
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_CONFIG_H_
#define WORKER_FRKERNEL_FRKERNEL_CONFIG_H_

#include "build_config.h"

#define FRKERNEL_CMD_PREFIX             "tag"
#define FRKERNEL_LINE_BUF_LEN          128U    /* max command line length (bytes)        */
#define FRKERNEL_RESP_BUF_LEN          256U    /* max response string (bytes)            */
#define FRKERNEL_LORA_QUEUE_LEN          4U    /* LoRa path: inbound queue depth         */
#define FRKERNEL_INACTIVITY_TIMEOUT_MS  (5U * 60U * 1000U)   /* auto-release after 5 min */
#define FRKERNEL_POLL_INTERVAL_MS       10000U /* inactivity check cadence (ms)          */

#endif /* WORKER_FRKERNEL_FRKERNEL_CONFIG_H_ */
