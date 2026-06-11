/*
 * FrKernel_Config.h
 *
 * Compile-time configuration for the FrKernel command interface.
 * Define exactly one transport in the project preprocessor settings:
 *   FRKERNEL_INTERFACE_UART  — debug UART (bench use)
 *   FRKERNEL_INTERFACE_LORA  — LoRa radio (deployed device)
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_CONFIG_H_
#define WORKER_FRKERNEL_FRKERNEL_CONFIG_H_

#if defined(FRKERNEL_INTERFACE_UART) && defined(FRKERNEL_INTERFACE_LORA)
#  error "Define only one of FRKERNEL_INTERFACE_UART or FRKERNEL_INTERFACE_LORA"
#endif
#if !defined(FRKERNEL_INTERFACE_UART) && !defined(FRKERNEL_INTERFACE_LORA)
#  error "Define one of FRKERNEL_INTERFACE_UART or FRKERNEL_INTERFACE_LORA"
#endif

#define FRKERNEL_CMD_PREFIX             "tag"
#define FRKERNEL_LINE_BUF_LEN          128U    /* max command line length (bytes)        */
#define FRKERNEL_RESP_BUF_LEN          256U    /* max response string (bytes)            */
#define FRKERNEL_LORA_QUEUE_LEN          4U    /* LoRa path: inbound queue depth         */
#define FRKERNEL_INACTIVITY_TIMEOUT_MS  (5U * 60U * 1000U)   /* auto-release after 5 min */
#define FRKERNEL_POLL_INTERVAL_MS       10000U /* inactivity check cadence (ms)          */

#endif /* WORKER_FRKERNEL_FRKERNEL_CONFIG_H_ */
