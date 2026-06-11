/*
 * FrKernel.h
 *
 * Device command interface Worker.
 * Transport selected at compile time via FRKERNEL_INTERFACE_UART or
 * FRKERNEL_INTERFACE_LORA (see FrKernel_Config.h).
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_H_
#define WORKER_FRKERNEL_FRKERNEL_H_

#include <stdbool.h>

void FRKERNEL_vInit(void);

/* Returns true while a kernel session is active (device must not sleep). */
bool FRKERNEL_bIsConnected(void);

#endif /* WORKER_FRKERNEL_FRKERNEL_H_ */
