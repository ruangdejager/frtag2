/*
 * FrKernel.h
 *
 * Device command interface Worker.
 * Transport selected at compile time via FRKERNEL_INTERFACE_UART or
 * FRKERNEL_INTERFACE_LORA (see FrKernel_Config.h).
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_H_
#define WORKER_FRKERNEL_FRKERNEL_H_

void FRKERNEL_vInit(void);

#endif /* WORKER_FRKERNEL_FRKERNEL_H_ */
