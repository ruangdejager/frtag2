/*
 * init.h
 *
 * Boot-sequence task — runs once at startup to initialise all subsystems.
 */

#ifndef INIT_H_
#define INIT_H_

#include <stdbool.h>

void INIT_vInitialization(void *parameters);
bool INIT_bIsSleepReady(void);

#endif /* INIT_H_ */
