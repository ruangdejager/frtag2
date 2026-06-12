/*
 * SolarPower.h
 *
 * Solar power measurement — public interface.
 *
 * VSOLAR (panel voltage) is sampled once every 10 seconds and is always
 * available via SOLAR_u16GetVSolarMV(). The RSENSE-derived current/power/
 * coulomb interface is only present when ENABLE_SOLAR_POWER_SENSE is defined
 * (see SolarPower_Config.h — disabled on this board revision).
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_H_
#define WORKER_SOLARPOWER_SOLARPOWER_H_

#include <stdint.h>

#include "SolarPower_Config.h"

void     SOLAR_vInit(void);

uint16_t SOLAR_u16GetVSolarMV(void);    /* last solar panel voltage, mV          */

#ifdef ENABLE_SOLAR_POWER_SENSE
uint16_t SOLAR_u16GetVRSenseMV(void);   /* last sense resistor voltage, mV       */
int32_t  SOLAR_i32GetCurrentMA(void);   /* panel current (Vrsense / 1.5 Ω), mA  */
uint32_t SOLAR_u32GetPowerMW(void);     /* panel power (Vsolar × I), mW          */

float    SOLAR_fGetCoulombs(void);      /* total harvested coulombs              */
void     SOLAR_vResetCoulombs(void);    /* reset coulomb counter to 0            */
#endif /* ENABLE_SOLAR_POWER_SENSE */

#endif /* WORKER_SOLARPOWER_SOLARPOWER_H_ */
