/*
 * SolarPower.h
 *
 * Solar power measurement — public interface.
 *
 * VSOLAR and RSENSE are sampled once every 10 seconds. The last measured
 * values are always available through the getters below. Coulombs are
 * accumulated on every measurement cycle and can be reset independently.
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_H_
#define WORKER_SOLARPOWER_SOLARPOWER_H_

#include <stdint.h>

void     SOLAR_vInit(void);

uint16_t SOLAR_u16GetVSolarMV(void);    /* last solar panel voltage, mV          */
uint16_t SOLAR_u16GetVRSenseMV(void);   /* last sense resistor voltage, mV       */
int32_t  SOLAR_i32GetCurrentMA(void);   /* panel current (Vrsense / 1.5 Ω), mA  */
uint32_t SOLAR_u32GetPowerMW(void);     /* panel power (Vsolar × I), mW          */

float    SOLAR_fGetCoulombs(void);      /* total harvested coulombs              */
void     SOLAR_vResetCoulombs(void);    /* reset coulomb counter to 0            */

#endif /* WORKER_SOLARPOWER_SOLARPOWER_H_ */
