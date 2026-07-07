/*
 * Version.h
 *
 * Firmware version access. The packed MMmmpp encoding (e.g. 1.2.13 -> 10213)
 * matches the fr9's FOTA version convention so version comparisons are
 * consistent across the OTA chain (server -> fr9 -> primary -> secondaries).
 */

#ifndef SERVICES_VERSION_VERSION_H_
#define SERVICES_VERSION_VERSION_H_

#include <stdint.h>

#include "Version_Config.h"

/* Returns this build's version packed as MMmmpp. */
#define VERSION_u32Get()   ((uint32_t)VERSION_SW_MAJOR * 10000UL + \
                            (uint32_t)VERSION_SW_MINOR * 100UL   + \
                            (uint32_t)VERSION_SW_PATCH)

#endif /* SERVICES_VERSION_VERSION_H_ */
