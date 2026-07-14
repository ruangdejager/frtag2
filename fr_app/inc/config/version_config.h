/*
 * version_config.h
 *
 * Firmware version of this build. Semantic versioning (MAJOR.MINOR.PATCH);
 * bump per the Farmranger coding standard section 12.
 *
 * Also provides VERSION_u32Get() — this build's version packed as MMmmpp
 * (e.g. 1.2.13 -> 10213), matching the fr9's FOTA version convention so
 * version comparisons are consistent across the OTA chain (server -> fr9 ->
 * primary -> secondaries).
 *
 * build_scripts/create_release_hex_files.ps1 (frtag2 and fr9_application)
 * reads VERSION_SW_MAJOR/MINOR/PATCH out of this file by regex to name the
 * release hex — keep the macro names and this file's path in sync with that
 * script if either changes.
 */

#ifndef FR_APP_INC_CONFIG_VERSION_CONFIG_H_
#define FR_APP_INC_CONFIG_VERSION_CONFIG_H_

#include <stdint.h>

#define VERSION_SW_MAJOR    1
#define VERSION_SW_MINOR    0
#define VERSION_SW_PATCH    0

#define VERSION_u32Get()   ((uint32_t)VERSION_SW_MAJOR * 10000UL + \
                            (uint32_t)VERSION_SW_MINOR * 100UL   + \
                            (uint32_t)VERSION_SW_PATCH)

#endif /* FR_APP_INC_CONFIG_VERSION_CONFIG_H_ */
