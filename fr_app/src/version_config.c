/*
 * version_config.c
 *
 * Defines gFwVersion (declared in version_config.h) at the fixed flash
 * address OTA_FW_INFO_ADDR via the ".fw_info" linker section (see
 * STM32WLE5CCUX_FLASH.ld). The OTA bootloader reads this struct directly
 * out of internal flash to determine the currently-installed app version
 * — see Fota_Config.h for the full contract.
 */

#include "version_config.h"

__attribute__((section(".fw_info")))
const FwVersion_t gFwVersion =
{
    .major = VERSION_SW_MAJOR,
    .minor = VERSION_SW_MINOR,
    .patch = VERSION_SW_PATCH,
};
