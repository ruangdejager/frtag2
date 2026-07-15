/*
 * Fota_Driver.h
 *
 * Board abstraction for the OTA image store: routes storage operations
 * onto the external NOR flash device driver (fr_app/Device/Flash). The
 * OTA store shares that device with the text log, partitioned per
 * Fota_Config.h.
 */

#ifndef WORKER_FOTA_FOTA_DRIVER_H_
#define WORKER_FOTA_FOTA_DRIVER_H_

#include "Flash.h"
#include "Flash_Config.h"

#define FOTA_DRIVER_bRead(addr, buf, len)    FLASH_vRead((addr), (buf), (len))
#define FOTA_DRIVER_bWrite(addr, buf, len)   FLASH_vPageWrite((addr), (buf), (len))
#define FOTA_DRIVER_bSectorErase(addr)       FLASH_vSectorErase((addr))
#define FOTA_DRIVER_SECTOR_SIZE              FLASH_SECTOR_SIZE_BYTES
#define FOTA_DRIVER_PAGE_SIZE                FLASH_PAGE_SIZE_BYTES

#endif /* WORKER_FOTA_FOTA_DRIVER_H_ */
