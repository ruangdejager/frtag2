/*
 * OtaStore_Driver.h
 *
 * Board abstraction for the OTA image store: routes all storage operations
 * onto the external NOR flash device driver (fr_app/Device/Flash). The OTA
 * store shares that device with the text log, partitioned per
 * OtaStore_Config.h.
 */

#ifndef SERVICES_OTASTORE_OTASTORE_DRIVER_H_
#define SERVICES_OTASTORE_OTASTORE_DRIVER_H_

#include "Flash.h"
#include "Flash_Config.h"

#define OTASTORE_DRIVER_bRead(addr, buf, len)    FLASH_vRead((addr), (buf), (len))
#define OTASTORE_DRIVER_bWrite(addr, buf, len)   FLASH_vPageWrite((addr), (buf), (len))
#define OTASTORE_DRIVER_bSectorErase(addr)       FLASH_vSectorErase((addr))
#define OTASTORE_DRIVER_SECTOR_SIZE              FLASH_SECTOR_SIZE_BYTES
#define OTASTORE_DRIVER_PAGE_SIZE                FLASH_PAGE_SIZE_BYTES

#endif /* SERVICES_OTASTORE_OTASTORE_DRIVER_H_ */
