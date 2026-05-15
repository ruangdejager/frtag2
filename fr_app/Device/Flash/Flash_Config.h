/*
 * Flash_Config.h
 *
 * AT25EU0041A-SSHN-T — 4 Mbit (512 KB) SPI NOR flash
 * Manufacturer: Adesto/Dialog (ID 0x1F)
 */

#ifndef DEVICE_FLASH_FLASH_CONFIG_H_
#define DEVICE_FLASH_FLASH_CONFIG_H_

/* Geometry */
#define FLASH_CAPACITY_BYTES        (512U * 1024U)
#define FLASH_PAGE_SIZE_BYTES       256U
#define FLASH_SECTOR_SIZE_BYTES     (4U   * 1024U)   /* 4 KB  sector erase */
#define FLASH_BLOCK_SIZE_BYTES      (64U  * 1024U)   /* 64 KB block  erase */
#define FLASH_NUM_SECTORS           128U
#define FLASH_NUM_BLOCKS            8U
#define FLASH_NUM_PAGES             2048U

/* JEDEC command codes */
#define FLASH_CMD_READ              0x03U
#define FLASH_CMD_READ_STATUS       0x05U
#define FLASH_CMD_WRITE_ENABLE      0x06U
#define FLASH_CMD_PAGE_PROGRAM      0x02U
#define FLASH_CMD_SECTOR_ERASE      0x20U   /* 4 KB  */
#define FLASH_CMD_BLOCK_ERASE       0xD8U   /* 64 KB */
#define FLASH_CMD_CHIP_ERASE        0x60U
#define FLASH_CMD_DEEP_PWR_DOWN     0xB9U
#define FLASH_CMD_RESUME            0xABU
#define FLASH_CMD_JEDEC_ID          0x9FU

/* Status register bits */
#define FLASH_STATUS_WIP            (1U << 0)   /* Write In Progress (busy) */

/* Expected JEDEC manufacturer ID */
#define FLASH_MANUFACTURER_ID       0x1FU       /* Adesto/Dialog */

#endif /* DEVICE_FLASH_FLASH_CONFIG_H_ */
