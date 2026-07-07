/*
 * OtaStore_Config.h
 *
 * OTA image storage layout — the single source of truth for the internal
 * flash split (bootloader/application), the external NOR flash partitions,
 * and the application/bootloader handoff contract.
 *
 * Philosophy mirrors the fr9's proven FOTA design: the OTA file travels as
 * "VS,<linecount>\r\n" + Intel HEX text; each receiver decodes HEX -> binary
 * into the external-flash scratchpad at (hexAddr - OTA_APP_BASE_ADDR);
 * validation is line count + 8-bit XOR; a metadata record plus a backup-
 * register boot flag hand the image to the bootloader, which programs
 * internal flash and jumps to the application.
 *
 * ---- Internal flash map (STM32WLE5CC, 256 KB, 2 KB pages) ----
 *
 *   0x08000000 - 0x08004FFF   bootloader (20 KB, future project)
 *   0x08005000 - 0x0803FFFF   application (236 KB)
 *
 * OTA_APP_BASE_ADDR is deliberately identical to the fr9's (0x08005000) so
 * the ported HEX decoder's address arithmetic matches fr9's fota.c exactly.
 *
 * ---- External NOR map (AT25EU0041A, 512 KB, 4 KB sectors) ----
 *
 *   0x00000 - 0x3AFFF   OTA image scratchpad (sectors 0-58, 236 KB;
 *                       scratch offset N maps to OTA_APP_BASE_ADDR + N)
 *   0x3B000 - 0x3BFFF   OTA metadata (sector 59)
 *   0x3C000 - 0x7FFFF   text log (sectors 60-127; see Services/Log/Log.h)
 *
 * ---- Metadata record (at OTA_META_ADDR) ----
 *
 *   +0x00  u32  magic "FROT" (little-endian in flash)
 *   +0x04  u32  fwVersion (MMmmpp)
 *   +0x08  u32  imageStopAddr (last written STM32 address, incl. base)
 *   +0x0C  u32  imageSizeBytes (= stopAddr - base + 1)
 *   +0x10  u8   xor8 (XOR over scratch bytes 0..size-1; erased gaps = 0xFF)
 *   +0x11  u8   VALID marker 0xA5 — written LAST by the application after
 *               its own verify pass
 *   +0x12  u8   CONSUMED marker 0xA5 — written by the BOOTLOADER after it
 *               programmed internal flash (NOR 1->0 write, no erase needed)
 *   +0x13  u8   DISTRIBUTED marker 0xA5 — written by the primary once the
 *               LoRa distribution session for this image has run
 *
 * ---- Bootloader contract (bootloader is a future project) ----
 *
 *   The application arms the handoff with:
 *     TAMP->BKP0R = OTA_BOOT_MAGIC, TAMP->BKP1R = fwVersion,
 *     then NVIC_SystemReset().
 *   On reset the bootloader reads the metadata record; if magic + VALID and
 *   not CONSUMED, it recomputes the XOR-8 over scratch 0..size-1, and on a
 *   match programs 0x08005000..stopAddr from the scratchpad (2 KB page
 *   erase/program), writes CONSUMED, clears BKP0R and jumps to the app
 *   (MSP from [base], reset handler from [base+4] — same as fr9's
 *   bootloader). On any mismatch it clears BKP0R and jumps to the existing
 *   application unchanged.
 */

#ifndef SERVICES_OTASTORE_OTASTORE_CONFIG_H_
#define SERVICES_OTASTORE_OTASTORE_CONFIG_H_

/* ---- Internal flash split ---- */
#define OTA_BOOTLOADER_SIZE      0x5000UL                   /* 20 KB          */
#define OTA_APP_BASE_ADDR        0x08005000UL               /* == fr9's base  */
#define OTA_APP_MAX_SIZE         0x3B000UL                  /* 236 KB         */

/* ---- External NOR partitions ---- */
#define OTA_SCRATCH_START_ADDR   0x000000UL
#define OTA_SCRATCH_SIZE         OTA_APP_MAX_SIZE           /* byte-mapped    */
#define OTA_META_ADDR            (OTA_SCRATCH_START_ADDR + OTA_SCRATCH_SIZE)
#define OTA_META_SECTOR_ADDR     OTA_META_ADDR              /* sector-aligned */

/* ---- Metadata field offsets (relative to OTA_META_ADDR) ---- */
#define OTA_META_OFF_MAGIC       0x00U
#define OTA_META_OFF_VERSION     0x04U
#define OTA_META_OFF_STOP_ADDR   0x08U
#define OTA_META_OFF_SIZE        0x0CU
#define OTA_META_OFF_XOR8        0x10U
#define OTA_META_OFF_VALID       0x11U
#define OTA_META_OFF_CONSUMED    0x12U
#define OTA_META_OFF_DISTRIBUTED 0x13U
#define OTA_META_RECORD_LEN      0x14U

#define OTA_META_MAGIC           0x46524F54UL               /* "FROT"         */
#define OTA_META_MARKER          0xA5U

/* ---- Application/bootloader boot flag (TAMP backup registers) ---- */
#define OTA_BOOT_MAGIC           0x4F544152UL               /* "OTAR"         */

/* ---- Verify pass ---- */
#define OTA_XOR_BUF_LEN          64U    /* streamed checksum read chunk       */

#endif /* SERVICES_OTASTORE_OTASTORE_CONFIG_H_ */
