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
#define FLASH_CMD_READ_STATUS       0x05U   /* Read Status Register 1 */
#define FLASH_CMD_READ_STATUS2      0x35U   /* Read Status Register 2 */
#define FLASH_CMD_WRITE_STATUS      0x01U   /* Write Status Register 1 (+2) */
#define FLASH_CMD_WRITE_ENABLE      0x06U
#define FLASH_CMD_WRITE_DISABLE     0x04U
#define FLASH_CMD_PAGE_PROGRAM      0x02U
#define FLASH_CMD_SECTOR_ERASE      0x20U   /* 4 KB  */
#define FLASH_CMD_BLOCK_ERASE       0xD8U   /* 64 KB */
#define FLASH_CMD_CHIP_ERASE        0x60U
#define FLASH_CMD_DEEP_PWR_DOWN     0xB9U
#define FLASH_CMD_RESUME            0xABU
#define FLASH_CMD_JEDEC_ID          0x9FU

/*
 * This part (JEDEC 1F 14 01) uses a Winbond-style two-register protection model.
 *
 * Status Register 1 (read 05h):
 *   bit0   RDY/BSY  - 1 = program/erase/WRSR in progress
 *   bit1   WEL      - write enable latch (set by 06h)
 *   bit6:2 BP4..BP0 - block-protect size (non-volatile); protects array
 *                     against program/erase. Reads are unaffected.
 *   bit7   SRP0     - Status Register Protect 0 (non-volatile)
 *
 * Status Register 2 (read 35h):
 *   bit0   SRP1     - Status Register Protect 1
 *   bit1   QE       - Quad Enable (when 0, WP#/HOLD# pins are active)
 *   bit6   CMP      - Complement Protect (inverts BP region meaning)
 *
 * SRP1:SRP0 + the WP# pin select how the status register itself is locked
 * (datasheet Table 5):
 *   0:0  -> Software Protected   - SR writable after WREN (factory default)
 *   0:1 + WP#=1 -> Hardware Unprotected - SR writable after WREN
 *   0:1 + WP#=0 -> Hardware Protected   - SR LOCKED, WRSR ignored
 *   1:1  -> One-Time-Program     - SR permanently locked
 * Because WP# is hardwired (no MCU GPIO), a chip that latches SRP0=1 with WP#
 * low can only be unlocked by driving WP# high. Once SRP0 is back to 0, WP#
 * no longer matters. Global Unprotect = WREN then WRSR 00h 00h (clears BP,
 * SRP0 and CMP). See FLASH_bGlobalUnprotect() / FLASH_vInit().
 */
#define FLASH_STATUS_WIP           (1U << 0)   /* SR1: Write In Progress   */
#define FLASH_STATUS_WEL           (1U << 1)   /* SR1: Write Enable Latch  */
#define FLASH_SR1_BP_MASK          (0x1FU << 2)/* SR1: BP0..BP4            */
#define FLASH_SR1_SRP0             (1U << 7)   /* SR1: Status Reg Protect 0 */

#define FLASH_SR2_SRP1             (1U << 0)   /* SR2: Status Reg Protect 1 */
#define FLASH_SR2_QE               (1U << 1)   /* SR2: Quad Enable          */
#define FLASH_SR2_CMP              (1U << 6)   /* SR2: Complement Protect   */

/* Any non-volatile SR1 state that protects the array / locks the SR. */
#define FLASH_STATUS_PROTECTED     (FLASH_SR1_BP_MASK | FLASH_SR1_SRP0)

/* Global Unprotect data byte (written to both SR1 and SR2). */
#define FLASH_STATUS_UNPROTECT     0x00U

/* Expected JEDEC manufacturer ID */
#define FLASH_MANUFACTURER_ID       0x1FU       /* Adesto/Dialog */

/* JEDEC-ID read attempts before declaring the part absent/faulty. A single
 * read is not trustworthy on this board — a transient (supply-rail sag from a
 * concurrent radio/GPS current spike) can corrupt one read of an otherwise
 * healthy chip, and calling that "flash FAIL" is both wrong and dangerous if
 * anything acts on it. */
#define FLASH_JEDEC_READ_ATTEMPTS   5U

/* Cooldown between automatic re-probes once the device has been declared
 * absent. The driver's bDevicePresent gate used to be latched once at boot and
 * never revisited, so ONE bad JEDEC read disabled every write and erase for the
 * whole power cycle - the log froze and FOTA_bEraseScratch failed instantly,
 * reporting FWDONE=ERR with 0 B delivered on every campaign until the unit was
 * power-cycled by hand. Field units cannot be reprogrammed by wire, so a
 * boot-lifetime latch on a transient is not survivable: the gate has to be able
 * to re-open by itself.
 *
 * Rate-limited because the gate exists to keep a genuinely absent chip cheap -
 * LOG_vInit alone probes every sector, and an unthrottled retried probe
 * (FLASH_JEDEC_READ_ATTEMPTS x the 2 ms settle delay) would put ~10 ms into
 * every one of those calls. One probe per interval bounds that to nothing
 * measurable while still recovering well inside a single FOTA session. */
#define FLASH_REPROBE_INTERVAL_MS   5000U

/* Extra probe rounds at boot only, each a full FLASH_JEDEC_READ_ATTEMPTS burst
 * separated by a longer settle. Boot is the one moment where getting this wrong
 * is most expensive and retrying is cheapest: FLASH_vInit runs a few
 * milliseconds after power-up, in the worst part of the rail for a clean SPI
 * read, and LOG_vInit immediately behind it scans every sector to find the log
 * head. Entering that scan with the gate wrongly shut costs a whole boot's
 * logging; spending ~150 ms up front to avoid it is free by comparison. */
#define FLASH_BOOT_PROBE_ROUNDS     3U
#define FLASH_BOOT_PROBE_SETTLE_MS  50U

/* Bytes sampled per erased region to confirm an erase actually took effect.
 * A block-protected device ACCEPTS the erase command, never sets WIP and
 * returns to ready - so FLASH_bWaitReady() succeeds and the erase silently
 * does nothing. Without a read-back the driver reports success and the old
 * contents survive; FOTA would then stage an image over stale bytes and only
 * discover it at the whole-image XOR check, or not at all. */
#define FLASH_ERASE_VERIFY_BYTES    32U

/* Erased NOR reads back all-ones. */
#define FLASH_ERASED_BYTE           0xFFU

/* ---- Durable flash-health word (TAMP->BKP4R) ----
 * Backup registers survive reset and brownout and need no external flash,
 * which is precisely why the health state lives here: when the log device is
 * the casualty, the log cannot be where the fault is recorded. BKP0R..BKP2R
 * belong to the bootloader handshake (see Fota_Config.h); BKP4R is free.
 *
 * Layout: [31:16] magic  [15:8] last JEDEC mfr byte  [7:0] status bits. */
#define FLASH_HEALTH_BKP_MAGIC      0xF1A50000UL
#define FLASH_HEALTH_BKP_MAGIC_MASK 0xFFFF0000UL
#define FLASH_HEALTH_BIT_PRESENT     (1UL << 0)  /* gate open              */
#define FLASH_HEALTH_BIT_EVERABSENT  (1UL << 1)  /* gate has been shut     */
#define FLASH_HEALTH_BIT_PROTECTED   (1UL << 2)  /* BP/SRP0 seen set       */
#define FLASH_HEALTH_BIT_UNPROT_FAIL (1UL << 3)  /* unprotect did not take */
#define FLASH_HEALTH_BIT_ERASEVFAIL  (1UL << 4)  /* erase did not blank    */

#endif /* DEVICE_FLASH_FLASH_CONFIG_H_ */
