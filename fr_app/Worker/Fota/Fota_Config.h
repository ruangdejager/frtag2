/*
 * Fota_Config.h
 *
 * SYNCED COPY — this file must be kept byte-identical to
 * frtag2_bootloader/fr_bootloader/inc/Fota_Config.h. It is the single
 * source of truth for the flash layout and handoff contract shared
 * between the application and the bootloader; update both copies
 * together.
 *
 * The OTA file is the raw application binary — the very same bytes that
 * get programmed into internal flash at OTA_APP_BASE_ADDR — hosted
 * publicly on GitHub Pages at
 *   https://ruangdejager.github.io/farmranger-firmware/frtag2-firmware/latest.bin
 * and pointed to by a sibling version.json. The fr9 fetches version.json,
 * decides based on the tag's own current version, then downloads
 * latest.bin to the modem filesystem and streams it byte-for-byte to the
 * primary over UART. On the primary each block is written straight into
 * the external scratchpad at the same offset (file offset N -> scratch
 * offset N); no encoding, no address remap. Validation is an 8-bit XOR
 * over the full image; a metadata record plus a backup-register boot
 * flag hand the image to the bootloader, which programs internal flash
 * and jumps to the application.
 *
 * ---- Internal flash map (STM32WLE5CC, 256 KB, 2 KB pages) ----
 *
 *   0x08000000 - 0x08004FFF   bootloader (20 KB, frtag2_bootloader project)
 *   0x08005000 - 0x0803FFFF   application (236 KB)
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
 *   +0x10  u8   xor8 (XOR over scratch bytes 0..size-1)
 *   +0x11  u8   VALID marker 0xA5 — written LAST by the application
 *               after its own verify pass
 *   +0x12  u8   DISTRIBUTED marker 0xA5 — written by the primary once
 *               the LoRa distribution session for this image has run
 *
 * ---- Bootloader contract ----
 *
 *   The application arms the handoff with:
 *     TAMP->BKP0R = OTA_BOOT_MAGIC, TAMP->BKP1R = fwVersion,
 *     then NVIC_SystemReset().
 *   The bootloader compares meta.fwVersion against the CURRENTLY INSTALLED
 *   app's own version, read directly out of internal flash at
 *   OTA_FW_INFO_ADDR (a FwVersion_t compiled into the app image at a fixed
 *   linker address — see below). If strictly newer, it recomputes the
 *   XOR-8 over scratch 0..size-1 and — on a match — programs
 *   0x08005000..stopAddr from the scratchpad (which, being a raw byte
 *   copy, carries the new image's own .fw_info along with it — no special
 *   handling needed), then clears BKP0R and jumps to the app. On any
 *   mismatch or same-or-older version it clears BKP0R and jumps to the
 *   existing application unchanged.
 *
 *   This deliberately does NOT use a backup register for the installed
 *   version (an earlier revision did, via TAMP->BKP3R written by the app
 *   on every boot): writing TAMP registers requires the RTC backup-domain
 *   clock to be enabled first, and the app's boot sequence calls
 *   FOTA_vInit() before HAL_RTC_vInit() — the write silently no-op'd,
 *   always reading back 0. Reading a fixed flash address has no such
 *   ordering dependency.
 */

#ifndef WORKER_FOTA_FOTA_CONFIG_H_
#define WORKER_FOTA_FOTA_CONFIG_H_

#include <stdint.h>

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
#define OTA_META_OFF_DISTRIBUTED 0x12U
#define OTA_META_RECORD_LEN      0x13U

#define OTA_META_MAGIC           0x46524F54UL               /* "FROT"         */
#define OTA_META_MARKER          0xA5U

/* ---- Application/bootloader boot flag (TAMP backup registers) ----
 *   BKP0R = OTA_BOOT_MAGIC when app wants bootloader attention
 *   BKP1R = staged image version (informational, set alongside magic)
 *   BKP2R = bootloader's own version (written by bootloader on every boot) */
#define OTA_BOOT_MAGIC           0x4F544152UL               /* "OTAR"         */

/* ---- Embedded firmware-version header ----
 * The app's own version is compiled directly into its image at this fixed
 * address (fr_app's linker script places a KEEP'd ".fw_info" section
 * here — see STM32WLE5CCUX_FLASH.ld) so the bootloader can read "what's
 * currently installed" straight out of internal flash with a plain
 * pointer dereference. Offset 0x200 sits comfortably past the vector
 * table (83 entries * 4 B = 332 B on this build) with margin for growth.
 * An erased app region reads back {0xFFFF,0xFFFF,0xFFFF} — the bootloader
 * treats that as "no app installed" (version 0), not as a huge version. */
#define OTA_FW_INFO_ADDR         (OTA_APP_BASE_ADDR + 0x200UL)

typedef struct
{
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
} FwVersion_t;

/* ---- Verify pass ---- */
#define OTA_XOR_BUF_LEN          64U    /* streamed checksum read chunk       */

/* ---- UART acquire (fr9 -> primary) ---- */
#define OTA_UART_BLOCK_LEN        1024U   /* raw file bytes per AT+FWGET pull  */
#define OTA_UART_BLOCK_RETRIES    3U      /* attempts per block                */
#define OTA_UART_RETRY_DELAY_MS   250U    /* settle time between attempts      */
#define OTA_FWREQ_WAIT_POLL_MS    2000U   /* re-poll period on FW,WAIT         */
#define OTA_FWREQ_WAIT_MAX_MS     90000U  /* covers fr9's own AT+FWCHECK round
                                             trip (modem wake + PSD activate +
                                             two HTTPS GETs)                  */
#define OTA_FWCHECK_ATSEND_TIMEOUT_MS 2000U

/* ---- LoRa distribution (primary -> secondaries) ---- */
#define OTA_LORA_CHUNK_LEN        224U    /* image payload bytes per OtaChunk  */
#define OTA_LORA_WINDOW_CHUNKS    64U     /* chunks per blast/repair window    */
#define OTA_LORA_MAX_TARGETS      8U      /* secondaries served per session    */
#define OTA_LORA_PREP_REPEATS     5U      /* OtaPrep announcements, 1 s apart  */
#define OTA_LORA_PREP_GAP_MS      1000U
#define OTA_LORA_CHUNK_GAP_MS     100U    /* pause after each chunk TX, before
                                             the next chunk's flash read. Not
                                             just radio pacing: the SX126x PA's
                                             current spike during TX sags the
                                             shared supply rail, and a flash
                                             read issued too soon after reads
                                             back corrupted bytes — confirmed
                                             on hardware (isolated single-TX
                                             test: a read right after TX was
                                             wrong, a second back-to-back
                                             re-read was ALSO wrong and
                                             different again, but a third read
                                             ~65 ms after TX matched the known-
                                             good value exactly). 15 ms was
                                             not enough settle time; 100 ms
                                             gives comfortable margin. */
#define OTA_LORA_POLL_TIMEOUT_MS  6000U   /* wait for one OtaReport. Needs to
                                             cover the primary's own poll TX
                                             sitting behind remaining chunks
                                             in the radio TX queue (up to
                                             ~1 s under a 64-chunk blast),
                                             plus secondary CAD + reply
                                             airtime + primary RX. 2500 ms
                                             was the boundary and every
                                             post-mid-session window was
                                             timing out just as the reply
                                             arrived on the air.            */
#define OTA_LORA_REPAIR_ROUNDS    3U      /* repair passes per window          */
#define OTA_LORA_RX_IDLE_MS       20000U  /* secondary: session silence abort  */

/* How long a latched-but-unserviced OtaPrep stays valid on a secondary.
 *
 * bPrepPending is set by the mesh-parser task and consumed by the AppTask.
 * If the AppTask has already left its campaign loop when the Prep lands (a
 * congested campaign can push the primary's TimeSync/OtaPrep phase out past
 * the secondary's silence timeout), nothing ever consumes it — and the
 * first-primary-wins latch then rejects EVERY later OtaPrep because its
 * session id differs. Field case: one unit sat locked to a dead session id
 * for ~18 h, refusing an update all 37 of its peers took, and only recovered
 * when a shake-wake happened to disarm acceptance.
 *
 * So a pending Prep expires: past this age with no receive started, a
 * different-session OtaPrep is allowed to take over. Sized off
 * OTA_LORA_RX_IDLE_MS — the same "this session is dead" scale used for an
 * in-progress transfer. */
#define OTA_PREP_PENDING_MAX_MS   OTA_LORA_RX_IDLE_MS
#define OTA_LORA_SESSION_MAX_MS   (12UL * 60UL * 1000UL)

/* ---- Multi-primary coexistence (listen-before-distribute) ----
 * Two primaries driven by the same fr9 schedule wake, TimeSync and start
 * distributing in the same second, so their OtaPrep/chunk/ack exchanges
 * collide and secondaries between them join neither. Before broadcasting
 * its first OtaPrep a primary waits a random backoff and listens; if it
 * hears another primary's live OTA session (any Ota* packet whose session
 * id isn't ours) it defers this campaign and lets the other finish. The
 * random backoff breaks the phase symmetry so one primary always Preps
 * first; per-campaign randomness keeps it fair and self-healing. */
#define OTA_DISTRIBUTE_BACKOFF_SPREAD_MS  5000U   /* random pre-Prep backoff; kept
                                                     well under APP_OTA_PREP_WAIT_MS
                                                     (15 s) so the winner's Prep
                                                     still lands in the secondary's
                                                     arm window                    */
#define OTA_FOREIGN_ACTIVE_MS             6000U   /* foreign OTA counts as "another
                                                     primary active" if heard this
                                                     recently (~one PREP burst:
                                                     REPEATS 5 * GAP 1000 ms)       */

#endif /* WORKER_FOTA_FOTA_CONFIG_H_ */
