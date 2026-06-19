/*
 * MicroSD_Config.h
 *
 * Geometry and SPI-mode command set for the MicroSD card.
 *
 * The card is addressed as raw 512-byte logical blocks (no filesystem). It is
 * partitioned into two fixed regions:
 *   - a small LOG region at the start, used as a circular text-log FIFO
 *     (mirrors the NOR-flash log), and
 *   - the remaining ACC region, a linear append-only store for timestamped
 *     accelerometer data that never wraps.
 */

#ifndef DEVICE_MICROSD_MICROSD_CONFIG_H_
#define DEVICE_MICROSD_MICROSD_CONFIG_H_

/* ---- Block geometry ---- */
#define SD_BLOCK_SIZE              512U

/* ---- Region partitioning (logical block addresses) ---- */
#define SD_LOG_REGION_BYTES        (16UL * 1024UL * 1024UL)   /* 16 MB log slice  */
#define SD_LOG_LBA_START           0UL
#define SD_LOG_LBA_COUNT           (SD_LOG_REGION_BYTES / SD_BLOCK_SIZE)
#define SD_ACC_LBA_START           (SD_LOG_LBA_START + SD_LOG_LBA_COUNT)
/* ACC region runs from SD_ACC_LBA_START to the end of the card (filled at
 * runtime from the queried block count); it never wraps. */

/* ---- SPI-mode commands (0x40 | index) ---- */
#define SD_CMD0_GO_IDLE_STATE      0U
#define SD_CMD8_SEND_IF_COND       8U
#define SD_CMD9_SEND_CSD           9U
#define SD_CMD16_SET_BLOCKLEN      16U
#define SD_CMD17_READ_SINGLE       17U
#define SD_CMD24_WRITE_SINGLE      24U
#define SD_CMD55_APP_CMD           55U
#define SD_CMD58_READ_OCR          58U
#define SD_ACMD41_SEND_OP_COND     41U

/* ---- R1 response bits ---- */
#define SD_R1_IDLE_STATE           0x01U
#define SD_R1_ILLEGAL_CMD          0x04U
#define SD_R1_READY                0x00U

/* ---- Data tokens ---- */
#define SD_TOKEN_START_BLOCK       0xFEU   /* CMD17/CMD24 single-block start    */
#define SD_DATA_RESP_MASK          0x1FU
#define SD_DATA_RESP_ACCEPTED      0x05U

/* ---- ACMD41 / CMD58 argument + OCR bits ---- */
#define SD_ARG_HCS                 0x40000000UL   /* host supports high capacity */
#define SD_OCR_CCS                 0x40000000UL   /* card capacity status (SDHC) */

/* ---- CMD8 check pattern (0x1AA = 2.7-3.6 V, pattern 0xAA) ---- */
#define SD_CMD8_ARG                0x000001AAUL
#define SD_CMD8_PATTERN            0xAAU

/* ---- Bounded retry / timeout budgets ---- */
#define SD_INIT_ACMD41_RETRIES     2000U
#define SD_READY_WAIT_RETRIES      100000UL
#define SD_TOKEN_WAIT_RETRIES      100000UL

#endif /* DEVICE_MICROSD_MICROSD_CONFIG_H_ */
