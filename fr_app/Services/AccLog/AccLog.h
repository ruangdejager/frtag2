/*
 * AccLog.h
 *
 * Accelerometer data logger -> MicroSD ACC region (raw blocks, linear,
 * append-only, never wraps). Active only when STORAGE_BACKEND_MICROSD is
 * selected; on the flash backend these functions are not built and must not
 * be called (callers guard on STORAGE_BACKEND_MICROSD).
 *
 * Driven from the Movement worker's existing 1 Hz ACC-FIFO drain so the
 * destructive hardware FIFO is read exactly once per sample:
 *
 *     ACCLOG_vBeginTick(RTC_u64GetUTC());
 *     for each sample drained: ACCLOG_vAddSample(raw6);
 *     ACCLOG_vEndTick();
 *
 * Each tick is stored as one framed record:
 *   [u16 magic][u64 utc][u8 count][count * 6 raw XYZ bytes]
 * packed into a 512-byte block stream and flushed every tick for durability.
 */

#ifndef SERVICES_ACCLOG_ACCLOG_H_
#define SERVICES_ACCLOG_ACCLOG_H_

#include <stdint.h>

/* Frame format */
#define ACCLOG_REC_MAGIC0     0xACU
#define ACCLOG_REC_MAGIC1     0x11U
#define ACCLOG_REC_HDR_LEN    11U                      /* magic(2)+utc(8)+count(1) */
#define ACCLOG_SAMPLE_LEN     6U                       /* int16 X,Y,Z              */
#define ACCLOG_MAX_SAMPLES    32U                      /* HW FIFO depth is 31      */
#define ACCLOG_REC_MAX        (ACCLOG_REC_HDR_LEN + ACCLOG_MAX_SAMPLES * ACCLOG_SAMPLE_LEN)

void ACCLOG_vInit(void);
void ACCLOG_vBeginTick(uint64_t u64Utc);
void ACCLOG_vAddSample(const uint8_t *pu8Raw6);
void ACCLOG_vEndTick(void);

/* Request a wipe of the ACC region (for 'sd clear'). Deferred: the reset runs
 * at the next 1 Hz tick on the movement task so it can't race acc writes.
 * No-op while in production sleep / when movement isn't ticking. */
void ACCLOG_vRequestErase(void);

#endif /* SERVICES_ACCLOG_ACCLOG_H_ */
