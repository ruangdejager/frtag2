/*
 * datetime.h
 *
 * Calendar date/time struct and UNIX-timestamp conversion utilities.
 */

#ifndef DATETIME_H_
#define DATETIME_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t  second;    //!< 0–59
    uint8_t  minute;    //!< 0–59
    uint8_t  hour;      //!< 0–23
    uint8_t  date;      //!< 0–30  (first day of month = 0)
    uint8_t  month;     //!< 0 = January … 11 = December
    uint16_t year;      //!< 1970–2105
    uint8_t  dayofweek; //!< 0 = Sunday … 6 = Saturday
} datetime_t;

bool     DATETIME_bIsDateValid(datetime_t *date);
void     DATETIME_vTimestampToDate(uint32_t timestamp, datetime_t *date_out);
void     DATETIME_vTimestampToDateTZ(uint32_t timestamp, int8_t hour, uint8_t min, datetime_t *date_out);
uint32_t DATETIME_u32DateTimetoTimestamp(datetime_t *date);
uint32_t DATETIME_u32DateTimetoTimestampTZ(datetime_t *date, int8_t hour, uint8_t min);

#endif /* DATETIME_H_ */
