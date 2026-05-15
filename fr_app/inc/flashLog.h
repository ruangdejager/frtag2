/*
 * flashLog.h
 *
 * Persistent flash event logger.
 * Events are queued from any context (including ISR) and written to
 * internal flash by a low-priority background task.
 * Each record is 8 bytes: 32-bit timestamp + 5-bit event + 27-bit value.
 *
 * Define ENABLE_FLASH_LOG to activate the logger.
 * When the define is absent every API call and LOG() compiles to nothing,
 * and the background task is never created.
 */

#ifndef INC_FLASHLOG_H_
#define INC_FLASHLOG_H_

#include <stdint.h>

#ifdef ENABLE_FLASH_LOG

#include "FreeRTOS.h"   /* BaseType_t — kept for ISR API compatibility */
#include "portmacro.h"

/* Flash address range used for the log (adjust to match linker script) */
#define FLASHLOG_START_ADDR   0x08020000UL
#define FLASHLOG_END_ADDR     0x0803FFE0UL
#define FLASHLOG_PAGE_SIZE    2048U

/* Log event identifiers */
typedef enum
{
    LOG_DISCOVERY_START   = 1,
    LOG_DISCOVERY_INIT,
    LOG_DISCOVERY_RECOVER,
    LOG_RX_DREQ,
    LOG_RX_BEACON,
    LOG_RX_ACK,
    LOG_RX_TS,
    LOG_TX_DREQ,
    LOG_TX_BEACON,
    LOG_TX_ACK,
    LOG_TX_TS,
    LOG_RESET_CAUSE,
    LOG_DISCOVERY_CMPLT,
    LOG_DEVICE_ENTERING_SLEEP,
    LOG_FRLOG_ERROR,
    LOG_DISCOVERY_COUNT
} FlashLogEvent;

#define LOG(event, value)   FLASHLOG_vWrite(event, value)

/* API */
void FLASHLOG_vInit(void);
void FLASHLOG_vWrite(uint16_t event, uint32_t value);

/*
 * ISR-safe write variant.
 * pxHigherPriorityTaskWoken is accepted for API compatibility with callers
 * that used the native FreeRTOS queue ISR API; it is set to pdFALSE
 * internally since CMSIS v2 osMessageQueuePut handles the yield itself.
 */
void FLASHLOG_vWriteFromISR(uint16_t event, int16_t value,
                             BaseType_t *pxHigherPriorityTaskWoken);

void FLASHLOG_vEncodeRXLogValue(uint32_t *pBuf, uint16_t id, int16_t rssi, uint8_t res);
void FLASHLOG_vDecodeRXLogValue(uint32_t value, uint16_t *id, int16_t *rssi, uint8_t *res);
void FLASHLOG_vDump(void);

#else /* ENABLE_FLASH_LOG not defined — compile everything out */

#define LOG(event, value)                                                do { } while (0)
#define FLASHLOG_vInit()                                                 do { } while (0)
#define FLASHLOG_vDump()                                                 do { } while (0)
#define FLASHLOG_vWrite(event, value)                                    do { } while (0)
#define FLASHLOG_vWriteFromISR(event, value, pWoken)                     do { } while (0)
/* Touch pBuf so the compiler does not warn about the output variable being set but unused */
#define FLASHLOG_vEncodeRXLogValue(pBuf, id, rssi, res)                  do { (void)(pBuf); } while (0)
#define FLASHLOG_vDecodeRXLogValue(value, id, rssi, res)                 do { } while (0)

#endif /* ENABLE_FLASH_LOG */

#endif /* INC_FLASHLOG_H_ */
