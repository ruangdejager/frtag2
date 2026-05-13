/*
 * dbg_log.h
 *
 * Debug logging macro.
 * Route depends on compile-time flags:
 *   LISTENER_MODE    — forwards output via FARMRANGER UART
 *   ENABLE_DBG_UART  — outputs via debug UART (USART1)
 *   (neither)        — all DBG calls expand to nothing
 */

#ifndef DBG_LOG_H_
#define DBG_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "debug_uart_output.h"

#ifdef LISTENER_MODE
#  define DBG(x, ...)   LISTENER_vPut(x, ##__VA_ARGS__)
#elif defined(ENABLE_DBG_UART)
#  define DBG(x, ...)   TERM_vPut(x, ##__VA_ARGS__)
#else
#  define DBG(x, ...)
#endif

#endif /* DBG_LOG_H_ */
