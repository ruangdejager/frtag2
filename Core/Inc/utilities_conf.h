/**
 ******************************************************************************
 * @file    utilities_conf.h
 * @brief   Configuration for ST Utilities (stm32_mem, critical sections).
 *          Required by Utilities/misc/stm32_mem.h which is pulled in by the
 *          SubGHz_Phy radio driver chain via radio_conf.h.
 ******************************************************************************
 */

#ifndef UTILITIES_CONF_H
#define UTILITIES_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_compiler.h"
#include <stdio.h>
#include <string.h>

/******************************************************************************
 * Common critical section helpers (used by RADIO_CONF CRITICAL_SECTION macros)
 ******************************************************************************/
#define UTILS_ENTER_CRITICAL_SECTION( )  uint32_t primask_bit = __get_PRIMASK(); \
                                         __disable_irq()

#define UTILS_EXIT_CRITICAL_SECTION( )   __set_PRIMASK( primask_bit )

#define UTILS_MEMSET8(dest, value, size) memset((dest),(value),(size))

/******************************************************************************
 * UTIL_ADV_TRACE configuration (required by Utilities/trace/adv_trace)
 ******************************************************************************/
#define UTIL_ADV_TRACE_FIFO_SIZE                    512U
#define UTIL_ADV_TRACE_TMP_BUF_SIZE                 256U
#define UTIL_ADV_TRACE_TMP_MAX_TIMESTMAP_SIZE       15U

#define UTIL_ADV_TRACE_MEMSET8(dest, value, size)   memset((dest), (value), (size))
#define UTIL_ADV_TRACE_VSNPRINTF(...)               vsnprintf(__VA_ARGS__)

#define UTIL_ADV_TRACE_INIT_CRITICAL_SECTION( )
#define UTIL_ADV_TRACE_ENTER_CRITICAL_SECTION( )    UTILS_ENTER_CRITICAL_SECTION( )
#define UTIL_ADV_TRACE_EXIT_CRITICAL_SECTION( )     UTILS_EXIT_CRITICAL_SECTION( )

#ifdef __cplusplus
}
#endif

#endif /* UTILITIES_CONF_H */
