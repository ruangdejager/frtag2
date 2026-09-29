/*
 * lorawan_conf.h
 *
 * Build configuration for the ST LoRaWAN middleware
 * (Middlewares/Third_Party/LoRaWAN, STM32CubeWL v1.3.0).
 *
 * Every option here is chosen for RAM first: this part's 64 KB is full to
 * within a few hundred bytes, and the stock ST settings cost ~5.5 KB of
 * .bss. What this build keeps is LoRaWAN 1.0.4 Class A over OTAA in EU868 -
 * the TTN pilot's needs - and nothing else:
 *
 *   - EU868 only. RegionNvm.h's channel table is trimmed to EU868's 16
 *     channels on the strength of that (see the frtag2 note there).
 *   - No Class B, no multicast beyond ST's minimum of 1 context.
 *   - CONTEXT_MANAGEMENT_ENABLED 0: drops the ~1.5 KB NvmBackup copy of the
 *     MAC state. The session is RAM-only, re-joined after every reboot; the
 *     one value that must survive a reboot, the 1.0.4 DevNonce counter, is
 *     persisted separately by LoRaWan.c.
 *   - KEY_EXTRACTABLE 0: the keys are never printed.
 */

#ifndef __LORAWAN_CONF_H__
#define __LORAWAN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "utilities_conf.h"

/* LoRaWAN spec version: 1.0.4 (TTN: "LoRaWAN Specification 1.0.4",
 * "RP002 Regional Parameters 1.0.1"). Its DevNonce is a counter the join
 * server requires to keep increasing, which is why LoRaWan.c persists it. */
#define LORAMAC_SPECIFICATION_VERSION                   0x01000400

#define LORAWAN_DATA_DISTRIB_MGT                        0
#define LORAWAN_KMS                                     0

#define REGION_EU868

#define HYBRID_ENABLED                                  0

#define KEY_EXTRACTABLE                                 0

#define CONTEXT_MANAGEMENT_ENABLED                      0

#define LORAMAC_CLASSB_ENABLED                          0

#define DISABLE_LORAWAN_RX_WINDOW                       0

#ifndef CRITICAL_SECTION_BEGIN
#define CRITICAL_SECTION_BEGIN( )      UTILS_ENTER_CRITICAL_SECTION( )
#endif /* !CRITICAL_SECTION_BEGIN */
#ifndef CRITICAL_SECTION_END
#define CRITICAL_SECTION_END( )        UTILS_EXIT_CRITICAL_SECTION( )
#endif /* !CRITICAL_SECTION_END */

#ifdef __cplusplus
}
#endif

#endif /* __LORAWAN_CONF_H__ */
