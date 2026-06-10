/*
 * dbg_log.h
 *
 * Forwarding shim — DBG(), LOG(), and DBG_LOG() are now defined in DbgLog.h.
 * Existing callers of #include "dbg_log.h" require no changes.
 */

#ifndef DBG_LOG_H_
#define DBG_LOG_H_
#include "DbgLog.h"
#endif /* DBG_LOG_H_ */
