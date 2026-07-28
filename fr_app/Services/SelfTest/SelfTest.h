/*
 * SelfTest.h
 *
 * Boot-time peripheral smoke tests. Called once from INIT_vInitialization
 * after all normal bring-up is done. Each check is memoized; a failure is
 * indicated live on the yellow LED (see SELFTEST_vRunAndReport) and can
 * also be queried later over FrKernel ("tag <ID> selftest [gps|acc|flash]").
 *
 * Never fatal: init always proceeds to the normal run state whatever the
 * test outcome. The point is a visual "this unit has a hardware problem"
 * signal at the bench, not a boot barrier.
 */

#ifndef SERVICES_SELFTEST_SELFTEST_H_
#define SERVICES_SELFTEST_SELFTEST_H_

#include <stdbool.h>

/* Yellow-LED flash codes. Ordering here is also the order used when
 * multiple failures are reported one after the other. */
typedef enum {
    SELFTEST_ERR_GPS   = 1,   /* secondary only; not run on primary  */
    SELFTEST_ERR_ACC   = 2,
    SELFTEST_ERR_FLASH = 3,   /* not run under STORAGE_BACKEND_MICROSD */
} SelfTestErr_e;

/* Runs the three checks, records each outcome, and if any failed flashes
 * the yellow LED with the code sequence:
 *   for each failed test (in the SelfTestErr_e order above):
 *     flash the code N times, pause 2 s, flash again, pause 2 s, flash a
 *     third time. If more than one test failed, pause 3 s between codes.
 *   Yellow LED off at the end.
 * Blocking; returns once the sequence is done. */
void SELFTEST_vRunAndReport(void);

/* Query API for FrKernel — read the memoized result. A "not applicable"
 * test (GPS on primary, flash on MicroSD build) counts as OK so callers
 * can decide with the *Applicable() pair. */
bool SELFTEST_bGpsOk(void);
bool SELFTEST_bAccOk(void);
bool SELFTEST_bFlashOk(void);

bool SELFTEST_bGpsApplicable(void);
bool SELFTEST_bFlashApplicable(void);

#endif /* SERVICES_SELFTEST_SELFTEST_H_ */
