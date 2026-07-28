/*
 * SelfTest.c — see SelfTest.h for the contract.
 */

#include "SelfTest.h"

#include "build_config.h"
#include "cmsis_os2.h"
#include "hal_bsp.h"
#include "dbg_log.h"

#include "DeviceDiscovery.h"       /* DEVICE_DISCOVERY_eGetDeviceRole */
#include "Acc.h"                   /* ACC_bDeviceIdOk                */
#include "Gps.h"                   /* GPS_bSelfTest                  */

#ifdef STORAGE_BACKEND_FLASH
#  include "Flash.h"               /* FLASH_bVerifyDevice            */
#endif

/* Result flags default to true so a not-applicable test naturally reports
 * OK (e.g. GPS on primary, flash under MicroSD backend). */
static bool bGpsOk       = true;
static bool bAccOk       = true;
static bool bFlashOk     = true;

static bool bGpsRan      = false;   /* whether the test actually executed */
static bool bFlashRan    = false;

/* --- LED flash sequence ---------------------------------------------------
 * User spec: for each failed test, flash the code (N short flashes) three
 * times with a 2 s gap between repeats. If more than one test failed, put
 * a 3 s gap between codes. Yellow LED off at the end so the existing
 * BSP_LED_Off(LED_YELLOW) at the init tail has nothing left to clean up.
 * -------------------------------------------------------------------------- */
#define SELFTEST_FLASH_ON_MS       200U
#define SELFTEST_FLASH_OFF_MS      200U
#define SELFTEST_GAP_REPEAT_MS     2000U
#define SELFTEST_GAP_BETWEEN_MS    3000U
#define SELFTEST_REPEATS           3U

static void selftest_vFlashCode(uint8_t u8Code)
{
    for (uint8_t r = 0; r < SELFTEST_REPEATS; r++)
    {
        for (uint8_t i = 0; i < u8Code; i++)
        {
            BSP_LED_On(LED_YELLOW);
            osDelay(SELFTEST_FLASH_ON_MS);
            BSP_LED_Off(LED_YELLOW);
            osDelay(SELFTEST_FLASH_OFF_MS);
        }
        if (r + 1U < SELFTEST_REPEATS)
            osDelay(SELFTEST_GAP_REPEAT_MS);
    }
}

void SELFTEST_vRunAndReport(void)
{
    /* GPS — secondary only. Primary has no GPS fitted; leave bGpsOk=true
     * (its default) so query cmds report the (n/a) OK state cleanly. */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        bGpsRan = true;
        bGpsOk  = GPS_bSelfTest(1500U);   /* > one 1 Hz NMEA epoch */
    }

    /* Accelerometer — fitted on both roles; WHO_AM_I via the existing
     * ACC_bDeviceIdOk() helper. */
    bAccOk = ACC_bDeviceIdOk();

    /* Ext flash — JEDEC-ID via the existing FLASH_bVerifyDevice helper.
     * Compiled out entirely when the storage backend is MicroSD. */
#ifdef STORAGE_BACKEND_FLASH
    bFlashRan = true;
    bFlashOk  = FLASH_bVerifyDevice();
#endif

    DBG_LOG("SelfTest: gps=%s acc=%s flash=%s\r\n",
            bGpsRan   ? (bGpsOk   ? "OK" : "FAIL") : "n/a",
            bAccOk    ? "OK" : "FAIL",
            bFlashRan ? (bFlashOk ? "OK" : "FAIL") : "n/a");

    /* Collect the failure codes in enum order so a multi-failure sequence
     * is deterministic. */
    uint8_t au8Codes[3];
    uint8_t u8Count = 0U;
    if (bGpsRan   && !bGpsOk)   au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_GPS;
    if              (!bAccOk)   au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_ACC;
    if (bFlashRan && !bFlashOk) au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_FLASH;

    for (uint8_t i = 0; i < u8Count; i++)
    {
        selftest_vFlashCode(au8Codes[i]);
        if (i + 1U < u8Count)
            osDelay(SELFTEST_GAP_BETWEEN_MS);
    }

    BSP_LED_Off(LED_YELLOW);
}

/* --- Query API ------------------------------------------------------------ */

bool SELFTEST_bGpsOk(void)   { return bGpsOk;   }
bool SELFTEST_bAccOk(void)   { return bAccOk;   }
bool SELFTEST_bFlashOk(void) { return bFlashOk; }

bool SELFTEST_bGpsApplicable(void)
{
    return DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY;
}

bool SELFTEST_bFlashApplicable(void)
{
#ifdef STORAGE_BACKEND_FLASH
    return true;
#else
    return false;
#endif
}
