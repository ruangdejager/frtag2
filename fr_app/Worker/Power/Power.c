/*
 * Power.c
 *
 * Power class management — battery-voltage driven normal/recovery switching.
 *
 * Power state is stored in an osEventFlags object so that any task can
 * query (POWER_tGetState) or block on a class (POWER_vWaitForClass).
 *
 * The PowerStateManager task only runs on secondary devices; it subscribes
 * to the 1-second platform heartbeat and adjusts the power class based on
 * the measured battery voltage.
 *
 * Uses CMSIS-RTOS v2 throughout.
 */

#include "Power.h"
#include "Power_Config.h"
#include "platform.h"
#include "Battery.h"
#include "DeviceDiscovery.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE */
#include "task.h"

#include <limits.h>

/* ---- CMSIS-RTOS v2 objects ---- */
static osEventFlagsId_t gPowerEvents           = NULL;
static osThreadId_t     xPowerStateManagerTaskHandle = NULL;

/* ---- Forward declarations ---- */
static void POWER_vStateManagerTask(void *arg);

/* --------------------------------------------------------------------------
 * POWER_vInit
 * -------------------------------------------------------------------------- */
void POWER_vInit(void)
{
    gPowerEvents = osEventFlagsNew(NULL);
    configASSERT(gPowerEvents != NULL);

    POWER_vSetModeNormal();

    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        static const osThreadAttr_t mgr_attr = {
            .name       = "PowerStateMgr",
            .stack_size = configMINIMAL_STACK_SIZE * sizeof(StackType_t),
            .priority   = osPriorityNormal,
        };
        xPowerStateManagerTaskHandle =
            osThreadNew(POWER_vStateManagerTask, NULL, &mgr_attr);
        configASSERT(xPowerStateManagerTaskHandle != NULL);
    }
}

/* --------------------------------------------------------------------------
 * POWER_vSetModeNormal
 * -------------------------------------------------------------------------- */
void POWER_vSetModeNormal(void)
{
    osEventFlagsSet(gPowerEvents,   POWER_CLASS_NORMAL | POWER_CLASS_ALWAYS);
    osEventFlagsClear(gPowerEvents, POWER_CLASS_LOW | POWER_CLASS_RECOVERY);
}

/* --------------------------------------------------------------------------
 * POWER_vSetModeLow
 * -------------------------------------------------------------------------- */
void POWER_vSetModeLow(void)
{
    osEventFlagsSet(gPowerEvents,   POWER_CLASS_LOW | POWER_CLASS_ALWAYS);
    osEventFlagsClear(gPowerEvents, POWER_CLASS_NORMAL | POWER_CLASS_RECOVERY);
}

/* --------------------------------------------------------------------------
 * POWER_vSetModeRecovery
 * -------------------------------------------------------------------------- */
void POWER_vSetModeRecovery(void)
{
    osEventFlagsSet(gPowerEvents,   POWER_CLASS_RECOVERY | POWER_CLASS_ALWAYS);
    osEventFlagsClear(gPowerEvents, POWER_CLASS_NORMAL | POWER_CLASS_LOW);
}

/* --------------------------------------------------------------------------
 * POWER_vWaitForClass — block until the requested power class is active
 * -------------------------------------------------------------------------- */
void POWER_vWaitForClass(uint32_t classMask)
{
    osEventFlagsWait(gPowerEvents,
                     classMask,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
}

/* --------------------------------------------------------------------------
 * POWER_tGetState — return current power class flags (non-blocking)
 * -------------------------------------------------------------------------- */
uint32_t POWER_tGetState(void)
{
    return osEventFlagsGet(gPowerEvents);
}

/* --------------------------------------------------------------------------
 * POWER_vStateManagerTask — heartbeat-driven voltage check (secondary only)
 *
 * Three-class state machine driven by battery voltage:
 *
 *   NORMAL   ──(V ≤ ENTER_LOW_MV)──►  LOW       ──(V ≤ ENTER_RECOVERY_MV)──►  RECOVERY
 *   NORMAL  ◄──(V ≥ EXIT_LOW_MV) ──   LOW      ◄──(V ≥ EXIT_RECOVERY_MV) ──   RECOVERY
 *
 * Hysteresis windows prevent oscillation:
 *   LOW ↔ NORMAL boundary  : 20 mV (3500 → 3520)
 *   LOW ↔ RECOVERY boundary: 50 mV (3400 → 3450)
 * -------------------------------------------------------------------------- */
static void POWER_vStateManagerTask(void *arg)
{
    (void)arg;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        uint16_t u16Bat   = BAT_u16GetVoltage();
        uint32_t u32State = POWER_tGetState();

        if (u32State & POWER_CLASS_NORMAL)
        {
            /* From NORMAL, drop to LOW when at/below ENTER_LOW_MV.
             * A direct NORMAL→RECOVERY transition is also covered: LOW will
             * collapse to RECOVERY on the next tick if voltage stays low. */
            if (u16Bat <= ENTER_LOW_MV)
                POWER_vSetModeLow();
        }
        else if (u32State & POWER_CLASS_LOW)
        {
            if (u16Bat <= ENTER_RECOVERY_MV)
                POWER_vSetModeRecovery();
            else if (u16Bat >= EXIT_LOW_MV)
                POWER_vSetModeNormal();
        }
        else if (u32State & POWER_CLASS_RECOVERY)
        {
            /* RECOVERY only releases up to LOW; LOW→NORMAL is a separate
             * step that requires another EXIT_LOW_MV crossing. This makes
             * the climb out of a deep brown-out conservative. */
            if (u16Bat >= EXIT_RECOVERY_MV)
                POWER_vSetModeLow();
        }
    }
}
