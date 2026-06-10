/*
 * DeviceDiscovery.h
 *
 * Application task that orchestrates the LoRa mesh discovery campaign,
 * GPS fix acquisition, and data upload to the logger.
 *
 * Uses CMSIS-RTOS v2 throughout.
 * The wake-up event uses an osEventFlags object (DISCOVERY_WAKEUP_BIT).
 * Time-sync notifications from MeshNetwork arrive as thread flags
 * (DEVICE_DISCOVERY_NOTIFY_TIMESYNC) on the AppTask thread.
 */

#ifndef TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_
#define TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_

#include "cmsis_os2.h"
#include "MeshNetwork.h"

/* ---- Discovery timing ---- */
#define APP_WAKEUP_BUFFER_MS                (5  * 1000)   /* buffer after sync wake-up   */
#define APP_DISCOVERY_WINDOW_TIMEOUT_MS     (180 * 1000)  /* max discovery window        */

/* GPS pre-trigger lead time: how many seconds before each scheduled wake the
 * wake-schedule task asks the GPS module for a fresh fix. The dispatcher runs
 * asynchronously; the AppTask never blocks waiting for it. */
#define DEVICE_DISCOVERY_GPS_PRETRIGGER_S   180U          /* 3 minutes              */

#define LOST_PRIMARY_TIMEOUT_MIN            480           /* ~8 hours before recovery    */

/* osEventFlags bit — set by wakeup-schedule task to start a campaign */
#define DISCOVERY_WAKEUP_BIT                (1UL << 0)

/* osEventFlags bit — set by Movement to bypass deep sleep into FrKernel */
#define DISCOVERY_KERNEL_BIT                (1UL << 1)

/* Thread flag bit — set by MeshNetwork when a TimeSync packet is received */
#define DEVICE_DISCOVERY_NOTIFY_TIMESYNC    (1UL << 0)

/* ---- Logger driver macros ---- */
#define DEVICE_DISCOVERY_DRIVER_bConnectLogger()             FARMRANGER_bDeviceOn()
#define DEVICE_DISCOVERY_DRIVER_vDisconnectLogger()          FARMRANGER_vDeviceOff()
#define DEVICE_DISCOVERY_DRIVER_u64RequestTS()               FARMRANGER_u64RequestTimestamp()
#define DEVICE_DISCOVERY_DRIVER_u8RequestInterval()          FARMRANGER_u8RequestInterval()
#define DEVICE_DISCOVERY_bSendDiscoveryData(items, size)     FARMRANGER_bLogData(items, size)

/* ---- Production sleep state ---- */
typedef enum {
    PRODUCTION_READY  = 0,  /* initial state — normal operation (dev/test) */
    PRODUCTION_SLEEP  = 1,  /* super deep sleep, waiting for solar activation */
    PRODUCTION_ACTIVE = 2,  /* solar-activated, full discovery running */
} ProductionState_e;

/* ---- Public API ---- */
void               DEVICE_DISCOVERY_vInit(void);
void               DEVICE_DISCOVERY_vAppTask(void *pvParameters);
void               DEVICE_DISCOVERY_vConfigDeviceRole(void);
DeviceRole_e       DEVICE_DISCOVERY_eGetDeviceRole(void);
osThreadId_t       DEVICE_DISCOVERY_xGetTaskHandle(void);
void               DEVICE_DISCOVERY_vTriggerKernelWakeup(void);

void               DEVICE_DISCOVERY_vEnterProductionSleep(void);
ProductionState_e  DEVICE_DISCOVERY_eGetProductionState(void);

#endif /* TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_ */
