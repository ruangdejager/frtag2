/*
 * Movement_Config.h
 *
 * Movement algorithm configuration — sensor health thresholds,
 * algorithm buffer sizes and alarm setting defaults.
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_CONFIG_H_
#define WORKER_MOVEMENT_MOVEMENT_CONFIG_H_

/* Sensor health check #1: device ID — consecutive failure alert limit */
#define MOVE_SENSOR_ID_ERR_CNT_ALERT            10

/* Sensor health check #2: no sample — reset / alert thresholds */
#define MOVE_SENSOR_SAMPLE_ERR_CNT_RESET        10
#define MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT        20

/* Sensor health check #3: no delta — reset / alert thresholds (at 25 Hz) */
#define MOVE_SENSOR_DELTA_ERR_CNT_RESET         (25 * 10)
#define MOVE_SENSOR_DELTA_ERR_CNT_ALERT         (25 * 20)

/* Max cumulative sensor health alerts */
#define MOVE_SENSOR_ALERT_COUNT_MAX             3

/* LIS3DH raw axis scale factor (±2g, same resolution as MMA7260) */
#define MOVE_ACC_RAW_SCALE_VALUE                192

/* DC buffer size (HPF window) */
#define MOVE_ALG_DC_BUF_SIZE                    32

/* Envelope buffer size */
#define MOVE_ALG_ENV_BUF_SIZE                   8

/* Accelerometer axis mask limits */
#define MOVE_SENSOR_AXIS_MIN                    0b00000000
#define MOVE_SENSOR_AXIS_DEF                    0b00000010
#define MOVE_SENSOR_AXIS_MAX                    0b00000111

#define MOVE_ALARM_TRACKING_UPDATE_INTERVAL     30

/* Time-of-day active bitmask defaults */
#define MOVE_ALARM_TIME_OF_DAY_ACT_MIN          UINT32_C(0)
#define MOVE_ALARM_TIME_OF_DAY_ACT_DEF          UINT32_C(0x0CCCCCCC)
#define MOVE_ALARM_TIME_OF_DAY_ACT_MAX          UINT32_C(0x0FFFFFFF)

/* Move alarm level settings (min / def / max) */
#define MOVE_ALARM_SETTING_ENABLED_MIN          0
#define MOVE_ALARM_SETTING_ENABLED_DEF          40
#define MOVE_ALARM_SETTING_ENABLED_MAX          254

/* Window A/B times (fixed) */
#define MOVE_ALARM_WINDOW_A_TIME_MIN            300
#define MOVE_ALARM_WINDOW_A_TIME_DEF            300
#define MOVE_ALARM_WINDOW_A_TIME_MAX            300
#define MOVE_ALARM_WINDOW_B_TIME_MIN            10
#define MOVE_ALARM_WINDOW_B_TIME_DEF            10
#define MOVE_ALARM_WINDOW_B_TIME_MAX            10

#define MOVE_HIGH_ACT_ALERTS_MAX_MIN            20
#define MOVE_HIGH_ACT_ALERTS_MAX_DEF            20
#define MOVE_HIGH_ACT_ALERTS_MAX_MAX            20

/* Max level (per time-of-day zone) */
#define MOVE_MAX_LEVEL_MIN                      5
#define MOVE_MAX_LEVEL_DTZ0_DEF                 30
#define MOVE_MAX_LEVEL_DTZ1_DEF                 30
#define MOVE_MAX_LEVEL_DTZ2_DEF                 25
#define MOVE_MAX_LEVEL_DTZ3_DEF                 25
#define MOVE_MAX_LEVEL_MAX                      254

/* Max window */
#define MOVE_MAX_WINDOW_MIN                     1
#define MOVE_MAX_WINDOW_DTZ0_DEF                15
#define MOVE_MAX_WINDOW_DTZ1_DEF                15
#define MOVE_MAX_WINDOW_DTZ2_DEF                15
#define MOVE_MAX_WINDOW_DTZ3_DEF                15
#define MOVE_MAX_WINDOW_MAX                     254

/* Hold-first-call */
#define MOVE_MAX_HOLD_FIRST_MIN                 0
#define MOVE_MAX_HOLD_FIRST_DTZ0_DEF            1
#define MOVE_MAX_HOLD_FIRST_DTZ1_DEF            1
#define MOVE_MAX_HOLD_FIRST_DTZ2_DEF            1
#define MOVE_MAX_HOLD_FIRST_DTZ3_DEF            1
#define MOVE_MAX_HOLD_FIRST_MAX                 1

/* Hold-second-call */
#define MOVE_MAX_HOLD_SECOND_MIN                0
#define MOVE_MAX_HOLD_SECOND_DTZ0_DEF           1
#define MOVE_MAX_HOLD_SECOND_DTZ1_DEF           1
#define MOVE_MAX_HOLD_SECOND_DTZ2_DEF           1
#define MOVE_MAX_HOLD_SECOND_DTZ3_DEF           0
#define MOVE_MAX_HOLD_SECOND_MAX                1

/* Min level */
#define MOVE_MIN_LEVEL_MIN                      0
#define MOVE_MIN_LEVEL_DTZ0_DEF                 3
#define MOVE_MIN_LEVEL_DTZ1_DEF                 3
#define MOVE_MIN_LEVEL_DTZ2_DEF                 3
#define MOVE_MIN_LEVEL_DTZ3_DEF                 3
#define MOVE_MIN_LEVEL_MAX                      0b00111111   /* 63 */

/* Min window */
#define MOVE_MIN_WINDOW_MIN                     1
#define MOVE_MIN_WINDOW_DTZ0_DEF                60
#define MOVE_MIN_WINDOW_DTZ1_DEF                60
#define MOVE_MIN_WINDOW_DTZ2_DEF                60
#define MOVE_MIN_WINDOW_DTZ3_DEF                60
#define MOVE_MIN_WINDOW_MAX                     254

/* RTOS state machine — STILL detection */
#define MOVE_STILL_TIMEOUT_S        300U   /* 5 min of envelope ≤ threshold → STILL */
#define MOVE_STILL_ENV_THRESHOLD      3U   /* envelope ≤ this counts as no motion   */

/* Gravity sequence detector */
#define MOVE_SEQ_GRAV_TH              7    /* |dcLevel| > this → axis carries gravity  */
#define MOVE_SEQ_GRAV_NULL            4    /* |dcLevel| < this → axis is free          */
#define MOVE_SEQ_HOLD_TICKS           2U   /* consecutive 1 Hz ticks to accept a step  */

#endif /* WORKER_MOVEMENT_MOVEMENT_CONFIG_H_ */
