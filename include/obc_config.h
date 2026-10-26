#ifndef OBC_CONFIG_H
#define OBC_CONFIG_H

/* Network configuration. */
#define WIFI_SSID               "CUBESAT-GROUND-STATION"
#define WIFI_PASSWORD           "12345678"
#define GS_IP                   "192.168.4.1"
#define UDP_PORT_GS             5000
#define UDP_PORT_PICO           5001
#define WIFI_RETRY_MS           5000
#define WIFI_BOOT_TIMEOUT_MS    15000

/* MPU6050 configuration. */
#define PIN_SDA                 16
#define PIN_SCL                 17
#define MPU6050_ADDR            0x68
#define I2C_TIMEOUT_US          5000
#define GYRO_SCALE              65.5f

/* FDIR configuration. */
#define FDIR_TEMP_LIMIT_C       65.0f
#define FDIR_GYRO_LIMIT_DPS     250.0f
#define IMU_FAIL_LIMIT          10
/* Three consecutive 50 ms FSM cycles must violate a limit before SAFE mode. */
#define FDIR_DEBOUNCE_SAMPLES   3

/* Task periods and watchdog configuration. */
#define IMU_PERIOD_MS           50
#define FSM_PERIOD_MS           50
#define TM_PERIOD_NOMINAL_MS    50
#define TM_PERIOD_SAFE_MS       500
#define NET_PERIOD_MS           500
#define WDG_WINDOW_MS           1000
#define WDT_TIMEOUT_MS          4000
#define DETUMBLE_DURATION_MS    5000

/* FreeRTOS task priorities (tskIDLE_PRIORITY is zero). */
#define PRIO_WDG                1
#define PRIO_NET                1
#define PRIO_TM                 2
#define PRIO_TC                 2
#define PRIO_IMU                4
#define PRIO_FSM                4

/* FreeRTOS task stack sizes, in words. */
#define STACK_INIT              2048
#define STACK_IMU               768
#define STACK_FSM               1024
#define STACK_TM                1024
#define STACK_TC                1024
#define STACK_WDG               768

/* Heartbeat bits: every supervised task sets its bit once per cycle. */
#define HB_IMU                  (1u << 0)
#define HB_FSM                  (1u << 1)
#define HB_TM                   (1u << 2)
#define HB_TC                   (1u << 3)
#define HB_NET                  (1u << 4)
#define HB_ALL                  (HB_IMU | HB_FSM | HB_TM | HB_TC | HB_NET)

#define NUM_TRACKED_TASKS       6

#endif /* OBC_CONFIG_H */
