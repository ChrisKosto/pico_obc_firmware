#ifndef OBC_TYPES_H
#define OBC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FSM_MODE_BOOT      = 0,
    FSM_MODE_DETUMBLE  = 1,
    FSM_MODE_NOMINAL   = 2,
    FSM_MODE_SAFE      = 3
} flight_mode_t;

typedef enum {
    TC_NONE        = 0,
    TC_SET_NOMINAL = 1,
    TC_SET_SAFE    = 2
} tc_cmd_t;

/* FDIR alarm values reported in telemetry. */
#define FDIR_OK                 0
#define FDIR_OVERHEAT           1
#define FDIR_TUMBLING           2
#define FDIR_IMU_FAULT          3

/* Boot / reset cause reported in telemetry. */
#define BOOT_POWER_ON           0
#define BOOT_WDT_TIMEOUT        1
#define BOOT_COMMANDED          2
#define BOOT_STACK_OVERFLOW     3
#define BOOT_HEAP_EXHAUSTED     4
#define BOOT_FATAL_INIT         5

/* Telecommand acknowledgement status. */
#define TC_ACK_ACCEPTED         0
#define TC_ACK_REJ_UNKNOWN      1
#define TC_ACK_REJ_MALFORMED    2
#define TC_ACK_REJ_QUEUE_FULL   3

#pragma pack(push, 1)
typedef struct {
    uint16_t packet_id;
    uint16_t packet_seq_ctrl;
    uint16_t packet_data_len;
} ccsds_primary_header_t;

typedef struct {
    ccsds_primary_header_t header;
    uint32_t uptime_ms;
    int16_t  mpu_temp_centi_c;
    int16_t  mcu_temp_centi_c;
    int16_t  roll_centi_deg;
    int16_t  pitch_centi_deg;
    int16_t  yaw_centi_deg;
    uint8_t  flight_mode;
    uint8_t  boot_cause;
    uint8_t  fdir_alarm;
    uint16_t crc16;
} ccsds_telemetry_packet_t;
#pragma pack(pop)

typedef struct {
    float         mpu_temp_c;
    float         mcu_temp_c;
    float         roll;
    float         pitch;
    float         yaw;
    float         gz_dps;
    bool          imu_ok;
    uint8_t       imu_fail_count;
    flight_mode_t mode;
    uint8_t       fdir_alarm;
} obc_state_t;

#endif /* OBC_TYPES_H */
