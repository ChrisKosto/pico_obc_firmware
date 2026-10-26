/*
 * pico_obc_firmware.c
 * Target: Raspberry Pi Pico 2 W (RP2350, Arm Cortex-M33) + FreeRTOS
 *
 * Satellite On-Board Computer (OBC) flight software for the CubeSat FlatSat.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "tc_parser.h"

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */
#define WIFI_SSID               "CUBESAT-GROUND-STATION"
#define WIFI_PASSWORD           "12345678"
#define GS_IP                   "192.168.4.1"
#define UDP_PORT_GS             5000
#define UDP_PORT_PICO           5001
#define WIFI_RETRY_MS           5000
#define WIFI_BOOT_TIMEOUT_MS    15000

#define I2C_PORT                i2c0
#define PIN_SDA                 16
#define PIN_SCL                 17
#define MPU6050_ADDR            0x68
#define I2C_TIMEOUT_US          5000

#define REG_PWR_MGMT_1          0x6B
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_XOUT          0x3B

#define GYRO_FS_500             0x08
#define GYRO_SCALE              65.5f

#define FDIR_TEMP_LIMIT_C       65.0f
#define FDIR_GYRO_LIMIT_DPS     250.0f
#define IMU_FAIL_LIMIT          10
/* A limit must be exceeded for this many consecutive FSM cycles (3 x 50 ms)
 * before the FDIR reacts, so a single noisy sample cannot trigger SAFE mode. */
#define FDIR_DEBOUNCE_SAMPLES   3

#define FDIR_OK                 0
#define FDIR_OVERHEAT           1
#define FDIR_TUMBLING           2
#define FDIR_IMU_FAULT          3

#define IMU_PERIOD_MS           50
#define FSM_PERIOD_MS           50
#define TM_PERIOD_NOMINAL_MS    50
#define TM_PERIOD_SAFE_MS       500
#define NET_PERIOD_MS           500
#define WDG_WINDOW_MS           1000
#define WDT_TIMEOUT_MS          4000
#define DETUMBLE_DURATION_MS    5000

#define PRIO_WDG                (tskIDLE_PRIORITY + 1)
#define PRIO_NET                (tskIDLE_PRIORITY + 1)
#define PRIO_TM                 (tskIDLE_PRIORITY + 2)
#define PRIO_TC                 (tskIDLE_PRIORITY + 2)
#define PRIO_IMU                (tskIDLE_PRIORITY + 4)
#define PRIO_FSM                (tskIDLE_PRIORITY + 4)

#define STACK_INIT              2048
#define STACK_IMU               768
#define STACK_FSM               1024   /* extra room for the mode-change printf */
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

/* Boot / reset cause reported in telemetry (field boot_cause). */
#define BOOT_POWER_ON           0   /* cold start                                  */
#define BOOT_WDT_TIMEOUT        1   /* hardware watchdog expired (hung task, ...)  */
#define BOOT_COMMANDED          2   /* telecommand $TC:99,01                       */
#define BOOT_STACK_OVERFLOW     3   /* FreeRTOS stack overflow hook                */
#define BOOT_HEAP_EXHAUSTED     4   /* FreeRTOS malloc-failed hook                 */
#define BOOT_FATAL_INIT         5   /* unrecoverable error during initialisation   */

/* The reset reason is kept in a watchdog scratch register, which survives a
 * watchdog reset (but not a power cycle). Upper 16 bits = magic, lower = cause. */
#define RESET_REASON_MAGIC      0xC0DE0000u
#define RESET_REASON_MASK       0xFFFF0000u

/* Telecommand acknowledgement status (sent as "$TM:ACK,<id>,<val>,<status>"). */
#define TC_ACK_ACCEPTED         0
#define TC_ACK_REJ_UNKNOWN      1
#define TC_ACK_REJ_MALFORMED    2
#define TC_ACK_REJ_QUEUE_FULL   3

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Globals                                                             */
/* ------------------------------------------------------------------ */
static obc_state_t        g_state;
static SemaphoreHandle_t  g_state_mutex;
static QueueHandle_t      g_tc_queue;
static EventGroupHandle_t g_hb;
static int                g_sock = -1;
static float              g_gz_offset = 0.0f;
static uint8_t            g_boot_cause = BOOT_POWER_ON;
static volatile bool      g_reboot_commanded = false;

#define NUM_TRACKED_TASKS 6
static TaskHandle_t  g_task_handles[NUM_TRACKED_TASKS];
static const char   *g_task_names[NUM_TRACKED_TASKS] = { "imu", "fsm", "tm", "tc", "wdg", "net" };

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint16_t calc_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int16_t to_centi(float v) {
    float s = v * 100.0f;
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
}

static inline void state_lock(void)   { xSemaphoreTake(g_state_mutex, portMAX_DELAY); }
static inline void state_unlock(void) { xSemaphoreGive(g_state_mutex); }

static obc_state_t state_snapshot(void) {
    obc_state_t s;
    state_lock();
    s = g_state;
    state_unlock();
    return s;
}

static float read_pico_temperature(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

static bool wifi_link_up(void) {
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

/* ------------------------------------------------------------------ */
/* Reset-cause bookkeeping and fatal-error handling                    */
/* ------------------------------------------------------------------ */
static void record_reset_reason(uint8_t cause) {
    watchdog_hw->scratch[0] = RESET_REASON_MAGIC | cause;
}

/* Determine why we booted, then clear the scratch register. */
static uint8_t fetch_boot_cause(void) {
    uint32_t scratch = watchdog_hw->scratch[0];
    watchdog_hw->scratch[0] = 0;

    if (!watchdog_caused_reboot()) {
        return BOOT_POWER_ON;
    }
    if ((scratch & RESET_REASON_MASK) == RESET_REASON_MAGIC) {
        return (uint8_t)(scratch & 0xFFu);
    }
    return BOOT_WDT_TIMEOUT;   /* watchdog expired without a recorded reason */
}

static void obc_fatal_reboot(uint8_t cause, const char *reason) {
    printf("[FATAL] %s -> rebooting\n", reason);
    record_reset_reason(cause);
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    record_reset_reason(BOOT_STACK_OVERFLOW);
    printf("[FATAL] stack overflow in task '%s'\n", pcTaskName);
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

void vApplicationMallocFailedHook(void) {
    obc_fatal_reboot(BOOT_HEAP_EXHAUSTED, "FreeRTOS heap exhausted");
}

/* ------------------------------------------------------------------ */
/* Downlink                                                            */
/* ------------------------------------------------------------------ */
static void udp_send_to_gs(const char *msg, size_t len) {
    if (g_sock < 0 || !wifi_link_up()) return;

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_len    = sizeof(dst);
    dst.sin_family = AF_INET;
    dst.sin_port   = lwip_htons(UDP_PORT_GS);
    dst.sin_addr.s_addr = ipaddr_addr(GS_IP);

    lwip_sendto(g_sock, msg, len, 0, (struct sockaddr *)&dst, sizeof(dst));
}

/* Telecommand acceptance report. id/val are reported as "--" when the
 * command could not be parsed. */
static void send_tc_ack(bool have_cmd, uint8_t id, uint8_t val, uint8_t status) {
    char msg[40];
    int n;
    if (have_cmd) {
        n = snprintf(msg, sizeof(msg), "$TM:ACK,%u,%u,%u\n",
                     (unsigned)id, (unsigned)val, (unsigned)status);
    } else {
        n = snprintf(msg, sizeof(msg), "$TM:ACK,--,--,%u\n", (unsigned)status);
    }
    if (n > 0 && (size_t)n < sizeof(msg)) {
        udp_send_to_gs(msg, (size_t)n);
    }
}

/* ------------------------------------------------------------------ */
/* MPU6050 driver                                                      */
/* ------------------------------------------------------------------ */
static bool mpu6050_init(void) {
    uint8_t wake[2] = { REG_PWR_MGMT_1, 0x00 };
    if (i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, wake, 2, false, I2C_TIMEOUT_US) < 0)
        return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t cfg[2] = { REG_GYRO_CONFIG, GYRO_FS_500 };
    return i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, cfg, 2, false, I2C_TIMEOUT_US) >= 0;
}

static bool mpu_read_burst(uint8_t *data) {
    uint8_t reg = REG_ACCEL_XOUT;
    if (i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, &reg, 1, true, I2C_TIMEOUT_US) < 0)
        return false;
    return i2c_read_timeout_us(I2C_PORT, MPU6050_ADDR, data, 14, false, I2C_TIMEOUT_US) == 14;
}

static void calibrate_gyro(void) {
    float sum_gz = 0.0f;
    int valid = 0;
    for (int i = 0; i < 150; i++) {
        uint8_t d[14];
        if (mpu_read_burst(d)) {
            int16_t gz = (int16_t)((d[12] << 8) | d[13]);
            sum_gz += (float)gz / GYRO_SCALE;
            valid++;
        }
        watchdog_update();   /* boot sequence: the init task feeds the watchdog */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    g_gz_offset = (valid > 0) ? (sum_gz / (float)valid) : 0.0f;
}

/* ------------------------------------------------------------------ */
/* Tasks                                                               */
/* ------------------------------------------------------------------ */
static void imu_task(void *arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = IMU_PERIOD_MS / 1000.0f;
    float yaw = 0.0f;
    uint32_t fails = 0;

    for (;;) {
        uint8_t d[14];
        if (mpu_read_burst(d)) {
            fails = 0;

            int16_t ax = (int16_t)((d[0]  << 8) | d[1]);
            int16_t ay = (int16_t)((d[2]  << 8) | d[3]);
            int16_t az = (int16_t)((d[4]  << 8) | d[5]);
            int16_t rt = (int16_t)((d[6]  << 8) | d[7]);
            int16_t gz = (int16_t)((d[12] << 8) | d[13]);

            float mpu_temp = (rt / 340.0f) + 36.53f;
            float roll  = atan2f((float)ay, (float)az) * 57.2957795f;
            float pitch = atan2f(-(float)ax, sqrtf((float)ay * (float)ay + (float)az * (float)az)) * 57.2957795f;

            float gz_dps = ((float)gz / GYRO_SCALE) - g_gz_offset;
            if (fabsf(gz_dps) > 0.5f) {
                yaw += gz_dps * dt;
                if (yaw > 180.0f)       yaw -= 360.0f;
                else if (yaw < -180.0f) yaw += 360.0f;
            }

            state_lock();
            g_state.mpu_temp_c     = mpu_temp;
            g_state.roll           = roll;
            g_state.pitch          = pitch;
            g_state.yaw            = yaw;
            g_state.gz_dps         = gz_dps;
            g_state.imu_ok         = true;
            g_state.imu_fail_count = 0;
            state_unlock();
        } else {
            fails++;
            if ((fails % IMU_FAIL_LIMIT) == 0) {
                (void)mpu6050_init();
            }
            state_lock();
            g_state.imu_ok         = false;
            g_state.imu_fail_count = (fails > 255) ? 255 : (uint8_t)fails;
            state_unlock();
        }

        xEventGroupSetBits(g_hb, HB_IMU);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/* Flight-mode state machine + FDIR. Single owner of g_state.mode. */
static void fsm_task(void *arg) {
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const uint32_t detumble_start = to_ms_since_boot(get_absolute_time());
    uint8_t overheat_cnt = 0;
    uint8_t tumble_cnt   = 0;
    flight_mode_t prev_mode  = FSM_MODE_BOOT;
    uint8_t       prev_alarm = FDIR_OK;

    for (;;) {
        tc_cmd_t cmd;
        while (xQueueReceive(g_tc_queue, &cmd, 0) == pdTRUE) {
            state_lock();
            if (cmd == TC_SET_NOMINAL) {
                g_state.mode = FSM_MODE_NOMINAL;
                g_state.fdir_alarm = FDIR_OK;
            } else if (cmd == TC_SET_SAFE) {
                g_state.mode = FSM_MODE_SAFE;
            }
            state_unlock();
        }

        float mcu_temp = read_pico_temperature();
        uint32_t now = to_ms_since_boot(get_absolute_time());

        state_lock();
        g_state.mcu_temp_c = mcu_temp;

        if (g_state.mode == FSM_MODE_DETUMBLE &&
            (now - detumble_start) > DETUMBLE_DURATION_MS) {
            g_state.mode = FSM_MODE_NOMINAL;
        }

        /* Debounced limit checks: count consecutive violations. */
        const bool overheat = (mcu_temp > FDIR_TEMP_LIMIT_C) ||
                              (g_state.imu_ok && g_state.mpu_temp_c > FDIR_TEMP_LIMIT_C);
        const bool tumbling = g_state.imu_ok && fabsf(g_state.gz_dps) > FDIR_GYRO_LIMIT_DPS;

        overheat_cnt = overheat ? (uint8_t)((overheat_cnt < 255) ? overheat_cnt + 1 : 255) : 0;
        tumble_cnt   = tumbling ? (uint8_t)((tumble_cnt   < 255) ? tumble_cnt   + 1 : 255) : 0;

        if (overheat_cnt >= FDIR_DEBOUNCE_SAMPLES) {
            g_state.mode = FSM_MODE_SAFE;
            g_state.fdir_alarm = FDIR_OVERHEAT;
        } else if (tumble_cnt >= FDIR_DEBOUNCE_SAMPLES) {
            g_state.mode = FSM_MODE_SAFE;
            g_state.fdir_alarm = FDIR_TUMBLING;
        } else if (g_state.imu_fail_count >= IMU_FAIL_LIMIT) {
            /* already a debounced condition: 10 consecutive failed reads */
            g_state.mode = FSM_MODE_SAFE;
            g_state.fdir_alarm = FDIR_IMU_FAULT;
        }
        const flight_mode_t cur_mode  = g_state.mode;
        const uint8_t       cur_alarm = g_state.fdir_alarm;
        const float         cur_gz    = g_state.gz_dps;
        state_unlock();

        if (cur_mode != prev_mode || cur_alarm != prev_alarm) {
            printf("[FSM] mode %u -> %u, alarm %u -> %u (mcu %.1fC, gz %.1f dps)\n",
                   (unsigned)prev_mode, (unsigned)cur_mode,
                   (unsigned)prev_alarm, (unsigned)cur_alarm, (double)mcu_temp, (double)cur_gz);
            prev_mode  = cur_mode;
            prev_alarm = cur_alarm;
        }

        xEventGroupSetBits(g_hb, HB_FSM);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

/* Builds CCSDS telemetry packets and sends them as "$CCSDS:<hex>\n". */
static void tm_task(void *arg) {
    (void)arg;
    static const char HEXCH[] = "0123456789ABCDEF";
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t seq = 0;

    for (;;) {
        obc_state_t s = state_snapshot();

        ccsds_telemetry_packet_t pkt;
        pkt.header.packet_id       = 0x0801;
        pkt.header.packet_seq_ctrl = (uint16_t)(0xC000 | (seq & 0x3FFF));
        pkt.header.packet_data_len = (uint16_t)(sizeof(pkt) - sizeof(pkt.header) - 1);
        pkt.uptime_ms        = to_ms_since_boot(get_absolute_time());
        pkt.mpu_temp_centi_c = to_centi(s.mpu_temp_c);
        pkt.mcu_temp_centi_c = to_centi(s.mcu_temp_c);
        pkt.roll_centi_deg   = to_centi(s.roll);
        pkt.pitch_centi_deg  = to_centi(s.pitch);
        pkt.yaw_centi_deg    = to_centi(s.yaw);
        pkt.flight_mode      = (uint8_t)s.mode;
        pkt.boot_cause       = g_boot_cause;
        pkt.fdir_alarm       = s.fdir_alarm;
        pkt.crc16 = calc_crc16((const uint8_t *)&pkt, sizeof(pkt) - 2);

        char msg[7 + 2 * sizeof(pkt) + 2];
        size_t pos = 0;
        memcpy(msg, "$CCSDS:", 7);
        pos = 7;
        const uint8_t *b = (const uint8_t *)&pkt;
        for (size_t i = 0; i < sizeof(pkt); i++) {
            msg[pos++] = HEXCH[b[i] >> 4];
            msg[pos++] = HEXCH[b[i] & 0x0F];
        }
        msg[pos++] = '\n';
        msg[pos]   = '\0';

        udp_send_to_gs(msg, pos);
        seq++;

        xEventGroupSetBits(g_hb, HB_TM);
        vTaskDelayUntil(&last_wake,
            pdMS_TO_TICKS(s.mode == FSM_MODE_SAFE ? TM_PERIOD_SAFE_MS : TM_PERIOD_NOMINAL_MS));
    }
}

/* Telecommand reception: strict parsing, source check, acceptance report.
 * Note: the ACK only confirms ACCEPTANCE (parsed and queued). Execution
 * verification (PUS service 1 completion reports) is future work. */
static void tc_task(void *arg) {
    (void)arg;
    char buf[64];
    const uint32_t gs_addr = ipaddr_addr(GS_IP);

    for (;;) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int n = lwip_recvfrom(g_sock, buf, sizeof(buf) - 1, 0,
                              (struct sockaddr *)&from, &fromlen);
        if (n > 0) {
            buf[n] = '\0';

            if (from.sin_addr.s_addr != gs_addr) {
                /* Basic sanity filter only - this is NOT authentication. */
                printf("[TC] ignored packet from unexpected source %s\n",
                       ip4addr_ntoa((const ip4_addr_t *)&from.sin_addr));
            } else {
                uint8_t id = 0, val = 0;
                if (!tc_parse(buf, (size_t)n, &id, &val)) {
                    printf("[TC] malformed command (%d bytes)\n", n);
                    send_tc_ack(false, 0, 0, TC_ACK_REJ_MALFORMED);
                } else {
                    uint8_t  status    = TC_ACK_ACCEPTED;
                    tc_cmd_t cmd       = TC_NONE;
                    bool     do_ping   = false;
                    bool     do_reboot = false;

                    if      (id == 10 && val == 0) do_ping   = true;
                    else if (id == 20 && val == 0) cmd       = TC_SET_NOMINAL;
                    else if (id == 20 && val == 1) cmd       = TC_SET_SAFE;
                    else if (id == 99 && val == 1) do_reboot = true;
                    else                           status    = TC_ACK_REJ_UNKNOWN;

                    if (cmd != TC_NONE && xQueueSend(g_tc_queue, &cmd, 0) != pdTRUE) {
                        status = TC_ACK_REJ_QUEUE_FULL;
                    }

                    printf("[TC] id=%u val=%u -> status %u\n",
                           (unsigned)id, (unsigned)val, (unsigned)status);
                    send_tc_ack(true, id, val, status);

                    if (status == TC_ACK_ACCEPTED) {
                        if (do_ping) {
                            const char *alive = "$TM:PUS17,ALIVE\n";
                            udp_send_to_gs(alive, strlen(alive));
                        }
                        if (do_reboot) {
                            /* Stop feeding the watchdog: the reset follows within WDT_TIMEOUT_MS. */
                            record_reset_reason(BOOT_COMMANDED);
                            g_reboot_commanded = true;
                        }
                    }
                }
            }
        }

        xEventGroupSetBits(g_hb, HB_TC);
    }
}

/* Watchdog supervisor: feeds the hardware watchdog only if all tasks reported in. */
static void wdg_task(void *arg) {
    (void)arg;   /* the watchdog itself is armed earlier, in init_task */
    uint32_t window_count = 0;

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(g_hb, HB_ALL,
                                               pdTRUE,    /* clear on exit */
                                               pdTRUE,    /* wait for all  */
                                               pdMS_TO_TICKS(WDG_WINDOW_MS));

        if ((bits & HB_ALL) == HB_ALL) {
            if (!g_reboot_commanded) {
                watchdog_update();
            }
        } else {
            printf("[WDG] missing heartbeat(s): 0x%02X\n", (unsigned)(HB_ALL & ~bits));
        }

        if ((++window_count % 30u) == 0u) {
            for (int i = 0; i < NUM_TRACKED_TASKS; i++) {
                if (g_task_handles[i] != NULL) {
                    printf("[STACK] %-4s min free: %u words\n", g_task_names[i],
                           (unsigned)uxTaskGetStackHighWaterMark(g_task_handles[i]));
                }
            }
            printf("[HEAP] free: %u bytes (min ever: %u)\n",
                   (unsigned)xPortGetFreeHeapSize(),
                   (unsigned)xPortGetMinimumEverFreeHeapSize());
        }
    }
}

/* Wi-Fi connection during boot. Feeds the watchdog while waiting, but only
 * within a bounded deadline, so a stuck connection attempt cannot hide a hang. */
static bool wifi_connect_at_boot(uint32_t timeout_ms) {
    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK) != 0) {
        return false;
    }
    const uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!wifi_link_up()) {
        if ((to_ms_since_boot(get_absolute_time()) - start) >= timeout_ms) {
            return false;
        }
        watchdog_update();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return true;
}

/* Boot sequence, then becomes the "net" task (Wi-Fi link supervision). */
static void init_task(void *arg) {
    (void)arg;

    /* Grace period so a serial monitor can be attached before the first logs. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    printf("\n[OBC] Boot. Cause: %u (0=POWER_ON 1=WDT 2=COMMANDED 3=STACK 4=HEAP 5=INIT)\n",
           (unsigned)g_boot_cause);

    /* Arm the hardware watchdog as early as possible. During the boot sequence
     * this task feeds it; wdg_task takes over once all tasks are running. */
    watchdog_enable(WDT_TIMEOUT_MS, 1);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    if (mpu6050_init()) {
        calibrate_gyro();
    } else {
        printf("[IMU] MPU6050 not responding (FDIR will report IMU FAULT)\n");
    }
    watchdog_update();

    if (cyw43_arch_init()) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "cyw43_arch_init failed");
    }
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    watchdog_update();

    if (!wifi_connect_at_boot(WIFI_BOOT_TIMEOUT_MS)) {
        printf("[WIFI] initial connection failed, will keep retrying\n");
    }
    watchdog_update();

    g_sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock < 0) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "socket creation failed");
    }
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_len    = sizeof(local);
    local.sin_family = AF_INET;
    local.sin_port   = lwip_htons(UDP_PORT_PICO);
    local.sin_addr.s_addr = lwip_htonl(INADDR_ANY);
    if (lwip_bind(g_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "socket bind failed");
    }
    int rcv_timeout_ms = 500;
    lwip_setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout_ms, sizeof(rcv_timeout_ms));

    state_lock();
    g_state.mode       = FSM_MODE_DETUMBLE;
    g_state.fdir_alarm = FDIR_OK;
    state_unlock();

    /* Create the application tasks. g_task_handles[5] ("net") is this task. */
    watchdog_update();
    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(imu_task, "imu", STACK_IMU, NULL, PRIO_IMU, &g_task_handles[0]);
    ok &= xTaskCreate(fsm_task, "fsm", STACK_FSM, NULL, PRIO_FSM, &g_task_handles[1]);
    ok &= xTaskCreate(tm_task,  "tm",  STACK_TM,  NULL, PRIO_TM,  &g_task_handles[2]);
    ok &= xTaskCreate(tc_task,  "tc",  STACK_TC,  NULL, PRIO_TC,  &g_task_handles[3]);
    ok &= xTaskCreate(wdg_task, "wdg", STACK_WDG, NULL, PRIO_WDG, &g_task_handles[4]);
    if (ok != pdPASS) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "task creation failed");
    }
    printf("[OBC] All tasks running\n");

    uint32_t last_try = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!wifi_link_up() && (now - last_try) >= WIFI_RETRY_MS) {
            last_try = now;
            printf("[WIFI] link down, reconnecting\n");
            cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
        }
        xEventGroupSetBits(g_hb, HB_NET);
        vTaskDelay(pdMS_TO_TICKS(NET_PERIOD_MS));
    }
}

int main(void) {
    stdio_init_all();

    g_boot_cause = fetch_boot_cause();

    g_state_mutex = xSemaphoreCreateMutex();
    g_tc_queue    = xQueueCreate(8, sizeof(tc_cmd_t));
    g_hb          = xEventGroupCreate();
    if (g_state_mutex == NULL || g_tc_queue == NULL || g_hb == NULL) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "RTOS object creation failed");
    }

    memset(&g_state, 0, sizeof(g_state));
    g_state.mode = FSM_MODE_BOOT;

    if (xTaskCreate(init_task, "init", STACK_INIT, NULL, PRIO_NET, &g_task_handles[5]) != pdPASS) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "init task creation failed");
    }

    vTaskStartScheduler();

    for (;;) { }
    return 0;
}
