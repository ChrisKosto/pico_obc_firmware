#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"      // Internal MCU temperature sensor
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

// ---------------------------------------------------------------------------
// Wi-Fi & Network
// ---------------------------------------------------------------------------
#define WIFI_SSID           "CUBESAT-GROUND-STATION"
#define WIFI_PASSWORD       "12345678"
#define GS_IP               "192.168.4.1"
#define UDP_PORT_GS         5000
#define UDP_PORT_PICO       5001
#define WIFI_RETRY_MS       5000    // reconnect attempt interval

// ---------------------------------------------------------------------------
// I2C0 / MPU6050
// ---------------------------------------------------------------------------
#define I2C_PORT            i2c0
#define PIN_SDA             16
#define PIN_SCL             17
#define MPU6050_ADDR        0x68
#define I2C_TIMEOUT_US      5000

#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_XOUT      0x3B

// Gyro range +/-500 deg/s (FS_SEL=1) -> 65.5 LSB per deg/s.
// (At the default +/-250 deg/s the sensor saturates at exactly 250, so a
//  250 deg/s FDIR limit could never trigger.)
#define GYRO_FS_500         0x08
#define GYRO_SCALE          65.5f

// ---------------------------------------------------------------------------
// FDIR limits
// ---------------------------------------------------------------------------
#define FDIR_TEMP_LIMIT_C   65.0f
#define FDIR_GYRO_LIMIT_DPS 250.0f   // now reachable thanks to +/-500 deg/s range
#define IMU_FAIL_LIMIT      10       // consecutive failed reads -> IMU FAULT

// fdir_alarm codes (must match the ground station decoder)
#define FDIR_OK             0
#define FDIR_OVERHEAT       1
#define FDIR_TUMBLING       2
#define FDIR_IMU_FAULT      3

typedef enum {
    FSM_MODE_BOOT      = 0,
    FSM_MODE_DETUMBLE  = 1,
    FSM_MODE_NOMINAL   = 2,
    FSM_MODE_SAFE      = 3
} flight_mode_t;

// CCSDS packet structure (little-endian fields, see note in README)
#pragma pack(push, 1)
typedef struct {
    uint16_t packet_id;
    uint16_t packet_seq_ctrl;
    uint16_t packet_data_len;
} ccsds_primary_header_t;

typedef struct {
    ccsds_primary_header_t header;
    uint32_t uptime_ms;
    int16_t  mpu_temp_centi_c;     // MPU6050 temperature x100
    int16_t  mcu_temp_centi_c;     // Pico MCU temperature x100
    int16_t  roll_centi_deg;
    int16_t  pitch_centi_deg;
    int16_t  yaw_centi_deg;        // wrapped to [-180, +180] deg
    uint8_t  flight_mode;
    uint8_t  boot_cause;           // 0 = POWER_ON, 1 = WATCHDOG_RESET
    uint8_t  fdir_alarm;           // 0 OK, 1 OVERHEAT, 2 TUMBLING, 3 IMU FAULT
    uint16_t crc16;
} ccsds_telemetry_packet_t;
#pragma pack(pop)

volatile flight_mode_t current_mode = FSM_MODE_BOOT;
struct udp_pcb *udp_client_pcb = NULL;
ip_addr_t gs_addr;
float gz_offset = 0.0f;
uint32_t detumble_timer_ms = 0;
uint8_t reset_reason = 0;
volatile uint8_t fdir_status = FDIR_OK;
volatile bool reboot_commanded = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
uint16_t calc_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc = crc << 1;
        }
    }
    return crc;
}

static int16_t to_centi(float v) {
    float s = v * 100.0f;
    if (s > 32767.0f)  s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
}

static bool wifi_link_up(void) {
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

void send_udp_msg(const char *msg) {
    if (udp_client_pcb == NULL || !wifi_link_up()) return;
    size_t n = strlen(msg);
    cyw43_arch_lwip_begin();
    struct pbuf *p_out = pbuf_alloc(PBUF_TRANSPORT, n, PBUF_RAM);
    if (p_out != NULL) {
        memcpy(p_out->payload, msg, n);
        udp_sendto(udp_client_pcb, p_out, &gs_addr, UDP_PORT_GS);
        pbuf_free(p_out);
    }
    cyw43_arch_lwip_end();
}

// Non-blocking Wi-Fi supervisor: retries the association every WIFI_RETRY_MS.
// Requires pico-sdk >= 1.5 (cyw43_arch_wifi_connect_async).
static void wifi_maintain(uint32_t now) {
    static uint32_t last_try = 0;
    if (wifi_link_up()) return;
    if (now - last_try < WIFI_RETRY_MS) return;
    last_try = now;
    printf("[WIFI] link down, reconnecting...\n");
    cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
}

// ---------------------------------------------------------------------------
// Telecommand receiver
// ---------------------------------------------------------------------------
void udp_cmd_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        char buffer[64];
        int len = p->len < 63 ? p->len : 63;
        memcpy(buffer, p->payload, len);
        buffer[len] = '\0';

        if (strstr(buffer, "$TC:10,00") != NULL) {
            send_udp_msg("$TM:PUS17,ALIVE\n");
        } else if (strstr(buffer, "$TC:20,00") != NULL) {
            current_mode = FSM_MODE_NOMINAL;
            fdir_status = FDIR_OK;
        } else if (strstr(buffer, "$TC:20,01") != NULL) {
            current_mode = FSM_MODE_SAFE;
        } else if (strstr(buffer, "$TC:99,01") != NULL) {
            reboot_commanded = true;
        }

        pbuf_free(p);
    }
}

// ---------------------------------------------------------------------------
// MPU6050
// ---------------------------------------------------------------------------
static bool mpu6050_init(void) {
    uint8_t wake[2] = {REG_PWR_MGMT_1, 0x00};
    if (i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, wake, 2, false, I2C_TIMEOUT_US) < 0)
        return false;
    sleep_ms(100);
    uint8_t cfg[2] = {REG_GYRO_CONFIG, GYRO_FS_500};
    return i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, cfg, 2, false, I2C_TIMEOUT_US) >= 0;
}

// Burst read of accel/temp/gyro (14 bytes). Returns false on any I2C error.
static bool mpu_read_burst(uint8_t *data) {
    uint8_t reg = REG_ACCEL_XOUT;
    if (i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, &reg, 1, true, I2C_TIMEOUT_US) < 0)
        return false;
    return i2c_read_timeout_us(I2C_PORT, MPU6050_ADDR, data, 14, false, I2C_TIMEOUT_US) == 14;
}

void calibrate_gyro(void) {
    float sum_gz = 0.0f;
    int valid = 0;
    for (int i = 0; i < 150; i++) {
        uint8_t data[14];
        if (mpu_read_burst(data)) {
            int16_t gz = (int16_t)((data[12] << 8) | data[13]);
            sum_gz += (float)gz / GYRO_SCALE;
            valid++;
        }
        sleep_ms(10);
    }
    gz_offset = (valid > 0) ? (sum_gz / (float)valid) : 0.0f;
}

float read_pico_temperature(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    stdio_init_all();
    sleep_ms(1500);

    reset_reason = watchdog_caused_reboot() ? 1 : 0;

    adc_init();
    adc_set_temp_sensor_enabled(true);

    if (cyw43_arch_init()) {
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    bool imu_present = mpu6050_init();
    if (imu_present) calibrate_gyro();
    else printf("[IMU] MPU6050 not responding at boot\n");

    // Initial blocking connect. Failure is not fatal: wifi_maintain() keeps
    // retrying in the main loop.
    int wifi_rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                                     CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (wifi_rc != 0) printf("[WIFI] initial connect failed (%d), will retry\n", wifi_rc);

    ip4addr_aton(GS_IP, &gs_addr);
    udp_client_pcb = udp_new();
    if (udp_client_pcb != NULL) {
        udp_bind(udp_client_pcb, IP_ADDR_ANY, UDP_PORT_PICO);
        udp_recv(udp_client_pcb, udp_cmd_recv_callback, NULL);
    }

    watchdog_enable(4000, 1);

    current_mode = FSM_MODE_DETUMBLE;
    detumble_timer_ms = to_ms_since_boot(get_absolute_time());

    uint16_t seq = 0;
    float yaw = 0.0f;
    uint32_t last_time = to_ms_since_boot(get_absolute_time());

    // Last known good IMU values (held while the IMU is failing)
    float mpu_temp_c = 0.0f, roll = 0.0f, pitch = 0.0f, gz_dps = 0.0f;
    int imu_fail_count = imu_present ? 0 : IMU_FAIL_LIMIT;
    bool imu_ok = false;

    while (1) {
        if (!reboot_commanded) {
            watchdog_update();
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        wifi_maintain(now);

        if (current_mode == FSM_MODE_DETUMBLE && (now - detumble_timer_ms > 5000)) {
            current_mode = FSM_MODE_NOMINAL;
        }

        float dt = (now - last_time) / 1000.0f;
        last_time = now;

        // ---- IMU acquisition ----
        uint8_t data[14];
        if (mpu_read_burst(data)) {
            imu_fail_count = 0;
            imu_ok = true;

            int16_t ax = (int16_t)((data[0] << 8) | data[1]);
            int16_t ay = (int16_t)((data[2] << 8) | data[3]);
            int16_t az = (int16_t)((data[4] << 8) | data[5]);
            int16_t raw_temp = (int16_t)((data[6] << 8) | data[7]);
            int16_t gz = (int16_t)((data[12] << 8) | data[13]);

            mpu_temp_c = (raw_temp / 340.0f) + 36.53f;
            roll  = atan2f((float)ay, (float)az) * 57.2957795f;
            pitch = atan2f(-(float)ax, sqrtf((float)ay * (float)ay + (float)az * (float)az)) * 57.2957795f;

            gz_dps = ((float)gz / GYRO_SCALE) - gz_offset;
            if (fabsf(gz_dps) > 0.5f) {
                yaw += gz_dps * dt;
                // Wrap to [-180, +180] so the int16 centi-degree field never overflows
                if (yaw > 180.0f)       yaw -= 360.0f;
                else if (yaw < -180.0f) yaw += 360.0f;
            }
        } else {
            imu_ok = false;
            if (imu_fail_count < 1000) imu_fail_count++;
            // Try to re-initialise the sensor periodically
            if (imu_fail_count % IMU_FAIL_LIMIT == 0) {
                if (mpu6050_init()) printf("[IMU] re-initialised\n");
            }
        }

        float pico_temp_c = read_pico_temperature();

        // ---- FDIR ----
        if (pico_temp_c > FDIR_TEMP_LIMIT_C || (imu_ok && mpu_temp_c > FDIR_TEMP_LIMIT_C)) {
            current_mode = FSM_MODE_SAFE;
            fdir_status = FDIR_OVERHEAT;
        } else if (imu_ok && fabsf(gz_dps) > FDIR_GYRO_LIMIT_DPS) {
            current_mode = FSM_MODE_SAFE;
            fdir_status = FDIR_TUMBLING;
        } else if (imu_fail_count >= IMU_FAIL_LIMIT) {
            current_mode = FSM_MODE_SAFE;
            fdir_status = FDIR_IMU_FAULT;
        }

        // ---- Telemetry (always sent, even with a failed IMU) ----
        ccsds_telemetry_packet_t pkt;
        pkt.header.packet_id = 0x0801;
        pkt.header.packet_seq_ctrl = 0xC000 | (seq & 0x3FFF);
        pkt.header.packet_data_len = sizeof(ccsds_telemetry_packet_t) - sizeof(ccsds_primary_header_t) - 1;
        pkt.uptime_ms = now;
        pkt.mpu_temp_centi_c = to_centi(mpu_temp_c);
        pkt.mcu_temp_centi_c = to_centi(pico_temp_c);
        pkt.roll_centi_deg = to_centi(roll);
        pkt.pitch_centi_deg = to_centi(pitch);
        pkt.yaw_centi_deg = to_centi(yaw);
        pkt.flight_mode = (uint8_t)current_mode;
        pkt.boot_cause = reset_reason;
        pkt.fdir_alarm = fdir_status;
        pkt.crc16 = calc_crc16((uint8_t *)&pkt, sizeof(ccsds_telemetry_packet_t) - 2);

        static const char HEX[] = "0123456789ABCDEF";
        char hex_msg[7 + 2 * sizeof(ccsds_telemetry_packet_t) + 2];
        size_t pos = 0;
        memcpy(hex_msg, "$CCSDS:", 7);
        pos = 7;
        const uint8_t *pkt_bytes = (const uint8_t *)&pkt;
        for (size_t i = 0; i < sizeof(ccsds_telemetry_packet_t); i++) {
            hex_msg[pos++] = HEX[pkt_bytes[i] >> 4];
            hex_msg[pos++] = HEX[pkt_bytes[i] & 0x0F];
        }
        hex_msg[pos++] = '\n';
        hex_msg[pos] = '\0';

        send_udp_msg(hex_msg);
        seq++;

        if (current_mode == FSM_MODE_SAFE) {
            sleep_ms(500);
        } else {
            sleep_ms(50);
        }
    }
    return 0;
}