#include "tasks.h"

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "board.h"
#include "fdir_fsm.h"
#include "mpu6050.h"
#include "network.h"
#include "obc_config.h"
#include "obc_context.h"
#include "obc_state.h"
#include "reset_cause.h"
#include "telemetry.h"

static const char *const TASK_NAMES[NUM_TRACKED_TASKS] = {
    "imu", "fsm", "tm", "tc", "wdg", "net"
};

/* Gyroscope-only relative yaw. Startup calibration is performed by
 * mpu6050_calibrate_gyro() before this task starts. No in-flight bias
 * learning: constant-rate Z rotations must remain measurable.
 * This is not absolute heading and will drift without an external reference.
 */
static void imu_task(void *argument) {
    obc_context_t *ctx = (obc_context_t *)argument;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_sample_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_log_ms = last_sample_ms;
    uint32_t failures = 0u;
    float yaw = 0.0f;
    uint32_t last_cal_attempt_ms = last_sample_ms;

    printf("[IMU] Relative yaw (not absolute heading); calibrated=%u startup offset=%+.4f dps\n",
           (unsigned)ctx->gyro_calibrated, (double)ctx->gz_offset);

    for (;;) {
        mpu6050_sample_t sample;
        /* A failed calibration must never produce plausible-looking yaw. */
        if (!ctx->gyro_calibrated) {
            const uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_cal_attempt_ms >= 5000u) {
                last_cal_attempt_ms = now;
                printf("[IMU] attempting device recovery and stationary calibration\n");
                if (mpu6050_init() && mpu6050_calibrate_gyro(&ctx->gz_offset)) {
                    ctx->gyro_calibrated = true;
                    yaw = 0.0f;
                    last_sample_ms = to_ms_since_boot(get_absolute_time());
                    printf("[IMU] recovered gyro calibration bias=%+.4f dps\n",
                           (double)ctx->gz_offset);
                }
            }
            obc_state_lock(ctx);
            ctx->state.imu_ok = false;
            ctx->state.imu_fail_count = IMU_FAIL_LIMIT;
            obc_state_unlock(ctx);
            xEventGroupSetBits(ctx->heartbeat, HB_IMU);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
            continue;
        }
        if (mpu6050_read_sample(ctx->gz_offset, &sample)) {
            failures = 0u;
            const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            const uint32_t elapsed_ms = now_ms - last_sample_ms;
            last_sample_ms = now_ms;
            /* Do not integrate large elapsed intervals after scheduling stalls. */
            const float dt = (elapsed_ms > 0u && elapsed_ms <= 200u)
                           ? (float)elapsed_ms * 0.001f : 0.0f;

            /* gz_dps is already corrected using the boot-time gyro bias. */
            const float yaw_rate = sample.gz_dps;
            if (dt > 0.0f && isfinite(yaw_rate)) {
                yaw += yaw_rate * dt;
                if (yaw > 180.0f) yaw -= 360.0f;
                else if (yaw < -180.0f) yaw += 360.0f;
            }

            obc_state_lock(ctx);
            ctx->state.mpu_temp_c = sample.temperature_c;
            ctx->state.roll = sample.roll_deg;
            ctx->state.pitch = sample.pitch_deg;
            ctx->state.yaw = yaw;
            ctx->state.gz_dps = sample.gz_dps;
            ctx->state.imu_ok = true;
            ctx->state.imu_fail_count = 0;
            obc_state_unlock(ctx);

            if (now_ms - last_log_ms >= 1000u) {
                last_log_ms = now_ms;
                printf("[IMU] gz=%+.4f dps offset=%+.4f dt=%.3f s yaw=%+.2f deg\n",
                       (double)yaw_rate, (double)ctx->gz_offset,
                       (double)dt, (double)yaw);
            }
        } else {
            failures++;
            last_sample_ms = to_ms_since_boot(get_absolute_time());
            /* Never reuse an old bias after repeated I2C/data failures. */
            if (failures == IMU_FAIL_LIMIT) {
                printf("[IMU] %u consecutive read failures; re-probe/calibration required\n",
                       (unsigned)failures);
                ctx->gyro_calibrated = false;
                ctx->gz_offset = 0.0f;
                yaw = 0.0f;
                last_cal_attempt_ms = last_sample_ms;
            }
            obc_state_lock(ctx);
            ctx->state.imu_ok = false;
            ctx->state.imu_fail_count = failures > 255u ? 255u : (uint8_t)failures;
            obc_state_unlock(ctx);
        }
        xEventGroupSetBits(ctx->heartbeat, HB_IMU);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

static void fsm_task(void *argument) {
    obc_context_t *ctx = (obc_context_t *)argument;
    TickType_t last_wake = xTaskGetTickCount();
    fdir_fsm_t fsm;
    fdir_fsm_init(&fsm, to_ms_since_boot(get_absolute_time()));
    flight_mode_t previous_mode = FSM_MODE_BOOT;
    uint8_t previous_alarm = FDIR_OK;

    for (;;) {
        tc_cmd_t command;
        while (xQueueReceive(ctx->tc_queue, &command, 0) == pdTRUE) {
            obc_state_lock(ctx);
            if (!fdir_fsm_apply_command(&fsm, &ctx->state, command)) {
                printf("[FSM] command rejected by safety interlock\n");
            }
            obc_state_unlock(ctx);
        }

        float mcu_temperature = board_read_temperature();
        uint32_t now = to_ms_since_boot(get_absolute_time());

        obc_state_lock(ctx);
        fdir_fsm_update(&fsm, &ctx->state, mcu_temperature, now);
        const flight_mode_t current_mode = ctx->state.mode;
        const uint8_t current_alarm = ctx->state.fdir_alarm;
        const float current_gz = ctx->state.gz_dps;
        obc_state_unlock(ctx);

        if (current_mode != previous_mode || current_alarm != previous_alarm) {
            printf("[FSM] mode %u -> %u, alarm %u -> %u (mcu %.1fC, gz %.1f dps)\n",
                   (unsigned)previous_mode, (unsigned)current_mode,
                   (unsigned)previous_alarm, (unsigned)current_alarm,
                   (double)mcu_temperature, (double)current_gz);
            previous_mode = current_mode;
            previous_alarm = current_alarm;
        }

        xEventGroupSetBits(ctx->heartbeat, HB_FSM);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

static void telemetry_task(void *argument) {
    obc_context_t *ctx = (obc_context_t *)argument;
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t sequence = 0;

    for (;;) {
        obc_state_t state = obc_state_snapshot(ctx);
        ccsds_telemetry_packet_t packet;
        telemetry_build_packet(&packet, &state,
                               to_ms_since_boot(get_absolute_time()),
                               ctx->boot_cause, sequence);

        char message[TELEMETRY_ASCII_CAPACITY];
        size_t length = telemetry_encode_ascii(&packet, message, sizeof(message));
        if (length > 0u) {
            network_send_to_gs(ctx, message, length);
        }
        sequence++;

        xEventGroupSetBits(ctx->heartbeat, HB_TM);
        vTaskDelayUntil(&last_wake,
            pdMS_TO_TICKS(state.mode == FSM_MODE_SAFE ? TM_PERIOD_SAFE_MS
                                                      : TM_PERIOD_NOMINAL_MS));
    }
}

static void telecommand_task(void *argument) {
    obc_context_t *ctx = (obc_context_t *)argument;
    
    // Fix: Setup a tick counter for our delay so we don't hog the CPU
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        network_poll_telecommand(ctx);
        xEventGroupSetBits(ctx->heartbeat, HB_TC);
        
        // Let the CPU breathe for 100ms so other tasks (like TM) can run
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

static void watchdog_task(void *argument) {
    obc_context_t *ctx = (obc_context_t *)argument;
    uint32_t window_count = 0;

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(ctx->heartbeat, HB_ALL,
                                               pdTRUE, pdTRUE,
                                               pdMS_TO_TICKS(WDG_WINDOW_MS));
        if ((bits & HB_ALL) == HB_ALL) {
            if (!ctx->reboot_commanded) {
                watchdog_update();
            }
        } else {
            printf("[WDG] missing heartbeat(s): 0x%02X\n",
                   (unsigned)(HB_ALL & ~bits));
        }

        if ((++window_count % 30u) == 0u) {
            for (int i = 0; i < NUM_TRACKED_TASKS; i++) {
                if (ctx->task_handles[i] != NULL) {
                    printf("[STACK] %-4s min free: %u words\n", TASK_NAMES[i],
                           (unsigned)uxTaskGetStackHighWaterMark(ctx->task_handles[i]));
                }
            }
            printf("[HEAP] free: %u bytes (min ever: %u)\n",
                   (unsigned)xPortGetFreeHeapSize(),
                   (unsigned)xPortGetMinimumEverFreeHeapSize());
        }
    }
}

void obc_init_task(void *argument) {
    obc_context_t *ctx = (obc_context_t *)argument;

    /* Enable supervision before any startup delay or peripheral initialization. */
    watchdog_enable(WDT_TIMEOUT_MS, 1);
    watchdog_update();
    /* Serial grace period is shorter than the watchdog interval. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    watchdog_update();
    printf("\n[OBC] Boot. Cause: %u (0=POWER_ON 1=WDT 2=COMMANDED 3=STACK 4=HEAP 5=INIT)\n",
           (unsigned)ctx->boot_cause);

    /* The init task feeds the watchdog until all supervised tasks are running. */
    board_temperature_init();
    mpu6050_bus_init();
    if (mpu6050_init()) {
        ctx->gyro_calibrated = mpu6050_calibrate_gyro(&ctx->gz_offset);
        if (!ctx->gyro_calibrated) {
            printf("[IMU] gyro calibration failed: yaw disabled until retry\n");
        }
    } else {
        ctx->gyro_calibrated = false;
        printf("[IMU] MPU6050 not responding (FDIR will report IMU FAULT)\n");
    }
    watchdog_update();

    if (!network_init()) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "cyw43_arch_init failed");
    }
    watchdog_update();

    if (!network_connect_at_boot(WIFI_BOOT_TIMEOUT_MS)) {
        printf("[WIFI] initial connection failed, will keep retrying\n");
    }
    watchdog_update();

    network_socket_result_t socket_result = network_open_socket(&ctx->socket);
    if (socket_result == NETWORK_SOCKET_CREATE_FAILED) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "socket creation failed");
    }
    if (socket_result == NETWORK_SOCKET_BIND_FAILED) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "socket bind failed");
    }

    obc_state_lock(ctx);
    ctx->state.mode = FSM_MODE_DETUMBLE;
    ctx->state.fdir_alarm = FDIR_OK;
    obc_state_unlock(ctx);

    watchdog_update();
    /* Fail with the task name instead of bitwise-combining status values. */
#define START_TASK(fn, name, stack, priority, index) do { \
    if (xTaskCreate(fn, name, stack, ctx, priority, &ctx->task_handles[index]) != pdPASS) { \
        obc_fatal_reboot(BOOT_FATAL_INIT, "could not create " name " task"); \
    } \
} while (0)
    START_TASK(imu_task, "imu", STACK_IMU, PRIO_IMU, 0);
    START_TASK(fsm_task, "fsm", STACK_FSM, PRIO_FSM, 1);
    START_TASK(telemetry_task, "tm", STACK_TM, PRIO_TM, 2);
    START_TASK(telecommand_task, "tc", STACK_TC, PRIO_TC, 3);
    START_TASK(watchdog_task, "wdg", STACK_WDG, PRIO_WDG, 4);
#undef START_TASK
    printf("[OBC] All tasks running\n");

    uint32_t last_try = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!network_wifi_link_up() && (now - last_try) >= WIFI_RETRY_MS) {
            last_try = now;
            printf("[WIFI] link down, reconnecting\n");
            network_reconnect();
        }
        xEventGroupSetBits(ctx->heartbeat, HB_NET);
        vTaskDelay(pdMS_TO_TICKS(NET_PERIOD_MS));
    }
}