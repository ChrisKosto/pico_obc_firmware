#include <stdio.h>

#include "fdir_fsm.h"
#include "obc_config.h"

static int failures = 0;

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        failures++; \
    } \
} while (0)

int main(void) {
    fdir_fsm_t fsm;
    obc_state_t state = {
        .mode = FSM_MODE_NOMINAL,
        .fdir_alarm = FDIR_OK,
        .imu_ok = true
    };
    fdir_fsm_init(&fsm, 0u);

    for (int i = 0; i < FDIR_DEBOUNCE_SAMPLES - 1; i++) {
        fdir_fsm_update(&fsm, &state, 70.0f, (uint32_t)i * 50u);
        CHECK(state.mode == FSM_MODE_NOMINAL);
    }
    fdir_fsm_update(&fsm, &state, 70.0f, 1000u);
    CHECK(state.mode == FSM_MODE_SAFE);
    CHECK(state.fdir_alarm == FDIR_OVERHEAT);

    CHECK(!fdir_fsm_apply_command(&fsm, &state, TC_SET_NOMINAL));
    CHECK(state.mode == FSM_MODE_SAFE);
    /* Recovery is qualified by 20 consecutive fault-free samples. */
    for (int i = 0; i < FDIR_RECOVERY_SAMPLES; ++i) {
        fdir_fsm_update(&fsm, &state, 20.0f, 1100u + (uint32_t)i * 50u);
    }
    CHECK(state.mode == FSM_MODE_SAFE);
    CHECK(fdir_fsm_apply_command(&fsm, &state, TC_SET_NOMINAL));
    CHECK(state.mode == FSM_MODE_NOMINAL);
    CHECK(state.fdir_alarm == FDIR_OK);

    state.mcu_temp_c = 20.0f;
    state.mpu_temp_c = 20.0f;
    state.gz_dps = 300.0f;
    fdir_fsm_init(&fsm, 0u);
    for (int i = 0; i < FDIR_DEBOUNCE_SAMPLES; i++) {
        fdir_fsm_update(&fsm, &state, 20.0f, (uint32_t)i * 50u);
    }
    CHECK(state.mode == FSM_MODE_SAFE);
    CHECK(state.fdir_alarm == FDIR_TUMBLING);

    state = (obc_state_t){
        .mode = FSM_MODE_NOMINAL,
        .imu_ok = false,
        .imu_fail_count = IMU_FAIL_LIMIT
    };
    fdir_fsm_init(&fsm, 0u);
    fdir_fsm_update(&fsm, &state, 20.0f, 0u);
    CHECK(state.mode == FSM_MODE_SAFE);
    CHECK(state.fdir_alarm == FDIR_IMU_FAULT);

    state = (obc_state_t){ .mode = FSM_MODE_DETUMBLE, .imu_ok = true };
    fdir_fsm_init(&fsm, 100u);
    fdir_fsm_update(&fsm, &state, 20.0f, 100u + DETUMBLE_DURATION_MS);
    CHECK(state.mode == FSM_MODE_DETUMBLE);
    fdir_fsm_update(&fsm, &state, 20.0f, 101u + DETUMBLE_DURATION_MS);
    CHECK(state.mode == FSM_MODE_NOMINAL);

    /* Qualification must be consecutive, not cumulative. */
    state = (obc_state_t){ .mode = FSM_MODE_SAFE, .imu_ok = true };
    fdir_fsm_init(&fsm, 0u);
    for (int i = 0; i < FDIR_RECOVERY_SAMPLES - 1; ++i)
        fdir_fsm_update(&fsm, &state, 20.0f, (uint32_t)i * 50u);
    state.imu_ok = false;
    fdir_fsm_update(&fsm, &state, 20.0f, 999u);
    state.imu_ok = true;
    CHECK(!fdir_fsm_apply_command(&fsm, &state, TC_SET_NOMINAL));
    CHECK(state.mode == FSM_MODE_SAFE);

    if (failures == 0) {
        printf("fdir_fsm: all tests passed\n");
        return 0;
    }
    printf("fdir_fsm: %d failure(s)\n", failures);
    return 1;
}
