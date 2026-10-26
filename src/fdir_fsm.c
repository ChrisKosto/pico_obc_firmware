#include "fdir_fsm.h"

#include <math.h>

#include "obc_config.h"

static uint8_t increment_saturated(uint8_t count) {
    return (count < 255) ? (uint8_t)(count + 1u) : count;
}

void fdir_fsm_init(fdir_fsm_t *fsm, uint32_t now_ms) {
    fsm->overheat_count = 0;
    fsm->tumble_count = 0;
    fsm->detumble_start_ms = now_ms;
}

void fdir_fsm_apply_command(obc_state_t *state, tc_cmd_t command) {
    if (command == TC_SET_NOMINAL) {
        state->mode = FSM_MODE_NOMINAL;
        state->fdir_alarm = FDIR_OK;
    } else if (command == TC_SET_SAFE) {
        state->mode = FSM_MODE_SAFE;
    }
}

void fdir_fsm_update(fdir_fsm_t *fsm, obc_state_t *state,
                     float mcu_temp_c, uint32_t now_ms) {
    state->mcu_temp_c = mcu_temp_c;

    if (state->mode == FSM_MODE_DETUMBLE &&
        (now_ms - fsm->detumble_start_ms) > DETUMBLE_DURATION_MS) {
        state->mode = FSM_MODE_NOMINAL;
    }

    const bool overheat = (mcu_temp_c > FDIR_TEMP_LIMIT_C) ||
                          (state->imu_ok && state->mpu_temp_c > FDIR_TEMP_LIMIT_C);
    const bool tumbling = state->imu_ok && fabsf(state->gz_dps) > FDIR_GYRO_LIMIT_DPS;

    fsm->overheat_count = overheat ? increment_saturated(fsm->overheat_count) : 0;
    fsm->tumble_count   = tumbling ? increment_saturated(fsm->tumble_count) : 0;

    if (fsm->overheat_count >= FDIR_DEBOUNCE_SAMPLES) {
        state->mode = FSM_MODE_SAFE;
        state->fdir_alarm = FDIR_OVERHEAT;
    } else if (fsm->tumble_count >= FDIR_DEBOUNCE_SAMPLES) {
        state->mode = FSM_MODE_SAFE;
        state->fdir_alarm = FDIR_TUMBLING;
    } else if (state->imu_fail_count >= IMU_FAIL_LIMIT) {
        state->mode = FSM_MODE_SAFE;
        state->fdir_alarm = FDIR_IMU_FAULT;
    }
}
