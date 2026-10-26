#include "fdir_fsm.h"
#include <math.h>
#include "obc_config.h"

static uint8_t saturate(uint8_t n) { return n < UINT8_MAX ? (uint8_t)(n + 1u) : n; }

void fdir_fsm_init(fdir_fsm_t *fsm, uint32_t now_ms) {
    *fsm = (fdir_fsm_t){ .detumble_start_ms = now_ms };
}

bool fdir_fsm_apply_command(fdir_fsm_t *fsm, obc_state_t *state, tc_cmd_t command) {
    if (command == TC_SET_SAFE) {
        state->mode = FSM_MODE_SAFE;
        fsm->manual_safe = true;
        return true;
    }
    if (command == TC_SET_NOMINAL) {
        if (state->mode != FSM_MODE_SAFE || fsm->healthy_count < FDIR_RECOVERY_SAMPLES) {
            return false;
        }
        state->mode = FSM_MODE_NOMINAL;
        state->fdir_alarm = FDIR_OK;
        fsm->manual_safe = false;
        fsm->healthy_count = 0;
        return true;
    }
    return false;
}

void fdir_fsm_update(fdir_fsm_t *fsm, obc_state_t *state,
                     float mcu_temp_c, uint32_t now_ms) {
    state->mcu_temp_c = mcu_temp_c;
    const bool temperature_valid = isfinite(mcu_temp_c) &&
        (!state->imu_ok || isfinite(state->mpu_temp_c));
    const bool overheat = !temperature_valid || mcu_temp_c > FDIR_TEMP_LIMIT_C ||
        (state->imu_ok && state->mpu_temp_c > FDIR_TEMP_LIMIT_C);
    const bool tumbling = state->imu_ok &&
        (!isfinite(state->gz_dps) || fabsf(state->gz_dps) > FDIR_GYRO_LIMIT_DPS);
    const bool imu_fault = !state->imu_ok || state->imu_fail_count >= IMU_FAIL_LIMIT;

    fsm->overheat_count = overheat ? saturate(fsm->overheat_count) : 0u;
    fsm->tumble_count = tumbling ? saturate(fsm->tumble_count) : 0u;
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

    const bool healthy = !overheat && !tumbling && !imu_fault;
    fsm->healthy_count = healthy ? saturate(fsm->healthy_count) : 0u;
    if (state->mode == FSM_MODE_DETUMBLE &&
        (uint32_t)(now_ms - fsm->detumble_start_ms) > DETUMBLE_DURATION_MS &&
        state->fdir_alarm == FDIR_OK) {
        state->mode = FSM_MODE_NOMINAL;
    }
    /* SAFE faults are latched: no autonomous transition to NOMINAL. */
}
