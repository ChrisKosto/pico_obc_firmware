#include "fdir_fsm.h"
#include <math.h>
#include <stdbool.h>
#include "obc_config.h"

// Simple counters for auto-recovery debounce
static uint8_t auto_recovery_count = 0;
static bool manual_override = false; // Flag to lock safe mode if triggered by command

#define AUTO_RECOVERY_SAMPLES 20 // Wait a bit before recovering

static uint8_t increment_saturated(uint8_t count) {
    return (count < 255) ? (uint8_t)(count + 1u) : count;
}

void fdir_fsm_init(fdir_fsm_t *fsm, uint32_t now_ms) {
    fsm->overheat_count = 0;
    fsm->tumble_count = 0;
    fsm->detumble_start_ms = now_ms;
    auto_recovery_count = 0;
    manual_override = false;
}

void fdir_fsm_apply_command(obc_state_t *state, tc_cmd_t command) {
    if (command == TC_SET_NOMINAL) {
        state->mode = FSM_MODE_NOMINAL;
        state->fdir_alarm = FDIR_OK;
        auto_recovery_count = 0; 
        manual_override = false; // Release the manual lock
    } else if (command == TC_SET_SAFE) {
        state->mode = FSM_MODE_SAFE;
        manual_override = true;  // Lock system in SAFE mode manually
    }
}

void fdir_fsm_update(fdir_fsm_t *fsm, obc_state_t *state,
                     float mcu_temp_c, uint32_t now_ms) {
                     
    state->mcu_temp_c = mcu_temp_c;

    if (state->mode == FSM_MODE_DETUMBLE &&
        (now_ms - fsm->detumble_start_ms) > DETUMBLE_DURATION_MS) {
        state->mode = FSM_MODE_NOMINAL;
    }

    // 1. Check for active fault conditions
    const bool overheat = (mcu_temp_c > FDIR_TEMP_LIMIT_C) ||
                          (state->imu_ok && state->mpu_temp_c > FDIR_TEMP_LIMIT_C);
    const bool tumbling = state->imu_ok && fabsf(state->gz_dps) > FDIR_GYRO_LIMIT_DPS;

    fsm->overheat_count = overheat ? increment_saturated(fsm->overheat_count) : 0;
    fsm->tumble_count   = tumbling ? increment_saturated(fsm->tumble_count) : 0;

    // 2. Fault Isolation: Enter SAFE mode if conditions persist
    if (fsm->overheat_count >= FDIR_DEBOUNCE_SAMPLES) {
        state->mode = FSM_MODE_SAFE;
        state->fdir_alarm = FDIR_OVERHEAT;
        auto_recovery_count = 0; 
        
    } else if (fsm->tumble_count >= FDIR_DEBOUNCE_SAMPLES) {
        state->mode = FSM_MODE_SAFE;
        state->fdir_alarm = FDIR_TUMBLING;
        auto_recovery_count = 0;
        
    } else if (state->imu_fail_count >= IMU_FAIL_LIMIT) {
        state->mode = FSM_MODE_SAFE;
        state->fdir_alarm = FDIR_IMU_FAULT;
        auto_recovery_count = 0;
    }
    // 3. Auto-Recovery: Try to return to NOMINAL if system is healthy AND not manually locked
    else if (state->mode == FSM_MODE_SAFE && !manual_override) {
        // Check if all faults are clear
        bool system_healthy = (!overheat) && (!tumbling) && (state->imu_fail_count < IMU_FAIL_LIMIT);

        if (system_healthy) {
            auto_recovery_count = increment_saturated(auto_recovery_count);
            
            // Recover when healthy for enough consecutive samples
            if (auto_recovery_count >= AUTO_RECOVERY_SAMPLES) {
                state->mode = FSM_MODE_NOMINAL;
                state->fdir_alarm = FDIR_OK;
                auto_recovery_count = 0;
            }
        } else {
            auto_recovery_count = 0; // Reset if conditions become bad again
        }
    }
}