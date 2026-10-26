#ifndef FDIR_FSM_H
#define FDIR_FSM_H

#include <stdint.h>
#include <stdbool.h>

#include "obc_types.h"

typedef struct {
    uint8_t overheat_count;
    uint8_t tumble_count;
    uint32_t detumble_start_ms;
    uint8_t healthy_count;
    bool manual_safe;
} fdir_fsm_t;

void fdir_fsm_init(fdir_fsm_t *fsm, uint32_t now_ms);
/* NOMINAL is accepted only after the fault-free qualification window. */
bool fdir_fsm_apply_command(fdir_fsm_t *fsm, obc_state_t *state, tc_cmd_t command);
void fdir_fsm_update(fdir_fsm_t *fsm, obc_state_t *state,
                     float mcu_temp_c, uint32_t now_ms);

#endif /* FDIR_FSM_H */
