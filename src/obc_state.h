#ifndef OBC_STATE_H
#define OBC_STATE_H

#include "obc_context.h"

void obc_state_lock(obc_context_t *ctx);
void obc_state_unlock(obc_context_t *ctx);
obc_state_t obc_state_snapshot(obc_context_t *ctx);

#endif /* OBC_STATE_H */
