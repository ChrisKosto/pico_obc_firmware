#include "obc_state.h"

void obc_state_lock(obc_context_t *ctx) {
    xSemaphoreTake(ctx->state_mutex, portMAX_DELAY);
}

void obc_state_unlock(obc_context_t *ctx) {
    xSemaphoreGive(ctx->state_mutex);
}

obc_state_t obc_state_snapshot(obc_context_t *ctx) {
    obc_state_t snapshot;
    obc_state_lock(ctx);
    snapshot = ctx->state;
    obc_state_unlock(ctx);
    return snapshot;
}
