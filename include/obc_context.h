#ifndef OBC_CONTEXT_H
#define OBC_CONTEXT_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "obc_config.h"
#include "obc_types.h"

/* Shared application state and RTOS resources, passed explicitly to tasks. */
typedef struct {
    obc_state_t        state;
    SemaphoreHandle_t  state_mutex;
    QueueHandle_t      tc_queue;
    EventGroupHandle_t heartbeat;

    int                socket;
    float              gz_offset;
    bool               gyro_calibrated;
    uint8_t            boot_cause;
    volatile bool      reboot_commanded;

    TaskHandle_t       task_handles[NUM_TRACKED_TASKS];
} obc_context_t;

#endif /* OBC_CONTEXT_H */