/*
 * Raspberry Pi Pico 2 W (RP2350, Arm Cortex-M33) + FreeRTOS
 * Satellite On-Board Computer flight software for the CubeSat FlatSat.
 */

#include <string.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "obc_config.h"
#include "obc_context.h"
#include "reset_cause.h"
#include "tasks.h"

static obc_context_t obc;

int main(void) {
    stdio_init_all();

    memset(&obc, 0, sizeof(obc));
    obc.socket = -1;
    obc.boot_cause = reset_cause_fetch();
    obc.state.mode = FSM_MODE_BOOT;

    obc.state_mutex = xSemaphoreCreateMutex();
    obc.tc_queue = xQueueCreate(8, sizeof(tc_cmd_t));
    obc.heartbeat = xEventGroupCreate();
    if (obc.state_mutex == NULL || obc.tc_queue == NULL || obc.heartbeat == NULL) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "RTOS object creation failed");
    }

    if (xTaskCreate(obc_init_task, "init", STACK_INIT, &obc,
                    PRIO_NET, &obc.task_handles[5]) != pdPASS) {
        obc_fatal_reboot(BOOT_FATAL_INIT, "init task creation failed");
    }

    vTaskStartScheduler();

    for (;;) { }
    return 0;
}
