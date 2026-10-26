#include "reset_cause.h"

#include <stdio.h>

#include "hardware/watchdog.h"

#include "obc_types.h"

#define RESET_REASON_MAGIC      0xC0DE0000u
#define RESET_REASON_MASK       0xFFFF0000u

void reset_cause_record(uint8_t cause) {
    watchdog_hw->scratch[0] = RESET_REASON_MAGIC | cause;
}

uint8_t reset_cause_fetch(void) {
    uint32_t scratch = watchdog_hw->scratch[0];
    watchdog_hw->scratch[0] = 0;

    if (!watchdog_caused_reboot()) {
        return BOOT_POWER_ON;
    }
    if ((scratch & RESET_REASON_MASK) == RESET_REASON_MAGIC) {
        return (uint8_t)(scratch & 0xFFu);
    }
    return BOOT_WDT_TIMEOUT;
}

void obc_fatal_reboot(uint8_t cause, const char *reason) {
    printf("[FATAL] %s -> rebooting\n", reason);
    reset_cause_record(cause);
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
    (void)task;
    reset_cause_record(BOOT_STACK_OVERFLOW);
    printf("[FATAL] stack overflow in task '%s'\n", task_name);
    watchdog_reboot(0, 0, 0);
    for (;;) { }
}

void vApplicationMallocFailedHook(void) {
    obc_fatal_reboot(BOOT_HEAP_EXHAUSTED, "FreeRTOS heap exhausted");
}
