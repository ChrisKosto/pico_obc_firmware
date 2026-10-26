#ifndef RESET_CAUSE_H
#define RESET_CAUSE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

uint8_t reset_cause_fetch(void);
void reset_cause_record(uint8_t cause);
void obc_fatal_reboot(uint8_t cause, const char *reason);

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name);
void vApplicationMallocFailedHook(void);

#endif /* RESET_CAUSE_H */
