// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"

extern "C" {
volatile uint32_t g_task_step[ECU_TASK_COUNT] = { 0, 0, 0, 0 };
volatile uint8_t  g_pit_diag_enabled = 0;
}

extern "C" void ecu_app_globals_init(void) {
    for (int i = 0; i < ECU_TASK_COUNT; ++i) {
        g_task_step[i] = 0u;
    }
}
