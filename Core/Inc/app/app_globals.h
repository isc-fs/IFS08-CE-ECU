/* SPDX-License-Identifier: proprietary
 *
 * C-visible globals shared between the CubeMX freertos.c (which creates the
 * tasks + queues) and the C++ app. Kept in a C header so freertos.c can call
 * ecu_app_globals_init() from its USER CODE.
 */

#ifndef ECU_APP_GLOBALS_H
#define ECU_APP_GLOBALS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Task indices for the 0x704 task_ran_mask liveness bitmap. */
enum {
    ECU_TASK_CONTROL = 0,
    ECU_TASK_CAN_RX  = 1,
    ECU_TASK_CAN_TX  = 2,
    ECU_TASK_DIAG    = 3,
    ECU_TASK_COUNT   = 4,
};

/* Free-running per-task step counters: each task bumps its own every loop.
 * DiagTask snapshots them once per health frame and reports, per task, whether
 * the counter advanced (bit set = the task ran). A cyclic task whose bit stays
 * clear has stalled -- the exact #48-class symptom we need visible over CAN. */
extern volatile uint32_t g_task_step[ECU_TASK_COUNT];

/* Pit-diag app-state stream (0x700-0x705) gate: set by CanRxTask on a 0x7E0
 * enable command (magic 0xDEADBEEF), read by ControlTask. The 0x704 health
 * frame is deliberately ALWAYS-ON (ungated) -- it's the diagnostic lifeline. */
extern volatile uint8_t g_pit_diag_enabled;

/* Called once from freertos.c USER CODE after osKernelInitialize(), before the
 * scheduler starts. Currently just zeroes the step counters; a hook point for
 * any app-side RTOS object the .ioc can't carry. */
void ecu_app_globals_init(void);

/* C-linkage app shim (defined in error_latch.cpp) so the CubeMX freertos.c
 * stack-overflow / malloc-failed hooks can latch a fault sentinel before they
 * spin into the watchdog reset. code = FaultCode (0xF5 stack-overflow, 0xF6
 * malloc-failed). */
void ecu_fault_latch_set_c(uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* ECU_APP_GLOBALS_H */
