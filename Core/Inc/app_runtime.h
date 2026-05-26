#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>
#include "telemetry.h"

/* Shared one-shot/task-step helpers extracted from freertos.c.
 * They let SIL reuse the real runtime logic instead of duplicating it
 * in a separate scheduler harness.
 */

typedef enum
{
  APP_TASK_ID_DEFAULT = 0,
  APP_TASK_ID_INIT,
  APP_TASK_ID_CONTROL,
  APP_TASK_ID_CAN_RX,
  APP_TASK_ID_CAN_TX,
  APP_TASK_ID_TELEMETRY,
  APP_TASK_ID_DIAG,
  APP_TASK_ID_RADIO_TX,
  APP_TASK_ID_SD_LOG,
  APP_TASK_ID_DASH,
  APP_TASK_ID_COUNT
} app_task_id_t;

typedef struct
{
  const char *name;
  uint32_t start_count;
  uint32_t step_count;
  uint32_t last_start_tick;
  uint32_t last_step_tick;
} app_task_metrics_t;

void AppRuntime_InitStep(void);
void AppRuntime_ControlStep(void);
void AppRuntime_CanRxStep(void);
void AppRuntime_CanTxStep(void);
void AppRuntime_TelemetryStep(void);
void AppRuntime_TelemetryEventStep(const telemetry_event_t *event);
void AppRuntime_RadioTxStep(void);
void AppRuntime_SdLogStep(void);
void AppRuntime_DashStep(void);

void AppRuntime_TaskMetricsReset(void);
uint32_t AppRuntime_TaskMetricsCount(void);
const app_task_metrics_t *AppRuntime_TaskMetricsGet(app_task_id_t task_id);
void AppRuntime_TaskMetricsSnapshot(app_task_metrics_t *out_metrics, uint32_t max_metrics);
uint8_t AppRuntime_TaskStarted(app_task_id_t task_id);
uint8_t AppRuntime_TaskStepped(app_task_id_t task_id);

#endif /* APP_RUNTIME_H */
