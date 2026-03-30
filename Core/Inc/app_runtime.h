#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

/* Shared one-shot/task-step helpers extracted from freertos.c.
 * They let SIL reuse the real runtime logic instead of duplicating it
 * in a separate scheduler harness.
 */

void AppRuntime_InitStep(void);
void AppRuntime_ControlStep(void);
void AppRuntime_CanRxStep(void);
void AppRuntime_CanTxStep(void);
void AppRuntime_TelemetryStep(void);

#endif /* APP_RUNTIME_H */
