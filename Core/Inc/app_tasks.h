#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"

void App_InitTask(void *argument);
void CanRxTask(void *argument);
void CanTxTask(void *argument);
void ControlTask(void *argument);
void TelemetryTask(void *argument);
void DiagTask(void *argument);

#endif /* APP_TASKS_H */
