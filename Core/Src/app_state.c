#include "app_state.h"
#include <string.h>

/* Defined/created in freertos.c USER CODE (so CubeMX keeps it). */
osMutexId_t  g_inMutex;
app_inputs_t g_in;

void AppState_Init(void)
{
  memset(&g_in, 0, sizeof(g_in));
}

void AppState_Snapshot(app_inputs_t *out)
{
  if (!out) return;
  if (g_inMutex) (void)osMutexAcquire(g_inMutex, osWaitForever);
  *out = g_in;
  if (g_inMutex) (void)osMutexRelease(g_inMutex);
}
