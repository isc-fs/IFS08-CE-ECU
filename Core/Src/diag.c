#include "diag.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* FreeRTOS heap metrics (available if you include FreeRTOS.h and heap APIs are enabled). */
#include "FreeRTOS.h"
#include "task.h"

void Diag_Report(osMessageQueueId_t rxQ, osMessageQueueId_t txQ)
{
  char buf[160];
  uint32_t rx_used = rxQ ? osMessageQueueGetCount(rxQ) : 0;
  uint32_t tx_used = txQ ? osMessageQueueGetCount(txQ) : 0;

  size_t free_heap = xPortGetFreeHeapSize();
  size_t min_ever  = xPortGetMinimumEverFreeHeapSize();

  (void)snprintf(buf, sizeof(buf),
                 "DIAG: rxQ=%lu txQ=%lu heapFree=%u heapMin=%u\r\n",
                 (unsigned long)rx_used, (unsigned long)tx_used,
                 (unsigned)free_heap, (unsigned)min_ever);
  Diag_Log("%s", buf);
}

__attribute__((weak)) void Diag_Log(const char *fmt, ...)
{
  (void)fmt;
  /* Implement UART logging in your project. */
}
