#include "diag.h"
#include "usart.h"
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
  char buf[192];
  size_t len;
  va_list args;

  if (!fmt || huart10.Instance == NULL)
  {
    return;
  }

  va_start(args, fmt);
  len = (size_t)vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len >= sizeof(buf))
  {
    len = sizeof(buf) - 1u;
  }

  if ((len + 2u) < sizeof(buf) &&
      (len == 0u || (buf[len - 1u] != '\n' && buf[len - 1u] != '\r')))
  {
    buf[len++] = '\r';
    buf[len++] = '\n';
    buf[len] = '\0';
  }

  (void)HAL_UART_Transmit(&huart10, (uint8_t *)buf, (uint16_t)len, 100u);
}
