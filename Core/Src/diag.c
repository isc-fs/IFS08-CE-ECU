#include "diag.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* FreeRTOS heap metrics (available if you include FreeRTOS.h and heap APIs are enabled). */
#include "FreeRTOS.h"
#include "task.h"

/* --- Firmware-health: reset cause + sticky fault (0x704, #36) ------------- */

static uint8_t s_reset_cause = DIAG_RESET_UNKNOWN;
static uint8_t s_last_fault  = DIAG_FAULT_NONE;

void Diag_CaptureResetCause(void)
{
  /* Read RCC->RSR before clearing. Check specific causes before PIN, which is
   * asserted on nearly every reset. IWDG is the one we are chasing (#36). */
  if      (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)) s_reset_cause = DIAG_RESET_IWDG;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST)) s_reset_cause = DIAG_RESET_WWDG;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))   s_reset_cause = DIAG_RESET_SOFT;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST) ||
           __HAL_RCC_GET_FLAG(RCC_FLAG_LPWR2RST)) s_reset_cause = DIAG_RESET_LPWR;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) ||
           __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))   s_reset_cause = DIAG_RESET_POR;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))   s_reset_cause = DIAG_RESET_PIN;
  else                                            s_reset_cause = DIAG_RESET_UNKNOWN;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Sticky fault sentinel from RTC backup BKP1R (BKP0R is the BL reboot magic).
   * Same backup-access path bootloader.c uses; leaves RTCEN on so a later
   * Diag_LatchFault() from a fault handler can write BKP1R. */
  HAL_PWR_EnableBkUpAccess();
  if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0u)
  {
    __HAL_RCC_RTC_ENABLE();
  }
  s_last_fault = (uint8_t)(RTC->BKP1R & 0xFFu);
  RTC->BKP1R = 0u;
  __DSB();
}

uint8_t Diag_ResetCause(void) { return s_reset_cause; }
uint8_t Diag_LastFault(void)  { return s_last_fault; }

void Diag_LatchFault(uint8_t code)
{
  /* Fault-handler context: raw register writes only, no HAL/RTOS calls.
   * RTCEN was left on by Diag_CaptureResetCause at boot. First fault wins
   * (BKP1R was cleared at boot) until the next reset reads + clears it. */
  PWR->CR1 |= PWR_CR1_DBP;
  RTC->BKP1R = (uint32_t)code;
  __DSB();
}

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
