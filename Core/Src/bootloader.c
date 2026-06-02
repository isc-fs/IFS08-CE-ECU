#include "bootloader.h"

#include "main.h"        /* HAL: RTC, RCC, PWR, IWDG1, NVIC_SystemReset, __DSB */
#include "cmsis_os2.h"   /* osDelay */

/* stm32-can-bootloader stay-in-BL magic (Core/Inc/bl_memmap.h). Keep in sync. */
#define BL_BOOT_REQ_MAGIC  0xB00710ADu

void Bootloader_RequestReboot(void)
{
  /* Let any in-flight TX (the ACK to whoever sent the trigger, the last
   * telemetry frame) reach the wire before the reset. 10 ms is plenty: a full
   * 8-byte frame is < 200 us at 500 kbps. */
  osDelay(10u);

  /* Stamp the boot-request magic into the backup register the bootloader reads
   * on every reset. Mirrors stm32-can-bootloader's Bootloader_IsBootRequestActive:
   * enable backup-domain write access, make sure the RTC clock (which gates the
   * BKP registers) is on, then write. Under the bootloader the RTC access path
   * is already configured -- this just re-asserts it. */
  HAL_PWR_EnableBkUpAccess();
  if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0u)
  {
    __HAL_RCC_RTC_ENABLE();
  }
  RTC->BKP0R = BL_BOOT_REQ_MAGIC;
  __DSB();

  /* The bootloader's reset handler sees the magic, clears it (one-shot) and
   * stays in BL mode awaiting flash on the CAN bus. */
  NVIC_SystemReset();

  for (;;) { }  /* unreachable; silences no-return warnings */
}

void Bootloader_KickWatchdog(void)
{
  /* Reload key for the independent watchdog the bootloader started (~8 s)
   * before jumping to us. Writing 0xAAAA to a non-running IWDG is a harmless
   * no-op, so this is safe even for a standalone (no-bootloader) debug image. */
  IWDG1->KR = 0x0000AAAAu;
}
