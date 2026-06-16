/* SPDX-License-Identifier: proprietary */
#include "iwdg.h"
#include "main.h"   /* HAL: IWDG_HandleTypeDef, HAL_IWDG_*, Error_Handler */

/* Independent watchdog, owned by the app via HAL -- the same pattern the AMS
 * uses (which configures IWDG1 in CubeMX). The bootloader starts IWDG1 (~8 s)
 * before jumping to us; MX_IWDG1_Init() re-arms it to ~500 ms here, and is
 * called before osKernelStart so the watchdog is alive in the pre-scheduler
 * window. ControlTask then refreshes it every 10 ms cycle via IWDG_Refresh();
 * on a fault path it stops refreshing, so the chip resets back to the BL.
 *
 * Prescaler 32, Reload 500 -> ~500 ms at nominal 32 kHz LSI (~340 ms fast
 * corner, ~940 ms slow). The longer reload vs the AMS's 100 ms gives margin for
 * the ECU's heavier boot (SDMMC/FATFS/USB) before the first refresh.
 *
 * NOTE: the IWDG is not yet declared in ECU08 NSIL.ioc -- add IWDG1 in CubeMX
 * to reflect it in the GUI. This module + the HAL_IWDG_MODULE_ENABLED build
 * define (firmware/CMakeLists.txt) keep it working and regen-safe meanwhile. */
static IWDG_HandleTypeDef hiwdg1;

void MX_IWDG1_Init(void)
{
  hiwdg1.Instance       = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg1.Init.Window    = 4095;   /* window disabled */
  hiwdg1.Init.Reload    = 500;    /* ~500 ms @ nominal LSI */
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
}

void IWDG_Refresh(void)
{
  (void)HAL_IWDG_Refresh(&hiwdg1);
}
