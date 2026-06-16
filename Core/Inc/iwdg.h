/* SPDX-License-Identifier: proprietary */
#ifndef ECU_IWDG_H
#define ECU_IWDG_H

/* App-owned independent watchdog, parity with IFS08-CE-AMS. The HAL handle is
 * private to iwdg.c, so this header stays HAL-type-free and is safe to include
 * from the host/SIL build (which never links the firmware-only iwdg.c). */
void MX_IWDG1_Init(void);   /* start/re-arm the IWDG; call before osKernelStart */
void IWDG_Refresh(void);    /* reload the counter; call once per control cycle */

#endif /* ECU_IWDG_H */
