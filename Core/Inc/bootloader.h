#ifndef BOOTLOADER_H
#define BOOTLOADER_H

/* Application-side entry back into the isc-fs/stm32-can-bootloader, plus the
 * watchdog the bootloader hands us.
 *
 * Trigger: a single CAN frame on the ACU bus. It SHARES id 0x002 with the AMS
 * (the bootloader's app-level boot-request id) but uses a DISTINCT payload, so
 * one frame targets exactly one board:
 *     AMS : 0x002  data = B0 07 AD 11
 *     ECU : 0x002  data = B0 07 AD 12   <- this firmware
 * The payload magic guards against an accidental reboot from bus noise.
 *
 * Mechanism: write BL_BOOT_REQ_MAGIC (0xB00710AD) to RTC->BKP0R, then
 * NVIC_SystemReset. The bootloader reads BKP0R on reset, sees the magic, and
 * stays in BL mode awaiting a flash. See stm32-can-bootloader Core/Inc/bl_memmap.h.
 */

#include <stdint.h>
#include "can.h"

#define BL_BOOT_REQ_CAN_ID   0x002u
#define BL_BOOT_REQ_DLC      4u
/* ECU boot-request payload (distinct from the AMS's B0 07 AD 11). */
#define BL_BOOT_REQ_B0       0xB0u
#define BL_BOOT_REQ_B1       0x07u
#define BL_BOOT_REQ_B2       0xADu
#define BL_BOOT_REQ_B3       0x12u

/* Pure predicate: true iff the frame is the ECU boot-request trigger.
 * Header-inline so the host SIL/unit build can exercise it without HAL. */
static inline uint8_t Bootloader_MatchesTrigger(const can_msg_t *m)
{
  if (!m) return 0u;
  if (m->bus != CAN_BUS_ACU)        return 0u;
  if (m->id  != BL_BOOT_REQ_CAN_ID) return 0u;
  if (m->dlc != BL_BOOT_REQ_DLC)    return 0u;
  return (uint8_t)(m->data[0] == BL_BOOT_REQ_B0 &&
                   m->data[1] == BL_BOOT_REQ_B1 &&
                   m->data[2] == BL_BOOT_REQ_B2 &&
                   m->data[3] == BL_BOOT_REQ_B3);
}

/* Stamp the boot-request magic into RTC->BKP0R and reset into the bootloader.
 * Never returns. Firmware-only (touches HAL / system registers). */
void Bootloader_RequestReboot(void);

/* The IWDG is owned by the app via HAL now (Core/Src/iwdg.c: MX_IWDG1_Init +
 * IWDG_Refresh) -- the old Bootloader_KickWatchdog() is retired. */

#endif /* BOOTLOADER_H */
