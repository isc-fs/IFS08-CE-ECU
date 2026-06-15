#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>
#include "cmsis_os2.h"

void Diag_Report(osMessageQueueId_t rxQ, osMessageQueueId_t txQ);

/* Formatted logging (printf-style). Default weak no-op. */
void Diag_Log(const char *fmt, ...);

/* --- Firmware-health diagnostics (PitDiag_health 0x704, IFS08-CE-ECU#36) --- */

/* reset_cause reported in 0x704 byte 5. */
#define DIAG_RESET_UNKNOWN 0u
#define DIAG_RESET_POR     1u
#define DIAG_RESET_PIN     2u
#define DIAG_RESET_SOFT    3u
#define DIAG_RESET_IWDG    4u
#define DIAG_RESET_WWDG    5u
#define DIAG_RESET_LPWR    6u

/* Sticky fault sentinels reported in 0x704 byte 7, latched in RTC backup so
 * they survive the reset (e.g. a hardfault that spins into an IWDG reset). */
#define DIAG_FAULT_NONE       0x00u
#define DIAG_FAULT_HARDFAULT  0xF1u
#define DIAG_FAULT_MEMMANAGE  0xF2u
#define DIAG_FAULT_BUSFAULT   0xF3u
#define DIAG_FAULT_USAGEFAULT 0xF4u
#define DIAG_FAULT_STACKOVF   0xF5u  /* FreeRTOS stack-overflow hook */
#define DIAG_FAULT_MALLOC     0xF6u  /* FreeRTOS malloc-failed hook */
#define DIAG_FAULT_ASSERT     0xF7u  /* configASSERT */

/* Latch the reset cause (RCC->RSR) + sticky fault (RTC BKP1R) ONCE at boot,
 * before the flags are cleared. Call early in MX_FREERTOS_Init. */
void Diag_CaptureResetCause(void);
uint8_t Diag_ResetCause(void);
uint8_t Diag_LastFault(void);
/* Stamp a fault sentinel that survives the reset; call from a fault handler. */
void Diag_LatchFault(uint8_t code);
/* configASSERT target: latch DIAG_FAULT_ASSERT, disable IRQs, spin. */
void Diag_AssertFault(void);

#endif /* DIAG_H */
