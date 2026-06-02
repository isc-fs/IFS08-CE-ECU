#ifndef PIT_DIAG_H
#define PIT_DIAG_H

/* Toggleable bench-visibility stream on the ACU bus (the wire shared with the
 * AMS), emitted at 100 ms while enabled. IDs deliberately do NOT collide with
 * the AMS pit-diag, which owns 0x680-0x6C8 and 0x7F0/0x7F1. */

#include <stdint.h>
#include "can.h"
#include "app_state.h"
#include "control.h"

#define PIT_DIAG_CMD_ID        0x7E0u   /* RX: enable/disable command */
#define PIT_DIAG_ACK_ID        0x7E1u   /* TX: ack */
#define PIT_DIAG_STATUS_ID     0x700u   /* TX: fsm / inverter / flags / torque */
#define PIT_DIAG_PEDALS_ID     0x701u   /* TX: apps1 / apps2 / brake (raw ADC) */
#define PIT_DIAG_INVERTER_ID   0x702u   /* TX: dc-bus / rpm / error */
#define PIT_DIAG_FWINFO_ID     0x703u   /* TX: version + git hash */

#define PIT_DIAG_ENABLE_MAGIC  0xDEADBEEFu  /* data[0..3] LE; anything else = off */

/* Command predicate (pure, header-inline so SIL/unit can use it). If the frame
 * is a pit-diag command, writes the requested enable state to *enable and
 * returns 1; otherwise returns 0 and leaves *enable untouched. */
static inline uint8_t PitDiag_MatchCommand(const can_msg_t *m, uint8_t *enable)
{
  uint32_t v;
  if (!m) return 0u;
  if (m->bus != CAN_BUS_ACU)     return 0u;
  if (m->id  != PIT_DIAG_CMD_ID) return 0u;
  if (m->dlc < 4u)               return 0u;
  v = (uint32_t)m->data[0] | ((uint32_t)m->data[1] << 8) |
      ((uint32_t)m->data[2] << 16) | ((uint32_t)m->data[3] << 24);
  if (enable) *enable = (v == PIT_DIAG_ENABLE_MAGIC) ? 1u : 0u;
  return 1u;
}

/* Pure frame encoders (no HAL; host-testable). */
void PitDiag_BuildAck(uint8_t enabled, can_msg_t *m);
void PitDiag_BuildStatus(const app_inputs_t *in, const control_out_t *out, can_msg_t *m);
void PitDiag_BuildPedals(const app_inputs_t *in, can_msg_t *m);
void PitDiag_BuildInverter(const app_inputs_t *in, can_msg_t *m);
void PitDiag_BuildFwInfo(uint8_t major, uint8_t minor, uint8_t patch,
                         const uint8_t *git4, can_msg_t *m);

/* Runtime enable state. */
uint8_t PitDiag_Enabled(void);
void    PitDiag_SetEnabled(uint8_t enabled);

/* When enabled, every 100 ms (10 control steps) build the frame set into
 * `frames` (capacity `max`); returns the count built, else 0. */
uint8_t PitDiag_Collect(const app_inputs_t *in, const control_out_t *out,
                        uint8_t major, uint8_t minor, uint8_t patch,
                        const uint8_t *git4, can_msg_t *frames, uint8_t max);

#endif /* PIT_DIAG_H */
