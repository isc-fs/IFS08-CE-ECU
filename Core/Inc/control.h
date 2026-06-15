#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "app_state.h"
#include "can.h"

typedef struct
{
  can_msg_t msgs[8];
  uint8_t  count;
  uint16_t torque_pct; /* 0..100 */
  uint8_t  flag_ev_2_3;
  uint8_t  flag_t11_8_9;
  uint8_t  rtds_active;
  uint8_t  fsm_state;  /* ctrl_state_t after this step (for pit-diag/telemetry) */
} control_out_t;

void Control_Init(void);
void Control_Step10ms(const app_inputs_t *in, control_out_t *out);

/* Computes torque percent and updates flags in a copy; caller decides what to store. */
uint16_t Control_ComputeTorque(const app_inputs_t *in, uint8_t *flag_ev_2_3, uint8_t *flag_t11_8_9);

/* Raw APPS ADC -> 0..100% pedal travel for sensor 1 or 2 (same calibration the
 * torque calc uses). Pure; used by Control_ComputeTorque and the pit-diag
 * pedals frame so the displayed % can't drift from the control %. */
uint8_t Control_AppsPct(uint8_t sensor, uint16_t raw);

#endif /* CONTROL_H */
