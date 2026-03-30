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
} control_out_t;

void Control_Init(void);
void Control_Step10ms(const app_inputs_t *in, control_out_t *out);

/* Computes torque percent and updates flags in a copy; caller decides what to store. */
uint16_t Control_ComputeTorque(const app_inputs_t *in, uint8_t *flag_ev_2_3, uint8_t *flag_t11_8_9);

#endif /* CONTROL_H */
