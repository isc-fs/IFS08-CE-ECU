#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include "cmsis_os2.h"

/* Application-wide shared inputs/state (protected by g_inMutex). */
typedef struct
{
  /* Sensors / inputs */
  uint16_t s1_aceleracion;   /* ADC raw */
  uint16_t s2_aceleracion;   /* ADC raw */
  uint16_t s_freno;          /* ADC raw */
  uint8_t  boton_arranque;   /* 0/1 */

  /* Inverter feedback */
  uint8_t  inv_state;        /* e.g. standby/ready/fault (project-specific) */
  uint16_t inv_dc_bus_voltage; /* raw / scaled, project-specific */
  int16_t  inv_motor_temp;
  int16_t  inv_igbt_temp;
  int16_t  inv_air_temp;
  int16_t  inv_rpm;

  /* Battery / misc */
  uint16_t v_celda_min;      /* raw / scaled */
  uint8_t  ok_precarga;      /* ack from ACU, etc. */

  /* Safety / plausibility flags */
  uint8_t flag_EV_2_3;
  uint8_t flag_T11_8_9;

  /* Derived */
  uint16_t torque_total;     /* 0..100% */

} app_inputs_t;

extern app_inputs_t g_in;
extern osMutexId_t  g_inMutex;

void AppState_Init(void);
void AppState_Snapshot(app_inputs_t *out);

#endif /* APP_STATE_H */
