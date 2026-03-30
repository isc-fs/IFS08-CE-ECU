#include "telemetry.h"
#include <string.h>

void Telemetry_Build32(const app_inputs_t *in, uint8_t out32[32])
{
  memset(out32, 0, 32);
  if (!in) return;

  /* Simple packing: keep stable over time; adjust as needed */
  out32[0] = in->inv_state;
  out32[1] = (uint8_t)(in->torque_total & 0xFFu);

  out32[2] = (uint8_t)(in->inv_dc_bus_voltage & 0xFFu);
  out32[3] = (uint8_t)((in->inv_dc_bus_voltage >> 8) & 0xFFu);

  out32[4] = (uint8_t)(in->v_celda_min & 0xFFu);
  out32[5] = (uint8_t)((in->v_celda_min >> 8) & 0xFFu);

  out32[6] = (uint8_t)(in->s1_aceleracion & 0xFFu);
  out32[7] = (uint8_t)((in->s1_aceleracion >> 8) & 0xFFu);
  out32[8] = (uint8_t)(in->s2_aceleracion & 0xFFu);
  out32[9] = (uint8_t)((in->s2_aceleracion >> 8) & 0xFFu);

  out32[10] = (uint8_t)(in->s_freno & 0xFFu);
  out32[11] = (uint8_t)((in->s_freno >> 8) & 0xFFu);

  out32[12] = in->flag_EV_2_3;
  out32[13] = in->flag_T11_8_9;
  out32[14] = in->ok_precarga;
  out32[15] = in->boton_arranque;
}

__attribute__((weak)) void Telemetry_Send32(const uint8_t payload[32])
{
  (void)payload;
  /* Implement transport (nRF24/UART/etc.) in your project. */
}
