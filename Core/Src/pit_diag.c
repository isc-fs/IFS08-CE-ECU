#include "pit_diag.h"

#include <string.h>

#define PIT_DIAG_DIVIDER 10u   /* 10 * 10 ms control step = 100 ms cadence */

static uint8_t  s_enabled = 0u;
static uint16_t s_div     = 0u;

uint8_t PitDiag_Enabled(void)              { return s_enabled; }
void    PitDiag_SetEnabled(uint8_t enabled) { s_enabled = enabled ? 1u : 0u; }

static void frame_init(can_msg_t *m, uint32_t id, uint8_t dlc)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_ACU;
  m->id  = id;
  m->dlc = dlc;
  m->ide = 0u;
}

static void put_u16_be(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFFu);
}

void PitDiag_BuildAck(uint8_t enabled, can_msg_t *m)
{
  frame_init(m, PIT_DIAG_ACK_ID, 1u);
  m->data[0] = enabled ? 1u : 0u;
}

void PitDiag_BuildStatus(const app_inputs_t *in, const control_out_t *out, can_msg_t *m)
{
  frame_init(m, PIT_DIAG_STATUS_ID, 6u);
  if (!in || !out) return;
  m->data[0] = out->fsm_state;
  m->data[1] = in->inv_state;
  m->data[2] = (uint8_t)((out->flag_ev_2_3  ? 0x01u : 0u) |
                         (out->flag_t11_8_9 ? 0x02u : 0u) |
                         (out->rtds_active  ? 0x04u : 0u) |
                         (in->ok_precarga   ? 0x08u : 0u) |
                         (in->boton_arranque? 0x10u : 0u));
  m->data[3] = (uint8_t)out->torque_pct;
  put_u16_be(&m->data[4], in->v_celda_min);
}

void PitDiag_BuildPedals(const app_inputs_t *in, can_msg_t *m)
{
  frame_init(m, PIT_DIAG_PEDALS_ID, 6u);
  if (!in) return;
  put_u16_be(&m->data[0], in->s1_aceleracion);
  put_u16_be(&m->data[2], in->s2_aceleracion);
  put_u16_be(&m->data[4], in->s_freno);
}

void PitDiag_BuildInverter(const app_inputs_t *in, can_msg_t *m)
{
  frame_init(m, PIT_DIAG_INVERTER_ID, 7u);
  if (!in) return;
  put_u16_be(&m->data[0], in->inv_dc_bus_voltage);
  m->data[2] = (uint8_t)((uint32_t)in->inv_rpm >> 24);
  m->data[3] = (uint8_t)((uint32_t)in->inv_rpm >> 16);
  m->data[4] = (uint8_t)((uint32_t)in->inv_rpm >> 8);
  m->data[5] = (uint8_t)((uint32_t)in->inv_rpm);
  m->data[6] = in->inv_error;
}

void PitDiag_BuildFwInfo(uint8_t major, uint8_t minor, uint8_t patch,
                         const uint8_t *git4, can_msg_t *m)
{
  frame_init(m, PIT_DIAG_FWINFO_ID, 7u);
  m->data[0] = major;
  m->data[1] = minor;
  m->data[2] = patch;
  if (git4)
  {
    m->data[3] = git4[0];
    m->data[4] = git4[1];
    m->data[5] = git4[2];
    m->data[6] = git4[3];
  }
}

uint8_t PitDiag_Collect(const app_inputs_t *in, const control_out_t *out,
                        uint8_t major, uint8_t minor, uint8_t patch,
                        const uint8_t *git4, can_msg_t *frames, uint8_t max)
{
  uint8_t n = 0u;
  if (!s_enabled || !frames) { s_div = 0u; return 0u; }
  if (++s_div < PIT_DIAG_DIVIDER) return 0u;
  s_div = 0u;
  if (n < max) PitDiag_BuildStatus(in, out, &frames[n++]);
  if (n < max) PitDiag_BuildPedals(in, &frames[n++]);
  if (n < max) PitDiag_BuildInverter(in, &frames[n++]);
  if (n < max) PitDiag_BuildFwInfo(major, minor, patch, git4, &frames[n++]);
  return n;
}
