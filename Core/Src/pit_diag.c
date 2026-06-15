#include "pit_diag.h"
#include "can/can_codecs.h"   /* code-first CAN DSL: generated pit-diag encoders */

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

/* Each builder is a thin adapter over the code-first CAN DSL: the byte layout,
 * id and dlc come from Core/Inc/can/messages/pit_diag_*.def via the generated
 * encoders + PitDiag_*_ID/DLC. frame_init still owns the envelope (bus/ide and
 * the zeroed buffer the null-input path returns). Parity vs the old hand-rolled
 * bytes is asserted by SIL --test-pit-diag and --test-dsl-parity. */

void PitDiag_BuildAck(uint8_t enabled, can_msg_t *m)
{
  PitDiag_ack_t a = {0};
  frame_init(m, PitDiag_ack_ID, PitDiag_ack_DLC);
  a.enabled = enabled ? 1u : 0u;
  encode_PitDiag_ack(&a, m->data);
}

void PitDiag_BuildStatus(const app_inputs_t *in, const control_out_t *out, can_msg_t *m)
{
  PitDiag_status_t s = {0};
  frame_init(m, PitDiag_status_ID, PitDiag_status_DLC);
  if (!in || !out) return;
  s.fsm_state     = out->fsm_state;
  s.inv_state     = in->inv_state;
  s.flags         = (uint8_t)((out->flag_ev_2_3  ? 0x01u : 0u) |
                              (out->flag_t11_8_9 ? 0x02u : 0u) |
                              (out->rtds_active  ? 0x04u : 0u) |
                              (in->ok_precarga   ? 0x08u : 0u) |
                              (in->boton_arranque? 0x10u : 0u));
  s.torque_pct    = (uint8_t)out->torque_pct;
  s.v_cell_min_mV = in->v_celda_min;
  encode_PitDiag_status(&s, m->data);
}

void PitDiag_BuildPedals(const app_inputs_t *in, can_msg_t *m)
{
  PitDiag_pedals_t p = {0};
  frame_init(m, PitDiag_pedals_ID, PitDiag_pedals_DLC);
  if (!in) return;
  p.apps1_raw = in->s1_aceleracion;
  p.apps2_raw = in->s2_aceleracion;
  p.brake_raw = in->s_freno;
  encode_PitDiag_pedals(&p, m->data);
}

void PitDiag_BuildInverter(const app_inputs_t *in, can_msg_t *m)
{
  PitDiag_inverter_t iv = {0};
  frame_init(m, PitDiag_inverter_ID, PitDiag_inverter_DLC);
  if (!in) return;
  iv.dc_bus_voltage = in->inv_dc_bus_voltage;
  iv.inv_rpm        = in->inv_rpm;
  iv.inv_error      = in->inv_error;
  encode_PitDiag_inverter(&iv, m->data);
}

void PitDiag_BuildFwInfo(uint8_t major, uint8_t minor, uint8_t patch,
                         const uint8_t *git4, can_msg_t *m)
{
  PitDiag_fwinfo_t fw = {0};
  frame_init(m, PitDiag_fwinfo_ID, PitDiag_fwinfo_DLC);
  fw.fw_major = major;
  fw.fw_minor = minor;
  fw.fw_patch = patch;
  if (git4)
  {
    fw.git_hash = ((uint32_t)git4[0] << 24) | ((uint32_t)git4[1] << 16) |
                  ((uint32_t)git4[2] << 8)  |  (uint32_t)git4[3];
  }
  encode_PitDiag_fwinfo(&fw, m->data);
}

void PitDiag_BuildHealth(uint16_t free_heap, uint16_t min_free_heap,
                         uint8_t task_ran_mask, uint8_t reset_cause,
                         uint8_t uptime_s, uint8_t last_fault, can_msg_t *m)
{
  PitDiag_health_t h = {0};
  frame_init(m, PitDiag_health_ID, PitDiag_health_DLC);
  h.free_heap     = free_heap;
  h.min_free_heap = min_free_heap;
  h.task_ran_mask = task_ran_mask;
  h.reset_cause   = reset_cause;
  h.uptime_s      = uptime_s;
  h.last_fault    = last_fault;
  encode_PitDiag_health(&h, m->data);
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
