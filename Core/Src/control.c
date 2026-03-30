#include "control.h"
#include <string.h>

/* Thresholds from your VCU header */
#define UMBRAL_FRENO_APPS 3000u

/* Very small helper */
static void out_push(control_out_t *out, const can_msg_t *m)
{
  if (!out || !m) return;
  if (out->count >= (uint8_t)(sizeof(out->msgs)/sizeof(out->msgs[0]))) return;
  out->msgs[out->count++] = *m;
}

/* Control states: cooperative replacement for blocking while-loops in main.c */
typedef enum
{
  CTRL_ST_BOOT = 0,
  CTRL_ST_WAIT_PRECHARGE_ACK,
  CTRL_ST_WAIT_START_BRAKE,
  CTRL_ST_R2D_DELAY,
  CTRL_ST_READY,
  CTRL_ST_RUN
} ctrl_state_t;

static ctrl_state_t s_state;
static uint32_t s_r2d_start_tick;

void Control_Init(void)
{
  s_state = CTRL_ST_BOOT;
  s_r2d_start_tick = 0;
}

/* Port of your torque mapping (simplified but consistent shape). */
uint16_t Control_ComputeTorque(const app_inputs_t *in, uint8_t *flag_ev_2_3, uint8_t *flag_t11_8_9)
{
  if (!in) return 0;

  float s1_pct = ((float)in->s1_aceleracion - 2050.0f) / (29.5f - 20.5f);
  float s2_pct = ((float)in->s2_aceleracion - 1915.0f) / (25.70f - 19.15f);

  if (s1_pct < 0) s1_pct = 0;
  if (s1_pct > 100) s1_pct = 100;
  if (s2_pct < 0) s2_pct = 0;
  if (s2_pct > 100) s2_pct = 100;

  uint16_t torque = 0;
  if (s1_pct > 8 && s2_pct > 8) torque = (uint16_t)((s1_pct + s2_pct) * 0.5f);
  else torque = 0;

  if (torque < 10) torque = 0;
  else if (torque > 90) torque = 100;

  /* EV 2.3: brake + >25% throttle => latch until throttle <5% and brake released */
  static uint8_t lat_ev23 = 0;
  if (in->s_freno > UMBRAL_FRENO_APPS && torque > 25) lat_ev23 = 1;
  else if (in->s_freno < UMBRAL_FRENO_APPS && torque < 5) lat_ev23 = 0;

  if (flag_ev_2_3) *flag_ev_2_3 = lat_ev23;

  /* Placeholder for T11.8.9 logic; keep 0 unless you implement full plausibility checks. */
  if (flag_t11_8_9) *flag_t11_8_9 = 0;

  if (lat_ev23) torque = 0;
  return torque;
}

/* Build example inverter command frame: ID/format must be aligned to your inverter protocol. */
static void build_inv_cmd(uint16_t torque_pct, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_INV;
  m->id  = 0x181u;    /* txID_inversor from vcu.txt */
  m->dlc = 8;
  /* Example payload: [torque_pct, ...] - adjust to your real inverter protocol */
  m->data[0] = (uint8_t)torque_pct;
}

/* Main 10ms step */
void Control_Step10ms(const app_inputs_t *in, control_out_t *out)
{
  if (!in || !out) return;
  memset(out, 0, sizeof(*out));

  /* Torque computation from inputs (used only in RUN state) */
  uint8_t ev23 = 0, t1189 = 0;
  uint16_t torque = Control_ComputeTorque(in, &ev23, &t1189);
  /* out->torque_pct stays 0 until state reaches CTRL_ST_RUN */

  switch (s_state)
  {
    case CTRL_ST_BOOT:
      if (in->ok_precarga) s_state = CTRL_ST_WAIT_START_BRAKE;
      else s_state = CTRL_ST_WAIT_PRECHARGE_ACK;
      break;

    case CTRL_ST_WAIT_PRECHARGE_ACK:
      /* TODO: enqueue precharge request frames here if required by your ACU. */
      if (in->ok_precarga) s_state = CTRL_ST_WAIT_START_BRAKE;
      break;

    case CTRL_ST_WAIT_START_BRAKE:
      if (in->boton_arranque && in->s_freno > UMBRAL_FRENO_APPS)
      {
        s_r2d_start_tick = osKernelGetTickCount();
        s_state = CTRL_ST_R2D_DELAY;
      }
      break;

    case CTRL_ST_R2D_DELAY:
      if ((osKernelGetTickCount() - s_r2d_start_tick) >= 2000u)
      {
        s_state = CTRL_ST_READY;
      }
      break;

    case CTRL_ST_READY:
      /* TODO: send "ready" inverter command if needed. */
      s_state = CTRL_ST_RUN;
      break;

    case CTRL_ST_RUN:
    default:
    {
      out->torque_pct = torque;   /* Only propagate torque in RUN state */
      can_msg_t cmd;
      build_inv_cmd(torque, &cmd);
      out_push(out, &cmd);
      break;
    }
  }
}
