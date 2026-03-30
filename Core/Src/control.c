#include "control.h"
#include <string.h>

/* Thresholds from your VCU header */
#define UMBRAL_FRENO_APPS 3000u
#define ID_DC_BUS_VOLTAGE 0x100u
#define ID_PRECHARGE_CMD  0x600u
#define RX_SETPOINT_1     0x360u
#define RX_SETPOINT_3     0x362u

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
static uint8_t s_ev23_latched;

void Control_Init(void)
{
  s_state = CTRL_ST_BOOT;
  s_r2d_start_tick = 0;
  s_ev23_latched = 0u;
}

static uint16_t saturate_pct(float value)
{
  if (value < 0.0f) return 0u;
  if (value > 100.0f) return 100u;
  return (uint16_t)value;
}

static uint16_t torque_pct_to_legacy_command(uint16_t torque_pct)
{
  uint16_t scaled = torque_pct;

  if (scaled >= 10u)
  {
    scaled = (uint16_t)(((uint32_t)scaled * 240u) / 90u - (2400u / 90u));
  }

  return (uint16_t)(~scaled + 1u);
}

static void build_inv_mode_cmd(uint8_t mode, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_INV;
  m->id  = RX_SETPOINT_1;
  m->dlc = 3;
  m->data[2] = mode;
}

static void build_inv_torque_cmd(uint16_t legacy_torque, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_INV;
  m->id  = RX_SETPOINT_3;
  m->dlc = 4;
  m->data[2] = (uint8_t)(legacy_torque & 0xFFu);
  m->data[3] = (uint8_t)((legacy_torque >> 8) & 0xFFu);
}

static void build_acu_dc_bus_frame(uint16_t dc_bus_voltage, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_ACU;
  m->id  = ID_DC_BUS_VOLTAGE;
  m->dlc = 2;
  m->ide = 1;
  m->data[0] = (uint8_t)(dc_bus_voltage & 0xFFu);
  m->data[1] = (uint8_t)((dc_bus_voltage >> 8) & 0xFFu);
}

static void build_acu_precharge_cmd(uint8_t button_pressed, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_ACU;
  m->id  = ID_PRECHARGE_CMD;
  m->dlc = 2;
  m->ide = 1;
  m->data[0] = button_pressed ? 1u : 0u;
}

/* Port of legacy main_polling.c torque mapping, but exposed here as 0..100%. */
uint16_t Control_ComputeTorque(const app_inputs_t *in, uint8_t *flag_ev_2_3, uint8_t *flag_t11_8_9)
{
  if (!in) return 0;

  uint16_t s1_pct = saturate_pct(((float)in->s1_aceleracion - 2050.0f) / (29.5f - 20.5f));
  uint16_t s2_pct = saturate_pct(((float)in->s2_aceleracion - 1915.0f) / (25.70f - 19.15f));

  uint16_t torque = 0;
  if (s1_pct > 8u && s2_pct > 8u) torque = (uint16_t)((s1_pct + s2_pct) / 2u);

  if (torque < 10u) torque = 0u;
  else if (torque > 90u) torque = 100u;

  /* EV 2.3 legacy latch. */
  if (in->s_freno > UMBRAL_FRENO_APPS && torque > 25u) s_ev23_latched = 1u;
  else if (in->s_freno < UMBRAL_FRENO_APPS && torque < 5u) s_ev23_latched = 0u;

  if (flag_ev_2_3) *flag_ev_2_3 = s_ev23_latched;

  {
    uint16_t diff = (s1_pct > s2_pct) ? (uint16_t)(s1_pct - s2_pct) : (uint16_t)(s2_pct - s1_pct);
    uint8_t t11 = (diff > 10u) ? 1u : 0u;
    if (flag_t11_8_9) *flag_t11_8_9 = t11;
    if (s_ev23_latched || t11)
    {
      torque = 0u;
    }
  }

  if (s_ev23_latched)
  {
    torque = 0u;
  }

  return torque;
}

/* Main 10ms step */
void Control_Step10ms(const app_inputs_t *in, control_out_t *out)
{
  if (!in || !out) return;
  memset(out, 0, sizeof(*out));

  /* Torque computation from inputs (used only in RUN state) */
  uint8_t ev23 = 0, t1189 = 0;
  uint16_t torque = Control_ComputeTorque(in, &ev23, &t1189);
  out->flag_ev_2_3 = ev23;
  out->flag_t11_8_9 = t1189;
  /* out->torque_pct stays 0 until state reaches CTRL_ST_RUN */

  switch (s_state)
  {
    case CTRL_ST_BOOT:
      if (in->ok_precarga)
      {
        s_state = CTRL_ST_WAIT_START_BRAKE;
      }
      else
      {
        can_msg_t dc_bus_cmd;
        build_acu_dc_bus_frame(in->inv_dc_bus_voltage, &dc_bus_cmd);
        out_push(out, &dc_bus_cmd);
        if (in->boton_arranque)
        {
          can_msg_t precharge_cmd;
          build_acu_precharge_cmd(in->boton_arranque, &precharge_cmd);
          out_push(out, &precharge_cmd);
        }
        s_state = CTRL_ST_WAIT_PRECHARGE_ACK;
      }
      break;

    case CTRL_ST_WAIT_PRECHARGE_ACK:
      if (in->ok_precarga)
      {
        s_state = CTRL_ST_WAIT_START_BRAKE;
      }
      else
      {
        can_msg_t dc_bus_cmd;
        build_acu_dc_bus_frame(in->inv_dc_bus_voltage, &dc_bus_cmd);
        out_push(out, &dc_bus_cmd);
        if (in->boton_arranque)
        {
          can_msg_t precharge_cmd;
          build_acu_precharge_cmd(in->boton_arranque, &precharge_cmd);
          out_push(out, &precharge_cmd);
        }
      }
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
    {
      can_msg_t mode_cmd, torque_cmd;
      build_inv_mode_cmd(0x04u, &mode_cmd);
      build_inv_torque_cmd(0u, &torque_cmd);
      out_push(out, &mode_cmd);
      out_push(out, &torque_cmd);
      s_state = CTRL_ST_RUN;
      break;
    }

    case CTRL_ST_RUN:
    default:
    {
      uint16_t legacy_torque = torque_pct_to_legacy_command(torque);
      out->torque_pct = torque;   /* Only propagate torque in RUN state */
      can_msg_t mode_cmd, torque_cmd;
      build_inv_mode_cmd(0x06u, &mode_cmd);
      build_inv_torque_cmd(legacy_torque, &torque_cmd);
      out_push(out, &mode_cmd);
      out_push(out, &torque_cmd);
      break;
    }
  }
}
