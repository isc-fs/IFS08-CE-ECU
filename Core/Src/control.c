#include "control.h"
#include <string.h>

/* APPS calibration — measure raw ADC at pedal min and max when mounted.
 * Formula: pct = (raw - MIN) / (MAX - MIN) * 100  →  saturated 0..100. */
#define APPS1_ADC_MIN  2050u   /* raw ADC at 0% pedal travel, sensor 1 — PENDING CALIBRATION */
#define APPS1_ADC_MAX  2950u   /* raw ADC at 100% pedal travel, sensor 1 — PENDING CALIBRATION */
#define APPS2_ADC_MIN  1915u   /* raw ADC at 0% pedal travel, sensor 2 — PENDING CALIBRATION */
#define APPS2_ADC_MAX  2570u   /* raw ADC at 100% pedal travel, sensor 2 — PENDING CALIBRATION */

/* Thresholds */
#define UMBRAL_FRENO_APPS 3000u
#define UMBRAL_FRENO_ARRANQUE 900u
#define ID_DC_BUS_VOLTAGE 0x100u
#define ID_PRECHARGE_CMD  0x600u
#define RX_SETPOINT_1     0x360u
#define RX_SETPOINT_3     0x362u
#define INV_MODE_STANDBY  0x01u
#define INV_MODE_READY    0x04u
#define INV_MODE_TORQUE   0x06u
#define INV_MODE_RESET    0x13u
#define VMIN_FULL_TORQUE  3500u
#define VMIN_MIN_TORQUE   2800u
#define PRECHARGE_TIMEOUT_MS 10000u

/* Very small helper */
static void out_push(control_out_t *out, const can_msg_t *m)
{
  if (!out || !m) return;
  if (out->count >= (uint8_t)(sizeof(out->msgs) / sizeof(out->msgs[0]))) return;
  out->msgs[out->count++] = *m;
}

/* Control states: cooperative replacement for blocking while-loops in main.c */
typedef enum
{
  CTRL_ST_WAIT_INV_VDC_CONFIG = 0,
  CTRL_ST_BOOT,
  CTRL_ST_WAIT_PRECHARGE_ACK,
  CTRL_ST_WAIT_START_BRAKE,
  CTRL_ST_R2D_DELAY,
  CTRL_ST_WAIT_INV_STANDBY,
  CTRL_ST_ACTIVE
} ctrl_state_t;

static ctrl_state_t s_state;
static uint32_t s_r2d_start_tick;
static uint32_t s_precharge_start_tick;
static uint8_t s_ev23_latched;
static uint8_t s_flag_r2d;
static uint8_t s_flag_react;

void Control_Init(void)
{
  s_state = CTRL_ST_WAIT_INV_VDC_CONFIG;
  s_r2d_start_tick = 0u;
  s_precharge_start_tick = 0u;
  s_ev23_latched = 0u;
  s_flag_r2d = 0u;
  s_flag_react = 0u;
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
  m->id = RX_SETPOINT_1;
  m->dlc = 3u;
  m->data[2] = mode;
}

static void build_inv_torque_cmd(uint16_t legacy_torque, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_INV;
  m->id = RX_SETPOINT_3;
  m->dlc = 4u;
  m->data[2] = (uint8_t)(legacy_torque & 0xFFu);
  m->data[3] = (uint8_t)((legacy_torque >> 8) & 0xFFu);
}

static void build_acu_dc_bus_frame(uint16_t dc_bus_voltage, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_ACU;
  m->id = ID_DC_BUS_VOLTAGE;
  m->dlc = 2u;
  m->ide = 0u;
  m->data[0] = (uint8_t)(dc_bus_voltage & 0xFFu);
  m->data[1] = (uint8_t)((dc_bus_voltage >> 8) & 0xFFu);
}

static void build_acu_precharge_cmd(uint8_t button_pressed, can_msg_t *m)
{
  memset(m, 0, sizeof(*m));
  m->bus = CAN_BUS_ACU;
  m->id = ID_PRECHARGE_CMD;
  m->dlc = 2u;
  m->ide = 0u;
  m->data[0] = button_pressed ? 1u : 0u;
}

static uint8_t precharge_complete(const app_inputs_t *in)
{
  if (!in) return 0u;
  /* The AMS owns the contactors and the precharge-complete decision: it closes
   * AIR+ only when the DC bus reaches >=95% of the *measured* pack, then
   * publishes ok_precarga on 0x020. The ECU must not second-guess that with a
   * fixed-voltage heuristic. The old "|| inv_dc_bus_voltage >= 300" was a
   * main_polling.c holdover (when the VCU drove precharge itself): 300 V is
   * non-adaptive (~75% of a full pack, unreachable on a low pack) and, being
   * below the AMS gate, let the ECU's startup run ahead of the contactors.
   * Gate solely on the AMS ACK; PRECHARGE_TIMEOUT_MS is the fallback if it
   * never arrives. This is an intentional divergence from main_polling.c. */
  return (uint8_t)(in->ok_precarga != 0u);
}

static uint8_t inverter_vdc_configured(const app_inputs_t *in)
{
  if (!in) return 0u;
  return (uint8_t)(in->inv_vdc_ready != 0u);
}

static void emit_inv_mode(control_out_t *out, uint8_t mode)
{
  can_msg_t msg;
  build_inv_mode_cmd(mode, &msg);
  out_push(out, &msg);
}

static void emit_inv_torque(control_out_t *out, uint16_t legacy_torque)
{
  can_msg_t msg;
  build_inv_torque_cmd(legacy_torque, &msg);
  out_push(out, &msg);
}

static void emit_inv_zero_torque(control_out_t *out)
{
  emit_inv_torque(out, 0u);
}

static uint16_t apply_vmin_torque_limit(uint16_t torque_pct, uint16_t v_celda_min)
{
  float limited = (float)torque_pct;

  if (torque_pct == 0u)
  {
    return 0u;
  }

  if (v_celda_min >= VMIN_FULL_TORQUE)
  {
    return torque_pct;
  }

  if (v_celda_min > VMIN_MIN_TORQUE)
  {
    limited *= ((1.357f * (float)v_celda_min) - 3750.0f) / 1000.0f;
  }
  else
  {
    limited *= 0.05f;
  }

  return saturate_pct(limited);
}

static void emit_legacy_inverter_runtime(const app_inputs_t *in, control_out_t *out, uint16_t torque_pct)
{
  if (!in || !out || !s_flag_r2d) return;

  if (in->inv_state == 4u || in->inv_state == 6u)
  {
    emit_inv_mode(out, INV_MODE_TORQUE);
  }

  switch (in->inv_state)
  {
    case 0u:
      emit_inv_mode(out, INV_MODE_STANDBY);
      /* Legacy switch fell through to standby/ready handling. */
      /* fallthrough */

    case 3u:
      s_flag_react = 0u;
      emit_inv_mode(out, INV_MODE_READY);
      /* Legacy switch also fell through to zero-torque handling. */
      /* fallthrough */

    case 4u:
      emit_inv_zero_torque(out);
      s_flag_react = 0u;
      break;

    case 6u:
      out->torque_pct = torque_pct;
      emit_inv_torque(out, torque_pct_to_legacy_command(torque_pct));
      break;

    case 10u:
      emit_inv_mode(out, INV_MODE_RESET);
      /* Legacy soft-fault path fell through into hard fault/shutdown commands. */
      /* fallthrough */

    case 11u:
      s_flag_react = 1u;
      emit_inv_mode(out, INV_MODE_RESET);
      /* fallthrough */

    case 13u:
      emit_inv_mode(out, INV_MODE_STANDBY);
      break;

    default:
      break;
  }
}

/* Port of legacy main_polling.c torque mapping, but exposed here as 0..100%. */
uint16_t Control_ComputeTorque(const app_inputs_t *in, uint8_t *flag_ev_2_3, uint8_t *flag_t11_8_9)
{
  uint16_t s1_pct;
  uint16_t s2_pct;
  uint16_t torque;

  if (!in) return 0u;

  s1_pct = saturate_pct(((float)in->s1_aceleracion - (float)APPS1_ADC_MIN) /
                        ((float)(APPS1_ADC_MAX - APPS1_ADC_MIN) / 100.0f));
  s2_pct = saturate_pct(((float)in->s2_aceleracion - (float)APPS2_ADC_MIN) /
                        ((float)(APPS2_ADC_MAX - APPS2_ADC_MIN) / 100.0f));

  torque = 0u;
  if (s1_pct > 8u && s2_pct > 8u)
  {
    torque = (uint16_t)((s1_pct + s2_pct) / 2u);
  }

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

  /* Legacy code computed torque_limitado from v_celda_min but never consumed it.
   * Here we apply the intended limiter to the returned torque percentage. */
  torque = apply_vmin_torque_limit(torque, in->v_celda_min);

  return torque;
}

/* Main 10ms step */
void Control_Step10ms(const app_inputs_t *in, control_out_t *out)
{
  uint8_t ev23 = 0u;
  uint8_t t1189 = 0u;
  uint8_t rerun = 0u;
  uint8_t guard = 0u;
  uint16_t torque = 0u;

  if (!in || !out) return;
  memset(out, 0, sizeof(*out));

  /* Torque computation from inputs (used only in RUN state) */
  torque = Control_ComputeTorque(in, &ev23, &t1189);
  out->flag_ev_2_3 = ev23;
  out->flag_t11_8_9 = t1189;
  /* out->torque_pct stays 0 until the inverter reaches torque state */

  for (guard = 0u; guard < 4u; guard++)
  {
    rerun = 0u;

    switch (s_state)
    {
      case CTRL_ST_WAIT_INV_VDC_CONFIG:
        if (inverter_vdc_configured(in))
        {
          s_state = CTRL_ST_BOOT;
          rerun = 1u;
        }
        break;

      case CTRL_ST_BOOT:
        if (precharge_complete(in))
        {
          s_state = CTRL_ST_WAIT_START_BRAKE;
          rerun = 1u;
        }
        else
        {
          /* 0x100 heartbeat is emitted unconditionally at the end of the
           * step (see Control_Step10ms tail); here we only add the
           * precharge command when the start button is held. */
          if (in->boton_arranque)
          {
            can_msg_t precharge_cmd;
            build_acu_precharge_cmd(in->boton_arranque, &precharge_cmd);
            out_push(out, &precharge_cmd);
          }
          s_precharge_start_tick = osKernelGetTickCount();
          s_state = CTRL_ST_WAIT_PRECHARGE_ACK;
        }
        break;

      case CTRL_ST_WAIT_PRECHARGE_ACK:
        if (precharge_complete(in))
        {
          s_state = CTRL_ST_WAIT_START_BRAKE;
          rerun = 1u;
        }
        else if ((osKernelGetTickCount() - s_precharge_start_tick) >= PRECHARGE_TIMEOUT_MS)
        {
          /* Precharge did not complete in time — retry from BOOT. */
          s_state = CTRL_ST_BOOT;
          rerun = 1u;
        }
        else
        {
          /* 0x100 heartbeat is emitted unconditionally at the end of the
           * step (see Control_Step10ms tail); here we only add the
           * precharge command when the start button is held. */
          if (in->boton_arranque)
          {
            can_msg_t precharge_cmd;
            build_acu_precharge_cmd(in->boton_arranque, &precharge_cmd);
            out_push(out, &precharge_cmd);
          }
        }
        break;

      case CTRL_ST_WAIT_START_BRAKE:
        if (in->boton_arranque && in->s_freno > UMBRAL_FRENO_ARRANQUE)
        {
          s_r2d_start_tick = osKernelGetTickCount();
          s_flag_r2d = 1u;
          out->rtds_active = 1u;
          s_state = CTRL_ST_R2D_DELAY;
        }
        break;

      case CTRL_ST_R2D_DELAY:
        if ((osKernelGetTickCount() - s_r2d_start_tick) >= 2000u)
        {
          s_state = CTRL_ST_WAIT_INV_STANDBY;
          rerun = 1u;
        }
        else
        {
          out->rtds_active = 1u;
        }
        break;

      case CTRL_ST_WAIT_INV_STANDBY:
        if (in->inv_state == 3u)
        {
          s_state = CTRL_ST_ACTIVE;
          emit_legacy_inverter_runtime(in, out, torque);
        }
        break;

      case CTRL_ST_ACTIVE:
      default:
        emit_legacy_inverter_runtime(in, out, torque);
        break;
    }

    if (rerun == 0u)
    {
      break;
    }
  }

  /* DC-bus heartbeat to the ACU/AMS — emitted every control step (10 ms) in
   * EVERY state, not only during precharge. The AMS arms a VcuStale predicate
   * the moment it locks Car mode and latches a sticky ERROR (opening the AIRs)
   * if 0x100 goes stale for >200 ms (AMS ams_config VcuStaleMs). The legacy
   * firmware sent this from the TIM16 ISR unconditionally; emitting it only
   * during precharge tripped the AMS the instant precharge completed. */
  {
    can_msg_t dc_bus_cmd;
    build_acu_dc_bus_frame(in->inv_dc_bus_voltage, &dc_bus_cmd);
    out_push(out, &dc_bus_cmd);
  }

  out->fsm_state = (uint8_t)s_state;  /* surfaced via pit-diag */
}
