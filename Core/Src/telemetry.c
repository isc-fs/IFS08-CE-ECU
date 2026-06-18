#include "telemetry.h"
#include "diag.h"
#include <string.h>

static uint16_t s_telemetry_sequence;

#define TELEMETRY_RADIO_MAGIC 0xECu
#define TELEMETRY_RADIO_VERSION 0x02u
#define TELEMETRY_DUMMY_SNAPSHOT 1u

static void telemetry_pack_u16le(uint16_t value, uint8_t out[2])
{
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void telemetry_pack_u32le(uint32_t value, uint8_t out[4])
{
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
  out[2] = (uint8_t)((value >> 16) & 0xFFu);
  out[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void telemetry_pack_s16le(int16_t value, uint8_t out[2])
{
  telemetry_pack_u16le((uint16_t)value, out);
}

static void telemetry_pack_s32le(int32_t value, uint8_t out[4])
{
  telemetry_pack_u32le((uint32_t)value, out);
}

static void telemetry_apply_dummy_snapshot(telemetry_snapshot_t *snapshot)
{
  uint32_t phase;
  uint8_t i;

  if (!snapshot) return;

  phase = (snapshot->tick_ms / 100u) % 40u;

  snapshot->ecu.fsm_state = 4u;
  snapshot->ecu.boton_arranque = 1u;
  snapshot->ecu.s1_aceleracion = (uint16_t)(2100u + ((phase % 10u) * 70u));
  snapshot->ecu.s2_aceleracion = (uint16_t)(2050u + ((phase % 10u) * 65u));
  snapshot->ecu.s_freno = (phase >= 30u) ? 3200u : 500u;
  snapshot->ecu.torque_total = (phase >= 30u) ? 0u : (uint16_t)(20u + ((phase % 10u) * 6u));
  snapshot->ecu.flag_ev_2_3 = 0u;
  snapshot->ecu.flag_t11_8_9 = 0u;

  snapshot->ams.ok_precarga = 1u;
  snapshot->ams.ams_state = 3u;
  snapshot->ams.soc = (uint8_t)(78u - ((phase / 5u) % 5u));
  snapshot->ams.corriente_accu = (int16_t)(45 + ((int16_t)(phase % 8u) * 3));
  snapshot->ams.corriente_dcdc = (int16_t)(8 + (int16_t)(phase % 4u));
  snapshot->ams.temp_dcdc = (int16_t)(34 + (int16_t)(phase % 6u));

  for (i = 0u; i < 5u; i++)
  {
    snapshot->ams.vmin_modulo[i] = (uint16_t)(3560u + (i * 4u) + (phase % 5u));
    snapshot->ams.vmax_modulo[i] = (uint16_t)(3640u + (i * 4u) + (phase % 5u));
    snapshot->ams.temp_max_modulo[i] = (int16_t)(28 + (i * 3) + (int16_t)(phase % 4u));
  }
  snapshot->ams.v_celda_min = snapshot->ams.vmin_modulo[0];

  snapshot->inverter.inv_state = 6u;
  snapshot->inverter.inv_vdc_ready = 1u;
  snapshot->inverter.inv_error = 0u;
  snapshot->inverter.inv_dc_bus_voltage = (uint16_t)(392u + (phase % 12u));
  snapshot->inverter.inv_motor_temp = (int16_t)(46 + (int16_t)(phase % 7u));
  snapshot->inverter.inv_igbt_temp = (int16_t)(42 + (int16_t)(phase % 6u));
  snapshot->inverter.inv_air_temp = (int16_t)(31 + (int16_t)(phase % 5u));
  snapshot->inverter.inv_rpm = (int32_t)(3200 + ((int32_t)(phase % 12u) * 180));
  snapshot->inverter.inv_speed_actual = (int32_t)(28 + ((int32_t)(phase % 12u) * 2));
  snapshot->inverter.inv_current_actual = (int32_t)(38 + ((int32_t)(phase % 10u) * 4));

  snapshot->gps.speed = (uint16_t)(42u + (phase % 15u));
  snapshot->gps.course_deg = (uint16_t)((phase * 9u) % 360u);
  snapshot->gps.altitude = 640;
  snapshot->gps.fix_type = 3u;
  snapshot->gps.sat_count = 11u;
  snapshot->gps.hdop = 85u;
  snapshot->gps.latitude = 40416500 + (int32_t)(phase * 20u);
  snapshot->gps.longitude = -3703500 - (int32_t)(phase * 20u);
}

static void telemetry_pack_snapshot_compat_v2(const telemetry_snapshot_t *snapshot, uint8_t out32[32])
{
  memset(out32, 0, 32);
  if (!snapshot) return;

  out32[0] = snapshot->inverter.inv_state;
  out32[1] = (uint8_t)(snapshot->ecu.torque_total & 0xFFu);

  telemetry_pack_u16le(snapshot->inverter.inv_dc_bus_voltage, &out32[2]);
  telemetry_pack_u16le(snapshot->ams.v_celda_min, &out32[4]);
  telemetry_pack_u16le(snapshot->ecu.s1_aceleracion, &out32[6]);
  telemetry_pack_u16le(snapshot->ecu.s2_aceleracion, &out32[8]);
  telemetry_pack_u16le(snapshot->ecu.s_freno, &out32[10]);

  out32[12] = snapshot->ecu.flag_ev_2_3;
  out32[13] = snapshot->ecu.flag_t11_8_9;
  out32[14] = snapshot->ams.ok_precarga;
  out32[15] = snapshot->ecu.boton_arranque;
}

static telemetry_event_type_t telemetry_classify_state_event(uint8_t inv_state)
{
  switch (inv_state)
  {
    case 10u:
    case 11u:
      return TELEMETRY_EVENT_FAULT;

    case 13u:
      return TELEMETRY_EVENT_SHUTDOWN;

    default:
      return TELEMETRY_EVENT_STATE_CHANGE;
  }
}

void Telemetry_Init(void)
{
  s_telemetry_sequence = 0u;
  if (telemetryEventQueueHandle)
  {
    (void)osMessageQueueReset(telemetryEventQueueHandle);
  }
}

void Telemetry_BuildFrame(const app_inputs_t *in,
                          const telemetry_event_t *event,
                          telemetry_frame_t *out)
{
  Telemetry_BuildFrameWithKind(in,
                               event,
                               event ? TELEMETRY_FRAME_RF_EVENT : TELEMETRY_FRAME_DASH,
                               out);
}

void Telemetry_BuildSnapshot(const app_inputs_t *in,
                             uint16_t sequence,
                             telemetry_snapshot_t *out)
{
  if (!out) return;

  memset(out, 0, sizeof(*out));
  out->tick_ms = osKernelGetTickCount();
  out->sequence = sequence;

  if (!in) return;

  out->ecu.fsm_state = in->fsm_state;
  out->ecu.boton_arranque = in->boton_arranque;
  out->ecu.s1_aceleracion = in->s1_aceleracion;
  out->ecu.s2_aceleracion = in->s2_aceleracion;
  out->ecu.s_freno = in->s_freno;
  out->ecu.torque_total = in->torque_total;
  out->ecu.flag_ev_2_3 = in->flag_EV_2_3;
  out->ecu.flag_t11_8_9 = in->flag_T11_8_9;

  out->ams.ok_precarga = in->ok_precarga;
  out->ams.ams_state = in->ams_state;
  out->ams.v_celda_min = in->v_celda_min;
  out->ams.soc = in->soc;
  memcpy(out->ams.vmin_modulo, in->vmin_modulo, sizeof(out->ams.vmin_modulo));
  memcpy(out->ams.vmax_modulo, in->vmax_modulo, sizeof(out->ams.vmax_modulo));
  out->ams.corriente_accu = in->corriente_accu;
  out->ams.corriente_dcdc = in->corriente_dcdc;
  out->ams.temp_dcdc = in->temp_dcdc;
  memcpy(out->ams.temp_max_modulo, in->temp_max_modulo, sizeof(out->ams.temp_max_modulo));

  out->inverter.inv_state = in->inv_state;
  out->inverter.inv_vdc_ready = in->inv_vdc_ready;
  out->inverter.inv_error = in->inv_error;
  out->inverter.inv_dc_bus_voltage = in->inv_dc_bus_voltage;
  out->inverter.inv_motor_temp = in->inv_motor_temp;
  out->inverter.inv_igbt_temp = in->inv_igbt_temp;
  out->inverter.inv_air_temp = in->inv_air_temp;
  out->inverter.inv_rpm = in->inv_rpm;
  out->inverter.inv_speed_actual = in->inv_speed_actual;
  out->inverter.inv_current_actual = in->inv_current_actual;

  out->gps.speed = in->gps_speed;
  out->gps.course_deg = in->gps_course_deg;
  out->gps.altitude = in->gps_altitude;
  out->gps.fix_type = in->gps_fix_type;
  out->gps.sat_count = in->gps_sat_count;
  out->gps.hdop = in->gps_hdop;
  out->gps.latitude = in->gps_latitude;
  out->gps.longitude = in->gps_longitude;

#if TELEMETRY_DUMMY_SNAPSHOT
  telemetry_apply_dummy_snapshot(out);
#endif
}

void Telemetry_BuildFrameWithKind(const app_inputs_t *in,
                                  const telemetry_event_t *event,
                                  telemetry_frame_kind_t kind,
                                  telemetry_frame_t *out)
{
  if (!out) return;

  memset(out, 0, sizeof(*out));
  out->kind = kind;
  out->sequence = s_telemetry_sequence++;
  Telemetry_BuildSnapshot(in, out->sequence, &out->snapshot);

  if (event)
  {
    out->event = *event;
  }
}

void Telemetry_SerializeFrame(const telemetry_frame_t *frame, uint8_t out32[32])
{
  memset(out32, 0, 32);
  if (!frame) return;

  telemetry_pack_snapshot_compat_v2(&frame->snapshot, out32);

  out32[16] = 1u;
  out32[17] = (uint8_t)frame->kind;
  out32[18] = (uint8_t)frame->event.type;
  out32[19] = frame->event.previous_inv_state;
  out32[20] = frame->event.current_inv_state;
  out32[21] = frame->event.inv_error;
  telemetry_pack_u16le(frame->sequence, &out32[22]);
  telemetry_pack_u32le(frame->event.tick_ms, &out32[24]);
  out32[28] = frame->snapshot.inverter.inv_error;
  out32[29] = frame->snapshot.inverter.inv_vdc_ready;
}

void Telemetry_SerializeRadioFragment(const telemetry_frame_t *frame,
                                      uint8_t fragment_index,
                                      uint8_t out32[32])
{
  memset(out32, 0, 32);
  if (!frame) return;

  out32[0] = TELEMETRY_RADIO_MAGIC;
  out32[1] = TELEMETRY_RADIO_VERSION;
  out32[2] = fragment_index;
  out32[3] = Telemetry_RadioFragmentCount(frame);
  telemetry_pack_u16le(frame->sequence, &out32[4]);
  out32[6] = (uint8_t)frame->kind;
  out32[7] = (uint8_t)frame->event.type;

  if (frame->kind == TELEMETRY_FRAME_RF_EVENT)
  {
    out32[8] = frame->snapshot.inverter.inv_state;
    out32[9] = frame->snapshot.ecu.flag_ev_2_3;
    out32[10] = frame->snapshot.ecu.flag_t11_8_9;
    out32[11] = frame->snapshot.inverter.inv_error;
    out32[12] = frame->event.previous_inv_state;
    out32[13] = frame->event.current_inv_state;
    out32[14] = frame->event.inv_error;
    out32[15] = frame->snapshot.ams.ok_precarga;
    out32[16] = frame->snapshot.inverter.inv_vdc_ready;
    telemetry_pack_u16le(frame->snapshot.ecu.torque_total, &out32[17]);
    telemetry_pack_u16le(frame->snapshot.inverter.inv_dc_bus_voltage, &out32[19]);
    telemetry_pack_u16le(frame->snapshot.ams.v_celda_min, &out32[21]);
    telemetry_pack_u32le(frame->event.tick_ms, &out32[23]);
    out32[27] = frame->snapshot.ecu.fsm_state;
    out32[28] = frame->snapshot.ams.ams_state;
    return;
  }

  if (frame->kind == TELEMETRY_FRAME_RF_SLOW)
  {
    if (fragment_index == 0u)
    {
      out32[8] = frame->snapshot.ams.soc;
      telemetry_pack_s16le(frame->snapshot.ams.corriente_accu, &out32[9]);
      telemetry_pack_s16le(frame->snapshot.ams.corriente_dcdc, &out32[11]);
      telemetry_pack_s16le(frame->snapshot.ams.temp_dcdc, &out32[13]);
      telemetry_pack_u16le(frame->snapshot.gps.speed, &out32[15]);
      telemetry_pack_u16le(frame->snapshot.gps.course_deg, &out32[17]);
      telemetry_pack_s32le(frame->snapshot.gps.altitude, &out32[19]);
      telemetry_pack_u32le(frame->snapshot.tick_ms, &out32[23]);
      out32[27] = frame->snapshot.gps.fix_type;
      out32[28] = frame->snapshot.gps.sat_count;
      telemetry_pack_u16le(frame->snapshot.gps.hdop, &out32[29]);
      return;
    }

    if (fragment_index == 1u)
    {
      telemetry_pack_s32le(frame->snapshot.gps.latitude, &out32[8]);
      telemetry_pack_s32le(frame->snapshot.gps.longitude, &out32[12]);
      return;
    }

    if (fragment_index == 2u)
    {
      telemetry_pack_u16le(frame->snapshot.ams.vmin_modulo[0], &out32[8]);
      telemetry_pack_u16le(frame->snapshot.ams.vmin_modulo[1], &out32[10]);
      telemetry_pack_u16le(frame->snapshot.ams.vmin_modulo[2], &out32[12]);
      telemetry_pack_u16le(frame->snapshot.ams.vmin_modulo[3], &out32[14]);
      telemetry_pack_u16le(frame->snapshot.ams.vmin_modulo[4], &out32[16]);
      return;
    }

    if (fragment_index == 3u)
    {
      telemetry_pack_u16le(frame->snapshot.ams.vmax_modulo[0], &out32[8]);
      telemetry_pack_u16le(frame->snapshot.ams.vmax_modulo[1], &out32[10]);
      telemetry_pack_u16le(frame->snapshot.ams.vmax_modulo[2], &out32[12]);
      telemetry_pack_u16le(frame->snapshot.ams.vmax_modulo[3], &out32[14]);
      telemetry_pack_u16le(frame->snapshot.ams.vmax_modulo[4], &out32[16]);
      return;
    }

    telemetry_pack_s16le(frame->snapshot.ams.temp_max_modulo[0], &out32[8]);
    telemetry_pack_s16le(frame->snapshot.ams.temp_max_modulo[1], &out32[10]);
    telemetry_pack_s16le(frame->snapshot.ams.temp_max_modulo[2], &out32[12]);
    telemetry_pack_s16le(frame->snapshot.ams.temp_max_modulo[3], &out32[14]);
    telemetry_pack_s16le(frame->snapshot.ams.temp_max_modulo[4], &out32[16]);
    return;
  }

  if (fragment_index == 0u)
  {
    out32[8] = frame->snapshot.ecu.fsm_state;
    out32[9] = frame->snapshot.inverter.inv_state;
    out32[10] = frame->snapshot.ams.ams_state;
    out32[11] = frame->snapshot.ecu.boton_arranque;
    out32[12] = frame->snapshot.ams.ok_precarga;
    out32[13] = frame->snapshot.ecu.flag_ev_2_3;
    out32[14] = frame->snapshot.ecu.flag_t11_8_9;
    out32[15] = frame->snapshot.inverter.inv_vdc_ready;
    out32[16] = frame->snapshot.inverter.inv_error;
    telemetry_pack_u16le(frame->snapshot.ecu.torque_total, &out32[17]);
    telemetry_pack_u16le(frame->snapshot.inverter.inv_dc_bus_voltage, &out32[19]);
    telemetry_pack_u16le(frame->snapshot.ams.v_celda_min, &out32[21]);
    telemetry_pack_u16le(frame->snapshot.ecu.s1_aceleracion, &out32[23]);
    telemetry_pack_u16le(frame->snapshot.ecu.s2_aceleracion, &out32[25]);
    telemetry_pack_u16le(frame->snapshot.ecu.s_freno, &out32[27]);
    return;
  }

  telemetry_pack_s16le(frame->snapshot.inverter.inv_motor_temp, &out32[8]);
  telemetry_pack_s16le(frame->snapshot.inverter.inv_igbt_temp, &out32[10]);
  telemetry_pack_s16le(frame->snapshot.inverter.inv_air_temp, &out32[12]);
  telemetry_pack_s32le(frame->snapshot.inverter.inv_rpm, &out32[14]);
  telemetry_pack_s32le(frame->snapshot.inverter.inv_speed_actual, &out32[18]);
  telemetry_pack_s32le(frame->snapshot.inverter.inv_current_actual, &out32[22]);
}

void Telemetry_TransportSendFragment(const telemetry_frame_t *frame, uint8_t fragment_index)
{
  uint8_t payload32[32];

  Telemetry_SerializeRadioFragment(frame, fragment_index, payload32);
  Telemetry_Send32(payload32);
}

uint8_t Telemetry_RadioFragmentCount(const telemetry_frame_t *frame)
{
  if (!frame)
  {
    return 0u;
  }

  switch (frame->kind)
  {
    case TELEMETRY_FRAME_RF_EVENT:
      return 1u;

    case TELEMETRY_FRAME_RF_SLOW:
      return 5u;

    case TELEMETRY_FRAME_RF_FAST:
      return TELEMETRY_RADIO_FRAGMENT_COUNT;

    default:
      return 0u;
  }
}

uint8_t Telemetry_EnqueueFrame(const telemetry_frame_t *frame)
{
  return Telemetry_EnqueueFrameTargets(frame, 1u, 1u, 1u);
}

uint8_t Telemetry_EnqueueFrameTargets(const telemetry_frame_t *frame,
                                      uint8_t to_radio,
                                      uint8_t to_sd,
                                      uint8_t to_dash)
{
  uint8_t queued = 1u;
  static uint32_t s_radio_enqueue_ok_count = 0u;

  if (!frame)
  {
    return 0u;
  }

  if (to_radio &&
      telemetryRadioQueueHandle &&
      osMessageQueuePut(telemetryRadioQueueHandle, frame, 0u, 0u) != osOK)
  {
    Diag_Log("TELQ RADIO PUT FAIL seq=%u kind=%u count=%lu space=%lu",
             (unsigned)frame->sequence,
             (unsigned)frame->kind,
             (unsigned long)osMessageQueueGetCount(telemetryRadioQueueHandle),
             (unsigned long)osMessageQueueGetSpace(telemetryRadioQueueHandle));
    queued = 0u;
  }
  else if (to_radio)
  {
    s_radio_enqueue_ok_count++;
    if ((s_radio_enqueue_ok_count <= 10u) || ((s_radio_enqueue_ok_count % 20u) == 0u))
    {
      Diag_Log("TELQ RADIO PUT #%lu seq=%u kind=%u count=%lu space=%lu",
               (unsigned long)s_radio_enqueue_ok_count,
               (unsigned)frame->sequence,
               (unsigned)frame->kind,
               (unsigned long)osMessageQueueGetCount(telemetryRadioQueueHandle),
               (unsigned long)osMessageQueueGetSpace(telemetryRadioQueueHandle));
    }
  }

  if (to_sd &&
      telemetrySdQueueHandle &&
      osMessageQueuePut(telemetrySdQueueHandle, frame, 0u, 0u) != osOK)
  {
    queued = 0u;
  }

  if (to_dash &&
      telemetryDashQueueHandle &&
      osMessageQueuePut(telemetryDashQueueHandle, frame, 0u, 0u) != osOK)
  {
    queued = 0u;
  }

  return queued;
}

uint8_t Telemetry_EventPublish(const telemetry_event_t *event)
{
  if (!event || !telemetryEventQueueHandle)
  {
    return 0u;
  }

  return (uint8_t)(osMessageQueuePut(telemetryEventQueueHandle, event, 0u, 0u) == osOK);
}

uint8_t Telemetry_PublishStateEvent(uint8_t previous_inv_state,
                                    uint8_t previous_inv_error,
                                    const app_inputs_t *after)
{
  telemetry_event_t event;

  if (!after) return 0u;
  if (previous_inv_state == after->inv_state)
  {
    return 0u;
  }

  memset(&event, 0, sizeof(event));
  event.type = telemetry_classify_state_event(after->inv_state);
  event.previous_inv_state = previous_inv_state;
  event.current_inv_state = after->inv_state;
  event.inv_error = (after->inv_error != 0u) ? after->inv_error : previous_inv_error;
  event.tick_ms = osKernelGetTickCount();

  return Telemetry_EventPublish(&event);
}

void Telemetry_Build32(const app_inputs_t *in, uint8_t out32[32])
{
  telemetry_snapshot_t snapshot;
  Telemetry_BuildSnapshot(in, 0u, &snapshot);
  telemetry_pack_snapshot_compat_v2(&snapshot, out32);
}

__attribute__((weak)) void Telemetry_Send32(const uint8_t payload[32])
{
  (void)payload;
  /* Implement transport (nRF24/UART/etc.) in your project. */
}

__attribute__((weak)) void Telemetry_SdStore32(const uint8_t payload[32])
{
  (void)payload;
  /* Implement SD logging transport in your project. */
}
