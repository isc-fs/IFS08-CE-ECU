#include "telemetry.h"
#include <string.h>

static uint16_t s_telemetry_sequence;

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

static void telemetry_pack_snapshot_compat(const app_inputs_t *in, uint8_t out32[32])
{
  memset(out32, 0, 32);
  if (!in) return;

  out32[0] = in->inv_state;
  out32[1] = (uint8_t)(in->torque_total & 0xFFu);

  telemetry_pack_u16le(in->inv_dc_bus_voltage, &out32[2]);
  telemetry_pack_u16le(in->v_celda_min, &out32[4]);
  telemetry_pack_u16le(in->s1_aceleracion, &out32[6]);
  telemetry_pack_u16le(in->s2_aceleracion, &out32[8]);
  telemetry_pack_u16le(in->s_freno, &out32[10]);

  out32[12] = in->flag_EV_2_3;
  out32[13] = in->flag_T11_8_9;
  out32[14] = in->ok_precarga;
  out32[15] = in->boton_arranque;
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
  if (!out) return;

  memset(out, 0, sizeof(*out));
  out->kind = event ? TELEMETRY_FRAME_EVENT : TELEMETRY_FRAME_HEARTBEAT;
  out->sequence = s_telemetry_sequence++;

  if (in)
  {
    out->snapshot = *in;
  }

  if (event)
  {
    out->event = *event;
  }
}

void Telemetry_SerializeFrame(const telemetry_frame_t *frame, uint8_t out32[32])
{
  memset(out32, 0, 32);
  if (!frame) return;

  telemetry_pack_snapshot_compat(&frame->snapshot, out32);

  out32[16] = 1u;
  out32[17] = (uint8_t)frame->kind;
  out32[18] = (uint8_t)frame->event.type;
  out32[19] = frame->event.previous_inv_state;
  out32[20] = frame->event.current_inv_state;
  out32[21] = frame->event.inv_error;
  telemetry_pack_u16le(frame->sequence, &out32[22]);
  telemetry_pack_u32le(frame->event.tick_ms, &out32[24]);
  out32[28] = frame->snapshot.inv_error;
  out32[29] = frame->snapshot.inv_vdc_ready;
}

void Telemetry_TransportSend(const telemetry_frame_t *frame)
{
  uint8_t payload32[32];

  Telemetry_SerializeFrame(frame, payload32);
  Telemetry_Send32(payload32);
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
  telemetry_pack_snapshot_compat(in, out32);
}

__attribute__((weak)) void Telemetry_Send32(const uint8_t payload[32])
{
  (void)payload;
  /* Implement transport (nRF24/UART/etc.) in your project. */
}
