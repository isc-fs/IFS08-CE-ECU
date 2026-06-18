#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "app_state.h"

typedef enum
{
  TELEMETRY_FRAME_DASH = 1,
  TELEMETRY_FRAME_SD = 2,
  TELEMETRY_FRAME_RF_FAST = 3,
  TELEMETRY_FRAME_RF_SLOW = 4,
  TELEMETRY_FRAME_RF_EVENT = 5
} telemetry_frame_kind_t;

/* Compatibility alias used by existing SIL/unit tests. */
#define TELEMETRY_FRAME_HEARTBEAT TELEMETRY_FRAME_DASH
#define TELEMETRY_FRAME_EVENT TELEMETRY_FRAME_RF_EVENT

typedef enum
{
  TELEMETRY_EVENT_NONE = 0,
  TELEMETRY_EVENT_STATE_CHANGE = 1,
  TELEMETRY_EVENT_FAULT = 2,
  TELEMETRY_EVENT_SHUTDOWN = 3
} telemetry_event_type_t;

typedef struct
{
  telemetry_event_type_t type;
  uint8_t previous_inv_state;
  uint8_t current_inv_state;
  uint8_t inv_error;
  uint32_t tick_ms;
} telemetry_event_t;

typedef struct
{
  uint32_t tick_ms;
  uint16_t sequence;

  struct
  {
    uint8_t boton_arranque;
    uint16_t s1_aceleracion;
    uint16_t s2_aceleracion;
    uint16_t s_freno;
    uint16_t torque_total;
    uint8_t flag_ev_2_3;
    uint8_t flag_t11_8_9;
    uint8_t fsm_state;
  } ecu;

  struct
  {
    uint8_t ok_precarga;
    uint8_t ams_state;
    uint16_t v_celda_min;
    uint8_t soc;
    uint16_t vmin_modulo[5];
    uint16_t vmax_modulo[5];
    int16_t corriente_accu;
    int16_t corriente_dcdc;
    int16_t temp_dcdc;
    int16_t temp_max_modulo[5];
  } ams;

  struct
  {
    uint8_t inv_state;
    uint8_t inv_vdc_ready;
    uint8_t inv_error;
    uint16_t inv_dc_bus_voltage;
    int16_t inv_motor_temp;
    int16_t inv_igbt_temp;
    int16_t inv_air_temp;
    int32_t inv_rpm;
    int32_t inv_speed_actual;
    int32_t inv_current_actual;
  } inverter;

  struct
  {
    uint16_t speed;
    uint16_t course_deg;
    int32_t altitude;
    uint8_t fix_type;
    uint8_t sat_count;
    uint16_t hdop;
    int32_t latitude;
    int32_t longitude;
  } gps;
} telemetry_snapshot_t;

typedef struct
{
  telemetry_frame_kind_t kind;
  telemetry_event_t event;
  telemetry_snapshot_t snapshot;
  uint16_t sequence;
} telemetry_frame_t;

extern osMessageQueueId_t telemetryEventQueueHandle;
extern osMessageQueueId_t telemetryRadioQueueHandle;
extern osMessageQueueId_t telemetrySdQueueHandle;
extern osMessageQueueId_t telemetryDashQueueHandle;

#define TELEMETRY_RADIO_FRAGMENT_COUNT 2u

void Telemetry_Init(void);
void Telemetry_BuildFrame(const app_inputs_t *in,
                          const telemetry_event_t *event,
                          telemetry_frame_t *out);
void Telemetry_BuildFrameWithKind(const app_inputs_t *in,
                                  const telemetry_event_t *event,
                                  telemetry_frame_kind_t kind,
                                  telemetry_frame_t *out);
void Telemetry_BuildSnapshot(const app_inputs_t *in,
                             uint16_t sequence,
                             telemetry_snapshot_t *out);
void Telemetry_SerializeFrame(const telemetry_frame_t *frame, uint8_t out32[32]);
void Telemetry_SerializeRadioFragment(const telemetry_frame_t *frame,
                                      uint8_t fragment_index,
                                      uint8_t out32[32]);
void Telemetry_TransportSendFragment(const telemetry_frame_t *frame, uint8_t fragment_index);
uint8_t Telemetry_RadioFragmentCount(const telemetry_frame_t *frame);
uint8_t Telemetry_EnqueueFrame(const telemetry_frame_t *frame);
uint8_t Telemetry_EnqueueFrameTargets(const telemetry_frame_t *frame,
                                      uint8_t to_radio,
                                      uint8_t to_sd,
                                      uint8_t to_dash);
uint8_t Telemetry_EventPublish(const telemetry_event_t *event);
uint8_t Telemetry_PublishStateEvent(uint8_t previous_inv_state,
                                    uint8_t previous_inv_error,
                                    const app_inputs_t *after);

void Telemetry_Build32(const app_inputs_t *in, uint8_t out32[32]);

/* Hook: implement in your project (UART/nRF24/etc.). Default is weak no-op. */
void Telemetry_Send32(const uint8_t payload[32]);
void Telemetry_SdStore32(const uint8_t payload[32]);

#endif /* TELEMETRY_H */
