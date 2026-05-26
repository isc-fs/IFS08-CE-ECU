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
  telemetry_frame_kind_t kind;
  telemetry_event_t event;
  app_inputs_t snapshot;
  uint16_t sequence;
} telemetry_frame_t;

extern osMessageQueueId_t telemetryEventQueueHandle;
extern osMessageQueueId_t telemetryRadioQueueHandle;
extern osMessageQueueId_t telemetrySdQueueHandle;
extern osMessageQueueId_t telemetryDashQueueHandle;

#define TELEMETRY_RADIO_FRAGMENT_COUNT 3u

void Telemetry_Init(void);
void Telemetry_BuildFrame(const app_inputs_t *in,
                          const telemetry_event_t *event,
                          telemetry_frame_t *out);
void Telemetry_BuildFrameWithKind(const app_inputs_t *in,
                                  const telemetry_event_t *event,
                                  telemetry_frame_kind_t kind,
                                  telemetry_frame_t *out);
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
