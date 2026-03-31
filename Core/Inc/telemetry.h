#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "app_state.h"

typedef enum
{
  TELEMETRY_FRAME_HEARTBEAT = 1,
  TELEMETRY_FRAME_EVENT = 2
} telemetry_frame_kind_t;

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

void Telemetry_Init(void);
void Telemetry_BuildFrame(const app_inputs_t *in,
                          const telemetry_event_t *event,
                          telemetry_frame_t *out);
void Telemetry_SerializeFrame(const telemetry_frame_t *frame, uint8_t out32[32]);
void Telemetry_TransportSend(const telemetry_frame_t *frame);
uint8_t Telemetry_EventPublish(const telemetry_event_t *event);
uint8_t Telemetry_PublishStateEvent(uint8_t previous_inv_state,
                                    uint8_t previous_inv_error,
                                    const app_inputs_t *after);

void Telemetry_Build32(const app_inputs_t *in, uint8_t out32[32]);

/* Hook: implement in your project (UART/nRF24/etc.). Default is weak no-op. */
void Telemetry_Send32(const uint8_t payload[32]);

#endif /* TELEMETRY_H */
