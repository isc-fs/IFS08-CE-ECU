#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "app_state.h"

void Telemetry_Build32(const app_inputs_t *in, uint8_t out32[32]);

/* Hook: implement in your project (UART/nRF24/etc.). Default is weak no-op. */
void Telemetry_Send32(const uint8_t payload[32]);

#endif /* TELEMETRY_H */
