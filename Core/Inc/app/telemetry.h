/* SPDX-License-Identifier: proprietary */
#ifndef ECU_APP_TELEMETRY_H
#define ECU_APP_TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { ECU_TELEMETRY_PAYLOAD_SIZE = 32 };

typedef struct {
    uint8_t payload[ECU_TELEMETRY_PAYLOAD_SIZE];
} EcuTelemetryFrame;

typedef struct {
    uint8_t payload[ECU_TELEMETRY_PAYLOAD_SIZE];
} EcuRadioPacket;

void Telemetry_RadioSend(const uint8_t payload[ECU_TELEMETRY_PAYLOAD_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* ECU_APP_TELEMETRY_H */
