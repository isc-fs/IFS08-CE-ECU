/**
 * sil_can_simulator.h
 * CAN message simulator for SIL
 */

#ifndef SIL_CAN_SIMULATOR_H
#define SIL_CAN_SIMULATOR_H

#include <stdint.h>

/* Initialize CAN simulator */
void SIL_CAN_Init(void);

/* Inject simulated throttle input (0-100%) */
void SIL_CAN_InjectThrottle(uint8_t throttle_pct);

/* Inject simulated brake input (0-100%) */
void SIL_CAN_InjectBrake(uint8_t brake_pct);

/* Inject simulated inverter state */
void SIL_CAN_InjectInverterState(uint8_t state);

/* Inject simulated DC bus voltage (in volts, e.g., 400) */
void SIL_CAN_InjectDCVoltage(uint16_t voltage_v);

/* Process pending CAN messages */
void SIL_CAN_Process(void);

#endif /* SIL_CAN_SIMULATOR_H */
