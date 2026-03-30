#ifndef TEST_MOCKS_H
#define TEST_MOCKS_H

#include <stdint.h>
#include <string.h>
#include "app_state.h"
#include "can.h"

/*
 * Mock/Stub layer para tests unit en host.
 * Reemplaza dependencias de RTOS, HAL, ISRs.
 */

/* ===== Mocks de Input ===== */

/**
 * Crea input nominal (sin throttle, sin freno, todo en range)
 */
app_inputs_t mock_input_nominal(void);

/**
 * Crea input con freno activado
 */
app_inputs_t mock_input_brake_engaged(void);

/**
 * Crea input con throttle al máximo
 */
app_inputs_t mock_input_throttle_max(void);

/**
 * Crea input con throttle a 50%
 */
app_inputs_t mock_input_throttle_50pct(void);

/**
 * Crea input con inversor en estado inválido
 */
app_inputs_t mock_input_inverter_fault(void);

/**
 * Crea input con tensión DC fuera de rango
 */
app_inputs_t mock_input_vdc_oob(void);

/* ===== Mocks de CAN ===== */

/**
 * Crea un frame CAN sintético con ID y payload dados
 */
can_msg_t mock_can_frame(uint32_t id, const uint8_t data[8]);

/**
 * Frame de ACK de precarga (ID 0x20)
 */
can_msg_t mock_can_precarga_ack(uint8_t ack);

/**
 * Frame de tensión DC del bus (ID 0x100, 400V = 0x0190 LE)
 */
can_msg_t mock_can_dc_bus_voltage(uint16_t voltage_raw);

/**
 * Frame de sensor S1 aceleración (ID 0x101)
 */
can_msg_t mock_can_throttle_s1(uint16_t adc_value);

/**
 * Frame de sensor S2 aceleración (ID 0x102)
 */
can_msg_t mock_can_throttle_s2(uint16_t adc_value);

/**
 * Frame de sensor freno (ID 0x103)
 */
can_msg_t mock_can_brake(uint16_t adc_value);

/**
 * Frame de tensión mínima celda (ID 0x12C)
 */
can_msg_t mock_can_cell_min_voltage(uint16_t voltage_raw);

/* ===== Mocks de RTOS ===== */

/**
 * Mock del tick del kernel RTOS
 */
extern uint32_t mock_kernel_tick_ms;

/**
 * Mock de osKernelGetTickCount() para tests
 */
uint32_t osKernelGetTickCount(void);

/**
 * Avanzar el tick simulado N ms
 */
void mock_tick_advance(uint32_t ms);

/**
 * Resetear el tick a 0
 */
void mock_tick_reset(void);

/* ===== Helpers para Assertion ===== */

/**
 * Verificar que dos frames CAN son idénticos
 */
int mock_can_frames_equal(const can_msg_t *a, const can_msg_t *b);

/**
 * Verificar que dos inputs son iguales (comparación de campos clave)
 */
int mock_app_inputs_equal(const app_inputs_t *a, const app_inputs_t *b);

#endif /* TEST_MOCKS_H */
