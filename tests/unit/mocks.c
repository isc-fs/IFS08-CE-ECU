#include "mocks.h"
#include <string.h>
#include <stdlib.h>

/* ===== Global state para mocks ===== */
uint32_t mock_kernel_tick_ms = 0;

/* ===== Mock Inputs ===== */

app_inputs_t mock_input_nominal(void)
{
    app_inputs_t in = {
        .s1_aceleracion = 2100,      /* ~0% throttle */
        .s2_aceleracion = 1960,
        .s_freno = 2000,              /* Sin freno */
        .boton_arranque = 0,
        .inv_state = 0x02,            /* READY */
        .inv_dc_bus_voltage = 400,    /* Nominal 400V */
        .inv_motor_temp = 20,
        .inv_igbt_temp = 25,
        .inv_air_temp = 30,
        .inv_rpm = 0,
        .v_celda_min = 150,           /* Nominal celda */
        .ok_precarga = 1,             /* ACK recibido */
        .flag_EV_2_3 = 0,
        .flag_T11_8_9 = 0,
        .torque_total = 0
    };
    return in;
}

app_inputs_t mock_input_brake_engaged(void)
{
    app_inputs_t in = mock_input_nominal();
    in.s_freno = 4500;  /* Freno activado (>UMBRAL_FRENO_APPS) */
    return in;
}

app_inputs_t mock_input_throttle_max(void)
{
    app_inputs_t in = mock_input_nominal();
    in.s1_aceleracion = 3100;  /* Máximo */
    in.s2_aceleracion = 2600;
    return in;
}

app_inputs_t mock_input_throttle_50pct(void)
{
    app_inputs_t in = mock_input_nominal();
    in.s1_aceleracion = 2550;  /* ~50% */
    in.s2_aceleracion = 2435;
    return in;
}

app_inputs_t mock_input_inverter_fault(void)
{
    app_inputs_t in = mock_input_nominal();
    in.inv_state = 0x99;  /* Estado inválido */
    return in;
}

app_inputs_t mock_input_vdc_oob(void)
{
    app_inputs_t in = mock_input_nominal();
    in.inv_dc_bus_voltage = 0xFFFF;  /* Out of bounds */
    return in;
}

/* ===== Mock CAN Frames ===== */

can_msg_t mock_can_frame(uint32_t id, const uint8_t data[8])
{
    can_msg_t m = {
        .bus = CAN_BUS_INV,
        .id = id,
        .dlc = 8,
        .ide = 0  /* Standard CAN ID */
    };
    if (data) {
        memcpy(m.data, data, 8);
    } else {
        memset(m.data, 0, 8);
    }
    return m;
}

can_msg_t mock_can_precarga_ack(uint8_t ack)
{
    uint8_t data[8] = {ack, 0, 0, 0, 0, 0, 0, 0};
    return mock_can_frame(0x20, data);
}

can_msg_t mock_can_dc_bus_voltage(uint16_t voltage_raw)
{
    uint8_t data[8];
    data[0] = (uint8_t)(voltage_raw & 0xFF);
    data[1] = (uint8_t)((voltage_raw >> 8) & 0xFF);
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    return mock_can_frame(0x100, data);
}

can_msg_t mock_can_throttle_s1(uint16_t adc_value)
{
    uint8_t data[8];
    data[0] = (uint8_t)(adc_value & 0xFF);
    data[1] = (uint8_t)((adc_value >> 8) & 0xFF);
    memset(&data[2], 0, 6);
    return mock_can_frame(0x101, data);
}

can_msg_t mock_can_throttle_s2(uint16_t adc_value)
{
    uint8_t data[8];
    data[0] = (uint8_t)(adc_value & 0xFF);
    data[1] = (uint8_t)((adc_value >> 8) & 0xFF);
    memset(&data[2], 0, 6);
    return mock_can_frame(0x102, data);
}

can_msg_t mock_can_brake(uint16_t adc_value)
{
    uint8_t data[8];
    data[0] = (uint8_t)(adc_value & 0xFF);
    data[1] = (uint8_t)((adc_value >> 8) & 0xFF);
    memset(&data[2], 0, 6);
    return mock_can_frame(0x103, data);
}

can_msg_t mock_can_cell_min_voltage(uint16_t voltage_raw)
{
    uint8_t data[8];
    data[0] = (uint8_t)(voltage_raw & 0xFF);
    data[1] = (uint8_t)((voltage_raw >> 8) & 0xFF);
    memset(&data[2], 0, 6);
    return mock_can_frame(0x12C, data);
}

/* ===== Mock RTOS ===== */

uint32_t osKernelGetTickCount(void)
{
    return mock_kernel_tick_ms;
}

void mock_tick_advance(uint32_t ms)
{
    mock_kernel_tick_ms += ms;
}

void mock_tick_reset(void)
{
    mock_kernel_tick_ms = 0;
}

/* ===== Helpers para Assertion ===== */

int mock_can_frames_equal(const can_msg_t *a, const can_msg_t *b)
{
    if (!a || !b) return 0;
    
    return (a->id == b->id &&
            a->bus == b->bus &&
            a->dlc == b->dlc &&
            a->ide == b->ide &&
            memcmp(a->data, b->data, 8) == 0);
}

int mock_app_inputs_equal(const app_inputs_t *a, const app_inputs_t *b)
{
    if (!a || !b) return 0;
    
    return (a->s1_aceleracion == b->s1_aceleracion &&
            a->s2_aceleracion == b->s2_aceleracion &&
            a->s_freno == b->s_freno &&
            a->inv_state == b->inv_state &&
            a->inv_dc_bus_voltage == b->inv_dc_bus_voltage &&
            a->torque_total == b->torque_total);
}

/* ===== Mock stubs para no-op RTOS calls ===== */

/* Stub: osMutexId_t y amigos (no operacionales en host) */
osMutexId_t g_inMutex = NULL;
osMessageQueueId_t canRxQueueHandle = NULL;
osMessageQueueId_t canTxQueueHandle = NULL;

/* Stubs de app_state.c */
app_inputs_t g_in = {0};

void AppState_Init(void)
{
    /* No-op en tests */
    memset(&g_in, 0, sizeof(g_in));
}
