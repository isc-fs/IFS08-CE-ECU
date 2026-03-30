/**
 * sil_can_simulator.c
 * CAN message simulator - injects simulated CAN messages into the app
 */

#include "sil_can_simulator.h"
#include "sil_hal_mocks.h"
#include <stdio.h>
#include <string.h>

static struct {
    uint8_t throttle;
    uint8_t brake;
    uint8_t inverter_state;
    uint16_t dc_voltage;
    uint32_t last_update_ms;
} sim_state = {
    .throttle = 0,
    .brake = 0,
    .inverter_state = 0x02,  /* READY */
    .dc_voltage = 400,
    .last_update_ms = 0
};

void SIL_CAN_Init(void)
{
    sim_state.throttle = 0;
    sim_state.brake = 0;
    sim_state.inverter_state = 0x02;
    sim_state.dc_voltage = 400;
    sim_state.last_update_ms = 0;
    printf("[CAN-SIM] CAN Simulator initialized\n");
}

void SIL_CAN_InjectThrottle(uint8_t throttle_pct)
{
    if (throttle_pct > 100) throttle_pct = 100;
    sim_state.throttle = throttle_pct;
    printf("[CAN-SIM] Throttle injected: %u%%\n", throttle_pct);
}

void SIL_CAN_InjectBrake(uint8_t brake_pct)
{
    if (brake_pct > 100) brake_pct = 100;
    sim_state.brake = brake_pct;
    printf("[CAN-SIM] Brake injected: %u%%\n", brake_pct);
}

void SIL_CAN_InjectInverterState(uint8_t state)
{
    sim_state.inverter_state = state;
    printf("[CAN-SIM] Inverter state injected: 0x%02x\n", state);
}

void SIL_CAN_InjectDCVoltage(uint16_t voltage_v)
{
    sim_state.dc_voltage = voltage_v;
    printf("[CAN-SIM] DC voltage injected: %u V\n", voltage_v);
}

void SIL_CAN_Process(void)
{
    /* Periodically inject CAN messages based on current simulated state */

    uint32_t current_time = SIL_GetTickMs();
    
    /* Send precharge ACK (ID 0x20) every 100ms */
    if ((current_time - sim_state.last_update_ms) >= 100) {
        uint8_t data[8] = {0x00, 0, 0, 0, 0, 0, 0, 0};  /* Legacy: ACK=0 => precarga OK */
        uint16_t s1_adc = (uint16_t)(2050u + ((uint32_t)sim_state.throttle * 900u) / 100u);
        uint16_t s2_adc = (uint16_t)(1915u + ((uint32_t)sim_state.throttle * 655u) / 100u);
        uint16_t brake_adc = (sim_state.brake == 0u)
            ? 0u
            : (uint16_t)(2000u + ((uint32_t)sim_state.brake * 2000u) / 100u);

        SIL_CAN_SendFrame(0x20, data, 8);
        
        /* Send DC bus voltage (ID 0x100) */
        data[0] = (uint8_t)(sim_state.dc_voltage & 0xFF);
        data[1] = (uint8_t)((sim_state.dc_voltage >> 8) & 0xFF);
        SIL_CAN_SendFrame(0x100, data, 8);
        
        /* Legacy DASH frame packs S1/S2 together in big-endian. */
        memset(data, 0, sizeof(data));
        data[0] = (uint8_t)((s1_adc >> 8) & 0xFFu);
        data[1] = (uint8_t)(s1_adc & 0xFFu);
        data[2] = (uint8_t)((s2_adc >> 8) & 0xFFu);
        data[3] = (uint8_t)(s2_adc & 0xFFu);
        SIL_CAN_SendFrame(0x101, data, 4);

        /* Send brake sensor */
        memset(data, 0, sizeof(data));
        data[0] = (uint8_t)(brake_adc & 0xFF);
        data[1] = (uint8_t)((brake_adc >> 8) & 0xFF);
        SIL_CAN_SendFrame(0x103, data, 8);

        /* Inverter state feedback (legacy TX_STATE_2). */
        memset(data, 0, sizeof(data));
        data[4] = (uint8_t)(sim_state.inverter_state & 0x0Fu);
        SIL_CAN_SendFrame(0x461, data, 8);
        
        sim_state.last_update_ms = current_time;
    }
}
