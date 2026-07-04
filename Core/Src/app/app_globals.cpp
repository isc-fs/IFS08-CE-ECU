// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"

extern "C" {
volatile uint32_t g_task_step[ECU_TASK_COUNT] = {};
volatile ecu_control_telemetry_t g_control_telemetry = {};
volatile ecu_radio_diag_t g_radio_diag = {};
volatile uint8_t  g_pit_diag_enabled = 0;
}

extern "C" void ecu_app_globals_init(void) {
    for (int i = 0; i < ECU_TASK_COUNT; ++i) {
        g_task_step[i] = 0u;
    }
    g_control_telemetry.state = 0u;
    g_control_telemetry.torque_pct = 0u;
    g_control_telemetry.ev_2_3 = 0u;
    g_control_telemetry.t11_8_9 = 0u;
    g_control_telemetry.ok_to_drive = 0u;
    g_control_telemetry.dv_mode = 0u;
    g_radio_diag.tx_ok = 0u;
    g_radio_diag.tx_fail = 0u;
    g_radio_diag.last_hal_status = 0u;
    g_radio_diag.last_nrf_status = 0u;
    g_radio_diag.config = 0u;
    g_radio_diag.rf_ch = 0u;
    g_radio_diag.raw[0] = 0u;
    g_radio_diag.raw[1] = 0u;
    g_radio_diag.raw[2] = 0u;
    g_radio_diag.raw[3] = 0u;
}
