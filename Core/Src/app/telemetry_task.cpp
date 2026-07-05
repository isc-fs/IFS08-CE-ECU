// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"
#include "app/app_tasks.h"
#include "app/can_tx.hpp"
#include "app/radio_snapshot.hpp"
#include "app/vehicle_service.hpp"

#include "telemetry.h"

#include "cmsis_os2.h"

#include <cstdint>
#include <cstring>

using namespace ecu;

// Radio telemetry = the v2 "fragmented snapshot" (radio_snapshot.hpp): one
// 102-byte snapshot in 5 nRF24 fragments per 200 ms cycle, matched byte-for-byte
// to the live ground-station parser (IFS08-TE feat/receptor_08 ISC_RTT_serial.py
// _decode_snapshot). This replaced the older RF_FAST/RF_SLOW multi-kind protocol
// the ground station no longer parses. The FDCAN3 dash frames (send_dashboard)
// are a separate wire contract and are unchanged.
namespace {

constexpr uint32_t PeriodMs = 200u;

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

void send_dash(uint32_t id, const uint8_t* data, uint8_t dlc) {
    CanFrame f{};
    f.bus = static_cast<uint8_t>(CanBus::Dash);
    f.id = id;
    f.dlc = dlc;
    for (uint8_t i = 0; i < dlc && i < CanFrameMaxData; ++i) f.data[i] = data[i];
    can_tx_post(f);
}

void send_dashboard(const VehicleState& v, uint16_t seq) {
    uint8_t d[8] = {};

    // 0x510 - status. Bits/bytes match docs/CAN3_MAP.md + DASH's
    // display_telemetry_can_config.c exactly (byte2 bit0=EV.2.3, bit1=T11.8.9;
    // byte4=start button raw from ControlTask, mirrored via g_last_*).
    d[0] = v.inv_state;
    d[1] = static_cast<uint8_t>(g_last_torque_pct);
    d[2] = static_cast<uint8_t>((g_last_ev_2_3 ? 0x01u : 0u) | (g_last_t11_8_9 ? 0x02u : 0u));
    d[3] = v.ok_precharge ? 1u : 0u;
    d[4] = g_last_start_button;
    put_u16(&d[6], seq);
    send_dash(0x510u, d, 8u);

    // 0x511 - pedales/freno (ADC raw, real -- mirrored from ControlTask's IoInputs).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], g_last_apps1_raw);
    put_u16(&d[2], g_last_apps2_raw);
    put_u16(&d[4], g_last_brake_raw);
    send_dash(0x511u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.inv_dc_bus_V);
    put_u16(&d[2], v.v_cell_min_mV);
    d[4] = v.inv_error;
    d[5] = v.last_vconfig_tick ? 1u : 0u;
    send_dash(0x512u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.inv_temp_motor1);
    put_u16(&d[2], v.inv_temp_pwrstg);
    put_u16(&d[4], v.inv_temp_board);
    send_dash(0x513u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u32(d, static_cast<uint32_t>(v.inv_rpm));
    send_dash(0x514u, d, 4u);

    // 0x515/0x516 - inverter speed/current "actual". PLACEHOLDER (0): the
    // inverter frames VehicleService decodes today (0x461/0x463/0x464/0x466)
    // don't carry these; no real source exists in this firmware yet.
    std::memset(d, 0, sizeof(d));
    send_dash(0x515u, d, 4u);
    send_dash(0x516u, d, 4u);

    std::memset(d, 0, sizeof(d));
    d[0] = g_last_ctrl_state;
    d[1] = v.ams_fsm_state;
    send_dash(0x517u, d, 2u);

    // 0x518 - AMS soc (PLACEHOLDER: no estimator, deferred on the AMS side
    // too -- see IFS08-CE-AMS acu_tx_encoders.hpp) + corrientes/temp_dcdc
    // (real, from ACU_currents 0x135 / ACU_tmax_module_b 0x137).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[1], static_cast<uint16_t>(v.current_accu_dA));
    put_u16(&d[3], static_cast<uint16_t>(v.current_dcdc_dA));
    put_u16(&d[5], static_cast<uint16_t>(v.tmax_dcdc));
    send_dash(0x518u, d, 7u);

    // 0x519/0x51A - GPS. PLACEHOLDER (0): no GPS driver in this firmware
    // (UART pins are routed per docs/PINES_RUTEADOS_IOC.md but unused).
    std::memset(d, 0, sizeof(d));
    send_dash(0x519u, d, 8u);
    send_dash(0x51Au, d, 8u);

    // 0x51B - GPS longitude PLACEHOLDER (0) + tick_ms (real, RTOS tick).
    std::memset(d, 0, sizeof(d));
    put_u32(&d[4], osKernelGetTickCount());
    send_dash(0x51Bu, d, 8u);

    // 0x51C/0x51D - AMS per-module vmin (real, from ACU_vmin_module_a/b
    // 0x131/0x132).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmin_module[0]);
    put_u16(&d[2], v.vmin_module[1]);
    put_u16(&d[4], v.vmin_module[2]);
    send_dash(0x51Cu, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmin_module[3]);
    put_u16(&d[2], v.vmin_module[4]);
    send_dash(0x51Du, d, 4u);

    // 0x51E/0x51F - AMS per-module vmax (real, from ACU_vmax_module_a/b
    // 0x133/0x134).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmax_module[0]);
    put_u16(&d[2], v.vmax_module[1]);
    put_u16(&d[4], v.vmax_module[2]);
    send_dash(0x51Eu, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmax_module[3]);
    put_u16(&d[2], v.vmax_module[4]);
    send_dash(0x51Fu, d, 4u);

    // 0x520/0x521 - AMS per-module tmax + temp_dcdc (real, from
    // ACU_tmax_module_a/b 0x136/0x137).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], static_cast<uint16_t>(v.tmax_module[0]));
    put_u16(&d[2], static_cast<uint16_t>(v.tmax_module[1]));
    put_u16(&d[4], static_cast<uint16_t>(v.tmax_module[2]));
    send_dash(0x520u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], static_cast<uint16_t>(v.tmax_module[3]));
    put_u16(&d[2], static_cast<uint16_t>(v.tmax_module[4]));
    put_u16(&d[4], static_cast<uint16_t>(v.tmax_dcdc));
    send_dash(0x521u, d, 6u);
}

// Populate the snapshot inputs from the shared vehicle state + the ControlTask
// g_last_* mirrors, serialize the 102-byte v2 snapshot, and send it as five
// nRF24 fragments. The byte layout is owned by radio_snapshot.cpp (matched to
// the ground-station parser); this is only the data plumbing.
void send_radio_snapshot(const VehicleState& v, uint16_t seq, uint32_t tick_ms) {
    RadioSnapshotInputs in{};
    in.tick_ms       = tick_ms;
    in.seq           = seq;
    in.start_button  = g_last_start_button;
    in.apps1_raw     = g_last_apps1_raw;
    in.apps2_raw     = g_last_apps2_raw;
    in.brake_raw     = g_last_brake_raw;
    in.torque_pct    = static_cast<uint8_t>(g_last_torque_pct);
    in.ev_2_3        = g_last_ev_2_3;
    in.t11_8_9       = g_last_t11_8_9;
    in.state         = g_last_ctrl_state;
    in.ok_precharge  = v.ok_precharge ? 1u : 0u;
    in.ams_fsm_state = v.ams_fsm_state;
    in.v_cell_min_mV = v.v_cell_min_mV;
    in.soc           = 0u;  // PLACEHOLDER: AMS has no SoC estimator yet.
    for (unsigned i = 0; i < 5; ++i) {
        in.vmin_module[i] = v.vmin_module[i];
        in.vmax_module[i] = v.vmax_module[i];
        in.tmax_module[i] = v.tmax_module[i];
    }
    in.current_accu_dA    = v.current_accu_dA;
    in.current_dcdc_dA    = v.current_dcdc_dA;
    in.tmax_dcdc          = v.tmax_dcdc;
    in.inv_state          = v.inv_state;
    in.inv_vconfig_active = v.last_vconfig_tick ? 1u : 0u;
    in.inv_error          = v.inv_error;
    in.inv_dc_bus_V       = v.inv_dc_bus_V;
    in.inv_temp_motor1    = v.inv_temp_motor1;
    in.inv_temp_pwrstg    = v.inv_temp_pwrstg;
    in.inv_temp_board     = v.inv_temp_board;
    in.inv_rpm            = v.inv_rpm;
    in.inv_speed_actual   = 0;  // PLACEHOLDER: no inverter speed/current source.
    in.inv_current_actual = 0;

    uint8_t snap[kRadioSnapshotWireSize];
    serialize_radio_snapshot(snap, in);
    for (uint8_t frag = 0; frag < kRadioSnapshotFragments; ++frag) {
        uint8_t p[kRadioFragmentSize];
        build_radio_fragment(p, snap, seq, frag);
        Telemetry_Send32(p);
    }
}

}  // namespace

#if defined(SIL_BUILD)
namespace ecu {

uint32_t telemetry_period_ms_for_test() {
    return PeriodMs;
}

void telemetry_emit_for_test(const VehicleState& v, uint16_t seq) {
    send_dashboard(v, seq);
    send_radio_snapshot(v, seq, osKernelGetTickCount());
}

}  // namespace ecu
#endif

extern "C" void ecu_telemetry_task_run(void *argument) {
    (void)argument;
    uint16_t seq = 0;
    uint32_t tick = osKernelGetTickCount();
    auto& vs = VehicleService::instance();

    for (;;) {
        ++g_task_step[ECU_TASK_TELEMETRY];
        const VehicleState v = vs.snapshot();
        ++seq;
        send_dashboard(v, seq);
        send_radio_snapshot(v, seq, osKernelGetTickCount());

        tick += PeriodMs;
        osDelayUntil(tick);
    }
}
