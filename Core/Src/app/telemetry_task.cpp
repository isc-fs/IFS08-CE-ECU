// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"
#include "app/app_tasks.h"
#include "app/can_tx.hpp"
#include "app/vehicle_service.hpp"

#include "telemetry.h"

#include "cmsis_os2.h"

#include <cstdint>
#include <cstring>

using namespace ecu;

namespace {

constexpr uint8_t  RfMagic = 0xECu;
constexpr uint8_t  RfVersion = 0x02u;
constexpr uint8_t  RfFastKind = 3u;
constexpr uint8_t  RfFastFragments = 2u;
constexpr uint32_t PeriodMs = 100u;

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
    d[0] = v.inv_state;
    d[1] = static_cast<uint8_t>(g_last_torque_pct);
    d[2] = v.inv_error ? 0x04u : 0u;
    d[3] = v.ok_precharge ? 1u : 0u;
    d[5] = RfFastKind;
    put_u16(&d[6], seq);
    send_dash(0x510u, d, 8u);

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

    std::memset(d, 0, sizeof(d));
    d[0] = g_last_ctrl_state;
    d[1] = v.ams_fsm_state;
    send_dash(0x517u, d, 2u);
}

void send_radio_fast(const VehicleState& v, uint16_t seq, uint8_t frag) {
    uint8_t p[32] = {};
    p[0] = RfMagic;
    p[1] = RfVersion;
    p[2] = frag;
    p[3] = RfFastFragments;
    put_u16(&p[4], seq);
    p[6] = RfFastKind;

    if (frag == 0u) {
        p[8] = g_last_ctrl_state;
        p[9] = v.inv_state;
        p[10] = v.ams_fsm_state;
        p[12] = v.ok_precharge ? 1u : 0u;
        p[15] = v.last_vconfig_tick ? 1u : 0u;
        p[16] = v.inv_error;
        put_u16(&p[17], static_cast<uint16_t>(g_last_torque_pct));
        put_u16(&p[19], v.inv_dc_bus_V);
        put_u16(&p[21], v.v_cell_min_mV);
    } else {
        put_u16(&p[8], v.inv_temp_motor1);
        put_u16(&p[10], v.inv_temp_pwrstg);
        put_u16(&p[12], v.inv_temp_board);
        put_u32(&p[14], static_cast<uint32_t>(v.inv_rpm));
    }

    Telemetry_Send32(p);
}

}  // namespace

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
        send_radio_fast(v, seq, 0u);
        send_radio_fast(v, seq, 1u);

        tick += PeriodMs;
        osDelayUntil(tick);
    }
}
