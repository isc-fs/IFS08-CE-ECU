// SPDX-License-Identifier: proprietary

#include "app/pit_diag.hpp"

#include "app/ecu_config.hpp"

#include "can/can_codecs.hpp"

#include <cstdint>

// firmware_info.cpp getters (no shared header -- only this TU needs them).
extern "C" {
std::uint8_t        ecu_fw_version_major(void);
std::uint8_t        ecu_fw_version_minor(void);
std::uint8_t        ecu_fw_version_patch(void);
const std::uint8_t* ecu_git_hash(void);
}

namespace ecu {

namespace {

// Wrap an encoded payload into an ACU-bus CanFrame. The DSL encoder fills a
// DLC-sized buffer; copy it into the 8-byte transport.
template <unsigned Dlc>
CanFrame make_acu(std::uint32_t id, const std::uint8_t (&buf)[Dlc]) noexcept {
    CanFrame f{};
    f.bus = static_cast<std::uint8_t>(CanBus::Acu);
    f.id  = id;
    f.dlc = static_cast<std::uint8_t>(Dlc);
    for (unsigned i = 0; i < Dlc; ++i) f.data[i] = buf[i];
    return f;
}

}  // namespace

CanFrame PitDiag::build_status(const CtrlOutput& c, const VehicleState& v,
                               bool start_button) noexcept {
    PitDiag_status_t s{};
    s.fsm_state = static_cast<std::uint8_t>(c.state);
    s.inv_state = v.inv_state;
    s.ev_2_3       = c.ev_2_3 ? 1u : 0u;
    s.t11_8_9      = c.t11_8_9 ? 1u : 0u;
    s.rtds_active  = c.rtds_on ? 1u : 0u;
    s.ok_precharge = v.ok_precharge ? 1u : 0u;
    s.start_button = start_button ? 1u : 0u;
    s.torque_pct    = c.torque_pct;
    s.v_cell_min_mV = v.v_cell_min_mV;
    s.torque_cmd    = 0;   // inverter unit-map deferred (task #10)
    std::uint8_t b[PitDiag_status_DLC];
    encode_PitDiag_status(s, b);
    return make_acu(PitDiag_status_ID, b);
}

CanFrame PitDiag::build_pedals(const IoInputs& io) noexcept {
    PitDiag_pedals_t p{};
    p.apps1_raw = io.apps1_raw;
    p.apps2_raw = io.apps2_raw;
    p.brake_raw = io.brake_raw;
    p.apps1_pct = apps_pct(io.apps1_raw, config::Apps1AdcMin, config::Apps1AdcMax);
    p.apps2_pct = apps_pct(io.apps2_raw, config::Apps2AdcMin, config::Apps2AdcMax);
    std::uint8_t b[PitDiag_pedals_DLC];
    encode_PitDiag_pedals(p, b);
    return make_acu(PitDiag_pedals_ID, b);
}

CanFrame PitDiag::build_inverter(const VehicleState& v) noexcept {
    PitDiag_inverter_t inv{};
    inv.dc_bus_voltage = v.inv_dc_bus_V;
    inv.inv_rpm        = v.inv_rpm;    // 0x463 EMachine_Speed_erpm (20-bit signed)
    inv.inv_error      = v.inv_error;
    std::uint8_t b[PitDiag_inverter_DLC];
    encode_PitDiag_inverter(inv, b);
    return make_acu(PitDiag_inverter_ID, b);
}

CanFrame PitDiag::build_inverter_temps(const VehicleState& v) noexcept {
    PitDiag_inverter_temps_t t{};
    t.temp_board_degC  = v.inv_temp_board;   // raw bytes; the DBC -50 offset -> degC
    t.temp_pwrstg_degC = v.inv_temp_pwrstg;
    t.temp_motor1_degC = v.inv_temp_motor1;
    t.temp_motor2_degC = v.inv_temp_motor2;
    std::uint8_t b[PitDiag_inverter_temps_DLC];
    encode_PitDiag_inverter_temps(t, b);
    return make_acu(PitDiag_inverter_temps_ID, b);
}

CanFrame PitDiag::build_fwinfo() noexcept {
    PitDiag_fwinfo_t fw{};
    fw.fw_major = ecu_fw_version_major();
    fw.fw_minor = ecu_fw_version_minor();
    fw.fw_patch = ecu_fw_version_patch();
    const std::uint8_t* g = ecu_git_hash();
    fw.git_hash = (static_cast<std::uint32_t>(g[0]) << 24) |
                  (static_cast<std::uint32_t>(g[1]) << 16) |
                  (static_cast<std::uint32_t>(g[2]) << 8) |
                   static_cast<std::uint32_t>(g[3]);
    std::uint8_t b[PitDiag_fwinfo_DLC];
    encode_PitDiag_fwinfo(fw, b);
    return make_acu(PitDiag_fwinfo_ID, b);
}

CanFrame PitDiag::build_health(const HealthMetrics& m) noexcept {
    PitDiag_health_t h{};
    h.free_heap     = m.free_heap;
    h.min_free_heap = m.min_free_heap;
    // split the liveness mask into 1-bit DBC signals (EcuTaskId bit order)
    h.task_control = (m.task_ran_mask >> 0) & 1u;
    h.task_can_rx  = (m.task_ran_mask >> 1) & 1u;
    h.task_can_tx  = (m.task_ran_mask >> 2) & 1u;
    h.task_diag    = (m.task_ran_mask >> 3) & 1u;
    h.reset_cause   = static_cast<std::uint8_t>(m.reset_cause);
    h.uptime_s      = m.uptime_s;
    h.last_fault    = static_cast<std::uint8_t>(m.last_fault);
    std::uint8_t b[PitDiag_health_DLC];
    encode_PitDiag_health(h, b);
    return make_acu(PitDiag_health_ID, b);
}

CanFrame PitDiag::build_brake(const IoInputs& io) noexcept {
    PitDiag_brake_t br{};
    // PENDING calibration: brake pressure stays 0 (0.1-bar) and % is a rough
    // 12-bit-range placeholder until the real S_BRAKE sensor map lands.
    br.brake_pressure = 0;
    br.brake_pct = static_cast<std::uint8_t>(
        io.brake_raw >= 4095u ? 100u : (static_cast<std::uint32_t>(io.brake_raw) * 100u / 4095u));
    std::uint8_t b[PitDiag_brake_DLC];
    encode_PitDiag_brake(br, b);
    return make_acu(PitDiag_brake_ID, b);
}

CanFrame PitDiag::build_ack(bool enabled) noexcept {
    PitDiag_ack_t a{};
    a.enabled = enabled ? 1u : 0u;
    std::uint8_t b[PitDiag_ack_DLC];
    encode_PitDiag_ack(a, b);
    return make_acu(PitDiag_ack_ID, b);
}

}  // namespace ecu
