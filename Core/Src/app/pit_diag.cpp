// SPDX-License-Identifier: proprietary

#include "app/pit_diag.hpp"

#include "app/app_globals.h"   // g_can_tx_dropped (tx_dropped sticky bit)
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
    s.dv_mode      = c.dv_mode ? 1u : 0u;   // #109: DV drive latched this cycle
    s.tx_dropped   = (g_can_tx_dropped != 0u) ? 1u : 0u;  // #127: any TX-queue drop since boot
    s.torque_pct    = c.torque_pct;
    s.v_cell_min_mV = v.v_cell_min_mV;
    s.torque_cmd    = 0;   // inverter unit-map deferred (task #10)
    std::uint8_t b[PitDiag_status_DLC];
    encode_PitDiag_status(s, b);
    return make_acu(PitDiag_status_ID, b);
}

CanFrame PitDiag::build_dv(const CtrlOutput& c, const CtrlInputs& in,
                           const VehicleState& v) noexcept {
    PitDiag_dv_t d{};
    d.dv_r2d_req       = in.dv_r2d_req ? 1u : 0u;                            // uDV 0x510 set+fresh
    d.dv_cmd_fresh     = in.dv_fresh ? 1u : 0u;                              // uDV 0x507 stream fresh
    d.ts_active        = in.ok_precharge ? 1u : 0u;                         // TX 0x504 view
    d.brake_over_limit = (in.brake_raw > in.cal.brake_dv_hard) ? 1u : 0u;  // TX 0x505 verdict
    d.r2d_confirm      = c.dv_mode ? 1u : 0u;                               // TX 0x511 (== latched)
    d.dv_torque_pct    = in.dv_torque_pct;                                  // conditioned 0x507
    d.motor_rpm_mech   = static_cast<std::int16_t>(v.inv_rpm / config::MotorPolePairs);
    std::uint8_t b[PitDiag_dv_DLC];
    encode_PitDiag_dv(d, b);
    return make_acu(PitDiag_dv_ID, b);
}

CanFrame PitDiag::build_pedals(const IoInputs& io, const PedalCal& cal) noexcept {
    PitDiag_pedals_t p{};
    p.apps1_raw = io.apps1_raw;
    p.apps2_raw = io.apps2_raw;
    p.brake_raw = io.brake_raw;
    p.apps1_pct = apps_pct(io.apps1_raw, cal.apps1_min, cal.apps1_max);
    p.apps2_pct = apps_pct(io.apps2_raw, cal.apps2_min, cal.apps2_max);
    std::uint8_t b[PitDiag_pedals_DLC];
    encode_PitDiag_pedals(p, b);
    return make_acu(PitDiag_pedals_ID, b);
}

CanFrame PitDiag::build_inverter(const VehicleState& v,
                                 std::uint8_t inv_mode_cmd) noexcept {
    PitDiag_inverter_t inv{};
    inv.dc_bus_voltage = v.inv_dc_bus_V;
    // 0x463 reports ELECTRICAL rpm; diag shows MECHANICAL shaft rpm, same
    // convention as the uDV 0x506 feed (erpm / MotorPolePairs, 10).
    inv.inv_rpm        = v.inv_rpm / config::MotorPolePairs;
    inv.inv_error      = v.inv_error;
    inv.dem_present    = v.inv_dem_present ? 1u : 0u;   // active fault vs latched history
    // 7-bit field; every InvMode fits (max 0x13 = 19). Mask defensively so a
    // bad value can never bleed into dem_present's bit.
    inv.inv_mode_cmd   = static_cast<std::uint8_t>(inv_mode_cmd & 0x7Fu);
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

CanFrame PitDiag::build_inv_faults(const VehicleState& v,
                                   const CtrlOutput& c,
                                   std::uint32_t now_ms) noexcept {
    PitDiag_inv_faults_t f{};
    const std::uint16_t p = v.inv_pwrstg_bits;   // L1, 9 bits
    const std::uint8_t  e = v.inv_emctrl_bits;   // L2, 8 bits
    f.pwrstg_alive          = (p >> 0) & 1u;
    f.pwrstg_enable         = (p >> 1) & 1u;
    f.pwrstg_uvlo           = (p >> 2) & 1u;
    f.pwrstg_desat          = (p >> 3) & 1u;
    f.pwrstg_dt_violation   = (p >> 4) & 1u;
    f.pwrstg_hvil_open      = (p >> 5) & 1u;
    f.pwrstg_ocp            = (p >> 6) & 1u;
    f.pwrstg_ovp_th1        = (p >> 7) & 1u;
    f.pwrstg_ovp_th2        = (p >> 8) & 1u;
    f.emctrl_init_ok        = (e >> 0) & 1u;
    f.emctrl_posfb          = (e >> 1) & 1u;
    f.emctrl_asc            = (e >> 2) & 1u;
    f.emctrl_curr_imbalance = (e >> 3) & 1u;
    f.emctrl_pwrstg_fault   = (e >> 4) & 1u;
    f.emctrl_curr_derating  = (e >> 5) & 1u;
    f.emctrl_loop_delocked  = (e >> 6) & 1u;
    f.emctrl_phcurr_acq     = (e >> 7) & 1u;
    // commanded side -- what the ECU decided to emit on FDCAN1 this cycle
    f.cmd_follow_n          = c.inv_mode_follow_n;
    f.cmd_flt_clear         = c.inv_flt_clear ? 1u : 0u;
    // Freshness of the 0x461 we are steering on. Saturate at 255 ms -- past
    // that the exact number stops mattering, it is simply far too stale.
    const std::uint32_t age = static_cast<std::uint32_t>(now_ms - v.last_inv_state_tick);
    f.inv_state_age_ms      = (age > 255u) ? 255u : static_cast<std::uint8_t>(age);
    f.inv_state_seq         = v.inv_state_seq;
    std::uint8_t b[PitDiag_inv_faults_DLC];
    encode_PitDiag_inv_faults(f, b);
    return make_acu(PitDiag_inv_faults_ID, b);
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
    h.task_control   = (m.task_ran_mask >> 0) & 1u;
    h.task_can_rx    = (m.task_ran_mask >> 1) & 1u;
    h.task_can_tx    = (m.task_ran_mask >> 2) & 1u;
    h.task_telemetry = (m.task_ran_mask >> 3) & 1u;   // EcuTaskId TELEMETRY=3
    h.task_diag      = (m.task_ran_mask >> 4) & 1u;   // EcuTaskId DIAG=4
    // Bench stub announce (#127): mirror the compile-time ecu_config toggles onto
    // the bus so a bring-up image can't pass for a flight one. ALL ZERO on flight.
    // stub_no_ams is the load-bearing one -- it forces ok_precharge, which is what
    // 0x504 VCU_ts_active reports to the uDV, so set => TS-active here is FAKE.
    // Boot-time calibration outcome (#169) -- see the .def for why it is here.
    h.cal_status       = static_cast<std::uint8_t>(g_cal_load_status & 0x03u);
    h.stub_no_ams      = config::StubNoAms ? 1u : 0u;
    h.stub_no_inverter = config::StubNoInverter ? 1u : 0u;
    h.stub_start       = config::StubStart ? 1u : 0u;
    // stub_brake (#137): restored to byte5 b3 (StubBrakeRaw != 0 injects a fake
    // brake_raw). Disables no safety gate -- also visible on 0x701 -- but carried
    // here so the flight-vs-bench announce on the ungated 0x704 is complete again.
    h.stub_brake       = (config::StubBrakeRaw != 0) ? 1u : 0u;
    h.reset_cause   = static_cast<std::uint8_t>(m.reset_cause);
    h.uptime_s      = m.uptime_s;
    h.last_fault    = static_cast<std::uint8_t>(m.last_fault);
    std::uint8_t b[PitDiag_health_DLC];
    encode_PitDiag_health(h, b);
    return make_acu(PitDiag_health_ID, b);
}

CanFrame PitDiag::build_brake(const IoInputs& io, const PedalCal& cal) noexcept {
    PitDiag_brake_t br{};
    // Pressure in bar still needs the S_BRAKE sensor transfer function (part
    // number, range, output span, any divider ahead of the 3V3 ADC), which we
    // do not have -- so it stays 0 rather than reporting an invented number.
    // Out of scope for #169; tracked separately.
    br.brake_pressure = 0;
    // Percentage now uses the CALIBRATED span once a rest point exists, and
    // falls back to the legacy full-range scaling until then (see brake_pct).
    br.brake_pct = brake_pct(io.brake_raw, cal);
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
