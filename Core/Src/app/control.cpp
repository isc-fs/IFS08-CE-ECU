// SPDX-License-Identifier: proprietary
//
// The pure ECU control core. See control.hpp for the contract. No HAL/RTOS
// includes here -- this translation unit is compiled as-is into the host
// unit-test build.

#include "app/control.hpp"

namespace ecu {

using namespace config;

uint8_t apps_pct(uint16_t raw, uint16_t adc_min, uint16_t adc_max) noexcept {
    if (adc_max <= adc_min) return 0;          // mis-calibration guard
    if (raw <= adc_min) return 0;
    if (raw >= adc_max) return 100;
    return static_cast<uint8_t>(
        (static_cast<uint32_t>(raw - adc_min) * 100u) / (adc_max - adc_min));
}

namespace {

// Low-cell-voltage torque derate: factor 1.0 at/above the knee, smoothly down
// to ~floor at the floor voltage, flat below. (Legacy intent -- it computed
// this into torque_limitado but transmitted the unlimited value.)
uint8_t derate_for_cell_voltage(uint8_t torque, uint16_t v_mV) noexcept {
    if (v_mV >= CellVDerateKneeMv) return torque;
    double factor = (v_mV > CellVDerateFloorMv)
        ? (CellVDerateSlope * v_mV - CellVDerateIntercept) / CellVDerateScale
        : CellVDerateFloorFactor;
    if (factor < 0.0) factor = 0.0;
    if (factor > 1.0) factor = 1.0;
    return static_cast<uint8_t>(static_cast<double>(torque) * factor);
}

}  // namespace

void Controller::enter_(CtrlState s, uint32_t now_ms) noexcept {
    state_ = s;
    state_entry_ms_ = now_ms;
}

CtrlOutput Controller::step(const CtrlInputs& in, uint32_t now_ms) noexcept {
    // ---- pedal % + torque + plausibility (computed every tick, even before
    //      Active, so the latches track and pit-diag shows live verdicts) ----
    const uint8_t a1 = apps_pct(in.apps1_raw, Apps1AdcMin, Apps1AdcMax);
    const uint8_t a2 = apps_pct(in.apps2_raw, Apps2AdcMin, Apps2AdcMax);

    uint8_t torque = 0;
    if (a1 > AppsAgreementPct && a2 > AppsAgreementPct) {
        torque = static_cast<uint8_t>((static_cast<uint16_t>(a1) + a2) / 2u);
    }
    if (torque < DeadbandLowPct) torque = 0;
    else if (torque > DeadbandHighPct) torque = 100;

    // EV.2.3 brake+throttle plausibility (latches; clears only when the pedal
    // returns below the reset threshold with the brake released).
    if (in.brake_raw > BrakePressedRaw && torque > Ev23SetPct) {
        ev23_latched_ = true;
    } else if (in.brake_raw < BrakePressedRaw && torque < Ev23ResetPct) {
        ev23_latched_ = false;
    }

    // T.11.8.9 APPS disagreement, honouring the 100 ms persistence window.
    const int diff = static_cast<int>(a1) - static_cast<int>(a2);
    const bool disagree = (diff < 0 ? -diff : diff) > static_cast<int>(AppsDisagreePct);
    if (disagree) {
        if (!apps_disagree_active_) {
            apps_disagree_active_ = true;
            apps_disagree_since_ms_ = now_ms;
        }
    } else {
        apps_disagree_active_ = false;
    }
    const bool t11 = apps_disagree_active_ &&
                     static_cast<uint32_t>(now_ms - apps_disagree_since_ms_) >= AppsDisagreePersistMs;

    if (ev23_latched_ || t11) torque = 0;

    // Low-cell-voltage derate (applied -- see note above).
    torque = derate_for_cell_voltage(torque, in.ams_fresh ? in.v_cell_min_mV : CellVDefaultMv);

    // ---- FSM: decide transitions FIRST, then derive outputs from the
    //      resulting state, so the emitted output always matches the state we
    //      report (a Moore machine -- no one-tick lag on entry actions). ----

    // AMS latched Error overrides every state: inhibit, do not retry precharge.
    if (in.ams_error && state_ != CtrlState::AmsError) enter_(CtrlState::AmsError, now_ms);

    switch (state_) {
    case CtrlState::WaitInvVdcConfig:
        if (in.inv_vconfig_ready) enter_(CtrlState::Precharge, now_ms);
        break;
    case CtrlState::Precharge:
        // Gate on the AMS verdict (0x020) ONLY -- the AMS owns precharge now
        // (0x600 retired). On timeout, restart the wait window (retry).
        if (in.ok_precharge) {
            enter_(CtrlState::WaitStartBrake, now_ms);
        } else if (static_cast<uint32_t>(now_ms - state_entry_ms_) >= PrechargeTimeoutMs) {
            enter_(CtrlState::Precharge, now_ms);
        }
        break;
    case CtrlState::WaitStartBrake:
        if (in.start_button && in.brake_raw > BrakeArmRaw) enter_(CtrlState::R2dDelay, now_ms);
        break;
    case CtrlState::R2dDelay:
        if (static_cast<uint32_t>(now_ms - state_entry_ms_) >= R2dSoundMs) {
            enter_(CtrlState::WaitInvStandby, now_ms);
        }
        break;
    case CtrlState::WaitInvStandby:
        if (in.inv_state == InvReadyState) enter_(CtrlState::Active, now_ms);
        break;
    case CtrlState::Active:
        // AMS opened the contactors (ok_precharge fell) -> re-arm.
        if (!in.ok_precharge) enter_(CtrlState::Precharge, now_ms);
        break;
    case CtrlState::AmsError:
        if (!in.ams_error) enter_(CtrlState::WaitInvVdcConfig, now_ms);
        break;
    }

    InvMode mode = InvMode::Off;
    bool    rtds = false;
    bool    drive = false;
    uint8_t cmd_torque = 0;

    switch (state_) {
    case CtrlState::WaitInvVdcConfig:
    case CtrlState::Precharge:
    case CtrlState::WaitStartBrake:
    case CtrlState::AmsError:
        mode = InvMode::Off;
        break;
    case CtrlState::R2dDelay:
        mode = InvMode::Off;
        rtds = true;                        // drive the RTDS buzzer
        break;
    case CtrlState::WaitInvStandby:
        mode = InvMode::Ready;              // command standby -> ready
        break;
    case CtrlState::Active:
        if (in.inv_state >= 10u) {
            mode = InvMode::Fault;          // inverter fault -> command reset, no torque
        } else {
            mode = InvMode::TorqueEnable;
            drive = true;
            cmd_torque = torque;
        }
        break;
    }

    CtrlOutput out{};
    out.state       = state_;
    out.torque_pct  = cmd_torque;
    out.inv_mode    = mode;
    out.rtds_on     = rtds;
    out.ok_to_drive = drive;
    out.ev_2_3      = ev23_latched_;
    out.t11_8_9     = t11;
    return out;
}

}  // namespace ecu
