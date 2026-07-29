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
    // The DV latch lives for one drive cycle: any exit from the drive ladder
    // (back to a pre-R2D state, or the AmsError inhibit) clears it. The next
    // R2D entry re-decides the mode from whichever trigger fires (#17).
    if (s < CtrlState::R2dDelay || s == CtrlState::AmsError) {
        dv_latched_ = false;
    }
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

    // DV torque source (#17): when the DV drive is latched the pedals are NOT
    // the torque source -- the conditioned uDV 0x507 command is, and a stale
    // command stream means torque 0, NEVER a fall-back to APPS (no driver is
    // seated). EV.2.3 / T.11.8.9 are driver-pedal rules and do not gate the DV
    // command (the EBS legitimately holds brake pressure in DV -- a naive
    // EV.2.3 would cut torque the moment uDV commands accel). The pedal
    // latches keep computing above (pedals idle; pit-diag verdicts stay live)
    // but their zeroing applies to the pedal torque only. The cell-voltage
    // derate below still applies -- pack protection is mode-independent.
    if (dv_latched_) {
        torque = in.dv_fresh ? in.dv_torque_pct : 0;
    }

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
        // The trigger IS the mode decision (#17): whichever gate fires latches
        // the mode for this drive cycle. Manual = seated driver (START + brake
        // past the arm threshold). DV = the uDV R2D request (0x510, fresh)
        // WHILE the EBS holds hard braking, verified on our own brake sensor
        // (brake_raw > BrakeDvHardRaw) -- no start button in DV. The two are
        // physically exclusive (driver seated vs ASMS on / AS mission running).
        if (in.start_button && in.brake_raw > BrakeArmRaw) {
            enter_(CtrlState::R2dDelay, now_ms);
        } else if (in.dv_r2d_req && in.brake_raw > BrakeDvHardRaw) {
            enter_(CtrlState::R2dDelay, now_ms);
            dv_latched_ = true;   // after enter_ (which clears it for pre-R2D targets)
        }
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
    // Follow-up mode words for the fault burst below (see CtrlOutput).
    InvMode follow[2] = { InvMode::Off, InvMode::Off };
    uint8_t follow_n = 0;
    bool    flt_clear = false;
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
        // Command Ready UNCONDITIONALLY, whatever the inverter currently reports.
        //
        // DO NOT make this reactive on inv_state again. #144 did exactly that --
        // it sent Off(0x01) instead when the inverter reported OFF(0)/SHUTDOWN(13),
        // on the theory that an off inverter needs an "on" word before it will
        // accept Ready. That theory came from the legacy IFS07 switch and was
        // never confirmed against this inverter. The W90 manual's state machine
        // (section 9.1) contradicts it: OFF --(App_State_Req = READY)--> READY is
        // a SINGLE direct transition, with no intermediate step. So commanding Off
        // to an inverter already in OFF just holds it there and Ready is never
        // sent -- it cannot climb at all. #144 did not fix the TS-off recovery it
        // targeted and was a plausible way to make it worse. See #148.
        //
        // A latched fault (10/11) is still overridden to its reset word by the
        // reactive block below -- that path IS bench-proven. Reaching Ready(4)
        // advances to Active (transition above).
        mode = InvMode::Ready;              // 0x04 -- OFF/standby -> ready
        break;
    case CtrlState::Active:
        // Healthy inverter -> drive. A faulted inverter (>= soft fault) gets its
        // recovery mode + no torque from the reactive block after this switch.
        if (in.inv_state < InvSoftFaultState) {
            mode = InvMode::TorqueEnable;
            drive = true;
            cmd_torque = torque;
        }
        break;
    }

    // Inverter fault recovery -- reactive, in ANY drive state (not just Active).
    // A faulted inverter ignores Off/Ready; it clears only when commanded its
    // recovery mode word. Mirrors the legacy VCU's per-state inverter switch
    // (pre-jarama main.c:1912/1956): soft fault (10) -> Fault (0x13), hard fault
    // (11) -> HardFaultReset (0x0D). The inverter can boot LATCHED in hard fault
    // before we ever reach Active, so this must run pre-Active or the FSM stalls
    // at WaitInvStandby forever waiting for a ready state that never comes. Torque
    // is already 0 outside the Active healthy path, so this never drives a fault.
    // Not applied in AmsError (that state inhibits -- Off is the safe command).
    //
    // The reset word is followed, IN THE SAME CYCLE, by Off(0x01) -- see
    // CtrlOutput::inv_mode_follow. Sending only the reset word leaves the fault
    // standing: manual 9.3 says going to OFF is what restarts a FAULT, and the
    // IFS07 VCU's fall-through switch always ended on 0x01. A 2026-07-24 bench
    // capture caught exactly this -- inverter latched SoftFault(10) with the DC
    // bus at 355 V while the ECU commanded 0x13 forever and it never cleared.
    if (state_ != CtrlState::AmsError) {
        if (in.inv_state == InvHardFaultState) {
            mode      = InvMode::HardFaultReset;   // 0x0D
            follow[0] = InvMode::Off;              // 0x01 -- the documented clear
            follow_n  = 1;
            flt_clear = true;                      // + Flt_Clear on that Off
        } else if (in.inv_state == InvSoftFaultState) {
            mode      = InvMode::Fault;            // 0x13
            follow[0] = InvMode::HardFaultReset;   // 0x0D
            follow[1] = InvMode::Off;              // 0x01 -- the documented clear
            follow_n  = 2;
            flt_clear = true;                      // + Flt_Clear on that Off
        }
    }

    CtrlOutput out{};
    out.state       = state_;
    out.torque_pct  = cmd_torque;
    out.inv_mode    = mode;
    out.inv_mode_follow[0]  = follow[0];
    out.inv_mode_follow[1]  = follow[1];
    out.inv_mode_follow_n   = follow_n;
    out.inv_flt_clear       = flt_clear;
    out.rtds_on     = rtds;
    out.ok_to_drive = drive;
    out.ev_2_3      = ev23_latched_;
    out.t11_8_9     = t11;
    out.dv_mode     = dv_latched_;
    return out;
}

}  // namespace ecu
