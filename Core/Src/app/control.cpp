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
    const uint8_t a1 = apps_pct(in.apps1_raw, in.cal.apps1_min, in.cal.apps1_max);
    const uint8_t a2 = apps_pct(in.apps2_raw, in.cal.apps2_min, in.cal.apps2_max);

    uint8_t torque = 0;
    if (a1 > AppsAgreementPct && a2 > AppsAgreementPct) {
        torque = static_cast<uint8_t>((static_cast<uint16_t>(a1) + a2) / 2u);
    }
    if (torque < DeadbandLowPct) torque = 0;
    else if (torque > DeadbandHighPct) torque = 100;

    // The EV.2.3 brake+throttle plausibility cut USED TO BE HERE. It was deleted
    // in FS-Rules 2024 and is gone from the firmware with it (#177).
    //
    // It was a latching torque cut: brake above cal.brake_pressed with demand
    // over 25 % zeroed torque until the driver fully lifted. Two reasons not to
    // keep it as an optional safety net once the rule stopped requiring it --
    // it tripped on brake_pressed, which is still COMMISSION-tagged and has
    // never been measured, and being a latch, a spurious trip took drive away
    // mid-corner until a full lift. An unnecessary cut on an unverified
    // threshold is a hazard of its own.
    //
    // T.11.8.9 below is UNAFFECTED -- APPS-disagreement is a different rule and
    // is still required.

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

    if (t11) torque = 0;

    // DV torque source (#17): when the DV drive is latched the pedals are NOT
    // the torque source -- the conditioned uDV 0x507 command is, and a stale
    // command stream means torque 0, NEVER a fall-back to APPS (no driver is
    // seated). T.11.8.9 is a driver-pedal rule and does not gate the DV
    // command. Its latch keeps computing above (pedals idle; the pit-diag
    // verdict stays live) but the zeroing applies to the pedal torque only.
    // The cell-voltage derate below still applies -- pack protection is
    // mode-independent.
    if (dv_latched_) {
        torque = in.dv_fresh ? in.dv_torque_pct : 0;
    }

    // Low-cell-voltage derate (applied -- see note above). Runs every tick,
    // including before Active, so the filter is already settled on the real
    // pack voltage by the time torque is first commanded rather than converging
    // through the first second of the run.
    //
    // The input is an ESTIMATED open-circuit voltage, not the loaded reading:
    // sag under acceleration is ohmic and transient, and derating on it makes
    // the derate a function of throttle instead of state of charge. See
    // cell_derate.hpp.
    CellDerateInputs cdi{};
    cdi.v_cell_min_mV = in.v_cell_min_mV;
    cdi.v_fresh       = in.ams_fresh;
    cdi.current_dA    = in.current_accu_dA;
    cdi.i_fresh       = in.current_fresh;
    cell_derate_ = cell_.update(cdi);

    torque = static_cast<uint8_t>(static_cast<uint32_t>(torque) * cell_derate_.cap_pct / 100u);

    // EV 2.2.1 tractive-power envelope (#177). LAST, so nothing downstream can
    // put torque back above it, and feed-forward from measured speed so it
    // closes no loop. Before this, nothing in the vehicle enforced the 80 kW
    // limit and the map commanded roughly twice it over most of the speed
    // range. Inert below ~2865 mech rpm, so normal cornering is untouched.
    bool power_capped = false;
    {
        const uint8_t cap = power_cap_pct(in.motor_rpm_mech);
        if (torque > cap) { torque = cap; power_capped = true; }
    }

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
        if (in.start_button && in.brake_raw > in.cal.brake_arm) {
            enter_(CtrlState::R2dDelay, now_ms);
        } else if (in.dv_r2d_req && in.brake_raw > in.cal.brake_dv_hard) {
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
        // Climb to Ready. From Standby(3) that is a direct Ready(0x04); from
        // Off(0)/Shutdown(13) it is NOT -- this A16 config will not take Ready
        // from those states.
        //
        // BENCH EVIDENCE (2026-07-29, firmware 44688b6, on stands at 355 V; #168):
        // parked in WaitInvStandby with inv_state=13 Shutdown, commanding
        // Ready(0x04) at 100 Hz, L1/L2 fault layers CLEAN (PwrStg 0x001 alive,
        // EMCtrl 0x01 init_ok) and the DEM only latched history -- and the
        // inverter never moved. Nothing was blocking it; it simply does not
        // accept Ready from Shutdown.
        //
        // THIS IS NOT A RETURN TO #144. #144 sent Off INSTEAD of Ready and never
        // followed with Ready, so the inverter could not climb at all and #155
        // rightly reverted it. The IFS07 VCU -- the only configuration known to
        // have recovered without a power cycle -- sent BOTH in one pass, via the
        // fall-through in its App_State switch (pre-jarama main.c:1788-1811):
        //     case 13 -> 0x01                (Shutdown: Off only)
        //     case 0  -> 0x01 then 0x04      (Off: Off THEN Ready, same pass)
        // That is what is reproduced here, using the same follow-word mechanism
        // as the fault burst. Ready is ALWAYS still sent for state 0; the Off
        // merely precedes it. See #148.
        //
        // Note the manual's 9.1 diagram shows OFF --(READY)--> READY as one
        // direct transition -- but its App_State_Req enum (1..5) does not match
        // this A16 config at all, so trust the bench for numbering and the
        // diagram for topology only.
        //
        // A latched fault (10/11) is still overridden to its reset word by the
        // reactive block below. Reaching Ready(4) advances to Active (above).
        if (in.inv_state == InvShutdownState) {
            mode = InvMode::Off;            // 0x01 -- legacy case 13
        } else if (in.inv_state == InvOffState) {
            mode      = InvMode::Off;       // 0x01 -- legacy case 0 ...
            follow[0] = InvMode::Ready;     // 0x04 -- ... falling through to case 3
            follow_n  = 1;
        } else {
            mode = InvMode::Ready;          // 0x04 -- standby(3) -> ready
        }
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
    out.power_capped = power_capped;
    out.torque_nm    = torque_pct_to_nm(cmd_torque);
    out.rtds_on     = rtds;
    out.ok_to_drive = drive;
    out.t11_8_9     = t11;
    out.dv_mode     = dv_latched_;
    return out;
}

}  // namespace ecu
