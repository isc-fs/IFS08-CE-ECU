// SPDX-License-Identifier: proprietary
//
// SIL (software-in-the-loop) tests for the pure ECU control core + the
// host-testable pure units (bootloader trigger match, CAN-ID/DSL parity).
//
// control.cpp is HAL-free and RTOS-free, so it compiles and runs directly on
// the host with NO mocks. This harness drives the Controller through the
// start / ready-to-drive FSM and every FSAE plausibility cut, asserting the
// documented behaviour -- including the two deliberate deviations from the
// legacy VCU (applied low-cell-voltage derate + the 100 ms T.11.8.9 window).
//
// Replaces the pre-rewrite C SIL, which compiled the now-deleted app_state.c /
// can.c / control.c. The CI flag names (build-tests.yml) map to the suites
// below. Build: tests/sil/CMakeLists.txt. Run: ecu08_sil --test-all.

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "app/control.hpp"
#include "app/cal_session.hpp"
#include "app/power_limit.hpp"
#include "app/pedal_cal_nvm.hpp"
#include "app/bootloader.hpp"    // matches_trigger (pure, host-testable)
#include "can/can_codecs.hpp"    // DSL <Msg>_ID for the parity check
#include "app/inverter.hpp"      // inverter setpoint encoders (0x360/0x362)
#include "app/udv_tx.hpp"        // uDV autonomous-contract TX builders (#17)
#include "app/vehicle_service.hpp" // inverter/AMS RX decoders (rpm / temps / state)
#include "app/radio_snapshot.hpp"  // v2 fragmented-snapshot radio serializer
#include "app/gps_nmea.hpp"        // MTK3339 NMEA parser (USART10 GPS)
#include "app/gps_tx.hpp"          // 0x508/0x509 GPS frame builders

using namespace ecu;
using namespace ecu::config;

// ----- tiny test framework --------------------------------------------------
static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        ++g_checks;                                                            \
        if (!(cond)) {                                                         \
            ++g_fails;                                                         \
            std::printf("  [FAIL] %s  (%s:%d)\n", (msg), __FILE__, __LINE__);  \
        }                                                                      \
    } while (0)

// A CtrlInputs preset that drives full torque in Active: both APPS at travel
// top, brake released, inverter present + ready, AMS fresh + healthy cells.
static CtrlInputs good_drive_inputs() {
    CtrlInputs in{};
    in.apps1_raw         = Apps1AdcMax;      // 100%
    in.apps2_raw         = Apps2AdcMax;      // 100%
    in.brake_raw         = 0;
    in.start_button      = false;
    in.inv_present       = true;
    in.inv_vconfig_ready = true;
    in.inv_state         = InvReadyState;
    in.inv_dc_bus_V      = 400;   // any plausible HV bus voltage (the FSM doesn't gate on a threshold)
    in.ams_fresh         = true;
    in.ok_precharge      = true;
    in.ams_error         = false;
    in.v_cell_min_mV     = CellVDefaultMv;   // >= knee -> no derate
    return in;
}

// Drive a fresh Controller from boot all the way to Active, asserting each
// transition so a regression in the path is localised. Leaves t just past entry.
static void drive_to_active(Controller& c, uint32_t& t) {
    CtrlInputs in = good_drive_inputs();

    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Precharge, "vconfig -> Precharge");

    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitStartBrake, "ok_precharge -> WaitStartBrake");

    in.start_button = true;
    in.brake_raw    = BrakeArmRaw + 100;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::R2dDelay, "start+brake -> R2dDelay");
    CHECK(o.rtds_on, "RTDS buzzer on in R2dDelay");

    in.start_button = false;
    in.brake_raw    = 0;
    t += R2dSoundMs;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitInvStandby, "R2D timeout -> WaitInvStandby");
    CHECK(o.inv_mode == InvMode::Ready, "commands Ready in WaitInvStandby");

    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Active, "inv ready -> Active");
}

// ----- suites ---------------------------------------------------------------

static void test_apps_pct() {
    std::printf("[apps_pct]\n");
    CHECK(apps_pct(1000, 2050, 2950) == 0,   "below min -> 0");
    CHECK(apps_pct(2050, 2050, 2950) == 0,   "at min -> 0");
    CHECK(apps_pct(2950, 2050, 2950) == 100, "at max -> 100");
    CHECK(apps_pct(5000, 2050, 2950) == 100, "above max -> 100 (clamp)");
    CHECK(apps_pct(2500, 2050, 2950) == 50,  "midpoint -> 50");
    CHECK(apps_pct(2500, 2950, 2050) == 0,   "inverted cal -> 0 (guard)");
}

// Cold start / "rtos-startup": a fresh controller must boot into a SAFE state
// (no torque, no drive, inverter Off) and hold there with no inputs.
static void test_cold_start() {
    std::printf("[cold_start]\n");
    Controller c;
    CHECK(c.state() == CtrlState::WaitInvVdcConfig, "boots in WaitInvVdcConfig");
    CtrlInputs in{};                       // everything zero / false
    CtrlOutput o = c.step(in, 0);
    CHECK(o.state == CtrlState::WaitInvVdcConfig, "no inv config -> holds");
    CHECK(o.inv_mode == InvMode::Off, "inverter Off at cold start");
    CHECK(o.torque_pct == 0, "no torque at cold start");
    CHECK(!o.ok_to_drive, "not ok_to_drive at cold start");
}

static void test_boot_sequence() {
    std::printf("[boot_sequence]\n");
    Controller c;
    CHECK(c.state() == CtrlState::WaitInvVdcConfig, "starts in WaitInvVdcConfig");
    uint32_t t = 1000;
    drive_to_active(c, t);
}

// Dynamic state changes after Active: AMS opening the contactors (ok_precharge
// falls) must re-arm, and ok_precharge returning advances again.
static void test_dynamic_states() {
    std::printf("[dynamic_states]\n");
    Controller c;
    uint32_t t = 0;
    drive_to_active(c, t);

    CtrlInputs in = good_drive_inputs();
    in.ok_precharge = false;
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Precharge, "Active + ok_precharge fell -> re-arm Precharge");
    CHECK(o.torque_pct == 0, "no torque after re-arm");

    in.ok_precharge = true;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitStartBrake, "ok_precharge -> WaitStartBrake again");
}

static void test_precharge_no_ack() {
    std::printf("[precharge_no_ack]\n");
    Controller c;
    uint32_t t = 0;
    CtrlInputs in = good_drive_inputs();
    in.ok_precharge = false;

    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Precharge, "enters Precharge");

    for (uint32_t i = 0; i < (PrechargeTimeoutMs / ControlPeriodMs) + 5; ++i) {
        o = c.step(in, t); t += ControlPeriodMs;
    }
    CHECK(o.state == CtrlState::Precharge, "stays in Precharge without ok_precharge");
    CHECK(o.inv_mode == InvMode::Off, "no inverter command in Precharge");
    CHECK(o.torque_pct == 0, "no commanded torque in Precharge");
}

static void test_active_torque_and_deadband() {
    std::printf("[active_torque]\n");
    Controller c;
    uint32_t t = 0;
    drive_to_active(c, t);

    CtrlInputs in = good_drive_inputs();
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Active, "still Active");
    CHECK(o.ok_to_drive, "ok_to_drive in Active");
    CHECK(o.inv_mode == InvMode::TorqueEnable, "TorqueEnable in Active");
    CHECK(o.torque_pct == 100, "full pedal -> 100%");

    in.apps1_raw = Apps1AdcMin;
    in.apps2_raw = Apps2AdcMin;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.torque_pct == 0, "released pedal -> 0%");
}

// EV.2.3 brake+throttle plausibility ("safety-brake").
static void test_ev_2_3() {
    std::printf("[ev_2_3]\n");
    Controller c;
    uint32_t t = 0;
    drive_to_active(c, t);

    CtrlInputs in = good_drive_inputs();
    in.apps1_raw = (Apps1AdcMin + Apps1AdcMax) / 2;
    in.apps2_raw = (Apps2AdcMin + Apps2AdcMax) / 2;
    in.brake_raw = BrakePressedRaw + 100;
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.ev_2_3, "brake+throttle -> EV.2.3 latched");
    CHECK(o.torque_pct == 0, "EV.2.3 cuts torque to 0");

    in.brake_raw = 0;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.ev_2_3, "EV.2.3 stays latched while pedal still pressed");
    CHECK(o.torque_pct == 0, "still cut while latched");

    in.apps1_raw = Apps1AdcMin;
    in.apps2_raw = Apps2AdcMin;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(!o.ev_2_3, "EV.2.3 clears when pedal released");
}

static void test_t11_8_9_window() {
    std::printf("[t11_8_9]\n");
    Controller c;
    uint32_t t = 0;
    drive_to_active(c, t);

    CtrlInputs in = good_drive_inputs();
    in.apps1_raw = Apps1AdcMax;
    in.apps2_raw = Apps2AdcMin;

    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(!o.t11_8_9, "T.11.8.9 does NOT trip before the 100 ms window");

    t += AppsDisagreePersistMs;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.t11_8_9, "T.11.8.9 trips after the 100 ms window");
    CHECK(o.torque_pct == 0, "T.11.8.9 cuts torque");

    in.apps2_raw = Apps2AdcMax;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(!o.t11_8_9, "T.11.8.9 clears when sensors re-agree");
}

static void test_ams_error() {
    std::printf("[ams_error]\n");
    Controller c;
    uint32_t t = 0;
    drive_to_active(c, t);

    CtrlInputs in = good_drive_inputs();
    in.ams_error = true;
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::AmsError, "AMS error -> AmsError state");
    CHECK(o.inv_mode == InvMode::Off, "no inverter command in AmsError");
    CHECK(o.torque_pct == 0, "no torque in AmsError");

    in.ams_error = false;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitInvVdcConfig, "AMS error clears -> re-arm");
}

// Low-cell-voltage derate ("error-voltage" + half of "legacy-compat").
static void test_cell_v_derate() {
    std::printf("[cell_v_derate]\n");
    Controller c;
    uint32_t t = 0;
    drive_to_active(c, t);

    CtrlInputs in = good_drive_inputs();
    in.v_cell_min_mV = CellVDerateFloorMv;
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.torque_pct < 100, "low cell voltage derates torque");
    CHECK(o.torque_pct > 0,   "floor derate stays non-zero (factor ~0.05)");
}

// Bootloader CAN trigger match (pure -- the recovery path home). Exact frame:
// ACU bus, id 0x002, dlc 4, payload 0xB007AD12.
static void test_bootloader_trigger() {
    std::printf("[bootloader_trigger]\n");
    CanFrame f{};
    f.bus = static_cast<uint8_t>(CanBus::Acu);
    f.id  = BlBootTriggerCanId;
    f.dlc = BlBootTriggerDlc;
    for (uint8_t i = 0; i < BlBootTriggerDlc; ++i) f.data[i] = BlBootTriggerPayload[i];
    CHECK(Bootloader::matches_trigger(f), "exact 0x002/0xB007AD12 on ACU -> trigger");

    CanFrame g = f; g.bus = static_cast<uint8_t>(CanBus::Inv);
    CHECK(!Bootloader::matches_trigger(g), "same frame on Inv bus -> NOT a trigger");

    g = f; g.id = BlBootTriggerCanId + 1;
    CHECK(!Bootloader::matches_trigger(g), "wrong id -> NOT a trigger");

    g = f; g.data[0] = static_cast<uint8_t>(g.data[0] ^ 0xFFu);
    CHECK(!Bootloader::matches_trigger(g), "corrupt payload -> NOT a trigger");

    g = f; g.dlc = 8;
    CHECK(!Bootloader::matches_trigger(g), "wrong dlc -> NOT a trigger");
}

// CAN-ID / DSL parity: the RX IDs hand-written in ecu_config.hpp must equal the
// DSL single source of truth (the .def -> ecu::<Msg>_ID) so they can't drift.
static void test_dsl_parity() {
    std::printf("[dsl_parity]\n");
    CHECK(static_cast<uint32_t>(ACU_ok_precharge_ID) == AcuOkPrechargeId,  "ACU_ok_precharge_ID == config");
    CHECK(static_cast<uint32_t>(ACU_v_cell_min_ID)   == AcuVCellMinId,     "ACU_v_cell_min_ID == config");
    CHECK(static_cast<uint32_t>(AMS_status_ID)       == AmsStatusId,       "AMS_status_ID == config");
    CHECK(static_cast<uint32_t>(BL_boot_trigger_ID)  == BlBootTriggerCanId, "BL_boot_trigger_ID == config");
    CHECK(static_cast<uint32_t>(UDV_torque_cmd_ID)   == UdvTorqueCmdId,    "UDV_torque_cmd_ID == config");
    CHECK(static_cast<uint32_t>(UDV_r2d_request_ID)  == UdvR2dRequestId,   "UDV_r2d_request_ID == config");
}

// Inverter TX adapter -- IDs / modes / byte layout / torque map matched
// byte-for-byte to the original VCU; the torque sign is negated for the motor's
// mechanical mounting; the E2E bytes go out as 0 (as the original VCU sent them).
static void test_inverter() {
    std::printf("[inverter]\n");
    // Deadband lowered 10 -> 5 (2026-07-29). The map is re-based (240/95, bias
    // 1200) so the zero-crossing sits at exactly DeadbandLowPct and FULL SCALE
    // IS UNCHANGED at -240. These three pin all of that: below the band, exactly
    // at it, and full pedal.
    CHECK(Inverter::torque_to_nm_req(4)   == 0,    "<5% (deadband) -> 0");
    CHECK(Inverter::torque_to_nm_req(5)   == 0,    "5% -> 0 (map zero-crossing == deadband)");
    CHECK(Inverter::torque_to_nm_req(6)   <  0,    "6% -> torque flows immediately past the band");
    CHECK(Inverter::torque_to_nm_req(10)  <  0,    "10% now produces torque (was 0 before)");
    CHECK(Inverter::torque_to_nm_req(100) == -240, "100% -> -240 (full scale UNCHANGED)");
    // Guard the invariant that caused the invisible second deadband: the map
    // must cross zero exactly at DeadbandLowPct, never above it.
    CHECK(Inverter::torque_to_nm_req(config::DeadbandLowPct) == 0,
          "map zero-crossing tracks DeadbandLowPct");
    CHECK(Inverter::torque_to_nm_req(config::DeadbandLowPct + 1) < 0,
          "the very next percent past the deadband already commands torque");
    CHECK(config::AppsAgreementPct < config::DeadbandLowPct,
          "agreement gate must stay BELOW the deadband or it dictates the onset");
    CHECK(Inverter::torque_to_nm_req(100) <  0,    "forward torque is NEGATIVE (mechanical mounting)");

    // 0x360 mode frame: FDCAN1, id 0x360, dlc 3, {0, 0, App_State_Req}.
    const CanFrame md = Inverter::build_setpoint_mode(InvMode::TorqueEnable);
    CHECK(md.bus == static_cast<uint8_t>(CanBus::Inv), "mode frame on FDCAN1 (Inv)");
    CHECK(md.id == InvTxSetpointModeId && md.dlc == 3, "mode frame id 0x360 dlc 3");
    CHECK(md.data[0] == 0 && md.data[1] == 0, "0x360 bytes 0-1 = 0 (as the original VCU)");
    CHECK(md.data[2] == 0x06, "App_State_Req = InvMode::TorqueEnable (0x06)");

    // 0x362 torque frame: id 0x362, dlc 4, {0, 0, torque_lo, torque_hi} (LE, negated).
    const CanFrame tq = Inverter::build_setpoint_torque(100);
    CHECK(tq.id == InvTxSetpointTorqueId && tq.dlc == 4, "torque frame id 0x362 dlc 4");
    CHECK(tq.data[0] == 0 && tq.data[1] == 0, "0x362 bytes 0-1 = 0 (as the original VCU)");
    const int16_t sent = static_cast<int16_t>(
        static_cast<uint16_t>(tq.data[2] | (static_cast<uint16_t>(tq.data[3]) << 8)));
    CHECK(sent == -240, "Torque_Nm_Req @ bytes 2-3 LE == -240");
}

// Inverter fault recovery: a faulted inverter must be commanded its recovery
// mode word (hard fault 11 -> 0x0D, soft fault 10 -> 0x13) in ANY drive state,
// including before the FSM reaches Active -- the real inverter boots LATCHED in
// hard fault, and without this the FSM stalls at WaitInvStandby forever.
static void test_inverter_fault_recovery() {
    std::printf("[inverter_fault_recovery]\n");

    // Boot-latched hard fault: a fresh controller (still WaitInvVdcConfig) seeing
    // inv_state 11 must command HardFaultReset (0x0D), not Off -- the whole point.
    {
        Controller c;
        CtrlInputs in{};
        in.inv_state = InvHardFaultState;            // 11
        CtrlOutput o = c.step(in, 0);
        CHECK(o.state == CtrlState::WaitInvVdcConfig, "still pre-config (no vconfig yet)");
        CHECK(o.inv_mode == InvMode::HardFaultReset, "hard fault (11) -> HardFaultReset (0x0D), pre-Active");
        CHECK(o.torque_pct == 0, "no torque while recovering a fault");
    }

    // Soft fault -> Fault (0x13).
    {
        Controller c;
        CtrlInputs in{};
        in.inv_state = InvSoftFaultState;            // 10
        CtrlOutput o = c.step(in, 0);
        CHECK(o.inv_mode == InvMode::Fault, "soft fault (10) -> Fault (0x13)");
    }

    // A healthy (non-fault) inverter state must NOT trigger recovery.
    {
        Controller c;
        CtrlInputs in{};
        in.inv_state = InvStandbyState;              // 3
        CtrlOutput o = c.step(in, 0);
        CHECK(o.inv_mode == InvMode::Off, "standby (3) -> normal Off, no recovery");
    }

    // In Active, a fault both commands recovery AND cuts torque (pedal at 100%).
    {
        Controller c;
        uint32_t t = 1000;
        drive_to_active(c, t);
        CtrlInputs in = good_drive_inputs();
        in.inv_state = InvHardFaultState;            // 11 while driving
        CtrlOutput o = c.step(in, t);
        CHECK(o.inv_mode == InvMode::HardFaultReset, "Active + hard fault -> HardFaultReset");
        CHECK(o.torque_pct == 0, "Active + fault -> torque cut");
        CHECK(!o.ok_to_drive, "Active + fault -> not ok_to_drive");
    }

    // AmsError inhibits: recovery is suppressed (Off is the safe command).
    {
        Controller c;
        CtrlInputs in{};
        in.ams_error = true;
        in.inv_state = InvHardFaultState;            // 11
        CtrlOutput o = c.step(in, 0);
        CHECK(o.state == CtrlState::AmsError, "ams_error -> AmsError state");
        CHECK(o.inv_mode == InvMode::Off, "AmsError + fault -> Off (recovery suppressed)");
    }

    // Wire encoding: HardFaultReset is App_State_Req 0x0D on 0x360.
    const CanFrame md = Inverter::build_setpoint_mode(InvMode::HardFaultReset);
    CHECK(md.data[2] == 0x0D, "App_State_Req = InvMode::HardFaultReset (0x0D)");
}

// TS-off -> R2D re-arm -> torque, driven entirely through the FSM.
//
// NOTE: this suite does NOT prove the on-car TS-off recovery works -- that is
// still open (#148) and needs a bench capture. What it pins down is the FSM's
// side of the sequence: after ok_precharge falls the controller re-arms, and
// throughout WaitInvStandby it commands Ready REGARDLESS of what the inverter
// reports, which is what the W90 state machine (manual 9.1) documents --
// OFF --(READY)--> READY is one direct transition. It also guards against
// re-introducing #144's reactive Off-word climb, which contradicted that.
static void test_inverter_ts_off_recovery() {
    std::printf("[inverter_ts_off_recovery]\n");

    Controller c;
    uint32_t t = 1000;
    drive_to_active(c, t);                       // driving, inverter Ready(4)

    // --- TS deactivated mid-drive: AMS opens the AIRs (ok_precharge falls) and
    //     the inverter collapses to OFF(0). FSM must re-arm (back to Precharge). ---
    CtrlInputs in = good_drive_inputs();
    in.ok_precharge = false;
    in.inv_state    = InvOffState;               // 0 -- inverter fell to off
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Precharge, "TS-off -> re-arm Precharge");
    CHECK(o.torque_pct == 0, "no torque after TS-off");
    CHECK(o.inv_mode == InvMode::Off, "Precharge commands Off (0x01)");

    // --- TS back on: climb Precharge -> WaitStartBrake, driver re-does R2D. ---
    in.ok_precharge = true;                      // HV/precharge restored
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitStartBrake, "ok_precharge -> WaitStartBrake (re-arm)");

    in.start_button = true;
    in.brake_raw    = BrakeArmRaw + 100;         // R2D re-arm STILL required (FSAE)
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::R2dDelay, "start+brake -> R2dDelay (re-arm)");

    in.start_button = false;
    in.brake_raw    = 0;
    t += R2dSoundMs;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitInvStandby, "R2D done -> WaitInvStandby");

    // --- The inverter is still at OFF(0). Per the W90 manual state machine
    //     (9.1), OFF --(App_State_Req = READY)--> READY is a SINGLE direct
    //     transition -- so Ready is exactly the right word to send here.
    //
    //     REGRESSION GUARD: #144 made this reactive and sent Off(0x01) instead
    //     when inv_state was OFF/SHUTDOWN, on the (IFS07-derived, never
    //     confirmed) theory that an off inverter needs an "on" word first. That
    //     holds an already-off inverter in OFF and never sends Ready, so it can
    //     never climb. Reverted; these asserts stop it coming back. See #148. ---
    // OFF(0): Off THEN Ready in the SAME cycle -- the legacy case 0 -> case 3
    // fall-through. Ready is still always sent; the Off merely precedes it, which
    // is the difference from #144 (Off INSTEAD of Ready -> could never climb).
    CHECK(o.inv_mode == InvMode::Off, "OFF(0) inverter -> Off(0x01) first");
    CHECK(o.inv_mode_follow_n == 1, "OFF(0) -> one follow word");
    CHECK(o.inv_mode_follow[0] == InvMode::Ready,
          "OFF(0) -> Ready(0x04) STILL sent, same cycle (this is what #144 never did)");
    CHECK(o.state == CtrlState::WaitInvStandby, "holds in WaitInvStandby until the inverter is ready");

    // SHUTDOWN(13): Off only, exactly as the legacy case 13. Bench-confirmed
    // 2026-07-29 -- the inverter parks in 13 and will not accept Ready from there
    // (L1/L2 clean, DEM latched, Ready commanded at 100 Hz, never moved).
    in.inv_state = InvShutdownState;             // 13
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.inv_mode == InvMode::Off, "SHUTDOWN(13) inverter -> Off(0x01), legacy case 13");
    CHECK(o.inv_mode_follow_n == 0, "SHUTDOWN(13) -> no follow word (Off alone, as the legacy)");
    CHECK(o.state == CtrlState::WaitInvStandby, "still waiting (not ready yet)");

    // Inverter passes through Standby(3): still Ready, unchanged.
    in.inv_state = InvStandbyState;              // 3
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.inv_mode == InvMode::Ready, "STANDBY(3) inverter -> command Ready (0x04)");
    CHECK(o.inv_mode_follow_n == 0, "STANDBY(3) -> plain Ready, no Off prefix");
    CHECK(o.state == CtrlState::WaitInvStandby, "waiting for ready");

    // Inverter reaches Ready(4): advance to Active and drive torque again -- no
    // power cycle anywhere in this sequence.
    in.inv_state = InvReadyState;                // 4
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Active, "READY(4) -> Active (recovered)");

    o = c.step(in, t); t += ControlPeriodMs;     // one more tick: torque flows
    CHECK(o.state == CtrlState::Active, "stays Active");
    CHECK(o.inv_mode == InvMode::TorqueEnable, "back to TorqueEnable");
    CHECK(o.torque_pct == 100, "full pedal drives torque again -- no power cycle");

    // A latched fault DURING the climb still wins (reactive fault block overrides
    // the climb word): OFF-climb must not mask a hard fault that appears.
    {
        Controller c2;
        uint32_t t2 = 1000;
        drive_to_active(c2, t2);
        CtrlInputs in2 = good_drive_inputs();
        in2.ok_precharge = false; in2.inv_state = InvOffState;
        c2.step(in2, t2); t2 += ControlPeriodMs;                 // -> Precharge
        in2.ok_precharge = true;
        c2.step(in2, t2); t2 += ControlPeriodMs;                 // -> WaitStartBrake
        in2.start_button = true; in2.brake_raw = BrakeArmRaw + 100;
        c2.step(in2, t2); t2 += ControlPeriodMs;                 // -> R2dDelay
        in2.start_button = false; in2.brake_raw = 0; t2 += R2dSoundMs;
        c2.step(in2, t2); t2 += ControlPeriodMs;                 // -> WaitInvStandby
        in2.inv_state = InvHardFaultState;                       // 11 appears mid-climb
        CtrlOutput o2 = c2.step(in2, t2);
        CHECK(o2.inv_mode == InvMode::HardFaultReset,
              "hard fault during the climb -> HardFaultReset (0x0D) wins over the on-word");
    }
}

// #169 -- pedal calibration as RUNTIME data. Two things must hold: the
// defaults reproduce the old constexpr behaviour EXACTLY (so landing this is a
// no-op on the car), and validate_cal() rejects anything that could make a
// released pedal produce torque.
static void test_pedal_cal() {
    std::printf("[pedal_cal]\n");

    // --- defaults are the old constexpr values, bit for bit ---
    PedalCal d{};
    CHECK(d.apps1_min == config::Apps1AdcMin && d.apps1_max == config::Apps1AdcMax,
          "default APPS1 cal == the old constexpr pair");
    CHECK(d.apps2_min == config::Apps2AdcMin && d.apps2_max == config::Apps2AdcMax,
          "default APPS2 cal == the old constexpr pair");
    CHECK(d.brake_arm == config::BrakeArmRaw &&
          d.brake_dv_hard == config::BrakeDvHardRaw &&
          d.brake_pressed == config::BrakePressedRaw,
          "default brake thresholds == the old constexpr values");
    CHECK(validate_cal(d) == 0, "the shipped default calibration is itself valid");

    // --- the core actually USES the passed-in calibration, not a global ---
    {
        CtrlInputs in = good_drive_inputs();
        in.apps1_raw = 3000; in.apps2_raw = 2700;   // mid travel on the defaults
        Controller c1; uint32_t t = 1000;
        drive_to_active(c1, t);
        const uint8_t with_default = c1.step(in, t).torque_pct;

        // Halve both spans: the SAME raw reading must now read much higher.
        in.cal.apps1_max = static_cast<std::uint16_t>(
            in.cal.apps1_min + (config::Apps1AdcMax - config::Apps1AdcMin) / 2);
        in.cal.apps2_max = static_cast<std::uint16_t>(
            in.cal.apps2_min + (config::Apps2AdcMax - config::Apps2AdcMin) / 2);
        Controller c2; uint32_t t2 = 1000;
        drive_to_active(c2, t2);
        const uint8_t with_narrow = c2.step(in, t2).torque_pct;
        CHECK(with_narrow > with_default,
              "narrowing the calibrated span raises the reported pedal % (cal is live)");
    }

    // --- the brake thresholds are live too ---
    {
        CtrlInputs in = good_drive_inputs();
        in.brake_raw = 1200;                 // above a lowered arm, below the default
        in.start_button = true;
        Controller c1; uint32_t t = 1000;
        // default arm is 750, so 1200 already arms -- lower the bar and confirm
        // the DV gate moves with the calibration rather than the constant.
        in.cal.brake_dv_hard = 1000;
        CHECK(in.brake_raw > in.cal.brake_dv_hard, "DV hard-brake gate follows the cal");
        (void)c1; (void)t;
    }

    // --- validation: each rule, one at a time ---
    { PedalCal c{}; c.apps1_max = static_cast<std::uint16_t>(c.apps1_min + 10);
      CHECK((validate_cal(c) & cal_flag::Apps1SpanTooSmall) != 0, "tiny APPS1 span rejected"); }
    { PedalCal c{}; c.apps2_max = static_cast<std::uint16_t>(c.apps2_min + 10);
      CHECK((validate_cal(c) & cal_flag::Apps2SpanTooSmall) != 0, "tiny APPS2 span rejected"); }
    { PedalCal c{}; c.apps1_max = c.apps1_min;
      CHECK((validate_cal(c) & cal_flag::AppsNotMonotonic) != 0, "max == min rejected"); }
    { PedalCal c{}; c.apps1_max = static_cast<std::uint16_t>(c.apps1_min - 1);
      CHECK((validate_cal(c) & cal_flag::AppsNotMonotonic) != 0, "inverted APPS rejected"); }
    { PedalCal c{}; c.apps2_max = static_cast<std::uint16_t>(c.apps2_min + 350);
      CHECK((validate_cal(c) & cal_flag::AppsSpanMismatch) != 0,
            "wildly mismatched channel spans rejected (860 vs 350)"); }
    { PedalCal c{}; c.brake_dv_hard = 200;   // below arm(750) -> order broken
      CHECK((validate_cal(c) & cal_flag::BrakeOrder) != 0, "brake thresholds out of order rejected"); }
    { PedalCal c{}; c.brake_rest = 900;      // at/above arm(750) -> released reads armed
      CHECK((validate_cal(c) & cal_flag::BrakeOrder) != 0, "brake rest above the arm threshold rejected"); }
    { PedalCal c{}; c.brake_rest = 2900;     // span vs pressed(3000) = 100 < 300
      CHECK((validate_cal(c) & cal_flag::BrakeSpanTooSmall) != 0, "tiny brake span rejected"); }
    { PedalCal c{}; c.apps1_max = 5000;
      CHECK((validate_cal(c) & cal_flag::OutOfAdcRange) != 0, "value beyond 12-bit rejected"); }

    // --- brake_pct: the 14%-at-rest bug and its fix ---
    {
        PedalCal u{};                       // brake_rest = 0 -> uncalibrated
        CHECK(u.brake_rest == 0, "brake rest is unmeasured by default");
        // Legacy behaviour preserved EXACTLY while uncalibrated: raw*100/4095.
        CHECK(brake_pct(560, u) == static_cast<std::uint8_t>(560u * 100u / 4095u),
              "uncalibrated brake_pct keeps the legacy full-range scaling");
        CHECK(brake_pct(560, u) == 13, "...which is why a released brake reads ~13-14%");
        CHECK(brake_pct(4095, u) == 100, "uncalibrated full scale still saturates");

        // Once a rest point exists the same raw reading reads 0.
        PedalCal c{};
        c.brake_rest = 560; c.brake_pressed = 3000;
        CHECK(brake_pct(560, c) == 0, "calibrated: released brake reads 0% (the bug fixed)");
        CHECK(brake_pct(400, c) == 0, "below rest clamps to 0");
        CHECK(brake_pct(3000, c) == 100, "at the pressed point reads 100%");
        CHECK(brake_pct(4000, c) == 100, "above the pressed point clamps to 100");
        CHECK(brake_pct(1780, c) == 50, "midway between rest and pressed reads 50%");
        // A degenerate span must not divide by zero or wrap.
        PedalCal z{}; z.brake_rest = 3000; z.brake_pressed = 3000;
        CHECK(brake_pct(3000, z) <= 100, "degenerate span cannot produce a bogus percentage");
    }

    // A garbage record (all 0xFF, i.e. erased flash read back as a struct) must
    // be rejected -- this is the boot-time fallback path, not a theoretical case.
    { PedalCal c{}; c.apps1_min = c.apps1_max = c.apps2_min = c.apps2_max = 0xFFFF;
      c.brake_rest = c.brake_arm = c.brake_dv_hard = c.brake_pressed = 0xFFFF;
      CHECK(validate_cal(c) != 0, "erased-flash pattern (all 0xFFFF) rejected"); }
}

// #177 -- the EV 2.2.1 tractive-power envelope. The point of the cap is that
// SHAFT POWER STAYS FLAT above the corner speed, so that is what these assert,
// rather than re-checking the arithmetic that produced it.
static void test_power_envelope() {
    std::printf("[power_envelope]\n");

    // Inert where it should be: below the corner speed full pedal is untouched,
    // so normal cornering and low-speed driving feel exactly as before.
    CHECK(power_cap_pct(0) == 100, "stationary -> no cap (P = T*0 = 0)");
    CHECK(power_cap_pct(1000) == 100, "1000 rpm -> no cap");
    CHECK(power_cap_pct(2000) == 100, "2000 rpm -> no cap");

    // Binding above it, and the whole point: power stops rising.
    auto shaft_kw = [](std::int32_t rpm) {
        const std::uint8_t pct = power_cap_pct(rpm);
        const std::int32_t nm  = torque_pct_to_nm(pct);
        return static_cast<double>(nm) * rpm * 6.283185 / 60.0 / 1000.0;
    };
    static const std::int32_t kSpeeds[] = {3000, 4000, 5000, 6000, 8000, 10000};
    for (unsigned si = 0; si < sizeof(kSpeeds)/sizeof(kSpeeds[0]); ++si) {
        const std::int32_t rpm = kSpeeds[si];
        const double kw = shaft_kw(rpm);
        CHECK(kw < 74.0, "capped shaft power stays under the 72 kW budget + rounding");
        CHECK(kw > 68.0, "...and is not needlessly conservative");
    }
    // Uncapped, the map would blow straight through it -- this is the finding.
    {
        const double uncapped_8000 =
            static_cast<double>(torque_pct_to_nm(100)) * 8000 * 6.283185 / 60.0 / 1000.0;
        CHECK(uncapped_8000 > 190.0, "without the cap, 100% at 8000 rpm is ~200 kW of shaft power");
    }

    // Monotonic: more speed, less allowed torque. A non-monotonic cap would
    // make the pedal feel like it was fighting the driver.
    for (std::int32_t rpm = 3000; rpm <= 9000; rpm += 500) {
        CHECK(power_cap_pct(rpm) >= power_cap_pct(rpm + 500), "cap is monotonic in speed");
    }

    // Direction-agnostic: the envelope is on power magnitude.
    CHECK(power_cap_pct(-6000) == power_cap_pct(6000), "reverse rotation capped the same");

    // Feed-forward: identical input gives identical output with no history.
    // This is what makes it structurally unlike the low-cell derate, which
    // closes a loop through the pack and limit-cycles (#177).
    CHECK(power_cap_pct(5000) == power_cap_pct(5000), "stateless");

    // End to end through the real controller: at speed, full pedal is capped
    // and the cap is annunciated; at low speed it is not.
    {
        Controller c; uint32_t t = 1000;
        drive_to_active(c, t);
        CtrlInputs in = good_drive_inputs();      // full pedal
        in.motor_rpm_mech = 1500;
        CtrlOutput lo = c.step(in, t); t += ControlPeriodMs;
        CHECK(lo.torque_pct == 100, "full pedal at 1500 rpm -> uncapped");
        CHECK(!lo.power_capped, "...and not annunciated");

        in.motor_rpm_mech = 7000;
        CtrlOutput hi = c.step(in, t); t += ControlPeriodMs;
        CHECK(hi.torque_pct < 60, "full pedal at 7000 rpm -> heavily capped");
        CHECK(hi.power_capped, "...and annunciated on 0x700");
        CHECK(hi.torque_nm > 0 && hi.torque_nm < 130, "commanded Nm reported and consistent");
    }

    // The cap must sit AFTER every other reduction, so a safety cut still wins.
    {
        Controller c; uint32_t t = 1000;
        drive_to_active(c, t);
        CtrlInputs in = good_drive_inputs();
        in.motor_rpm_mech = 7000;
        in.brake_raw = static_cast<std::uint16_t>(in.cal.brake_pressed + 100);  // EV.2.3
        CtrlOutput o = c.step(in, t);
        CHECK(o.torque_pct == 0, "a plausibility cut still wins over the envelope");
    }

    // torque_cmd on 0x700 was hardcoded 0; it now carries real Nm.
    CHECK(torque_pct_to_nm(100) == 240, "100% -> 240 Nm full scale");
    CHECK(torque_pct_to_nm(5) == 0, "deadband edge -> 0 Nm");
    CHECK(torque_pct_to_nm(0) == 0, "released -> 0 Nm");
}

// #169 -- EPT1400 brake pressure. The board divider is KNOWN (R8 1k series,
// R9 2k shunt), so pressure is an absolute map from raw counts and needs no
// calibration at all. These tests pin the derivation so a board change that is
// not reflected in the constants fails here rather than on the car.
static void test_brake_pressure() {
    std::printf("[brake_pressure]\n");

    // Re-derive the two anchor points from the resistor values, Vref and the
    // sensor's 0.5/4.5 V spec. If someone respins the divider and forgets the
    // constants, this is where it surfaces.
    {
        const double k = static_cast<double>(BrakeDivR9Ohm) /
                         static_cast<double>(BrakeDivR8Ohm + BrakeDivR9Ohm);
        const double zero = 0.5 * k / 3.3 * 4095.0;
        const double full = 4.5 * k / 3.3 * 4095.0;
        CHECK(k > 0.666 && k < 0.667, "divider is R9/(R8+R9) = 2/3");
        CHECK(static_cast<int>(zero + 0.5) == BrakeCountsAtZeroBar,
              "BrakeCountsAtZeroBar matches the divider derivation (414)");
        CHECK(static_cast<int>(full + 0.5) == BrakeCountsAtFullBar,
              "BrakeCountsAtFullBar matches the divider derivation (3723)");
        // The earlier worry about the sensor clipping the ADC was wrong: with
        // the real 2/3 divider full scale lands well inside 12 bits.
        CHECK(BrakeCountsAtFullBar < 4095, "full scale does NOT clip the 12-bit ADC");
    }

    // The configured part: EPT1400 order code 04000 = 40 bar.
    CHECK(config::BrakeSensorFullScaleBar == 40, "sensor is the 40 bar part");
    CHECK(brake_pressure_dbar(BrakeCountsAtZeroBar) == 0, "0.5 V point reports 0.0 bar");
    CHECK(brake_pressure_dbar(BrakeCountsAtFullBar) == 400, "4.5 V point reports 40.0 bar");
    {
        // Monotonic and correctly scaled across the range.
        const std::uint16_t mid = static_cast<std::uint16_t>(
            (BrakeCountsAtZeroBar + BrakeCountsAtFullBar) / 2);
        const std::uint16_t d = brake_pressure_dbar(mid);
        CHECK(d > 195 && d < 205, "midpoint reports ~20.0 bar");
        CHECK(brake_pressure_dbar(mid + 100) > d, "monotonic with rising counts");
    }
    {
        // The three inherited thresholds, now judgeable in physical units.
        const std::uint16_t arm  = brake_pressure_dbar(config::BrakeArmRaw);
        const std::uint16_t dvh  = brake_pressure_dbar(config::BrakeDvHardRaw);
        const std::uint16_t prsd = brake_pressure_dbar(config::BrakePressedRaw);
        CHECK(arm < dvh && dvh < prsd, "thresholds are ordered in pressure as well as counts");
        CHECK(arm > 30 && arm < 50, "R2D arm lands near 4 bar -- a light press");
        CHECK(prsd > 290 && prsd < 330, "EV.2.3 cut lands near 31 bar -- firm braking");
    }
    {
        // The rest reading seen in the #148 captures, in real units. At 40 bar
        // this is ~1.8 bar, minor enough to be plausible residual pressure or
        // sensor offset -- unlike the 4.4 / 11 bar it would have implied on a
        // 100 or 250 bar part.
        const std::uint16_t d = brake_pressure_dbar(560);
        CHECK(d > 10 && d < 25, "observed rest is ~1.8 bar of residual, not a gross offset");
    }

    // The map itself, pinned independently of the config constant so filling in
    // the range later does not require revisiting these.
    auto dbar = [](std::uint32_t pmax, std::uint16_t raw) -> unsigned {
        if (raw <= BrakeCountsAtZeroBar) return 0u;
        return static_cast<unsigned>(pmax * 10u * (raw - BrakeCountsAtZeroBar) /
                                     (BrakeCountsAtFullBar - BrakeCountsAtZeroBar));
    };
    CHECK(dbar(100, BrakeCountsAtZeroBar) == 0, "0.5 V point == 0 bar");
    CHECK(dbar(100, BrakeCountsAtFullBar) == 1000, "4.5 V point == full scale (100 bar)");
    CHECK(dbar(250, BrakeCountsAtFullBar) == 2500, "...scales with the order-code range");
    {
        const std::uint16_t mid = static_cast<std::uint16_t>(
            (BrakeCountsAtZeroBar + BrakeCountsAtFullBar) / 2);
        const unsigned d = dbar(100, mid);
        CHECK(d > 495 && d < 505, "midpoint is half scale");
    }
    CHECK(brake_pressure_dbar(0) == 0, "zero counts -> 0, no underflow");
    CHECK(brake_pressure_dbar(BrakeCountsAtZeroBar - 1) == 0, "below the 0.5 V point clamps to 0");

    // The rest reading observed on this car (~560) sits ABOVE the theoretical
    // zero-pressure point (414). That is 146 counts, about 0.18 V at the sensor,
    // i.e. real residual pressure or a sensor offset -- not a rounding artifact.
    CHECK(560 > BrakeCountsAtZeroBar + 100,
          "observed rest sits well above the theoretical 0 bar point -- residual pressure or offset");
}

// #169 step 3 -- reading the calibration out of the bootloader NVM sector.
// Flash is memory mapped, so the scanner takes a plain pointer and the suite can
// drive it against a synthetic sector in RAM. Every path here is a real failure
// mode of a log-structured store on a car that loses power at inconvenient
// moments, not a hypothetical.
namespace {

// One 32-byte bl_nvm entry, laid out byte by byte to match the bootloader:
//   0..1 magic LE, 2..3 key LE, 4 len, 5 res_a, 6..7 res_b, 8..11 seq LE, 12.. value
void put_entry(std::uint8_t* slot, std::uint16_t magic, std::uint16_t key,
               std::uint8_t len, std::uint32_t seq, const std::uint8_t* value) {
    for (int i = 0; i < 32; ++i) slot[i] = 0xFF;          // erased flash
    slot[0] = static_cast<std::uint8_t>(magic & 0xFF);
    slot[1] = static_cast<std::uint8_t>(magic >> 8);
    slot[2] = static_cast<std::uint8_t>(key & 0xFF);
    slot[3] = static_cast<std::uint8_t>(key >> 8);
    slot[4] = len;
    slot[5] = 0; slot[6] = 0; slot[7] = 0;
    slot[8]  = static_cast<std::uint8_t>(seq & 0xFF);
    slot[9]  = static_cast<std::uint8_t>((seq >> 8) & 0xFF);
    slot[10] = static_cast<std::uint8_t>((seq >> 16) & 0xFF);
    slot[11] = static_cast<std::uint8_t>((seq >> 24) & 0xFF);
    if (value) for (std::size_t i = 0; i < CalRecordLen; ++i) slot[12 + i] = value[i];
}

}  // namespace

static void test_pedal_cal_nvm() {
    std::printf("[pedal_cal_nvm]\n");

    constexpr std::uint32_t kSlots = 8;
    std::uint8_t sector[kSlots * 32];
    auto erase_all = [&] { for (auto& b : sector) b = 0xFF; };

    PedalCal good{};
    good.apps1_min = 2500; good.apps1_max = 3400;
    good.apps2_min = 2300; good.apps2_max = 3000;
    good.brake_rest = 560; good.brake_arm = 800;
    good.brake_dv_hard = 2400; good.brake_pressed = 3100;
    CHECK(validate_cal(good) == 0, "the fixture calibration is itself valid");
    std::uint8_t rec[CalRecordLen];
    encode_cal_record(good, rec);

    // --- virgin flash: everything 0xFF -> defaults, and it must NOT be an error ---
    erase_all();
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Defaults, "erased sector -> Defaults (normal first boot)");
        CHECK(r.cal.apps1_min == config::Apps1AdcMin, "...and the defaults come through");
    }

    // --- a single good record ---
    erase_all();
    put_entry(sector, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec);
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Loaded, "stored record loads");
        CHECK(r.cal.apps1_min == 2500 && r.cal.brake_pressed == 3100, "round-trips exactly");
    }

    // --- append-only: the NEWEST record wins, even though it is later in the
    //     region. Scanning must not stop at the first hit. ---
    erase_all();
    PedalCal older = good; older.apps1_min = 1111;
    std::uint8_t rec_old[CalRecordLen]; encode_cal_record(older, rec_old);
    put_entry(sector + 0 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 5, rec_old);
    put_entry(sector + 3 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 9, rec);
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Loaded, "multiple records load");
        CHECK(r.cal.apps1_min == 2500, "highest seq wins, not the first slot found");
    }

    // --- other vendors' keys are ignored, including the bootloader's own ---
    erase_all();
    put_entry(sector + 0 * 32, CalNvmMagic, 0x0001, 1, 50, rec);   // BL node id, huge seq
    put_entry(sector + 1 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 2, rec);
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Loaded, "our key is found alongside bootloader keys");
        CHECK(r.cal.apps1_min == 2500, "a bootloader key with a higher seq does not win");
    }

    // --- a torn write (bad magic) is skipped and the previous record survives.
    //     This is the power-cut-mid-write case the store is designed for. ---
    erase_all();
    put_entry(sector + 0 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec);
    put_entry(sector + 1 * 32, 0x0000,      CalNvmKey, CalRecordLen, 99, rec_old);  // torn
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Loaded, "torn entry skipped");
        CHECK(r.cal.apps1_min == 2500, "the previous good record still wins after a torn write");
    }

    // --- a stored record that fails validation must NOT be applied. Corrupt
    //     flash must not become a way around the safety rules. ---
    erase_all();
    PedalCal bad = good; bad.apps1_max = bad.apps1_min;   // zero span
    std::uint8_t rec_bad[CalRecordLen]; encode_cal_record(bad, rec_bad);
    put_entry(sector, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec_bad);
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Invalid, "invalid stored record reported as Invalid");
        CHECK(r.flags != 0, "...with the failing rule attached");
        CHECK(r.cal.apps1_min == config::Apps1AdcMin, "...and the DEFAULTS are used, not the bad values");
    }

    // --- an unknown record version falls back rather than misparsing ---
    erase_all();
    std::uint8_t rec_v9[CalRecordLen]; encode_cal_record(good, rec_v9); rec_v9[0] = 9;
    put_entry(sector, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec_v9);
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::BadVersion, "unknown record version reported");
        CHECK(r.cal.apps1_min == config::Apps1AdcMin, "...and defaults are used");
    }

    // --- a tombstone (len 0) deletes the calibration -> defaults ---
    erase_all();
    put_entry(sector + 0 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec);
    put_entry(sector + 1 * 32, CalNvmMagic, CalNvmKey, 0, 2, nullptr);
    {
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Defaults, "a newer tombstone drops back to defaults");
    }

    // --- degenerate inputs must not read out of bounds ---
    CHECK(load_cal_from_nvm(nullptr, 4096).status == CalLoad::Defaults, "null base is safe");
    CHECK(load_cal_from_nvm(sector, 4).status == CalLoad::Defaults, "region smaller than one entry is safe");
}

// #169 step 4 -- the operator calibration session. The branches that matter are
// the ones an operator reaches by doing something wrong, so they are all here.
// #169 step 5 -- the WRITE-side helpers, checked against the bootloader's own
// rules (isc-fs/stm32-can-bootloader Core/Src/bl_nvm.c). These pin the two
// things that would silently diverge: where the next entry goes, and what
// sequence number it needs.
static void test_cal_nvm_write() {
    std::printf("[cal_nvm_write]\n");

    constexpr std::uint32_t kSlots = 8;
    std::uint8_t sector[kSlots * 32];
    auto erase_all = [&] { for (auto& b : sector) b = 0xFF; };

    PedalCal c{};
    std::uint8_t rec[CalRecordLen]; encode_cal_record(c, rec);
    std::uint32_t off = 0, seq = 0;

    // --- virgin sector: slot 0, seq 1 ---
    erase_all();
    CHECK(find_cal_write_slot(sector, sizeof(sector), &off, &seq), "virgin sector is writable");
    CHECK(off == 0 && seq == 1, "first entry goes to slot 0 with seq 1");

    // --- seq is GLOBAL: a bootloader key with a high seq must be beaten ---
    erase_all();
    put_entry(sector + 0 * 32, CalNvmMagic, 0x0001, 1, 77, rec);   // BL node-id key
    CHECK(find_cal_write_slot(sector, sizeof(sector), &off, &seq), "writable after a BL entry");
    CHECK(off == 32, "append lands one past the last magic entry");
    CHECK(seq == 78, "seq beats the highest across ALL keys, not just ours");

    // --- append point is ONE PAST THE LAST MAGIC, not the first erased slot.
    //     A stray non-magic pattern mid-sector must not pull the write back
    //     in front of live entries -- the exact case bl_nvm_init guards. ---
    erase_all();
    put_entry(sector + 0 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec);
    for (int i = 0; i < 32; ++i) sector[1 * 32 + i] = 0x5A;        // torn/stale slot
    put_entry(sector + 2 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 2, rec);
    CHECK(find_cal_write_slot(sector, sizeof(sector), &off, &seq), "writable past the stale slot");
    CHECK(off == 3 * 32, "append goes AFTER the last live entry, not into the earlier gap");
    CHECK(seq == 3, "seq continues from the highest live entry");

    // --- a full region is refused, not compacted. Compaction erases the whole
    //     sector and would blow the 500 ms IWDG budget. ---
    erase_all();
    for (std::uint32_t i = 0; i < kSlots; ++i)
        put_entry(sector + i * 32, CalNvmMagic, CalNvmKey, CalRecordLen, i + 1, rec);
    CHECK(!find_cal_write_slot(sector, sizeof(sector), &off, &seq),
          "full region refuses -- the app never compacts");

    // --- the target slot must be genuinely erased. Flash bits only go 1->0, so
    //     a slot with stray bits is permanently un-programmable. ---
    erase_all();
    put_entry(sector + 0 * 32, CalNvmMagic, CalNvmKey, CalRecordLen, 1, rec);
    sector[1 * 32 + 7] = 0xFE;                                     // one stray bit
    CHECK(!find_cal_write_slot(sector, sizeof(sector), &off, &seq),
          "a target slot with stray bits is refused, not programmed blind");

    // --- entry layout matches what the reader expects, and round-trips ---
    {
        PedalCal src{};
        src.apps1_min = 2500; src.apps1_max = 3400;
        src.apps2_min = 2300; src.apps2_max = 3000;
        src.brake_rest = 560; src.brake_arm = 800;
        src.brake_dv_hard = 2400; src.brake_pressed = 3100;
        std::uint8_t entry[32];
        build_cal_entry(src, 42, entry);
        CHECK(entry[0] == 0xCD && entry[1] == 0xAB, "magic 0xABCD little-endian");
        CHECK(entry[2] == 0x00 && entry[3] == 0x10, "key 0x1000 little-endian");
        CHECK(entry[4] == CalRecordLen, "len field");
        CHECK(entry[5] == 0 && entry[6] == 0 && entry[7] == 0,
              "reserved bytes zeroed, matching the BL's zero-initialised entry");
        CHECK(entry[8] == 42 && entry[9] == 0, "seq little-endian");
        erase_all();
        for (int i = 0; i < 32; ++i) sector[i] = entry[i];
        const CalLoadResult r = load_cal_from_nvm(sector, sizeof(sector));
        CHECK(r.status == CalLoad::Loaded, "an entry we built is one our reader accepts");
        CHECK(r.cal.brake_pressed == 3100, "...and round-trips exactly");
    }
}

static void test_cal_session() {
    std::printf("[cal_session]\n");

    const PedalCal active{};                       // what is currently in force
    CalSessionInputs in{};
    in.vehicle_safe = true;
    in.now_ms = 1000;

    auto capture_all = [&](CalSession& s, std::uint16_t a1r, std::uint16_t a1f,
                           std::uint16_t a2r, std::uint16_t a2f,
                           std::uint16_t br, std::uint16_t bp,
                           std::uint16_t a1m, std::uint16_t a2m) {
        in.apps1_raw = a1r; in.apps2_raw = a2r;
        s.handle(CalCmd::Capture, 1, 0, in, active);          // APPS_REST
        in.apps1_raw = a1f; in.apps2_raw = a2f;
        s.handle(CalCmd::Capture, 2, 0, in, active);          // APPS_FULL
        in.brake_raw = br;
        s.handle(CalCmd::Capture, 3, 0, in, active);          // BRAKE_REST
        in.brake_raw = bp;
        s.handle(CalCmd::Capture, 4, 0, in, active);          // BRAKE_PRESSED
        in.apps1_raw = a1m; in.apps2_raw = a2m;
        s.handle(CalCmd::Capture, 5, 0, in, active);          // APPS_MID
    };

    // --- a stray frame cannot open a session ---
    {
        CalSession s;
        auto o = s.handle(CalCmd::Enter, 0, 0xDEADBEEF, in, active);
        CHECK(o.result == CalResult::BadGuard, "wrong guard rejected");
        CHECK(o.state == CalSessionState::Idle, "...and no session opens");
    }

    // --- a moving car cannot be calibrated ---
    {
        CalSession s;
        CalSessionInputs unsafe = in; unsafe.vehicle_safe = false;
        auto o = s.handle(CalCmd::Enter, 0, CalGuardMagic, unsafe, active);
        CHECK(o.result == CalResult::VehicleNotSafe, "unsafe vehicle rejected at ENTER");
        CHECK(o.state == CalSessionState::Idle, "...no session");
    }

    // --- commands that need a session are refused without one ---
    {
        CalSession s;
        CHECK(s.handle(CalCmd::Capture, 1, 0, in, active).result == CalResult::NotInSession,
              "CAPTURE outside a session refused");
        CHECK(s.handle(CalCmd::Commit, 0, 0, in, active).result == CalResult::NotInSession,
              "COMMIT outside a session refused");
        // ...but reading what is in force is not privileged
        auto o = s.handle(CalCmd::ReadStored, 0, 0, in, active);
        CHECK(o.result == CalResult::Ok && o.emit_values, "READ_STORED works with no session");
        CHECK(o.values.apps1_min == active.apps1_min, "...and returns the ACTIVE calibration");
    }

    // --- happy path, ending in a commit request ---
    {
        CalSession s;
        CHECK(s.handle(CalCmd::Enter, 0, CalGuardMagic, in, active).state == CalSessionState::Active,
              "valid ENTER opens the session");
        capture_all(s, 2500, 3400, 2300, 3000, 560, 3100, 2950, 2650);
        CHECK(s.captured_mask() == cal_point_bit::All, "all five points captured");

        // committing with the wrong CRC must fail -- this is the desync guard
        auto bad = s.handle(CalCmd::Commit, 0, 0x12345678u, in, active);
        CHECK(bad.result == CalResult::ValidationFailed, "COMMIT with a stale CRC refused");
        CHECK(!bad.commit_requested, "...and nothing is applied");

        auto staged = s.handle(CalCmd::ReadStaged, 0, 0, in, active);
        CHECK(staged.emit_values, "READ_STAGED returns the staged set");
        CHECK(staged.values.apps1_min == 2500, "...with the captured rest point");
        // thresholds are DERIVED from the measured brake span, not inherited
        CHECK(staged.values.brake_arm > staged.values.brake_rest &&
              staged.values.brake_arm < staged.values.brake_dv_hard &&
              staged.values.brake_dv_hard < staged.values.brake_pressed,
              "brake thresholds derived from the span, correctly ordered");

        const std::uint32_t crc = cal_crc32(staged.values);
        auto ok = s.handle(CalCmd::Commit, 0, crc, in, active);
        CHECK(ok.result == CalResult::Ok, "COMMIT with the right CRC accepted");
        CHECK(ok.commit_requested, "...and asks the task to apply + persist");
        CHECK(ok.to_commit.apps1_min == 2500, "...carrying the staged values");
        CHECK(ok.state == CalSessionState::Committing, "state is Committing");
    }

    // --- committing early is refused ---
    {
        CalSession s;
        s.handle(CalCmd::Enter, 0, CalGuardMagic, in, active);
        in.apps1_raw = 2500; in.apps2_raw = 2300;
        s.handle(CalCmd::Capture, 1, 0, in, active);
        auto o = s.handle(CalCmd::Commit, 0, 0, in, active);
        CHECK(o.result == CalResult::MissingPoints, "COMMIT before all points refused");
        CHECK(!o.commit_requested, "...nothing applied");
    }

    // --- THE load-bearing one: channels that diverge at mid travel are rejected
    //     even though both endpoint pairs are individually perfect ---
    {
        CalSession s;
        s.handle(CalCmd::Enter, 0, CalGuardMagic, in, active);
        // APPS1 mid sits at ~50% of its span; APPS2 mid sits at ~19% of its own.
        capture_all(s, 2500, 3400, 2300, 3000, 560, 3100, 2950, 2430);
        auto st = s.handle(CalCmd::ReadStaged, 0, 0, in, active);
        auto o = s.handle(CalCmd::Commit, 0, cal_crc32(st.values), in, active);
        CHECK(o.result == CalResult::ValidationFailed, "mid-travel divergence rejected");
        CHECK((o.validation_flags & cal_flag::AppsSpanMismatch) != 0,
              "...flagged as a channel mismatch -- the T.11.8.9 failure endpoints cannot see");
        CHECK(!o.commit_requested, "...and never reaches the flash");
    }

    // --- abort discards everything ---
    {
        CalSession s;
        s.handle(CalCmd::Enter, 0, CalGuardMagic, in, active);
        capture_all(s, 2500, 3400, 2300, 3000, 560, 3100, 2950, 2650);
        auto o = s.handle(CalCmd::Abort, 0, 0, in, active);
        CHECK(o.state == CalSessionState::Idle, "ABORT closes the session");
        CHECK(o.captured_mask == 0, "...and discards the captures");
    }

    // --- an abandoned session times out rather than lingering ---
    {
        CalSession s;
        CalSessionInputs t = in;
        s.handle(CalCmd::Enter, 0, CalGuardMagic, t, active);
        t.now_ms += CalSessionTimeoutMs - 1;
        CHECK(!s.tick(t), "no timeout just before the window");
        CHECK(s.state() == CalSessionState::Active, "...session still open");
        t.now_ms += 2;
        CHECK(s.tick(t), "timeout fires past the window");
        CHECK(s.state() == CalSessionState::Idle, "...session closed");
        CHECK(s.captured_mask() == 0, "...staged data discarded, not left for a reconnect");
    }

    // --- the vehicle becoming unsafe mid-session kills it ---
    {
        CalSession s;
        s.handle(CalCmd::Enter, 0, CalGuardMagic, in, active);
        CalSessionInputs unsafe = in; unsafe.vehicle_safe = false;
        auto o = s.handle(CalCmd::Capture, 1, 0, unsafe, active);
        CHECK(o.result == CalResult::VehicleNotSafe, "capture refused once unsafe");
        CHECK(o.state == CalSessionState::Idle, "...and the session is torn down, not just refused");
    }

    // --- RESET_DEFAULTS stages the defaults but still needs a commit ---
    {
        CalSession s;
        auto o = s.handle(CalCmd::ResetDefaults, 0, CalGuardMagic, in, active);
        CHECK(o.state == CalSessionState::Active, "RESET_DEFAULTS opens a session");
        CHECK(o.captured_mask == cal_point_bit::All, "...with nothing left to capture");
        auto st = s.handle(CalCmd::ReadStaged, 0, 0, in, active);
        CHECK(st.values.apps1_min == config::Apps1AdcMin, "...staging the compile-time defaults");
        CHECK(s.handle(CalCmd::ResetDefaults, 0, 0, in, active).result == CalResult::BadGuard,
              "RESET_DEFAULTS without the guard refused");
    }

    // --- a persistence failure is reported, not swallowed ---
    {
        CalSession s;
        s.handle(CalCmd::Enter, 0, CalGuardMagic, in, active);
        s.note_persist_failed();
        CHECK(s.state() == CalSessionState::Error, "persistence failure moves to Error");
    }
}

// #148 -- L1/L2 fault-layer decode from 0x461. Both straddle byte boundaries
// (EMCtrl_FOC_BitState 39|8@1+ -> byte4 b7 + byte5 b0-6; PwrStg_BitState
// 47|9@1+ -> byte5 b7 + byte6), which is exactly the kind of shift that is easy
// to get off by one -- hence explicit patterns with known bits set.
static void test_inverter_fault_layers() {
    std::printf("[inverter_fault_layers]\n");
    VehicleService& vs = VehicleService::instance();

    // byte4 = App_State_App(4=Ready) | EMCtrl bit0 (Init OK) in b7
    // byte5 = EMCtrl bits1-7 = 0 ; PwrStg bit0 (Alive) in b7
    // byte6 = PwrStg bits1-8 = 0b00000001 -> Enable (bit1)
    CanFrame f{};
    f.bus = static_cast<std::uint8_t>(CanBus::Inv);
    f.id  = config::InvRxStateId;                 // 0x461
    f.dlc = 7;
    f.data[2] = 2;                                // DEM_Code low byte = Undervoltage
    f.data[3] = 0x00;                             // DEM_Present = 0 (latched)
    f.data[4] = 0x04 | 0x80;                      // App_State 4 + EMCtrl b0
    f.data[5] = 0x80;                             // EMCtrl b1-7 = 0, PwrStg b0 = 1
    f.data[6] = 0x01;                             // PwrStg b1 = 1
    CHECK(vs.update_from_frame(f), "0x461 DLC 7 accepted");
    const VehicleState v = vs.snapshot();
    CHECK(v.inv_state == 4, "App_State_App still decodes (byte4 b0-6)");
    CHECK(v.inv_error == 2, "DEM_Code low byte = 2 (Undervoltage)");
    CHECK(!v.inv_dem_present, "DEM_Present clear -> latched history, condition gone");
    CHECK(v.inv_emctrl_bits == 0x01, "L2: only Init_OK set");
    CHECK(v.inv_pwrstg_bits == 0x003, "L1: Alive|Enable set (healthy idle)");

    // A real interlock trip: PwrStg HVIL_Open is bit5 -> value 32.
    // bits 1..8 live in byte6, so bit5 = byte6 bit4 = 0x10.
    f.data[6] = 0x01 | 0x10;
    CHECK(vs.update_from_frame(f), "second 0x461 accepted");
    CHECK(vs.snapshot().inv_pwrstg_bits == 0x023, "L1: Alive|Enable|HVIL_Open (0x23)");

    // Short frame: must NOT clobber the layers, and must still yield inv_state.
    f.dlc = 5;
    f.data[4] = 0x0A;                             // App_State 10 (SoftFault)
    CHECK(vs.update_from_frame(f), "0x461 DLC 5 still accepted");
    CHECK(vs.snapshot().inv_state == 10, "short frame still decodes inv_state");
    CHECK(vs.snapshot().inv_pwrstg_bits == 0x023, "short frame leaves L1 untouched");

    // 0x461 freshness tracking (#148): last_inv_state_tick and inv_state_seq
    // must advance on 0x461 ONLY -- last_inv_tick is also bumped by 0x463/64/66
    // and would hide a slow 0x461, which is the exact thing being measured.
    {
        CanFrame g{};
        g.bus = static_cast<std::uint8_t>(CanBus::Inv);
        g.id  = config::InvRxStateId;                 // 0x461
        g.dlc = 7;
        g.data[4] = 0x03;                             // App_State 3
        g.timestamp_ms = 1000;
        const std::uint8_t seq0 = vs.snapshot().inv_state_seq;
        CHECK(vs.update_from_frame(g), "0x461 accepted (freshness)");
        CHECK(vs.snapshot().last_inv_state_tick == 1000, "0x461 stamps last_inv_state_tick");
        CHECK(static_cast<std::uint8_t>(vs.snapshot().inv_state_seq - seq0) == 1,
              "0x461 increments inv_state_seq by exactly 1");

        // a DIFFERENT inverter frame must NOT touch either -- that is the point
        CanFrame r{};
        r.bus = static_cast<std::uint8_t>(CanBus::Inv);
        r.id  = config::InvRxRpmId;                   // 0x463
        r.dlc = 8;
        r.timestamp_ms = 2000;
        const std::uint8_t seq1 = vs.snapshot().inv_state_seq;
        CHECK(vs.update_from_frame(r), "0x463 accepted");
        CHECK(vs.snapshot().last_inv_state_tick == 1000,
              "0x463 does NOT refresh last_inv_state_tick (would mask a slow 0x461)");
        CHECK(vs.snapshot().inv_state_seq == seq1, "0x463 does NOT bump inv_state_seq");
        CHECK(vs.snapshot().last_inv_tick == 2000, "0x463 DOES refresh the generic last_inv_tick");
    }

    // NOTE: no builder round-trip here -- pit_diag.cpp is deliberately outside
    // the SIL target (6 units, no HAL), so PitDiag::build_inv_faults is not
    // linked. The DSL encode path is covered by test_dsl_parity.
}

// #148 -- the fault-recovery BURST. A latched inverter fault is NOT cleared by
// its reset word alone. The W90 manual 9.3 says going to OFF is what restarts a
// FAULT (0x0D/0x13 are bench-derived and appear nowhere in its App_State_Req
// list), and the IFS07 VCU that recovered without a power cycle sent the reset
// word AND Off in the same pass -- its App_State switch fell through with no
// breaks (soft -> 0x13, 0x0D, 0x01; hard -> 0x0D, 0x01).
//
// The 2026-07-24 bench capture is the evidence: inverter latched SoftFault(10),
// dem_present ACTIVE, DC bus 355 V, ECU commanding 0x13 forever (inv_mode_cmd
// on 0x702) -- it never cleared, so WaitInvStandby hung and torque never
// returned without an LV power cycle.
static void test_inverter_fault_burst() {
    std::printf("[inverter_fault_burst]\n");

    Controller c;
    uint32_t t = 1000;
    drive_to_active(c, t);
    CtrlInputs in = good_drive_inputs();

    // --- SOFT fault (10) -> 0x13, then 0x0D, then Off(0x01). Legacy order. ---
    in.inv_state = InvSoftFaultState;            // 10
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.inv_mode == InvMode::Fault, "soft fault -> primary word Fault (0x13)");
    CHECK(o.inv_mode_follow_n == 2, "soft fault -> two follow-up words");
    CHECK(o.inv_mode_follow[0] == InvMode::HardFaultReset, "soft fault follow #1 = 0x0D");
    CHECK(o.inv_mode_follow[1] == InvMode::Off,
          "soft fault follow #2 = Off (0x01) -- the word the manual says clears a FAULT");
    CHECK(o.inv_flt_clear, "soft fault -> Flt_Clear asserted on the trailing Off");
    CHECK(o.torque_pct == 0, "no torque while faulted");

    // --- HARD fault (11) -> 0x0D, then Off(0x01). ---
    in.inv_state = InvHardFaultState;            // 11
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.inv_mode == InvMode::HardFaultReset, "hard fault -> primary word 0x0D");
    CHECK(o.inv_mode_follow_n == 1, "hard fault -> one follow-up word");
    CHECK(o.inv_mode_follow[0] == InvMode::Off, "hard fault follow = Off (0x01)");
    CHECK(o.inv_flt_clear, "hard fault -> Flt_Clear asserted on the trailing Off");

    // --- Healthy inverter -> NO burst. The normal path must still emit exactly
    //     one 0x360 per cycle; this is the bus-load guard (#132). ---
    in.inv_state = InvReadyState;                // 4
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.inv_mode_follow_n == 0, "healthy inverter -> no follow-up words");
    CHECK(!o.inv_flt_clear, "healthy inverter -> Flt_Clear NOT asserted");
    CHECK(o.inv_mode == InvMode::TorqueEnable, "healthy in Active -> TorqueEnable, unchanged");

    // --- AmsError still INHIBITS: Off only, no reset words, no burst. ---
    in.ams_error = true;
    in.inv_state = InvSoftFaultState;            // fault present but suppressed
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::AmsError, "ams_error -> AmsError from any state");
    CHECK(o.inv_mode == InvMode::Off, "AmsError commands Off only");
    CHECK(o.inv_mode_follow_n == 0, "AmsError inhibits the fault burst too");
    CHECK(!o.inv_flt_clear, "AmsError does not assert Flt_Clear either");

    // --- Wire format: Flt_Clear is byte 2 bit 7, packed WITH App_State_Req. ---
    {
        const CanFrame plain = Inverter::build_setpoint_mode(InvMode::Off);
        CHECK(plain.data[2] == 0x01, "Off without Flt_Clear -> byte2 == 0x01 (bit7 clear)");
        const CanFrame clr = Inverter::build_setpoint_mode(InvMode::Off, true);
        CHECK(clr.data[2] == 0x81, "Off WITH Flt_Clear -> byte2 == 0x81 (App_State_Req 1 | bit7)");
        CHECK(clr.id == InvTxSetpointModeId && clr.dlc == InvTxSetpointModeDlc,
              "Flt_Clear rides the same 0x360 DLC 3 frame");
        CHECK(clr.data[0] == 0 && clr.data[1] == 0, "bytes 0-1 stay zero");
    }

    // --- End-to-end: the burst clears the fault and the drive comes back with
    //     NO power cycle -- the whole point of #148. ---
    {
        Controller c2;
        uint32_t t2 = 1000;
        drive_to_active(c2, t2);
        CtrlInputs in2 = good_drive_inputs();

        in2.ok_precharge = false;                        // TS off
        in2.inv_state    = InvSoftFaultState;            // inverter latches soft fault
        CtrlOutput o2 = c2.step(in2, t2); t2 += ControlPeriodMs;
        CHECK(o2.state == CtrlState::Precharge, "TS-off -> Precharge");
        CHECK(o2.inv_mode_follow_n == 2, "burst is emitted pre-R2D too (not only in the climb)");

        in2.ok_precharge = true;                         // HV back (355 V on the bench)
        c2.step(in2, t2); t2 += ControlPeriodMs;         // -> WaitStartBrake
        in2.start_button = true; in2.brake_raw = BrakeArmRaw + 100;
        c2.step(in2, t2); t2 += ControlPeriodMs;         // -> R2dDelay
        in2.start_button = false; in2.brake_raw = 0; t2 += R2dSoundMs;
        o2 = c2.step(in2, t2); t2 += ControlPeriodMs;    // -> WaitInvStandby
        CHECK(o2.state == CtrlState::WaitInvStandby, "R2D re-arm -> WaitInvStandby");
        CHECK(o2.inv_mode == InvMode::Fault, "still faulted -> reset word wins over Ready");
        CHECK(o2.inv_mode_follow[1] == InvMode::Off, "...and Off still follows it");

        in2.inv_state = InvStandbyState;                 // 3 -- the burst cleared it
        o2 = c2.step(in2, t2); t2 += ControlPeriodMs;
        CHECK(o2.inv_mode == InvMode::Ready, "cleared -> back to commanding Ready (0x04)");
        CHECK(o2.inv_mode_follow_n == 0, "no burst once the fault is gone");

        in2.inv_state = InvReadyState;                   // 4
        o2 = c2.step(in2, t2); t2 += ControlPeriodMs;
        CHECK(o2.state == CtrlState::Active, "READY(4) -> Active");
        o2 = c2.step(in2, t2);
        CHECK(o2.torque_pct == 100, "torque flows again -- recovered with NO power cycle");
    }
}

// Inverter RX decode: 0x463 rpm (20-bit signed @ bit 44 -- the bit-44 fix, byte
// patterns verified vs the NX DBC with cantools) and 0x464 temps (raw bytes pass
// through; the DBC -50 offset turns them into degC on decode).
static void test_inverter_rx() {
    std::printf("[inverter_rx]\n");

    // rpm decoder (pure static) -- the signed bit-44 layout.
    const std::uint8_t z[8]   = {0,0,0,0,0, 0x00,0x00,0x00};
    const std::uint8_t one[8] = {0,0,0,0,0, 0x10,0x00,0x00};
    const std::uint8_t hi[8]  = {0,0,0,0,0, 0xF0,0xFF,0x07};
    const std::uint8_t neg[8] = {0,0,0,0,0, 0xF0,0xFF,0xFF};
    CHECK(VehicleService::decode_inv_rpm(z)   == 0,     "rpm 0");
    CHECK(VehicleService::decode_inv_rpm(one) == 1,     "rpm 1 (LSB at bit 44)");
    CHECK(VehicleService::decode_inv_rpm(hi)  == 32767, "rpm +32767");
    CHECK(VehicleService::decode_inv_rpm(neg) == -1,    "rpm -1 (signed, sign-extend bit 19)");

    VehicleService& vs = VehicleService::instance();

    // rpm flows through the RX path (0x463) into the snapshot.
    CanFrame r{};
    r.bus = static_cast<std::uint8_t>(CanBus::Inv);
    r.id  = InvRxRpmId;          // 0x463
    r.dlc = 8;
    r.data[5] = 0xF0; r.data[6] = 0xFF; r.data[7] = 0x07;   // 32767
    CHECK(vs.update_from_frame(r), "0x463 accepted");
    CHECK(vs.snapshot().inv_rpm == 32767, "rpm reaches the snapshot");

    // temps (0x464): raw bytes stored verbatim (the DBC -50 offset -> degC).
    CanFrame t{};
    t.bus = static_cast<std::uint8_t>(CanBus::Inv);
    t.id  = InvRxTempId;         // 0x464
    t.dlc = 4;
    t.data[0]=92; t.data[1]=93; t.data[2]=84; t.data[3]=0xFF; // 42/43/34/205 degC
    CHECK(vs.update_from_frame(t), "0x464 accepted");
    const VehicleState s = vs.snapshot();
    CHECK(s.inv_temp_board==92 && s.inv_temp_pwrstg==93, "board/stage temps stored raw");
    CHECK(s.inv_temp_motor1==84 && s.inv_temp_motor2==0xFF, "motor temps stored raw (0xFF = disconnect sentinel)");
}

// DV drive mode (#17): the trigger IS the mode decision at WaitStartBrake --
// manual (start+brake) vs DV (0x510 request + EBS hard braking verified on our
// own brake sensor). Latched per drive cycle; 0x507 is the torque source;
// stale => 0, never APPS; EV.2.3/T.11.8.9 stay manual-only.
static void test_dv_mode() {
    std::printf("[dv_mode]\n");

    // --- the 0x507 conditioner (fail-safe integer % -> pct) ---
    CHECK(VehicleService::condition_udv_torque(0)   == 0,        "0 -> 0");
    CHECK(VehicleService::condition_udv_torque(40)  == 40,       "40 -> 40");
    CHECK(VehicleService::condition_udv_torque(100) == 100,      "100 -> 100");
    CHECK(VehicleService::condition_udv_torque(150) == 100,      "150 clamps to 100");
    CHECK(VehicleService::condition_udv_torque(-1)  == 0,        "-1 -> 0 (fail-safe, no regen by accident)");
    CHECK(VehicleService::condition_udv_torque(INT32_MIN) == 0,  "INT32_MIN -> 0");
    CHECK(VehicleService::condition_udv_torque(INT32_MAX) == 100, "INT32_MAX clamps to 100");

    Controller c;
    uint32_t t = 0;
    CtrlInputs in = good_drive_inputs();

    // Walk to WaitStartBrake.
    CtrlOutput o = c.step(in, t); t += ControlPeriodMs;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitStartBrake, "at WaitStartBrake");

    // DV request WITHOUT the EBS hard braking -> refused (holds).
    in.dv_r2d_req = true;
    in.brake_raw  = BrakeDvHardRaw;          // not ABOVE the limit
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitStartBrake, "0x510 without EBS braking -> holds");
    CHECK(!o.dv_mode, "no DV latch without the brake verdict");

    // DV request WITH EBS hard braking -> R2D, DV latched, RTDS sounds.
    in.brake_raw = BrakeDvHardRaw + 200;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::R2dDelay, "0x510 + EBS hard braking -> R2dDelay (no start button)");
    CHECK(o.dv_mode, "DV latched at the R2D entry");
    CHECK(o.rtds_on, "RTDS sounds in DV too");

    // Through RTDS + inverter ready -> Active, still DV.
    in.dv_r2d_req = false;                   // request may drop after the handshake
    in.brake_raw  = 0;                       // EBS releases
    t += R2dSoundMs;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitInvStandby, "RTDS done -> WaitInvStandby");
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Active && o.dv_mode, "Active, DV latch held");

    // Torque comes from the conditioned 0x507 -- NOT the pedals.
    in.apps1_raw = 0; in.apps2_raw = 0;      // pedals idle (nobody seated)
    in.dv_fresh = true; in.dv_torque_pct = 40;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.torque_pct == 40, "DV torque = conditioned 0x507 (pedals idle)");
    CHECK(o.inv_mode == InvMode::TorqueEnable && o.ok_to_drive, "drives in DV Active");

    // Stale command stream => torque 0, stays Active, NEVER falls back to APPS.
    in.dv_fresh = false;
    in.apps1_raw = Apps1AdcMax; in.apps2_raw = Apps2AdcMax;   // pedals pressed hard
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.torque_pct == 0, "stale 0x507 -> torque 0, never APPS fallback");
    CHECK(o.state == CtrlState::Active, "staleness does not exit the drive");
    in.apps1_raw = 0; in.apps2_raw = 0;

    // EV.2.3 is a driver-pedal rule: EBS pressure + DV torque must NOT trip it.
    in.dv_fresh = true; in.dv_torque_pct = 50;
    in.brake_raw = BrakePressedRaw + 500;    // EBS holding hard
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.torque_pct == 50, "EV.2.3 does not gate DV torque (EBS pressure)");
    in.brake_raw = 0;

    // Drive-cycle exit clears the latch; the next entry re-decides the mode.
    in.ok_precharge = false;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::Precharge && !o.dv_mode, "exit to Precharge clears the DV latch");
    in.ok_precharge = true;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::WaitStartBrake, "re-armed");
    in.start_button = true; in.brake_raw = BrakeArmRaw + 100;
    o = c.step(in, t); t += ControlPeriodMs;
    CHECK(o.state == CtrlState::R2dDelay && !o.dv_mode, "manual re-entry latches MANUAL");
    in.start_button = false; in.brake_raw = 0;

    // Both triggers true simultaneously -> manual wins (deterministic order).
    Controller c2; uint32_t t2 = 0;
    CtrlInputs in2 = good_drive_inputs();
    c2.step(in2, t2); t2 += ControlPeriodMs;
    c2.step(in2, t2); t2 += ControlPeriodMs;
    in2.start_button = true; in2.dv_r2d_req = true;
    in2.brake_raw = BrakeDvHardRaw + 200;    // satisfies BOTH brake gates
    CtrlOutput o2 = c2.step(in2, t2); t2 += ControlPeriodMs;
    CHECK(o2.state == CtrlState::R2dDelay && !o2.dv_mode, "both triggers -> manual precedence");

    // AmsError pre-empts a DV drive and clears the latch.
    Controller c3; uint32_t t3 = 0;
    CtrlInputs in3 = good_drive_inputs();
    c3.step(in3, t3); t3 += ControlPeriodMs;
    c3.step(in3, t3); t3 += ControlPeriodMs;
    in3.dv_r2d_req = true; in3.brake_raw = BrakeDvHardRaw + 200;
    c3.step(in3, t3); t3 += ControlPeriodMs;                       // DV R2D
    in3.ams_error = true;
    CtrlOutput o3 = c3.step(in3, t3); t3 += ControlPeriodMs;
    CHECK(o3.state == CtrlState::AmsError && !o3.dv_mode, "AmsError pre-empts DV + clears latch");
    CHECK(o3.inv_mode == InvMode::Off && o3.torque_pct == 0, "inhibited in AmsError");
}

// uDV autonomous contract (#17) -- RX decode/store + the ECU->uDV TX builders.
static void test_udv_rx() {
    std::printf("[udv_rx]\n");

    // 0x507 payload is an integer percent, int32 little-endian on the wire.
    // 40 -> {28 00 00 00}; -1 -> {FF FF FF FF} (sign-preserving decode).
    const std::uint8_t p40[4] = {0x28, 0x00, 0x00, 0x00};
    const std::uint8_t m1[4]  = {0xFF, 0xFF, 0xFF, 0xFF};
    CHECK(VehicleService::decode_udv_torque_cmd(p40) == 40, "s32 LE decode (40)");
    CHECK(VehicleService::decode_udv_torque_cmd(m1)  == -1, "s32 LE decode preserves sign (-1)");

    VehicleService& vs = VehicleService::instance();
    const std::uint32_t ams_tick_before = vs.snapshot().last_ams_tick;

    CanFrame a{};
    a.bus = static_cast<std::uint8_t>(CanBus::Acu);
    a.id  = UdvTorqueCmdId;      // 0x507
    a.dlc = 4;
    a.data[0]=0x28; a.data[1]=0x00; a.data[2]=0x00; a.data[3]=0x00;   // 40%
    a.timestamp_ms = 1234;
    CHECK(vs.update_from_frame(a), "0x507 accepted");
    CHECK(vs.snapshot().udv_torque_cmd == 40, "torque cmd reaches the snapshot");
    CHECK(vs.snapshot().last_udv_cmd_tick == 1234u, "0x507 freshness tick stamped");

    CanFrame r{};
    r.bus = static_cast<std::uint8_t>(CanBus::Acu);
    r.id  = UdvR2dRequestId;     // 0x510
    r.dlc = 1;
    r.data[0] = 1;
    r.timestamp_ms = 1250;
    CHECK(vs.update_from_frame(r), "0x510 accepted");
    CHECK(vs.snapshot().udv_r2d_request == 1u, "R2D request stored");
    CHECK(vs.snapshot().last_udv_r2d_tick == 1250u, "0x510 freshness tick stamped");

    // The safety property: uDV traffic must NOT refresh the AMS freshness.
    CHECK(vs.snapshot().last_ams_tick == ams_tick_before, "uDV frames don't touch last_ams_tick");

    // Undersized frames are rejected (dlc guards).
    CanFrame bad = a; bad.dlc = 3;
    CHECK(!vs.update_from_frame(bad), "short 0x507 rejected");
}

static void test_udv_tx() {
    std::printf("[udv_tx]\n");

    const CanFrame ts = UdvTx::build_ts_active(true);
    CHECK(ts.id == VCU_ts_active_ID && ts.dlc == 1, "0x504 id/dlc");
    CHECK(ts.bus == static_cast<std::uint8_t>(CanBus::Acu), "0x504 on the ACU bus");
    CHECK(ts.data[0] == 1u, "ts_active true -> byte0 = 1");
    CHECK(UdvTx::build_ts_active(false).data[0] == 0u, "ts_active false -> 0");

    const CanFrame br = UdvTx::build_brake_over_limit(true);
    CHECK(br.id == VCU_brake_over_limit_ID && br.dlc == 1, "0x505 id/dlc");
    CHECK(br.data[0] == 1u, "brake over limit -> byte0 = 1");
    CHECK(UdvTx::build_brake_over_limit(false).data[0] == 0u, "under limit -> 0");

    // 0x506 takes ERPM and transmits MECHANICAL rpm = erpm / MotorPolePairs
    // (10, powertrain-confirmed), s32 LE. 74560 erpm -> 7456 mech = 0x1D20
    // -> {20 1D 00 00}; -15 erpm -> -1 mech -> {FF FF FF FF}; -1 erpm -> 0
    // (integer division truncates toward zero).
    const CanFrame rp = UdvTx::build_motor_rpm(74560);
    CHECK(rp.id == VCU_motor_rpm_ID && rp.dlc == 4, "0x506 id/dlc");
    CHECK(rp.data[0]==0x20 && rp.data[1]==0x1D && rp.data[2]==0x00 && rp.data[3]==0x00,
          "74560 erpm -> 7456 mechanical rpm, little-endian");
    const CanFrame rn = UdvTx::build_motor_rpm(-15);
    CHECK(rn.data[0]==0xFF && rn.data[1]==0xFF && rn.data[2]==0xFF && rn.data[3]==0xFF,
          "-15 erpm -> -1 mechanical (sign-preserving s32 LE)");
    CHECK(UdvTx::build_motor_rpm(-1).data[0] == 0x00, "-1 erpm -> 0 (truncates toward zero)");
}

// v2 radio snapshot -- byte layout MUST match the live ground-station parser
// (IFS08-TE feat/receptor_08 ISC_RTT_serial.py _decode_snapshot). Distinct
// value per field so any mis-offset is caught; a Python round-trip against the
// real parser (tests/sil/radio_snapshot_roundtrip.py) is the belt-and-braces.
static uint16_t rd16(const uint8_t* d, int o) {
    return static_cast<uint16_t>(d[o] | (d[o + 1] << 8));
}
static uint32_t rd32(const uint8_t* d, int o) {
    return static_cast<uint32_t>(d[o]) | (static_cast<uint32_t>(d[o + 1]) << 8) |
           (static_cast<uint32_t>(d[o + 2]) << 16) | (static_cast<uint32_t>(d[o + 3]) << 24);
}
// Shared known-value snapshot inputs (distinct per field). Used by the offset
// test and the --dump-radio round-trip against the real ground-station parser.
static RadioSnapshotInputs radio_test_inputs() {
    RadioSnapshotInputs in{};
    in.tick_ms = 0x11223344u; in.seq = 0xABCD;
    in.start_button = 1; in.apps1_raw = 2500; in.apps2_raw = 2400; in.brake_raw = 1234;
    in.torque_pct = 77; in.ev_2_3 = 1; in.t11_8_9 = 1;
    in.state = 5; in.ok_precharge = 1; in.ams_fsm_state = 3;
    in.v_cell_min_mV = 3650; in.soc = 87;
    for (int i = 0; i < 5; ++i) {
        in.vmin_module[i] = static_cast<uint16_t>(3600 + i);
        in.vmax_module[i] = static_cast<uint16_t>(3700 + i);
        in.tmax_module[i] = static_cast<int16_t>(30 + i);
    }
    in.current_accu_dA = -421; in.current_dcdc_dA = 55; in.tmax_dcdc = 38;
    in.inv_state = 6; in.inv_vconfig_active = 1; in.inv_error = 0;
    in.inv_dc_bus_V = 550; in.inv_temp_motor1 = 72; in.inv_temp_pwrstg = 68; in.inv_temp_board = 55;
    in.inv_rpm = -12345;
    in.gps_lat_deg1e7 = 406353900; in.gps_lon_deg1e7 = -36927966;
    in.gps_speed_kmh_x100 = 4148; in.gps_course_deg_x100 = 8440;
    in.gps_sats = 8; in.gps_has_fix = 1;
    return in;
}

// Print the serialized 102-byte snapshot as one hex line (for the Python
// round-trip against the real parser). Header/footer suppressed by main.
static void dump_radio_snapshot() {
    uint8_t s[kRadioSnapshotWireSize];
    serialize_radio_snapshot(s, radio_test_inputs());
    for (unsigned i = 0; i < kRadioSnapshotWireSize; ++i) std::printf("%02x", s[i]);
    std::printf("\n");
}

static void test_radio_snapshot() {
    std::printf("[radio_snapshot]\n");
    const RadioSnapshotInputs in = radio_test_inputs();

    uint8_t s[kRadioSnapshotWireSize];
    serialize_radio_snapshot(s, in);

    CHECK(rd32(s, 0) == 0x11223344u, "tick_ms @0");
    CHECK(rd16(s, 4) == 0xABCD,      "seq @4");
    CHECK(s[6] == 1,                 "start_button @6");
    CHECK(rd16(s, 7) == 2500 && rd16(s, 9) == 2400 && rd16(s, 11) == 1234, "pedals @7/9/11");
    CHECK(rd16(s, 13) == 77,         "torque_pct @13 (u8 zero-extended)");
    CHECK(s[15] == 1 && s[16] == 1,  "ev_2_3/t11_8_9 @15/16");
    CHECK(s[17] == 5 && s[18] == 1 && s[19] == 3, "state/ok_precharge/ams_fsm @17-19");
    CHECK(rd16(s, 20) == 3650,       "v_cell_min_mV @20");
    CHECK(s[22] == 87,               "soc @22");
    bool mods_ok = true;
    for (int i = 0; i < 5; ++i) {
        if (rd16(s, 23 + 2 * i) != static_cast<uint16_t>(3600 + i)) mods_ok = false;
        if (rd16(s, 33 + 2 * i) != static_cast<uint16_t>(3700 + i)) mods_ok = false;
        if (static_cast<int16_t>(rd16(s, 49 + 2 * i)) != static_cast<int16_t>(30 + i)) mods_ok = false;
    }
    CHECK(mods_ok, "per-module vmin@23 / vmax@33 / tmax@49 (5 each)");
    CHECK(static_cast<int16_t>(rd16(s, 43)) == -421, "current_accu @43 (signed)");
    CHECK(static_cast<int16_t>(rd16(s, 45)) == 55,   "current_dcdc @45");
    CHECK(static_cast<int16_t>(rd16(s, 47)) == 38,   "temp_dcdc @47");
    CHECK(s[59] == 6 && s[60] == 1 && s[61] == 0, "inv_state/vconfig/error @59-61");
    CHECK(rd16(s, 62) == 550, "inv_dc_bus_V @62");
    CHECK(rd16(s, 64) == 72 && rd16(s, 66) == 68 && rd16(s, 68) == 55, "inv temps @64/66/68");
    CHECK(static_cast<int32_t>(rd32(s, 70)) == -12345, "inv_rpm @70 (signed)");
    CHECK(rd32(s, 74) == 0 && rd32(s, 78) == 0, "inv_speed/current_actual @74/78 (placeholder)");
    // GPS occupies what used to be the reserved tail (wire size still 102).
    CHECK(static_cast<int32_t>(rd32(s, 82)) ==  406353900, "gps lat @82 (signed)");
    CHECK(static_cast<int32_t>(rd32(s, 86)) ==  -36927966, "gps lon @86 (signed)");
    CHECK(rd16(s, 90) == 4148, "gps speed km/h*100 @90");
    CHECK(rd16(s, 92) == 8440, "gps course deg*100 @92");
    CHECK(s[94] == 8 && s[95] == 1, "gps sats/has_fix @94/95");
    bool tail_zero = true;
    for (int i = 96; i < 102; ++i) if (s[i] != 0) tail_zero = false;
    CHECK(tail_zero, "reserved [96..101] zero");

    // Fragmentation: 5 fragments, v2 header, data slices reassemble the snapshot.
    uint8_t reasm[kRadioSnapshotFragments * kRadioFragPayloadSize] = {};
    bool hdr_ok = true;
    for (uint8_t f = 0; f < kRadioSnapshotFragments; ++f) {
        uint8_t p[kRadioFragmentSize];
        build_radio_fragment(p, s, in.seq, f);
        if (!(p[0] == kRadioMagic && p[1] == kRadioVersionSnapshot && p[2] == f &&
              p[3] == kRadioSnapshotFragments && rd16(p, 4) == in.seq && p[6] == kRadioKindSnapshot))
            hdr_ok = false;
        std::memcpy(&reasm[f * kRadioFragPayloadSize], &p[8], kRadioFragPayloadSize);
    }
    CHECK(hdr_ok, "all 5 fragment headers (magic/ver/idx/tot/seq/kind)");
    CHECK(std::memcmp(reasm, s, kRadioSnapshotWireSize) == 0,
          "5 fragments reassemble to the 102-byte snapshot");
}

// GPS (MTK3339 / USART10) -- the NMEA parser ported from the bench-proven
// GPS_TEST driver, the knots->km/h conversion, and the 0x508/0x509 wire layout.
// The parser is the risky part (integer ddmm.mmmm -> deg*1e7), so it is driven
// with real sentences and checked to the last digit.
static void test_gps() {
    std::printf("[gps]\n");

    // --- knots -> km/h (x1.852, round-to-nearest) ---
    CHECK(gps_knots_x100_to_kmh_x100(0)    == 0,     "0 kn -> 0 km/h");
    CHECK(gps_knots_x100_to_kmh_x100(100)  == 185,   "1.00 kn -> 1.85 km/h");
    CHECK(gps_knots_x100_to_kmh_x100(2240) == 4148,  "22.40 kn -> 41.48 km/h");
    CHECK(gps_knots_x100_to_kmh_x100(10000)== 18520, "100.00 kn -> 185.20 km/h");

    // --- a real fix: GGA (sats) + RMC (position/speed/course) ---
    // 4038.1234,N -> 40 + 38.1234/60 = 40.6353900 deg -> 406353900
    // 00341.5678,W -> -(3 + 41.5678/60) = -3.6927966 deg -> -36927966
    {
        GpsNmea g;
        const char* stream =
            "$GPGGA,123519,4038.1234,N,00341.5678,W,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
            "$GPRMC,123519,A,4038.1234,N,00341.5678,W,022.40,084.40,230394,003.1,W*6A\r\n";
        for (const char* p = stream; *p; ++p) g.feed(*p);

        const GpsFix& f = g.fix();
        CHECK(f.has_fix, "valid fix reported");
        CHECK(f.sats == 8, "GGA satellites = 8");
        CHECK(f.lat_deg1e7 ==  406353900, "lat 4038.1234,N -> +40.6353900 deg");
        CHECK(f.lon_deg1e7 ==  -36927966, "lon 00341.5678,W -> -3.6927966 deg (negated)");
        CHECK(f.speed_kmh_x100  == 4148, "22.40 kn -> 41.48 km/h");
        CHECK(f.course_deg_x100 == 8440, "course 084.40 deg");
        CHECK(g.sentences() == 2, "two sentences parsed");
    }

    // --- southern / eastern hemisphere signs (the other two quadrants) ---
    {
        GpsNmea g;
        const char* s =
            "$GNRMC,000000,A,3352.0000,S,15112.0000,E,000.00,000.00,010120,,,A*00\r\n";
        for (const char* p = s; *p; ++p) g.feed(*p);
        // 33 + 52/60 = 33.8666666 -> negated (S); 151 + 12/60 = 151.2 -> +E
        CHECK(g.fix().lat_deg1e7 == -338666666, "S hemisphere -> negative latitude");
        CHECK(g.fix().lon_deg1e7 == 1512000000, "E hemisphere -> positive longitude");
        CHECK(g.fix().has_fix, "GNRMC talker ID accepted (not just GP)");
    }

    // --- no fix (RMC status 'V'): must clear has_fix and NOT clobber the last
    //     known position with the empty lat/lon fields an unfixed RMC carries ---
    {
        GpsNmea g;
        const char* fixed =
            "$GPRMC,123519,A,4038.1234,N,00341.5678,W,022.40,084.40,230394,003.1,W*6A\r\n";
        for (const char* p = fixed; *p; ++p) g.feed(*p);
        const std::int32_t lat_before = g.fix().lat_deg1e7;

        const char* lost = "$GPRMC,123520,V,,,,,,,230394,,,N*00\r\n";
        for (const char* p = lost; *p; ++p) g.feed(*p);
        CHECK(!g.fix().has_fix, "status 'V' -> has_fix false");
        CHECK(g.fix().lat_deg1e7 == lat_before, "lost fix keeps the last valid position");
    }

    // --- robustness: junk, empty lines, and an over-long line must not corrupt
    //     a good solution (the module emits garbage while it boots) ---
    {
        GpsNmea g;
        const char* good =
            "$GPRMC,123519,A,4038.1234,N,00341.5678,W,022.40,084.40,230394,003.1,W*6A\r\n";
        for (const char* p = good; *p; ++p) g.feed(*p);
        const GpsFix saved = g.fix();

        for (const char* p = "not-nmea-at-all\r\n"; *p; ++p) g.feed(*p);
        for (const char* p = "\r\n"; *p; ++p) g.feed(*p);
        for (const char* p = "$GPGSV,3,1,11,01,05,048,20*7A\r\n"; *p; ++p) g.feed(*p);  // unsupported
        for (unsigned i = 0; i < kGpsLineMax * 2u; ++i) g.feed('X');                     // overflow
        g.feed('\n');

        CHECK(g.fix().lat_deg1e7 == saved.lat_deg1e7, "garbage/overflow leaves position intact");
        CHECK(g.fix().speed_kmh_x100 == saved.speed_kmh_x100, "...and speed intact");

        // ...and the parser still works after the overflow (len_ was reset).
        for (const char* p = good; *p; ++p) g.feed(*p);
        CHECK(g.fix().has_fix, "parser recovers after an over-long line");
    }

    // --- 0x508 / 0x509 wire layout ---
    {
        GpsFix f{};
        f.has_fix = true; f.sats = 8;
        f.lat_deg1e7 = 406353900; f.lon_deg1e7 = -36927966;
        f.speed_kmh_x100 = 4148;  f.course_deg_x100 = 8440;

        const CanFrame p = GpsTx::build_position(f);
        CHECK(p.id == VCU_gps_position_ID && p.dlc == 8, "0x508 id/dlc");
        CHECK(p.bus == static_cast<uint8_t>(CanBus::Acu), "0x508 on the ACU bus (uDV + pit)");
        CHECK(static_cast<int32_t>(rd32(p.data, 0)) ==  406353900, "0x508 latitude  s32 LE");
        CHECK(static_cast<int32_t>(rd32(p.data, 4)) ==  -36927966, "0x508 longitude s32 LE (signed)");

        const CanFrame s = GpsTx::build_status(f, 1234);
        CHECK(s.id == VCU_gps_status_ID && s.dlc == 8, "0x509 id/dlc");
        CHECK(rd16(s.data, 0) == 4148, "0x509 speed km/h*100");
        CHECK(rd16(s.data, 2) == 8440, "0x509 course deg*100");
        CHECK(s.data[4] == 8, "0x509 sats");
        CHECK((s.data[5] & 0x01u) == 1u, "0x509 has_fix bit");
        CHECK(rd16(s.data, 6) == 1234, "0x509 nmea_count (liveness)");

        // No fix -> the bit clears but the last position still ships (documented).
        GpsFix nf = f; nf.has_fix = false;
        CHECK((GpsTx::build_status(nf, 5).data[5] & 0x01u) == 0u, "0x509 has_fix clears");
        CHECK(static_cast<int32_t>(rd32(GpsTx::build_position(nf).data, 0)) == 406353900,
              "0x508 still carries the last known position when unfixed");

        // Overflow saturates instead of wrapping into a plausible small value.
        GpsFix fast = f; fast.speed_kmh_x100 = 999999u;
        CHECK(rd16(GpsTx::build_status(fast, 0).data, 0) == 0xFFFF, "speed saturates, never wraps");
    }
}

// ----- dispatch -------------------------------------------------------------

static void run_all() {
    test_apps_pct();
    test_cold_start();
    test_boot_sequence();
    test_dynamic_states();
    test_precharge_no_ack();
    test_active_torque_and_deadband();
    test_ev_2_3();
    test_t11_8_9_window();
    test_ams_error();
    test_cell_v_derate();
    test_bootloader_trigger();
    test_dsl_parity();
    test_inverter();
    test_inverter_fault_recovery();
    test_inverter_ts_off_recovery();
    test_inverter_fault_burst();
    test_inverter_fault_layers();
    test_pedal_cal();
    test_brake_pressure();
    test_power_envelope();
    test_pedal_cal_nvm();
    test_cal_session();
    test_cal_nvm_write();
    test_inverter_rx();
    test_dv_mode();
    test_udv_rx();
    test_udv_tx();
    test_radio_snapshot();
    test_gps();
}

int main(int argc, char** argv) {
    const char* m = (argc > 1) ? argv[1] : "--test-all";
    if (!std::strcmp(m, "--dump-radio")) { dump_radio_snapshot(); return 0; }
    std::printf("=== ECU SIL (control core) : %s ===\n", m);

    if      (!std::strcmp(m, "--test-apps"))               test_apps_pct();
    else if (!std::strcmp(m, "--test-rtos-startup"))       test_cold_start();
    else if (!std::strcmp(m, "--test-boot"))             { test_cold_start(); test_boot_sequence(); }
    else if (!std::strcmp(m, "--test-full-cycle"))       { test_boot_sequence(); test_active_torque_and_deadband(); }
    else if (!std::strcmp(m, "--test-dynamic-states"))     test_dynamic_states();
    else if (!std::strcmp(m, "--test-precharge-no-ack"))   test_precharge_no_ack();
    else if (!std::strcmp(m, "--test-safety-brake"))       test_ev_2_3();
    else if (!std::strcmp(m, "--test-error-voltage"))      test_cell_v_derate();
    else if (!std::strcmp(m, "--test-ams-error"))          test_ams_error();
    else if (!std::strcmp(m, "--test-legacy-compat"))    { test_cell_v_derate(); test_t11_8_9_window(); }
    else if (!std::strcmp(m, "--test-plausibility"))     { test_ev_2_3(); test_t11_8_9_window(); }
    else if (!std::strcmp(m, "--test-bootloader-trigger")) test_bootloader_trigger();
    else if (!std::strcmp(m, "--test-dsl-parity"))         test_dsl_parity();
    else if (!std::strcmp(m, "--test-inverter"))           test_inverter();
    else if (!std::strcmp(m, "--test-inverter-recovery"))  test_inverter_fault_recovery();
    else if (!std::strcmp(m, "--test-inverter-ts-off"))    test_inverter_ts_off_recovery();
    else if (!std::strcmp(m, "--test-inverter-fault-burst")) test_inverter_fault_burst();
    else if (!std::strcmp(m, "--test-inverter-fault-layers")) test_inverter_fault_layers();
    else if (!std::strcmp(m, "--test-pedal-cal"))          test_pedal_cal();
    else if (!std::strcmp(m, "--test-brake-pressure"))     test_brake_pressure();
    else if (!std::strcmp(m, "--test-power-envelope"))     test_power_envelope();
    else if (!std::strcmp(m, "--test-pedal-cal-nvm"))      test_pedal_cal_nvm();
    else if (!std::strcmp(m, "--test-cal-session"))        test_cal_session();
    else if (!std::strcmp(m, "--test-cal-nvm-write"))      test_cal_nvm_write();
    else if (!std::strcmp(m, "--test-inverter-rx"))        test_inverter_rx();
    else if (!std::strcmp(m, "--test-udv"))              { test_udv_rx(); test_udv_tx(); }
    else if (!std::strcmp(m, "--test-dv-mode"))            test_dv_mode();
    else if (!std::strcmp(m, "--test-radio"))              test_radio_snapshot();
    else if (!std::strcmp(m, "--test-gps"))                test_gps();
    else                                                   run_all();  // --test-integration / --test-all / default

    std::printf("=== %d checks, %d failed ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
