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
#include "app/bootloader.hpp"    // matches_trigger (pure, host-testable)
#include "can/can_codecs.hpp"    // DSL <Msg>_ID for the parity check

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
    in.inv_dc_bus_V      = PrechargeTargetV;
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
}

int main(int argc, char** argv) {
    const char* m = (argc > 1) ? argv[1] : "--test-all";
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
    else                                                   run_all();  // --test-integration / --test-all / default

    std::printf("=== %d checks, %d failed ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
