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
#include "app/inverter.hpp"      // inverter setpoint encoders (0x360/0x362)
#include "app/vehicle_service.hpp" // inverter/AMS RX decoders (rpm / temps / state)
#include "app/app_globals.h"     // telemetry-visible last control state

using namespace ecu;
using namespace ecu::config;

namespace ecu {
uint32_t telemetry_period_ms_for_test();
void telemetry_emit_for_test(const VehicleState& v, uint16_t seq);
}

static CanFrame g_dash_tx[24];
static int g_dash_tx_count = 0;
static uint8_t g_radio_tx[8][32];
static int g_radio_tx_count = 0;

namespace ecu {
bool can_tx_post(const CanFrame& f) noexcept {
    if (g_dash_tx_count < static_cast<int>(sizeof(g_dash_tx) / sizeof(g_dash_tx[0]))) {
        g_dash_tx[g_dash_tx_count++] = f;
    }
    return true;
}
}

extern "C" void Telemetry_Send32(const uint8_t payload[32]) {
    if (g_radio_tx_count < static_cast<int>(sizeof(g_radio_tx) / sizeof(g_radio_tx[0]))) {
        std::memcpy(g_radio_tx[g_radio_tx_count++], payload, 32);
    }
}

extern "C" uint32_t osKernelGetTickCount(void) { return 123456u; }
extern "C" void osDelayUntil(uint32_t) {}

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
    CHECK(static_cast<uint32_t>(ACU_vmin_module_a_ID) == AcuVminModuleAId, "ACU_vmin_module_a_ID == config");
    CHECK(static_cast<uint32_t>(ACU_vmin_module_b_ID) == AcuVminModuleBId, "ACU_vmin_module_b_ID == config");
    CHECK(static_cast<uint32_t>(ACU_vmax_module_a_ID) == AcuVmaxModuleAId, "ACU_vmax_module_a_ID == config");
    CHECK(static_cast<uint32_t>(ACU_vmax_module_b_ID) == AcuVmaxModuleBId, "ACU_vmax_module_b_ID == config");
    CHECK(static_cast<uint32_t>(ACU_currents_ID)      == AcuCurrentsId,    "ACU_currents_ID == config");
    CHECK(static_cast<uint32_t>(ACU_tmax_module_a_ID) == AcuTmaxModuleAId, "ACU_tmax_module_a_ID == config");
    CHECK(static_cast<uint32_t>(ACU_tmax_module_b_ID) == AcuTmaxModuleBId, "ACU_tmax_module_b_ID == config");
    CHECK(static_cast<uint32_t>(AMS_status_ID)       == AmsStatusId,       "AMS_status_ID == config");
    CHECK(static_cast<uint32_t>(BL_boot_trigger_ID)  == BlBootTriggerCanId, "BL_boot_trigger_ID == config");
}

// Inverter TX adapter -- IDs / modes / byte layout / torque map matched
// byte-for-byte to the original VCU; the torque sign is negated for the motor's
// mechanical mounting; the E2E bytes go out as 0 (as the original VCU sent them).
static void test_inverter() {
    std::printf("[inverter]\n");
    CHECK(Inverter::torque_to_nm_req(5)   == 0,    "<10% (deadband) -> 0");
    CHECK(Inverter::torque_to_nm_req(10)  == 0,    "10% -> 0 (map start)");
    CHECK(Inverter::torque_to_nm_req(100) == -240, "100% -> -240 (mapped + mechanically negated)");
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

// Per-module AMS RX (0x131-0x137): all big-endian on the wire (AMS-owned
// layout), unlike the dash-facing CAN3 frames TelemetryTask builds (LE).
static void test_ams_module_rx() {
    std::printf("[ams_module_rx]\n");
    VehicleService& vs = VehicleService::instance();

    CanFrame vmin_a{};
    vmin_a.bus = static_cast<std::uint8_t>(CanBus::Acu);
    vmin_a.id  = AcuVminModuleAId;            // 0x131
    vmin_a.dlc = 6;
    vmin_a.data[0]=0x0C; vmin_a.data[1]=0xE4; // 3300 mV BE
    vmin_a.data[2]=0x0C; vmin_a.data[3]=0xEE; // 3310 mV BE
    vmin_a.data[4]=0x0C; vmin_a.data[5]=0xF8; // 3320 mV BE
    CHECK(vs.update_from_frame(vmin_a), "0x131 accepted");

    CanFrame vmin_b{};
    vmin_b.bus = static_cast<std::uint8_t>(CanBus::Acu);
    vmin_b.id  = AcuVminModuleBId;            // 0x132
    vmin_b.dlc = 4;
    vmin_b.data[0]=0x0D; vmin_b.data[1]=0x02; // 3330 mV BE
    vmin_b.data[2]=0x0D; vmin_b.data[3]=0x0C; // 3340 mV BE
    CHECK(vs.update_from_frame(vmin_b), "0x132 accepted");

    CanFrame currents{};
    currents.bus = static_cast<std::uint8_t>(CanBus::Acu);
    currents.id  = AcuCurrentsId;              // 0x135
    currents.dlc = 4;
    currents.data[0]=0xFF; currents.data[1]=0xCE; // -50 dA BE (signed, discharge negative here = charging)
    currents.data[2]=0x00; currents.data[3]=0x0C; // 12 dA BE
    CHECK(vs.update_from_frame(currents), "0x135 accepted");

    CanFrame tmax_b{};
    tmax_b.bus = static_cast<std::uint8_t>(CanBus::Acu);
    tmax_b.id  = AcuTmaxModuleBId;              // 0x137
    tmax_b.dlc = 6;
    tmax_b.data[0]=0x00; tmax_b.data[1]=0x21;   // 33 degC BE
    tmax_b.data[2]=0x00; tmax_b.data[3]=0x22;   // 34 degC BE
    tmax_b.data[4]=0x00; tmax_b.data[5]=0x2A;   // 42 degC BE
    CHECK(vs.update_from_frame(tmax_b), "0x137 accepted");

    const VehicleState s = vs.snapshot();
    CHECK(s.vmin_module[0]==3300 && s.vmin_module[1]==3310 && s.vmin_module[2]==3320,
          "0x131 per-module vmin 0..2 decoded BE");
    CHECK(s.vmin_module[3]==3330 && s.vmin_module[4]==3340,
          "0x132 per-module vmin 3..4 decoded BE");
    CHECK(s.current_accu_dA==-50 && s.current_dcdc_dA==12,
          "0x135 currents decoded BE signed");
    CHECK(s.tmax_module[3]==33 && s.tmax_module[4]==34 && s.tmax_dcdc==42,
          "0x137 tmax 3..4 + dcdc stub decoded BE signed");
}

static void test_telemetry_dash() {
    std::printf("[telemetry_dash]\n");
    std::memset(g_dash_tx, 0, sizeof(g_dash_tx));
    std::memset(g_radio_tx, 0, sizeof(g_radio_tx));
    g_dash_tx_count = 0;
    g_radio_tx_count = 0;
    g_last_torque_pct = 42;
    g_last_ctrl_state = 7;
    g_last_apps1_raw = 2500;
    g_last_apps2_raw = 2100;
    g_last_brake_raw = 1200;
    g_last_start_button = 1;
    g_last_ev_2_3 = 1;
    g_last_t11_8_9 = 0;

    VehicleState v{};
    v.inv_state = 4;
    v.inv_error = 5;
    v.inv_dc_bus_V = 388;
    v.inv_temp_board = 90;
    v.inv_temp_pwrstg = 91;
    v.inv_temp_motor1 = 82;
    v.inv_rpm = 123456;
    v.last_vconfig_tick = 99;
    v.ok_precharge = true;
    v.v_cell_min_mV = 3300;
    v.ams_fsm_state = 2;
    v.vmin_module[0] = 100; v.vmin_module[1] = 101; v.vmin_module[2] = 102;
    v.vmin_module[3] = 103; v.vmin_module[4] = 104;
    v.vmax_module[0] = 200; v.vmax_module[1] = 201; v.vmax_module[2] = 202;
    v.vmax_module[3] = 203; v.vmax_module[4] = 204;
    v.current_accu_dA = -50;
    v.current_dcdc_dA = 12;
    v.tmax_module[0] = 30; v.tmax_module[1] = 31; v.tmax_module[2] = 32;
    v.tmax_module[3] = 33; v.tmax_module[4] = 34;
    v.tmax_dcdc = 42;

    telemetry_emit_for_test(v, 0x1234u);

    CHECK(telemetry_period_ms_for_test() == 200u, "DASH telemetry period is 200 ms");
    CHECK(g_dash_tx_count == 18, "emits all 18 DASH CAN3 frames (0x510..0x521, docs/CAN3_MAP.md)");
    for (int i = 0; i < g_dash_tx_count; ++i) {
        CHECK(g_dash_tx[i].bus == static_cast<uint8_t>(CanBus::Dash), "DASH frame uses FDCAN3 bus");
    }

    static const uint32_t kExpectedIds[18] = {
        0x510u, 0x511u, 0x512u, 0x513u, 0x514u, 0x515u, 0x516u, 0x517u, 0x518u,
        0x519u, 0x51Au, 0x51Bu, 0x51Cu, 0x51Du, 0x51Eu, 0x51Fu, 0x520u, 0x521u
    };
    for (int i = 0; i < 18; ++i) {
        CHECK(g_dash_tx[i].id == kExpectedIds[i], "DASH CAN3 frame order matches docs/CAN3_MAP.md");
    }

    CHECK(g_dash_tx[0].dlc == 8, "status frame 0x510 DLC 8");
    CHECK(g_dash_tx[0].data[0] == 4 && g_dash_tx[0].data[1] == 42, "0x510 inv state + torque");
    CHECK(g_dash_tx[0].data[2] == 0x01, "0x510 byte2: bit0 EV.2.3 set, bit1 T11.8.9 clear");
    CHECK(g_dash_tx[0].data[3] == 1, "0x510 precharge");
    CHECK(g_dash_tx[0].data[4] == 1, "0x510 start button (real, from ControlTask)");
    CHECK(g_dash_tx[0].data[6] == 0x34 && g_dash_tx[0].data[7] == 0x12, "0x510 sequence LE");

    CHECK(g_dash_tx[1].dlc == 6, "pedals frame 0x511 DLC 6");
    CHECK(g_dash_tx[1].data[0] == 0xC4 && g_dash_tx[1].data[1] == 0x09, "0x511 apps1 raw LE (2500)");
    CHECK(g_dash_tx[1].data[2] == 0x34u && g_dash_tx[1].data[3] == 0x08, "0x511 apps2 raw LE (2100)");
    CHECK(g_dash_tx[1].data[4] == 0xB0u && g_dash_tx[1].data[5] == 0x04, "0x511 brake raw LE (1200)");

    CHECK(g_dash_tx[2].id == 0x512u && g_dash_tx[2].data[0] == 0x84 && g_dash_tx[2].data[1] == 0x01, "0x512 VDC LE");
    CHECK(g_dash_tx[7].id == 0x517u && g_dash_tx[7].data[0] == 7 && g_dash_tx[7].data[1] == 2, "0x517 control + AMS state");

    // 0x518: soc placeholder (byte0) + real currents/temp_dcdc (from ACU_currents
    // 0x135 / ACU_tmax_module_b 0x137, forwarded LE for the dash).
    CHECK(g_dash_tx[8].data[0] == 0, "0x518 soc placeholder (AMS has no SOC estimator)");
    CHECK(g_dash_tx[8].data[1] == 0xCEu && g_dash_tx[8].data[2] == 0xFFu, "0x518 corriente_accu LE (-50 dA)");
    CHECK(g_dash_tx[8].data[3] == 0x0Cu && g_dash_tx[8].data[4] == 0x00u, "0x518 corriente_dcdc LE (12 dA)");
    CHECK(g_dash_tx[8].data[5] == 0x2Au && g_dash_tx[8].data[6] == 0x00u, "0x518 temp_dcdc LE (42 degC)");

    // 0x515/0x516/0x519/0x51A: no real source yet (see telemetry_task.cpp
    // comments) -- placeholder, must stay all-zero payload.
    static const int kPlaceholderIdx[] = {5, 6, 9, 10};
    for (int idx : kPlaceholderIdx) {
        bool all_zero = true;
        for (uint8_t b = 0; b < g_dash_tx[idx].dlc; ++b) {
            if (g_dash_tx[idx].data[b] != 0) { all_zero = false; break; }
        }
        CHECK(all_zero, "placeholder DASH frame payload is all-zero (no real source yet)");
    }

    // 0x51C/0x51D: per-module vmin (real, from ACU_vmin_module_a/b).
    CHECK(g_dash_tx[12].data[0]==100 && g_dash_tx[12].data[2]==101 && g_dash_tx[12].data[4]==102,
          "0x51C vmin modulos 0..2 LE");
    CHECK(g_dash_tx[13].data[0]==103 && g_dash_tx[13].data[2]==104,
          "0x51D vmin modulos 3..4 LE");

    // 0x51E/0x51F: per-module vmax (real, from ACU_vmax_module_a/b).
    CHECK(g_dash_tx[14].data[0]==200 && g_dash_tx[14].data[2]==201 && g_dash_tx[14].data[4]==202,
          "0x51E vmax modulos 0..2 LE");
    CHECK(g_dash_tx[15].data[0]==203 && g_dash_tx[15].data[2]==204,
          "0x51F vmax modulos 3..4 LE");

    // 0x520/0x521: per-module tmax + temp_dcdc (real, from ACU_tmax_module_a/b).
    CHECK(g_dash_tx[16].data[0]==30 && g_dash_tx[16].data[2]==31 && g_dash_tx[16].data[4]==32,
          "0x520 tmax modulos 0..2 LE");
    CHECK(g_dash_tx[17].data[0]==33 && g_dash_tx[17].data[2]==34 && g_dash_tx[17].data[4]==42,
          "0x521 tmax modulos 3..4 + temp_dcdc LE");

    // 0x51B: gps_longitude placeholder (bytes 0-3) + tick_ms real (bytes 4-7).
    CHECK(g_dash_tx[11].data[0] == 0 && g_dash_tx[11].data[1] == 0 &&
          g_dash_tx[11].data[2] == 0 && g_dash_tx[11].data[3] == 0,
          "0x51B gps_longitude placeholder is zero");
    CHECK(g_dash_tx[11].data[4] == 0x40u && g_dash_tx[11].data[5] == 0xE2u &&
          g_dash_tx[11].data[6] == 0x01u && g_dash_tx[11].data[7] == 0x00u,
          "0x51B tick_ms LE (123456, real RTOS tick)");

    // Radio: 2 RF_FAST + 5 RF_SLOW fragments, byte layout matching
    // IFS08-TE-main's ISC_RTT_serial.py parse_radio_v2_frame() exactly.
    CHECK(g_radio_tx_count == 7, "emits 2 RF_FAST + 5 RF_SLOW radio fragments");
    for (int i = 0; i < g_radio_tx_count; ++i) {
        CHECK(g_radio_tx[i][0] == 0xEC && g_radio_tx[i][1] == 0x02, "radio magic/version");
    }
    CHECK(g_radio_tx[0][2] == 0 && g_radio_tx[0][3] == 2 && g_radio_tx[0][6] == 3,
          "RF_FAST frag0 header (kind=3, count=2)");
    CHECK(g_radio_tx[1][2] == 1 && g_radio_tx[1][3] == 2 && g_radio_tx[1][6] == 3,
          "RF_FAST frag1 header");
    for (int i = 2; i < 7; ++i) {
        CHECK(g_radio_tx[i][2] == (i - 2) && g_radio_tx[i][3] == 5 && g_radio_tx[i][6] == 4,
              "RF_SLOW frag header (kind=4, count=5)");
    }

    // RF_FAST frag0: matches ISC_RTT_serial.py's `kind == RF_FAST and fragment_index == 0`.
    const uint8_t* rf0 = g_radio_tx[0];
    CHECK(rf0[8]==7 && rf0[9]==4 && rf0[10]==2, "RF_FAST frag0 ecu/inv/ams state");
    CHECK(rf0[11]==1, "RF_FAST frag0 boton_arranque");
    CHECK(rf0[12]==1, "RF_FAST frag0 ok_precarga");
    CHECK(rf0[13]==1 && rf0[14]==0, "RF_FAST frag0 flag_ev_2_3/flag_t11_8_9");
    CHECK(rf0[15]==1 && rf0[16]==5, "RF_FAST frag0 inv_vdc_ready + inv_error");
    CHECK(rf0[17]==0x2A && rf0[18]==0x00, "RF_FAST frag0 torque_total LE (42)");
    CHECK(rf0[19]==0x84 && rf0[20]==0x01, "RF_FAST frag0 inv_dc_bus_voltage LE (388)");
    CHECK(rf0[21]==0xE4 && rf0[22]==0x0C, "RF_FAST frag0 v_celda_min LE (3300)");
    CHECK(rf0[23]==0xC4 && rf0[24]==0x09, "RF_FAST frag0 s1_aceleracion LE (2500)");
    CHECK(rf0[25]==0x34 && rf0[26]==0x08, "RF_FAST frag0 s2_aceleracion LE (2100)");
    CHECK(rf0[27]==0xB0 && rf0[28]==0x04, "RF_FAST frag0 s_freno LE (1200)");

    // RF_FAST frag1: temps/rpm real; speed/current placeholder (0x465 unverified).
    const uint8_t* rf1 = g_radio_tx[1];
    CHECK(rf1[8]==82 && rf1[10]==91 && rf1[12]==90, "RF_FAST frag1 temps");
    CHECK(rf1[14]==0x40 && rf1[15]==0xE2 && rf1[16]==0x01 && rf1[17]==0x00, "RF_FAST frag1 rpm LE (123456)");
    bool speed_current_zero = true;
    for (int b = 18; b <= 25; ++b) if (rf1[b] != 0) speed_current_zero = false;
    CHECK(speed_current_zero, "RF_FAST frag1 speed/current placeholder is zero");

    // RF_SLOW frag0: soc placeholder + real currents/temp_dcdc + gps placeholder + real tick_ms.
    const uint8_t* rs0 = g_radio_tx[2];
    CHECK(rs0[8]==0, "RF_SLOW frag0 soc placeholder");
    CHECK(rs0[9]==0xCE && rs0[10]==0xFF, "RF_SLOW frag0 corriente_accu LE (-50 dA)");
    CHECK(rs0[11]==0x0C && rs0[12]==0x00, "RF_SLOW frag0 corriente_dcdc LE (12 dA)");
    CHECK(rs0[13]==0x2A && rs0[14]==0x00, "RF_SLOW frag0 temp_dcdc LE (42)");
    CHECK(rs0[23]==0x40 && rs0[24]==0xE2 && rs0[25]==0x01 && rs0[26]==0x00, "RF_SLOW frag0 tick_ms LE (123456)");

    // RF_SLOW frag1: GPS lat/lon, all placeholder.
    bool latlon_zero = true;
    for (int b = 8; b <= 15; ++b) if (g_radio_tx[3][b] != 0) latlon_zero = false;
    CHECK(latlon_zero, "RF_SLOW frag1 gps lat/lon placeholder is zero");

    // RF_SLOW frag2/3/4: real per-module vmin/vmax/tmax.
    const uint8_t* rs2 = g_radio_tx[4];
    CHECK(rs2[8]==100 && rs2[10]==101 && rs2[12]==102 && rs2[14]==103 && rs2[16]==104,
          "RF_SLOW frag2 vmin_modulo[0..4] LE");
    const uint8_t* rs3 = g_radio_tx[5];
    CHECK(rs3[8]==200 && rs3[10]==201 && rs3[12]==202 && rs3[14]==203 && rs3[16]==204,
          "RF_SLOW frag3 vmax_modulo[0..4] LE");
    const uint8_t* rs4 = g_radio_tx[6];
    CHECK(rs4[8]==30 && rs4[10]==31 && rs4[12]==32 && rs4[14]==33 && rs4[16]==34,
          "RF_SLOW frag4 temp_max_modulo[0..4] LE");
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
    test_inverter_rx();
    test_ams_module_rx();
    test_telemetry_dash();
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
    else if (!std::strcmp(m, "--test-inverter"))           test_inverter();
    else if (!std::strcmp(m, "--test-inverter-recovery"))  test_inverter_fault_recovery();
    else if (!std::strcmp(m, "--test-inverter-rx"))        test_inverter_rx();
    else if (!std::strcmp(m, "--test-ams-module-rx"))      test_ams_module_rx();
    else if (!std::strcmp(m, "--test-telemetry-dash"))     test_telemetry_dash();
    else                                                   run_all();  // --test-integration / --test-all / default

    std::printf("=== %d checks, %d failed ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
