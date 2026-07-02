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
#include "app/udv_tx.hpp"        // uDV autonomous-contract TX builders (#17)
#include "app/vehicle_service.hpp" // inverter/AMS RX decoders (rpm / temps / state)

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
    CHECK(static_cast<uint32_t>(UDV_accel_cmd_ID)    == UdvAccelCmdId,     "UDV_accel_cmd_ID == config");
    CHECK(static_cast<uint32_t>(UDV_r2d_request_ID)  == UdvR2dRequestId,   "UDV_r2d_request_ID == config");
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

// uDV autonomous contract (#17) -- RX decode/store + the ECU->uDV TX builders.
static void test_udv_rx() {
    std::printf("[udv_rx]\n");

    // 0x507 payload is the raw IEEE-754 f32 pattern, little-endian on the wire.
    // 2.5f = 0x40200000 -> LE bytes {00 00 20 40}.
    const std::uint8_t f25[4] = {0x00, 0x00, 0x20, 0x40};
    CHECK(VehicleService::decode_udv_accel_raw(f25) == 0x40200000u, "f32 LE raw pattern (2.5f)");

    VehicleService& vs = VehicleService::instance();
    const std::uint32_t ams_tick_before = vs.snapshot().last_ams_tick;

    CanFrame a{};
    a.bus = static_cast<std::uint8_t>(CanBus::Acu);
    a.id  = UdvAccelCmdId;       // 0x507
    a.dlc = 4;
    a.data[0]=0x00; a.data[1]=0x00; a.data[2]=0x20; a.data[3]=0x40;
    a.timestamp_ms = 1234;
    CHECK(vs.update_from_frame(a), "0x507 accepted");
    CHECK(vs.snapshot().udv_accel_raw == 0x40200000u, "accel raw reaches the snapshot");
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

    // 0x506: s32 LE. 74560 = 0x00012340 -> {40 23 01 00}; -1 -> {FF FF FF FF}.
    const CanFrame rp = UdvTx::build_motor_rpm(74560);
    CHECK(rp.id == VCU_motor_rpm_ID && rp.dlc == 4, "0x506 id/dlc");
    CHECK(rp.data[0]==0x40 && rp.data[1]==0x23 && rp.data[2]==0x01 && rp.data[3]==0x00,
          "rpm +74560 little-endian");
    const CanFrame rn = UdvTx::build_motor_rpm(-1);
    CHECK(rn.data[0]==0xFF && rn.data[1]==0xFF && rn.data[2]==0xFF && rn.data[3]==0xFF,
          "rpm -1 sign-preserving (s32 LE)");
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
    test_udv_rx();
    test_udv_tx();
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
    else if (!std::strcmp(m, "--test-udv"))              { test_udv_rx(); test_udv_tx(); }
    else                                                   run_all();  // --test-integration / --test-all / default

    std::printf("=== %d checks, %d failed ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
