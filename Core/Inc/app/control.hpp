// SPDX-License-Identifier: proprietary
//
// control.hpp -- the pure ECU control core: the start / ready-to-drive FSM and
// the APPS/torque/plausibility computation. HAL-free and RTOS-free (it only
// takes a snapshot struct + a millisecond tick), so it compiles and runs in a
// host unit test with no mocks -- the AMS state_machine.hpp pattern.
//
// It is a non-blocking re-expression of the legacy VCU pre-jarama behaviour
// (its blocking startup sequence + the per-tick inverter switch + setTorque()).
// Two deliberate deviations from that legacy, both toward its documented
// intent (flagged inline in control.cpp):
//   1. the low-cell-voltage derate is APPLIED (legacy computed it but sent the
//      undated value);
//   2. the T.11.8.9 APPS-disagreement cut honours the 100 ms persistence window
//      (legacy had it commented out -> instant trip).
//
// What is NOT here: reading the ADC/GPIO (io_signals), the CAN wire encoding,
// and the inverter unit-map + two's-complement + E2E framing (the deferred
// inverter adapter, task #10). The core emits a torque PERCENTAGE and an
// abstract inverter MODE; the firmware layer encodes them.

#ifndef ECU_CONTROL_HPP_
#define ECU_CONTROL_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

// Abstract inverter command. The numeric values are the NX/EMC App_State_Req
// mode words (EMC_RX_SETPOINT_1 / 0x360); the inverter adapter writes them
// (with E2E) -- the core only decides which one.
enum class InvMode : uint8_t {
    Off          = 0x01,
    Ready        = 0x04,  // drive standby -> ready
    TorqueEnable = 0x06,
    Fault        = 0x13,  // fault / reset
};

// The start / ready-to-drive FSM.
enum class CtrlState : uint8_t {
    WaitInvVdcConfig = 0,  // wait for the inverter to report its DC-bus config (0x466)
    Precharge,             // stream 0x100; wait for AMS ok_precharge (0x020)
    WaitStartBrake,        // wait for START button + brake pressed
    R2dDelay,              // drive the RTDS buzzer for R2dSoundMs
    WaitInvStandby,        // command Ready; wait for inverter to reach ready
    Active,                // runtime torque
    AmsError,              // AMS latched Error -> inhibited, no precharge retry
};

// One 10 ms snapshot. Filled by control_task from io_signals (pedals/brake/
// button) + the vehicle_service snapshot (inverter + AMS). All HAL-free PODs.
struct CtrlInputs {
    // pedals / brake (raw 12-bit ADC) + debounced START button
    uint16_t apps1_raw = 0;
    uint16_t apps2_raw = 0;
    uint16_t brake_raw = 0;
    bool     start_button = false;
    // inverter feedback (FDCAN1 / EMC)
    bool     inv_present = false;        // any inverter frame seen recently
    bool     inv_vconfig_ready = false;  // 0x466 DC-bus config frame seen
    uint8_t  inv_state = 0;              // App_State_App (0x461); >=10 = fault
    uint16_t inv_dc_bus_V = 0;           // 0x466 DCBus_Voltage_V
    // AMS / ACU (FDCAN2)
    bool     ams_fresh = false;          // 0x020/0x4A0 recently received
    bool     ok_precharge = false;       // 0x020
    bool     ams_error = false;          // 0x4A0 fsm_state == AmsFsmError
    uint16_t v_cell_min_mV = 0;          // 0x12C / 0x4A0
};

struct CtrlOutput {
    CtrlState state = CtrlState::WaitInvVdcConfig;
    uint8_t   torque_pct = 0;    // 0..100, commanded (non-zero only in Active)
    InvMode   inv_mode = InvMode::Off;
    bool      rtds_on = false;   // drive the RTDS buzzer (R2dDelay)
    bool      ok_to_drive = false;
    // plausibility verdicts (driven into 0x700.flags for pit-diag / Block F)
    bool      ev_2_3 = false;    // brake+throttle implausibility (latched)
    bool      t11_8_9 = false;   // APPS disagreement past the 100 ms window
};

// Holds the FSM state + the stateful plausibility latches across 10 ms steps.
// Pure logic, no HAL/RTOS -> host-unit-testable.
class Controller {
public:
    CtrlOutput step(const CtrlInputs& in, uint32_t now_ms) noexcept;
    CtrlState  state() const noexcept { return state_; }

private:
    void enter_(CtrlState s, uint32_t now_ms) noexcept;

    CtrlState state_ = CtrlState::WaitInvVdcConfig;
    uint32_t  state_entry_ms_ = 0;
    bool      ev23_latched_ = false;
    bool      apps_disagree_active_ = false;
    uint32_t  apps_disagree_since_ms_ = 0;
};

// APPS travel %, clamped 0..100. Exposed for the pit-diag adapter (0x701) and
// unit tests.
uint8_t apps_pct(uint16_t raw, uint16_t adc_min, uint16_t adc_max) noexcept;

}  // namespace ecu

#endif  // ECU_CONTROL_HPP_
