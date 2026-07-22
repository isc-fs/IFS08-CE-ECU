// SPDX-License-Identifier: proprietary
//
// The vehicle RX state the ECU receives off the two CAN buses, held in one
// lock-free single-writer service (the AMS VehicleService pattern):
//   - inverter (FDCAN1 / EMC): standard telemetry 0x460..0x468
//   - AMS / ACU (FDCAN2): ok_precharge (0x020), min cell mV (0x12C), FSM (0x4A0)
//
// Single writer: CanRxTask (the only task that calls update_from_frame).
// Many readers: ControlTask via snapshot(). No mutex -- Cortex-M7 32-bit
// aligned loads/stores are atomic, every field is word-or-smaller, and the
// control predicates tolerate a one-cycle-stale snapshot. snapshot() returns
// the whole struct by value so a reader can't tear it at the struct level.
//
// RX decode is hand-rolled here (like the AMS) rather than via the DSL's
// sized-array decoders; the layouts mirror the .def files, and a host parity
// test guards against drift.

#pragma once

#include "app/can_frame.hpp"

#include <cstdint>

namespace ecu {

struct VehicleState {
    // --- inverter (FDCAN1 / EMC) ---
    std::uint32_t inv_uptime_ms     = 0;  // 0x460
    std::uint8_t  inv_core0_load_pct = 0;
    std::uint8_t  inv_core1_load_pct = 0;
    std::uint8_t  inv_state         = 0;  // 0x461 App_State_App (>=10 = fault)
    std::uint8_t  inv_error         = 0;  // 0x461 DEM_Code low byte
    std::uint16_t inv_dem_code      = 0;  // 0x461 DEM_Code (15 bits)
    bool          inv_dem_present   = false;  // 0x461 byte3 bit7: DEM active NOW vs latched history
    std::uint8_t  inv_foc_bit_state = 0;  // 0x461
    std::uint16_t inv_pwrstg_bit_state = 0;  // 0x461 (9 bits)
    std::int32_t  inv_speed_actual_rpm = 0;  // 0x462 mechanical rpm
    std::int32_t  inv_rpm           = 0;  // 0x463 EMachine_Speed_erpm (20-bit signed)
    std::int16_t  inv_current_d_raw = 0;  // 0x463, 1/32 A per bit
    std::int16_t  inv_current_q_raw = 0;  // 0x463, 1/32 A per bit
    std::int32_t  inv_current_actual_A = 0;  // rounded magnitude sqrt(Id^2 + Iq^2)
    std::uint16_t inv_volt_modulus_permil = 0;  // 0x463, 12 bits
    std::uint16_t inv_dc_bus_V      = 0;  // 0x466 DCBus_Voltage_V (10-bit)
    std::uint8_t  inv_temp_board    = 0;  // 0x464 board temp       (raw byte; -50 -> degC)
    std::uint8_t  inv_temp_pwrstg   = 0;  // 0x464 power-stage temp  (raw)
    std::uint8_t  inv_temp_motor1   = 0;  // 0x464 motor temp 1      (raw)
    std::uint8_t  inv_temp_motor2   = 0;  // 0x464 motor temp 2      (raw)
    std::uint16_t inv_kl30_mV       = 0;  // 0x465
    std::uint8_t  inv_cmd_src       = 0;
    std::uint8_t  inv_ctrl_type     = 0;
    std::uint8_t  inv_ctrl_mode     = 0;
    std::uint8_t  inv_pos_fb_src    = 0;
    std::int32_t  inv_ac_bus_power_W = 0;  // 0x466, rounded integer watts
    std::int16_t  inv_torque_max_feas_Ndm = 0;  // 0x467
    std::int16_t  inv_setpoint_d_raw = 0;  // 0x467, 1/32 A per bit
    std::int16_t  inv_setpoint_q_raw = 0;  // 0x467, 1/32 A per bit
    std::int16_t  inv_torque_est_Nm = 0;  // 0x468
    std::uint32_t last_inv_tick     = 0;  // any inverter frame
    std::uint32_t last_vconfig_tick = 0;  // 0x466 seen -> gates Precharge
    // --- AMS / ACU (FDCAN2) ---
    bool          ok_precharge      = false;  // 0x020 byte0 != 0
    std::uint16_t v_cell_min_mV     = 0;      // 0x12C / 0x4A0
    std::uint8_t  ams_fsm_state     = 0;      // 0x4A0 byte0 (== AmsFsmError -> Error)
    std::uint32_t last_ams_tick     = 0;      // any AMS frame
    // --- uDV / autonomous (FDCAN2, #17). Own freshness ticks -- uDV traffic
    //     must NOT keep the AMS freshness alive (or vice versa). ---
    std::int32_t  udv_torque_cmd    = 0;      // 0x507 torque command, s32 LE (integer %, unconditioned)
    std::uint8_t  udv_r2d_request   = 0;      // 0x510 byte0 (!= 0 = requesting R2D)
    std::uint32_t last_udv_cmd_tick = 0;      // 0x507 seen
    std::uint32_t last_udv_r2d_tick = 0;      // 0x510 seen

    // --- AMS per-module telemetry (FDCAN2, 0x131-0x137) for the radio/dash ---
    std::uint16_t vmin_module[5]    = {};     // 0x131/0x132, mV, per module
    std::uint16_t vmax_module[5]    = {};     // 0x133/0x134, mV, per module
    std::int16_t  current_accu_dA   = 0;      // 0x135, deciamps (+ = discharge)
    std::int16_t  current_dcdc_dA   = 0;      // 0x135, deciamps
    std::int16_t  tmax_module[5]    = {};     // 0x136/0x137, degC, per module
    std::int16_t  tmax_dcdc         = 0;      // 0x137, degC (AMS-side stub until a real sensor exists)
};

class VehicleService {
public:
    static VehicleService& instance() noexcept;

    // Single-writer: returns true if the frame matched a known RX id and the
    // state was updated (false -> the caller may count the drop).
    bool update_from_frame(const CanFrame& f) noexcept;

    [[nodiscard]] VehicleState snapshot() const noexcept;

    // Rollover-safe freshness (future-tick safe): a tick stamped slightly ahead
    // of `now` counts as just-seen, never ancient. Pure; public for tests.
    [[nodiscard]] static bool is_fresh(std::uint32_t now, std::uint32_t last,
                                       std::uint32_t window) noexcept;

    // Pure RX decoders, public for the parity + unit tests. `d` is the 8-byte
    // frame payload; layouts mirror the .def files (the source of truth).
    static bool          decode_ok_precharge(const std::uint8_t* d) noexcept;  // 0x020
    static std::uint16_t decode_v_cell_min(const std::uint8_t* d)  noexcept;   // 0x12C  BE16
    static std::uint8_t  decode_ams_fsm_state(const std::uint8_t* d) noexcept; // 0x4A0 byte0
    static std::uint16_t decode_ams_min_cell(const std::uint8_t* d) noexcept;  // 0x4A0 BE16 @ b4-5
    static std::uint8_t  decode_inv_state(const std::uint8_t* d)   noexcept;   // 0x461 b4 & 0x7F
    static std::int32_t  decode_inv_speed_actual(const std::uint8_t* d) noexcept; // 0x462 offset binary
    static std::int32_t  decode_inv_rpm(const std::uint8_t* d)     noexcept;   // 0x463 20-bit signed @ bit44
    static std::int32_t  decode_inv_current_actual_A(const std::uint8_t* d) noexcept; // 0x463 D/Q magnitude
    static std::uint16_t decode_inv_dc_bus_V(const std::uint8_t* d) noexcept;  // 0x466 10-bit @ bit16
    static std::int32_t  decode_udv_torque_cmd(const std::uint8_t* d) noexcept; // 0x507 s32 LE (integer %)

    // Condition the 0x507 integer percent into what the control core consumes:
    // negative -> 0 (fail-safe), > 100 clamps to 100. Pure.
    static std::uint8_t  condition_udv_torque(std::int32_t cmd) noexcept;

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ecu
