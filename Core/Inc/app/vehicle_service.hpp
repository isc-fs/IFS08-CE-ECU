// SPDX-License-Identifier: proprietary
//
// The vehicle RX state the ECU receives off the two CAN buses, held in one
// lock-free single-writer service (the AMS VehicleService pattern):
//   - inverter (FDCAN1 / EMC): App_State (0x461), rpm (0x463), temps (0x464),
//     DC-bus volts (0x466)
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
    std::uint8_t  inv_state         = 0;  // 0x461 App_State_App (>=10 = fault)
    std::uint8_t  inv_error         = 0;  // 0x461 DEM_Code low byte
    std::int32_t  inv_rpm           = 0;  // 0x463 EMachine_Speed_erpm (20-bit signed)
    std::uint16_t inv_dc_bus_V      = 0;  // 0x466 DCBus_Voltage_V (10-bit)
    std::uint8_t  inv_temp_board    = 0;  // 0x464 board temp       (raw byte; -50 -> degC)
    std::uint8_t  inv_temp_pwrstg   = 0;  // 0x464 power-stage temp  (raw)
    std::uint8_t  inv_temp_motor1   = 0;  // 0x464 motor temp 1      (raw)
    std::uint8_t  inv_temp_motor2   = 0;  // 0x464 motor temp 2      (raw)
    std::uint32_t last_inv_tick     = 0;  // any inverter frame
    std::uint32_t last_vconfig_tick = 0;  // 0x466 seen -> gates Precharge
    // --- AMS / ACU (FDCAN2) ---
    bool          ok_precharge      = false;  // 0x020 byte0 != 0
    std::uint16_t v_cell_min_mV     = 0;      // 0x12C / 0x4A0
    std::uint8_t  ams_fsm_state     = 0;      // 0x4A0 byte0 (== AmsFsmError -> Error)
    std::uint32_t last_ams_tick     = 0;      // any AMS frame
    // --- uDV / autonomous (FDCAN2, #17). Own freshness ticks -- uDV traffic
    //     must NOT keep the AMS freshness alive (or vice versa). ---
    std::uint32_t udv_accel_raw     = 0;      // 0x507 raw IEEE-754 f32 bits (LE wire); app bit-casts
    std::uint8_t  udv_r2d_request   = 0;      // 0x510 byte0 (!= 0 = requesting R2D)
    std::uint32_t last_udv_cmd_tick = 0;      // 0x507 seen
    std::uint32_t last_udv_r2d_tick = 0;      // 0x510 seen
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
    static std::int32_t  decode_inv_rpm(const std::uint8_t* d)     noexcept;   // 0x463 20-bit signed @ bit44
    static std::uint16_t decode_inv_dc_bus_V(const std::uint8_t* d) noexcept;  // 0x466 10-bit @ bit16
    static std::uint32_t decode_udv_accel_raw(const std::uint8_t* d) noexcept; // 0x507 LE32 (raw f32 bits)

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ecu
