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
    bool          inv_dem_present   = false;  // 0x461 byte3 bit7: DEM active NOW vs latched history
    // The two LOWER fault layers, same frame (0x461), never decoded until #148.
    // The W90's layers cascade upward (manual 9.2), so a latched L3 DEM that
    // will not clear may be held up by a live L1/L2 condition -- which no CAN
    // command can clear. Bitmasks, not enums: several bits can be set at once.
    std::uint16_t inv_pwrstg_bits   = 0;  // 0x461 bits 47-55, L1 PwrStg_BitState (9 bits)
    std::uint8_t  inv_emctrl_bits   = 0;  // 0x461 bits 39-46, L2 EMCtrl_FOC_BitState (8 bits)
    std::int32_t  inv_rpm           = 0;  // 0x463 EMachine_Speed_erpm (20-bit signed)
    std::uint16_t inv_dc_bus_V      = 0;  // 0x466 DCBus_Voltage_V (10-bit)
    std::uint8_t  inv_temp_board    = 0;  // 0x464 board temp       (raw byte; -50 -> degC)
    std::uint8_t  inv_temp_pwrstg   = 0;  // 0x464 power-stage temp  (raw)
    std::uint8_t  inv_temp_motor1   = 0;  // 0x464 motor temp 1      (raw)
    std::uint8_t  inv_temp_motor2   = 0;  // 0x464 motor temp 2      (raw)
    // 0x464 gets its OWN timestamp, like 0x135. last_inv_tick is stamped by
    // 0x461/0x463/0x464/0x466 alike, so a dead 0x464 with the rest of the
    // inverter still talking would leave the thermal cap running off
    // temperatures frozen minutes ago -- and frozen-cold reads as healthy.
    std::uint32_t last_inv_temps_tick = 0;   // 0x464 seen
    std::uint32_t last_inv_tick     = 0;  // any inverter frame
    // 0x461 SPECIFICALLY (not any inverter frame): inv_state and the DEM ride
    // on it, and the whole climb/fault ladder is steered by them, so its own
    // arrival rate is what matters -- last_inv_tick is also refreshed by
    // 0x463/0x464/0x466 and would mask a slow 0x461 (#148).
    std::uint32_t last_inv_state_tick = 0;  // 0x461 only
    std::uint8_t  inv_state_seq       = 0;  // wrapping count of 0x461 frames
    std::uint32_t last_vconfig_tick = 0;  // 0x466 seen -> gates Precharge
    // --- AMS / ACU (FDCAN2) ---
    bool          ok_precharge      = false;  // 0x020 byte0 != 0
    std::uint16_t v_cell_min_mV     = 0;      // 0x12C / 0x4A0
    std::uint8_t  ams_fsm_state     = 0;      // 0x4A0 byte0 (== AmsFsmError -> Error)
    // 0x4A0 byte2, bit N = module N reporting. The pack thermal cap uses this to
    // decide which per-module temperature is worth acting on -- an offline
    // module's slot still holds whatever arrived last, and stale-warm misleads
    // exactly as much as stale-cold.
    std::uint8_t  module_online_mask = 0;     // 0x4A0 byte2
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
    // 0x135 gets its OWN timestamp, unlike the rest of the AMS block. The IR
    // compensation in the low-cell derate reads current_accu_dA, and a
    // bus-level liveness flag (last_ams_tick, which every AMS frame stamps)
    // would let a dead 0x135 keep compensating off a value frozen minutes ago
    // -- masking an empty pack, which is the one direction that is unsafe.
    std::uint32_t last_currents_tick = 0;     // 0x135 seen
    std::int16_t  tmax_module[5]    = {};     // 0x136/0x137, degC, per module
    std::int16_t  tmax_dcdc         = 0;      // 0x137, degC (AMS-side stub until a real sensor exists)
    // Own timestamp, like 0x135 and 0x464 before it. last_ams_tick is stamped by
    // every AMS frame, and for the pack thermal cap freshness is not a backstop
    // -- 0 degC is a real pack temperature, so staleness is the ONLY thing that
    // can catch an uninitialised state.
    std::uint32_t last_tmax_tick    = 0;      // 0x136/0x137 seen
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
    static std::int32_t  decode_udv_torque_cmd(const std::uint8_t* d) noexcept; // 0x507 s32 LE (integer %)

    // Condition the 0x507 integer percent into what the control core consumes:
    // negative -> 0 (fail-safe), > 100 clamps to 100. Pure.
    static std::uint8_t  condition_udv_torque(std::int32_t cmd) noexcept;

private:
    VehicleService() = default;
    mutable VehicleState state_ = {};
};

}  // namespace ecu
