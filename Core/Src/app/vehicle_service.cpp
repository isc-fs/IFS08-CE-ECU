// SPDX-License-Identifier: proprietary

#include "app/vehicle_service.hpp"

#include "app/ecu_config.hpp"

#include <cstdint>

namespace ecu {

VehicleService& VehicleService::instance() noexcept {
    static VehicleService Instance;
    return Instance;
}

// ---- pure decoders (layouts mirror the .def files) -------------------------

bool VehicleService::decode_ok_precharge(const std::uint8_t* d) noexcept {
    return d[0] != 0u;
}

std::uint16_t VehicleService::decode_v_cell_min(const std::uint8_t* d) noexcept {
    return static_cast<std::uint16_t>(                       // BE16 @ bytes 0-1
        (static_cast<std::uint16_t>(d[0]) << 8) | d[1]);
}

std::uint8_t VehicleService::decode_ams_fsm_state(const std::uint8_t* d) noexcept {
    return d[0];
}

std::uint16_t VehicleService::decode_ams_min_cell(const std::uint8_t* d) noexcept {
    return static_cast<std::uint16_t>(                       // BE16 @ bytes 4-5
        (static_cast<std::uint16_t>(d[4]) << 8) | d[5]);
}

std::uint8_t VehicleService::decode_inv_state(const std::uint8_t* d) noexcept {
    return static_cast<std::uint8_t>(d[4] & 0x7Fu);          // App_State_App @ bit 32, 7 bits
}

std::uint16_t VehicleService::decode_inv_dc_bus_V(const std::uint8_t* d) noexcept {
    // DCBus_Voltage_V: 10-bit little-endian starting at frame bit 16.
    return static_cast<std::uint16_t>(
        d[2] | ((static_cast<std::uint16_t>(d[3]) & 0x03u) << 8));
}

std::int32_t VehicleService::decode_inv_rpm(const std::uint8_t* d) noexcept {
    // EMC_TX_STATE_4 (0x463) EMachine_Speed_erpm: 20-bit signed, Intel, DBC start
    // bit 44 -> the signal's low 4 bits are d[5] bits 4-7, then d[6], then d[7]
    // (NX0001-STS04_A16.dbc). 0x463 carries no E2E.
    std::uint32_t raw = (static_cast<std::uint32_t>(d[5]) >> 4)
                      | (static_cast<std::uint32_t>(d[6]) << 4)
                      | (static_cast<std::uint32_t>(d[7]) << 12);
    if (raw & 0x80000u) raw |= 0xFFF00000u;            // sign-extend from bit 19
    return static_cast<std::int32_t>(raw);
}

// ---- freshness -------------------------------------------------------------

bool VehicleService::is_fresh(std::uint32_t now, std::uint32_t last,
                              std::uint32_t window) noexcept {
    if (last == 0u) return false;
    const std::uint32_t age = (now >= last) ? (now - last) : 0u;
    return age <= window;
}

// ---- single-writer update --------------------------------------------------

bool VehicleService::update_from_frame(const CanFrame& f) noexcept {
    // --- inverter bus (FDCAN1) ---
    if (f.bus == static_cast<std::uint8_t>(CanBus::Inv)) {
        if (f.id == config::InvRxStateId) {                 // 0x461
            if (f.dlc < 5) return false;
            state_.inv_state     = decode_inv_state(f.data);
            state_.inv_error     = f.data[2];               // DEM_Code low byte
            state_.last_inv_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxDcBusId) {                 // 0x466
            if (f.dlc < 4) return false;
            state_.inv_dc_bus_V      = decode_inv_dc_bus_V(f.data);
            state_.last_vconfig_tick = f.timestamp_ms;
            state_.last_inv_tick     = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxRpmId) {                   // 0x463
            if (f.dlc < 8) return false;
            state_.inv_rpm       = decode_inv_rpm(f.data);
            state_.last_inv_tick = f.timestamp_ms;
            return true;
        }
        return false;
    }

    // --- AMS / ACU bus (FDCAN2) ---
    if (f.bus == static_cast<std::uint8_t>(CanBus::Acu)) {
        if (f.id == config::AcuOkPrechargeId) {             // 0x020
            state_.ok_precharge  = decode_ok_precharge(f.data);
            state_.last_ams_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuVCellMinId) {                // 0x12C
            if (f.dlc < 2) return false;
            state_.v_cell_min_mV = decode_v_cell_min(f.data);
            state_.last_ams_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AmsStatusId) {                  // 0x4A0
            if (f.dlc < 8) return false;
            state_.ams_fsm_state = decode_ams_fsm_state(f.data);
            state_.v_cell_min_mV = decode_ams_min_cell(f.data);
            state_.last_ams_tick = f.timestamp_ms;
            return true;
        }
        return false;
    }
    return false;
}

VehicleState VehicleService::snapshot() const noexcept {
    return state_;
}

}  // namespace ecu
