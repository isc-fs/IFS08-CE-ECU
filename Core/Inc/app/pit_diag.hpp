// SPDX-License-Identifier: proprietary
//
// Pit-diag frame builders: thin adapters that turn the live system state into
// the 0x700-0x705 / 0x7E1 CanFrames (the ECU's CAN-only observability stream,
// gated by 0x7E0). Pure (no HAL) -- the scaling lives here, the wire layout in
// the DSL .def files. CanTxTask/DiagTask call these; the gate + cadence live
// there.

#pragma once

#include "app/can_frame.hpp"
#include "app/control.hpp"
#include "app/error_latch.hpp"
#include "app/io_signals.hpp"
#include "app/reset_cause.hpp"
#include "app/vehicle_service.hpp"

#include <cstdint>

namespace ecu {

// Runtime health gathered by DiagTask for the 0x704 frame.
struct HealthMetrics {
    std::uint16_t free_heap     = 0;
    std::uint16_t min_free_heap = 0;
    std::uint8_t  task_ran_mask = 0;
    ResetCause    reset_cause   = ResetCause::Unknown;
    std::uint8_t  uptime_s      = 0;
    FaultCode     last_fault    = FaultCode::None;
};

class PitDiag {
public:
    static CanFrame build_status(const CtrlOutput& c, const VehicleState& v,
                                 bool start_button) noexcept;   // 0x700
    static CanFrame build_dv(const CtrlOutput& c, const CtrlInputs& in,
                             const VehicleState& v) noexcept;   // 0x707
    static CanFrame build_pedals(const IoInputs& io) noexcept;  // 0x701
    static CanFrame build_inverter(const VehicleState& v) noexcept; // 0x702
    static CanFrame build_inverter_temps(const VehicleState& v) noexcept; // 0x706
    static CanFrame build_fwinfo() noexcept;                    // 0x703
    static CanFrame build_health(const HealthMetrics& m) noexcept;  // 0x704
    static CanFrame build_brake(const IoInputs& io) noexcept;   // 0x705
    static CanFrame build_ack(bool enabled) noexcept;           // 0x7E1
};

}  // namespace ecu
