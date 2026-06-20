// SPDX-License-Identifier: proprietary
//
// Application-side entry into the stm32-can-bootloader -- the ECU's recovery
// path home. The BL (sector 0) reads RTC->BKP0R on every reset: if it holds
// BlBootReqMagic (0xB00710AD) the BL stays in its CAN listen loop awaiting a
// flash; otherwise it validates the app's firmware_info record and jumps to
// 0x08020000.
//
// request_reboot() is the one-way path from a running app back into the BL.
// It is triggered by a single CAN frame (id 0x002, payload 0xB007AD12) on the
// ACU/shared bus (FDCAN2) -- the same externally-accessible bus the pit tool
// and the flasher use. matches_trigger() is pure logic, kept inline so the host
// unit-test build exercises it without HAL/FreeRTOS.
//
// NOTE vs the AMS: the ECU owns no contactors (the AMS does), so request_reboot
// does not drive relays -- the reset alone stops all ECU TX, and the inverter
// falls back to its own safe state when our commands stop.

#pragma once

#include "app/can_frame.hpp"
#include "app/ecu_config.hpp"

#include <cstdint>
#include <cstring>

namespace ecu {

// Why we're rebooting into the BL -- stamped into BKP2R, survives the reset.
enum class JumpReason : std::uint32_t {
    ManualRequest = 1,   // a CAN 0x002 trigger from the pit tool
};

class Bootloader {
public:
    // Drain in-flight TX, stamp the reason into BKP2R, write BlBootReqMagic to
    // BKP0R, NVIC_SystemReset. Never returns. (Defined in bootloader.cpp, which
    // pulls the HAL -- not part of the host build.)
    [[noreturn]] static void request_reboot(
        JumpReason reason = JumpReason::ManualRequest) noexcept;

    // True iff a frame is the boot-request trigger: the ACU bus, id 0x002,
    // dlc 4, payload 0xB007AD12. Pure -> host-testable.
    [[nodiscard]] static bool matches_trigger(const CanFrame& f) noexcept {
        if (f.bus != static_cast<std::uint8_t>(CanBus::Acu)) return false;
        if (f.id  != config::BlBootTriggerCanId)             return false;
        if (f.dlc != config::BlBootTriggerDlc)               return false;
        return std::memcmp(f.data, config::BlBootTriggerPayload,
                           config::BlBootTriggerDlc) == 0;
    }
};

}  // namespace ecu
