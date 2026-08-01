// SPDX-License-Identifier: proprietary
//
// pedal_cal.hpp -- the pedal calibration set, as RUNTIME data.
//
// These seven values used to be compile-time constexpr in ecu_config.hpp, so
// calibrating a pedal meant editing a header, rebuilding and reflashing -- only
// a firmware engineer could do it, which is why BrakePressedRaw and
// BrakeDvHardRaw were still uncalibrated IFS06 placeholders (#169). They are
// runtime now so an operator can calibrate over CAN from the pit tool.
//
// HAL-free and dependency-free on purpose: the pure control core includes this,
// so it must stay host-compilable for SIL. Defaults are the values that were
// previously constexpr, so a build with no stored calibration behaves EXACTLY
// as before.
//
// NOT calibratable and deliberately still compile-time: the deadbands, the
// FSAE plausibility percentages (Ev23*, AppsDisagree*) and every timing. Those
// are design decisions, not per-car measurements, and an operator must not be
// able to move them.

#ifndef PEDAL_CAL_HPP_
#define PEDAL_CAL_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

// The full calibration record. Field order is the on-the-wire order for
// 0x7E4 (the four APPS values) and 0x7E5 (the four brake values).
struct PedalCal {
    // --- APPS: raw ADC at rest and at the mechanical stop, per channel ---
    std::uint16_t apps1_min = config::Apps1AdcMin;
    std::uint16_t apps1_max = config::Apps1AdcMax;
    std::uint16_t apps2_min = config::Apps2AdcMin;
    std::uint16_t apps2_max = config::Apps2AdcMax;
    // --- Brake: the span, plus the three thresholds derived from it ---
    // brake_rest is 0 in the defaults because it has never been measured; 0
    // means "span unknown" and callers fall back to their previous behaviour.
    std::uint16_t brake_rest    = config::BrakeRestRaw;
    std::uint16_t brake_arm     = config::BrakeArmRaw;
    std::uint16_t brake_dv_hard = config::BrakeDvHardRaw;
    std::uint16_t brake_pressed = config::BrakePressedRaw;
};

// Validation flag bits, mirrored onto 0x7E3 byte 4 so the pit tool can tell the
// operator which rule failed. 0 = the candidate calibration is acceptable.
namespace cal_flag {
inline constexpr std::uint8_t Apps1SpanTooSmall = 1u << 0;
inline constexpr std::uint8_t Apps2SpanTooSmall = 1u << 1;
inline constexpr std::uint8_t AppsSpanMismatch  = 1u << 2;
inline constexpr std::uint8_t AppsNotMonotonic  = 1u << 3;
inline constexpr std::uint8_t BrakeSpanTooSmall = 1u << 4;
inline constexpr std::uint8_t BrakeOrder        = 1u << 5;
inline constexpr std::uint8_t OutOfAdcRange     = 1u << 6;
}  // namespace cal_flag

// Minimum usable travel, in raw 12-bit counts. Today's calibration spans 860
// (APPS1) and 680 (APPS2); anything under a few hundred counts means the pedal
// was not swept properly or a sensor is not moving, and would turn ADC noise
// into torque.
inline constexpr std::uint16_t CalMinAppsSpan  = 300;
inline constexpr std::uint16_t CalMinBrakeSpan = 300;
// The two APPS channels are different sensors and legitimately have different
// spans (860 vs 680 today, a ratio of 1.26). A much larger disparity means one
// channel was mis-captured. Expressed as a percentage of the LARGER span.
inline constexpr std::uint16_t CalMaxAppsSpanRatioPct = 200;
inline constexpr std::uint16_t CalAdcMax = 4095;   // 12-bit

// Validate a candidate calibration. Returns a bitmask of cal_flag::*; 0 means
// acceptable. Pure -- no HAL, no state -- so the SIL suite covers it directly
// and the same function gates both the CAN commit path and boot-time loading.
//
// LIMITATION, deliberate and documented: rest+full endpoints alone CANNOT
// detect the failure T.11.8.9 exists to catch. Both channels are normalised to
// 0..100 % by construction, so they agree at the endpoints no matter how badly
// they diverge in between -- mid-travel divergence comes from sensor
// non-linearity, which endpoint capture cannot see. AppsSpanMismatch is a weak
// proxy. Catching the real thing needs either a mid-travel capture point or a
// post-commit verification sweep; see #169.
[[nodiscard]] std::uint8_t validate_cal(const PedalCal& c) noexcept;

// Brake travel as a percentage, 0..100.
//
// Uses the CALIBRATED span (brake_rest..brake_pressed) when a rest point has
// been captured. Until then brake_rest is 0 and this falls back to the legacy
// full-12-bit-range scaling, which is what shipped before and is why a released
// brake reads about 14 % in the pit tool today (raw ~560 of 4095). That number
// cannot be fixed without measuring rest -- scaling to a span nobody has
// measured would just be a different wrong answer -- so it stays until an
// operator captures BRAKE_REST (#169).
[[nodiscard]] std::uint8_t brake_pct(std::uint16_t raw, const PedalCal& c) noexcept;

}  // namespace ecu

#endif  // PEDAL_CAL_HPP_
