// SPDX-License-Identifier: proprietary

#include "app/io_signals.hpp"

#include "adc.h"     // hadc3
#include "main.h"    // HAL + the CubeMX GPIO label defines (START_Pin, ...)

namespace ecu {
namespace {

// One single-shot ADC3 conversion on `channel`. Safe-fails to 0 on any HAL
// error -- a bad read must look like "pedal released", never a stale value
// (FSAE: an APPS fault drives torque to 0).
std::uint16_t read_adc3(std::uint32_t channel) noexcept {
    ADC_ChannelConfTypeDef cfg = {};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &cfg) != HAL_OK) return 0;
    if (HAL_ADC_Start(&hadc3) != HAL_OK)               return 0;
    std::uint16_t v = 0;
    if (HAL_ADC_PollForConversion(&hadc3, 2u) == HAL_OK) {
        v = static_cast<std::uint16_t>(HAL_ADC_GetValue(&hadc3));
    }
    HAL_ADC_Stop(&hadc3);
    return v;
}

}  // namespace

void IoSignals::read(IoInputs& out) noexcept {
    // ADC3 channels: brake IN3 (PF7), APPS1 IN7 (PF8, shared-analog), APPS2 IN2 (PF9).
#if defined(ECU_STUB_BRAKE)
    // Bring-up: the brake line may be unpressurized, so the sensor can't reach
    // BrakeArmRaw. Assume the brake is fully pressed (skip the ADC) to exercise the
    // real R2D/RTDS sequence. Compile-time only -- never a flight build.
    out.brake_raw = 4095u;
#else
    out.brake_raw = read_adc3(ADC_CHANNEL_3);
#endif
    out.apps1_raw = read_adc3(ADC_CHANNEL_7);
    out.apps2_raw = read_adc3(ADC_CHANNEL_2);

#if defined(ECU_STUB_START)
    // Bring-up: the start button may not be wired, so assume it's pressed (skip
    // PB5). Compile-time only -- never a flight build.
    const bool pressed = true;
#else
    const bool pressed =
        (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET);
#endif
    out.start_button = debounce_start(pressed);
}

}  // namespace ecu
