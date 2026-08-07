// SPDX-License-Identifier: proprietary
//
// ECU-held DC-link discharge. See the header for the topology and for why the
// ECU can only ever ADD a reason to discharge.

#include "app/discharge.hpp"

namespace ecu {

using namespace config;

DischargeState Discharge::update(const DischargeInputs& in) noexcept {
    DischargeState st{};

    // The fault latch clears only when the AMS withdraws the request. Without
    // this the timeout would fire, release, and be re-armed on the very next
    // tick by a request that is still true -- because the condition that
    // produced it (a stranded link) is exactly what a failed discharge leaves
    // behind. That is an oscillation, not a retry.
    if (fault_ && !in.request) fault_ = false;

    if (!secured_) {
        // LATCH. Entering on the level, not an edge: the AMS's request is
        // "the link is stranded", which stays true while it is stranded, so
        // there is no transient to catch.
        if (in.request && !fault_) {
            secured_      = true;
            secured_at_ms_ = in.now_ms;
        }
    } else {
        // RELEASE ON OUR OWN MEASUREMENT, never on the request going away.
        // A single lost 0x021 mid-discharge must not abort it and re-strand the
        // link -- that asymmetry is the whole reason the hold lives here rather
        // than in the AMS.
        //
        // dc_bus_valid is required: a HELD 0x466 reading is not a measurement,
        // and a stale-low one would release the bleed early on a link that is
        // still charged. "Cannot confirm" therefore keeps securing, and the
        // timeout below is what stops that becoming indefinite.
        if (in.dc_bus_valid && in.dc_bus_V < DischargeReleaseV) {
            secured_ = false;
        } else if (static_cast<std::uint32_t>(in.now_ms - secured_at_ms_) >= DischargeTimeoutMs) {
            // The link is not falling: bleed resistor open, sense fault, or the
            // coil-interrupt relay did not obey. Stop securing and SAY SO.
            // Releasing is safe -- the AMS gates on its own voltage as well, so
            // it still will not arm; the difference is that now there is a
            // reason attached instead of a car that mysteriously never arms.
            secured_ = false;
            fault_   = true;
        }
    }

    st.secure = secured_;
    st.fault  = fault_;
    // COMMANDED, not confirmed. #198 asked for an auxiliary-contact readback
    // here and the team decided against wiring one; the header spells out the
    // two failures that leaves undetectable. Separate field rather than reusing
    // `secure` at the call sites, so adding the readback later is one line.
    st.engaged = secured_;
    return st;
}

}  // namespace ecu
