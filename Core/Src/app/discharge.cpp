// SPDX-License-Identifier: proprietary
//
// ECU-held DC-link discharge. See the header for the topology and for why the
// ECU can only ever ADD a reason to discharge.

#include "app/discharge.hpp"

namespace ecu {

using namespace config;

DischargeState Discharge::update(const DischargeInputs& in) noexcept {
    DischargeState st{};

    // A FRESH 0x021 saying the AMS is out of Start is a report, not a lost
    // frame (header: the precharge edge). tsms does not enter into it: with the
    // SDC open the hardware bleed is on and our hold changes nothing either way.
    const bool left_start = in.interlock_fresh && !in.fsm_in_start;

    // The fault latch clears only when the AMS withdraws the request. Without
    // this the timeout would fire, release, and be re-armed on the very next
    // tick by a request that is still true -- because the condition that
    // produced it (a stranded link) is exactly what a failed discharge leaves
    // behind. That is an oscillation, not a retry.
    // Cleared when the stranding condition goes away -- keyed on the AMS's
    // observations rather than the full `stranded` term, because after a failed
    // discharge the link is still charged by definition, so including the
    // voltage would make the fault unclearable.
    if (fault_ && !(in.fsm_in_start && in.tsms)) fault_ = false;

    // Precharge edge, rule 1. Remember a drained link for as long as the AMS
    // stays in Start; a later reading above the threshold is then the AMS
    // precharging on a 0x021 we have not received yet, never charge left behind
    // -- in Start every contactor is open. Forgotten on a fresh "out of Start",
    // so a link left charged when the AMS comes back is secured as before.
    if (left_start) {
        link_seen_drained_ = false;
    } else if (in.dc_bus_valid && in.dc_bus_V <= DischargeReleaseV) {
        link_seen_drained_ = true;
    }

    // The ECU's half of the decision. All three terms, and the third is ours:
    // a charged link we can actually SEE. Without it, entering Start with an
    // already-drained link and a stale 0x466 would secure every time and hold to
    // the timeout, because the release path needs a valid reading.
    const bool stranded = in.fsm_in_start && in.tsms &&
                          in.dc_bus_valid && (in.dc_bus_V > DischargeReleaseV);

    if (!secured_) {
        // LATCH. On the level, not an edge: a stranded link keeps these
        // observations true for as long as it is stranded, so there is no fast
        // SDC transient anyone has to catch.
        if (stranded && !fault_ && !link_seen_drained_) {
            secured_      = true;
            secured_at_ms_ = in.now_ms;
        }
    } else {
        // RELEASE ON OUR OWN MEASUREMENT, never on the request going away.
        // A single lost 0x021 mid-discharge must not abort it and re-strand the
        // link -- that asymmetry is the whole reason the hold lives here rather
        // than in the AMS.
        //
        // The one exception is not the request going away but a positive report
        // (precharge edge, rule 2): a FRESH 0x021 saying the AMS has left Start.
        // It is then charging the link or has the SDC open, and the bleed must
        // not stay across it. A stale frame never gets here as left_start.
        //
        // dc_bus_valid is required: a HELD 0x466 reading is not a measurement,
        // and a stale-low one would release the bleed early on a link that is
        // still charged. "Cannot confirm" therefore keeps securing, and the
        // timeout below is what stops that becoming indefinite.
        if (left_start) {
            secured_ = false;
        } else if (in.dc_bus_valid && in.dc_bus_V < DischargeReleaseV) {
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
    // COMMANDED, not confirmed. The design asked for an auxiliary-contact readback
    // here and the team decided against wiring one; the header spells out the
    // two failures that leaves undetectable. Separate field rather than reusing
    // `secure` at the call sites, so adding the readback later is one line.
    st.engaged = secured_;
    return st;
}

}  // namespace ecu
