// SPDX-License-Identifier: proprietary
//
// discharge.hpp -- ECU-held DC-link discharge (#198).
//
// THE CASE. The discharge relay follows the SDC and nothing else: SDC open ->
// coil de-energised -> its NC contact closes -> bleed across the link. That is
// fail-safe and it stays exactly as it is. What it cannot do is FINISH: cycle
// the SDC inside the discharge time -- an e-stop tapped and released, a
// connector bounce, a driver resetting immediately -- and the coil re-energises,
// the contact opens, and the discharge stops part-way. The link is then frozen
// at an intermediate voltage with nothing draining it.
//
// If that residual sits above 95 % of pack, the AMS's precharge-complete
// criterion (dc_bus >= 95 % of pack) is already true when precharge STARTS. The
// precharge resistor never carries meaningful current and the check whose whole
// purpose is proving that resistor and contactor work is satisfied by leftover
// charge. A DEAD PRECHARGE PATH PASSES ITS OWN SELF-TEST.
//
// THE TOPOLOGY (AMS team's proposal, and it is better than a parallel driver).
// A normally-closed, ECU-driven relay sits in SERIES WITH THE DISCHARGE RELAY
// COIL -- not in the bleed path:
//
//      coil energised  =  (SDC closed)  AND  (ECU permits)
//      discharging     =  (SDC open)    OR   (ECU secures)
//
// The ECU can therefore only ever ADD a reason to discharge. It is physically
// incapable of defeating the SDC's authority, which is the property that makes
// a new component in this path defensible at all. A parallel driver would have
// fought the SDC for the coil; a relay in series with the BLEED could only ever
// interrupt a discharge, which is a rules problem.
//
// WHO DECIDES. Split, deliberately, and the split is the AMS's design (0x021
// ACU_discharge_interlock). They publish two RAW OBSERVATIONS -- never a
// pre-computed request -- and WE supply the third term and combine them:
//
//      secure = fsm_in_start  AND  tsms  AND  (OUR dc_bus > DischargeReleaseV)
//               \____________ 0x021 ____________/   \____ our measurement ____/
//
// Their reasoning, and it is right: the third term is a measurement only the ECU
// has, and routing the whole decision through a CAN frame would put a stale
// value in the middle of it.
//
// The Start qualifier is load-bearing: TSMS-on with a charged link is ALSO true
// in normal driving, so without it this would command a bleed across a live
// tractive system at speed. In Start the AIRs are all commanded open, so the
// same reading uniquely means "the shutdown circuit was cycled and the discharge
// did not finish". The AMS knows FSM state and TSMS; we know neither.
//
// The dc_bus term is not optional either. Without it, entering Start with an
// already-drained link and a stale 0x466 secures on every entry and holds to the
// timeout -- the release needs a VALID reading, which never comes.
//
// Note this is a LEVEL condition, not an edge. The stranded link keeps the
// request true for as long as it is stranded, so nothing has to catch a fast
// SDC transient -- which is what an ECU-side SDC sense would have had to do,
// and would probably have missed at a 50 ms bounce.
//
// WHAT WE OWN. The hold. Latch on the request, release on OUR OWN measurement
// -- so a single lost CAN frame mid-discharge cannot abort it and re-strand the
// link. That asymmetry is the entire value the ECU adds.
//
// TIMEOUT. If we secure and the link does NOT fall -- bleed resistor gone open,
// sense fault -- an indefinite hold would leave a car that never arms with
// nothing indicating why. So: give up after DischargeTimeoutMs, report a fault,
// and stop securing. Releasing is safe because the AMS gates on its OWN voltage
// too; it simply will not arm, which is the correct outcome, but now with a
// reason attached.
//
// BOOT. Issue #198 asks for "power-up default engaged". Taken literally that is
// DANGEROUS with this topology: an ECU watchdog reset mid-drive would boot with
// the TS live and the AIRs closed, and securing would put a transient-duty bleed
// resistor across a live pack. What the requirement actually protects against is
// the ECU REPORTING "not engaged" before it knows, so that is what we do
// instead: dc_bus_valid reads 0 until 0x466 is fresh, and the AMS treats
// "cannot confirm" as "do not arm". Satisfied without ever forcing a discharge
// into a link we have not looked at.
//
// >>> NO AUXILIARY CONTACT READBACK -- TEAM DECISION, AND IT HAS A COST. <<<
// #198 asked for the reported bit to come from a genuine auxiliary contact
// rather than from the command, on the grounds that "I commanded permit" is
// worthless if the relay did not obey. We report the COMMAND. Two things are
// therefore NOT detectable from the ECU, and both were named in that issue:
//
//   1. Coil-interrupt relay stuck OPEN (driver shorted on, contact welded).
//      We release the command, report engaged = 0, and the bleed is STILL
//      connected. The AMS then closes an AIR into a transient-duty resistor.
//      This is the failure the readback existed to prevent.
//   2. A discharge the SDC started on its own. The bit only ever means "the ECU
//      is commanding one", not "the bleed is connected". Narrower in practice
//      -- an SDC-driven discharge implies the SDC is open, so the AMS is not
//      arming anyway -- but the two are NOT the same statement and the AMS gate
//      is written against the second.
//
// If an auxiliary pole is ever wired, feed it in here and report THAT: the
// change is one input and one line, and it buys back case 1.

#ifndef DISCHARGE_HPP_
#define DISCHARGE_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

struct DischargeInputs {
    // The AMS's two raw observations (0x021), ALREADY conditioned on that
    // frame's freshness by the caller. NOT a request -- see the header note.
    bool          fsm_in_start   = false;  // 0x021 bit 0: AIRs and precharge all open
    bool          tsms           = false;  // 0x021 bit 1: SDC complete -> bleed disconnected
    std::uint16_t dc_bus_V       = 0;      // 0x466, relayed
    bool          dc_bus_valid   = false;  // 0x466 fresh -- a held reading is not a measurement
    std::uint32_t now_ms         = 0;
};

struct DischargeState {
    // Drive the coil-interrupt relay. true = open the coil path = force a
    // discharge. This is a COMMAND; it is not what gets reported.
    bool secure  = false;
    // What goes on 0x100. COMMANDED state -- see the readback note in the file
    // header for what that cannot catch. Mirrors `secure`; kept as a separate
    // field so that wiring an auxiliary contact later changes one assignment
    // rather than every call site.
    bool engaged = false;
    // Secured for longer than DischargeTimeoutMs without the link falling.
    // Sticky until the request is withdrawn.
    bool fault   = false;
};

class Discharge {
public:
    DischargeState update(const DischargeInputs& in) noexcept;

private:
    bool          secured_       = false;
    bool          fault_         = false;
    std::uint32_t secured_at_ms_ = 0;
};

}  // namespace ecu

#endif  // DISCHARGE_HPP_
