// SPDX-License-Identifier: proprietary
//
// ecu_config.hpp -- the single place every tunable ECU constant lives
// (analogue of the AMS ams_config.hpp). HAL-free: only plain integral /
// floating constants, so the pure control core that includes it stays
// host-testable. CAN IDs/DLCs are NOT here -- they come from the code-first
// DSL (ecu::<Msg>_ID / _DLC). Board pin / ADC-channel mapping (HAL-coupled)
// lives in the firmware layer (io_signals / tasks), not here.
//
// Constants tagged COMMISSION are placeholders carried from the legacy VCU
// (IFS06 board) and MUST be re-measured on the assembled car with real
// sensors before any drive.

#ifndef ECU_CONFIG_HPP_
#define ECU_CONFIG_HPP_

#include <cstdint>

namespace ecu::config {

// ---- Task periods ----------------------------------------------------------
inline constexpr uint32_t ControlPeriodMs      = 10;    // the realtime ControlTask tick
inline constexpr uint32_t DiagPeriodMs         = 1000;  // DiagTask (0x704 health)
inline constexpr uint32_t PitDiagStreamMs      = 100;   // 0x700-0x705 cadence while enabled
inline constexpr uint32_t CanRxWaitMs          = 100;   // CanRxTask queue wait -> 0x704 liveness wake on a quiescent bus (< DiagPeriodMs)
// uDV contract (#17/#127) cyclic: 0x504 ts_active / 0x505 brake_over_limit /
// 0x511 r2d_confirm. CONTRACT with the uDV -- their AS state machine times out
// TS-active at 400 ms (4 missed) and FS-Rules T11.9.4 caps detection at 500 ms,
// so do NOT raise this above ~125 ms without agreeing it on #127 first.
inline constexpr uint32_t UdvTxPeriodMs        = 100;

// ---- Start / ready-to-drive FSM -------------------------------------------
inline constexpr uint32_t PrechargeTimeoutMs   = 10000; // no precharge -> retry
inline constexpr uint32_t R2dSoundMs           = 2000;  // RTDS buzzer duration

// Inverter App_State feedback values (EMC_TX_STATE_2 / 0x461, App_State_App).
// NOTE on provenance: 3/4/10/11 are BENCH-PROVEN on this inverter (the fault
// recovery chain 11 -> 0x0D -> 3 -> 0x04 -> 4 -> Active was observed). 0 and 13
// come from the legacy IFS07 switch and are NOT confirmed here -- the W90
// manual's state machine (section 9.1) lists OFF/READY/SPEED/TORQUE/CURRENT/
// FAULT with no "standby" or "shutdown" at all, so treat them as unverified
// until a real capture says otherwise (#148). They are kept because the pit-diag
// VAL tables and the SIL use them, NOT because the values are established.
inline constexpr uint8_t  InvOffState          = 0;   // UNVERIFIED (IFS07-derived)
inline constexpr uint8_t  InvStandbyState      = 3;   // bench-proven
inline constexpr uint8_t  InvReadyState        = 4;   // bench-proven
// Torque enabled -- the state the inverter reports once it accepts 0x06 from
// Ready. Needed to answer "is the inverter still IN the drive?", which is not
// the same question as "is it faulted?" (#191).
inline constexpr uint8_t  InvTorqueEnableState = 6;   // bench-observed
inline constexpr uint8_t  InvSoftFaultState    = 10;  // soft fault -> reset with InvMode::Fault (0x13)
inline constexpr uint8_t  InvHardFaultState    = 11;  // hard fault -> recover with InvMode::HardFaultReset (0x0D)
// 13 is now BENCH-CONFIRMED (2026-07-29): observed on 0x700 inv_state after a
// TS-off cycle, with the DEM cleared to latched history and L1/L2 clean -- the
// inverter genuinely parks here and refuses Ready. 0 remains IFS07-derived.
inline constexpr uint8_t  InvShutdownState     = 13;  // bench-confirmed 2026-07-29

// AMS FSM state (0x4A0 byte0) that means a latched Error (vs a re-armable Start).
inline constexpr uint8_t  AmsFsmError          = 5;

// ---- Pedals / brake (raw 12-bit ADC) --------------------------------------
// APPS travel calibration: pct = clamp((raw - min) * 100 / (max - min), 0, 100).
inline constexpr uint16_t Apps1AdcMin          = 2490;  // bench-cal 2026-06-22 (rest 2476 + margin)
inline constexpr uint16_t Apps1AdcMax          = 3350;  // bench-cal (full 3363 - headroom)
inline constexpr uint16_t Apps2AdcMin          = 2345;  // bench-cal (rest 2332 + margin)
inline constexpr uint16_t Apps2AdcMax          = 3025;  // bench-cal (full 3037 - headroom)

// Brake reading with the pedal fully released. NEVER MEASURED -- 0 means
// "span unknown", which keeps brake_pct on its legacy full-ADC-range scaling.
// Captured by the operator calibration wizard (#169); once non-zero it also
// enables the brake-span validation rule.
inline constexpr uint16_t BrakeRestRaw         = 0;     // COMMISSION: unmeasured
// Brake pressure sensor: Variohm EuroSensor EPT1400 (docs/EPT1400_pressure_sensor.pdf),
// ratiometric 0.5 V at zero pressure to 4.5 V at full scale. The board divider
// ahead of the ADC is known (R8 1k series, R9 2k shunt = 2/3), so pressure is an
// ABSOLUTE map from raw counts and needs no calibration -- see pedal_cal.hpp.
//
// FULL-SCALE RANGE from the order code (EPT1400-K-04000-B-...): 40 bar.
// This is the one value the maths cannot derive; everything else follows from
// the datasheet and the divider. 0 would mean unknown and suppress the reading.
//
// At 40 bar the scale is 82.7 counts/bar, which finally makes the three brake
// thresholds reviewable in physical units:
//   BrakeArmRaw      750 ->  4.1 bar   light press, arms R2D
//   BrakeDvHardRaw  2500 -> 25.2 bar   DV R2D gate + the 0x505 verdict to uDV
//   BrakePressedRaw 3000 -> 31.3 bar   brake full travel (brake_pct 100 %)
// Those are the inherited IFS06 numbers -- now at least judgeable rather than
// opaque. 4.1 bar to arm and ~31 bar for "brake pressed" are plausible; the
// 25.2 bar DV gate still wants checking against what the EBS actually holds.
inline constexpr uint16_t BrakeSensorFullScaleBar = 40;  // EPT1400 order code: 40 bar (04000)
inline constexpr uint16_t BrakeArmRaw          = 750;   // on-car cal 2026-06-27: brake-to-arm (R2D); released ~580 (noise to ~730), arm just above
// Brake FULL TRAVEL, the top of the brake_pct scale. It used to also gate the
// EV.2.3 brake+throttle cut; that rule was deleted in FS-Rules 2024 and the cut
// with it, so this is now purely a scaling endpoint and no longer arms anything.
inline constexpr uint16_t BrakePressedRaw      = 3000;  // COMMISSION: brake full travel
// DV (#17): the "established" hard-braking limit. The EBS holds HARD braking for
// the autonomous R2D; the ECU verifies it on its own brake sensor before honouring
// a 0x510 R2D request, and streams the binary verdict on 0x505 (same threshold).
inline constexpr uint16_t BrakeDvHardRaw       = 2500;  // COMMISSION: set from the brake cal
// BRING-UP brake stub, controlled by THIS value (no build flag): != 0 makes
// io_signals inject it as brake_raw instead of reading the ADC; 0 = real ADC
// (flight). Set ABOVE BrakeDvHardRaw (2500) to arm the DV R2D (bench: 2700).
// It no longer has to stay below BrakePressedRaw -- the EV.2.3 cut that used to
// trip there was deleted with the rule in FS-Rules 2024. MUST be 0 for
// flight — folds away at compile time (constexpr), so a 0 build carries no stub.
inline constexpr uint16_t StubBrakeRaw         = 0;

// ---- BENCH STUBS (bring-up only) — config toggles, NOT build flags ----------
// All OFF on dev. Each is consumed as `if constexpr (config::StubX)`, so a false
// toggle DISCARDS the stub code at compile time — a flight build (all false)
// carries none of it, same guarantee the old -D flags gave. The bench/car-stubs
// branch flips the ones a bench needs. ⚠ NEVER true for a flight/drive build —
// StubNoAms / StubNoInverter DISABLE safety gates.
inline constexpr bool StubNoAms      = false;  // assume precharge-OK + AMS-healthy (no AMS on the bus). DISABLES the AMS gate.
inline constexpr bool StubNoInverter = false;  // fake inverter present/vconfig/Ready (no inverter). DISABLES the inverter handshake.
inline constexpr bool StubStart      = false;  // assume start button pressed (PB5 unwired). MANUAL R2D only — keep false for a DV/uDV R2D test (else it preempts the 0x510 path).

// ---- BENCH TELEMETRY stub (bring-up only) — config toggle, NOT a build flag -
// When true, TelemetryTask fills the VehicleState with a deterministic synthetic
// SWEEP (keyed on the frame seq) instead of the live snapshot, so the nRF24 radio
// snapshot AND the FDCAN3 dashboard carry MOVING values with no live AMS/inverter
// on the bus -- lets you validate the ground station / dash decode + display on a
// bare bench. Consumed as `if constexpr (config::StubTelemetryDummy)`, so false
// discards it at compile time. ⚠ NEVER true for a flight/drive build (it would
// broadcast fake pack/inverter telemetry). Covers the CAN-sourced fields; the
// pedals/torque/flags still come from ControlTask's g_last_* mirrors.
inline constexpr bool     StubTelemetryDummy   = false;

// ---- Torque / FSAE plausibility -------------------------------------------
// Both sensors must exceed this before any torque is produced. This is a
// sanity gate (a sensor failed low reads 0 and blocks torque whatever the other
// says), NOT the T.11.8.9 plausibility check -- that is AppsDisagreePct below.
// Kept BELOW DeadbandLowPct so the deadband alone decides the pedal onset;
// at 8 it silently dominated a 5% deadband and pushed the real onset to ~9%.
inline constexpr uint8_t  AppsAgreementPct     = 3;     // both sensors must exceed to produce torque
// Pedal deadband: commanded torque below this is zeroed. Lowered 10 -> 5
// (2026-07-29, driver reported too much dead travel). NOTE this value must stay
// in step with the InvTorqueMap* zero-crossing below -- the map is built so that
// exactly DeadbandLowPct maps to 0 Nm, and if the two disagree you get a second,
// invisible deadband on top of this one.
inline constexpr uint8_t  DeadbandLowPct       = 5;     // below -> 0
inline constexpr uint8_t  DeadbandHighPct      = 90;    // above -> 100
// BRING-UP torque cap (% of commanded torque, applied in control_task). 100 = no cap.
// Clamps torque for on-stands / freewheel testing. *** MUST be 100 for any flight /
// drive build *** -- unlike the old off-by-default ECU_BRINGUP_TORQUE_CAP_PCT build
// flag this is ALWAYS applied; lower it only on stands.
inline constexpr uint8_t  TorqueCap            = 100;
inline constexpr uint8_t  AppsDisagreePct      = 10;    // T.11.8.9: |apps1-apps2| > this is implausible
inline constexpr uint32_t AppsDisagreePersistMs= 100;   // T.11.8.9: must persist this long before cut

// ---- Low-cell-voltage torque derate ---------------------------------------
// Linear ramp: 100 % at/above the knee, down to CellVDerateFloorPct at the
// floor, flat below. The ramp is DERIVED from the three numbers below -- it used
// to carry a hand-fitted slope/intercept pair (1.357 / 3750.0) valid only for
// the exact 3500/2800 knee/floor it was fitted to, so moving a threshold left
// the curve silently wrong. Now the thresholds are the only tunables.
//
// KNEE = 2800, not 3500. The old 3500 mV knee fired during NORMAL driving: a
// healthy pack sags well past 3500 mV under race current, so the derate was
// effectively always active and the car ran permanently scaled back. This is
// end-of-pack protection, not a load limiter. Bounding power is the EV 2.2.1
// envelope's job (#177); this only has to keep the last of the pack from being
// pulled under the AMS cut.
inline constexpr uint16_t CellVDerateKneeMv    = 2800;  // at/above: no derate
// COMMISSION: must sit at or above the AMS undervoltage cut, which lives in the
// AMS firmware and is NOT visible from this repo. 2500 mV is the usual Li-ion
// NMC discharge floor; confirm against the AMS config and the cell datasheet
// before any endurance run. Too low and the AMS opens the AIRs mid-corner with
// the car still at 5 % torque instead of coasting down.
inline constexpr uint16_t CellVDerateFloorMv   = 2500;  // at/below: flat floor pct
// 13 %, NOT 5 %. The percentage the core works in is not a linear 0..100 scale
// to 240 Nm -- InvTorqueMap* is re-based so that DeadbandLowPct (5) maps to
// EXACTLY 0 Nm, which is what stops the pedal deadband and the map deadband
// stacking. So a floor of "5 %" commanded literally nothing: 5*240/95 -
// 1200/95 = 12 - 12 = 0. The comment here used to claim "deliberately
// non-zero" and was simply wrong -- both paths that reach this floor (the ramp
// and the raw backstop) cut torque completely while the FSM stays in Active
// with TorqueEnable asserted, which from the driver's seat is indistinguishable
// from the car breaking. 13 % is 20 Nm: a real limp-home, same intent as the
// two thermal floors (20 % -> 38 Nm). The static_assert below is what stops
// this recurring.
inline constexpr uint8_t  CellVDerateFloorPct  = 13;    // -> 20 Nm; see the assert below
inline constexpr uint16_t CellVDefaultMv       = 3600;  // assumed when AMS data not yet fresh (no derate)

// ---- IR compensation (what makes the derate tolerant of accelerations) -----
// The derate runs on an ESTIMATED open-circuit voltage, not on the loaded
// reading: v_ocv = v_cell_min + I_pack * R_cell. See cell_derate.hpp for why
// this is the mechanism and filtering alone cannot substitute for it.
//
// *** COMMISSION -- THE ONE VALUE THAT MUST COME FROM THE CAR. ***
// Per-SERIES-ELEMENT resistance in milliohms (one cell, or one parallel group
// if the pack is xSyP), including its share of busbar and contact resistance.
// Shipped low ON PURPOSE: under-compensating leaves some sag in the estimate
// and derates EARLIER, which is the safe direction. Over-compensating hides a
// genuinely empty pack.
//
// Measure it without a dyno: stream pit-diag, do one acceleration run, plot
// est_ocv_mV from 0x709. Still dips under load -> raise it. Humps upward ->
// lower it. Flat -> correct.
// 0 disables compensation entirely (derate on raw loaded voltage, as before).
inline constexpr uint16_t CellIrMilliOhm       = 1;     // COMMISSION: measure on car
// Ceiling on the correction, so a current sensor reading nonsense cannot mask
// an empty pack without limit. 500 A is the EV 2.2.2 cap, so at the shipped
// 1 mOhm this only binds on an implausible reading.
inline constexpr uint16_t CellIrCompMaxMv      = 500;
// Backstop: a RAW loaded cell at/below this derates to the floor regardless of
// what the compensation claims. Sits under the derate floor -- by here the AMS
// is about to open the AIRs and coasting down beats being cut mid-corner.
inline constexpr uint16_t CellVRawFloorMv      = 2400;
// Trim filter on the compensated estimate, as a right-shift: tau ~= (1 <<
// shift) * ControlPeriodMs. 7 -> ~1.3 s, long enough to swallow AMS
// quantisation and a stray frame, far too short to hide a real discharge.
inline constexpr uint8_t  CellVFilterShift     = 7;

// The ramp divides by (knee - floor) and computes (100 - floor_pct) unsigned.
// Both are silent catastrophes if a future edit inverts the thresholds, and the
// whole point of deriving the curve is that the thresholds are now editable.
static_assert(CellVDerateFloorMv < CellVDerateKneeMv,
              "cell derate floor must be below the knee (the ramp divides by their span)");
static_assert(CellVDerateFloorPct <= 100,
              "cell derate floor pct is a percentage");
// The floor must command REAL torque. Expressed against the torque map rather
// than as "> 0", because the map's zero point is DeadbandLowPct, not 0 --
// checking the percentage alone is exactly the mistake this is guarding.
// (The two thermal floors assert > 0 only; they are at 20 % and clear of the
// zero point, but the same reasoning applies if either is ever lowered.)
static_assert(CellVDerateFloorPct > DeadbandLowPct,
              "the cell derate floor must be ABOVE the torque map's zero point "
              "(DeadbandLowPct), or it commands 0 Nm and strands the car");
static_assert(CellVDefaultMv >= CellVDerateKneeMv,
              "the stale-AMS default must sit at/above the knee, i.e. imply no derate");
static_assert(CellVRawFloorMv <= CellVDerateFloorMv,
              "the raw backstop must sit under the derate floor -- above it, it would "
              "pre-empt the ramp and undo the IR compensation it exists to guard");
static_assert(CellVFilterShift < 24,
              "filter shift must leave headroom in the q8 accumulator");

// ---- Motor ------------------------------------------------------------------
// The inverter reports EMachine_Speed_erpm (0x463) -- ELECTRICAL rpm. Mechanical
// shaft rpm = erpm / pole pairs. Powertrain-confirmed 2026-07-03: 10 pole pairs.
inline constexpr int32_t  MotorPolePairs       = 10;

// ---- Inverter command unit map (used by the deferred inverter E2E adapter) -
// torque_units = pct*240/90 - 2400/90  maps 10..100% -> 0..240, then the
// inverter's signed two's-complement convention is applied in the adapter.
// Torque map: Nm = pct*Mul/Div - Bias/Div, built so DeadbandLowPct -> 0 Nm and
// 100% -> 240 Nm (full scale unchanged). Legacy VCU used 240/90 with bias 2400,
// i.e. zero at 10%. Re-based for the 5% deadband: slope 240/(100-5) = 240/95,
// bias 5*240 = 1200. Full scale stays 240 Nm -- only the zero-crossing moved.
// ---- FS-Rules EV 2.2.1 tractive-power envelope (#177) ----------------------
// "The TS power at the outlet of the TSAC must not exceed 80 kW", judged on a
// 500 ms moving average (D 10.4.1). Nothing in the vehicle enforced this before
// -- see power_limit.hpp.
// 76 kW, NOT the 80 kW of the rule. This is DELIBERATE MARGIN, and it is
// lowered here rather than by fudging DrivetrainEffPct because the two mean
// different things -- one is the rule, the other is a physical property of the
// drivetrain, and conflating them would hide the assumption instead of bounding
// it.
//
// WHY. The envelope caps COMMANDED SHAFT torque; EV 2.2.1 is judged on a 500 ms
// moving average at the TSAC OUTLET (D 10.4.1). The bridge between them is
// DrivetrainEffPct, which has never been measured. At 80 kW the design point sat
// at 71.98 kW of shaft power at the knee -- 79.98 kW at the assumed 90 %, i.e.
// 99.97 % of the legal limit, on an unmeasured constant. And there is no relief
// from the averaging window: the capped power is a flat plateau above the knee,
// and a plateau's moving average IS its instantaneous value.
//
// 76 kW gives 5 % headroom, so the envelope stays legal down to eta = 0.855. It
// costs ~5 % of straight-line power and moves the knee from 2864 to 2721 rpm.
// A 5 % power loss is cheap; a disqualification is not.
//
// RAISE THIS BACK TOWARDS 80000 ONLY once the real efficiency is measured -- see
// 0x70D, which now publishes commanded shaft power next to the inverter's own
// AC power measurement and the DC bus, so the number can be read off the car
// rather than assumed.
inline constexpr uint32_t PowerLimitW          = 76000;
// Assumed inverter+motor efficiency. The rule is on ELECTRICAL power at the
// TSAC outlet but we can only cap SHAFT torque, so the shaft budget is the
// electrical limit times efficiency. 90 % is an assumption, not a measurement:
// too high and we exceed the limit, too low and we leave performance unused.
// Confirm against a real 500 ms average measured on the car (0x70D gives the
// commanded shaft power next to the inverter's own AC measurement) and adjust.
inline constexpr uint32_t DrivetrainEffPct     = 90;
// Folded constant: K = P * eta * 60/(2*pi), so T_max[Nm] = K / rpm_mech.
// 60/(2*pi) = 9.5493, carried as 9549/1000. Ordered to stay inside uint32:
// 800 * 90 = 72000, 72000 * 9549 = 6.875e8, well under 4.29e9.
inline constexpr uint32_t PowerCapK =
    (PowerLimitW / 100u) * DrivetrainEffPct * 9549u / 1000u;   // 687528
// Shaft torque at 100 % on the map below. Kept explicit so power_cap_pct can
// tell when the envelope is not binding without re-deriving it.
inline constexpr uint32_t InvTorqueFullScaleNm = 240;

// ---- Motor thermal torque cap (#177) ---------------------------------------
// The inverter reports two motor temperatures on 0x464 as raw bytes with a -50
// offset. See motor_thermal.hpp for why this is a cap rather than a gain and
// why losing the sensors does NOT mean "no limit".
//
// EMRAX 228 MV (10 pole pairs -- consistent with MotorPolePairs above). EMRAX
// rates the WINDING to ~120 degC, so the floor sits at 110: 10 degC of margin,
// deliberately unspent. The cap reaches its floor AT the limit rather than
// starting there -- a limiter that waits has already let the winding arrive.
//
// >>> THE SENSOR CANNOT SEE THE PART THAT ACTUALLY DIES. <<<
// The thermistor is in the STATOR WINDING. What fails permanently is the ROTOR
// MAGNETS (irreversible demagnetisation), and the rotor has no direct cooling
// path, so under sustained load it can run hotter than the winding while 0x464
// reports something comfortable. The 10 degC held back from the winding limit is
// rotor headroom that no reading here will ever show. Do NOT spend it chasing
// lap time.
//
// COMMISSION -- VERIFY THE SENSOR SCALING BEFORE TRUSTING ANY OF THIS. EMRAX
// ships with PT100 or KTY81-210 depending on the order; if the inverter is
// configured for the wrong one then every temperature is wrong and the failure
// is silent. With the motor COLD, temp_motor1_degC on 0x706 must read ambient.
// Thirty seconds, and it validates the entire thermal chain.
inline constexpr int16_t  MotorTempDerateStartDegC = 90;   // begin capping
inline constexpr int16_t  MotorTempLimitDegC       = 110;  // floor reached here
// Heat goes as torque squared, so 20 % torque is ~4 % of the heating -- the
// motor cools under any realistic load while the car can still drive off track.
// A thermal limiter that strands the car has traded one failure for another.
inline constexpr uint8_t  MotorTempFloorPct        = 20;
// No usable sensor. Deliberately NOT 100: an unmonitored motor at full power is
// the failure this whole module exists to prevent, and raw 0 (an untouched
// VehicleState) decodes to -50 degC, so "looks cold" is the DEFAULT at boot.
// Drivable, but not enough to cook anything. Annunciated on 0x706.
inline constexpr uint8_t  MotorTempUnknownCapPct   = 60;
inline constexpr uint8_t  MotorTempDisconnectedRaw = 0xFFu;  // inverter sentinel (would read 205 degC)
inline constexpr int16_t  MotorTempRawOffsetDegC   = -50;    // NX encoding: degC = raw - 50
inline constexpr int16_t  MotorTempMinPlausibleDegC = -40;   // below this the reading is not a temperature
// tau ~= (1 << shift) * ControlPeriodMs = 2.56 s. The sensor is 1 degC/count and
// the ramp is 8 points of cap per degC, so without this a reading dithering
// between two counts steps torque by 8 points at 100 Hz. Temperature is slow
// enough that the lag costs nothing.
inline constexpr uint8_t  MotorTempFilterShift     = 8;

static_assert(MotorTempDerateStartDegC < MotorTempLimitDegC,
              "thermal cap must start below the limit (the ramp divides by their span)");
static_assert(MotorTempFloorPct <= 100 && MotorTempUnknownCapPct <= 100,
              "thermal caps are percentages");
static_assert(MotorTempFloorPct > 0,
              "the floor is a limp-home: a thermal cap that strands the car is not a safe default");
// ---- Accumulator thermal torque cap (#177) ---------------------------------
// Per-module maxima on 0x136/0x137, signed degC (no offset -- unlike the
// inverter's byte encoding). See pack_thermal.hpp.
//
// 50 degC cell limit, from the team (2026-08-02) -- conservative against the
// usual 60 degC Li-ion NMC discharge ceiling. Still worth confirming it sits at
// or below whatever the AMS itself trips on: if the AMS opens first this cap
// never engages and is decorative.
//
// NOTE this engages EARLY in practice. 40 degC is reachable partway into a hot
// endurance run, so expect the car to spend real time capped -- that is the
// intent of a 50 degC ceiling, not a fault. Narrowing the band (start 45) delays
// onset at the cost of a ramp twice as steep.
inline constexpr int16_t  PackTempDerateStartDegC = 40;   // begin capping
inline constexpr int16_t  PackTempLimitDegC       = 50;   // floor reached here
inline constexpr uint8_t  PackTempFloorPct        = 20;
// No usable module. Same reasoning as the motor: an unmonitored pack at full
// power is the failure the cap exists to prevent.
inline constexpr uint8_t  PackTempUnknownCapPct   = 60;
// Plausible band for a module reading. NOTE 0 degC is a REAL pack temperature
// (cold morning), so unlike the motor there is no "implausibly cold" check that
// can catch an uninitialised state -- freshness alone carries that case.
inline constexpr int16_t  PackTempMinPlausibleDegC = -40;
inline constexpr int16_t  PackTempMaxPlausibleDegC = 125;
// An accumulator has minutes of thermal mass, so this can be slower than the
// motor's without losing anything. tau ~= 2.56 s at shift 8.
inline constexpr uint8_t  PackTempFilterShift      = 8;

static_assert(PackTempDerateStartDegC < PackTempLimitDegC,
              "pack cap must start below the limit (the ramp divides by their span)");
static_assert(PackTempFloorPct > 0 && PackTempFloorPct <= 100,
              "the pack floor is a limp-home, never a torque cut");
static_assert(PackTempUnknownCapPct <= 100, "cap is a percentage");
static_assert(PackTempFilterShift < 16,
              "the q16 accumulator needs the shift below its fractional width");

static_assert(MotorTempFilterShift < 16,
              "the q16 filter accumulator needs the shift below its fractional width, "
              "or the per-tick increment truncates to zero and the filter stalls short");

inline constexpr int32_t  InvTorqueMapMul      = 240;
inline constexpr int32_t  InvTorqueMapDiv      = 95;
inline constexpr int32_t  InvTorqueMapBias     = 1200;  // /Div

// ---- Input conditioning ----------------------------------------------------
inline constexpr uint8_t  StartDebounceSamples = 5;     // x ControlPeriodMs (=50 ms)

// ---- GPS (MTK3339 on USART10, 9600 8N1 -- PG11 RX / PG12 TX) ---------------
// The UART runs at 9600 (the module's default and what the bench GPS_TEST
// validated); USART10's baud is set in usart.c / ECU.ioc, NOT here.
inline constexpr uint32_t GpsPollPeriodMs      = 20;    // GpsTask drain cadence
inline constexpr uint32_t GpsTxPeriodMs        = 200;   // 0x508/0x509 cadence (5 Hz)
// RX ring between the USART10 ISR and GpsTask. MUST be a power of two (the ring
// masks instead of dividing). At 9600 baud a 20 ms drain window takes in ~19
// bytes, so 256 is ~13x headroom -- enough to ride out a long task preemption.
inline constexpr uint32_t GpsRxRingSize        = 256;
// PMTK fix-rate command sent at task start (checksum is appended by the task).
// "PMTK220,200" = 200 ms = 5 Hz, matching GpsTxPeriodMs. The MTK3339 defaults to
// 1 Hz, which is too coarse to track a car. Valid range is 100..10000 ms; do NOT
// go below 200 ms without also cutting the sentence set further (at 9600 baud
// RMC+GGA at 10 Hz does not fit in the available bandwidth).
inline constexpr const char* GpsPmtkUpdateRate = "PMTK220,200";

// ---- Freshness / staleness (ms) -------------------------------------------
inline constexpr uint32_t AmsStaleMs           = 200;   // matches the AMS VcuStale window
// 0x135 currents, tracked separately from the AMS block (see VehicleState).
// The frame is 50 ms cyclic; 200 ms is four missed in a row before the IR
// compensation gives up and the derate falls back to raw loaded voltage.
inline constexpr uint32_t AcuCurrentsStaleMs   = 200;
inline constexpr uint32_t AcuDischargeInterlockId = 0x021u;  // AMS fsm_in_start + tsms (#198)
// 0x136/0x137 per-module temperatures, 250 ms cyclic. Four missed in a row
// before the pack cap falls back to its unknown-sensor value. This window is the
// WHOLE fail-safe for the uninitialised case (0 degC is a real pack
// temperature, so no value check can catch it) -- do not widen it casually.
inline constexpr uint32_t AcuTmaxStaleMs       = 1000;
// 0x4A0 AMS_status is 500 ms cyclic; 1500 ms is three missed. Only gates whether
// module_online_mask is trustworthy -- losing it does not fault the pack cap, it
// falls back to using every plausible reading.
inline constexpr uint32_t AmsStatusStaleMs     = 1500;
inline constexpr uint32_t InvStaleMs           = 200;   // inverter feedback considered stale
// 0x466 DCBus_Voltage_V specifically, for the 0x100 heartbeat we publish to the
// AMS. DELIBERATELY GENEROUS: this frame's cycle time is recorded NOWHERE -- not
// in the vendor DBC (it carries no GenMsgCycleTime at all) and not measured on
// the car -- so a tight window would false-trip and republish 0 V during normal
// running. Narrow it once the period is captured; see #198.
inline constexpr uint32_t InvDcBusStaleMs      = 500;

// ---- ECU-held DC-link discharge (#198) -------------------------------------
// See discharge.hpp for the topology, why the ECU can only ever ADD a reason to
// discharge, and why the AMS -- not the ECU -- decides when one is needed.
//
// RELEASE THRESHOLD. 10 V, deliberately far below the 60 V of the FS rule and
// of the AMS's own gate (their DcBusDischargedV = 60).
//
// The cross-repo invariant still holds and in fact holds more strongly. #198
// asks that the AMS gate sit AT OR ABOVE ours so the two never fight over the
// boundary: at 10 vs 60 the AMS is already satisfied long before we release, so
// no AMS change is needed for this.
//
// >>> 10 V MAY BE BELOW WHAT THE INVERTER CAN REPORT. <<<
// The link voltage is not measured by the ECU -- it is relayed from the
// inverter's 0x466, and the W90's nominal DC-link operating range starts around
// 48 V. If the inverter browns out or its measurement pins before the link
// reaches 10 V, dc_bus_valid goes false and the release condition can NEVER be
// satisfied: the hold runs to DischargeTimeoutMs and reports a fault EVERY time,
// instead of completing. At 60 V we sat above that floor; at 10 V we are well
// under it.
//
// This is the measurement nobody has taken yet (#198): open the SDC and watch
// whether 0x466 keeps counting down past 10 V or stops. If it stops, either
// raise this back towards the inverter's floor or give the ECU its own DC-link
// sense -- PF10 (GPIO10) is free and ADC-capable.
//
// The timeout margin also shrinks. The FS rule requires below 60 V within 5 s;
// on a simple RC decay from ~400 V that is about 1.9 time constants, so tau is
// roughly 2.6 s and reaching 10 V takes about 9.6 s. Still inside the 30 s
// timeout, but the margin goes from ~6x to ~3x.
inline constexpr uint16_t DischargeReleaseV     = 10;
// Give up after this long without the link falling -- bleed resistor gone open,
// sense fault, or the coil-interrupt relay not obeying. Holding forever would
// leave a car that never arms with nothing indicating why.
//
// GENEROUS ON PURPOSE: the discharge curve has never been measured (#198 says
// they no longer need it to proceed, so it may stay that way for a while). The
// FS rule requires below 60 V within 5 s, so 30 s is 6x that. Narrow it once
// the real time constant is on a log.
inline constexpr uint32_t DischargeTimeoutMs    = 30000;
// 0x021 is 100 ms cyclic. 500 ms is five missed before we stop believing the
// request -- but note that losing it does NOT abort a discharge in progress:
// the latch releases on OUR measurement, never on the request going away.
inline constexpr uint32_t DischargeReqStaleMs   = 500;
// 0x464 temperatures, tracked separately from the inverter block (see
// VehicleState). Feeds the motor thermal cap, which must fall back to its
// unknown-sensor cap when the TEMPERATURES stop, not when the inverter does.
inline constexpr uint32_t InvTempsStaleMs      = 500;
inline constexpr uint32_t UdvCmdStaleMs        = 100;   // 0x507 accel stream stale -> DV torque 0 (never APPS)
inline constexpr uint32_t UdvR2dStaleMs        = 200;   // 0x510 R2D request considered current

// ---- FDCAN ------------------------------------------------------------------
// Non-overlapping MessageRAM offset for FDCAN2, in words. FDCAN1 keeps offset 0
// and occupies 1 std + 2 ext + 32*4*3 = 387 words of the shared 10 KB SRAMCAN;
// FDCAN2 starts right after so the two instances DON'T overlap -- overlap was
// the #48 TX-dead root cause. CubeMX resets it to 0 on EVERY regen, so it MUST be
// re-applied by hand in MX_FDCAN2_Init (fdcan.c) each time -- NOT regen-stable, and
// App_InitTask does NOT re-apply it. (2026-07-01: a regen silently reset this + the
// FDCAN2 AutoRetransmission=ENABLE; both had to be restored in fdcan.c.)
inline constexpr std::uint32_t Fdcan2MessageRamOffset = 387u;

// ---- CAN IDs the ECU CONSUMES (RX) -----------------------------------------
// Mirror the .def files / inverter DBC. A host test asserts parity with the
// DSL-generated <Msg>_ID so these can't silently drift.
inline constexpr uint32_t AcuOkPrechargeId     = 0x020u;     // AMS precharge-OK
inline constexpr uint32_t AcuVCellMinId        = 0x12Cu;     // AMS min cell voltage
inline constexpr uint32_t AcuVminModuleAId     = 0x131u;     // AMS per-module vmin, modules 0..2
inline constexpr uint32_t AcuVminModuleBId     = 0x132u;     // AMS per-module vmin, modules 3..4
inline constexpr uint32_t AcuVmaxModuleAId     = 0x133u;     // AMS per-module vmax, modules 0..2
inline constexpr uint32_t AcuVmaxModuleBId     = 0x134u;     // AMS per-module vmax, modules 3..4
inline constexpr uint32_t AcuCurrentsId        = 0x135u;     // AMS accu/dcdc currents (deciamps)
inline constexpr uint32_t AcuTmaxModuleAId     = 0x136u;     // AMS per-module tmax, modules 0..2
inline constexpr uint32_t AcuTmaxModuleBId     = 0x137u;     // AMS per-module tmax, modules 3..4 + dcdc stub
inline constexpr uint32_t AmsStatusId          = 0x4A0u;     // AMS FSM status
inline constexpr uint32_t UdvTorqueCmdId       = 0x507u;     // uDV torque command (s32 LE, integer %)
inline constexpr uint32_t UdvR2dRequestId      = 0x510u;     // uDV DV ready-to-drive request
inline constexpr uint32_t InvRxStateId         = 0x461u;     // EMC_TX_STATE_2 (App_State_App)
inline constexpr uint32_t InvRxRpmId           = 0x463u;     // EMC_TX_STATE_4 (EMachine_Speed_erpm, 20-bit signed @ bit44)
inline constexpr uint32_t InvRxTempId          = 0x464u;     // EMC_TX_STATE_5 (board/stage/motor temps, raw -50 = degC)
inline constexpr uint32_t InvRxDcBusId         = 0x466u;     // EMC_TX_STATE_7 (DCBus_Voltage_V)
// The inverter's own view of what it is doing and what it is willing to do.
// All three were arriving already -- FDCAN1 accepts every standard ID into
// FIFO0 -- and were simply never decoded, which left "is the INVERTER limiting
// us?" unanswerable from the car (#177).
inline constexpr uint32_t InvRxCtrlModeId      = 0x465u;     // EMC_TX_STATE_6 (Cmd_Src/Ctrl_Type/Ctrl_Mode/PosFb_Src)
inline constexpr uint32_t InvRxTorqueLimId     = 0x467u;     // EMC_TX_STATE_8 (Torque_Max_Feas + Setpoint_App_D/Q_A)
inline constexpr uint32_t InvRxTorqueEstId     = 0x468u;     // EMC_TX_STATE_9 (Torque_Est_Nm)
// ACBus_Power_W rides on 0x466 (EMC_TX_STATE_7) alongside the DC-bus voltage we
// already decode -- 16-bit signed at frame bit 26, 32.767 W/LSB. The handler
// guarded dlc < 4 and dropped bytes 3-5, so the inverter's OWN measurement of
// its AC output power was arriving and being discarded. It is the cheapest
// route to a real drivetrain efficiency (#177).
inline constexpr int32_t  InvAcPowerScaleMilli = 32767;      // W per LSB * 1000
// 0x463/0x465/0x467/0x468 freshness. The inverter cyclics are fast; 200 ms is
// the same window the rest of the inverter feedback uses.
inline constexpr uint32_t InvFeedbackStaleMs   = 200;
// Mechanical rpm ASSUMED when 0x463 goes stale, for the EV 2.2.1 envelope only.
//
// It must be HIGH, and the reason is not obvious. power_cap_pct(0) returns
// 100 % -- correct, because a stationary car draws no power however much torque
// is commanded. But an uninitialised VehicleState reads 0 too, and so does a
// value frozen at boot. So the ONE fallback that must never be chosen is the
// natural-looking one: falling back to 0 rpm hands out full torque for the rest
// of the run while power_capped correctly reports "not limiting".
//
// Assuming max rpm instead caps at ~54 %, which is legal at any speed this car
// reaches and still drives it off the track. Slow and obvious beats fast and
// unlimited.
inline constexpr int32_t  MotorRpmStaleAssumed  = 5500;  // EMRAX 228 max-ish

// ---- Inverter (NX/EMC) TX setpoints (FDCAN1, standard IDs) -----------------
// IDs / mode words / byte layout / torque map all verified against the original
// VCU (IFS08-CE/VCU pre-jarama). The control core's InvMode values ARE the
// App_State_Req mode words (Off 0x01 / Ready 0x04 / TorqueEnable 0x06).
inline constexpr uint32_t InvTxSetpointModeId    = 0x360u;   // EMC_RX_SETPOINT_1 (App_State_Req @ byte2)
inline constexpr uint8_t  InvTxSetpointModeDlc   = 3u;
// 0x360 byte 2 carries TWO signals (vendor DBC NX0001_STS04_A16):
//   App_State_Req : 16|7@1+  -> bits 0-6 (the InvMode word)
//   Flt_Clear     : 23|1@1+  -> bit 7    (explicit fault-clear request)
// Every InvMode is <= 0x13, so bit 7 was always 0 -- Flt_Clear had never been
// asserted. Used to clear a LATCHED inverter fault (dem_present = 0, condition
// already gone) that the reset mode words alone do not shift (#148).
inline constexpr uint8_t  InvFltClearBit         = 0x80u;
inline constexpr uint32_t InvTxSetpointTorqueId  = 0x362u;   // EMC_RX_SETPOINT_3 (Torque_Nm_Req @ bytes 2-3, s16 LE)
inline constexpr uint8_t  InvTxSetpointTorqueDlc = 4u;
// Torque map (reuses InvTorqueMap* above): pct>=5 -> pct*240/95 - 1200/95
// (10%->0, 100%->240), then NEGATED. The negation is a MECHANICAL constraint of
// the motor (its mounting): forward drive = NEGATIVE Torque_Nm_Req. NOT optional
// and NOT a protocol quirk -- removing it drives the car the wrong way (verified
// against the original VCU + confirmed mechanically).
//
// The inverter takes these setpoints WITHOUT E2E -- bytes 0-1 go out as 0x00,
// matching the original VCU's inverter comms byte-for-byte. (The NX DBC does
// define E2E Profile 1 fields, so if the new inverter turns out to enforce them
// on the bench, an E2E CRC engine can be added -- the original code did not.)

// ---- Bootloader / firmware-info / backup domain ----------------------------
inline constexpr uint8_t  EcuNodeId            = 0x01;        // stm32-can-bootloader multi-node id (ECU=0x01, AMS=0x02, uDV=0x03)

// RTC backup-register allocation. Warm-reset persistent (software/IWDG/pin
// reset survive; a power-cycle does not). Shared with the BL, which reads BKP0R.
inline constexpr uint32_t BkpBootReqReg        = 0;          // BKP0R: BL boot-request magic handshake
inline constexpr uint32_t BkpFaultLatchReg     = 1;          // BKP1R: sticky fault/error latch (error_latch)
inline constexpr uint32_t BkpJumpReasonReg     = 2;          // BKP2R: jump-to-BL reason word

inline constexpr uint32_t BlBootReqMagic       = 0xB00710ADu; // -> BKP0R: BL stays in bootloader on the next reset (== stm32-can-bootloader BL_BOOT_REQ_MAGIC)
inline constexpr uint32_t BlBootTriggerCanId   = 0x002u;      // the CAN frame (on the ACU/shared bus) that asks the app to reboot into the BL
inline constexpr uint8_t  BlBootTriggerDlc     = 4u;
inline constexpr uint8_t  BlBootTriggerPayload[4] = { 0xB0, 0x07, 0xAD, 0x12 };  // 0xB007AD12, big-endian on the wire

// ---- Magics ----------------------------------------------------------------
inline constexpr uint32_t PitDiagEnableMagic   = 0xDEADBEEFu; // 0x7E0 payload that enables the pit-diag stream

}  // namespace ecu::config

#endif  // ECU_CONFIG_HPP_
