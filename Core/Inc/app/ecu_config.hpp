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
//   BrakePressedRaw 3000 -> 31.3 bar   EV.2.3 brake+throttle cut
// Those are the inherited IFS06 numbers -- now at least judgeable rather than
// opaque. 4.1 bar to arm and ~31 bar for "brake pressed" are plausible; the
// 25.2 bar DV gate still wants checking against what the EBS actually holds.
inline constexpr uint16_t BrakeSensorFullScaleBar = 40;  // EPT1400 order code: 40 bar (04000)
inline constexpr uint16_t BrakeArmRaw          = 750;   // on-car cal 2026-06-27: brake-to-arm (R2D); released ~580 (noise to ~730), arm just above
inline constexpr uint16_t BrakePressedRaw      = 3000;  // COMMISSION: EV.2.3 "brake pressed"
// DV (#17): the "established" hard-braking limit. The EBS holds HARD braking for
// the autonomous R2D; the ECU verifies it on its own brake sensor before honouring
// a 0x510 R2D request, and streams the binary verdict on 0x505 (same threshold).
inline constexpr uint16_t BrakeDvHardRaw       = 2500;  // COMMISSION: set from the brake cal
// BRING-UP brake stub, controlled by THIS value (no build flag): != 0 makes
// io_signals inject it as brake_raw instead of reading the ADC; 0 = real ADC
// (flight). Set ABOVE BrakeDvHardRaw (2500) to arm the DV R2D and BELOW
// BrakePressedRaw (3000) to dodge the EV.2.3 cut (bench: 2700). MUST be 0 for
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
inline constexpr uint8_t  Ev23SetPct           = 25;    // EV.2.3: brake + torque>this -> latch
inline constexpr uint8_t  Ev23ResetPct         = 5;     // EV.2.3: clears when torque<this (brake released)

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
inline constexpr uint8_t  CellVDerateFloorPct  = 5;     // limp-home, deliberately non-zero
inline constexpr uint16_t CellVDefaultMv       = 3600;  // assumed when AMS data not yet fresh (no derate)

// The ramp divides by (knee - floor) and computes (100 - floor_pct) unsigned.
// Both are silent catastrophes if a future edit inverts the thresholds, and the
// whole point of deriving the curve is that the thresholds are now editable.
static_assert(CellVDerateFloorMv < CellVDerateKneeMv,
              "cell derate floor must be below the knee (the ramp divides by their span)");
static_assert(CellVDerateFloorPct <= 100,
              "cell derate floor pct is a percentage");
static_assert(CellVDefaultMv >= CellVDerateKneeMv,
              "the stale-AMS default must sit at/above the knee, i.e. imply no derate");

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
inline constexpr uint32_t PowerLimitW          = 80000;
// Assumed inverter+motor efficiency. The rule is on ELECTRICAL power at the
// TSAC outlet but we can only cap SHAFT torque, so the shaft budget is the
// electrical limit times efficiency. 90 % is an assumption, not a measurement:
// too high and we exceed the limit, too low and we leave performance unused.
// Confirm against a real 500 ms average from scripts/packlog.py and adjust.
inline constexpr uint32_t DrivetrainEffPct     = 90;
// Folded constant: K = P * eta * 60/(2*pi), so T_max[Nm] = K / rpm_mech.
// 60/(2*pi) = 9.5493, carried as 9549/1000. Ordered to stay inside uint32:
// 800 * 90 = 72000, 72000 * 9549 = 6.875e8, well under 4.29e9.
inline constexpr uint32_t PowerCapK =
    (PowerLimitW / 100u) * DrivetrainEffPct * 9549u / 1000u;   // 687528
// Shaft torque at 100 % on the map below. Kept explicit so power_cap_pct can
// tell when the envelope is not binding without re-deriving it.
inline constexpr uint32_t InvTorqueFullScaleNm = 240;

// ---- Motor thermal limit (for the thermal derate, #177) --------------------
// 80 degC, from the team. The inverter reports motor temperature on 0x464 as a
// raw byte with a -50 offset. NOT yet used to derate -- recorded here so the
// number is not lost, and so the thermal step has its one missing input.
inline constexpr int16_t  MotorTempLimitDegC    = 80;

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
inline constexpr uint32_t InvStaleMs           = 200;   // inverter feedback considered stale
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
