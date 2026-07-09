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

// ---- Start / ready-to-drive FSM -------------------------------------------
inline constexpr uint32_t PrechargeTimeoutMs   = 10000; // no precharge -> retry
inline constexpr uint32_t R2dSoundMs           = 2000;  // RTDS buzzer duration
inline constexpr uint16_t PrechargeTargetV     = 300;   // inverter DC-bus "precharged" (V)

// Inverter App_State feedback values (EMC_TX_STATE_2 / 0x461, App_State_App).
inline constexpr uint8_t  InvStandbyState      = 3;
inline constexpr uint8_t  InvReadyState        = 4;
inline constexpr uint8_t  InvSoftFaultState    = 10;  // soft fault -> reset with InvMode::Fault (0x13)
inline constexpr uint8_t  InvHardFaultState    = 11;  // hard fault -> recover with InvMode::HardFaultReset (0x0D)

// AMS FSM state (0x4A0 byte0) that means a latched Error (vs a re-armable Start).
inline constexpr uint8_t  AmsFsmError          = 5;

// ---- Pedals / brake (raw 12-bit ADC) --------------------------------------
// APPS travel calibration: pct = clamp((raw - min) * 100 / (max - min), 0, 100).
inline constexpr uint16_t Apps1AdcMin          = 2490;  // bench-cal 2026-06-22 (rest 2476 + margin)
inline constexpr uint16_t Apps1AdcMax          = 3350;  // bench-cal (full 3363 - headroom)
inline constexpr uint16_t Apps2AdcMin          = 2345;  // bench-cal (rest 2332 + margin)
inline constexpr uint16_t Apps2AdcMax          = 3025;  // bench-cal (full 3037 - headroom)

inline constexpr uint16_t BrakeArmRaw          = 900;   // COMMISSION: brake-to-arm (R2D)
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

// ---- Torque / FSAE plausibility -------------------------------------------
inline constexpr uint8_t  AppsAgreementPct     = 8;     // both sensors must exceed to produce torque
inline constexpr uint8_t  DeadbandLowPct       = 10;    // below -> 0
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
// factor = 1.0 at knee, smoothly down to ~0.05 at floor, then flat 0.05 below.
inline constexpr uint16_t CellVDerateKneeMv    = 3500;  // at/above: no derate
inline constexpr uint16_t CellVDerateFloorMv   = 2800;  // below: flat floor factor
inline constexpr double   CellVDerateSlope     = 1.357; // factor = (slope*mV - intercept)/scale
inline constexpr double   CellVDerateIntercept = 3750.0;
inline constexpr double   CellVDerateScale     = 1000.0;
inline constexpr double   CellVDerateFloorFactor = 0.05;
inline constexpr uint16_t CellVDefaultMv       = 3600;  // assumed when AMS data not yet fresh (no derate)

// ---- Motor ------------------------------------------------------------------
// The inverter reports EMachine_Speed_erpm (0x463) -- ELECTRICAL rpm. Mechanical
// shaft rpm = erpm / pole pairs. Powertrain-confirmed 2026-07-03: 10 pole pairs.
inline constexpr int32_t  MotorPolePairs       = 10;

// ---- Inverter command unit map (used by the deferred inverter E2E adapter) -
// torque_units = pct*240/90 - 2400/90  maps 10..100% -> 0..240, then the
// inverter's signed two's-complement convention is applied in the adapter.
inline constexpr int32_t  InvTorqueMapMul      = 240;
inline constexpr int32_t  InvTorqueMapDiv      = 90;
inline constexpr int32_t  InvTorqueMapBias     = 2400;  // /Div

// ---- Input conditioning ----------------------------------------------------
inline constexpr uint8_t  StartDebounceSamples = 5;     // x ControlPeriodMs (=50 ms)

// ---- Freshness / staleness (ms) -------------------------------------------
inline constexpr uint32_t AmsStaleMs           = 200;   // matches the AMS VcuStale window
inline constexpr uint32_t InvStaleMs           = 200;   // inverter feedback considered stale
inline constexpr uint32_t UdvCmdStaleMs        = 100;   // 0x507 accel stream stale -> DV torque 0 (never APPS)
inline constexpr uint32_t UdvR2dStaleMs        = 200;   // 0x510 R2D request considered current

// ---- FDCAN ------------------------------------------------------------------
// Non-overlapping MessageRAM offset for FDCAN2, in words. FDCAN1 keeps offset 0
// and occupies 1 std + 2 ext + 32*4*3 = 387 words of the shared 10 KB SRAMCAN;
// FDCAN2 starts right after so the two instances DON'T overlap -- overlap was
// the #48 TX-dead root cause. CubeMX assigns 0 to every instance and reverts it
// on regen, so App_InitTask re-applies it (regen-stable).
inline constexpr std::uint32_t Fdcan2MessageRamOffset = 387u;

// ---- CAN IDs the ECU CONSUMES (RX) -----------------------------------------
// Mirror the .def files / inverter DBC. A host test asserts parity with the
// DSL-generated <Msg>_ID so these can't silently drift.
inline constexpr uint32_t AcuOkPrechargeId     = 0x020u;     // AMS precharge-OK
inline constexpr uint32_t AcuVCellMinId        = 0x12Cu;     // AMS min cell voltage
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
inline constexpr uint32_t InvTxSetpointTorqueId  = 0x362u;   // EMC_RX_SETPOINT_3 (Torque_Nm_Req @ bytes 2-3, s16 LE)
inline constexpr uint8_t  InvTxSetpointTorqueDlc = 4u;
// Torque map (reuses InvTorqueMap* above): pct>=10 -> pct*240/90 - 2400/90
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
