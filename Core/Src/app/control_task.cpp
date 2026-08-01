// SPDX-License-Identifier: proprietary
//
// ControlTask: the one realtime task and sole safety actor (the AMS MainTask
// pattern). Every 10 ms it reads the local IO, snapshots the vehicle service,
// steps the pure controller, drives the outputs (RTDS + status LEDs), emits the
// 0x100 heartbeat in EVERY state, optionally the pit-diag stream, and refreshes
// the IWDG. It is the only IWDG kicker -- if it wedges, the dog fires and the
// bootloader catches the reset.

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/can_tx.hpp"
#include "app/control.hpp"
#include "app/ecu_config.hpp"
#include "app/inverter.hpp"
#include "app/io_signals.hpp"
#include "app/pit_diag.hpp"
#include "app/udv_tx.hpp"
#include "app/vehicle_service.hpp"
#include "app/watchdog.hpp"

#include "can/can_codecs.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "fdcan.h"      // hfdcan1 -- FDCAN1 TX-health debug probe (ECU_DEBUG_INV_BRIDGE)

using namespace ecu;

extern "C" void ecu_control_task_run(void *argument) {
    (void)argument;

    Controller ctrl;
    IoSignals  io;
    // The ACTIVE pedal calibration. Compile-time defaults for now; step 3 of
    // #169 loads it from the bootloader NVM here (validated, falling back to
    // these defaults if the stored record is absent or fails validate_cal()).
    PedalCal   cal{};
    auto&      vs       = VehicleService::instance();
    uint32_t   last_pit = 0;
    uint32_t   last_udv = 0;
    uint32_t   tick     = osKernelGetTickCount();

    for (;;) {
        ++g_task_step[ECU_TASK_CONTROL];
        const uint32_t now = osKernelGetTickCount();

        // --- inputs ---
        IoInputs in{};
        io.read(in);
        const VehicleState veh = vs.snapshot();

        CtrlInputs ci{};
        ci.cal = cal;
        ci.apps1_raw         = in.apps1_raw;
        ci.apps2_raw         = in.apps2_raw;
        ci.brake_raw         = in.brake_raw;
        ci.start_button      = in.start_button;
        // Bench stub (config::StubNoInverter, ecu_config.hpp — folds away when false):
        // no inverter on FDCAN1, so bypass BOTH inverter gates (0x466 vconfig + Ready
        // state) and the FSM walks to Active on its own for the R2D/RTDS sequence +
        // APPS sweep. Ready(4) < fault(10) -> stays in TorqueEnable. DISABLES the
        // inverter handshake -- NEVER true for flight.
        if constexpr (config::StubNoInverter) {
            ci.inv_present       = true;
            ci.inv_vconfig_ready = true;
            ci.inv_state         = config::InvReadyState;
            ci.inv_dc_bus_V      = veh.inv_dc_bus_V;
        } else {
            ci.inv_present       = VehicleService::is_fresh(now, veh.last_inv_tick, config::InvStaleMs);
            ci.inv_vconfig_ready = (veh.last_vconfig_tick != 0u);
            ci.inv_state         = veh.inv_state;
            ci.inv_dc_bus_V      = veh.inv_dc_bus_V;
        }
        // Bench stub (config::StubNoAms): no AMS on the bus (inverter on bench PSUs)
        // -> assume precharge complete + AMS healthy so the FSM can reach Active.
        // WaitInvVdcConfig still gates on the inverter's 0x466 DC-bus report, so it
        // won't arm into a dead bus. DISABLES the AMS safety gate -- NEVER for flight.
        if constexpr (config::StubNoAms) {
            ci.ams_fresh         = true;
            ci.ok_precharge      = true;
            ci.ams_error         = false;
            ci.v_cell_min_mV     = config::CellVDefaultMv;   // healthy default -> no low-cell derate
        } else {
            const bool ams_fresh = VehicleService::is_fresh(now, veh.last_ams_tick, config::AmsStaleMs);
            ci.ams_fresh         = ams_fresh;
            ci.ok_precharge      = veh.ok_precharge && ams_fresh;   // stale AMS -> not ok (fail-safe)
            ci.ams_error         = (veh.ams_fsm_state == config::AmsFsmError) && ams_fresh;
            ci.v_cell_min_mV     = veh.v_cell_min_mV;
        }
        // uDV / autonomous (#17): the DV R2D request (0x510, value AND fresh)
        // and the conditioned 0x507 accel command with its own freshness --
        // stale command => the core commands torque 0 (never APPS fallback).
        ci.dv_r2d_req    = (veh.udv_r2d_request != 0u) &&
                           VehicleService::is_fresh(now, veh.last_udv_r2d_tick, config::UdvR2dStaleMs);
        ci.dv_fresh      = VehicleService::is_fresh(now, veh.last_udv_cmd_tick, config::UdvCmdStaleMs);
        ci.dv_torque_pct = VehicleService::condition_udv_torque(veh.udv_torque_cmd);

        // --- step the pure controller ---
        CtrlOutput out = ctrl.step(ci, now);
        // Bring-up torque cap (config::TorqueCap; 100 = no cap). Clamps the
        // commanded torque for on-stands / freewheel testing -- MUST be 100 for flight.
        if (out.torque_pct > config::TorqueCap) {
            out.torque_pct = config::TorqueCap;
        }
        g_last_torque_pct = out.torque_pct;
        g_last_ctrl_state = static_cast<std::uint8_t>(out.state);
        g_last_apps1_raw     = in.apps1_raw;
        g_last_apps2_raw     = in.apps2_raw;
        g_last_brake_raw     = in.brake_raw;
        g_last_start_button  = in.start_button ? 1u : 0u;
        g_last_ev_2_3        = out.ev_2_3   ? 1u : 0u;
        g_last_t11_8_9       = out.t11_8_9  ? 1u : 0u;

        // --- 0x100 heartbeat: EVERY state, every cycle (the AMS VcuStale contract) ---
        {
            VCU_heartbeat_t hb{};
            hb.dc_bus_voltage = veh.inv_dc_bus_V;
            uint8_t b[VCU_heartbeat_DLC];
            encode_VCU_heartbeat(hb, b);
            CanFrame f{};
            f.bus = static_cast<uint8_t>(CanBus::Acu);
            f.id  = VCU_heartbeat_ID;
            f.dlc = VCU_heartbeat_DLC;
            for (unsigned i = 0; i < VCU_heartbeat_DLC; ++i) f.data[i] = b[i];
            can_tx_post(f);
        }

        // --- uDV autonomous contract (#17), ACU bus, UNGATED (not pit-diag) ---
        // 0x506 motor rpm every cycle (10 ms -- the uDV SLAM/odometry input);
        // 0x504 TS-active + 0x505 brake-over-limit at 100 ms. ts_active is the
        // FSM's own ok_precharge view (stub-consistent); the brake verdict uses
        // the SAME threshold that will gate the DV R2D entry.
        can_tx_post(UdvTx::build_motor_rpm(veh.inv_rpm));
        if (static_cast<uint32_t>(now - last_udv) >= config::UdvTxPeriodMs) {
            last_udv = now;
            can_tx_post(UdvTx::build_ts_active(ci.ok_precharge));
            can_tx_post(UdvTx::build_brake_over_limit(in.brake_raw > ci.cal.brake_dv_hard));
            // 0x511 R2D confirm: repeated at 100 ms (loss-robust) with the DV
            // latch as the value -- uDV keys on byte0 != 0.
            can_tx_post(UdvTx::build_r2d_confirm(out.dv_mode));
        }

        // --- inverter setpoints (FDCAN1), EVERY cycle: PLAIN 0x360 mode + 0x362
        //     torque (bytes 0-1 = 0, no E2E -- byte-for-byte as the IFS07 VCU, which
        //     blasted these continuously regardless of HV). Off pre-R2D, Ready at
        //     WaitInvStandby, TorqueEnable in Active. ---
        const CanFrame sp_mode = Inverter::build_setpoint_mode(out.inv_mode);
        const CanFrame sp_tq   = Inverter::build_setpoint_torque(out.torque_pct);
        can_tx_post(sp_mode);
        // Fault-recovery burst: the reset word alone does not clear a latched
        // inverter fault -- the follow-up Off(0x01) does (manual 9.3, and the
        // IFS07 VCU that recovered without a power cycle sent the same
        // sequence). Non-empty ONLY while inv_state is 10/11, so the healthy
        // path still emits exactly one 0x360 per cycle (#148).
        // Flt_Clear rides the LAST follow word only -> the bit pulses low-low-high
        // each cycle, so an edge-triggered clear sees a fresh rising edge at 100 Hz
        // instead of one stuck-high level.
        for (uint8_t i = 0; i < out.inv_mode_follow_n; ++i) {
            const bool last = (i + 1u == out.inv_mode_follow_n);
            can_tx_post(Inverter::build_setpoint_mode(out.inv_mode_follow[i],
                                                      last && out.inv_flt_clear));
        }
        can_tx_post(sp_tq);
#if defined(ECU_DEBUG_INV_BRIDGE)
        // DEBUG: mirror our OWN setpoints onto FDCAN2 (0x560/0x562). NEVER flight.
        {
            CanFrame d = sp_mode; d.bus = static_cast<uint8_t>(CanBus::Acu); d.id = 0x560u; can_tx_post(d);
            CanFrame e = sp_tq;   e.bus = static_cast<uint8_t>(CanBus::Acu); e.id = 0x562u; can_tx_post(e);
        }
#endif

        // --- outputs: RTDS + status LEDs ---
        HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin,
                          out.rtds_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin,
                          (out.state == CtrlState::AmsError) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(ERR_STATUS_GPIO_Port, ERR_STATUS_Pin,
                          (out.state == CtrlState::AmsError) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // --- pit-diag app-state stream @100 ms (gated by 0x7E0) ---
        if (g_pit_diag_enabled &&
            static_cast<uint32_t>(now - last_pit) >= config::PitDiagStreamMs) {
            last_pit = now;
            can_tx_post(PitDiag::build_status(out, veh, in.start_button));
            can_tx_post(PitDiag::build_dv(out, ci, veh));   // 0x707 DV/autonomy (#109)
            can_tx_post(PitDiag::build_pedals(in, ci.cal));
            can_tx_post(PitDiag::build_inverter(veh, static_cast<std::uint8_t>(out.inv_mode)));
            can_tx_post(PitDiag::build_inverter_temps(veh));
            can_tx_post(PitDiag::build_inv_faults(veh, out, now));  // 0x708 L1/L2 + cmd + 0x461 freshness (#148)
            can_tx_post(PitDiag::build_fwinfo());
            can_tx_post(PitDiag::build_brake(in, ci.cal));
#if defined(ECU_DEBUG_INV_BRIDGE)
            // DEBUG: FDCAN1 (inverter bus) TX health. 0x57F =
            // [TxErrCnt, RxErrCnt, LastErrorCode, flags(b0=busoff,b1=errpassive,b2=warn), activity].
            // LEC 3 = ACK error (no node ACKed our TX -> inverter not receiving / bus issue);
            // LEC 0 or 7 = no error (TX is ACKed -> inverter IS receiving -> rejecting on its E2E).
            {
                FDCAN_ErrorCountersTypeDef ec{};
                FDCAN_ProtocolStatusTypeDef ps{};
                HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
                HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
                CanFrame g{};
                g.bus = static_cast<uint8_t>(CanBus::Acu);
                g.id  = 0x57Fu;
                g.dlc = 5;
                g.data[0] = static_cast<uint8_t>(ec.TxErrorCnt);
                g.data[1] = static_cast<uint8_t>(ec.RxErrorCnt);
                g.data[2] = static_cast<uint8_t>(ps.LastErrorCode);
                g.data[3] = static_cast<uint8_t>((ps.BusOff ? 1u : 0u) |
                                                 (ps.ErrorPassive ? 2u : 0u) |
                                                 (ps.Warning ? 4u : 0u));
                g.data[4] = static_cast<uint8_t>(ps.Activity);
                can_tx_post(g);
            }
#endif
        }

        // --- IWDG (sole kicker) ---
        Watchdog::refresh();

        tick += config::ControlPeriodMs;
        osDelayUntil(tick);
    }
}
