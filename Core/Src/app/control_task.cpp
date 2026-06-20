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
#include "app/io_signals.hpp"
#include "app/pit_diag.hpp"
#include "app/vehicle_service.hpp"
#include "app/watchdog.hpp"

#include "can/can_codecs.hpp"

#include "cmsis_os2.h"
#include "main.h"

using namespace ecu;

extern "C" void ecu_control_task_run(void *argument) {
    (void)argument;

    Controller ctrl;
    IoSignals  io;
    auto&      vs       = VehicleService::instance();
    uint32_t   last_pit = 0;
    uint32_t   tick     = osKernelGetTickCount();

    for (;;) {
        ++g_task_step[ECU_TASK_CONTROL];
        const uint32_t now = osKernelGetTickCount();

        // --- inputs ---
        IoInputs in{};
        io.read(in);
        const VehicleState veh = vs.snapshot();

        CtrlInputs ci{};
        ci.apps1_raw         = in.apps1_raw;
        ci.apps2_raw         = in.apps2_raw;
        ci.brake_raw         = in.brake_raw;
        ci.start_button      = in.start_button;
        ci.inv_present       = VehicleService::is_fresh(now, veh.last_inv_tick, config::InvStaleMs);
        ci.inv_vconfig_ready = (veh.last_vconfig_tick != 0u);
        ci.inv_state         = veh.inv_state;
        ci.inv_dc_bus_V      = veh.inv_dc_bus_V;
        const bool ams_fresh = VehicleService::is_fresh(now, veh.last_ams_tick, config::AmsStaleMs);
        ci.ams_fresh         = ams_fresh;
        ci.ok_precharge      = veh.ok_precharge && ams_fresh;   // stale AMS -> not ok (fail-safe)
        ci.ams_error         = (veh.ams_fsm_state == config::AmsFsmError) && ams_fresh;
        ci.v_cell_min_mV     = veh.v_cell_min_mV;

        // --- step the pure controller ---
        const CtrlOutput out = ctrl.step(ci, now);

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

        // TODO(#10): inverter command TX (0x360 mode <- out.inv_mode, 0x362
        // torque <- out.torque_pct) is AUTOSAR E2E Profile-1 protected -> lands
        // with the inverter adapter. The values are computed and ready here.

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
            can_tx_post(PitDiag::build_pedals(in));
            can_tx_post(PitDiag::build_inverter(veh));
            can_tx_post(PitDiag::build_fwinfo());
            can_tx_post(PitDiag::build_brake(in));
        }

        // --- IWDG (sole kicker) ---
        Watchdog::refresh();

        tick += config::ControlPeriodMs;
        osDelayUntil(tick);
    }
}
