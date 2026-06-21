// SPDX-License-Identifier: proprietary
//
// CanRxTask: drains can_rx_queue (filled by the FDCAN RX ISR) and dispatches
// each frame. Three jobs, in priority order: the bootloader trigger (the
// recovery path home), the pit-diag enable/disable command, and otherwise feed
// the VehicleService. It is the SOLE writer of VehicleService.

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/bootloader.hpp"
#include "app/can_tx.hpp"
#include "app/ecu_config.hpp"
#include "app/pit_diag.hpp"
#include "app/vehicle_service.hpp"

#include "can/can_codecs.hpp"

#include "cmsis_os2.h"
#include "main.h"

using namespace ecu;

extern "C" {
extern osMessageQueueId_t can_rx_queueHandle;
}

extern "C" void ecu_can_rx_task_run(void *argument) {
    (void)argument;

    auto&    vs = VehicleService::instance();
    CanFrame f;

    for (;;) {
        const osStatus_t st =
            osMessageQueueGet(can_rx_queueHandle, &f, NULL, config::CanRxWaitMs);
        // Liveness: bump on EVERY wake -- a frame OR the periodic timeout -- so a
        // healthy-but-idle RX task still advances 0x704's task_ran_mask bit on a
        // quiescent bus. The bit means "scheduled + running", not "saw traffic";
        // a genuine hang stops the wakes too, which is the stall it must catch.
        ++g_task_step[ECU_TASK_CAN_RX];
        if (st != osOK) {
            continue;   // timeout: no frame, just the liveness tick
        }

        // (1) bootloader trigger (0x002 / 0xB007AD12) -> reboot to BL. The
        // recovery path home; never returns.
        if (Bootloader::matches_trigger(f)) {
            Bootloader::request_reboot(JumpReason::ManualRequest);
        }

        // (2) pit-diag enable/disable (0x7E0, magic 0xDEADBEEF) -> set the gate,
        // ack with 0x7E1.
        if (f.bus == static_cast<uint8_t>(CanBus::Acu) &&
            f.id == static_cast<uint32_t>(PitDiag_cmd_ID) && f.dlc >= 4u) {
            const uint32_t magic =
                (static_cast<uint32_t>(f.data[0]) << 24) |
                (static_cast<uint32_t>(f.data[1]) << 16) |
                (static_cast<uint32_t>(f.data[2]) << 8)  |
                 static_cast<uint32_t>(f.data[3]);
            const bool en = (magic == config::PitDiagEnableMagic);
            g_pit_diag_enabled = en ? 1u : 0u;
#if defined(ECU_HIL_STUB_BRAKE)
            // Bring-up: payload byte 5 injects the brake level (brake_raw =
            // byte5 << 4, 0..4080) so R2D can arm with an unpurged brake line.
            if (f.dlc >= 6u) {
                g_hil_force_brake = f.data[5];
            }
#endif
            can_tx_post(PitDiag::build_ack(en));
            continue;
        }

        // (3) vehicle state: inverter (0x461/0x466) + AMS (0x020/0x12C/0x4A0).
        vs.update_from_frame(f);
    }
}
