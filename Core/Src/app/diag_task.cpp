// SPDX-License-Identifier: proprietary
//
// DiagTask: emits the 0x704 firmware-health frame once a second. Runs at low
// priority and -- crucially -- SEPARATELY from ControlTask, so it survives a
// ControlTask stall (the exact #48-class failure it exists to diagnose). The
// 0x704 frame is always-on (ungated) so reset_cause + task liveness are visible
// over CAN immediately on boot, with no UART/SWD.

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/can_tx.hpp"
#include "app/ecu_config.hpp"
#include "app/error_latch.hpp"
#include "app/pit_diag.hpp"
#include "app/reset_cause.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"

#include "FreeRTOS.h"   // xPortGetFreeHeapSize

using namespace ecu;

namespace {

void append_char(char*& out, char* end, char c) {
    if (out < end) *out++ = c;
}

void append_str(char*& out, char* end, const char* s) {
    while (*s != '\0') append_char(out, end, *s++);
}

void append_u32(char*& out, char* end, uint32_t v) {
    char tmp[10];
    unsigned n = 0;
    do {
        tmp[n++] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    while (n != 0u) append_char(out, end, tmp[--n]);
}

void append_hex8(char*& out, char* end, uint8_t v) {
    static constexpr char hex[] = "0123456789ABCDEF";
    append_char(out, end, hex[(v >> 4) & 0x0Fu]);
    append_char(out, end, hex[v & 0x0Fu]);
}

// Plain-text bench UART diagnosis (115200 8N1 on USART10). Task fields are
// per-second step deltas, so a healthy CTRL is ~100, TEL ~5, RTX ~25.
void print_diag_uart(const uint32_t delta[ECU_TASK_COUNT],
                     ecu_control_telemetry_t ctrl,
                     ecu_radio_diag_t rf,
                     uint32_t heap) {
    static const char* const names[ECU_TASK_COUNT] = {
        "CTRL", "CANRX", "CANTX", "DIAG", "TEL", "RTX"
    };
    char line[320];
    char* out = line;
    char* const end = &line[sizeof(line) - 1u];

    append_str(out, end, "TASK ");
#ifdef ECU_TELEMETRY_DUMMY_DATA
    append_str(out, end, "DMY:1 ");
#else
    append_str(out, end, "DMY:0 ");
#endif
    for (int t = 0; t < ECU_TASK_COUNT; ++t) {
        append_str(out, end, names[t]);
        append_char(out, end, ':');
        append_u32(out, end, delta[t]);
        append_char(out, end, ' ');
    }
    append_str(out, end, "CTL st:");
    append_u32(out, end, ctrl.state);
    append_str(out, end, " tq:");
    append_u32(out, end, ctrl.torque_pct);
    append_str(out, end, " ok:");
    append_u32(out, end, ctrl.ok_to_drive);
    append_str(out, end, " ev:");
    append_u32(out, end, ctrl.ev_2_3);
    append_str(out, end, " t11:");
    append_u32(out, end, ctrl.t11_8_9);
    append_str(out, end, " dv:");
    append_u32(out, end, ctrl.dv_mode);
    append_str(out, end, " RF ok:");
    append_u32(out, end, rf.tx_ok);
    append_str(out, end, " fail:");
    append_u32(out, end, rf.tx_fail);
    append_str(out, end, " hal:");
    append_u32(out, end, rf.last_hal_status);
    append_str(out, end, " nrf:0x");
    append_hex8(out, end, rf.last_nrf_status);
    append_str(out, end, " cfg:0x");
    append_hex8(out, end, rf.config);
    append_str(out, end, " ch:");
    append_u32(out, end, rf.rf_ch);
    append_str(out, end, " raw:");
    append_hex8(out, end, rf.raw[0]);
    append_char(out, end, ',');
    append_hex8(out, end, rf.raw[1]);
    append_char(out, end, ',');
    append_hex8(out, end, rf.raw[2]);
    append_char(out, end, ',');
    append_hex8(out, end, rf.raw[3]);
    append_str(out, end, " heap:");
    append_u32(out, end, heap);
    append_str(out, end, "\r\n");

    (void)HAL_UART_Transmit(&huart10, reinterpret_cast<uint8_t*>(line),
                            static_cast<uint16_t>(out - line), 50u);
}

}  // namespace

extern "C" void ecu_diag_task_run(void *argument) {
    (void)argument;

    // Captured once: the cause of THIS boot, and the fault latched in backup RAM
    // before the previous reset (0 if a clean boot). Both reported every frame.
    const ResetCause boot_cause = ResetInfo::read_and_clear();
    const FaultCode  last_fault = FaultLatch::get();
    const uint32_t   boot_tick  = osKernelGetTickCount();

    uint32_t min_heap            = 0xFFFFFFFFu;
    uint32_t prev_step[ECU_TASK_COUNT] = { 0, 0, 0, 0 };
    uint32_t tick                = osKernelGetTickCount();

    for (;;) {
        ++g_task_step[ECU_TASK_DIAG];

        const uint32_t heap = static_cast<uint32_t>(xPortGetFreeHeapSize());
        if (heap < min_heap) min_heap = heap;

        // Per-task liveness: bit set = the task's step counter advanced since the
        // last health frame. A cyclic task whose bit stays clear has stalled.
        uint8_t mask = 0;
        uint32_t delta[ECU_TASK_COUNT] = {};
        for (int i = 0; i < ECU_TASK_COUNT; ++i) {
            const uint32_t step = g_task_step[i];
            delta[i] = step - prev_step[i];
            if (delta[i] != 0u) mask |= static_cast<uint8_t>(1u << i);
            prev_step[i] = step;
        }
        const ecu_control_telemetry_t ctrl = {
            g_control_telemetry.state,
            g_control_telemetry.torque_pct,
            g_control_telemetry.ev_2_3,
            g_control_telemetry.t11_8_9,
            g_control_telemetry.ok_to_drive,
            g_control_telemetry.dv_mode
        };
        const ecu_radio_diag_t rf = {
            g_radio_diag.tx_ok,
            g_radio_diag.tx_fail,
            g_radio_diag.last_hal_status,
            g_radio_diag.last_nrf_status,
            g_radio_diag.config,
            g_radio_diag.rf_ch,
            {
                g_radio_diag.raw[0],
                g_radio_diag.raw[1],
                g_radio_diag.raw[2],
                g_radio_diag.raw[3]
            }
        };
        print_diag_uart(delta, ctrl, rf, heap);

        HealthMetrics m{};
        m.free_heap     = static_cast<uint16_t>(heap     > 0xFFFFu ? 0xFFFFu : heap);
        m.min_free_heap = static_cast<uint16_t>(min_heap > 0xFFFFu ? 0xFFFFu : min_heap);
        m.task_ran_mask = mask;
        m.reset_cause   = boot_cause;
        m.uptime_s      = static_cast<uint8_t>(((osKernelGetTickCount() - boot_tick) / 1000u) & 0xFFu);
        m.last_fault    = last_fault;
        can_tx_post(PitDiag::build_health(m));

        tick += config::DiagPeriodMs;
        osDelayUntil(tick);
    }
}
