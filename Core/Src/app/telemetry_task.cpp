// SPDX-License-Identifier: proprietary

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/can_tx.hpp"
#include "app/io_signals.hpp"
#include "app/telemetry.h"
#include "app/vehicle_service.hpp"

#include "cmsis_os2.h"
#include "nrf24.h"
#include "usart.h"

#include <cstdint>
#include <cstring>

extern "C" {
extern osMessageQueueId_t telemetry_radio_queueHandle;
}

namespace {

constexpr std::uint32_t kPeriodMs = 200u;
constexpr std::uint8_t kRadioMagic = 0xECu;
constexpr std::uint8_t kRadioVersionSnapshot = 0x03u;
constexpr std::uint8_t kRadioKindSnapshot = 6u;
constexpr std::uint8_t kRadioFragmentPayloadSize = 24u;
constexpr std::uint8_t kRadioSnapshotWireSize = 102u;
constexpr std::uint8_t kRadioSnapshotFragmentCount =
    (kRadioSnapshotWireSize + kRadioFragmentPayloadSize - 1u) / kRadioFragmentPayloadSize;

static void put_le16(std::uint8_t* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

static void put_le32(std::uint8_t* p, std::int32_t v) noexcept {
    const auto u = static_cast<std::uint32_t>(v);
    p[0] = static_cast<std::uint8_t>(u);
    p[1] = static_cast<std::uint8_t>(u >> 8);
    p[2] = static_cast<std::uint8_t>(u >> 16);
    p[3] = static_cast<std::uint8_t>(u >> 24);
}

static void put_u32le(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}

static void post_dash(std::uint32_t id, const std::uint8_t* data, std::uint8_t dlc) noexcept {
    ecu::CanFrame f{};
    f.bus = static_cast<std::uint8_t>(ecu::CanBus::Dash);
    f.id = id;
    f.dlc = dlc;
    std::memcpy(f.data, data, dlc);
    ecu::can_tx_post(f);
}

static EcuTelemetryFrame build_frame(const ecu::IoInputs& io,
                                     const ecu::VehicleState& veh,
                                     ecu_control_telemetry_t ctrl,
                                     std::uint16_t seq) noexcept {
    EcuTelemetryFrame frame{};
    auto* p = frame.payload;
    p[0] = veh.inv_state;
    p[1] = ctrl.torque_pct;
    put_le16(&p[2], veh.inv_dc_bus_V);
    put_le16(&p[4], veh.v_cell_min_mV);
    put_le16(&p[6], io.apps1_raw);
    put_le16(&p[8], io.apps2_raw);
    put_le16(&p[10], io.brake_raw);
    p[12] = ctrl.ev_2_3;
    p[13] = ctrl.t11_8_9;
    p[14] = veh.ok_precharge ? 1u : 0u;
    p[15] = io.start_button ? 1u : 0u;
    p[16] = ctrl.state;
    p[17] = veh.inv_error;
    p[18] = veh.last_vconfig_tick != 0u ? 1u : 0u;
    p[19] = ctrl.ok_to_drive;
    p[20] = veh.inv_temp_motor1;
    p[21] = veh.inv_temp_pwrstg;
    p[22] = veh.inv_temp_board;
    p[23] = veh.inv_temp_motor2;
    put_le32(&p[24], veh.inv_rpm);
    p[28] = veh.ams_fsm_state;
    p[29] = ctrl.dv_mode;
    put_le16(&p[30], seq);
    return frame;
}

static ecu_control_telemetry_t control_snapshot() noexcept {
    ecu_control_telemetry_t ctrl{};
    ctrl.state = g_control_telemetry.state;
    ctrl.torque_pct = g_control_telemetry.torque_pct;
    ctrl.ev_2_3 = g_control_telemetry.ev_2_3;
    ctrl.t11_8_9 = g_control_telemetry.t11_8_9;
    ctrl.ok_to_drive = g_control_telemetry.ok_to_drive;
    ctrl.dv_mode = g_control_telemetry.dv_mode;
    return ctrl;
}

#ifdef ECU_TELEMETRY_DUMMY_DATA
static void apply_dummy_dataset(ecu::IoInputs& io,
                                ecu::VehicleState& veh,
                                ecu_control_telemetry_t& ctrl,
                                std::uint16_t seq) noexcept {
    io.apps1_raw = static_cast<std::uint16_t>(2500u + (seq % 300u));
    io.apps2_raw = static_cast<std::uint16_t>(2350u + (seq % 250u));
    io.brake_raw = static_cast<std::uint16_t>(1200u + (seq % 100u));
    io.start_button = (seq & 1u) != 0u;

    ctrl.state = 2u;
    ctrl.torque_pct = static_cast<std::uint8_t>((seq * 3u) % 100u);
    ctrl.ev_2_3 = 1u;
    ctrl.t11_8_9 = 1u;
    ctrl.ok_to_drive = 1u;
    ctrl.dv_mode = 0u;

    veh.inv_state = 4u;
    veh.inv_dc_bus_V = 410u;
    veh.v_cell_min_mV = 3650u;
    veh.ok_precharge = true;
    veh.last_vconfig_tick = 1u;
    veh.inv_temp_motor1 = 72u;
    veh.inv_temp_pwrstg = 68u;
    veh.inv_temp_board = 55u;
    veh.inv_temp_motor2 = 70u;
    veh.inv_rpm = static_cast<std::int32_t>(1000 + seq);
    veh.inv_speed_actual = static_cast<std::int32_t>(1200 + seq);
    veh.inv_current_actual = 42;
    veh.ams_fsm_state = 3u;
    veh.soc = 87u;
    veh.corriente_accu = 23;
    veh.corriente_dcdc = 7;
    veh.temp_dcdc = 38;
    for (std::uint8_t i = 0; i < 5u; ++i) {
        veh.vmin_modulo[i] = static_cast<std::uint16_t>(3600u + i);
        veh.vmax_modulo[i] = static_cast<std::uint16_t>(3700u + i);
        veh.temp_max_modulo[i] = static_cast<std::int16_t>(30 + i);
    }
}
#endif

static void post_dash_frames(const EcuTelemetryFrame& frame,
                             const ecu::VehicleState& veh,
                             std::uint32_t tick_ms) noexcept {
    const auto* p = frame.payload;
    std::uint8_t d[8]{};

    d[0] = p[0]; d[1] = p[1]; d[2] = static_cast<std::uint8_t>((p[12] ? 1u : 0u) |
        (p[13] ? 2u : 0u) | (p[17] ? 4u : 0u));
    d[3] = p[14]; d[4] = p[15]; d[5] = p[16]; d[6] = p[30]; d[7] = p[31];
    post_dash(0x510u, d, 8);

    post_dash(0x511u, &p[6], 6);
    d[0] = p[2]; d[1] = p[3]; d[2] = p[4]; d[3] = p[5]; d[4] = p[17]; d[5] = p[18];
    post_dash(0x512u, d, 6);
    d[0] = p[20]; d[1] = 0; d[2] = p[21]; d[3] = 0; d[4] = p[22]; d[5] = 0;
    post_dash(0x513u, d, 6);
    post_dash(0x514u, &p[24], 4);

    put_le32(d, veh.inv_speed_actual);
    post_dash(0x515u, d, 4);
    put_le32(d, veh.inv_current_actual);
    post_dash(0x516u, d, 4);

    d[0] = p[16]; d[1] = p[28];
    post_dash(0x517u, d, 2);

    d[0] = veh.soc;
    put_le16(&d[1], static_cast<std::uint16_t>(veh.corriente_accu));
    put_le16(&d[3], static_cast<std::uint16_t>(veh.corriente_dcdc));
    put_le16(&d[5], static_cast<std::uint16_t>(veh.temp_dcdc));
    post_dash(0x518u, d, 7);

    std::memset(d, 0, sizeof(d));
    post_dash(0x519u, d, 8);
    post_dash(0x51Au, d, 8);
    put_le32(&d[4], static_cast<std::int32_t>(tick_ms));
    post_dash(0x51Bu, d, 8);

    put_le16(&d[0], veh.vmin_modulo[0]);
    put_le16(&d[2], veh.vmin_modulo[1]);
    put_le16(&d[4], veh.vmin_modulo[2]);
    post_dash(0x51Cu, d, 6);
    put_le16(&d[0], veh.vmin_modulo[3]);
    put_le16(&d[2], veh.vmin_modulo[4]);
    post_dash(0x51Du, d, 4);
    put_le16(&d[0], veh.vmax_modulo[0]);
    put_le16(&d[2], veh.vmax_modulo[1]);
    put_le16(&d[4], veh.vmax_modulo[2]);
    post_dash(0x51Eu, d, 6);
    put_le16(&d[0], veh.vmax_modulo[3]);
    put_le16(&d[2], veh.vmax_modulo[4]);
    post_dash(0x51Fu, d, 4);
    put_le16(&d[0], static_cast<std::uint16_t>(veh.temp_max_modulo[0]));
    put_le16(&d[2], static_cast<std::uint16_t>(veh.temp_max_modulo[1]));
    put_le16(&d[4], static_cast<std::uint16_t>(veh.temp_max_modulo[2]));
    post_dash(0x520u, d, 6);
    put_le16(&d[0], static_cast<std::uint16_t>(veh.temp_max_modulo[3]));
    put_le16(&d[2], static_cast<std::uint16_t>(veh.temp_max_modulo[4]));
    put_le16(&d[4], static_cast<std::uint16_t>(veh.temp_dcdc));
    post_dash(0x521u, d, 6);
}

static void serialize_radio_snapshot(std::uint8_t* data,
                                     const ecu::IoInputs& io,
                                     const ecu::VehicleState& veh,
                                     ecu_control_telemetry_t ctrl,
                                     std::uint16_t seq,
                                     std::uint32_t tick_ms) noexcept {
    std::memset(data, 0, kRadioSnapshotWireSize);

    put_u32le(&data[0], tick_ms);
    put_le16(&data[4], seq);
    data[6] = io.start_button ? 1u : 0u;
    put_le16(&data[7], io.apps1_raw);
    put_le16(&data[9], io.apps2_raw);
    put_le16(&data[11], io.brake_raw);
    put_le16(&data[13], ctrl.torque_pct);
    data[15] = ctrl.ev_2_3;
    data[16] = ctrl.t11_8_9;
    data[17] = ctrl.state;

    data[18] = veh.ok_precharge ? 1u : 0u;
    data[19] = veh.ams_fsm_state;
    put_le16(&data[20], veh.v_cell_min_mV);
    data[22] = veh.soc;
    for (std::uint8_t i = 0; i < 5u; ++i) {
        put_le16(&data[23u + 2u * i], veh.vmin_modulo[i]);
        put_le16(&data[33u + 2u * i], veh.vmax_modulo[i]);
        put_le16(&data[49u + 2u * i], static_cast<std::uint16_t>(veh.temp_max_modulo[i]));
    }
    put_le16(&data[43], static_cast<std::uint16_t>(veh.corriente_accu));
    put_le16(&data[45], static_cast<std::uint16_t>(veh.corriente_dcdc));
    put_le16(&data[47], static_cast<std::uint16_t>(veh.temp_dcdc));

    data[59] = veh.inv_state;
    data[60] = veh.last_vconfig_tick != 0u ? 1u : 0u;
    data[61] = veh.inv_error;
    put_le16(&data[62], veh.inv_dc_bus_V);
    put_le16(&data[64], veh.inv_temp_motor1);
    put_le16(&data[66], veh.inv_temp_pwrstg);
    put_le16(&data[68], veh.inv_temp_board);
    put_le32(&data[70], veh.inv_rpm);
    put_le32(&data[74], veh.inv_speed_actual);
    put_le32(&data[78], veh.inv_current_actual);
}

static void enqueue_radio_snapshot(const ecu::IoInputs& io,
                                   const ecu::VehicleState& veh,
                                   ecu_control_telemetry_t ctrl,
                                   std::uint16_t seq,
                                   std::uint32_t tick_ms) noexcept {
    if (telemetry_radio_queueHandle == NULL) {
        return;
    }

    std::uint8_t snapshot[kRadioSnapshotWireSize]{};
    serialize_radio_snapshot(snapshot, io, veh, ctrl, seq, tick_ms);

    for (std::uint8_t frag = 0; frag < kRadioSnapshotFragmentCount; ++frag) {
        EcuRadioPacket packet{};
        packet.payload[0] = kRadioMagic;
        packet.payload[1] = kRadioVersionSnapshot;
        packet.payload[2] = frag;
        packet.payload[3] = kRadioSnapshotFragmentCount;
        put_le16(&packet.payload[4], seq);
        packet.payload[6] = kRadioKindSnapshot;
        packet.payload[7] = 0u;

        const std::uint16_t start = static_cast<std::uint16_t>(frag) * kRadioFragmentPayloadSize;
        const std::uint16_t remaining = start < kRadioSnapshotWireSize
            ? static_cast<std::uint16_t>(kRadioSnapshotWireSize - start)
            : 0u;
        const std::uint8_t copy_len = remaining < kRadioFragmentPayloadSize
            ? static_cast<std::uint8_t>(remaining)
            : kRadioFragmentPayloadSize;
        if (copy_len != 0u) {
            std::memcpy(&packet.payload[8], &snapshot[start], copy_len);
        }
        (void)osMessageQueuePut(telemetry_radio_queueHandle, &packet, 0u, 0u);
    }
}

}  // namespace

extern "C" void Telemetry_RadioSend(const std::uint8_t payload[ECU_TELEMETRY_PAYLOAD_SIZE]) {
    const HAL_StatusTypeDef status = NRF24_SendPayload(payload, ECU_TELEMETRY_PAYLOAD_SIZE, 10u);
    std::uint8_t nrf_status = 0u;
    std::uint8_t config = 0u;
    std::uint8_t rf_ch = 0u;
    std::uint8_t raw[4] = {0u, 0u, 0u, 0u};
    (void)NRF24_ReadStatusNop(&nrf_status);
    (void)NRF24_ReadRawNop4(raw);
    (void)NRF24_ReadRegister(0x00u, &config, NULL);
    (void)NRF24_ReadRegister(0x05u, &rf_ch, NULL);
    g_radio_diag.last_hal_status = static_cast<std::uint8_t>(status);
    g_radio_diag.last_nrf_status = nrf_status;
    g_radio_diag.config = config;
    g_radio_diag.rf_ch = rf_ch;
    g_radio_diag.raw[0] = raw[0];
    g_radio_diag.raw[1] = raw[1];
    g_radio_diag.raw[2] = raw[2];
    g_radio_diag.raw[3] = raw[3];
    if (status == HAL_OK) {
        ++g_radio_diag.tx_ok;
    } else {
        ++g_radio_diag.tx_fail;
    }
}

extern "C" void ecu_telemetry_task_run(void* argument) {
    (void)argument;
    ecu::IoSignals io;
    std::uint16_t seq = 0;
    std::uint32_t tick = osKernelGetTickCount();

    for (;;) {
        ++g_task_step[ECU_TASK_TELEMETRY];
        ecu::IoInputs inputs{};
        io.read(inputs);
        auto veh = ecu::VehicleService::instance().snapshot();
        auto ctrl = control_snapshot();
        const auto frame_seq = seq++;
        const auto now = osKernelGetTickCount();
#ifdef ECU_TELEMETRY_DUMMY_DATA
        apply_dummy_dataset(inputs, veh, ctrl, frame_seq);
#endif
        const auto frame = build_frame(inputs, veh, ctrl, frame_seq);

        post_dash_frames(frame, veh, now);
        enqueue_radio_snapshot(inputs, veh, ctrl, frame_seq, now);

        tick += kPeriodMs;
        osDelayUntil(tick);
    }
}

extern "C" void ecu_radio_tx_task_run(void* argument) {
    (void)argument;
    EcuRadioPacket packet{};

    // One-shot bench verdict at boot (never pre-scheduler -- see
    // NRF24_DiagnoseSerial). This task is the sole owner of the nRF24 bus, so
    // it's the only safe place to run it. The write-readback restores RF_CH=76,
    // so real sends are unaffected.
    NRF24_DiagnoseSerial(&huart10);

    for (;;) {
        if (osMessageQueueGet(telemetry_radio_queueHandle, &packet, NULL, osWaitForever) == osOK) {
            ++g_task_step[ECU_TASK_RADIO_TX];
            Telemetry_RadioSend(packet.payload);
        }
    }
}
