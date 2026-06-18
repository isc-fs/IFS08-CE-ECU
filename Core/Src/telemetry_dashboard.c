#include "telemetry_dashboard.h"

#include "can.h"

#include <string.h>

static void dashboard_push_u16le(uint16_t value, uint8_t out[2])
{
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void dashboard_push_u32le(uint32_t value, uint8_t out[4])
{
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
  out[2] = (uint8_t)((value >> 16) & 0xFFu);
  out[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint8_t dashboard_publish_slow_now(const telemetry_frame_t *frame)
{
  if (!frame) return 0u;
  return (uint8_t)((frame->sequence % 5u) == 0u);
}

static void dashboard_send_msg(const can_msg_t *msg)
{
  if (!msg) return;
  (void)CanTx_SendHal(msg);
}

void Telemetry_DashboardPublishFrame(const telemetry_frame_t *frame)
{
  can_msg_t msg;
  uint8_t fault_bits = 0u;
  uint8_t publish_slow;

  if (!frame) return;
  publish_slow = dashboard_publish_slow_now(frame);

  if (frame->snapshot.ecu.flag_ev_2_3)  fault_bits |= 0x01u;
  if (frame->snapshot.ecu.flag_t11_8_9) fault_bits |= 0x02u;
  if (frame->snapshot.inverter.inv_error) fault_bits |= 0x04u;

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x510u;
  msg.dlc = 8u;
  msg.data[0] = frame->snapshot.inverter.inv_state;
  msg.data[1] = (uint8_t)frame->snapshot.ecu.torque_total;
  msg.data[2] = fault_bits;
  msg.data[3] = frame->snapshot.ams.ok_precarga;
  msg.data[4] = frame->snapshot.ecu.boton_arranque;
  msg.data[5] = (uint8_t)frame->kind;
  msg.data[6] = (uint8_t)(frame->sequence & 0xFFu);
  msg.data[7] = (uint8_t)((frame->sequence >> 8) & 0xFFu);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x511u;
  msg.dlc = 6u;
  dashboard_push_u16le(frame->snapshot.ecu.s1_aceleracion, &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.ecu.s2_aceleracion, &msg.data[2]);
  dashboard_push_u16le(frame->snapshot.ecu.s_freno, &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x512u;
  msg.dlc = 6u;
  dashboard_push_u16le(frame->snapshot.inverter.inv_dc_bus_voltage, &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.ams.v_celda_min, &msg.data[2]);
  msg.data[4] = frame->snapshot.inverter.inv_error;
  msg.data[5] = frame->snapshot.inverter.inv_vdc_ready;
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x513u;
  msg.dlc = 6u;
  dashboard_push_u16le((uint16_t)frame->snapshot.inverter.inv_motor_temp, &msg.data[0]);
  dashboard_push_u16le((uint16_t)frame->snapshot.inverter.inv_igbt_temp, &msg.data[2]);
  dashboard_push_u16le((uint16_t)frame->snapshot.inverter.inv_air_temp, &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x514u;
  msg.dlc = 4u;
  dashboard_push_u32le((uint32_t)frame->snapshot.inverter.inv_rpm, &msg.data[0]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x515u;
  msg.dlc = 4u;
  dashboard_push_u32le((uint32_t)frame->snapshot.inverter.inv_speed_actual, &msg.data[0]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x516u;
  msg.dlc = 4u;
  dashboard_push_u32le((uint32_t)frame->snapshot.inverter.inv_current_actual, &msg.data[0]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x517u;
  msg.dlc = 2u;
  msg.data[0] = frame->snapshot.ecu.fsm_state;
  msg.data[1] = frame->snapshot.ams.ams_state;
  dashboard_send_msg(&msg);

  if (!publish_slow)
  {
    return;
  }

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x518u;
  msg.dlc = 7u;
  msg.data[0] = frame->snapshot.ams.soc;
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.corriente_accu, &msg.data[1]);
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.corriente_dcdc, &msg.data[3]);
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_dcdc, &msg.data[5]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x519u;
  msg.dlc = 8u;
  dashboard_push_u16le(frame->snapshot.gps.speed, &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.gps.course_deg, &msg.data[2]);
  dashboard_push_u32le((uint32_t)frame->snapshot.gps.altitude, &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x51Au;
  msg.dlc = 8u;
  msg.data[0] = frame->snapshot.gps.fix_type;
  msg.data[1] = frame->snapshot.gps.sat_count;
  dashboard_push_u16le(frame->snapshot.gps.hdop, &msg.data[2]);
  dashboard_push_u32le((uint32_t)frame->snapshot.gps.latitude, &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x51Bu;
  msg.dlc = 8u;
  dashboard_push_u32le((uint32_t)frame->snapshot.gps.longitude, &msg.data[0]);
  dashboard_push_u32le(frame->snapshot.tick_ms, &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x51Cu;
  msg.dlc = 6u;
  dashboard_push_u16le(frame->snapshot.ams.vmin_modulo[0], &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.ams.vmin_modulo[1], &msg.data[2]);
  dashboard_push_u16le(frame->snapshot.ams.vmin_modulo[2], &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x51Du;
  msg.dlc = 4u;
  dashboard_push_u16le(frame->snapshot.ams.vmin_modulo[3], &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.ams.vmin_modulo[4], &msg.data[2]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x51Eu;
  msg.dlc = 6u;
  dashboard_push_u16le(frame->snapshot.ams.vmax_modulo[0], &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.ams.vmax_modulo[1], &msg.data[2]);
  dashboard_push_u16le(frame->snapshot.ams.vmax_modulo[2], &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x51Fu;
  msg.dlc = 4u;
  dashboard_push_u16le(frame->snapshot.ams.vmax_modulo[3], &msg.data[0]);
  dashboard_push_u16le(frame->snapshot.ams.vmax_modulo[4], &msg.data[2]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x520u;
  msg.dlc = 6u;
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_max_modulo[0], &msg.data[0]);
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_max_modulo[1], &msg.data[2]);
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_max_modulo[2], &msg.data[4]);
  dashboard_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x521u;
  msg.dlc = 6u;
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_max_modulo[3], &msg.data[0]);
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_max_modulo[4], &msg.data[2]);
  dashboard_push_u16le((uint16_t)frame->snapshot.ams.temp_dcdc, &msg.data[4]);
  dashboard_send_msg(&msg);
}
