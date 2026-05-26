#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include "cmsis_os2.h"

/* Application-wide shared inputs/state (protected by g_inMutex). */
typedef struct
{
  /* Sensors / inputs */
  uint16_t s1_aceleracion;   /* ADC raw */
  uint16_t s2_aceleracion;   /* ADC raw */
  uint16_t s_freno;          /* ADC raw */
  uint8_t  boton_arranque;   /* 0/1 */

  /* Inverter feedback */
  uint8_t  inv_state;        /* e.g. standby/ready/fault (project-specific) */
  uint16_t inv_dc_bus_voltage; /* raw / scaled, project-specific */
  uint8_t  inv_vdc_ready;    /* legacy TX_STATE_7 (0x466) already observed */
  int16_t  inv_motor_temp;
  int16_t  inv_igbt_temp;
  int16_t  inv_air_temp;
  int32_t  inv_rpm;
  int32_t  inv_speed_actual;
  int32_t  inv_current_actual;
  uint8_t  inv_error;

  /* Battery / misc */
  uint16_t v_celda_min;      /* raw / scaled */
  uint8_t  ok_precarga;      /* ack from ACU, etc. */

  /* Battery expansion */
  uint8_t  soc;              /* 0..100 or project-specific scaling */
  uint16_t vmin_modulo[5];
  uint16_t vmax_modulo[5];
  int16_t  corriente_accu;
  int16_t  corriente_dcdc;
  uint16_t temp_dcdc;
  uint16_t temp_max_modulo[5];

  /* IMU */
  int16_t  imu_accel_xyz[3];
  int16_t  imu_gyro_xyz[3];
  int16_t  imu_roll_deg;
  int16_t  imu_pitch_deg;
  int16_t  imu_yaw_deg;

  /* GPS */
  uint16_t gps_speed;
  uint16_t gps_course_deg;
  int32_t  gps_altitude;
  uint8_t  gps_fix_type;
  uint8_t  gps_sat_count;
  uint16_t gps_hdop;
  int32_t  gps_latitude;
  int32_t  gps_longitude;

  /* Safety / plausibility flags */
  uint8_t flag_EV_2_3;
  uint8_t flag_T11_8_9;

  /* Derived */
  uint16_t torque_total;     /* 0..100% */

} app_inputs_t;

extern app_inputs_t g_in;
extern osMutexId_t  g_inMutex;

void AppState_Init(void);
void AppState_Snapshot(app_inputs_t *out);

#endif /* APP_STATE_H */
