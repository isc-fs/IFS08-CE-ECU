/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "diag.h"
#include "telemetry.h"
#include "test_integration.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_state.h"
#include "app_runtime.h"
#include "bootloader.h"
#include "pit_diag.h"
#include "firmware_info.h"
#include "control.h"
#include "io_signals.h"
#ifndef SIL_BUILD
#include "i2c.h"
#endif
#include <stdio.h>
#include <string.h>
#ifndef SIL_BUILD
extern HAL_StatusTypeDef FDCAN_RuntimeBringUp(void);
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct
{
  uint32_t generation;
  uint32_t next_release;
} app_periodic_state_t;

typedef struct
{
  const char *name;
  uint32_t start_count;
  uint32_t step_count;
  uint32_t last_start_tick;
  uint32_t last_step_tick;
} app_task_metrics_internal_t;

static uint32_t s_periodic_generation;
static uint8_t s_telemetry_radio_divider;
static uint8_t s_telemetry_sd_divider;
static app_task_metrics_internal_t s_task_metrics[APP_TASK_ID_COUNT] = {
  { "defaultTask", 0u, 0u, 0u, 0u },
  { "App_InitTask", 0u, 0u, 0u, 0u },
  { "ControlTask", 0u, 0u, 0u, 0u },
  { "CanRxTask", 0u, 0u, 0u, 0u },
  { "CanTxTask", 0u, 0u, 0u, 0u },
  { "TelemetryTask", 0u, 0u, 0u, 0u },
  { "DiagTask", 0u, 0u, 0u, 0u },
  { "RadioTxTask", 0u, 0u, 0u, 0u },
  { "SdLogTask", 0u, 0u, 0u, 0u },
  { "DashTask", 0u, 0u, 0u, 0u }
};

#ifndef SIL_BUILD
#define BMI088_ACC_I2C_ADDR_LO   (0x18u << 1)
#define BMI088_ACC_I2C_ADDR_HI   (0x19u << 1)
#define BMI088_GYRO_I2C_ADDR_LO  (0x68u << 1)
#define BMI088_GYRO_I2C_ADDR_HI  (0x69u << 1)
#define BMI088_REG_CHIP_ID       0x00u
#define BMI088_ACC_CHIP_ID       0x1Eu
#define BMI088_GYRO_CHIP_ID      0x0Fu
#define BMI088_ACC_REG_DATA      0x12u
#define BMI088_GYRO_REG_DATA     0x02u
#define BMI088_ACC_REG_CONF      0x40u
#define BMI088_ACC_REG_RANGE     0x41u
#define BMI088_ACC_REG_PWR_CONF  0x7Cu
#define BMI088_ACC_REG_PWR_CTRL  0x7Du
#define BMI088_ACC_PWR_ACTIVE    0x00u
#define BMI088_ACC_ENABLE        0x04u
#define BMI088_ACC_CONF_100HZ    0xA8u
#define BMI088_ACC_RANGE_6G      0x01u

typedef struct
{
  uint16_t acc_addr;
  uint16_t gyro_addr;
  uint8_t acc_id;
  uint8_t gyro_id;
  uint8_t acc_found;
  uint8_t gyro_found;
  uint8_t acc_initialized;
  HAL_StatusTypeDef acc_status;
  HAL_StatusTypeDef gyro_status;
} app_imu_probe_state_t;

static app_imu_probe_state_t s_imu_probe = {0u};
#endif

static uint8_t app_task_started(app_task_id_t task_id)
{
  if ((uint32_t)task_id >= (uint32_t)APP_TASK_ID_COUNT)
  {
    return 0u;
  }

  if (s_task_metrics[task_id].start_count == 0u)
  {
    s_task_metrics[task_id].start_count = 1u;
    s_task_metrics[task_id].last_start_tick = osKernelGetTickCount();
    return 1u;
  }

  return 0u;
}

static void app_task_stepped(app_task_id_t task_id)
{
  if ((uint32_t)task_id >= (uint32_t)APP_TASK_ID_COUNT)
  {
    return;
  }

  s_task_metrics[task_id].step_count++;
  s_task_metrics[task_id].last_step_tick = osKernelGetTickCount();
}

static void app_task_metrics_log_summary(void)
{
  uint32_t task_index;

  for (task_index = 0u; task_index < (uint32_t)APP_TASK_ID_COUNT; task_index++)
  {
    const app_task_metrics_internal_t *metrics = &s_task_metrics[task_index];
    Diag_Log("RTOSCHK: %-13s start=%lu steps=%lu lastStart=%lu lastStep=%lu",
             metrics->name,
             (unsigned long)metrics->start_count,
             (unsigned long)metrics->step_count,
             (unsigned long)metrics->last_start_tick,
             (unsigned long)metrics->last_step_tick);
  }
}

#ifndef SIL_BUILD
static HAL_StatusTypeDef app_imu_read_reg(uint16_t dev_addr, uint8_t reg, uint8_t *value)
{
  if (!value)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(&hi2c2,
                          dev_addr,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1u,
                          20u);
}

static HAL_StatusTypeDef app_imu_write_reg(uint16_t dev_addr, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2,
                           dev_addr,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1u,
                           20u);
}

static void app_imu_probe_one(uint16_t cached_addr,
                              uint16_t addr0,
                              uint16_t addr1,
                              uint8_t expected_id,
                              uint16_t *resolved_addr,
                              uint8_t *resolved_id,
                              uint8_t *found,
                              HAL_StatusTypeDef *status)
{
  uint8_t chip_id = 0u;
  HAL_StatusTypeDef read_status = HAL_ERROR;
  uint16_t candidates[2];
  uint32_t i;

  if (!resolved_addr || !resolved_id || !found || !status)
  {
    return;
  }

  candidates[0] = (cached_addr != 0u) ? cached_addr : addr0;
  candidates[1] = (cached_addr != 0u && cached_addr != addr0) ? addr0 : addr1;

  *found = 0u;
  *resolved_id = 0u;
  *resolved_addr = 0u;
  *status = HAL_ERROR;

  for (i = 0u; i < 2u; i++)
  {
    chip_id = 0u;
    read_status = app_imu_read_reg(candidates[i], BMI088_REG_CHIP_ID, &chip_id);
    if (read_status == HAL_OK && chip_id == expected_id)
    {
      *found = 1u;
      *resolved_addr = candidates[i];
      *resolved_id = chip_id;
      *status = HAL_OK;
      return;
    }

    *status = read_status;
    *resolved_id = chip_id;
  }
}

static void app_imu_probe_bmi088(void)
{
  app_imu_probe_one(s_imu_probe.acc_addr,
                    BMI088_ACC_I2C_ADDR_LO,
                    BMI088_ACC_I2C_ADDR_HI,
                    BMI088_ACC_CHIP_ID,
                    &s_imu_probe.acc_addr,
                    &s_imu_probe.acc_id,
                    &s_imu_probe.acc_found,
                    &s_imu_probe.acc_status);

  app_imu_probe_one(s_imu_probe.gyro_addr,
                    BMI088_GYRO_I2C_ADDR_LO,
                    BMI088_GYRO_I2C_ADDR_HI,
                    BMI088_GYRO_CHIP_ID,
                    &s_imu_probe.gyro_addr,
                    &s_imu_probe.gyro_id,
                    &s_imu_probe.gyro_found,
                    &s_imu_probe.gyro_status);
}

static HAL_StatusTypeDef app_imu_init_accelerometer(void)
{
  HAL_StatusTypeDef status;

  if (!s_imu_probe.acc_found || s_imu_probe.acc_addr == 0u)
  {
    return HAL_ERROR;
  }

  status = app_imu_write_reg(s_imu_probe.acc_addr, BMI088_ACC_REG_PWR_CONF, BMI088_ACC_PWR_ACTIVE);
  if (status != HAL_OK)
  {
    return status;
  }

  osDelay(2u);

  status = app_imu_write_reg(s_imu_probe.acc_addr, BMI088_ACC_REG_PWR_CTRL, BMI088_ACC_ENABLE);
  if (status != HAL_OK)
  {
    return status;
  }

  osDelay(2u);

  status = app_imu_write_reg(s_imu_probe.acc_addr, BMI088_ACC_REG_CONF, BMI088_ACC_CONF_100HZ);
  if (status != HAL_OK)
  {
    return status;
  }

  osDelay(1u);

  status = app_imu_write_reg(s_imu_probe.acc_addr, BMI088_ACC_REG_RANGE, BMI088_ACC_RANGE_6G);
  if (status == HAL_OK)
  {
    s_imu_probe.acc_initialized = 1u;
  }

  osDelay(1u);
  return status;
}

static void app_diag_log_imu_probe(void)
{
  app_imu_probe_bmi088();
  if (s_imu_probe.acc_found && !s_imu_probe.acc_initialized)
  {
    (void)app_imu_init_accelerometer();
  }
  Diag_Log("IMUDBG: acc found=%u addr=0x%02X id=0x%02X st=%d gyro found=%u addr=0x%02X id=0x%02X st=%d",
           (unsigned)s_imu_probe.acc_found,
           (unsigned)(s_imu_probe.acc_addr >> 1),
           (unsigned)s_imu_probe.acc_id,
           (int)s_imu_probe.acc_status,
           (unsigned)s_imu_probe.gyro_found,
           (unsigned)(s_imu_probe.gyro_addr >> 1),
           (unsigned)s_imu_probe.gyro_id,
           (int)s_imu_probe.gyro_status);
}

static void app_diag_log_i2c_scan(void)
{
  char buf[160];
  size_t len = 0u;
  uint16_t addr7;
  uint8_t found = 0u;

  len += (size_t)snprintf(buf + len, sizeof(buf) - len, "I2CSCAN:");
  for (addr7 = 0x08u; addr7 <= 0x77u && len < (sizeof(buf) - 8u); addr7++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr7 << 1), 1u, 5u) == HAL_OK)
    {
      len += (size_t)snprintf(buf + len, sizeof(buf) - len, " 0x%02X", (unsigned)addr7);
      found = 1u;
    }
  }

  if (!found)
  {
    (void)snprintf(buf + len, sizeof(buf) - len, " none");
  }

  Diag_Log("%s", buf);
}
#else
static void app_diag_log_imu_probe(void)
{
}

static void app_diag_log_i2c_scan(void)
{
}
#endif

static void app_diag_log_imu_snapshot(void)
{
  app_inputs_t snapshot;

  AppState_Snapshot(&snapshot);
  Diag_Log("IMU: acc=[%d,%d,%d] gyro=[%d,%d,%d] rpy=[%d,%d,%d]",
           (int)snapshot.imu_accel_xyz[0],
           (int)snapshot.imu_accel_xyz[1],
           (int)snapshot.imu_accel_xyz[2],
           (int)snapshot.imu_gyro_xyz[0],
           (int)snapshot.imu_gyro_xyz[1],
           (int)snapshot.imu_gyro_xyz[2],
           (int)snapshot.imu_roll_deg,
           (int)snapshot.imu_pitch_deg,
           (int)snapshot.imu_yaw_deg);
}

static void app_dash_publish_frame(const telemetry_frame_t *frame);

static uint32_t app_ms_to_ticks(uint32_t ms)
{
  uint32_t tick_freq = osKernelGetTickFreq();
  uint32_t ticks = (ms * tick_freq + 999u) / 1000u;
  return (ticks == 0u) ? 1u : ticks;
}

static void app_periodic_delay_until(app_periodic_state_t *state, uint32_t period_ms)
{
  uint32_t period_ticks = app_ms_to_ticks(period_ms);

  if (!state)
  {
    (void)osDelayUntil(osKernelGetTickCount() + period_ticks);
    return;
  }

  if (state->generation != s_periodic_generation)
  {
    state->generation = s_periodic_generation;
    state->next_release = osKernelGetTickCount();
  }

  state->next_release += period_ticks;
  (void)osDelayUntil(state->next_release);
}

static uint8_t app_tick_reached(uint32_t now, uint32_t deadline)
{
  return (uint8_t)(((int32_t)(now - deadline)) >= 0);
}

static void app_runtime_process_can_rx_item(const can_qitem16_t *rx_qitem)
{
  can_msg_t rx_msg;
  uint8_t previous_inv_state = 0u;
  uint8_t previous_inv_error = 0u;

  if (!rx_qitem)
  {
    return;
  }

  CAN_Unpack16(rx_qitem, &rx_msg);

#ifndef SIL_BUILD
  /* Operator command to re-enter the CAN bootloader for a reflash. Payload-
   * gated (shares id 0x002 with the AMS, distinct payload). Never returns. */
  if (Bootloader_MatchesTrigger(&rx_msg))
  {
    Bootloader_RequestReboot();
  }
  else
  {
    /* Pit-diag enable/disable (0x7E0); ack on 0x7E1. */
    uint8_t pd_en = 0u;
    if (PitDiag_MatchCommand(&rx_msg, &pd_en))
    {
      can_msg_t     pd_ack;
      can_qitem16_t pd_qi;
      PitDiag_SetEnabled(pd_en);
      PitDiag_BuildAck(pd_en, &pd_ack);
      CAN_Pack16(&pd_ack, &pd_qi);
      (void)osMessageQueuePut(canTxQueueHandle, &pd_qi, 0, 0);
    }
  }
#endif

  if (g_inMutex) {
    osMutexAcquire(g_inMutex, osWaitForever);
  }
  previous_inv_state = g_in.inv_state;
  previous_inv_error = g_in.inv_error;
  CanRx_ParseAndUpdate(&rx_msg, &g_in);
  (void)Telemetry_PublishStateEvent(previous_inv_state, previous_inv_error, &g_in);
  if (g_inMutex) {
    osMutexRelease(g_inMutex);
  }
}

static void app_runtime_process_can_tx_item(const can_qitem16_t *tx_qitem)
{
  can_msg_t tx_msg;

  if (!tx_qitem)
  {
    return;
  }

  CAN_Unpack16(tx_qitem, &tx_msg);
  (void)CanTx_SendHal(&tx_msg);
}

static void app_runtime_process_radio_frame(const telemetry_frame_t *frame)
{
  uint8_t fragment_count;
  uint8_t fragment_index;

  if (!frame)
  {
    return;
  }

  fragment_count = Telemetry_RadioFragmentCount(frame);
  if (fragment_count == 0u)
  {
    return;
  }

  for (fragment_index = 0u; fragment_index < fragment_count; fragment_index++)
  {
    Telemetry_TransportSendFragment(frame, fragment_index);
  }
}

static void app_runtime_process_sd_frame(const telemetry_frame_t *frame)
{
  uint8_t payload32[32];

  if (!frame)
  {
    return;
  }

  Telemetry_SerializeFrame(frame, payload32);
  Telemetry_SdStore32(payload32);
}

void AppRuntime_TaskMetricsReset(void)
{
  uint32_t task_index;

  for (task_index = 0u; task_index < (uint32_t)APP_TASK_ID_COUNT; task_index++)
  {
    s_task_metrics[task_index].start_count = 0u;
    s_task_metrics[task_index].step_count = 0u;
    s_task_metrics[task_index].last_start_tick = 0u;
    s_task_metrics[task_index].last_step_tick = 0u;
  }
}

uint32_t AppRuntime_TaskMetricsCount(void)
{
  return (uint32_t)APP_TASK_ID_COUNT;
}

const app_task_metrics_t *AppRuntime_TaskMetricsGet(app_task_id_t task_id)
{
  if ((uint32_t)task_id >= (uint32_t)APP_TASK_ID_COUNT)
  {
    return NULL;
  }

  return (const app_task_metrics_t *)&s_task_metrics[task_id];
}

void AppRuntime_TaskMetricsSnapshot(app_task_metrics_t *out_metrics, uint32_t max_metrics)
{
  uint32_t task_index;
  uint32_t copy_count = ((uint32_t)APP_TASK_ID_COUNT < max_metrics) ?
                        (uint32_t)APP_TASK_ID_COUNT : max_metrics;

  if (!out_metrics)
  {
    return;
  }

  for (task_index = 0u; task_index < copy_count; task_index++)
  {
    out_metrics[task_index] = *(const app_task_metrics_t *)&s_task_metrics[task_index];
  }
}

uint8_t AppRuntime_TaskStarted(app_task_id_t task_id)
{
  const app_task_metrics_t *metrics = AppRuntime_TaskMetricsGet(task_id);
  return (uint8_t)((metrics != NULL && metrics->start_count > 0u) ? 1u : 0u);
}

uint8_t AppRuntime_TaskStepped(app_task_id_t task_id)
{
  const app_task_metrics_t *metrics = AppRuntime_TaskMetricsGet(task_id);
  return (uint8_t)((metrics != NULL && metrics->step_count > 0u) ? 1u : 0u);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t RadioTxTaskHandle;
const osThreadAttr_t RadioTxTask_attributes = {
  .name = "RadioTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t SdLogTaskHandle;
const osThreadAttr_t SdLogTask_attributes = {
  .name = "SdLogTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t DashTaskHandle;
const osThreadAttr_t DashTask_attributes = {
  .name = "DashTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osMessageQueueId_t telemetryRadioQueueHandle;
const osMessageQueueAttr_t telemetryRadioQueue_attributes = {
  .name = "telemetryRadioQueue"
};

osMessageQueueId_t telemetrySdQueueHandle;
const osMessageQueueAttr_t telemetrySdQueue_attributes = {
  .name = "telemetrySdQueue"
};

osMessageQueueId_t telemetryDashQueueHandle;
const osMessageQueueAttr_t telemetryDashQueue_attributes = {
  .name = "telemetryDashQueue"
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for App_InitTask */
osThreadId_t App_InitTaskHandle;
const osThreadAttr_t App_InitTask_attributes = {
  .name = "App_InitTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CanRxTask */
osThreadId_t CanRxTaskHandle;
const osThreadAttr_t CanRxTask_attributes = {
  .name = "CanRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CanTxTask */
osThreadId_t CanTxTaskHandle;
const osThreadAttr_t CanTxTask_attributes = {
  .name = "CanTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DiagTask */
osThreadId_t DiagTaskHandle;
const osThreadAttr_t DiagTask_attributes = {
  .name = "DiagTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for canTxQueue */
osMessageQueueId_t canTxQueueHandle;
const osMessageQueueAttr_t canTxQueue_attributes = {
  .name = "canTxQueue"
};
/* Definitions for telemetryEventQueue */
osMessageQueueId_t telemetryEventQueueHandle;
const osMessageQueueAttr_t telemetryEventQueue_attributes = {
  .name = "telemetryEventQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartRadioTxTask(void *argument);
void StartSdLogTask(void *argument);
void StartDashTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAppInitTask(void *argument);
void StartControlTask(void *argument);
void StartCanRxTask(void *argument);
void StartCanTxTask(void *argument);
void StartTelemetryTask(void *argument);
void StartDiagTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Diag_Log("RTOS: MX_FREERTOS_Init enter");
  AppRuntime_TaskMetricsReset();

  s_periodic_generation++;
  if (s_periodic_generation == 0u)
  {
    s_periodic_generation = 1u;
  }

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Mutex global de acceso a g_in (app_state). DEBE crearse antes de
   * cualquier tarea que llame AppState_Snapshot / AppState_Init.       */
  g_inMutex = osMutexNew(NULL);
  Diag_Log("RTOS: g_inMutex %s", (g_inMutex != NULL) ? "ok" : "FAIL");
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew(128, sizeof(can_qitem16_t), NULL);
  Diag_Log("RTOS: canRxQueue %s", (canRxQueueHandle != NULL) ? "ok" : "FAIL");

  /* creation of canTxQueue */
  canTxQueueHandle = osMessageQueueNew(64, sizeof(can_qitem16_t), NULL);
  Diag_Log("RTOS: canTxQueue %s", (canTxQueueHandle != NULL) ? "ok" : "FAIL");

  /* creation of telemetryEventQueue */
  telemetryEventQueueHandle = osMessageQueueNew(32, sizeof(telemetry_event_t), NULL);
  Diag_Log("RTOS: telemetryEventQueue %s", (telemetryEventQueueHandle != NULL) ? "ok" : "FAIL");

  /* USER CODE BEGIN RTOS_QUEUES */
telemetryRadioQueueHandle = osMessageQueueNew(64, sizeof(telemetry_frame_t), NULL);
telemetrySdQueueHandle = osMessageQueueNew(64, sizeof(telemetry_frame_t), NULL);
telemetryDashQueueHandle = osMessageQueueNew(64, sizeof(telemetry_frame_t), NULL);
  Diag_Log("RTOS: telemetryRadioQueue %s", (telemetryRadioQueueHandle != NULL) ? "ok" : "FAIL");
  Diag_Log("RTOS: telemetrySdQueue %s", (telemetrySdQueueHandle != NULL) ? "ok" : "FAIL");
  Diag_Log("RTOS: telemetryDashQueue %s", (telemetryDashQueueHandle != NULL) ? "ok" : "FAIL");
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of App_InitTask */
  App_InitTaskHandle = osThreadNew(StartAppInitTask, NULL, &App_InitTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of CanRxTask */
  CanRxTaskHandle = osThreadNew(StartCanRxTask, NULL, &CanRxTask_attributes);

  /* creation of CanTxTask */
  CanTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &CanTxTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of DiagTask */
  DiagTaskHandle = osThreadNew(StartDiagTask, NULL, &DiagTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  RadioTxTaskHandle = osThreadNew(StartRadioTxTask, NULL, &RadioTxTask_attributes);
  SdLogTaskHandle = osThreadNew(StartSdLogTask, NULL, &SdLogTask_attributes);
  DashTaskHandle = osThreadNew(StartDashTask, NULL, &DashTask_attributes);
  Diag_Log("RTOS: RadioTxTask %s", (RadioTxTaskHandle != NULL) ? "ok" : "FAIL");
  Diag_Log("RTOS: SdLogTask %s", (SdLogTaskHandle != NULL) ? "ok" : "FAIL");
  Diag_Log("RTOS: DashTask %s", (DashTaskHandle != NULL) ? "ok" : "FAIL");
  Diag_Log("RTOS: MX_FREERTOS_Init exit");
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  if (app_task_started(APP_TASK_ID_DEFAULT))
  {
    Diag_Log("TASK: defaultTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  app_task_stepped(APP_TASK_ID_DEFAULT);
  osDelay(1);
  return;
#else
  for (;;)
  {
    app_task_stepped(APP_TASK_ID_DEFAULT);
    osDelay(app_ms_to_ticks(100u));
  }
#endif
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAppInitTask */
/**
* @brief Function implementing the App_InitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAppInitTask */
void StartAppInitTask(void *argument)
{
  /* USER CODE BEGIN StartAppInitTask */
  (void)argument;
  if (app_task_started(APP_TASK_ID_INIT))
  {
    Diag_Log("TASK: App_InitTask enter");
  }
  app_task_stepped(APP_TASK_ID_INIT);
  AppRuntime_InitStep();
  // Exit this init task - scheduler will run other tasks
  osThreadExit();

  /* USER CODE END StartAppInitTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Control loop: 10ms period (100Hz) */
  (void)argument;
  if (app_task_started(APP_TASK_ID_CONTROL))
  {
    Diag_Log("TASK: ControlTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_control_periodic = {0};
  app_task_stepped(APP_TASK_ID_CONTROL);
  AppRuntime_ControlStep();
  app_periodic_delay_until(&s_control_periodic, 10u);
  return;
#else
  uint32_t next_release = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(10u);
  for (;;)
  {
    app_task_stepped(APP_TASK_ID_CONTROL);
    AppRuntime_ControlStep();
    Bootloader_KickWatchdog();   /* service the BL-inherited IWDG (~8 s) */
    next_release += period_ticks;
    (void)osDelayUntil(next_release);
  }
#endif
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCanRxTask */
/**
* @brief Function implementing the CanRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void *argument)
{
  /* USER CODE BEGIN StartCanRxTask */
  /* CAN Receive task: 5ms period */
  (void)argument;
  if (app_task_started(APP_TASK_ID_CAN_RX))
  {
    Diag_Log("TASK: CanRxTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_can_rx_periodic = {0};
  app_task_stepped(APP_TASK_ID_CAN_RX);
  AppRuntime_CanRxStep();
  app_periodic_delay_until(&s_can_rx_periodic, 5u);
  return;
#else
  can_qitem16_t rx_qitem;
  for (;;)
  {
    if (osMessageQueueGet(canRxQueueHandle, &rx_qitem, NULL, osWaitForever) == osOK)
    {
      app_task_stepped(APP_TASK_ID_CAN_RX);
      app_runtime_process_can_rx_item(&rx_qitem);
      AppRuntime_CanRxStep();
    }
  }
#endif
  /* USER CODE END StartCanRxTask */
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief Function implementing the CanTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
void StartCanTxTask(void *argument)
{
  /* USER CODE BEGIN StartCanTxTask */
  /* CAN Transmit task: 10ms period (100Hz), aligned with control cadence */
  (void)argument;
  if (app_task_started(APP_TASK_ID_CAN_TX))
  {
    Diag_Log("TASK: CanTxTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_can_tx_periodic = {0};
  app_task_stepped(APP_TASK_ID_CAN_TX);
  AppRuntime_CanTxStep();
  app_periodic_delay_until(&s_can_tx_periodic, 10u);
  return;
#else
  can_qitem16_t tx_qitem;
  for (;;)
  {
    if (osMessageQueueGet(canTxQueueHandle, &tx_qitem, NULL, osWaitForever) == osOK)
    {
      app_task_stepped(APP_TASK_ID_CAN_TX);
      app_runtime_process_can_tx_item(&tx_qitem);
      AppRuntime_CanTxStep();
    }
  }
#endif
  /* USER CODE END StartCanTxTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  (void)argument;
  if (app_task_started(APP_TASK_ID_TELEMETRY))
  {
    Diag_Log("TASK: TelemetryTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  static uint32_t s_next_release = 0u;
  telemetry_event_t event;
  uint32_t now = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(100u);

  if (s_next_release == 0u)
  {
    app_task_stepped(APP_TASK_ID_TELEMETRY);
    AppRuntime_TelemetryStep();
    s_next_release = now + period_ticks;
  }
  else if (telemetryEventQueueHandle &&
           osMessageQueueGet(telemetryEventQueueHandle, &event, NULL, 0u) == osOK)
  {
    app_task_stepped(APP_TASK_ID_TELEMETRY);
    AppRuntime_TelemetryEventStep(&event);
  }
  else if (app_tick_reached(now, s_next_release))
  {
    app_task_stepped(APP_TASK_ID_TELEMETRY);
    AppRuntime_TelemetryStep();
    s_next_release += period_ticks;
  }

  (void)osDelayUntil((s_next_release > now + 1u) ? (now + 1u) : s_next_release);
  return;
#else
  telemetry_event_t event;
  uint32_t next_release = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(100u);
  AppRuntime_TelemetryStep();
  next_release += period_ticks;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    if (app_tick_reached(now, next_release))
    {
      app_task_stepped(APP_TASK_ID_TELEMETRY);
      AppRuntime_TelemetryStep();
      next_release += period_ticks;
      continue;
    }

    if (telemetryEventQueueHandle &&
        osMessageQueueGet(telemetryEventQueueHandle, &event, NULL, next_release - now) == osOK)
    {
      app_task_stepped(APP_TASK_ID_TELEMETRY);
      AppRuntime_TelemetryEventStep(&event);
      continue;
    }

    (void)osDelayUntil(next_release);
  }
#endif
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartDiagTask */
/**
* @brief Function implementing the DiagTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDiagTask */
void StartDiagTask(void *argument)
{
  /* USER CODE BEGIN StartDiagTask */
  (void)argument;
  if (app_task_started(APP_TASK_ID_DIAG))
  {
    Diag_Log("TASK: DiagTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_diag_periodic = {0};
  app_task_stepped(APP_TASK_ID_DIAG);
  app_task_metrics_log_summary();
  app_diag_log_imu_probe();
  app_diag_log_i2c_scan();
  app_diag_log_imu_snapshot();
  app_periodic_delay_until(&s_diag_periodic, 1000u);
  return;
#else
  for (;;)
  {
    app_task_stepped(APP_TASK_ID_DIAG);
    app_task_metrics_log_summary();
    app_diag_log_imu_probe();
    app_diag_log_i2c_scan();
    app_diag_log_imu_snapshot();
    osDelay(app_ms_to_ticks(1000u));
  }
#endif
  /* USER CODE END StartDiagTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartRadioTxTask(void *argument)
{
  (void)argument;
  if (app_task_started(APP_TASK_ID_RADIO_TX))
  {
    Diag_Log("TASK: RadioTxTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  app_task_stepped(APP_TASK_ID_RADIO_TX);
  AppRuntime_RadioTxStep();
  osDelay(1u);
  return;
#else
  telemetry_frame_t frame;
  for (;;)
  {
    if (osMessageQueueGet(telemetryRadioQueueHandle, &frame, NULL, osWaitForever) == osOK)
    {
      app_task_stepped(APP_TASK_ID_RADIO_TX);
      app_runtime_process_radio_frame(&frame);
      AppRuntime_RadioTxStep();
    }
  }
#endif
}

void StartSdLogTask(void *argument)
{
  (void)argument;
  if (app_task_started(APP_TASK_ID_SD_LOG))
  {
    Diag_Log("TASK: SdLogTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  app_task_stepped(APP_TASK_ID_SD_LOG);
  AppRuntime_SdLogStep();
  osDelay(1u);
  return;
#else
  telemetry_frame_t frame;
  for (;;)
  {
    if (osMessageQueueGet(telemetrySdQueueHandle, &frame, NULL, osWaitForever) == osOK)
    {
      app_task_stepped(APP_TASK_ID_SD_LOG);
      app_runtime_process_sd_frame(&frame);
      AppRuntime_SdLogStep();
    }
  }
#endif
}

void StartDashTask(void *argument)
{
  (void)argument;
  if (app_task_started(APP_TASK_ID_DASH))
  {
    Diag_Log("TASK: DashTask enter");
  }
#ifdef SIL_USE_THREAD_SCHEDULER
  app_task_stepped(APP_TASK_ID_DASH);
  AppRuntime_DashStep();
  osDelay(1u);
  return;
#else
  telemetry_frame_t frame;
  for (;;)
  {
    if (osMessageQueueGet(telemetryDashQueueHandle, &frame, NULL, osWaitForever) == osOK)
    {
      app_task_stepped(APP_TASK_ID_DASH);
      app_dash_publish_frame(&frame);
      AppRuntime_DashStep();
    }
  }
#endif
}

void AppRuntime_InitStep(void)
{
  Diag_Log("\n=== ECU08 NSIL INITIALIZATION ===\n");

  AppState_Init();
  Diag_Log("State machine initialized (WAIT_INV_VDC_CONFIG)\n");

  Telemetry_Init();
  s_telemetry_radio_divider = 2u;
  s_telemetry_sd_divider = 9u;
  Diag_Log("Telemetry subsystem initialized\n");

  Control_Init();
  Diag_Log("Control module initialized\n");

  IoSignals_Init();
  Diag_Log("Physical IO signal bridge initialized\n");

#ifndef SIL_BUILD
  if (FDCAN_RuntimeBringUp() != HAL_OK) {
    Diag_Log("FDCAN runtime bring-up failed\n");
    Error_Handler();
  }
  Diag_Log("FDCAN runtime bring-up complete\n");
#endif

  Diag_Log("=== INITIALIZATION COMPLETE ===\n");
}

void AppRuntime_ControlStep(void)
{
  app_inputs_t state_snapshot;
  control_out_t control_output;

  IoSignals_InputStep();
  AppState_Snapshot(&state_snapshot);
  Control_Step10ms(&state_snapshot, &control_output);

  if (g_inMutex) {
    osMutexAcquire(g_inMutex, osWaitForever);
  }
  g_in.torque_total = control_output.torque_pct;
  g_in.flag_EV_2_3 = control_output.flag_ev_2_3;
  g_in.flag_T11_8_9 = control_output.flag_t11_8_9;
  if (g_inMutex) {
    osMutexRelease(g_inMutex);
  }

  IoSignals_ApplyOutputs(&control_output);

  for (uint8_t i = 0; i < control_output.count; i++) {
    can_qitem16_t qitem;
    CAN_Pack16(&control_output.msgs[i], &qitem);
    (void)osMessageQueuePut(canTxQueueHandle, &qitem, 0, 0);
  }

#ifndef SIL_BUILD
  /* Pit-diag bench stream (0x700..0x703) when enabled via 0x7E0, at 100 ms. */
  {
    can_msg_t diag[4];
    uint8_t dn = PitDiag_Collect(&state_snapshot, &control_output,
                                 ecu_fw_version_major(), ecu_fw_version_minor(),
                                 ecu_fw_version_patch(), ecu_git_hash(),
                                 diag, 4u);
    for (uint8_t i = 0; i < dn; i++) {
      can_qitem16_t qitem;
      CAN_Pack16(&diag[i], &qitem);
      (void)osMessageQueuePut(canTxQueueHandle, &qitem, 0, 0);
    }
  }
#endif
}

void AppRuntime_CanRxStep(void)
{
  can_qitem16_t rx_qitem;
  osStatus_t status;

  status = osMessageQueueGet(canRxQueueHandle, &rx_qitem, NULL, 0);
  while (status == osOK) {
    app_runtime_process_can_rx_item(&rx_qitem);
    status = osMessageQueueGet(canRxQueueHandle, &rx_qitem, NULL, 0);
  }
}

void AppRuntime_CanTxStep(void)
{
  can_qitem16_t tx_qitem;
  osStatus_t status;

  status = osMessageQueueGet(canTxQueueHandle, &tx_qitem, NULL, 0);
  while (status == osOK) {
    app_runtime_process_can_tx_item(&tx_qitem);
    status = osMessageQueueGet(canTxQueueHandle, &tx_qitem, NULL, 0);
  }
}

void AppRuntime_TelemetryStep(void)
{
  app_inputs_t state_snapshot;
  telemetry_frame_t dash_frame;
  telemetry_frame_t radio_frame;
  telemetry_frame_t sd_frame;
  uint8_t send_radio;
  uint8_t send_sd;

  AppState_Snapshot(&state_snapshot);
  Telemetry_BuildFrameWithKind(&state_snapshot, NULL, TELEMETRY_FRAME_DASH, &dash_frame);
  (void)Telemetry_EnqueueFrameTargets(&dash_frame, 0u, 0u, 1u);

  s_telemetry_radio_divider++;
  if (s_telemetry_radio_divider >= 3u)
  {
    s_telemetry_radio_divider = 0u;
    send_radio = 1u;
  }
  else
  {
    send_radio = 0u;
  }

  s_telemetry_sd_divider++;
  if (s_telemetry_sd_divider >= 10u)
  {
    s_telemetry_sd_divider = 0u;
    send_sd = 1u;
  }
  else
  {
    send_sd = 0u;
  }

  if (send_radio)
  {
    Telemetry_BuildFrameWithKind(&state_snapshot, NULL, TELEMETRY_FRAME_RF_FAST, &radio_frame);
    (void)Telemetry_EnqueueFrameTargets(&radio_frame, 1u, 0u, 0u);
  }

  if (send_sd)
  {
    Telemetry_BuildFrameWithKind(&state_snapshot, NULL, TELEMETRY_FRAME_RF_SLOW, &radio_frame);
    (void)Telemetry_EnqueueFrameTargets(&radio_frame, 1u, 0u, 0u);

    Telemetry_BuildFrameWithKind(&state_snapshot, NULL, TELEMETRY_FRAME_SD, &sd_frame);
    (void)Telemetry_EnqueueFrameTargets(&sd_frame, 0u, 1u, 0u);
  }
}

void AppRuntime_TelemetryEventStep(const telemetry_event_t *event)
{
  app_inputs_t state_snapshot;
  telemetry_frame_t dash_frame;
  telemetry_frame_t radio_frame;
  telemetry_frame_t sd_frame;

  if (!event) return;

  AppState_Snapshot(&state_snapshot);
  Telemetry_BuildFrameWithKind(&state_snapshot, event, TELEMETRY_FRAME_DASH, &dash_frame);
  (void)Telemetry_EnqueueFrameTargets(&dash_frame, 0u, 0u, 1u);

  Telemetry_BuildFrameWithKind(&state_snapshot, event, TELEMETRY_FRAME_RF_EVENT, &radio_frame);
  (void)Telemetry_EnqueueFrameTargets(&radio_frame, 1u, 0u, 0u);

  Telemetry_BuildFrameWithKind(&state_snapshot, event, TELEMETRY_FRAME_SD, &sd_frame);
  (void)Telemetry_EnqueueFrameTargets(&sd_frame, 0u, 1u, 0u);
}

void AppRuntime_RadioTxStep(void)
{
  telemetry_frame_t frame;
  osStatus_t status;

  status = osMessageQueueGet(telemetryRadioQueueHandle, &frame, NULL, 0u);
  if (status != osOK)
  {
    return;
  }
  app_runtime_process_radio_frame(&frame);
}

void AppRuntime_SdLogStep(void)
{
  telemetry_frame_t frame;
  osStatus_t status;

  status = osMessageQueueGet(telemetrySdQueueHandle, &frame, NULL, 0u);
  while (status == osOK)
  {
    app_runtime_process_sd_frame(&frame);
    status = osMessageQueueGet(telemetrySdQueueHandle, &frame, NULL, 0u);
  }
}

static void app_dash_push_u16le(uint16_t value, uint8_t out[2])
{
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void app_dash_push_u32le(uint32_t value, uint8_t out[4])
{
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
  out[2] = (uint8_t)((value >> 16) & 0xFFu);
  out[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void app_dash_send_msg(const can_msg_t *msg)
{
  if (!msg) return;
  (void)CanTx_SendHal(msg);
}

static void app_dash_publish_frame(const telemetry_frame_t *frame)
{
  can_msg_t msg;
  uint8_t fault_bits = 0u;

  if (!frame) return;

  if (frame->snapshot.flag_EV_2_3)  fault_bits |= 0x01u;
  if (frame->snapshot.flag_T11_8_9) fault_bits |= 0x02u;
  if (frame->snapshot.inv_error)    fault_bits |= 0x04u;

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x510u;
  msg.dlc = 8u;
  msg.data[0] = frame->snapshot.inv_state;
  msg.data[1] = (uint8_t)frame->snapshot.torque_total;
  msg.data[2] = fault_bits;
  msg.data[3] = frame->snapshot.ok_precarga;
  msg.data[4] = frame->snapshot.boton_arranque;
  msg.data[5] = (uint8_t)frame->kind;
  msg.data[6] = (uint8_t)(frame->sequence & 0xFFu);
  msg.data[7] = (uint8_t)((frame->sequence >> 8) & 0xFFu);
  app_dash_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x511u;
  msg.dlc = 6u;
  app_dash_push_u16le(frame->snapshot.s1_aceleracion, &msg.data[0]);
  app_dash_push_u16le(frame->snapshot.s2_aceleracion, &msg.data[2]);
  app_dash_push_u16le(frame->snapshot.s_freno, &msg.data[4]);
  app_dash_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x512u;
  msg.dlc = 6u;
  app_dash_push_u16le(frame->snapshot.inv_dc_bus_voltage, &msg.data[0]);
  app_dash_push_u16le(frame->snapshot.v_celda_min, &msg.data[2]);
  msg.data[4] = frame->snapshot.inv_error;
  msg.data[5] = frame->snapshot.inv_vdc_ready;
  app_dash_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x513u;
  msg.dlc = 6u;
  app_dash_push_u16le((uint16_t)frame->snapshot.inv_motor_temp, &msg.data[0]);
  app_dash_push_u16le((uint16_t)frame->snapshot.inv_igbt_temp, &msg.data[2]);
  app_dash_push_u16le((uint16_t)frame->snapshot.inv_air_temp, &msg.data[4]);
  app_dash_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x514u;
  msg.dlc = 4u;
  app_dash_push_u32le((uint32_t)frame->snapshot.inv_rpm, &msg.data[0]);
  app_dash_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x515u;
  msg.dlc = 4u;
  app_dash_push_u32le((uint32_t)frame->snapshot.inv_speed_actual, &msg.data[0]);
  app_dash_send_msg(&msg);

  memset(&msg, 0, sizeof(msg));
  msg.bus = CAN_BUS_DASH;
  msg.id = 0x516u;
  msg.dlc = 4u;
  app_dash_push_u32le((uint32_t)frame->snapshot.inv_current_actual, &msg.data[0]);
  app_dash_send_msg(&msg);
}

void AppRuntime_DashStep(void)
{
  telemetry_frame_t frame;
  osStatus_t status;

  status = osMessageQueueGet(telemetryDashQueueHandle, &frame, NULL, 0u);
  while (status == osOK)
  {
    app_dash_publish_frame(&frame);
    status = osMessageQueueGet(telemetryDashQueueHandle, &frame, NULL, 0u);
  }
}

/* USER CODE END Application */

