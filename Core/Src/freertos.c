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
#include "can.h"        /* can_qitem16_t, CAN_Pack16, etc.          */
#include "diag.h"       /* Diag_Log                                  */
#include "telemetry.h"  /* Telemetry task + frame/event plumbing     */
#include "test_integration.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_state.h"
#include "app_runtime.h"
#include "control.h"
#include "io_signals.h"
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

static uint32_t s_periodic_generation;
static uint8_t s_telemetry_radio_divider;
static uint8_t s_telemetry_sd_divider;

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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DiagTask */
osThreadId_t DiagTaskHandle;
const osThreadAttr_t DiagTask_attributes = {
  .name = "DiagTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
#ifdef APP_ENABLE_INTEGRATION_TEST_TASK
/* Definitions for IntegrationTestTask */
osThreadId_t IntegrationTestTaskHandle;
const osThreadAttr_t IntegrationTestTask_attributes = {
  .name = "IntegrationTest",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
#endif
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
#ifdef APP_ENABLE_INTEGRATION_TEST_TASK
void StartIntegrationTestTask(void *argument);
#endif

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Diag_Log("RTOS: MX_FREERTOS_Init enter");

  /* USER CODE END Init */

  s_periodic_generation++;
  if (s_periodic_generation == 0u)
  {
    s_periodic_generation = 1u;
  }

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
  Diag_Log("RTOS: defaultTask %s", (defaultTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of App_InitTask */
  App_InitTaskHandle = osThreadNew(StartAppInitTask, NULL, &App_InitTask_attributes);
  Diag_Log("RTOS: App_InitTask %s", (App_InitTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);
  Diag_Log("RTOS: ControlTask %s", (ControlTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of CanRxTask */
  CanRxTaskHandle = osThreadNew(StartCanRxTask, NULL, &CanRxTask_attributes);
  Diag_Log("RTOS: CanRxTask %s", (CanRxTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of CanTxTask */
  CanTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &CanTxTask_attributes);
  Diag_Log("RTOS: CanTxTask %s", (CanTxTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);
  Diag_Log("RTOS: TelemetryTask %s", (TelemetryTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of DiagTask */
  DiagTaskHandle = osThreadNew(StartDiagTask, NULL, &DiagTask_attributes);
  Diag_Log("RTOS: DiagTask %s", (DiagTaskHandle != NULL) ? "ok" : "FAIL");

  /* creation of IntegrationTestTask */
#ifdef APP_ENABLE_INTEGRATION_TEST_TASK
#ifndef SIL_USE_THREAD_SCHEDULER
  IntegrationTestTaskHandle = osThreadNew(StartIntegrationTestTask, NULL, &IntegrationTestTask_attributes);
  Diag_Log("RTOS: IntegrationTestTask %s", (IntegrationTestTaskHandle != NULL) ? "ok" : "FAIL");
#else
  IntegrationTestTaskHandle = NULL;
  Diag_Log("RTOS: IntegrationTestTask skipped in SIL scheduler");
#endif
#endif

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
  Diag_Log("TASK: defaultTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  osDelay(1);
  return;
#else
  for (;;)
  {
    osDelay(1);
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
  Diag_Log("TASK: App_InitTask enter");
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
  Diag_Log("TASK: ControlTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_control_periodic = {0};
  AppRuntime_ControlStep();
  app_periodic_delay_until(&s_control_periodic, 10u);
  return;
#else
  uint32_t next_release = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(10u);
  for (;;)
  {
    AppRuntime_ControlStep();
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
  Diag_Log("TASK: CanRxTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_can_rx_periodic = {0};
  AppRuntime_CanRxStep();
  app_periodic_delay_until(&s_can_rx_periodic, 5u);
  return;
#else
  uint32_t next_release = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(5u);
  for (;;)
  {
    AppRuntime_CanRxStep();
    next_release += period_ticks;
    (void)osDelayUntil(next_release);
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
  Diag_Log("TASK: CanTxTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  static app_periodic_state_t s_can_tx_periodic = {0};
  AppRuntime_CanTxStep();
  app_periodic_delay_until(&s_can_tx_periodic, 10u);
  return;
#else
  uint32_t next_release = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(10u);
  for (;;)
  {
    AppRuntime_CanTxStep();
    next_release += period_ticks;
    (void)osDelayUntil(next_release);
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
  Diag_Log("TASK: TelemetryTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  static uint32_t s_next_release = 0u;
  telemetry_event_t event;
  uint32_t now = osKernelGetTickCount();
  uint32_t period_ticks = app_ms_to_ticks(100u);

  if (s_next_release == 0u)
  {
    AppRuntime_TelemetryStep();
    s_next_release = now + period_ticks;
  }
  else if (telemetryEventQueueHandle &&
           osMessageQueueGet(telemetryEventQueueHandle, &event, NULL, 0u) == osOK)
  {
    AppRuntime_TelemetryEventStep(&event);
  }
  else if (app_tick_reached(now, s_next_release))
  {
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
      AppRuntime_TelemetryStep();
      next_release += period_ticks;
      continue;
    }

    if (telemetryEventQueueHandle &&
        osMessageQueueGet(telemetryEventQueueHandle, &event, NULL, next_release - now) == osOK)
    {
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
  Diag_Log("TASK: DiagTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  osDelay(1);
  return;
#else
  for (;;)
  {
    osDelay(1);
  }
#endif
  /* USER CODE END StartDiagTask */
}

#ifdef APP_ENABLE_INTEGRATION_TEST_TASK
/* USER CODE BEGIN Header_StartIntegrationTestTask */
/**
* @brief Function implementing the IntegrationTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIntegrationTestTask */
void StartIntegrationTestTask(void *argument)
{
  /* USER CODE BEGIN StartIntegrationTestTask */
  (void)argument;
  Diag_Log("TASK: IntegrationTestTask enter");

  Diag_Log("\n\nIntegrationTestTask started - esperando estabilizacion del sistema...");
  osDelay(200);

  {
    test_result_t res = Test_IntegrationRunAll();
    (void)res;
  }

  osThreadSuspend(NULL);
  
  /* USER CODE END StartIntegrationTestTask */
}
#endif

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartRadioTxTask(void *argument)
{
  (void)argument;
  Diag_Log("TASK: RadioTxTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  AppRuntime_RadioTxStep();
  osDelay(1u);
  return;
#else
  for (;;)
  {
    AppRuntime_RadioTxStep();
    osDelay(1u);
  }
#endif
}

void StartSdLogTask(void *argument)
{
  (void)argument;
  Diag_Log("TASK: SdLogTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  AppRuntime_SdLogStep();
  osDelay(1u);
  return;
#else
  for (;;)
  {
    AppRuntime_SdLogStep();
    osDelay(1u);
  }
#endif
}

void StartDashTask(void *argument)
{
  (void)argument;
  Diag_Log("TASK: DashTask enter");
#ifdef SIL_USE_THREAD_SCHEDULER
  AppRuntime_DashStep();
  osDelay(1u);
  return;
#else
  for (;;)
  {
    AppRuntime_DashStep();
    osDelay(1u);
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
}

void AppRuntime_CanRxStep(void)
{
  can_qitem16_t rx_qitem;
  can_msg_t rx_msg;
  osStatus_t status;

  status = osMessageQueueGet(canRxQueueHandle, &rx_qitem, NULL, 0);
  while (status == osOK) {
    uint8_t previous_inv_state = 0u;
    uint8_t previous_inv_error = 0u;

    CAN_Unpack16(&rx_qitem, &rx_msg);

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

    status = osMessageQueueGet(canRxQueueHandle, &rx_qitem, NULL, 0);
  }
}

void AppRuntime_CanTxStep(void)
{
  can_qitem16_t tx_qitem;
  can_msg_t tx_msg;
  osStatus_t status;

  status = osMessageQueueGet(canTxQueueHandle, &tx_qitem, NULL, 0);
  while (status == osOK) {
    CAN_Unpack16(&tx_qitem, &tx_msg);
    (void)CanTx_SendHal(&tx_msg);
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
  uint8_t fragment_count;
  uint8_t fragment_index;

  status = osMessageQueueGet(telemetryRadioQueueHandle, &frame, NULL, 0u);
  if (status != osOK)
  {
    return;
  }

  fragment_count = Telemetry_RadioFragmentCount(&frame);
  if (fragment_count == 0u)
  {
    return;
  }

  for (fragment_index = 0u; fragment_index < fragment_count; fragment_index++)
  {
    Telemetry_TransportSendFragment(&frame, fragment_index);
  }
}

void AppRuntime_SdLogStep(void)
{
  telemetry_frame_t frame;
  uint8_t payload32[32];
  osStatus_t status;

  status = osMessageQueueGet(telemetrySdQueueHandle, &frame, NULL, 0u);
  while (status == osOK)
  {
    Telemetry_SerializeFrame(&frame, payload32);
    Telemetry_SdStore32(payload32);
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

