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
#include "diag.h"        /* Diag_Log                                  */
#include "telemetry.h"   /* Telemetry task + frame/event plumbing     */
#include "test_integration.h"  /* Integration tests – modo HIL (hardware)  */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_state.h"
#include "app_runtime.h"
#include "control.h"
#include "io_signals.h"
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
canTxQueueHandle = osMessageQueueNew(64,  sizeof(can_qitem16_t), NULL);
telemetryEventQueueHandle = osMessageQueueNew(32, sizeof(telemetry_event_t), NULL);


  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
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

  /* creation of IntegrationTestTask */
#ifdef APP_ENABLE_INTEGRATION_TEST_TASK
#ifndef SIL_USE_THREAD_SCHEDULER
  IntegrationTestTaskHandle = osThreadNew(StartIntegrationTestTask, NULL, &IntegrationTestTask_attributes);
#else
  IntegrationTestTaskHandle = NULL;
#endif
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  
  Diag_Log("\n\nIntegrationTestTask started – esperando estabilizacion del sistema...");

  /* Esperar a que el resto de tareas se inicialicen (AppInitTask termina
   * en ~0 ms, pero necesitamos que g_inMutex y las colas existan).       */
  osDelay(200);

  /* Ejecutar todas las suites de integración y generar informe */
  test_result_t res = Test_IntegrationRunAll();
  (void)res;  /* resultado ya impreso por Test_IntegrationRunAll            */

  /* Tarea de test finalizada – se suspende indefinidamente */
  osThreadSuspend(NULL);
  
  /* USER CODE END StartIntegrationTestTask */
}
#endif

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void AppRuntime_InitStep(void)
{
  Diag_Log("\n=== ECU08 NSIL INITIALIZATION ===\n");

  AppState_Init();
  Diag_Log("State machine initialized (WAIT_INV_VDC_CONFIG)\n");

  Telemetry_Init();
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
  telemetry_frame_t frame;

  AppState_Snapshot(&state_snapshot);
  Telemetry_BuildFrame(&state_snapshot, NULL, &frame);
  Telemetry_TransportSend(&frame);
}

void AppRuntime_TelemetryEventStep(const telemetry_event_t *event)
{
  app_inputs_t state_snapshot;
  telemetry_frame_t frame;

  if (!event) return;

  AppState_Snapshot(&state_snapshot);
  Telemetry_BuildFrame(&state_snapshot, event, &frame);
  Telemetry_TransportSend(&frame);
}

/* USER CODE END Application */

