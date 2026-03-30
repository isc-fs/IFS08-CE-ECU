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
#include "telemetry.h"   /* Telemetry_Build32, Telemetry_Send32       */
#include "test_integration.h"  /* Integration tests – modo HIL (hardware)  */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_state.h"
#include "app_runtime.h"
#include "control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef SIL_USE_THREAD_SCHEDULER
#define APP_TASK_LOOP_BEGIN() do {
#define APP_TASK_LOOP_DELAY(ms) do { osDelay((ms)); return; } while (0)
#define APP_TASK_LOOP_END() } while (0)
#else
#define APP_TASK_LOOP_BEGIN() for(;;) {
#define APP_TASK_LOOP_DELAY(ms) osDelay((ms))
#define APP_TASK_LOOP_END() }
#endif

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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DiagTask */
osThreadId_t DiagTaskHandle;
const osThreadAttr_t DiagTask_attributes = {
  .name = "DiagTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IntegrationTestTask */
osThreadId_t IntegrationTestTaskHandle;
const osThreadAttr_t IntegrationTestTask_attributes = {
  .name = "IntegrationTest",
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
void StartIntegrationTestTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
#ifndef SIL_USE_THREAD_SCHEDULER
  IntegrationTestTaskHandle = osThreadNew(StartIntegrationTestTask, NULL, &IntegrationTestTask_attributes);
#else
  IntegrationTestTaskHandle = NULL;
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
  APP_TASK_LOOP_BEGIN()
    APP_TASK_LOOP_DELAY(1);
  APP_TASK_LOOP_END();
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
  APP_TASK_LOOP_BEGIN()
    AppRuntime_ControlStep();
    APP_TASK_LOOP_DELAY(10);
  APP_TASK_LOOP_END();
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
  APP_TASK_LOOP_BEGIN()
    AppRuntime_CanRxStep();
    APP_TASK_LOOP_DELAY(5);  // 5ms polling rate (200Hz)
  APP_TASK_LOOP_END();
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
  /* CAN Transmit task: 20ms period (50Hz) */
  (void)argument;
  APP_TASK_LOOP_BEGIN()
    AppRuntime_CanTxStep();
    APP_TASK_LOOP_DELAY(20);
  APP_TASK_LOOP_END();
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
  /* Telemetry logging task: 100ms period (10Hz) */
  (void)argument;
  APP_TASK_LOOP_BEGIN()
    AppRuntime_TelemetryStep();
    APP_TASK_LOOP_DELAY(100);
  APP_TASK_LOOP_END();
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
  APP_TASK_LOOP_BEGIN()
    APP_TASK_LOOP_DELAY(1);
  APP_TASK_LOOP_END();
  /* USER CODE END StartDiagTask */
}

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void AppRuntime_InitStep(void)
{
  Diag_Log("\n=== ECU08 NSIL INITIALIZATION ===\n");

  AppState_Init();
  Diag_Log("State machine initialized (BOOT)\n");

  Control_Init();
  Diag_Log("Control module initialized\n");

  Diag_Log("=== INITIALIZATION COMPLETE ===\n");
}

void AppRuntime_ControlStep(void)
{
  app_inputs_t state_snapshot;
  control_out_t control_output;

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
    CAN_Unpack16(&rx_qitem, &rx_msg);

    if (g_inMutex) {
      osMutexAcquire(g_inMutex, osWaitForever);
    }
    CanRx_ParseAndUpdate(&rx_msg, &g_in);
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
  uint8_t payload32[32];

  AppState_Snapshot(&state_snapshot);
  Telemetry_Build32(&state_snapshot, payload32);
  Telemetry_Send32(payload32);
}

/* USER CODE END Application */

