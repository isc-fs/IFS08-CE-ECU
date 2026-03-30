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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
  IntegrationTestTaskHandle = osThreadNew(StartIntegrationTestTask, NULL, &IntegrationTestTask_attributes);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
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

  Diag_Log("\n=== ECU08 NSIL INITIALIZATION ===\n");
  
  // Initialize application state machine (BOOT state)
  AppState_Init();
  Diag_Log("State machine initialized (BOOT)\n");
  
  // Initialize control logic
  Control_Init();
  Diag_Log("Control module initialized\n");
  
  Diag_Log("=== INITIALIZATION COMPLETE ===\n");
  
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
  
  app_inputs_t state_snapshot;
  control_out_t control_output;
  
  for(;;)
  {
    // 1. Take snapshot of application state (thread-safe via mutex)
    AppState_Snapshot(&state_snapshot);
    
    // 2. Execute control logic (10ms timestep)
    Control_Step10ms(&state_snapshot, &control_output);
    
    // 3. Process CAN messages to send (if any)
    for (uint8_t i = 0; i < control_output.count; i++) {
      can_qitem16_t qitem;
      CAN_Pack16(&control_output.msgs[i], &qitem);
      osMessageQueuePut(canTxQueueHandle, &qitem, 0, 0);
    }
    
    // 4. Sleep for 10ms (100Hz control loop)
    osDelay(10);
  }
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
  
  can_qitem16_t rx_qitem;
  can_msg_t rx_msg;
  osStatus_t status;
  app_inputs_t snapshot;
  
  for(;;)
  {
    // Check if messages in queue (non-blocking)
    status = osMessageQueueGet(canRxQueueHandle, &rx_qitem, NULL, 0);
    
    if (status == osOK) {
      // Unpack queue item to CAN message
      CAN_Unpack16(&rx_qitem, &rx_msg);
      
      // Take snapshot, parse and update
      AppState_Snapshot(&snapshot);
      CanRx_ParseAndUpdate(&rx_msg, &snapshot);
      // (Caller should update shared state under mutex)
    }
    
    osDelay(5);  // 5ms polling rate (200Hz)
  }
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
  
  can_qitem16_t tx_qitem;
  can_msg_t tx_msg;
  osStatus_t status;
  
  for(;;)
  {
    // Check if messages pending in TX queue (non-blocking)
    status = osMessageQueueGet(canTxQueueHandle, &tx_qitem, NULL, 0);
    
    while (status == osOK) {
      // Unpack and transmit
      CAN_Unpack16(&tx_qitem, &tx_msg);
      CanTx_SendHal(&tx_msg);
      
      // Check for next message (non-blocking)
      status = osMessageQueueGet(canTxQueueHandle, &tx_qitem, NULL, 0);
    }
    
    // Sleep for 20ms (50Hz TX rate)
    osDelay(20);
  }
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
  
  app_inputs_t state_snapshot;
  uint8_t payload32[32];
  
  for(;;)
  {
    // Take snapshot of current state
    AppState_Snapshot(&state_snapshot);
    
    // Build telemetry payload (32 bytes)
    Telemetry_Build32(&state_snapshot, payload32);
    
    // Send telemetry (UART/nRF24/etc)
    Telemetry_Send32(payload32);
    
    // Sleep for 100ms (10Hz logging rate)
    osDelay(100);
  }
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
  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
  }
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

/* USER CODE END Application */

