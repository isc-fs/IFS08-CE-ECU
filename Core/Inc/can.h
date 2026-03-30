#ifndef CAN_APP_H
#define CAN_APP_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "app_state.h"
#ifdef SIL_BUILD
#include <main.h>  /* mocks/main.h: tipos HAL/FDCAN sin STM32 HAL real */
#else
#include "main.h"  /* Core/Inc/main.h: stm32h7xx_hal.h + FDCAN real   */
#endif

/* CubeMX-friendly 16-byte queue item */
typedef struct { uint32_t w[4]; } can_qitem16_t;

typedef enum
{
  CAN_BUS_INV  = 1,  /* inverter */
  CAN_BUS_ACU  = 2,  /* accumulator */
  CAN_BUS_DASH = 3   /* dashboard / misc */
} can_bus_t;

typedef struct
{
  can_bus_t bus;
  uint32_t id;
  uint8_t  dlc;      /* 0..8 */
  uint8_t  ide;      /* 0=std, 1=ext */
  uint8_t  data[8];
} can_msg_t;

/* Queue handles are created in freertos.c USER CODE. */
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t canTxQueueHandle;

/* Pack/unpack helpers */
void CAN_Pack16(const can_msg_t *m, can_qitem16_t *q);
void CAN_Unpack16(const can_qitem16_t *q, can_msg_t *m);

/* RX parsing: updates app state based on CAN IDs (called from CanRxTask under mutex). */
void CanRx_ParseAndUpdate(const can_msg_t *m, app_inputs_t *st);

/* TX: central HAL sender (called only from CanTxTask). */
HAL_StatusTypeDef CanTx_SendHal(const can_msg_t *m);

/* ISR helper: call from HAL_FDCAN_RxFifo0Callback to enqueue into canRxQueueHandle. */
void Can_ISR_PushRxFifo0(FDCAN_HandleTypeDef *hfdcan);

#endif /* CAN_APP_H */
