/* Replace your HAL_FDCAN_RxFifo0Callback in main.c with this minimal version.
 * It reads the message and pushes it to canRxQueueHandle.
 */
#include "can.h"

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
  {
    Can_ISR_PushRxFifo0(hfdcan);
  }
}
