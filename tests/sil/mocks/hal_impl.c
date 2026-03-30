/**
 * hal_impl.c  –  Stubs HAL/FDCAN para build SIL
 *
 * Define los objetos globales de handle FDCAN (hfdcan1/2/3) que can.c
 * declara como extern, y provee implementaciones stub de las funciones
 * HAL FDCAN que retornan HAL_ERROR (no hay hardware en SIL).
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------
   Handles FDCAN globales (extern en can.c, definidos aquí en SIL)
   ---------------------------------------------------------------------- */
FDCAN_HandleTypeDef hfdcan1 = { .Instance = 0x40006400UL };
FDCAN_HandleTypeDef hfdcan2 = { .Instance = 0x40006800UL };
FDCAN_HandleTypeDef hfdcan3 = { .Instance = 0x40006C00UL };

/* -------------------------------------------------------------------------
   Stubs HAL FDCAN
   En SIL no hay hardware, así que estas funciones son no-ops seguros.
   CanTx_SendHal() llama a HAL_FDCAN_AddMessageToTxFifoQ → retorna ERROR,
   pero en los tests de integración no se llama a CanTx_SendHal directamente.
   ---------------------------------------------------------------------- */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                  FDCAN_TxHeaderTypeDef *pTxHeader,
                                                  uint8_t *pTxData)
{
    (void)hfdcan; (void)pTxHeader; (void)pTxData;
    /* SIL: TX hardware no disponible */
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxLocation,
                                          FDCAN_RxHeaderTypeDef *pRxHeader,
                                          uint8_t *pRxData)
{
    (void)hfdcan; (void)RxLocation;
    if (pRxHeader) memset(pRxHeader, 0, sizeof(*pRxHeader));
    if (pRxData)   memset(pRxData,   0, 8);
    return HAL_ERROR;
}

/* -------------------------------------------------------------------------
   Error handler  (en STM32 entra en loop infinito; en SIL solo imprime)
   ---------------------------------------------------------------------- */
void Error_Handler(void)
{
    fprintf(stderr, "[SIL][ERROR] Error_Handler() llamado\n");
}
