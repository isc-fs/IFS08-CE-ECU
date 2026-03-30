/**
 ******************************************************************************
 * @file           : test_integration.h
 * @brief          : Integration tests ECU08 NSIL – FreeRTOS + Hardware
 *
 * MODOS DE EJECUCION:
 *   TEST_MODE_SIL  – Software-In-the-Loop (PC/QEMU, sin hardware real)
 *                    Definir: -DTEST_MODE_SIL en las opciones de compilación
 *   TEST_MODE_HIL  – Hardware-In-the-Loop (STM32H733, periféricos reales)
 *                    Definir: -DTEST_MODE_HIL (o ninguno, es el default)
 *
 * SUITES DISPONIBLES:
 *   S1  – Mutex / AppState sincronización
 *   S2  – Máquina de estados del inversor BAMOCAR
 *   S3  – Secuencia de arranque (BOOT → PRECHARGE → R2D → RUN)
 *   S4  – CAN RX parsing (todos los IDs del proyecto)
 *   S5  – CAN TX pack/unpack round-trip
 *   S6  – Lógica de control y cálculo de torque (valores ADC reales)
 *   S7  – Seguridad EV2.3 + plausibilidad APPS
 *   S8  – Pipeline completo: sensor → control → CAN → telemetría
 *   S9  – Concurrencia FreeRTOS (sin deadlocks, sin race conditions)
 *   S10 – Estrés y casos límite
 ******************************************************************************
 */

#ifndef TEST_INTEGRATION_H
#define TEST_INTEGRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "app_state.h"
#include "control.h"
#include "can.h"

/* -------------------------------------------------------------------------- */
/* DETECCIÓN AUTOMÁTICA DE MODO                                               */
/* -------------------------------------------------------------------------- */
#if !defined(TEST_MODE_SIL) && !defined(TEST_MODE_HIL)
  #define TEST_MODE_HIL   /* Default: hardware real */
#endif

/* -------------------------------------------------------------------------- */
/* CONSTANTES DE PROYECTO (alineadas con VCU.h / can.c)                      */
/* -------------------------------------------------------------------------- */

/* IDs CAN Inversor BAMOCAR (desde perspectiva ECU) */
#define TINT_RXID_INV          0x201u   /* ECU recibe desde el inversor       */
#define TINT_TXID_INV          0x181u   /* ECU envía al inversor              */

/* IDs CAN estados inversor (TX_STATE_x del inversor) */
#define TINT_TX_STATE_2        0x461u
#define TINT_TX_STATE_4        0x463u
#define TINT_TX_STATE_5        0x464u
#define TINT_TX_STATE_6        0x465u
#define TINT_TX_STATE_7        0x466u

/* IDs CAN sensores / ACU */
#define TINT_ID_ACK_PRECARGA   0x020u
#define TINT_ID_DC_BUS_V       0x100u
#define TINT_ID_S1_ACEL        0x101u
#define TINT_ID_S2_ACEL        0x102u
#define TINT_ID_S_FRENO        0x103u
#define TINT_ID_V_CELDA_MIN    0x12Cu

/* Umbrales ADC (del sensor físico, alineados con Control_ComputeTorque) */
/* s_pct = (raw - 2050) / 9  para s1; raw=2050 → 0%, raw=2950 → 100%  */
#define TINT_ADC_S1_0PCT       2050u   /* 0% acelerador s1                   */
#define TINT_ADC_S1_25PCT      2275u   /* 25% acelerador s1                  */
#define TINT_ADC_S1_50PCT      2500u   /* 50% acelerador s1                  */
#define TINT_ADC_S1_100PCT     2950u   /* 100% acelerador s1                 */
/* s2_pct = (raw - 1915) / 6.55  para s2 */
#define TINT_ADC_S2_0PCT       1915u   /* 0% acelerador s2                   */
#define TINT_ADC_S2_25PCT      2079u   /* 25% acelerador s2                  */
#define TINT_ADC_S2_50PCT      2243u   /* 50% acelerador s2                  */
#define TINT_ADC_S2_100PCT     2570u   /* 100% acelerador s2 (aprox)         */
#define TINT_ADC_FRENO_OFF     0u      /* Freno sin pulsar                   */
#define TINT_ADC_FRENO_ON      3500u   /* Freno pulsado (> UMBRAL_FRENO_APPS=3000) */

/* -------------------------------------------------------------------------- */
/* ESTRUCTURA DE RESULTADO                                                    */
/* -------------------------------------------------------------------------- */
typedef struct {
  uint32_t total;
  uint32_t passed;
  uint32_t failed;
  uint32_t execution_time_ms;
  char     mode[8];           /* "SIL" o "HIL"                               */
} test_result_t;

/* -------------------------------------------------------------------------- */
/* PUNTO DE ENTRADA PRINCIPAL                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Ejecuta TODAS las suites de integración y genera informe completo.
 *         Imprime resumen final por Diag_Log.
 * @return Estructura con estadísticas detalladas.
 */
test_result_t Test_IntegrationRunAll(void);

/* -------------------------------------------------------------------------- */
/* SUITES INDIVIDUALES                                                        */
/* -------------------------------------------------------------------------- */

/** S1: Mutex y sincronización de AppState */
uint32_t test_suite_state_machine(void);

/** S2: Máquina de estados del inversor BAMOCAR (estados 2,4,5,6,7) */
uint32_t test_suite_inverter_state_machine(void);

/** S3: Secuencia de arranque completa BOOT→PRECHARGE→R2D→RUN */
uint32_t test_suite_boot_sequence(void);

/** S4: CAN RX – parsing de todos los IDs del proyecto */
uint32_t test_suite_can_rx_parsing(void);

/** S5: CAN TX – pack/unpack round-trip + integridad de datos */
uint32_t test_suite_can_tx_packing(void);

/** S6: Control Logic – torque con valores ADC reales del sensor físico */
uint32_t test_suite_control_logic(void);

/** S7: Seguridad EV2.3 y plausibilidad APPS (freno + acelerador) */
uint32_t test_suite_safety_logic(void);

/** S8: Pipeline completo sensor → control → CAN → telemetría */
uint32_t test_suite_full_pipeline(void);

/** S9: Concurrencia FreeRTOS (ausencia de deadlock/race) */
uint32_t test_suite_concurrency(void);

/** S10: Estrés – transiciones rápidas, desbordamiento de cola, 1000ms */
uint32_t test_suite_stress_testing(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_INTEGRATION_H */
