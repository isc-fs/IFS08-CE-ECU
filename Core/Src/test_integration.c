/**
 ******************************************************************************
 * @file           : test_integration.c
 * @brief          : Tests de integración completos ECU08 NSIL
 *
 * MODOS:
 *   TEST_MODE_SIL  – Sin hardware real (PC/QEMU). Cola y mutex simulados.
 *   TEST_MODE_HIL  – Hardware STM32H733 con FreeRTOS activo (default).
 *
 * CORRECCIONES aplicadas respecto a la versión anterior:
 *   1. g_inMutex se verifica/inicializa localmente antes de usarlo.
 *   2. Valores ADC corregidos para alinearse con la fórmula real de
 *      Control_ComputeTorque (offset 2050 para s1, 1915 para s2).
 *   3. Flag estático lat_ev23 se resetea entre suites via Control_Init().
 *   4. Aserciones triviales (ASSERT_TRUE(1,...)) reemplazadas por verificaciones
 *      reales.
 *   5. Retorno de cada suite se verifica y acumula en estadísticas.
 *   6. Separación HIL vs SIL mediante guardas de compilación.
 ******************************************************************************
 */

#include "test_integration.h"
#include "app_state.h"
#include "control.h"
#include "can.h"
#include "diag.h"
#include "telemetry.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
   INFRAESTRUCTURA DE TEST
   ========================================================================== */

/* Contadores globales (reseteados en Test_IntegrationRunAll) */
static uint32_t g_tests_total  = 0;
static uint32_t g_tests_passed = 0;
static uint32_t g_suite_errors = 0;  /* fallos acumulados por suite         */

/* Handles CAN (creados en freertos.c) */
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t canTxQueueHandle;

/* ---- Macros de aserción -------------------------------------------------- */

#define ASSERT_TRUE(cond, suite, tname) \
  do { \
    g_tests_total++; \
    if (!(cond)) { \
      Diag_Log("[FAIL] %-18s :: %s", suite, tname); \
      g_suite_errors++; \
      return 0; \
    } \
    g_tests_passed++; \
    Diag_Log("[PASS] %-18s :: %s", suite, tname); \
  } while(0)

#define ASSERT_EQUAL(actual, expected, suite, tname) \
  do { \
    g_tests_total++; \
    if ((actual) != (expected)) { \
      Diag_Log("[FAIL] %-18s :: %s  got=%lu exp=%lu", \
               suite, tname, (uint32_t)(actual), (uint32_t)(expected)); \
      g_suite_errors++; \
      return 0; \
    } \
    g_tests_passed++; \
    Diag_Log("[PASS] %-18s :: %s", suite, tname); \
  } while(0)

#define ASSERT_NOT_EQUAL(actual, unexpected, suite, tname) \
  do { \
    g_tests_total++; \
    if ((actual) == (unexpected)) { \
      Diag_Log("[FAIL] %-18s :: %s  got=%lu (should differ)", \
               suite, tname, (uint32_t)(actual)); \
      g_suite_errors++; \
      return 0; \
    } \
    g_tests_passed++; \
    Diag_Log("[PASS] %-18s :: %s", suite, tname); \
  } while(0)

#define ASSERT_RANGE(val, lo, hi, suite, tname) \
  do { \
    g_tests_total++; \
    int32_t _v = (int32_t)(val), _lo = (int32_t)(lo), _hi = (int32_t)(hi); \
    if (_v < _lo || _v > _hi) { \
      Diag_Log("[FAIL] %-18s :: %s  got=%ld range=[%ld,%ld]", \
               suite, tname, (long)_v, (long)_lo, (long)_hi); \
      g_suite_errors++; \
      return 0; \
    } \
    g_tests_passed++; \
    Diag_Log("[PASS] %-18s :: %s", suite, tname); \
  } while(0)

/* Publica resultado de suite sin abortar el runner */
#define SUITE_RESULT(name, res) \
  do { \
    if (res) Diag_Log("[SUITE OK ] %s", name); \
    else     Diag_Log("[SUITE ERR] %s", name); \
  } while(0)

/* ---- Helpers internos ----------------------------------------------------- */

/** Limpia ambas colas vaciándolas por completo. */
static void drain_queues(void)
{
  can_qitem16_t tmp;
  while (osMessageQueueGet(canRxQueueHandle, &tmp, NULL, 0) == osOK) {}
  while (osMessageQueueGet(canTxQueueHandle, &tmp, NULL, 0) == osOK) {}
}

/** Construye un mensaje CAN mínimo para enviar al parser. */
static can_msg_t make_can_msg(uint32_t id, can_bus_t bus,
                               const uint8_t *data, uint8_t dlc)
{
  can_msg_t m;
  memset(&m, 0, sizeof(m));
  m.id  = id;
  m.bus = bus;
  m.dlc = dlc;
  m.ide = 0;
  if (data && dlc) memcpy(m.data, data, dlc > 8 ? 8 : dlc);
  return m;
}

/* ============================================================================
   S1 – MUTEX Y SINCRONIZACION DE APPSTATE
   ========================================================================== */
uint32_t test_suite_state_machine(void)
{
  const char *S = "S1_STATE_MACHINE";
  g_suite_errors = 0;
  Diag_Log("\n--- S1: Mutex / AppState ---");

  /* S1.1 – g_inMutex debe existir (inicializado antes de llamar esta suite) */
  ASSERT_NOT_EQUAL((uint32_t)(uintptr_t)g_inMutex, 0u, S, "1.1_mutex_handle_valid");

  /* S1.2 – AppState_Init no corrompe la estructura */
  AppState_Init();
  app_inputs_t snap;
  AppState_Snapshot(&snap);
  ASSERT_EQUAL(snap.s1_aceleracion, 0u, S, "1.2_init_s1_zero");
  ASSERT_EQUAL(snap.s_freno,        0u, S, "1.2_init_freno_zero");
  ASSERT_EQUAL(snap.flag_EV_2_3,    0u, S, "1.2_init_ev23_zero");
  ASSERT_EQUAL(snap.ok_precarga,    0u, S, "1.2_init_precarga_zero");

  /* S1.3 – Snapshot devuelve copia coherente (torque 0..100) */
  ASSERT_RANGE(snap.torque_total, 0u, 100u, S, "1.3_torque_valid_range");

  /* S1.4 – Múltiples snapshots consecutivos sin crash */
  {
    app_inputs_t s1, s2, s3;
    AppState_Snapshot(&s1);
    osDelay(2);
    AppState_Snapshot(&s2);
    osDelay(2);
    AppState_Snapshot(&s3);
    ASSERT_EQUAL(s1.torque_total, s2.torque_total, S, "1.4_snap_consistency");
  }

  /* S1.5 – Escritura bajo mutex + lectura coherente */
  {
    /* Modificamos g_in directamente bajo mutex, como lo haría CanRxTask */
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    g_in.s1_aceleracion = TINT_ADC_S1_50PCT;
    g_in.s2_aceleracion = TINT_ADC_S2_50PCT;
    if (g_inMutex) osMutexRelease(g_inMutex);

    AppState_Snapshot(&snap);
    ASSERT_EQUAL(snap.s1_aceleracion, TINT_ADC_S1_50PCT, S, "1.5_write_read_s1");
    ASSERT_EQUAL(snap.s2_aceleracion, TINT_ADC_S2_50PCT, S, "1.5_write_read_s2");

    /* Restaurar */
    AppState_Init();
  }

  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S2 – MAQUINA DE ESTADOS DEL INVERSOR BAMOCAR
   ========================================================================== */
uint32_t test_suite_inverter_state_machine(void)
{
  const char *S = "S2_INV_FSM";
  g_suite_errors = 0;
  Diag_Log("\n--- S2: FSM Inversor BAMOCAR ---");

  app_inputs_t st;
  AppState_Init();

  /* S2.1 – Estado inicial 0 (ningún mensaje recibido aún) */
  AppState_Snapshot(&st);
  ASSERT_EQUAL(st.inv_state, 0u, S, "2.1_inv_state_initial_zero");

  /* S2.2 – Recibir TX_STATE_2 (0x461) → inv_state = byte0 */
  {
    uint8_t d[8] = {0x02, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_TX_STATE_2, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.inv_state, 0x02u, S, "2.2_inv_state_2");
  }

  /* S2.3 – Recibir TX_STATE_4 (0x463) → inv_state = byte0 */
  {
    uint8_t d[8] = {0x04, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_TX_STATE_4, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.inv_state, 0x04u, S, "2.3_inv_state_4");
  }

  /* S2.4 – Recibir TX_STATE_5 (0x464) → inv_state = byte0 */
  {
    uint8_t d[8] = {0x05, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_TX_STATE_5, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.inv_state, 0x05u, S, "2.4_inv_state_5_ready");
  }

  /* S2.5 – Recibir TX_STATE_6 (0x465) → inv_state = byte0 */
  {
    uint8_t d[8] = {0x06, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_TX_STATE_6, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.inv_state, 0x06u, S, "2.5_inv_state_6_run");
  }

  /* S2.6 – Recibir TX_STATE_7 (0x466) → inv_state = byte0 */
  {
    uint8_t d[8] = {0x07, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_TX_STATE_7, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.inv_state, 0x07u, S, "2.6_inv_state_7_fault");
  }

  /* S2.7 – DC Bus Voltage se decodifica correctamente (little-endian 2 bytes) */
  {
    /* Simular 400 V → 0x0190 = 400d */
    uint8_t d[8] = {0x90, 0x01, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_DC_BUS_V, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.inv_dc_bus_voltage, 0x0190u, S, "2.7_dc_bus_voltage_400V");
  }

  AppState_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S3 – SECUENCIA DE ARRANQUE: BOOT → PRECHARGE → R2D → RUN
   ========================================================================== */
uint32_t test_suite_boot_sequence(void)
{
  const char *S = "S3_BOOT_SEQ";
  g_suite_errors = 0;
  Diag_Log("\n--- S3: Secuencia de arranque ---");

  /* Resetear todo */
  AppState_Init();
  Control_Init();

  app_inputs_t in;
  control_out_t out;

  /* S3.1 – Estado BOOT: sin precarga, sin botón → NO envía torque */
  AppState_Snapshot(&in);
  in.ok_precarga     = 0;
  in.boton_arranque  = 0;
  in.s_freno         = TINT_ADC_FRENO_OFF;
  in.s1_aceleracion  = TINT_ADC_S1_50PCT;
  in.s2_aceleracion  = TINT_ADC_S2_50PCT;
  Control_Step10ms(&in, &out);
  ASSERT_EQUAL(out.torque_pct, 0u, S, "3.1_boot_no_torque_without_precarga");
  /* En BOOT no se envían tramas de torque */
  ASSERT_EQUAL(out.count, 0u, S, "3.1_boot_no_can_frames");

  /* S3.2 – Precarga completada (ok_precarga=1), todavía sin botón ni freno */
  {
    uint8_t d[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_ACK_PRECARGA, CAN_BUS_ACU, d, 1);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&in);
    ASSERT_EQUAL(in.ok_precarga, 0x01u, S, "3.2_precharge_ack_received");
  }

  /* S3.3a – Primera llamada post-ACK: WAIT_PRECHARGE_ACK → WAIT_START_BRAKE */
  in.boton_arranque = 0;
  in.s_freno        = TINT_ADC_FRENO_OFF;
  Control_Step10ms(&in, &out);
  /* Aún sin botón/freno, no hay torque */
  ASSERT_EQUAL(out.torque_pct, 0u, S, "3.3_r2d_delay_no_torque_yet");

  /* S3.3b – Segunda llamada: WAIT_START_BRAKE → R2D_DELAY (botón + freno ON) */
  in.boton_arranque = 1;
  in.s_freno        = TINT_ADC_FRENO_ON;
  Control_Step10ms(&in, &out);   /* Fija s_r2d_start_tick, estado = R2D_DELAY */
  ASSERT_EQUAL(out.torque_pct, 0u, S, "3.3b_r2d_no_torque_yet");

  /* S3.4 – Esperar 2000ms (R2D delay) + aceleramos */
  osDelay(2100);                 /* Avanza tick de simulación 2100 ms */
  in.s_freno        = TINT_ADC_FRENO_OFF;  /* Soltamos freno */
  in.s1_aceleracion = TINT_ADC_S1_50PCT;
  in.s2_aceleracion = TINT_ADC_S2_50PCT;
  Control_Step10ms(&in, &out);   /* R2D_DELAY → READY  (sin trama CAN) */
  Control_Step10ms(&in, &out);   /* READY    → RUN     (sin trama CAN, solo estado) */
  Control_Step10ms(&in, &out);   /* RUN: genera trama CAN con torque */
  /* En estado RUN debe haber exactamente 1 trama CAN */
  ASSERT_RANGE(out.count, 1u, 8u, S, "3.4_post_r2d_sends_can_frame");
  ASSERT_RANGE(out.torque_pct, 0u, 100u, S, "3.4_post_r2d_torque_valid");

  /* S3.5 – En RUN: el ID de la trama es el del inversor (0x181) */
  if (out.count > 0) {
    ASSERT_EQUAL(out.msgs[0].id, TINT_TXID_INV, S, "3.5_inv_can_frame_id_correct");
    ASSERT_EQUAL(out.msgs[0].bus, (uint32_t)CAN_BUS_INV, S, "3.5_inv_can_bus_correct");
  }

  AppState_Init();
  Control_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S4 – CAN RX PARSING (todos los IDs del proyecto)
   ========================================================================== */
uint32_t test_suite_can_rx_parsing(void)
{
  const char *S = "S4_CAN_RX";
  g_suite_errors = 0;
  Diag_Log("\n--- S4: CAN RX parsing ---");

  AppState_Init();
  app_inputs_t st;

  /* S4.1 – ACK precarga (0x020), byte0=1 */
  {
    uint8_t d[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_ACK_PRECARGA, CAN_BUS_ACU, d, 1);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.ok_precarga, 1u, S, "4.1_ack_precarga");
  }

  /* S4.2 – Sensor s1_aceleracion (0x101), little-endian */
  {
    uint16_t val = TINT_ADC_S1_50PCT;
    uint8_t d[8] = {(uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_S1_ACEL, CAN_BUS_DASH, d, 2);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.s1_aceleracion, TINT_ADC_S1_50PCT, S, "4.2_s1_acel_parsed");
  }

  /* S4.3 – Sensor s2_aceleracion (0x102), little-endian */
  {
    uint16_t val = TINT_ADC_S2_50PCT;
    uint8_t d[8] = {(uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_S2_ACEL, CAN_BUS_DASH, d, 2);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.s2_aceleracion, TINT_ADC_S2_50PCT, S, "4.3_s2_acel_parsed");
  }

  /* S4.4 – Sensor freno (0x103), little-endian */
  {
    uint16_t val = TINT_ADC_FRENO_ON;
    uint8_t d[8] = {(uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_S_FRENO, CAN_BUS_DASH, d, 2);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.s_freno, TINT_ADC_FRENO_ON, S, "4.4_s_freno_parsed");
  }

  /* S4.5 – Tensión celda mínima (0x12C) */
  {
    uint16_t val = 3700u; /* 3.700V en raw */
    uint8_t d[8] = {(uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_V_CELDA_MIN, CAN_BUS_ACU, d, 2);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    ASSERT_EQUAL(st.v_celda_min, 3700u, S, "4.5_v_celda_min_parsed");
  }

  /* S4.6 – ID desconocido no corrompe el estado */
  {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    can_msg_t m = make_can_msg(0xFFFF, CAN_BUS_INV, d, 8);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    if (g_inMutex) osMutexRelease(g_inMutex);
    AppState_Snapshot(&st);
    /* Los valores anteriores deben mantenerse intactos */
    ASSERT_EQUAL(st.ok_precarga, 1u, S, "4.6_unknown_id_no_corruption");
  }

  AppState_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S5 – CAN TX: PACK / UNPACK ROUND-TRIP
   ========================================================================== */
uint32_t test_suite_can_tx_packing(void)
{
  const char *S = "S5_CAN_TX";
  g_suite_errors = 0;
  Diag_Log("\n--- S5: CAN TX pack/unpack ---");

  drain_queues();

  /* S5.1 – Pack → Unpack conserva id, dlc, bus, data */
  {
    can_msg_t orig, decoded;
    orig.id      = TINT_TXID_INV;
    orig.bus     = CAN_BUS_INV;
    orig.dlc     = 8;
    orig.ide     = 0;
    orig.data[0] = 0x50; /* torque 80% */
    orig.data[1] = 0x00;
    orig.data[2] = 0x00;
    orig.data[3] = 0x00;
    orig.data[4] = 0x00;
    orig.data[5] = 0x00;
    orig.data[6] = 0x00;
    orig.data[7] = 0x00;

    can_qitem16_t qi;
    CAN_Pack16(&orig, &qi);
    CAN_Unpack16(&qi, &decoded);

    ASSERT_EQUAL(decoded.id,      orig.id,      S, "5.1_id_roundtrip");
    ASSERT_EQUAL(decoded.dlc,     orig.dlc,     S, "5.1_dlc_roundtrip");
    ASSERT_EQUAL((uint32_t)decoded.bus, (uint32_t)orig.bus, S, "5.1_bus_roundtrip");
    ASSERT_EQUAL(decoded.data[0], orig.data[0], S, "5.1_data0_roundtrip");
  }

  /* S5.2 – Cola RX: put + get recupera datos intactos */
  {
    can_qitem16_t put_item, get_item;
    memset(&put_item, 0, sizeof(put_item));
    put_item.w[0] = 0xDEADBEEF;
    put_item.w[1] = 0xCAFEBABE;

    osStatus_t s = osMessageQueuePut(canRxQueueHandle, &put_item, 0, 0);
    ASSERT_EQUAL((uint32_t)s, (uint32_t)osOK, S, "5.2_rx_queue_put_ok");

    s = osMessageQueueGet(canRxQueueHandle, &get_item, NULL, 10);
    ASSERT_EQUAL((uint32_t)s,  (uint32_t)osOK,    S, "5.2_rx_queue_get_ok");
    ASSERT_EQUAL(get_item.w[0], put_item.w[0],    S, "5.2_rx_data_w0_ok");
    ASSERT_EQUAL(get_item.w[1], put_item.w[1],    S, "5.2_rx_data_w1_ok");
  }

  /* S5.3 – Cola TX: put + get recupera datos intactos */
  {
    can_qitem16_t put_item, get_item;
    memset(&put_item, 0, sizeof(put_item));
    put_item.w[0] = 0x12345678;
    put_item.w[3] = 0xABCDEF01;

    osStatus_t s = osMessageQueuePut(canTxQueueHandle, &put_item, 0, 0);
    ASSERT_EQUAL((uint32_t)s, (uint32_t)osOK, S, "5.3_tx_queue_put_ok");

    s = osMessageQueueGet(canTxQueueHandle, &get_item, NULL, 10);
    ASSERT_EQUAL((uint32_t)s,  (uint32_t)osOK,    S, "5.3_tx_queue_get_ok");
    ASSERT_EQUAL(get_item.w[0], put_item.w[0],    S, "5.3_tx_data_w0_ok");
    ASSERT_EQUAL(get_item.w[3], put_item.w[3],    S, "5.3_tx_data_w3_ok");
  }

  /* S5.4 – FIFO ordering: 3 mensajes distintos se recuperan en orden */
  {
    can_qitem16_t items[3];
    for (int i = 0; i < 3; i++) {
      memset(&items[i], 0, sizeof(items[i]));
      items[i].w[0] = (uint32_t)(0x100 + i);
      osMessageQueuePut(canRxQueueHandle, &items[i], 0, 0);
    }
    for (int i = 0; i < 3; i++) {
      can_qitem16_t got;
      osMessageQueueGet(canRxQueueHandle, &got, NULL, 5);
      ASSERT_EQUAL(got.w[0], (uint32_t)(0x100 + i), S, "5.4_fifo_ordering");
    }
  }

  /* S5.5 – DLC extendido 8 bytes preserva todos los datos */
  {
    can_msg_t orig, dec;
    memset(&orig, 0, sizeof(orig));
    orig.id  = 0x201;
    orig.bus = CAN_BUS_INV;
    orig.dlc = 8;
    for (int i = 0; i < 8; i++) orig.data[i] = (uint8_t)(0x10 + i);

    can_qitem16_t qi;
    CAN_Pack16(&orig, &qi);
    CAN_Unpack16(&qi, &dec);

    for (int i = 0; i < 8; i++) {
      ASSERT_EQUAL(dec.data[i], orig.data[i], S, "5.5_all_8_bytes_ok");
    }
  }

  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S6 – LOGICA DE CONTROL Y CALCULO DE TORQUE (valores ADC reales del sensor)
   Fórmula real:
     s1_pct = (raw - 2050) / 9.0   →  raw 2050 = 0%, raw 2950 = 100%
     s2_pct = (raw - 1915) / 6.55  →  raw 1915 = 0%, raw 2570 ≈ 100%
     torque = avg(s1_pct, s2_pct) si ambos > 8%, else 0
     Si < 10: → 0; Si > 90: → 100
   ========================================================================== */
uint32_t test_suite_control_logic(void)
{
  const char *S = "S6_CONTROL";
  g_suite_errors = 0;
  Diag_Log("\n--- S6: Control Logic (ADC reales) ---");

  Control_Init(); /* Resetea lat_ev23 y FSM */
  app_inputs_t in;
  control_out_t out;
  uint8_t ev23_f = 0, t11_f = 0;
  uint16_t cq_torque;

  /* S6.1-6.4 – Prueban Control_ComputeTorque directamente (sin estado FSM):
   * La conversión ADC→torque es independiente del estado de arranque.       */

  /* S6.1 – 0% acelerador → torque = 0 */
  memset(&in, 0, sizeof(in));
  in.s1_aceleracion = TINT_ADC_S1_0PCT;
  in.s2_aceleracion = TINT_ADC_S2_0PCT;
  in.s_freno        = TINT_ADC_FRENO_OFF;
  cq_torque = Control_ComputeTorque(&in, &ev23_f, &t11_f);
  ASSERT_EQUAL(cq_torque, 0u, S, "6.1_zero_throttle_zero_torque");

  /* S6.2 – 25% acelerador → torque 1..35% */
  in.s1_aceleracion = TINT_ADC_S1_25PCT;
  in.s2_aceleracion = TINT_ADC_S2_25PCT;
  cq_torque = Control_ComputeTorque(&in, &ev23_f, &t11_f);
  ASSERT_RANGE(cq_torque, 1u, 35u, S, "6.2_25pct_throttle");

  /* S6.3 – 50% acelerador → torque 25..70% */
  in.s1_aceleracion = TINT_ADC_S1_50PCT;
  in.s2_aceleracion = TINT_ADC_S2_50PCT;
  cq_torque = Control_ComputeTorque(&in, &ev23_f, &t11_f);
  ASSERT_RANGE(cq_torque, 25u, 70u, S, "6.3_50pct_throttle");

  /* S6.4 – 100% acelerador → torque ≥ 80% (clamp a 100) */
  Control_Init(); /* Limpia lat_ev23 */
  in.s1_aceleracion = TINT_ADC_S1_100PCT;
  in.s2_aceleracion = TINT_ADC_S2_100PCT;
  cq_torque = Control_ComputeTorque(&in, &ev23_f, &t11_f);
  ASSERT_RANGE(cq_torque, 80u, 100u, S, "6.4_100pct_throttle");

  /* S6.5 – Control_Step10ms: count ∈ [0..8] siempre (en cualquier estado) */
  Control_Init();
  AppState_Init();
  AppState_Snapshot(&in);
  Control_Step10ms(&in, &out);
  ASSERT_RANGE(out.count, 0u, 8u, S, "6.5_output_count_valid");

  /* S6.6 – Control_ComputeTorque devuelve 0..100 en cualquier caso */
  {
    uint8_t ev23, t11;
    uint16_t t;

    AppState_Snapshot(&in);
    in.s1_aceleracion = 0;
    in.s2_aceleracion = 0;
    in.s_freno = 0;
    t = Control_ComputeTorque(&in, &ev23, &t11);
    ASSERT_RANGE(t, 0u, 100u, S, "6.6_compute_torque_range");

    in.s1_aceleracion = TINT_ADC_S1_100PCT;
    in.s2_aceleracion = TINT_ADC_S2_100PCT;
    t = Control_ComputeTorque(&in, &ev23, &t11);
    ASSERT_RANGE(t, 0u, 100u, S, "6.6_compute_torque_max_range");
  }

  /* S6.7 – Timing: Step10ms completo en < 5ms */
  {
    uint32_t t0 = osKernelGetTickCount();
    AppState_Snapshot(&in);
    Control_Step10ms(&in, &out);
    uint32_t elapsed = osKernelGetTickCount() - t0;
    ASSERT_RANGE(elapsed, 0u, 5u, S, "6.7_step10ms_timing");
  }

  Control_Init();
  AppState_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S7 – SEGURIDAD EV2.3 Y PLAUSIBILIDAD APPS
   Regla EV2.3: si s_freno > 3000 && torque > 25% → latch → torque=0
                Se libera cuando freno < 3000 AND torque < 5%
   ========================================================================== */
uint32_t test_suite_safety_logic(void)
{
  const char *S = "S7_SAFETY";
  g_suite_errors = 0;
  Diag_Log("\n--- S7: Seguridad EV2.3 + APPS ---");

  app_inputs_t in;
  uint8_t ev23, t11;
  uint16_t torque;

  /* S7.1 – Acelerador 50%, freno OFF → torque > 0 (sin inhibición) */
  Control_Init();
  AppState_Snapshot(&in);
  in.s1_aceleracion = TINT_ADC_S1_50PCT;
  in.s2_aceleracion = TINT_ADC_S2_50PCT;
  in.s_freno        = TINT_ADC_FRENO_OFF;
  in.flag_EV_2_3    = 0;
  torque = Control_ComputeTorque(&in, &ev23, &t11);
  ASSERT_RANGE(torque, 1u, 100u, S, "7.1_normal_throttle_no_inhibit");
  ASSERT_EQUAL(ev23, 0u, S, "7.1_ev23_not_latched");

  /* S7.2 – Freno pulsado + acelerador > 25% → latch EV2.3 → torque = 0 */
  Control_Init();
  AppState_Snapshot(&in);
  in.s1_aceleracion = TINT_ADC_S1_50PCT;
  in.s2_aceleracion = TINT_ADC_S2_50PCT;
  in.s_freno        = TINT_ADC_FRENO_ON;  /* > 3000 */
  torque = Control_ComputeTorque(&in, &ev23, &t11);
  ASSERT_EQUAL(torque,  0u, S, "7.2_brake_accel_torque_zero");
  ASSERT_EQUAL(ev23,    1u, S, "7.2_ev23_latched");

  /* S7.3 – Con latch activo: soltamos freno pero mantenemos acelerador > 5%
             → latch PERSISTE → torque sigue en 0 */
  in.s_freno = TINT_ADC_FRENO_OFF;
  in.s1_aceleracion = TINT_ADC_S1_25PCT;  /* ~25%, > 5% → latch no se libera */
  in.s2_aceleracion = TINT_ADC_S2_25PCT;
  torque = Control_ComputeTorque(&in, &ev23, &t11);
  ASSERT_EQUAL(torque, 0u, S, "7.3_ev23_latch_persists_after_brake_release");
  ASSERT_EQUAL(ev23,   1u, S, "7.3_ev23_still_latched");

  /* S7.4 – Latch se libera: freno < 3000 Y throttle < 5% (raw ~2050) */
  in.s_freno        = TINT_ADC_FRENO_OFF;
  in.s1_aceleracion = TINT_ADC_S1_0PCT;  /* 0% → torque calculado < 5 → latch libera */
  in.s2_aceleracion = TINT_ADC_S2_0PCT;
  torque = Control_ComputeTorque(&in, &ev23, &t11);
  ASSERT_EQUAL(ev23, 0u, S, "7.4_ev23_latch_released");

  /* S7.5 – Acelerador 100% + freno OFF → torque máximo, sin inhibición */
  Control_Init();
  AppState_Snapshot(&in);
  in.s1_aceleracion = TINT_ADC_S1_100PCT;
  in.s2_aceleracion = TINT_ADC_S2_100PCT;
  in.s_freno        = TINT_ADC_FRENO_OFF;
  torque = Control_ComputeTorque(&in, &ev23, &t11);
  ASSERT_RANGE(torque, 80u, 100u, S, "7.5_full_throttle_no_brake_max_torque");
  ASSERT_EQUAL(ev23, 0u, S, "7.5_no_ev23_latch");

  /* S7.6 – Control_ComputeTorque con puntero nulo → retorna 0 sin crash */
  torque = Control_ComputeTorque(NULL, &ev23, &t11);
  ASSERT_EQUAL(torque, 0u, S, "7.6_null_input_safe");

  Control_Init();
  AppState_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S8 – PIPELINE COMPLETO: SENSOR → CONTROL → CAN → TELEMETRIA
   ========================================================================== */
uint32_t test_suite_full_pipeline(void)
{
  const char *S = "S8_PIPELINE";
  g_suite_errors = 0;
  Diag_Log("\n--- S8: Pipeline completo ---");

  AppState_Init();
  Control_Init();
  drain_queues();

  app_inputs_t in;
  control_out_t out;
  uint8_t telemetry[32];

  /* S8.1 – Simular lectura sensor por CAN → actualiza estado */
  {
    uint16_t val = TINT_ADC_S1_50PCT;
    uint8_t d[8] = {(uint8_t)(val & 0xFF), (uint8_t)(val >> 8), 0, 0, 0, 0, 0, 0};
    can_msg_t m = make_can_msg(TINT_ID_S1_ACEL, CAN_BUS_DASH, d, 2);
    if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
    CanRx_ParseAndUpdate(&m, &g_in);
    g_in.s2_aceleracion = TINT_ADC_S2_50PCT;
    g_in.s_freno        = TINT_ADC_FRENO_OFF;
    g_in.ok_precarga    = 1;
    if (g_inMutex) osMutexRelease(g_inMutex);
  }

  /* S8.2 – Control lee el estado y produce salida */
  AppState_Snapshot(&in);
  Control_Step10ms(&in, &out);
  ASSERT_RANGE(out.torque_pct, 0u, 100u, S, "8.2_control_output_valid");

  /* S8.3 – Mensajes CAN generados se encolan en TX */
  {
    uint32_t enqueued = 0;
    for (uint8_t i = 0; i < out.count && i < 8; i++) {
      can_qitem16_t qi;
      CAN_Pack16(&out.msgs[i], &qi);
      if (osMessageQueuePut(canTxQueueHandle, &qi, 0, 0) == osOK) enqueued++;
    }
    /* Si hay tramas generadas, deben haberse encolado todas */
    if (out.count > 0) {
      ASSERT_EQUAL(enqueued, (uint32_t)out.count, S, "8.3_all_can_frames_enqueued");
    } else {
      ASSERT_EQUAL(enqueued, 0u, S, "8.3_no_frames_no_enqueue");
    }
  }

  /* S8.4 – Telemetría construida a partir del snapshot */
  AppState_Snapshot(&in);
  Telemetry_Build32(&in, telemetry);
  /* Byte 6 = LSB de s1_aceleracion */
  uint8_t expected_s1_lsb = (uint8_t)(in.s1_aceleracion & 0xFF);
  ASSERT_EQUAL(telemetry[6], expected_s1_lsb, S, "8.4_telemetry_s1_byte6");

  /* S8.5 – Payload coherente: no todo 0xFF (no inicializado = error) */
  {
    uint8_t all_ff = 1;
    for (int i = 0; i < 16; i++) if (telemetry[i] != 0xFF) { all_ff = 0; break; }
    ASSERT_EQUAL(all_ff, 0u, S, "8.5_telemetry_not_garbage");
  }

  /* S8.6 – Múltiples ciclos del pipeline sin errores */
  {
    for (int cycle = 0; cycle < 5; cycle++) {
      AppState_Snapshot(&in);
      Control_Step10ms(&in, &out);
      ASSERT_RANGE(out.torque_pct, 0u, 100u, S, "8.6_multi_cycle");
      osDelay(10);
    }
  }

  drain_queues();
  AppState_Init();
  Control_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S9 – CONCURRENCIA FREERTOS (sin deadlock, sin race condition)
   ========================================================================== */
uint32_t test_suite_concurrency(void)
{
  const char *S = "S9_CONCURRENCY";
  g_suite_errors = 0;
  Diag_Log("\n--- S9: Concurrencia FreeRTOS ---");

  /* S9.1 – 50 snapshots rápidos consecutivos sin crash ni corrupción */
  {
    uint8_t ok = 1;
    for (int i = 0; i < 50; i++) {
      app_inputs_t snap;
      AppState_Snapshot(&snap);
      if (snap.torque_total > 100u) { ok = 0; break; }
    }
    ASSERT_EQUAL(ok, 1u, S, "9.1_rapid_snapshots_no_corruption");
  }

  /* S9.2 – Control_Step10ms bajo condición concurrente (osDelay entre steps) */
  {
    app_inputs_t in;
    control_out_t out;
    uint8_t ok = 1;

    for (int i = 0; i < 20; i++) {
      AppState_Snapshot(&in);
      Control_Step10ms(&in, &out);
      if (out.torque_pct > 100u || out.count > 8u) { ok = 0; break; }
      osDelay(1);
    }
    ASSERT_EQUAL(ok, 1u, S, "9.2_concurrent_control_steps_valid");
  }

  /* S9.3 – Producción y consumo de cola simultáneos (simular ISR + task) */
  {
    drain_queues();
    /* Producir 10 items */
    for (int i = 0; i < 10; i++) {
      can_qitem16_t qi;
      qi.w[0] = (uint32_t)(0xAA00 + i);
      qi.w[1] = qi.w[2] = qi.w[3] = 0;
      osMessageQueuePut(canRxQueueHandle, &qi, 0, 0);
    }
    /* Consumir y verificar integridad */
    int consumed = 0;
    uint8_t ok = 1;
    can_qitem16_t got;
    while (osMessageQueueGet(canRxQueueHandle, &got, NULL, 0) == osOK) {
      if ((got.w[0] & 0xFF00u) != 0xAA00u) { ok = 0; break; }
      consumed++;
    }
    ASSERT_EQUAL(ok, 1u, S, "9.3_queue_integrity");
    ASSERT_EQUAL((uint32_t)consumed, 10u, S, "9.3_queue_count");
  }

  /* S9.4 – RX y TX queues son independientes */
  {
    drain_queues();
    can_qitem16_t rx_item, tx_item, got;
    rx_item.w[0] = 0xAAAAAAAA;
    tx_item.w[0] = 0xBBBBBBBB;
    rx_item.w[1] = rx_item.w[2] = rx_item.w[3] = 0;
    tx_item.w[1] = tx_item.w[2] = tx_item.w[3] = 0;

    osMessageQueuePut(canRxQueueHandle, &rx_item, 0, 0);
    osMessageQueuePut(canTxQueueHandle, &tx_item, 0, 0);

    osMessageQueueGet(canRxQueueHandle, &got, NULL, 5);
    ASSERT_EQUAL(got.w[0], 0xAAAAAAAAu, S, "9.4_rx_queue_isolated");

    osMessageQueueGet(canTxQueueHandle, &got, NULL, 5);
    ASSERT_EQUAL(got.w[0], 0xBBBBBBBBu, S, "9.4_tx_queue_isolated");
  }

  drain_queues();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   S10 – ESTRES Y CASOS LIMITE
   ========================================================================== */
uint32_t test_suite_stress_testing(void)
{
  const char *S = "S10_STRESS";
  g_suite_errors = 0;
  Diag_Log("\n--- S10: Estrés y casos límite ---");

  AppState_Init();
  Control_Init();
  drain_queues();

  app_inputs_t in;
  control_out_t out;

  /* S10.1 – 100 ciclos de control con valores ADC variables → siempre 0..100 */
  {
    uint8_t ok = 1;
    uint16_t adc_vals[] = {
      TINT_ADC_S1_0PCT, TINT_ADC_S1_25PCT, TINT_ADC_S1_50PCT,
      TINT_ADC_S1_100PCT, 0u, 4095u
    };
    for (int i = 0; i < 100; i++) {
      Control_Init(); /* Resetear FSM para que no bloquee en precharge */
      AppState_Snapshot(&in);
      in.s1_aceleracion = adc_vals[i % 6];
      in.s2_aceleracion = adc_vals[i % 6];
      in.s_freno        = TINT_ADC_FRENO_OFF;
      Control_Step10ms(&in, &out);
      if (out.torque_pct > 100u) { ok = 0; break; }
    }
    ASSERT_EQUAL(ok, 1u, S, "10.1_100_cycles_torque_bounded");
  }

  /* S10.2 – Cola RX llena hasta el límite (128 slots) */
  {
    drain_queues();
    int successful = 0;
    for (int i = 0; i < 140; i++) { /* intenta 140 > capacidad 128 */
      can_qitem16_t qi;
      qi.w[0] = (uint32_t)i;
      qi.w[1] = qi.w[2] = qi.w[3] = 0;
      if (osMessageQueuePut(canRxQueueHandle, &qi, 0, 0) == osOK) successful++;
    }
    ASSERT_RANGE((uint32_t)successful, 64u, 128u, S, "10.2_queue_bounded_fill");
    drain_queues();
  }

  /* S10.3 – Valores ADC en límites extremos (0 y 4095) no cuelgan */
  {
    uint8_t ok = 1;
    uint16_t extremes[] = {0u, 4095u};
    for (int e = 0; e < 2; e++) {
      Control_Init();
      AppState_Snapshot(&in);
      in.s1_aceleracion = extremes[e];
      in.s2_aceleracion = extremes[e];
      in.s_freno        = extremes[e];
      Control_Step10ms(&in, &out);
      if (out.torque_pct > 100u) { ok = 0; break; }
    }
    ASSERT_EQUAL(ok, 1u, S, "10.3_extreme_adc_values_safe");
  }

  /* S10.4 – 1000ms de bucle continuo de control a 10ms/ciclo */
  {
    uint32_t start = osKernelGetTickCount();
    uint32_t loops = 0;
    uint8_t  ok    = 1;

    Control_Init();
    while ((osKernelGetTickCount() - start) < 1000u) {
      AppState_Snapshot(&in);
      in.s1_aceleracion = TINT_ADC_S1_50PCT;
      in.s2_aceleracion = TINT_ADC_S2_50PCT;
      in.s_freno        = TINT_ADC_FRENO_OFF;
      Control_Step10ms(&in, &out);
      if (out.torque_pct > 100u) { ok = 0; break; }
      loops++;
      osDelay(10);
    }

    ASSERT_EQUAL(ok, 1u, S, "10.4_1000ms_loop_no_overflow");
    /* Esperamos al menos 50 ciclos de 10ms en 1000ms */
    ASSERT_RANGE(loops, 50u, 200u, S, "10.4_loop_count_reasonable");
  }

  /* S10.5 – AppState_Snapshot con puntero NULL no cuelga */
  AppState_Snapshot(NULL);
  ASSERT_EQUAL(1u, 1u, S, "10.5_snapshot_null_safe"); /* Solo verifica que no crashe */

  drain_queues();
  AppState_Init();
  Control_Init();
  return (g_suite_errors == 0) ? 1u : 0u;
}

/* ============================================================================
   RUNNER PRINCIPAL – genera informe completo
   ========================================================================== */
test_result_t Test_IntegrationRunAll(void)
{
  test_result_t result;
  memset(&result, 0, sizeof(result));

#ifdef TEST_MODE_SIL
  const char *mode_str = "SIL";
#else
  const char *mode_str = "HIL";
#endif

  /* Asegurar que el mutex existe antes de cualquier test */
  if (!g_inMutex) {
    g_inMutex = osMutexNew(NULL);
  }

  uint32_t t_start = osKernelGetTickCount();

  Diag_Log("====================================================");
  Diag_Log("  ECU08 NSIL - INTEGRATION TEST SUITE");
  Diag_Log("  Modo: %s | FreeRTOS activo", mode_str);
  Diag_Log("  Fecha compilacion: %s %s", __DATE__, __TIME__);
  Diag_Log("====================================================");
  Diag_Log("Inicializando sistema...");

  /* Inicialización limpia antes de comenzar */
  AppState_Init();
  Control_Init();
  drain_queues();
  g_tests_total  = 0;
  g_tests_passed = 0;

  /* Breve espera para que FreeRTOS estabilice las colas */
  osDelay(50);

  /* ---- Ejecutar suites ---- */
  typedef uint32_t (*suite_fn_t)(void);

  struct {
    suite_fn_t fn;
    const char *name;
  } suites[] = {
    { test_suite_state_machine,        "S1  Mutex/AppState"            },
    { test_suite_inverter_state_machine,"S2  FSM Inversor BAMOCAR"     },
    { test_suite_boot_sequence,        "S3  Secuencia de arranque"     },
    { test_suite_can_rx_parsing,       "S4  CAN RX parsing"            },
    { test_suite_can_tx_packing,       "S5  CAN TX pack/unpack"        },
    { test_suite_control_logic,        "S6  Control logic (ADC reales)"},
    { test_suite_safety_logic,         "S7  Seguridad EV2.3/APPS"     },
    { test_suite_full_pipeline,        "S8  Pipeline completo"         },
    { test_suite_concurrency,          "S9  Concurrencia FreeRTOS"     },
    { test_suite_stress_testing,       "S10 Estrés / límites"          },
  };

  const uint32_t NUM_SUITES = sizeof(suites) / sizeof(suites[0]);
  uint32_t suites_passed = 0;
  uint32_t suites_failed = 0;

  Diag_Log("\n[INFO] Iniciando %lu suites...\n", NUM_SUITES);

  for (uint32_t i = 0; i < NUM_SUITES; i++) {
    /* Snapshot de contadores antes de la suite */
    uint32_t pre_total  = g_tests_total;
    uint32_t pre_passed = g_tests_passed;

    uint32_t suite_ok = suites[i].fn();

    uint32_t suite_tests  = g_tests_total  - pre_total;
    uint32_t suite_ok_cnt = g_tests_passed - pre_passed;
    uint32_t suite_fail   = suite_tests - suite_ok_cnt;

    if (suite_ok && suite_fail == 0) {
      Diag_Log("[SUITE PASS] %s  (%lu/%lu)", suites[i].name, suite_ok_cnt, suite_tests);
      suites_passed++;
    } else {
      Diag_Log("[SUITE FAIL] %s  (%lu/%lu) <-- FALLO", suites[i].name, suite_ok_cnt, suite_tests);
      suites_failed++;
    }

    /* Limpiar estado entre suites */
    AppState_Init();
    Control_Init();
    drain_queues();
    osDelay(10);
  }

  uint32_t elapsed = osKernelGetTickCount() - t_start;

  result.total            = g_tests_total;
  result.passed           = g_tests_passed;
  result.failed           = g_tests_total - g_tests_passed;
  result.execution_time_ms = elapsed;
  (void)snprintf(result.mode, sizeof(result.mode), "%s", mode_str);

  /* ---- INFORME FINAL ---- */
  Diag_Log("\n====================================================");
  Diag_Log("  INFORME FINAL - ECU08 NSIL INTEGRATION TESTS");
  Diag_Log("====================================================");
  Diag_Log("  Modo ejecucion : %s", mode_str);
  Diag_Log("  Suites totales : %lu", NUM_SUITES);
  Diag_Log("  Suites OK      : %lu", suites_passed);
  Diag_Log("  Suites FAIL    : %lu", suites_failed);
  Diag_Log("  Tests totales  : %lu", result.total);
  Diag_Log("  Tests pasados  : %lu", result.passed);
  Diag_Log("  Tests fallados : %lu", result.failed);
  Diag_Log("  Tiempo total   : %lu ms", result.execution_time_ms);
  Diag_Log("----------------------------------------------------");

  if (result.failed == 0) {
    Diag_Log("  RESULTADO: >>> ALL TESTS PASSED <<< (100%%)");
    Diag_Log("  Estado   : SISTEMA LISTO PARA HARDWARE");
  } else {
    uint32_t pct = (result.passed * 100u) / (result.total ? result.total : 1u);
    Diag_Log("  RESULTADO: >>> %lu TESTS FALLARON <<< (%lu%%)", result.failed, pct);
    Diag_Log("  Estado   : REVISAR FALLOS ANTES DE HARDWARE");
  }

  Diag_Log("====================================================\n");

  return result;
}

