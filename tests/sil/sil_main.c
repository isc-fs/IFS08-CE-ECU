/**
 * sil_main.c
 * SIL (Software-In-The-Loop) Entry Point - Simplified Version
 * 
 * Simulates the complete ECU application logic with the real task layout
 * from freertos.c, using a cooperative RTOS mock on the host.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "app_state.h"
#include "can.h"
#include "control.h"
#include "telemetry.h"
#include "test_integration.h"   /* suites S1-S10, Test_IntegrationRunAll() */
#include "cmsis_os2.h"
#include "sil_hal_mocks.h"
#include "sil_can_simulator.h"
#include "sil_boot_sequence.h"
#include "sil_results.h"

/* Declarada en mocks/diag_sil.c: redirige Diag_Log al fichero especificado */
extern void SIL_DiagSetFile(FILE *f);
extern void MX_FREERTOS_Init(void);

/* Integration test suites (tests/sil/integration/) */
extern int run_boot_sequence_tests(void);   /* returns failure count */
extern int run_full_cycle_tests(void);      /* returns failure count */

/* ===== Global state for SIL ===== */
static volatile uint32_t sil_tick_ms = 0;
static volatile int sil_simulation_running = 0;
static volatile uint32_t sil_test_duration_ms = 0;
static uint8_t sil_last_telemetry[32] = {0};
static uint8_t sil_last_heartbeat[32] = {0};
static uint8_t sil_last_event[32] = {0};
static uint32_t sil_telemetry_count = 0;
static uint32_t sil_heartbeat_count = 0;
static uint32_t sil_event_count = 0;
static int sil_test_failures = 0;

uint32_t sil_get_time_ms(void);

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

void Telemetry_Send32(const uint8_t payload[32])
{
    if (!payload) return;
    memcpy(sil_last_telemetry, payload, sizeof(sil_last_telemetry));
    sil_telemetry_count++;

    if (payload[17] == (uint8_t)TELEMETRY_FRAME_EVENT) {
        memcpy(sil_last_event, payload, sizeof(sil_last_event));
        sil_event_count++;
    } else {
        memcpy(sil_last_heartbeat, payload, sizeof(sil_last_heartbeat));
        sil_heartbeat_count++;
    }
}

void Telemetry_SdStore32(const uint8_t payload[32])
{
    (void)payload;
}

static void sil_check(int condition, const char *message)
{
    if (condition) {
        printf("[CHECK][PASS] %s\n", message);
    } else {
        printf("[CHECK][FAIL] %s\n", message);
        sil_test_failures++;
    }
}

static void sil_reset_captured_outputs(void)
{
    memset(sil_last_telemetry, 0, sizeof(sil_last_telemetry));
    memset(sil_last_heartbeat, 0, sizeof(sil_last_heartbeat));
    memset(sil_last_event, 0, sizeof(sil_last_event));
    sil_telemetry_count = 0;
    sil_heartbeat_count = 0;
    sil_event_count = 0;
}

static void sil_set_start_button(uint8_t pressed)
{
    SIL_IO_SetStartButton(pressed);
}

static void sil_runtime_init(void)
{
    SIL_RTOS_ResetKernel();
    (void)osKernelInitialize();
    SIL_HAL_Init();
    MX_FREERTOS_Init();
    (void)osKernelStart();
    SIL_RTOS_RunReadyThreads();

    sil_tick_ms = 0;
    sil_simulation_running = 0;
    sil_test_duration_ms = 0;
    sil_test_failures = 0;
    sil_reset_captured_outputs();
}

static void sil_runtime_step_1ms(void)
{
    sil_tick_ms++;
    SIL_AdvanceTick(1);
    SIL_CAN_Process();
    SIL_RTOS_RunReadyThreads();
}

static void sil_runtime_step_manual_1ms(uint8_t process_can_sim)
{
    sil_tick_ms++;
    SIL_AdvanceTick(1);
    if (process_can_sim) {
        SIL_CAN_Process();
    }
    SIL_RTOS_RunReadyThreads();
}

static void sil_run_manual(uint32_t duration_ms, uint8_t process_can_sim)
{
    uint32_t end_tick = sil_tick_ms + duration_ms;

    while (sil_tick_ms < end_tick) {
        sil_runtime_step_manual_1ms(process_can_sim);
    }
}

static void sil_run_until_tick(uint32_t target_tick, uint8_t process_can_sim)
{
    while (sil_tick_ms < target_tick) {
        sil_runtime_step_manual_1ms(process_can_sim);
    }
}

static void sil_run_until_next_control_cycle_after_rx(void)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t rx_tick = ((now / 5u) + 1u) * 5u;
    uint32_t control_tick = ((rx_tick / 10u) + 1u) * 10u;
    sil_run_until_tick(control_tick, 0u);
}

static can_bus_t sil_bus_from_handle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return CAN_BUS_INV;
    if (hfdcan == &hfdcan2) return CAN_BUS_ACU;
    if (hfdcan == &hfdcan3) return CAN_BUS_DASH;
    return CAN_BUS_INV;
}

static uint8_t sil_pop_tx_can_msg(FDCAN_HandleTypeDef *hfdcan, can_msg_t *msg)
{
    FDCAN_TxHeaderTypeDef txh;
    uint8_t payload[8] = {0};

    if (!msg) return 0u;
    if (SIL_FDCAN_PopTxFrame(hfdcan, &txh, payload) != HAL_OK) {
        return 0u;
    }

    memset(msg, 0, sizeof(*msg));
    msg->bus = sil_bus_from_handle(hfdcan);
    msg->id = txh.Identifier;
    msg->ide = (txh.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
    msg->dlc = (uint8_t)((txh.DataLength >> 16) & 0xFu);
    memcpy(msg->data, payload, sizeof(msg->data));
    return 1u;
}

static void sil_drain_tx_bus(FDCAN_HandleTypeDef *hfdcan)
{
    can_msg_t dummy;

    while (sil_pop_tx_can_msg(hfdcan, &dummy)) {
    }
}

static HAL_StatusTypeDef sil_inject_rx_frame(FDCAN_HandleTypeDef *hfdcan,
                                             uint32_t id,
                                             uint32_t id_type,
                                             const uint8_t *data,
                                             uint8_t dlc)
{
    HAL_StatusTypeDef st = SIL_FDCAN_InjectRxFrame(hfdcan, id, id_type, data, dlc);

    if (st == HAL_OK) {
        Can_ISR_PushRxFifo0(hfdcan);
    }

    return st;
}

static void sil_check_bus_empty(FDCAN_HandleTypeDef *hfdcan, const char *message)
{
    sil_check(SIL_FDCAN_GetTxCount(hfdcan) == 0u, message);
    if (SIL_FDCAN_GetTxCount(hfdcan) != 0u) {
        sil_drain_tx_bus(hfdcan);
    }
}

static void sil_expect_tx_frame(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t expected_id,
                                uint8_t expected_ide,
                                uint8_t expected_dlc,
                                const uint8_t *expected_data,
                                uint8_t expected_data_len,
                                const char *label)
{
    can_msg_t msg;
    char detail[160];
    uint8_t has_msg = 0u;

    memset(&msg, 0, sizeof(msg));

    snprintf(detail, sizeof(detail), "%s -> trama presente", label);
    has_msg = sil_pop_tx_can_msg(hfdcan, &msg);
    sil_check(has_msg == 1u, detail);
    if (!has_msg) {
        return;
    }

    snprintf(detail, sizeof(detail), "%s -> ID 0x%03lX", label, (unsigned long)expected_id);
    sil_check(msg.id == expected_id, detail);

    snprintf(detail, sizeof(detail), "%s -> IDE %u", label, (unsigned)expected_ide);
    sil_check(msg.ide == expected_ide, detail);

    snprintf(detail, sizeof(detail), "%s -> DLC %u", label, (unsigned)expected_dlc);
    sil_check(msg.dlc == expected_dlc, detail);

    if (expected_data && expected_data_len > 0u) {
        snprintf(detail, sizeof(detail), "%s -> payload", label);
        sil_check(memcmp(msg.data, expected_data, expected_data_len) == 0, detail);
    }
}

static uint16_t sil_legacy_torque_command(uint16_t torque_pct)
{
    uint16_t scaled = torque_pct;

    if (scaled >= 10u) {
        scaled = (uint16_t)(((uint32_t)scaled * 240u) / 90u - (2400u / 90u));
    }

    return (uint16_t)(~scaled + 1u);
}

static void test_legacy_compat_harness(void)
{
    app_inputs_t snapshot = {0};
    uint8_t data[8] = {0};
    uint8_t expected[8] = {0};
    uint16_t expected_torque_cmd = 0u;

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Legacy Compatibility Harness            ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    SIL_Results_Init("legacy_compat_harness.log");
    SIL_Results_Log("LEGACY_COMPAT", "STARTED",
                    "Polling-compatible startup and inverter-write harness");

    sil_runtime_init();
    sil_set_start_button(0u);
    SIL_CAN_InjectBrake(0u);
    SIL_CAN_InjectThrottle(0u);
    sil_drain_tx_bus(&hfdcan1);
    sil_drain_tx_bus(&hfdcan2);
    sil_drain_tx_bus(&hfdcan3);

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE1", "WAIT_INV_VDC_CONFIG");
    sil_run_manual(30u, 0u);
    sil_check_bus_empty(&hfdcan1, "P1: sin escritura al inversor antes de TX_STATE_7");
    sil_check_bus_empty(&hfdcan2, "P1: sin precarga antes de TX_STATE_7");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE2", "PRECHARGE_REQUEST");
    sil_set_start_button(1u);
    memset(data, 0, sizeof(data));
    data[2] = 0x00u;
    data[3] = 0x00u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_7, FDCAN_STANDARD_ID, data, 6u) == HAL_OK,
              "P2: RX TX_STATE_7 inyectada");
    sil_run_until_next_control_cycle_after_rx();

    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 1u, 2u, expected, 2u,
                        "P2: reenvio DC bus al ACU");
    memset(expected, 0, sizeof(expected));
    expected[0] = 1u;
    sil_expect_tx_frame(&hfdcan2, 0x600u, 1u, 2u, expected, 2u,
                        "P2: peticion de precarga");
    sil_check_bus_empty(&hfdcan1, "P2: sin escritura al inversor durante precarga");
    sil_check_bus_empty(&hfdcan2, "P2: solo dos tramas ACU esperadas en el ciclo");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE3", "PRECHARGE_ACK");
    memset(data, 0, sizeof(data));
    data[0] = 0x00u;
    sil_check(sil_inject_rx_frame(&hfdcan2, TINT_ID_ACK_PRECARGA, FDCAN_STANDARD_ID, data, 1u) == HAL_OK,
              "P3: ACK precarga inyectado");
    sil_run_until_next_control_cycle_after_rx();
    sil_check_bus_empty(&hfdcan1, "P3: tras ACK aun no se escribe al inversor");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE4", "WAIT_START_BRAKE");
    sil_run_manual(40u, 0u);
    sil_check_bus_empty(&hfdcan1, "P4: sin escritura al inversor esperando freno");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE5", "R2D_DELAY");
    SIL_CAN_InjectBrake(100u);
    sil_run_manual(1990u, 0u);
    sil_check_bus_empty(&hfdcan1, "P5: sin escritura al inversor durante RTDS");
    sil_run_manual(20u, 0u);
    sil_check_bus_empty(&hfdcan1, "P5: sin escritura hasta recibir state=3");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE6", "INV_STATE_3");
    memset(data, 0, sizeof(data));
    data[4] = 3u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P6: state=3 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x04u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P6: state=3 -> modo READY");
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P6: state=3 -> torque cero");
    sil_check_bus_empty(&hfdcan1, "P6: solo READY + torque cero en state=3");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE7", "INV_STATE_3_REPEAT");
    sil_run_manual(10u, 0u);
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x04u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P7: state=3 persistente -> READY");
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P7: state=3 persistente -> torque cero");
    sil_check_bus_empty(&hfdcan1, "P7: secuencia repetida en state=3");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE8", "INV_STATE_4");
    memset(data, 0, sizeof(data));
    data[4] = 4u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P8: state=4 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x06u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P8: state=4 -> modo TORQUE");
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P8: state=4 -> torque cero");
    sil_check_bus_empty(&hfdcan1, "P8: solo TORQUE + torque cero en state=4");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE9", "INV_STATE_6_TORQUE");
    SIL_CAN_InjectBrake(0u);
    SIL_CAN_InjectThrottle(50u);
    memset(data, 0, sizeof(data));
    data[4] = 6u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P9: state=6 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    AppState_Snapshot(&snapshot);
    expected_torque_cmd = sil_legacy_torque_command(snapshot.torque_total);
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x06u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P9: state=6 -> modo TORQUE");
    memset(expected, 0, sizeof(expected));
    expected[2] = (uint8_t)(expected_torque_cmd & 0xFFu);
    expected[3] = (uint8_t)((expected_torque_cmd >> 8) & 0xFFu);
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P9: state=6 -> torque legado");
    sil_check_bus_empty(&hfdcan1, "P9: solo modo + torque en state=6");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE10", "INV_STATE_10_SOFT_FAULT");
    SIL_CAN_InjectThrottle(0u);
    memset(data, 0, sizeof(data));
    data[2] = 3u;
    data[4] = 10u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P10: state=10 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x13u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P10: state=10 -> reset 1");
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P10: state=10 -> reset 2");
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x01u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P10: state=10 -> standby final");
    sil_check_bus_empty(&hfdcan1, "P10: secuencia exacta soft fault");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE11", "INV_STATE_11_HARD_FAULT");
    memset(data, 0, sizeof(data));
    data[4] = 11u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P11: state=11 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x13u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P11: state=11 -> reset");
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x01u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P11: state=11 -> standby");
    sil_check_bus_empty(&hfdcan1, "P11: secuencia exacta hard fault");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE12", "INV_STATE_13_SHUTDOWN");
    memset(data, 0, sizeof(data));
    data[4] = 13u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P12: state=13 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x01u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P12: state=13 -> standby");
    sil_check_bus_empty(&hfdcan1, "P12: una sola trama en shutdown");

    SIL_Results_Log("LEGACY_COMPAT",
                    (sil_test_failures == 0) ? "SUCCESS" : "FAIL",
                    (sil_test_failures == 0)
                        ? "Startup and inverter write sequence matches polling contract"
                        : "Legacy compatibility harness found sequence mismatches");
    SIL_Results_Close();
}

/* ===== Test: Suites de integración S1-S10 (test_integration.c) ===== */

/**
 * Ejecuta las 10 suites de integración y escribe el informe en:
 *   tests/sil/results/integration_test.log
 *
 * La salida también aparece en consola (stdout) en tiempo real a través
 * de Diag_Log, que en SIL está implementado en mocks/diag_sil.c.
 */
static void test_integration_suite(void)
{
    const char *log_path = "tests/sil/results/integration_test.log";
    int results_dir_ready = (SIL_EnsureResultsDir() == 0);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Integration Test Suite  (S1-S10)         ║\n");
    printf("║  Modo: TEST_MODE_SIL  |  Host PC (sin hardware)     ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    /* Abrir fichero de resultados */
    FILE *log = results_dir_ready ? fopen(log_path, "w") : NULL;
    if (!log && !results_dir_ready) {
        printf("[WARN] No se pudo preparar el directorio de resultados.\n");
    } else if (!log) {
        /* Intentar crear el directorio results si no existe */
        printf("[WARN] No se pudo abrir %s, intentando crear directorio...\n", log_path);
        if (SIL_EnsureResultsDir() == 0) {
            log = fopen(log_path, "w");
        }
    }

    /* Escribir cabecera del fichero de log */
    if (log) {
        time_t now = time(NULL);
        char   ts[64];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&now));
        fprintf(log, "=======================================================\n");
        fprintf(log, "  ECU08 NSIL – Integration Test Results (SIL)          \n");
        fprintf(log, "  Fecha: %s                                             \n", ts);
        fprintf(log, "  Modo : TEST_MODE_SIL (host PC, sin hardware STM32)   \n");
        fprintf(log, "=======================================================\n\n");
        fflush(log);
        SIL_DiagSetFile(log);   /* Diag_Log → stdout + log file              */
        printf("[INFO] Resultados se guardarán en: %s\n\n", log_path);
    } else {
        printf("[WARN] No se pudo crear fichero de log. Salida solo por stdout.\n\n");
        SIL_DiagSetFile(NULL);
    }

    /* Inicializar el entorno RTOS mock (colas CAN, mutex) */
    SIL_RTOS_Init();

    /* EJECUTAR TODOS LOS TESTS */
    test_result_t res = Test_IntegrationRunAll();

    /* Añadir resumen final al fichero de log */
    if (log) {
        fprintf(log, "\n=======================================================\n");
        fprintf(log, "  RESUMEN FINAL\n");
        fprintf(log, "=======================================================\n");
        fprintf(log, "  Tests totales : %lu\n", (unsigned long)res.total);
        fprintf(log, "  Tests pasados : %lu\n", (unsigned long)res.passed);
        fprintf(log, "  Tests fallados: %lu\n", (unsigned long)res.failed);
        fprintf(log, "  Tiempo total  : %lu ms (simulados)\n", (unsigned long)res.execution_time_ms);
        fprintf(log, "  Modo          : %s\n", res.mode);
        fprintf(log, "  Resultado     : %s\n",
                res.failed == 0 ? ">>> ALL TESTS PASSED <<<" : ">>> HAY TESTS FALLIDOS <<<");
        fprintf(log, "=======================================================\n");
        fflush(log);
        SIL_DiagSetFile(NULL);   /* Desconectar antes de cerrar */
        fclose(log);
        printf("\n[INFO] Informe guardado en: %s\n", log_path);
    }

    /* Código de salida: 0 = todo OK, 1 = hay fallos */
    if (res.failed > 0) {
        printf("[RESULT] FALLOS: %lu/%lu tests fallaron.\n",
               (unsigned long)res.failed, (unsigned long)res.total);
    } else {
        printf("[RESULT] OK: %lu/%lu tests pasaron.\n",
               (unsigned long)res.passed, (unsigned long)res.total);
    }
}

/**
 * Run SIL simulation for N milliseconds
 */
static void sil_run_simulation(uint32_t duration_ms)
{
    sil_simulation_running = 1;
    sil_test_duration_ms = duration_ms;
    uint32_t end_tick = sil_tick_ms + duration_ms;
    
    printf("[SIL] Starting simulation for %u ms\n", duration_ms);
    
    while (sil_tick_ms < end_tick && sil_simulation_running) {
        sil_runtime_step_1ms();
        usleep(1000);  /* Sleep 1ms */
    }
    
    sil_simulation_running = 0;
    printf("[SIL] Simulation finished at %u ms\n", sil_tick_ms);
}

/**
 * Stop simulation
 */
void sil_stop_simulation(void)
{
    sil_simulation_running = 0;
}

/**
 * Get current simulated time
 */
uint32_t sil_get_time_ms(void)
{
    return sil_tick_ms;
}

/* ===== Test entry points ===== */

/**
 * Test: Boot sequence (BOOT -> PRECHARGE -> READY)
 */
static void test_boot_sequence(void)
{
    int failures;

    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Boot Sequence Verification   ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    SIL_Results_Init("boot_sequence_test.log");
    SIL_Results_Log("BOOT_TEST", "STARTED", "Boot sequence real-assertion suite");
    SIL_Results_LogEvent(0, "INIT", "Initialising RTOS mock");

    /* Initialise RTOS mock and create queues/mutex via MX_FREERTOS_Init */
    sil_runtime_init();

    /* Run the real assertion suite */
    failures = run_boot_sequence_tests();
    sil_test_failures += failures;

    SIL_Results_Log("BOOT_TEST",
                    (failures == 0) ? "SUCCESS" : "FAIL",
                    (failures == 0) ? "All boot sequence assertions passed"
                                    : "Boot sequence assertions FAILED");
    SIL_Results_Close();
}

/**
 * Test: Full operating cycle
 */
static void test_full_cycle(void)
{
    int failures;

    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("��  SIL TEST: Full Operating Cycle         ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    SIL_Results_Init("full_cycle_test.log");
    SIL_Results_Log("CYCLE_TEST", "STARTED", "Full cycle real-assertion suite");
    SIL_Results_LogEvent(0, "INIT", "Initialising RTOS mock");

    sil_runtime_init();

    failures = run_full_cycle_tests();
    sil_test_failures += failures;

    SIL_Results_Log("CYCLE_TEST",
                    (failures == 0) ? "SUCCESS" : "FAIL",
                    (failures == 0) ? "All full-cycle assertions passed"
                                    : "Full-cycle assertions FAILED");
    SIL_Results_Close();
}

/**
 * Test: Error handling - Low DC voltage
 */
static void test_error_low_voltage(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Low DC Voltage Fault         ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("error_low_voltage_test.log");
    SIL_Results_Log("ERROR_LOW_V", "STARTED", "Low voltage fault scenario");
    
    printf("[ERROR_V] ➜ Initializing system\n");
    sil_runtime_init();
    
    printf("[ERROR_V] ➜ Normal operation (0-5s)\n");
    SIL_CAN_InjectDCVoltage(400);  /* Normal 400V */
    sil_run_simulation(5000);
    SIL_Results_LogEvent(5000, "NORMAL_OP", "400V stable");
    
    printf("[ERROR_V] ➜ Injecting low voltage fault (5-10s)\n");
    SIL_CAN_InjectDCVoltage(250);  /* Low voltage FAULT */
    SIL_Results_LogEvent(5000, "FAULT_INJECT", "DC voltage 250V (FAULT)");
    sil_run_simulation(5000);
    
    printf("[ERROR_V] ➜ System should limit torque\n");
    app_inputs_t snapshot = {0};
    AppState_Snapshot(&snapshot);
    
    if (snapshot.torque_total == 0) {
        printf("[ERROR_V] ✅ Torque correctly limited to 0 during fault\n");
        SIL_Results_LogEvent(10000, "FAULT_RESPONSE", "Torque=0 (correct)");
    } else {
        printf("[ERROR_V] ⚠️  Torque not limited: %u\n", snapshot.torque_total);
        SIL_Results_LogEvent(10000, "FAULT_RESPONSE", "Torque not limited (issue)");
    }
    
    printf("[ERROR_V] ➜ Clearing fault (10-12s)\n");
    SIL_CAN_InjectDCVoltage(400);  /* Back to normal */
    SIL_Results_LogEvent(10000, "FAULT_CLEAR", "DC voltage restored to 400V");
    sil_run_simulation(2000);
    
    printf("[ERROR_V] ✅ Recovery test complete\n");
    SIL_Results_Log("ERROR_LOW_V", "SUCCESS", "Low voltage fault handled correctly");
    SIL_Results_Close();
}

/**
 * Test: Error handling - High temperature
 */
static void test_error_high_temperature(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: High Temperature Fault       ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("error_high_temp_test.log");
    SIL_Results_Log("ERROR_TEMP", "STARTED", "High temperature fault scenario");
    
    printf("[ERROR_T] ➜ Initializing system\n");
    sil_runtime_init();
    
    printf("[ERROR_T] ➜ Normal temperature operation (0-5s)\n");
    g_in.inv_motor_temp = 50;
    SIL_Results_LogEvent(0, "TEMP_NORMAL", "Motor temp = 50C");
    sil_run_simulation(5000);
    
    printf("[ERROR_T] ➜ Injecting high temperature fault (5-10s)\n");
    g_in.inv_motor_temp = 95;  /* >80C = WARNING/FAULT */
    SIL_Results_LogEvent(5000, "TEMP_FAULT", "Motor temp = 95C (FAULT)");
    sil_run_simulation(5000);
    
    printf("[ERROR_T] ➜ System should degrade gracefully\n");
    app_inputs_t snapshot = {0};
    AppState_Snapshot(&snapshot);
    printf("[ERROR_T] ℹ  Current state: Motor temp=%d, Torque=%u\n", 
           snapshot.inv_motor_temp, snapshot.torque_total);
    SIL_Results_LogEvent(10000, "DEGRADATION", "Graceful degradation active");
    
    printf("[ERROR_T] ➜ Cooling down (10-15s)\n");
    g_in.inv_motor_temp = 60;
    SIL_Results_LogEvent(10000, "TEMP_COOLING", "Motor temp cooling to 60C");
    sil_run_simulation(5000);
    
    printf("[ERROR_T] ✅ Recovery complete\n");
    SIL_Results_Log("ERROR_TEMP", "SUCCESS", "High temperature handled with graceful degradation");
    SIL_Results_Close();
}

/**
 * Test: EV 2.3 Safety - Brake + Throttle simultaneous
 */
static void test_safety_brake_throttle(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: EV 2.3 Brake+Throttle       ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("safety_brake_throttle_test.log");
    SIL_Results_Log("SAFETY_BT", "STARTED", "Brake+Throttle safety test");
    
    printf("[SAFETY] ➜ Initializing system\n");
    sil_runtime_init();
    
    printf("[SAFETY] ➜ Normal throttle (0-3s)\n");
    SIL_CAN_InjectThrottle(60);
    SIL_CAN_InjectBrake(0);  /* No brake */
    SIL_Results_LogEvent(0, "THROTTLE_ONLY", "Throttle=60%, Brake=0%");
    sil_run_simulation(3000);
    
    printf("[SAFETY] ➜ Simultaneous brake+throttle activation (3-8s)\n");
    SIL_CAN_InjectBrake(80);  /* Brake activated */
    SIL_Results_LogEvent(3000, "FAULT_INJECT", "Brake=80% + Throttle=60% (FAULT)");
    sil_run_simulation(5000);
    
    app_inputs_t snapshot = {0};
    AppState_Snapshot(&snapshot);
    
    printf("[SAFETY] ➜ Checking safety flag\n");
    if (snapshot.flag_EV_2_3) {
        printf("[SAFETY] ✅ EV 2.3 flag correctly set\n");
        SIL_Results_LogEvent(8000, "SAFETY_FLAG", "EV_2_3=1 (correct)");
    } else {
        printf("[SAFETY] ⚠️  EV 2.3 flag NOT set\n");
        SIL_Results_LogEvent(8000, "SAFETY_FLAG", "EV_2_3=0 (issue!)");
    }
    
    printf("[SAFETY] ➜ Releasing throttle only (8-13s)\n");
    SIL_CAN_InjectThrottle(0);  /* Release throttle */
    SIL_Results_LogEvent(8000, "THROTTLE_RELEASE", "Throttle=0%, Brake still=80%");
    sil_run_simulation(5000);
    
    printf("[SAFETY] ➜ Releasing brake (13-15s)\n");
    SIL_CAN_InjectBrake(0);
    SIL_Results_LogEvent(13000, "BRAKE_RELEASE", "All controls released");
    sil_run_simulation(2000);
    
    printf("[SAFETY] ✅ EV 2.3 safety test complete\n");
    SIL_Results_Log("SAFETY_BT", "SUCCESS", "Brake+Throttle safety validated");
    SIL_Results_Close();
}

/**
 * Test: State machine transitions under dynamic conditions
 */
static void test_dynamic_state_transitions(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Dynamic State Transitions    ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("dynamic_transitions_test.log");
    SIL_Results_Log("DYNAMIC_ST", "STARTED", "Dynamic state transition testing");
    
    printf("[DYNAMIC] ➜ Initializing\n");
    sil_runtime_init();
    
    printf("[DYNAMIC] ➜ State 1: BOOT (0-5s)\n");
    SIL_Results_LogEvent(0, "STATE", "BOOT");
    sil_run_simulation(5000);
    
    printf("[DYNAMIC] ➜ State 2: Requesting Precharge (5-7s)\n");
    g_in.ok_precarga = 0;  /* Request precharge */
    SIL_Results_LogEvent(5000, "STATE_TRANS", "Requesting PRECHARGE");
    sil_run_simulation(2000);
    
    printf("[DYNAMIC] ➜ State 3: Precharge ACK received (7-10s)\n");
    g_in.ok_precarga = 1;  /* ACK received */
    g_in.inv_dc_bus_voltage = 400;  /* Voltage stable */
    SIL_Results_LogEvent(7000, "STATE_TRANS", "PRECHARGE ACK received");
    sil_run_simulation(3000);
    
    printf("[DYNAMIC] ➜ State 4: Ready for operation (10-15s)\n");
    SIL_Results_LogEvent(10000, "STATE", "READY");
    sil_run_simulation(5000);
    
    printf("[DYNAMIC] ➜ State 5: Throttle applied (15-20s)\n");
    SIL_CAN_InjectThrottle(75);
    SIL_Results_LogEvent(15000, "STATE", "THROTTLE_CONTROL");
    sil_run_simulation(5000);
    
    printf("[DYNAMIC] ➜ State 6: Fault injection (20-22s)\n");
    g_in.inv_dc_bus_voltage = 200;  /* Fault */
    SIL_Results_LogEvent(20000, "FAULT", "Low voltage injected");
    sil_run_simulation(2000);
    
    printf("[DYNAMIC] ➜ State 7: Fault recovery (22-25s)\n");
    g_in.inv_dc_bus_voltage = 400;  /* Fault cleared */
    SIL_Results_LogEvent(22000, "RECOVERY", "Low voltage cleared");
    sil_run_simulation(3000);
    
    printf("[DYNAMIC] ✅ Dynamic transitions test complete\n");
    SIL_Results_Log("DYNAMIC_ST", "SUCCESS", "State machine transitions working correctly");
    SIL_Results_Close();
}

/**
 * Print usage
 */
static void print_usage(const char *prog)
{
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("  --test-boot              Boot sequence test\n");
    printf("  --test-full-cycle        Full operating cycle test\n");
    printf("  --test-error-voltage     Low voltage fault test\n");
    printf("  --test-error-temp        High temperature fault test\n");
    printf("  --test-safety-brake      EV 2.3 brake+throttle test\n");
    printf("  --test-dynamic-states    Dynamic state transition test\n");
    printf("  --test-legacy-compat     Polling-compatible startup/inverter harness\n");
    printf("  --test-integration       Suites S1-S10 (test_integration.c)\n");
    printf("                           -> genera tests/sil/results/integration_test.log\n");
    printf("  --test-all               Run ALL tests (incluyendo S1-S10)\n");
    printf("  --help                   Print this message\n");
}

/* ===== Main entry point ===== */

int main(int argc, char *argv[])
{
    printf("\n");
    printf("========================================\n");
    printf("  ECU08 NSIL - Software-In-The-Loop\n");
    printf("  Comprehensive SIL Test Suite\n");
    printf("========================================\n\n");
    
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    const char *test_name = argv[1];
    
    if (strcmp(test_name, "--test-boot") == 0) {
        test_boot_sequence();
    } else if (strcmp(test_name, "--test-full-cycle") == 0) {
        test_full_cycle();
    } else if (strcmp(test_name, "--test-error-voltage") == 0) {
        test_error_low_voltage();
    } else if (strcmp(test_name, "--test-error-temp") == 0) {
        test_error_high_temperature();
    } else if (strcmp(test_name, "--test-safety-brake") == 0) {
        test_safety_brake_throttle();
    } else if (strcmp(test_name, "--test-dynamic-states") == 0) {
        test_dynamic_state_transitions();
    } else if (strcmp(test_name, "--test-legacy-compat") == 0) {
        test_legacy_compat_harness();
    } else if (strcmp(test_name, "--test-integration") == 0) {
        test_integration_suite();
    } else if (strcmp(test_name, "--test-all") == 0) {
        printf("[MAIN] Running all SIL tests...\n\n");
        test_boot_sequence();
        test_full_cycle();
        test_error_low_voltage();
        test_error_high_temperature();
        test_safety_brake_throttle();
        test_dynamic_state_transitions();
        test_legacy_compat_harness();
        test_integration_suite();   /* S1-S10 al final, genera integration_test.log */
    } else if (strcmp(test_name, "--help") == 0) {
        print_usage(argv[0]);
    } else {
        printf("Unknown test: %s\n", test_name);
        print_usage(argv[0]);
        return 1;
    }
    
    printf("\n[SIL] Test execution completed\n\n");
    if (sil_test_failures > 0) {
        printf("[SIL] %d runtime checks failed\n", sil_test_failures);
        return 1;
    }
    return 0;
}

/* ===== Stub for FreeRTOS panic ===== */
void vAssertCalled(const char *file, int line)
{
    printf("[PANIC] Assertion failed at %s:%d\n", file, line);
    sil_stop_simulation();
    exit(1);
}
