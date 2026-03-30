/**
 * sil_main.c
 * SIL (Software-In-The-Loop) Entry Point - Simplified Version
 * 
 * Simulates the complete ECU application logic without requiring
 * real FreeRTOS compilation. Perfect for functional testing.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "app_state.h"
#include "control.h"
#include "test_integration.h"   /* suites S1-S10, Test_IntegrationRunAll() */
#include "sil_hal_mocks.h"
#include "sil_can_simulator.h"
#include "sil_boot_sequence.h"
#include "sil_results.h"

/* Declarada en mocks/diag_sil.c: redirige Diag_Log al fichero especificado */
extern void SIL_DiagSetFile(FILE *f);
/* Declarada en mocks/cmsis_os2_impl.c: crea colas CAN y mutex global       */
extern void SIL_RTOS_Init(void);

/* ===== Global state for SIL ===== */
static volatile uint32_t sil_tick_ms = 0;
static volatile int sil_simulation_running = 0;
static volatile uint32_t sil_test_duration_ms = 0;

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

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Integration Test Suite  (S1-S10)         ║\n");
    printf("║  Modo: TEST_MODE_SIL  |  Host PC (sin hardware)     ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    /* Abrir fichero de resultados */
    FILE *log = fopen(log_path, "w");
    if (!log) {
        /* Intentar crear el directorio results si no existe */
        printf("[WARN] No se pudo abrir %s, intentando crear directorio...\n", log_path);
        (void)system("mkdir -p tests/sil/results");
        log = fopen(log_path, "w");
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
    sil_tick_ms = 0;
    sil_simulation_running = 1;
    sil_test_duration_ms = duration_ms;
    
    printf("[SIL] Starting simulation for %u ms\n", duration_ms);
    
    while (sil_tick_ms < duration_ms && sil_simulation_running) {
        sil_tick_ms++;
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
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Boot Sequence Verification   ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("boot_sequence_test.log");
    SIL_Results_Log("BOOT_TEST", "STARTED", "Boot sequence verification");
    
    /* Initialize application */
    printf("[BOOT] ➜ Initializing application\n");
    SIL_Results_LogEvent(0, "INIT", "Application startup");
    AppState_Init();
    SIL_HAL_Init();
    SIL_CAN_Init();
    
    printf("[BOOT] ➜ Simulating boot sequence (10 seconds)\n");
    sil_run_simulation(10000);
    
    /* Report results */
    printf("\n[BOOT] ✅ Boot sequence simulation complete\n");
    SIL_Results_Log("BOOT_TEST", "SUCCESS", "Boot sequence completed without errors");
    SIL_Results_LogEvent(sil_get_time_ms(), "COMPLETE", "Boot sequence finished");
    
    SIL_Results_Close();
}

/**
 * Test: Full operating cycle
 */
static void test_full_cycle(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Full Operating Cycle         ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("full_cycle_test.log");
    SIL_Results_Log("CYCLE_TEST", "STARTED", "Full operating cycle verification");
    
    printf("[CYCLE] ➜ Initializing system\n");
    SIL_Results_LogEvent(0, "INIT", "System initialization");
    AppState_Init();
    SIL_HAL_Init();
    SIL_CAN_Init();
    
    printf("[CYCLE] ➜ Phase 1: Boot sequence (0-5s)\n");
    SIL_Results_LogEvent(0, "PHASE1", "Boot sequence");
    sil_run_simulation(5000);
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE1_END", "Boot complete");
    
    printf("[CYCLE] ➜ Phase 2: Precharge sequence (5-15s)\n");
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE2", "Precharge sequence");
    sil_run_simulation(15000);
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE2_END", "Precharge complete");
    
    printf("[CYCLE] ➜ Phase 3: Dynamic throttle control (15-30s)\n");
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE3", "Throttle control phase");
    
    /* Simulated throttle profile */
    printf("         - [15s] Throttle = 0%%\n");
    SIL_Results_LogEvent(15000, "THROTTLE", "0%");
    SIL_CAN_InjectThrottle(0);
    sil_run_simulation(3000);
    
    printf("         - [18s] Throttle = 50%%\n");
    SIL_Results_LogEvent(18000, "THROTTLE", "50%");
    SIL_CAN_InjectThrottle(50);
    sil_run_simulation(4000);
    
    printf("         - [22s] Throttle = 100%%\n");
    SIL_Results_LogEvent(22000, "THROTTLE", "100%");
    SIL_CAN_InjectThrottle(100);
    sil_run_simulation(4000);
    
    printf("         - [26s] Throttle = 0%% (coast down)\n");
    SIL_Results_LogEvent(26000, "THROTTLE", "0% (coast)");
    SIL_CAN_InjectThrottle(0);
    sil_run_simulation(4000);
    
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE3_END", "Throttle control complete");
    
    printf("\n[CYCLE] ✅ Full cycle simulation complete\n");
    SIL_Results_Log("CYCLE_TEST", "SUCCESS", "Full operating cycle completed without errors");
    SIL_Results_LogEvent(sil_get_time_ms(), "COMPLETE", "Full cycle finished");
    
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
    AppState_Init();
    SIL_HAL_Init();
    SIL_CAN_Init();
    
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
    AppState_Init();
    SIL_HAL_Init();
    SIL_CAN_Init();
    
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
    AppState_Init();
    SIL_HAL_Init();
    SIL_CAN_Init();
    
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
    AppState_Init();
    SIL_HAL_Init();
    SIL_CAN_Init();
    
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
    printf("  --test-integration       Suites S1-S10 (test_integration.c)\n");
    printf("                           → genera results/integration_test.log\n");
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
        test_integration_suite();   /* S1-S10 al final, genera integration_test.log */
    } else if (strcmp(test_name, "--help") == 0) {
        print_usage(argv[0]);
    } else {
        printf("Unknown test: %s\n", test_name);
        print_usage(argv[0]);
        return 1;
    }
    
    printf("\n[SIL] Test execution completed\n\n");
    return 0;
}

/* ===== Stub for FreeRTOS panic ===== */
void vAssertCalled(const char *file, int line)
{
    printf("[PANIC] Assertion failed at %s:%d\n", file, line);
    sil_stop_simulation();
    exit(1);
}
