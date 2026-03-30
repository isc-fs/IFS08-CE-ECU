cmake_minimum_required(VERSION 3.15)

# =============================================================================
# ECU08 NSIL  –  SIL (Software-In-The-Loop) build
#
# Compila toda la lógica de aplicación + tests de integración en el host PC
# sin necesitar FreeRTOS real ni hardware STM32.
#
# MOCKS (directorio mocks/):
#   cmsis_os2.h      → tipos CMSIS-RTOS v2 (sobreescribe el del middleware)
#   main.h           → stubs HAL/FDCAN     (sobreescribe Core/Inc/main.h)
#   cmsis_os2_impl.c → implementaciones: queues ring-buffer, mutex no-op, tick
#   hal_impl.c       → hfdcan1/2/3 + stubs HAL_FDCAN_*
#   diag_sil.c       → Diag_Log real → stdout + fichero .log
# =============================================================================

# ---- Fuentes de la aplicación (lógica pura, sin RTOS ni HAL real) ----------
set(APP_SOURCES
    ../../Core/Src/app_state.c
    ../../Core/Src/can.c
    ../../Core/Src/control.c
    ../../Core/Src/telemetry.c
    ../../Core/Src/test_integration.c   # suites de integración S1-S10
)

# ---- Fuentes del entorno SIL -----------------------------------------------
set(SIL_SOURCES
    sil_main.c
    sil_hal_mocks.c
    sil_can_simulator.c
    sil_boot_sequence.c
    sil_results.c
    integration/test_boot_sequence.c
    integration/test_full_cycle.c
)

# ---- Mocks RTOS / HAL (necesarios para compilar APP_SOURCES en host) --------
set(MOCK_SOURCES
    mocks/cmsis_os2_impl.c   # tick simulado, ring-buffer queues, mutex no-op
    mocks/hal_impl.c         # hfdcan1/2/3, HAL_FDCAN stubs
    mocks/diag_sil.c         # Diag_Log → stdout + fichero .log
)

# ---- Ejecutable -------------------------------------------------------------
add_executable(ecu08_sil
    ${APP_SOURCES}
    ${SIL_SOURCES}
    ${MOCK_SOURCES}
)

# ---- Directorios de include -------------------------------------------------
# IMPORTANTE: mocks/ va PRIMERO para que cmsis_os2.h y main.h del mock
# tengan precedencia sobre Core/Inc/main.h y el middleware real.
target_include_directories(ecu08_sil PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks   # 1º: overrides de cabeceras
    ${CMAKE_CURRENT_SOURCE_DIR}         # 2º: sil_*.h
    ../../Core/Inc                      # 3º: app_state.h, control.h, etc.
)

# ---- Opciones de compilación ------------------------------------------------
target_compile_options(ecu08_sil PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

target_compile_definitions(ecu08_sil PRIVATE
    SIL_BUILD=1
    TEST_MODE_SIL=1        # activa guardas de compilación en test_integration.h
    SIL_ANSI_COLORS=1      # colores ANSI en stdout (quitar si el terminal no los soporta)
)

# ---- Enlazar con la librería matemática (por si control.c usa floats) -------
target_link_libraries(ecu08_sil m)

# ---- Tests CTest ------------------------------------------------------------
enable_testing()

add_test(
    NAME SIL_Boot
    COMMAND ecu08_sil --test-boot
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

add_test(
    NAME SIL_Integration
    COMMAND ecu08_sil --test-integration
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

add_test(
    NAME SIL_All
    COMMAND ecu08_sil --test-all
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

add_test(
    NAME SILFullCycle
    COMMAND ecu08_sil --test-full-cycle
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

