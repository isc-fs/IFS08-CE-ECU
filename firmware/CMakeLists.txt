# ============================================================================
# ECU08 firmware — arm-none-eabi cross-compile for STM32H733ZG.
#
#   cmake -S firmware -B build-fw -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake
#   cmake --build build-fw
#
# Produces build-fw/ECU08.elf. The host SIL/unit tests live at the repo root
# CMakeLists (configured with -S .) and are unaffected by this build.
# ============================================================================
cmake_minimum_required(VERSION 3.15)

project(ECU08 C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Repo root (this file is in firmware/).
set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(LINKER_SCRIPT ${REPO}/STM32H733ZGTX_FLASH.ld)

# --- Firmware identity baked into the bootloader fwinfo record -------------
# (Core/Src/firmware_info.c). Git short hash split into 4 bytes; build epoch.
find_package(Git QUIET)
set(ECU_GIT_SHORT "00000000")
if(Git_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} -C ${REPO} rev-parse --short=8 HEAD
    OUTPUT_VARIABLE ECU_GIT_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
string(SUBSTRING "${ECU_GIT_SHORT}00000000" 0 8 ECU_GIT_SHORT)
string(SUBSTRING "${ECU_GIT_SHORT}" 0 2 H0)
string(SUBSTRING "${ECU_GIT_SHORT}" 2 2 H1)
string(SUBSTRING "${ECU_GIT_SHORT}" 4 2 H2)
string(SUBSTRING "${ECU_GIT_SHORT}" 6 2 H3)
string(TIMESTAMP ECU_BUILD_TS "%s" UTC)

add_compile_definitions(
  USE_HAL_DRIVER STM32H733xx USE_PWR_LDO_SUPPLY
  ECU_FW_VERSION_MAJOR=0 ECU_FW_VERSION_MINOR=1 ECU_FW_VERSION_PATCH=0
  ECU_GIT_HASH_0=0x${H0} ECU_GIT_HASH_1=0x${H1}
  ECU_GIT_HASH_2=0x${H2} ECU_GIT_HASH_3=0x${H3}
  ECU_BUILD_TIMESTAMP=${ECU_BUILD_TS}ULL
)

include_directories(
  ${REPO}/Core/Inc
  ${REPO}/Drivers/STM32H7xx_HAL_Driver/Inc
  ${REPO}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  ${REPO}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
  ${REPO}/Drivers/CMSIS/Include
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/include
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
  ${REPO}/Middlewares/Third_Party/FatFs/src
  ${REPO}/FATFS/App
  ${REPO}/FATFS/Target
)

# --- Application sources (CubeMX-managed + hand-written) -------------------
# Excluded on purpose:
#   test_integration.c          -> SIL-only test harness
#   main_rx_callback_snippet.c  -> doc snippet; redefines HAL_FDCAN_RxFifo0Callback
set(APP_SRC
  ${REPO}/Core/Src/main.c
  ${REPO}/Core/Src/firmware_info.c
  ${REPO}/Core/Src/bootloader.c
  ${REPO}/Core/Src/pit_diag.c
  ${REPO}/Core/Src/freertos.c
  ${REPO}/Core/Src/control.c
  ${REPO}/Core/Src/can.c
  ${REPO}/Core/Src/fdcan.c
  ${REPO}/Core/Src/adc.c
  ${REPO}/Core/Src/gpio.c
  ${REPO}/Core/Src/i2c.c
  ${REPO}/Core/Src/io_signals.c
  ${REPO}/Core/Src/memorymap.c
  ${REPO}/Core/Src/nrf24.c
  ${REPO}/Core/Src/sdmmc.c
  ${REPO}/Core/Src/spi.c
  ${REPO}/Core/Src/telemetry.c
  ${REPO}/Core/Src/telemetry_dashboard.c
  ${REPO}/Core/Src/diag.c
  ${REPO}/Core/Src/app_state.c
  ${REPO}/Core/Src/app_tasks.c
  ${REPO}/Core/Src/tim.c
  ${REPO}/Core/Src/usart.c
  ${REPO}/Core/Src/usb_otg.c
  ${REPO}/Core/Src/stm32h7xx_it.c
  ${REPO}/Core/Src/stm32h7xx_hal_msp.c
  ${REPO}/Core/Src/system_stm32h7xx.c
  ${REPO}/Core/Src/syscalls.c
  ${REPO}/Core/Src/sysmem.c
)

# --- STM32 HAL (compile all; unused modules are #ifdef'd out by hal_conf) ---
file(GLOB HAL_SRC ${REPO}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER HAL_SRC EXCLUDE REGEX ".*_template\\.c$")

# --- FreeRTOS (CMSIS-RTOS v2, ARM_CM4F port, heap_4) -----------------------
set(RTOS_SRC
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/list.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# --- FatFs -----------------------------------------------------------------
set(FATFS_SRC
  ${REPO}/FATFS/App/fatfs.c
  ${REPO}/FATFS/Target/sd_diskio.c
  ${REPO}/FATFS/Target/bsp_driver_sd.c
  ${REPO}/Middlewares/Third_Party/FatFs/src/ff.c
  ${REPO}/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
  ${REPO}/Middlewares/Third_Party/FatFs/src/diskio.c
  ${REPO}/Middlewares/Third_Party/FatFs/src/option/syscall.c
)

add_executable(ECU08
  ${APP_SRC} ${HAL_SRC} ${RTOS_SRC} ${FATFS_SRC}
  ${REPO}/Core/Startup/startup_stm32h733zgtx.s
)

target_link_options(ECU08 PRIVATE
  -T${LINKER_SCRIPT}
  -Wl,-Map=ECU08.map,--cref
  -Wl,--gc-sections
  -Wl,--print-memory-usage
  -lc -lm
)

# Print size after build.
add_custom_command(TARGET ECU08 POST_BUILD
  COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:ECU08>)
