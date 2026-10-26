cmake_minimum_required(VERSION 3.13)

# Target board
if(NOT PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

# Arm Cortex-M33 (secure) build of the RP2350
set(PICO_PLATFORM rp2350-arm-s)

include(pico_sdk_import.cmake)

project(pico_obc_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# FreeRTOS kernel (git submodule)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel)
if(NOT EXISTS ${FREERTOS_KERNEL_PATH}/include/FreeRTOS.h)
    message(FATAL_ERROR "FreeRTOS-Kernel not found. Run: git submodule update --init")
endif()
include(FreeRTOS_Kernel_import.cmake)

add_executable(pico_obc_firmware
    src/main.c
    src/board.c
    src/fdir_fsm.c
    src/mpu6050.c
    src/network.c
    src/obc_state.c
    src/reset_cause.c
    src/tasks.c
    src/telemetry.c
)

target_include_directories(pico_obc_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_options(pico_obc_firmware PRIVATE -Wall -Wextra)

target_link_libraries(pico_obc_firmware
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_watchdog
    pico_cyw43_arch_lwip_sys_freertos
    pico_async_context_freertos        # required for FreeRTOS-based cyw43 arch (fixes USB COM port crash)
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(pico_obc_firmware 1)
pico_enable_stdio_uart(pico_obc_firmware 0)

pico_add_extra_outputs(pico_obc_firmware)
