cmake_minimum_required(VERSION 3.13)

# Το board μας
if(NOT PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

# Πάμε με ARM (Cortex-M33) για το επαγγελματικό setup
set(PICO_PLATFORM rp2350-arm-s)

include(pico_sdk_import.cmake)

project(pico_obc_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Φέρνουμε τον kernel του FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel)
if(NOT EXISTS ${FREERTOS_KERNEL_PATH}/include/FreeRTOS.h)
    message(FATAL_ERROR "FreeRTOS-Kernel not found. Run: git submodule update --init")
endif()
include(FreeRTOS_Kernel_import.cmake)

add_executable(pico_obc_firmware
    pico_obc_firmware.c
)

target_include_directories(pico_obc_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_options(pico_obc_firmware PRIVATE -Wall -Wextra)

# Εδώ είναι τα sos για να κάνει compile και να μη σκάει το USB
target_link_libraries(pico_obc_firmware
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_watchdog
    pico_cyw43_arch_lwip_sys_freertos   
    pico_async_context_freertos        # <-- Το fix για το COM port crash!
    FreeRTOS-Kernel-Heap4               
)

pico_enable_stdio_usb(pico_obc_firmware 1)
pico_enable_stdio_uart(pico_obc_firmware 0)

pico_add_extra_outputs(pico_obc_firmware)