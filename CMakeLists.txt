cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w)
include(pico_sdk_import.cmake)

project(pico_obc_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_obc_firmware
    pico_obc_firmware.c
)

# Δηλωση του τρεχοντος φακελου ωστε το lwIP να βρισκει το lwipopts.h
target_include_directories(pico_obc_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(pico_obc_firmware
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_watchdog
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(pico_obc_firmware 1)
pico_enable_stdio_uart(pico_obc_firmware 0)

pico_add_extra_outputs(pico_obc_firmware)