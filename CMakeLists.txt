# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion RISCV_PICO_2_3_1_0)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_obc_firmware C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(pico_obc_firmware pico_obc_firmware.c)

pico_set_program_name(pico_obc_firmware "pico_obc_firmware")
pico_set_program_version(pico_obc_firmware "0.1")

# USB Serial Output
pico_enable_stdio_uart(pico_obc_firmware 0)
pico_enable_stdio_usb(pico_obc_firmware 1)

# Λέμε στον compiler να βρίσκει το FreeRTOSConfig.h στον ίδιο φάκελο
target_include_directories(pico_obc_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Σύνδεση των απαραίτητων βιβλιοθηκών (Hardware + FreeRTOS)
target_link_libraries(pico_obc_firmware
    pico_stdlib
    hardware_adc
    hardware_watchdog
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_add_extra_outputs(pico_obc_firmware)