# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
#
# lora915_tx_test — minimal standalone Pico 2 W GPS UART parser/autobaud test.
#
# LoRa/RadioLib code is intentionally disabled in this target while validating
# the current GPS driver and board/device profile plumbing.
#
# Build (from this directory):
#   mkdir build && cd build
#   cmake .. && make
# Output: build/lora915_tx_test.uf2  (drag onto the Pico in BOOTSEL mode)

cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s)
set(PICO_BOARD gs_pcb_v1)

# Locate the avionics root (4 levels up: projects/pico_examples/radio_examples/<name> -> Avionics)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

# Custom GSPCB board definition lives in boards/gs_pcb_v1.h (layers on pico2_w).
set(PICO_BOARD_HEADER_DIRS ${AVIONICS_ROOT}/boards)

# Pull in Pico SDK (must be before project())
set(PICO_SDK_ROOT ${PICO_SDK_PATH})
if (NOT EXISTS "${PICO_SDK_ROOT}/pico_sdk_init.cmake")
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_ROOT}. "
            "Initialize the submodule or set PICO_SDK_PATH.")
endif()
include(${PICO_SDK_ROOT}/pico_sdk_init.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "Disable host -arch when cross-compiling" FORCE)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(lora915_tx_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

pico_sdk_init()

# -- GPS-only test build -------------------------------------------------------
# LoRa/RadioLib/common RF logging are intentionally disabled for this target
# while testing GPS parsing with only the GPS module connected.
if (NOT TARGET gps)
    add_subdirectory(${GPS_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/gps)
endif()

add_executable(lora915_tx_test main.cpp)

# boards/board.hpp + active board pins + this project's board_profile.hpp.
target_include_directories(lora915_tx_test PRIVATE
    ${AVIONICS_ROOT}
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(lora915_tx_test
    gps
    pico_stdlib
    hardware_gpio
    hardware_uart
    hardware_i2c
    hardware_dma
)

# USB serial for printf(); no UART.
pico_enable_stdio_usb(lora915_tx_test 1)
pico_enable_stdio_uart(lora915_tx_test 0)

pico_set_program_name(lora915_tx_test "lora915_tx_test")
pico_set_program_version(lora915_tx_test "0.1")

pico_add_extra_outputs(lora915_tx_test)
