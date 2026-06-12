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
# rfm69_433_rx_test — minimal standalone Pico 2 W program that puts the 433 MHz
# RFM69HCW (RF69, LoRa1, SPI1) into continuous GFSK receive and prints packets +
# RSSI + packet-loss (PER) over USB serial.
#
# Build (from this directory):
#   mkdir build && cd build
#   cmake .. && make
# Output: build/rfm69_433_rx_test.uf2  (drag onto the Pico in BOOTSEL mode)

cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s)
set(PICO_BOARD pico2_w)

# Locate the avionics root (4 levels up: projects/pico_examples/radio_examples/<name> -> Avionics)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

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

project(rfm69_433_rx_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)

pico_sdk_init()

# -- RadioLib (local copy at the Avionics root, RADIOLIB_PATH from deps.cmake) --
if (NOT EXISTS "${RADIOLIB_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "RadioLib not found at '${RADIOLIB_PATH}'. "
            "Set RADIOLIB_PATH to the RadioLib-7.x source directory.")
endif()
set(RADIOLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RADIOLIB_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
add_subdirectory(${RADIOLIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/radiolib EXCLUDE_FROM_ALL)

# -- littlefs file logging (flash block device + rf_log + CSV schema) ----------
include(${CMAKE_CURRENT_LIST_DIR}/../common/radio_common.cmake)

add_executable(rfm69_433_rx_test main.cpp ${RADIO_COMMON_SOURCES})

# PicoHal.h (Pico SDK implementation of RadioLibHal) lives in lora/hal/.
# rf_csv.h (shared CSV output schema) lives in ../common/.
target_include_directories(rfm69_433_rx_test PRIVATE
    ${LORA_ROOT}/hal
    ${RADIO_COMMON_INCLUDE}
)

target_link_libraries(rfm69_433_rx_test
    RadioLib
    pico_stdlib
    hardware_spi
    hardware_gpio
    ${RADIO_COMMON_LIBS}
)

# USB serial for printf(); no UART.
pico_enable_stdio_usb(rfm69_433_rx_test 1)
pico_enable_stdio_uart(rfm69_433_rx_test 0)

pico_set_program_name(rfm69_433_rx_test "rfm69_433_rx_test")
pico_set_program_version(rfm69_433_rx_test "0.1")

pico_add_extra_outputs(rfm69_433_rx_test)
