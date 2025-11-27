cmake_minimum_required(VERSION 3.12)

set(PICO_PLATFORM "rp2350")
# Default to Pico 2 W (CYW43 present) unless caller overrides; also upgrade the
# SDK's default 'pico2' to 'pico2_w' so Wi‑Fi headers (cyw43) are available.
set(PICO_BOARD "pico2_w")

# Shared dependency roots (LIBS_ROOT, PICO_SDK_PATH, FREERTOS_KERNEL_PATH, etc.)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

# All Third_Party deps live under projects/libs
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "Disable host -arch flag when cross-compiling for bare metal" FORCE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

if(NOT DEFINED CMAKE_C_FLAGS)
    set(CMAKE_C_FLAGS "-mcpu=cortex-m33 -mthumb")
endif()
if(NOT DEFINED CMAKE_CXX_FLAGS)
    set(CMAKE_CXX_FLAGS "-mcpu=cortex-m33 -mthumb")
endif()

if (WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" CACHE STRING "Prefer static libs for cross compile on Windows")
endif()

message(STATUS "Toolchain: using C=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}")

set(PICOTOOL_FETCH_FROM_GIT_PATH "${LIBS_ROOT}/Third_Party/picotool/fetch_from_git.cmake")

project(picow_pwm_servos C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 26)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.2.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.2.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Fail fast if an MSVC generator is selected: Visual Studio's generator uses MSVC
# and won't respect ARM GCC cross-toolchain settings. Encourage the user to
# instead use the 'Ninja' generator and set the provided toolchain file.
if (CMAKE_GENERATOR MATCHES "Visual Studio")
        message(FATAL_ERROR "Visual Studio generator selected; this project targets ARM GCC and Visual Studio cannot cross-compile using MSVC. Use -G 'Ninja' and set CMAKE_TOOLCHAIN_FILE to the ARM toolchain.")
endif()

# Pull in the SDK (must be before projects or subdirectories are added)

# Initialize the Pico SDK now so its helper functions (e.g. pico_add_extra_outputs)
# and toolchain flags take effect prior to any subdirectory or project() calls.
pico_sdk_init()
set(FREERTOS_THIRD_PARTY_DIR ${FREERTOS_KERNEL_PATH}/portable/ThirdParty)
include(${FREERTOS_THIRD_PARTY_DIR}/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)


# On Windows, CMake may choose the Visual Studio generator by default — detect
# and error early if an MSVC generator is selected and we're targeting the
# cross-compiler (ARM) because MSVC cannot cross-compile with GCC by default.
if (CMAKE_GENERATOR MATCHES "Visual Studio")
        message(FATAL_ERROR "Visual Studio generator selected; this project expects the ARM GCC cross-compiler. Use -G 'Ninja' and provide CMAKE_TOOLCHAIN_FILE to compile for ARM.")
endif()



add_executable(picow_pwm_servos
        picow_pwm_servos.c
        )

target_link_libraries(picow_pwm_servos
        pico_stdlib          # for core functionality
        hardware_pwm         # for PWM helpers
        )

pico_enable_stdio_usb(picow_pwm_servos 1)
pico_enable_stdio_uart(picow_pwm_servos 1)
# create map/bin/hex file etc.
pico_add_extra_outputs(picow_pwm_servos)


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -fpermissive
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
