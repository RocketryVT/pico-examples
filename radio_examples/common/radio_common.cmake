# radio_common.cmake — shared FreeRTOS + logging + GPS support for the RF tools.
#
# include() this from an example's CMakeLists AFTER pico_sdk_init(). It defines
# the littlefs / gps libraries and the FreeRTOS kernel, then exposes three lists
# the example folds straight into its own executable:
#
#   ${RADIO_COMMON_SOURCES}  — common .c/.cpp (console, logging, GPS, flash FS,
#                              CSV, FreeRTOS scaffolding)
#   ${RADIO_COMMON_INCLUDE}  — the common/ include dir (also holds FreeRTOSConfig.h)
#   ${RADIO_COMMON_LIBS}     — libraries to link
#
# Example:
#   include(${CMAKE_CURRENT_LIST_DIR}/../common/radio_common.cmake)
#   add_executable(foo main.cpp ${RADIO_COMMON_SOURCES})
#   target_include_directories(foo PRIVATE ${RADIO_COMMON_INCLUDE} ...)
#   target_link_libraries(foo ${RADIO_COMMON_LIBS} RadioLib hardware_spi ...)
#
# The common sources are compiled into each executable (rather than a static
# library) because the FreeRTOS-Kernel port exposes its port.c as INTERFACE
# sources; routing them through an intermediate static lib would double-compile
# them into both the lib and the exe and clash at link time.
#
# Flash split is fixed in lfs_pico_flash.h: bottom 1 MB program, top 3 MB the
# littlefs file region. Flash erase/program is made SMP-safe with
# flash_safe_execute() (pico_flash), which is supported under FreeRTOS SMP.

if (NOT DEFINED LITTLEFS_PATH)
    message(FATAL_ERROR "LITTLEFS_PATH not set — include cmake/deps.cmake first.")
endif()
if (NOT EXISTS "${LITTLEFS_PATH}/lfs.c")
    message(FATAL_ERROR "littlefs sources not found at '${LITTLEFS_PATH}'.")
endif()
if (NOT DEFINED GPS_ROOT)
    message(FATAL_ERROR "GPS_ROOT not set — include cmake/deps.cmake first.")
endif()
if (NOT EXISTS "${GPS_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "GPS library not found at '${GPS_ROOT}'.")
endif()
if (NOT DEFINED FREERTOS_KERNEL_PATH)
    message(FATAL_ERROR "FREERTOS_KERNEL_PATH not set — include cmake/deps.cmake first.")
endif()

# -- FreeRTOS kernel (RP2350 ARM, non-TrustZone, SMP) -------------------------
# FreeRTOSConfig.h lives in this common/ directory.
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL "")
if (NOT TARGET FreeRTOS-Kernel)
    set(_freertos_import
        "${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake")
    if (NOT EXISTS "${_freertos_import}")
        message(FATAL_ERROR "FreeRTOS RP2350 import not found at '${_freertos_import}'.")
    endif()
    include(${_freertos_import})
endif()

# -- littlefs core ------------------------------------------------------------
if (NOT TARGET littlefs)
    add_library(littlefs STATIC
        ${LITTLEFS_PATH}/lfs.c
        ${LITTLEFS_PATH}/lfs_util.c
    )
    target_include_directories(littlefs PUBLIC ${LITTLEFS_PATH})
    # Silence littlefs' debug/warn/error printf so they don't pollute the CSV
    # stream. Errors still surface as lfs_* return codes (rf_log reports them).
    target_compile_definitions(littlefs PUBLIC
        LFS_NO_DEBUG
        LFS_NO_WARN
        LFS_NO_ERROR
        LFS_NO_ASSERT
    )
endif()

# -- u-blox GPS driver library ------------------------------------------------
if (NOT TARGET gps)
    add_subdirectory(${GPS_ROOT} ${CMAKE_BINARY_DIR}/gps)
endif()

# -- Exported lists for each example to compile into its own executable -------
set(RADIO_COMMON_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/rtos.cpp
    ${CMAKE_CURRENT_LIST_DIR}/rf_console.cpp
    ${CMAKE_CURRENT_LIST_DIR}/rf_csv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/rf_log.cpp
    ${CMAKE_CURRENT_LIST_DIR}/lfs_pico_flash.c
    ${CMAKE_CURRENT_LIST_DIR}/gps_task.cpp
)

set(RADIO_COMMON_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}
)

set(RADIO_COMMON_LIBS
    littlefs
    gps
    pico_stdlib
    pico_multicore
    pico_flash
    hardware_flash
    hardware_sync
    hardware_uart
    hardware_i2c   # gps_driver.hpp includes hardware/i2c.h (I2cTransport)
    hardware_dma   # gps_driver.hpp includes hardware/dma.h (DMA ring RX)
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)
