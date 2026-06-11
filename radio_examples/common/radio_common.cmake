# radio_common.cmake — shared file-logging support for the RF bench-test tools.
#
# include() this from an example's CMakeLists AFTER pico_sdk_init(), then link
# the `radio_common` target into the executable. That target pulls in:
#   * littlefs (lfs.c / lfs_util.c, compiled from LITTLEFS_PATH)
#   * the Pico on-chip-flash block device (lfs_pico_flash.c)
#   * the rf_log helper + the rf_csv.h schema include dir
#   * the UART0 GPS task (gps_task.cpp) + the gps driver library (GPS_ROOT)
#
# Flash split is fixed in lfs_pico_flash.h: the bottom 1 MB of the 4 MB part is
# the program image, the top 3 MB is the littlefs file region. These tools are
# tens of kB, so the program never approaches the 1 MB boundary; the split is
# enforced by the block device's base offset rather than a custom linker script
# (PICO_FLASH_SIZE_BYTES stays at the board default 4 MB so flash_range_program
# is allowed to write the upper 3 MB).

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

# -- littlefs core ------------------------------------------------------------
if (NOT TARGET littlefs)
    add_library(littlefs STATIC
        ${LITTLEFS_PATH}/lfs.c
        ${LITTLEFS_PATH}/lfs_util.c
    )
    target_include_directories(littlefs PUBLIC ${LITTLEFS_PATH})
    # Silence littlefs' debug/warn/error printf so they don't pollute the CSV
    # stream on USB (those lines don't start with '#', so the analysis scripts
    # in scripts/ would fail to parse them). Errors still surface as lfs_*
    # return codes, which rf_log handles and reports as "# [log]" comments.
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

# -- flash block device + CSV file logger + UART0 GPS task --------------------
if (NOT TARGET radio_common)
    add_library(radio_common STATIC
        ${CMAKE_CURRENT_LIST_DIR}/lfs_pico_flash.c
        ${CMAKE_CURRENT_LIST_DIR}/rf_log.cpp
        ${CMAKE_CURRENT_LIST_DIR}/gps_task.cpp
        ${CMAKE_CURRENT_LIST_DIR}/rf_console.cpp
    )
    # PUBLIC so consumers also get rf_csv.h / rf_log.h / gps_task.h on their path.
    target_include_directories(radio_common PUBLIC ${CMAKE_CURRENT_LIST_DIR})
    target_link_libraries(radio_common PUBLIC
        littlefs
        gps
        pico_stdlib
        hardware_flash
        hardware_sync
        hardware_uart
        hardware_i2c   # gps_driver.hpp includes hardware/i2c.h (I2cTransport)
    )
endif()
