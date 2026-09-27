# Host-compiled unit tests for firmware C code.
# These run on macOS/Linux — no PlutoSDR hardware required.
#
# Run from the repo root with: make test

enable_testing()

set(TEST_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/../platform/styx/firmware/src/
    ${CMAKE_SOURCE_DIR}/../platform/styx/firmware/tools/common/
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/tools/common
    ${CMAKE_SOURCE_DIR}/../extern/lib80211/include
)

# Object library: deimos_rx.c + hardware stubs.
# Each test that calls firmware src functions links against this.
add_library(rx_test_stubs OBJECT
    ${CMAKE_SOURCE_DIR}/src/deimos_rx.c
    stubs.c
)
target_include_directories(rx_test_stubs PUBLIC ${TEST_INCLUDE_DIRS})

macro(add_firmware_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE rx_test_stubs)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

# --- Phase 2 tests ---
add_firmware_test(test_classify)
add_firmware_test(test_parse_header)

# --- Phase 4: IE parsing ---
add_firmware_test(test_parse_ies)

# --- deimos_tool: JSON waveform parser (malformed-input hardening) ---
add_executable(test_waveform_parse
    test_waveform_parse.c
    ${CMAKE_SOURCE_DIR}/tools/common/deimos_tool.c
)
target_include_directories(test_waveform_parse PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_waveform_parse PRIVATE rx_test_stubs m)
add_test(NAME test_waveform_parse COMMAND test_waveform_parse)
