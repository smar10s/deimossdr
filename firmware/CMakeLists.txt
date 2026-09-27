cmake_minimum_required(VERSION 3.20)
project(deimos-firmware C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The on-device tools are ARM-only (static, target headers). A host-native
# configure for the unit tests sets this OFF so add_subdirectory(tools) is
# skipped and no cross-compiled binary is attempted. See top-level `make test`.
option(DEIMOS_BUILD_TOOLS "Build on-device ARM tools" ON)

add_compile_options(-Wall -Wextra -Werror)

enable_testing()

set(LIB80211_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LIB80211_BUILD_OTA OFF CACHE BOOL "" FORCE)
set(LIB80211_FFT_BACKEND "fftw3" CACHE STRING "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../extern/lib80211 ${CMAKE_BINARY_DIR}/lib80211 EXCLUDE_FROM_ALL)

# Styx HAL — DMA + conversion + hal (all from submodule)
add_library(styx_hal STATIC
    ../platform/styx/firmware/src/hal.c
    ../platform/styx/firmware/src/dma_rx.c
    ../platform/styx/firmware/src/dma_tx.c
    ../platform/styx/firmware/src/convert.c
)
target_include_directories(styx_hal PUBLIC
    src/
    ${CMAKE_CURRENT_SOURCE_DIR}/../platform/styx/firmware/src/
)
target_link_libraries(styx_hal PUBLIC m)

# Styx tool scaffolding — signal handling, RF config, TX+capture (from submodule)
add_library(styx_tool STATIC
    ../platform/styx/firmware/tools/common/styx_tool.c
)
target_include_directories(styx_tool PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../platform/styx/firmware/tools/common/
)
target_link_libraries(styx_tool PUBLIC styx_hal)

# Deimos-specific library — pipeline
add_library(deimos_hal STATIC
    src/deimos_rx.c
)
target_include_directories(deimos_hal PUBLIC src)
target_link_libraries(deimos_hal PUBLIC styx_hal lib80211)

# Deimos tool scaffolding — JSON waveform parsing, DDR quantization, frame builders
add_library(deimos_tool STATIC
    tools/common/deimos_tool.c
)
target_include_directories(deimos_tool PUBLIC tools/common/)
target_link_libraries(deimos_tool PUBLIC deimos_hal styx_tool)

# Diagnostic tools (ARM target only) or host unit tests. The test sources are
# host-only (they exercise libc stdio/alloc), so they are never cross-built.
if(DEIMOS_BUILD_TOOLS)
    add_subdirectory(tools)
else()
    add_subdirectory(test)
endif()
