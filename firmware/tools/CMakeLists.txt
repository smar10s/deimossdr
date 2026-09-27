# Deimos fabric validation tools
#
# All tools link against deimos_tool (which provides styx_tool + deimos_hal +
# shared utilities). Individual tools only need explicit lib80211/fftw3f when
# they call lib80211 TX/RX directly.

set(TOOL_LIBS deimos_tool m)

add_executable(deimos_hil_inject deimos_hil_inject.c)
target_link_libraries(deimos_hil_inject ${TOOL_LIBS})
target_link_options(deimos_hil_inject PRIVATE -static)

add_executable(deimos_fabric_loopback deimos_fabric_loopback.c)
target_link_libraries(deimos_fabric_loopback ${TOOL_LIBS})
target_link_options(deimos_fabric_loopback PRIVATE -static)

add_executable(deimos_adc_capture deimos_adc_capture.c)
target_link_libraries(deimos_adc_capture ${TOOL_LIBS})
target_link_options(deimos_adc_capture PRIVATE -static)

add_executable(deimos_burst_loopback deimos_burst_loopback.c)
target_link_libraries(deimos_burst_loopback ${TOOL_LIBS})
target_link_options(deimos_burst_loopback PRIVATE -static)

add_executable(deimos_rx_dump deimos_rx_dump.c)
target_link_libraries(deimos_rx_dump ${TOOL_LIBS})
target_link_options(deimos_rx_dump PRIVATE -static)

# ONE-OFF diagnostic (M2-loss investigation): M1-triggered raw-DDR window
# capture. Built here (needs the ARM toolchain) but deliberately NOT in the
# `make deploy` list — deploy with a one-off scp. Delete when the
# investigation closes.
add_executable(deimos_ota_capture deimos_ota_capture.c)
target_link_libraries(deimos_ota_capture ${TOOL_LIBS})
target_link_options(deimos_ota_capture PRIVATE -static)

# Platform tools (from styx submodule — RF path and DMA validation)
add_subdirectory(platform)
