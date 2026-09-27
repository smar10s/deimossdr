# Platform tools — from styx submodule
#
# These validate the PlutoSDR platform infrastructure (AD9361 config,
# DMA engines, DDR ring buffer, analog RF path). They use lib80211 on
# ARM for decode — they do NOT exercise the FPGA fabric decode pipeline.
#
# Sources come from platform/styx/ submodule to stay converged with styxsdr.

set(STYX_TOOLS_DIR ${CMAKE_SOURCE_DIR}/../platform/styx/firmware/tools)

add_executable(pluto_loopback ${STYX_TOOLS_DIR}/platform/pluto_loopback.c)
target_link_libraries(pluto_loopback styx_tool lib80211)
target_link_options(pluto_loopback PRIVATE -static)

add_executable(pluto_sigladder ${STYX_TOOLS_DIR}/platform/pluto_sigladder.c)
target_link_libraries(pluto_sigladder styx_tool lib80211 fftw3f)
target_link_options(pluto_sigladder PRIVATE -static)

add_executable(pluto_dma_test ${STYX_TOOLS_DIR}/platform/pluto_dma_test.c)
target_link_libraries(pluto_dma_test styx_tool)
target_link_options(pluto_dma_test PRIVATE -static)

add_executable(pluto_stream_test ${STYX_TOOLS_DIR}/platform/pluto_stream_test.c)
target_link_libraries(pluto_stream_test styx_tool)
target_link_options(pluto_stream_test PRIVATE -static)
# -- styx upstream sources `pluto_stream_test.c` warnings (unused-const,
#    maybe-uninitialized) that deimos's global -Werror promotes to errors;
#    styx builds without -Werror so upstream never sees them. Suppress here
#    until styx fixes the warnings; then drop this line.
target_compile_options(pluto_stream_test PRIVATE -Wno-error)
