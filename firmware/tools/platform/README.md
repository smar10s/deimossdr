# Platform Tools

These tools validate the PlutoSDR platform: AD9361 configuration, DMA engines,
DDR ring buffer, and analog RF path. They use **lib80211 on ARM** for decode.

**They do NOT exercise the FPGA fabric decode pipeline.**

Source code lives in the `platform/styx/` submodule (converged with styxsdr).
Only the CMakeLists.txt build configuration is local to deimos.

## Tools

| Tool | Purpose |
|------|---------|
| `pluto_loopback` | Cable loopback: TX waveform → DAC → cable → ADC → DDR → ARM lib80211 decode |
| `pluto_sigladder` | Graduated signal complexity: tone → chirp → preamble → full frame |
| `pluto_dma_test` | DMA register-level start/stop/restart validation |

## When to use

- Suspected DMA or RF path regression
- After modifying `iq_dma_rx`, `adc_sync`, DDR layout, or AD9361 config
- After lib80211 submodule bumps
- **NEVER as validation for fabric/RTL decode work**

## For fabric validation

Use `deimos_hil_inject` (in parent directory). That's the only tool that
exercises the FPGA decode pipeline.
