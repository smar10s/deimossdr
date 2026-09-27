# Architecture

WiFi monitor-mode receiver on PlutoSDR (Zynq Z-7010). Real-time 802.11
legacy OFDM decode in FPGA fabric. Goal, constraints, and design
rationale for contributors; for the pitch, quick start, and headline
numbers see the root `README.md`.

## Goal

Near-100% duty cycle for legacy OFDM traffic. The fabric decodes rates
6–54 Mbps inline at wire speed, freeing ARM for application logic, channel
hopping, and supplementary decode of traffic the fabric doesn't handle
(HT/VHT).

Concrete success metric: reliably capture a 4-way EAPOL handshake on a
target channel using streaming fabric decode. This exercises STF detection,
SIGNAL decode, DATA decode, and continuous operation without frame loss.

## Why Fabric Decode (Not ARM)

The ARM cannot keep up. At 667 MHz single-core, decoding 20 MHz OFDM in
real time is not feasible — and USB 2.0 cannot stream raw IQ to a host
either. If decode happens on ARM, it processes frames sequentially and
drops everything that arrives while it's busy. Measured beacon rate on a
single AP is ~10/sec; a busy channel has 30–50+ frames/sec at various
rates. ARM chokes on beacon decode and misses EAPOLs.

Fabric decode eliminates this: every frame is decoded at wire speed,
ARM just reads metadata from the tag FIFO, and the channel has effectively
100% duty cycle for supported rates.

## Why Legacy-Only Is Sufficient

Measured on live 5 GHz traffic (reproducible via
`extern/lib80211/tools/hardware/traffic_analysis_c.py`):

| Rate | Share | Frame types |
|------|-------|-------------|
| 6 Mbps | 51% | Beacons, probes, auth, deauth |
| 24 Mbps | 44% | EAPOL, null-data, association |
| 12 Mbps | 4% | Misc management |

95%+ of decodable frames use legacy OFDM. 100% of decoded frames in the
sample were legacy format (zero HT/VHT in the management/control plane).

Rates 6, 12, and 24 are all code rate 1/2 — the simplest Viterbi path.
Higher rates (36–54) use code rates 2/3 and 3/4, requiring only a
depuncturer change. The decode pipeline is the same for all 8 rates.

## Architecture

```
AD9363 ADC → [fabric: STF → CFO → FFT → EQ → pilot → demod → Viterbi → FCS]
                                                                    |
                                                              tag FIFO → ARM
```

Streaming pipeline: each module processes one sample or symbol per clock,
front-to-back, within a 400-cycle symbol budget (80 samples × 5 fabric
clocks/sample at 100 MHz fabric / 20 MSPS ADC). No bulk buffering between
stages except where pipeline latency requires it (FFT delay lines, Viterbi
traceback).

### Tag + PSDU Output

Fabric emits a tag per frame: `{fcs_ok, rate[3:0], length[11:0], psdu_addr[13:0]}`.
For FCS-OK frames, PSDU bytes are stored in an 8×RAMB18E1 ring buffer (16 KB)
and read back via a cursor register. ARM reads PSDU_DATA × (length - 4) times
after popping the tag.

Tag FIFO is 16 entries deep. ARM must drain within ~200ms at typical frame
rates to avoid drops.

### Frame Classification (Application-Level)

Tag fields (rate, length, fcs_ok) support zero-cost frame classification
without parsing PSDU bytes:

| Class | Criteria (from tag) |
|-------|---------------------|
| `beacon` | rate 6, len 300-500, FCS OK |
| `mgmt` | rate 6, len 50-299, FCS OK |
| `probe` | rate 6, len >500, FCS OK |
| `ack` | len ≤14, or len=20 rate 6/12/24 |
| `ba` | rate 12/24 len 28-32 |
| `ht_vht` | FCS fail + known L-SIG pattern; or rate 6 len=23 (D15) |
| `data` | other FCS OK |
| `unknown` | other FCS fail |

This classification is useful for filtering, logging, and future application
logic (e.g., EAPOL hunting). It lives in `deimos_rx.c:deimos_rx_classify_frame()`
and is independent of any decode decision.

### OTA Performance (ch36, 5 GHz antenna)

- Frame rate: ~76 frames/sec (fabric STF detections producing tags)
- Traffic mix: 45% HT/VHT (correctly rejected), 55% real legacy
- True legacy FCS rate: 79-95% (depends on channel conditions)

Reference: `docs/fcs_rate_analysis.md`

### Firmware Tools

- `deimos_hil_inject` — HIL golden vector injection (+ PSDU verify)
- `deimos_fabric_loopback` — single-frame cable loopback via snap probe
- `deimos_burst_loopback` — multi-frame burst loopback via tag FIFO + PSDU verify
- `deimos_adc_capture` — raw ADC IQ capture to DDR, dump as JSON
- `deimos_rx_dump` — streaming fabric decode to JSONL (OTA monitoring, EAPOL)

Built with `make firmware` (ARM cross-compile via Docker). The styx platform
also provides `pluto_loopback` / `pluto_sigladder` for ARM-side reference
decode — these validate the RF test setup, not the fabric (see lib80211
Boundary below).

## Hardware Constraints

| Resource | Limit | Implication |
|----------|-------|-------------|
| Z-7010 LUTs | 17,600 | Every module must be resource-estimated before commit |
| Z-7010 DSP48E1 | 80 | Multiplies are precious; prefer shifts/adds |
| Z-7010 BRAM | 60 | Available for FIFOs and ROMs |
| USB 2.0 | ~5 MB/s | Cannot stream 20 MHz IQ to host |
| ARM | 667 MHz single-core | Cannot decode OFDM in real time |
| DDR3 | 512 MB shared | 128 MB RX ring, 32 MB TX, rest Linux |

## Verification

Four layers: simulation, HIL digital injection, cable loopback, OTA.
Each catches different bug classes — skipping layers wastes debug time.
A module is not done until it passes HIL. The system is not done until
OTA demonstrates continuous EAPOL capture.

Full ladder definitions, sub-ladders, and the ADC replay ratchet:
`docs/verification.md`.

## Pipeline Features

Non-obvious implementation details documented in `docs/pipeline-features.md`.
Each entry describes what's there and why — a compact architecture changelog.

## Design Decisions

Recorded in `DECISIONS.md` with full rationale. Read the relevant entry
before revisiting any settled decision.

## Project Layout

```
LICENSE             — MIT
README.md           — public landing page (pitch, quick start, numbers)
AGENTS.md           — agent session rules
DECISIONS.md        — numbered design decisions with rationale
requirements.txt    — Python deps for simulation and tooling
config.mk.example   — build configuration template (copy to config.mk)
docs/architecture.md — this file (goal, constraints, architecture)
docs/registers.md   — AXI register map (GENERATED, see docs/handbook.md)
docs/debugging.md   — debug methodology playbook
extern/             — external dependencies (lib80211 submodule)
platform/styx/      — platform submodule (SDR base: DMA, HIL, snap, HAL)
fpga/rtl/           — all custom Verilog
fpga/test/          — cocotb tests
firmware/src/       — firmware library + HAL
firmware/tools/     — on-device tools (HIL, loopback, capture, rx_dump)
scripts/            — build, flash, test automation
scripts/hooks/      — git hooks (install with `make hooks`)
bin/                — flash.sh (write pluto.frm), validate.sh (verify flashed build)
packaging/          — pluto.frm assembly: FIT image, device-tree overlay, rootfs branding
stimulus/           — generated IQ stimulus (deterministic, seeded); captures/ = real ADC recordings only
```

`STATUS.md` and `logs/` are private-dev-repo artifacts (session handoff
state and machine-written hardware evidence) and are not part of the
public mirror.

Build commands, wall times, and platform reference: `docs/handbook.md`.

## lib80211 Boundary

Git submodule at `extern/lib80211/`. Dual purpose: golden vectors for RTL verification, and ARM
reference decode for test-setup validation. Bug fixes go upstream first.

ARM decode results (via `pluto_loopback`, `pluto_sigladder`) validate the
analog/DMA path only. They prove the test setup works. They are NOT fabric
progress and NEVER constitute "done" for any RTL task.
