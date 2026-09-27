# DeimosSDR

Pluto's slightly more martial relative - a 20 MSPS fixed-rate, near-real-time
802.11a/g accelerator in FPGA fabric for the PlutoSDR (Zynq Z-7010).

Supports all eight legacy OFDM rates from 6–54 Mbps. 6–36 Mbps runs within the
live symbol budget (real-time). 48/54 decode correctly but run 36 clocks/symbol
over budget, trading sustained throughput for buffering and clean drops.

This is a complete and open receiver implementation: no Xilinx IP blocks in the PHY.

While there is no direct fabric support for transmission (i.e. frame constructions/triggers), it is possible with [software](https://github.com/smar10s/lib80211) and the included [DMA engine](https://github.com/smar10s/styxsdr/).

## Why?

Because ADI [told us](https://archive.fosdem.org/2018/schedule/event/plutosdr/) you could go wardriving with it but apparently forgot to ship the WiFi PHY, and openwifi doesn't fit.

## Why fabric?

The Z-7010's ARM core cannot decode 20 MHz OFDM in real time — frames
are processed sequentially and everything that arrives mid-decode is lost.
USB 2.0 cannot stream raw IQ to a host either. So decode happens where the
samples are: the fabric consumes one sample per clock, front-to-back, and
emits a small metadata tag per frame. The ARM's only job is reading tags.

Full decode through FCS-OK is required because a hybrid solution like FPGA
for STF detection or SIGNAL decode only with ARM hand-off would still trigger
on too much noise.

Measured on live 5 GHz traffic (local AP): ~76 frames/sec of tags, 45%
HT/VHT frames correctly rejected (L-SIG decoded, DATA not), 79–95% true
legacy FCS rate depending on channel conditions. On OTA traffic, rates
6/12/24 alone carry 95%+ of decodable frames — beacons, probes, and
EAPOLs are all legacy OFDM.

Per frame, the fabric emits:

```
tag: { fcs_ok, rate[3:0], length[11:0], psdu_addr[13:0] }
```

FCS-OK frames have their PSDU bytes stored in a 16 KB BRAM ring; the
tag carries the address, and the ARM reads bytes back through a cursor
register.

## Rate support and throughput

The per-symbol budget is 400 fabric clocks (80 samples × 5 clocks at
100 MHz fabric / 20 MSPS ADC). Measured via `diag_symbol_cycle` /
`diag_throughput` (deterministic compute; the period is feed-jittery at low
rates; ratcheted in `fpga/test/test_latency_ratchet.py`):

| Rate (Mbps) | Mod / code rate | clk/sym | vs budget | Status |
|-------------|-----------------|---------|-----------|--------|
| 6           | BPSK, 1/2       | 327     | −73       | real-time |
| 9           | BPSK, 3/4       | 316     | −84       | real-time |
| 12, 18      | QPSK, 1/2 · 3/4 | 340     | −60       | real-time |
| 24, 36      | 16-QAM, 1/2 · 3/4 | 388  | −12       | real-time |
| 48, 54      | 64-QAM, 2/3 · 3/4 | 436  | +36       | over budget — buffered, clean-drop |

All eight rates decode correctly (HIL digital injection: 100%). Caveats:

- **48/54** run 36 clocks over the per-symbol budget. Single frames and
  normal traffic decode fine (32K-sample IQ buffer absorbs the backlog);
  sustained SIFS-spaced bursts eventually saturate and the design
  **drops cleanly** — `tag_abort`, never a corrupt tag (D23). 48 is
  reachable by lead-in reduction; 54 additionally needs a ≥2 bit/clk
  depuncturer (its 1 bit/clk output needs 432 clk/sym, a hard floor no
  lead-in work can move). See `docs/throughput-analysis.md`.
- **48/54 cable-loopback EVM** sits at −18 to −22 dB versus the < −25 dB
  64-QAM needs — the AD9363 DAC→cable→ADC path's noise floor (D16/D17).
  This is separate from the throughput gap above. They are report-only in
  the test ladders, and they decode better OTA (real transmitters have
  better EVM than our test setup).

## Resource usage

Flattened `system_top` after placement (`ExtraPostPlacementOpt` place +
`ExploreArea` opt, non-incremental), Z-7010, build fingerprint
`0x647f83bb`:

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUTs     | 15,695 | 17,600 | 89% |
| FFs      | 15,063 | 35,200 | 43% |
| DSP48E1  | 78 | 80 | 98% |
| BRAM     | 43 | 60 | 72% |

Timing: WNS 0.169 ns, WHS 0.024 ns at 100 MHz. LUT headroom is the binding
constraint (~1,900 LUTs) — the DSP/BRAM margins are incidental, not targets. At this
utilization the place directive is netlist-sensitive: see the D21
amendments before assuming a given strategy still fits.

## Validation

| Layer | Gate | Description |
|-------|------|-------------|
| Simulation | `sim.sh` (full suite) | Generated sim-view freshness (D19), golden vectors, latency ratchets, ADC replay |
| HIL | `hil_regression.sh` | 10 trials × 8 rates, 80/80 must pass |
| HIL ladder | `hil_test.sh` | 8 layers: sanity, statistical decode, CFO, rapid-fire, PSDU verify, burst, capture replay, impairments |
| Cable loopback | `deimos_fabric_loopback -n 20` | 20/20 at gate rates 6–36 |
| Loopback ladder | `loopback_test.sh` | 7 layers: single-frame, burst, rate mixing, capture replay, SIFS, PSDU verify, EAPOL |
| OTA | `deimos_rx_dump` | continuous tag stream, ~76 frames/sec |
| OTA interop | `eapol_toggle_test.sh` | macOS based STA/AP interop |

48/54 are report-only in cable loopback (see above). A 4-way EAPOL
burst with SIFS timing is a standing loopback ladder gate (layer 7).

## Usage

### What needs what

Simulation is the zero-hardware entry point: clone, install five pip
packages plus verilator, and the full 50-test gate suite runs against
golden vectors from the `extern/lib80211` submodule. Nothing else in the
table below is required for that.

| Task | Needs |
|------|-------|
| **Simulation** (`sim.sh`) | Python 3.11 + `requirements.txt`, verilator |
| Bitstream (`make bitstream`) | Vivado 2025.2 on Linux (Z-7010 is WebPACK-eligible, free) |
| Firmware (`make firmware`) | Docker (ARM cross-compile toolchain image) |
| Base firmware (`make setup`) | Docker, **and Vivado on PATH** (builds the ADI IP library) |
| `pluto.frm` (`make package`) | Linux: `fakeroot`, `dtc`, `mkimage`, `md5sum` |
| Any hardware step (flash, HIL, loopback) | `sshpass`, a PlutoSDR at `192.168.2.1` |
| Cable loopback ladder | SMA cable + attenuator between TX and RX |
| DFU recovery | `dfu-util` |

Python is pinned to 3.11 as a legacy pin, not a cocotb constraint (cocotb 2.0.1 supports through 3.13); 3.12+ would let `py80211` install as a package. See `docs/handbook.md`.

`make bitstream` and `make package` assume Linux Vivado, set `REMOTE` in `config.mk` to
build over SSH from other systems.

### Prerequisites

```bash
# simulation toolchain
brew install verilator            # 5.044 validated
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt

# submodules — REQUIRED, before sim: lib80211 supplies py80211 (imported
# by the sim tests via sys.path injection), platform/ supplies the STYX base
git submodule update --init --recursive

# optional: direnv auto-activates .venv and sets SIM=verilator
direnv allow
```

Without direnv, `scripts/sim.sh` still finds `.venv` on its own — the
path is hardcoded relative to the repo root. `scripts/sim.sh` does not
check for the submodules, so skipping the step above surfaces as a
`py80211` import error partway through the run.

Then verify the toolchain before touching hardware:

```bash
./scripts/sim.sh              # 50 gate tests, ~7 min, no hardware needed
```

### Build and flash

```bash
cp config.mk.example config.mk   # defaults build locally; edit for remote
make setup        # one-time: init submodules, build plutosdr-fw (~2h)
make bitstream    # Vivado (local, or remote via config.mk)
make firmware     # ARM cross-compile (Docker)
make package      # pluto.frm (bitstream + kernel + rootfs)
make flash        # flash PlutoSDR
make deploy       # deploy on-device tools over SSH (faster than reflash)
make validate     # confirm the flashed build matches your local build's fingerprint
```

`make setup` needs Vivado on PATH — it builds the ADI HDL IP library that
`make bitstream` consumes. See `bin/` (flash, validate) and `packaging/`
(FIT image assembly, device-tree overlay) for what the last four do.

Try DFU recovery if a flash produces a non-booting device.

### Controlling the receiver

The decode engine is held in reset until enabled. AXI register blocks
control it (full map: `docs/registers.md`, generated from RTL):

| Register | Address | Bits | Meaning |
|----------|---------|------|---------|
| `STF_THRESH` | `0x7C520000` | [7:0] | STF detection threshold (0 = most sensitive) |
| `STF_ENABLE` | `0x7C520004` | [0] | detector enable; 0 = held in reset (default) |
| `SNAP_MODE` | `0x7C52000C` | [2:0] | snap observation point selector |
| `VERSION` | `0x7C52001C` | [31:0] | firmware ID, read-only |

Start the receiver by writing `STF_ENABLE = 1` after configuring the
AD9361; write `0` to hold it in reset (default state).

### Reading tags and PSDU bytes

Tag FIFO at `0x7C510000` (16 entries). The ARM-side protocol:

```
1. Read  TAG_LO    — peek {fcs_ok, rate[3:0], length[11:0]}
2. Read  TAG_HI    — pop: psdu_addr[13:0], sets the PSDU cursor
3. Read  PSDU_DATA — length−4 times: PSDU bytes, cursor advances
4. Repeat from 1
```

For FCS-failed frames, skip step 3. `STATUS` (0x00) carries
`tag_empty`/`tag_full`/`tag_count`; `FRAME_CNT`/`DROP_CNT` (0x10/0x14)
count produced and dropped frames. If the FIFO is not drained, drops
are clean: the tag is counted in `DROP_CNT`, never half-emitted.

### On-device tools

All of the above is already implemented in the firmware tools — most
users never touch registers directly:

```bash
# live OTA decode to JSONL (rate, FCS, class, MACs, PSDU hex per frame)
deimos_rx_dump -c 36 -d 30

# HIL: inject a golden vector into the fabric, verify PSDU (no RF needed)
deimos_hil_inject 6mbps -v

# cable loopback: DAC → cable → ADC → fabric decode, per-rate trials
deimos_fabric_loopback -c 149 -r 24 -n 20

# capture raw ADC IQ to DDR (for replay/characterization)
deimos_adc_capture -r 24
```

`deimos_burst_loopback` covers multi-frame bursts with PSDU verify.
Each tool prints `-h` on the device.

### Simulation

```bash
./scripts/sim.sh                    # full gate suite (cocotb + verilator)
./scripts/sim.sh test_rx_frontend   # one target
make waves TARGET=test_rx_frontend  # VCD waveforms
```

## Documentation

`docs/architecture.md` covers the goal, constraints, and design.
`DECISIONS.md` records numbered design decisions with full rationale.
`docs/verification.md` defines the hardware validation ladder.
`docs/handbook.md` has build commands, platform access, and environment
setup. See `docs/` for analysis documents (throughput, Q-format budget,
correlator reduction, pipeline features).

The RTL is plain Verilog under `fpga/rtl/`, tested with cocotb under
`fpga/test/`, against golden vectors from the `extern/lib80211`
submodule. `platform/styx` is the platform submodule (DMA, HIL, snap).

## License

MIT — see `LICENSE`. Submodules (`extern/lib80211`, `platform/styx`)
are MIT under their own terms.
