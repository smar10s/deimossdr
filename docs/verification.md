# Verification

How correctness is established, layer by layer. RTL work is not done
until it runs on hardware — simulation proves logic, hardware proves
the system.

For operational commands and wall times, see `docs/handbook.md`.

## Principle

Treat RTL like any compiled language: write, build, execute, test. The
build/flash/validate cycle is a compilation step, not an optional extra.
Wall time (20-25 minutes for build + flash) is not an excuse to defer
hardware verification — it runs unattended.

```
make bitstream          # compile
make package && make flash   # deploy
make validate           # confirm identity
```

A change that passes simulation but has never been flashed is an
untested change.

## Hardware Ladder

Four layers, each catching different classes of bugs. Skipping layers
wastes debug time — failures at layer N are cheaper to diagnose than
the same bug surfacing at layer N+2.

| Layer | Tool | Proves | Data path |
|-------|------|--------|-----------|
| Simulation | `sim.sh` | RTL logic correct | cocotb + golden vectors |
| HIL | `deimos_hil_inject` | Fabric decodes on real hardware | DDR -> fabric -> tag |
| Cable loopback | `deimos_fabric_loopback` | Full RF -> fabric decode | DAC -> cable -> ADC -> fabric |
| OTA | `deimos_rx_dump` | Real-world traffic decode | Antenna -> ADC -> fabric |

Each layer must pass before proceeding to the next.

## Canonical Full Validation

"Run a full RTL validation" means this sequence, in order:

```
./scripts/sim.sh                 # Layer 0 + full gate suite, no args
make bitstream                   # remote if REMOTE is set in config.mk
cat build/fpga/timing_status     # must read "met"
make firmware && make package && make flash
make validate                    # fingerprint + bitstream identity
make deploy                      # flash wipes /usr/bin; redeploy tools
./scripts/hil_regression.sh      # 80/80 session gate
./scripts/session_end.sh         # authoritative: sim + HIL + loopback, logged
```

`hil_test.sh` (8 layers) and `loopback_test.sh` (7 layers) are deeper
ladders for investigating a specific layer, not the merge gate.
`session_end.sh` is the authoritative end-of-session gate; it re-runs
sim, HIL, and loopback itself, so don't also run those standalone.

## Simulation Gate (`sim.sh`)

The full suite runs ~50 tests in ~7 minutes with no hardware. It is the
first gate for any change.

```
Layer 0: gen_sim_pipeline --check — sim view matches system_bd.tcl (~30s)
Layer 1: sim.sh (no args)    — full gate suite, ALL must pass
Layer 2: sim.sh test_rx_frontend  — live-mode front-end (40s)
Layer 2b: sim.sh test_adc_replay  — real ADC capture regression (30s)
```

Layer 0 verifies `rx_pipeline.v`/`rx_frontend.v` are exactly what the block
design generates: the BD is the single source of truth for pipeline wiring
(D19), so the sim view cannot drift. Wiring is authored in
`system_bd.tcl` (hardware) and `scripts/pipeline_sim_view.py` (sim harness
delta); regenerate with `python3 scripts/gen_sim_pipeline.py`. See D19.

## HIL Test Ladder (`hil_test.sh` — 8 layers)

Digital injection into the fabric, bypassing RF. Layer definitions are
in `scripts/hil_test.sh -h`. Layers 1-2 are the session gates (Layer 2
= `hil_regression.sh` 80/80); layers 3-8 cover CFO, rapid-fire, PSDU
verify, burst, capture replay, and impairments.

Session gate: **80 trials, 80 pass** (`hil_regression.sh`).

## Loopback Test Ladder (`loopback_test.sh` — 7 layers)

Cable loopback through the full analog path. Layer definitions are in
`scripts/loopback_test.sh -h`.

| Layer | What |
|-------|------|
| 1 | Single-frame RF decode (6 rates) |
| 2 | Burst detection (20 frames, 6 rates) |
| 3 | Rate mixing (3 combos) |
| 4 | Capture replay through cable (11 captures) |
| 5 | SIFS timing (gap=320, 6 rates) |
| 6 | PSDU verify (fabric BRAM readback, 3 rates) |
| 7 | EAPOL burst (SIFS + PSDU verify, 3 rates) |

Rates 48/54 are report-only in cable loopback. The AD9363
DAC-cable-ADC path's EVM floor (~-18 to -22 dB) is below the
-25 dB 64-QAM requires. These rates decode better OTA where real
transmitters have better EVM than the test setup (D16/D17).

Rate 48/54 loopback is **non-stationary** near the EVM floor: 54M has
measured 5/20 and 94/100 on the same bitstream across adjacent days. Do
not read a single trial delta — in either direction — as a regression.
The throughput claim for 48/54 is proven in simulation, never by a
loopback pass rate.

## ADC Replay Ratchet

Real ADC captures serve as permanent regression gates. Captures in
`captures/passing/` are replayed through the pipeline in simulation
(`test_adc_replay`) — a change that breaks replay of a previously-passing
capture is a regression, even if all synthetic tests pass.

The workflow for improving a rate:

```
1. Capture a failing frame: deimos_adc_capture -r <rate>
2. Characterize: ./scripts/characterize_failure.sh <rate>
3. Identify the failure mode from EVM analysis:
   - Progressive degradation -> channel tracking
   - Uniformly high EVM -> H estimation or timing error
   - Isolated bad symbols -> FFT or timing glitch
4. Fix in RTL, replay to verify
5. When replay passes: promote capture to captures/passing/
6. Full sim suite must still pass (no regression on other rates)
```

Characterization before proposing a fix is mandatory. The EVM analysis
takes 30 seconds and identifies the failure mode; guessing wastes hours.

## What Counts as Validation

| Valid | Not valid |
|-------|-----------|
| `sim.sh` gate tests | "It compiles" |
| `hil_regression.sh` 80/80 | `pluto_loopback` (ARM decode) |
| `deimos_fabric_loopback` | `pluto_sigladder` (ARM decode) |
| ADC capture -> sim replay | "SIGNAL decodes" without FCS |

ARM tools (`pluto_loopback`, `pluto_sigladder`) validate the RF test
setup. They prove the analog path works. They prove nothing about
fabric correctness.

## Commit Evidence

Commit messages include method and trial count for any validation claim:

- `sim 50/50 (sim.sh)`
- `HIL 80/80 (hil_regression.sh)`
- `cable loopback rate 24: 20/20 (deimos_fabric_loopback -n 20)`
- Timing: WNS, LUTs/17600, DSP/80, BRAM/60

## Stimulus Policy

`stimulus/` holds deterministic generated IQ streams (see
`stimulus/README.md`); `captures/` holds real ADC recordings only.
Real OTA captures (containing real MAC addresses, SSIDs, or EAPOL key
material) must never be committed. New waveform shapes go through
`scripts/gen_stimulus.py` (fixed seed 1337, `--verify` byte-compares).

Test value lives in the structure — frame lengths, rates, measured
STF-to-STF deltas, channel effects — not in the specific bytes. Generated
stimulus is reproducible, diffable, and releasable; field captures are not.
The generated burst preserves the measured gap schedule verbatim; do not
"normalize" it to uniform SIFS/DIFS, since the tight ACK chains are part of
the coverage.

## Tests vs Diagnostics

| | Tests (`test_*`) | Diagnostics (`diag_*`) |
|---|---|---|
| Purpose | Gate code correctness | Characterize behavior |
| Assertion | Hard assert | Never assert |
| `sim.sh` default | Included | NOT included |

Tests may not be weakened, skipped, or added in a failing state. If a
test fails, either the code is wrong or the test is wrong — both must
be fixed.
