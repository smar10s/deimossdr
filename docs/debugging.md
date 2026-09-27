# Debugging Methodology

Living document. Records what methods work for what classes of bugs in
this project. Add to it as new patterns emerge.

## Method: Differential Signal Capture (cocotb)

**When to use:** Two code paths should produce identical results but don't.
A/B comparison bugs. "Normal path works but an alternate path fails."

**What it does:** Runs both paths in the same test, captures numerical
arrays at every module boundary, diffs them in Python. Identifies the
exact pipeline stage, symbol, and sample where divergence begins.

**How:**
1. Write a `_feed_with_capture()` helper that hooks into DUT signals on
   each `RisingEdge(clk)`, accumulating values into lists per stage.
2. Run Case A (known-good path), collect arrays.
3. Run Case B (failing path), collect arrays.
4. Compare stage-by-stage. First divergence = bug location.

**Capture points for the RX pipeline:**
```
buf_re/buf_im       — BRAM read output (pre-mixer)
feed_mixer_re/im    — mixer output (post-CFO correction)
eq_data_re/im       — equalizer output (48 subcarriers/symbol)
eq_pilot_re/im      — equalizer pilots (4/symbol)
pilot_track.phase_acc — PLL state per symbol
pt_data_re/im       — pilot-corrected output (48 subcarriers/symbol)
```

**Key patterns revealed:**
- If buffer reads diverge but rd_ptr/wr_ptr match → data was corrupted
  in the buffer (write timing issue, overwrite, or sample drop).
- If buffer reads match but mixer diverges → NCO/phase_inc mismatch.
- If EQ diverges but mixer matches → channel estimate or FFT issue.
- If PT diverges but EQ matches → pilot tracking state leak.
- 1-sample shift pattern (B[N] = A[N+1]) → sample dropped in buffer
  write path, causing all subsequent reads to be off-by-one.

**Strengths:**
- 22 seconds per run (fast iteration)
- Definitive (shows exact values, not inferred from VCD transitions)
- Handles BRAM pipeline latency correctly (captures on valid signals)
- Works in Verilator (no simulator-specific timing issues)

**Example:** `test_pending_trigger.py::test_pending_differential_capture`

---

## Method: VCD + vcd_query.py

**When to use:** Timing relationships between signals. "When does X fire
relative to Y?" Symbol-level timing budgets. Understanding FSM sequencing.

**What it does:** Generates VCD waveforms via `make waves`, then queries
with `vcd_query.py` for transitions, timing, and symbol comparisons.

**Tools:**
```bash
make waves TARGET=<target> [TESTCASE=name]
python scripts/vcd_query.py list <vcd> <pattern>
python scripts/vcd_query.py extract <vcd> signal1 signal2 --start T --end T
python scripts/vcd_query.py symbol-timing <vcd>
python scripts/vcd_query.py compare-symbols <vcd> --good N --bad M
```

**Limitations:**
- VCD records transitions, not per-clock values. For registered BRAM
  outputs (1-cycle latency), it's ambiguous which clock edge "owns"
  which transition at picosecond timescales.
- Cannot reliably compare numerical arrays between two runs (different
  VCD files have different absolute timestamps).
- Large VCD files (>100MB for full pipeline tests).

**When NOT to use:**
- Per-sample numerical comparison between two code paths (use
  differential capture instead).
- Debugging BRAM read/write races (VCD shows both read and write
  changing on the same timescale — can't distinguish which won).

---

## Caveat: cocotb internal-signal reads

Reading internal (non-port) signals through the public-flattened hierarchy
is unreliable — values can lag a cycle or come back wrong. They are not
ground truth. Read module output ports, or use VCD + `vcd_query.py` to
observe internal state. When a test result depends on an internal signal
read, re-verify it against a port or VCD before trusting it.

---

## Method: EVM Analysis (diag_evm)

**When to use:** Hardware failures at specific rates. "Rate 48 passes 70%
but rate 54 only 50%." Understanding whether the dominant error is phase
drift, noise, or timing.

**What it does:** Runs ADC capture through the full pipeline in sim,
computes per-symbol EVM (error vector magnitude) pre- and post-pilot
tracking. Shows where in the frame errors accumulate.

**Tools:**
```bash
./scripts/characterize_failure.sh <rate>        # hardware capture + EVM
make -C fpga/test diag_evm ADC_CAPTURE_FILE=<path>  # sim-only
```

**Key patterns:**
- Progressive EVM degradation → channel tracking / pilot PLL drift
- Uniformly high EVM → H estimation or timing error
- Isolated bad symbols → FFT window misalignment or CP contamination

---

## Method: Buffer Pointer Tracking

**When to use:** Suspected buffer overwrite, underrun, or timing race.
"Data was correct earlier but corrupted by the time it's read."

**What it does:** Captures `rd_ptr`, `wr_ptr`, and headroom
(`wr_ptr - rd_ptr`) at the start of each feed burst. Reveals:
- Whether the producer is keeping ahead of the consumer
- Whether the write window has gaps where samples can be dropped
- Whether addresses wrap and collide

**Key insight from 2026-06-21 session:** A buffer write window that
doesn't span the full inter-frame transition causes 1-sample drops.
The symptom is a 1-sample shift in buffer reads starting at the first
DATA symbol whose buffer position exceeds the wr_ptr at frame-end.

---

## Method: Golden Model Comparison (lib80211)

**When to use:** Need to determine whether hardware/RTL output is
numerically correct for a given input. "Is the equalizer producing the
right subcarrier values?" Reference decode of the same IQ.

**What it does:** Feeds identical IQ to both the RTL (via cocotb) and
the Python golden model (lib80211), compares outputs sample-by-sample.

**Not yet used in practice** — reserved for cases where differential
capture shows divergence at a specific stage and the correct values
aren't obvious from the A/B comparison.

---

## Method: OTA IQ Capture → Replay Fork (fabric vs upstream)

**When to use:** OTA decode loses a specific frame that an independent monitor
receiver catches. "Is the loss in the fabric, upstream of it (RF/AGC/SNR), or
in the ARM/tag-drain path?"

**What it does:** Captures one long raw-ADC window around a trigger frame,
then replays the exact IQ through (a) the RTL in sim and (b) lib80211, and
compares both against the fabric's own OTA tag log for that window.

| lib80211 | sim RTL | OTA fabric | Verdict |
|----------|---------|-----------|---------|
| decodes | misses | misses | **Fabric defect** — deterministic, failing vector captured |
| decodes | decodes | misses | **Fabric state/timing or drain** during the live run |
| misses | misses | misses | IQ/SNR unrecoverable (or the frame was never on air) |

The sim replay feeds `iq_valid` at 1-in-5 (live ADC timing), so it preserves
the live re-arm/accumulator state; replay the **whole** window, not just the
target frame, or the state interaction is lost.

**Why not cable loopback:** loopback re-modulates a *golden* vector through
DAC→cable→ADC with fixed manual gain (D18). It cannot replay the captured OTA
waveform and adds analog EVM — it answers a different question.

**HIL caveat:** `deimos_hil_inject` feeds `iq_valid` 1-per-clock (test mode),
not the live 1-in-5. It can mask live-timing/state bugs — use HIL to confirm a
fix on the real netlist, not as the primary repro.

**Trigger hygiene:** match the trigger frame's *direction*, not just
rate/length. M1 (137, AP→STA) and M4 (137, STA→AP) share a length; a
length-only trigger selects M4 windows in which the "M2 absent" check is
trivially true. `deimos_ota_capture --trigger-dir ap` enforces this.

**Drop caveat:** `deimos_ota_capture`'s post-window bulk `ring_read` stalls the
tag drain and can overflow the 16-deep FIFO. Trust the IQ + sim/lib80211 for
the verdict, not the capture's OTA tag log.

**Tools:** `firmware/tools/deimos_ota_capture`,
`scripts/host/ddr_capture_to_stimulus.py`, `fpga/test/diag_ota_window_replay.py`,
`scripts/host/validate_window.c`.

---

## Anti-patterns (What Doesn't Work)

| Approach | Why it fails |
|----------|-------------|
| VCD per-clock sampling of BRAM outputs | Picosecond ambiguity; can't determine which clock edge owns which value |
| Manual `cocotb.log.info` at one signal | Too narrow; need all boundaries to localize |
| Guessing "it's probably pilot_track" | 3 sessions wasted. Instrument first, hypothesize second |
| Testing with zero CFO only | Zero-CFO masks timing bugs because phase errors don't accumulate |
| Extending debug without fresh sim | Stale sim_build directories cause confusing results |

## General Principles

1. **Instrument first, hypothesize second.** Build the measurement tool
   before guessing at the cause. A 22-second instrumented test beats
   hours of reasoning about VCD transitions.

2. **Compare at every boundary.** Don't skip stages. The first divergence
   IS the bug location. Later stages just propagate the error.

3. **Clean sim builds.** After RTL changes, `rm -rf sim_build_*` for the
   affected targets. Stale Verilator builds with --trace flags cause
   confusing performance and correctness issues.

4. **Include CFO in test cases.** Zero-CFO hides bugs that only manifest
   when phase accumulates over many symbols. Use 100kHz+ CFO for stress.

5. **Count samples, not clocks.** The pipeline runs at 1 IQ sample per 5
   clocks. Buffer positions, frame offsets, and symbol boundaries are all
   in sample-space. Clock counts are misleading.

## Debug Snap Register Reference

32-bit × 1024 sample circular buffer with trigger. Captures live ADC IQ
at fabric speed for ARM-side inspection.

**Address**: 0x7C4E_0000

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| 0x00 | CONTROL | RW | [0]=arm, [1]=sw_trigger(W1S), [2]=circular |
| 0x04 | STATUS | RO | [0]=captured, [1]=armed, [25:16]=trig_pos |
| 0x08 | TRIG_CYCLE | RO | Cycle counter at trigger |
| 0x0C | RD_ADDR | RW | Read address (10-bit) |
| 0x10 | RD_DATA | RO | Data at RD_ADDR |

**Sample format**: `{8'b0, I[11:0], Q[11:0]}` — upper 8 bits reserved.

**Usage from ARM**:
```bash
P="sshpass -p analog ssh root@192.168.2.1"

# Arm
$P "devmem 0x7C4E0000 32 0x01"

# Trigger (arm must already be set)
$P "devmem 0x7C4E0000 32 0x03"

# Check capture complete
$P "devmem 0x7C4E0004 32"   # bit 0 = captured

# Read sample at address N
$P "devmem 0x7C4E000C 32 N; devmem 0x7C4E0010 32"
```

**Connection**: Taps adc_sync output (same IQ stream as iq_dma_rx).
ext_trig currently tied to 0 — software trigger only.
