# Deimos Decisions

Choices that constrain future work. An entry is a decision — not a finding,
status update, build log, or plan. Constraints appear *inside* an entry: they
either forced the choice (most resource decisions here are forced by the
Z-7010 or the AD936x) or the choice creates one (a "do not regress"). A
constraint that is merely current state — what is binding today, measured
numbers — lives in the document that owns the subject, not here.

Each entry states **Decision** (what we will and won't do), **Rationale**
(why), and **Consequence / do not regress** (what it now constrains), plus an
optional **Rejected** for alternatives already tried. Evidence appears only
where it justifies the choice.

Entries are not a log: no chronological amendment trails, no run counts that
move, no links to plans. When a choice changes, either revise the entry to be
evergreen (fold the history into the *why*) or open a new entry that
supersedes it. Numbers are stable IDs; a withdrawn entry keeps its number as
a one-line tombstone.

---

## Authorship and approval

Entries in this file are authored by humans only. An agent may propose a
decision — as draft text offered for review — but may not create, edit,
renumber, or tombstone an entry, nor change this section, without the user's
explicit prior approval. Approval is per-change: it does not carry forward to
later edits. Suggestions are welcome; unilateral edits are not.

---

## D1: Legacy-only decode in fabric (2026-05-11)

**Decision:** FPGA implements 802.11a legacy OFDM decode only (20 MHz, rates
6-54 Mbps, BCC). HT/VHT frames are decoded on ARM from DDR when needed.

**Rationale:** Management and control frames (beacons, probe req/resp, auth,
deauth, EAPOL) are transmitted at legacy rates. This is our target traffic.
HT/VHT adds complexity (LDPC, STBC, wider bandwidths) that doesn't fit in
Z-7010 and doesn't serve the primary use case.

**Consequence:** We will see SIGNAL for HT/VHT frames (the L-SIG is always
legacy) but won't decode their DATA in fabric. ARM can chase those from DDR
if desired.

---

## D2: Stock ADI Linux, flash-every-deploy (2026-05-11)

**Decision:** Use stock ADI Pluto Linux. No runtime bitstream loading. Every
change (bitstream or firmware) requires a full flash cycle.

**Rationale:** Runtime bitstream loading causes stability issues on this
platform. Full flash is slower but deterministic and reliable. Wall time
is acceptable; unpredictable crashes are not.

**Consequence:** Build/flash/validate is a single atomic step and sessions
must budget for it (wall times: `docs/handbook.md`).

---

## D3: lib80211 as git submodule (2026-05-12)

**Decision:** lib80211 is a git submodule at `extern/lib80211/`. Fixes go upstream
first, then the submodule pin is bumped (`git submodule update --remote`).
Deimos never modifies files inside `extern/lib80211/`.

**Rationale:** Submodule gives clean provenance, pinned commits, and
trivial updates. No copy drift. lib80211 remains publishable independently.

**Consequence:** Clone requires `--recurse-submodules`. CI/build scripts
must init submodules. The submodule serves dual purpose: (a) `liblib80211.a`
linked into firmware for ARM decode/encode, (b) `vectors/` consumed by
cocotb for RTL golden-vector verification.

---

## D4: DDR ring buffer for IQ storage (2026-05-11)

**Decision:** Raw IQ samples from AD9363 are written to a DDR ring buffer by
fabric DMA. ARM and fabric both read from this buffer.

**Rationale:** This decouples sample capture from decode. In Phase 1, ARM
reads and decodes at its own pace. In Phase 2, fabric decodes in real-time
while ARM can "rewind" to DDR offsets for supplementary decode. The ring
buffer also enables the HIL test mode: ARM writes known IQ, fabric reads it.

**Consequence:** DDR bandwidth is shared between Linux, IQ buffer, and any
fabric decode reads. Must partition carefully. 128 MB for IQ gives ~3.3
seconds of capture at 20 MSPS (16-bit I + 16-bit Q). See D9 for final
layout.

---

## D5: Verification ladder — cocotb then HIL then OTA (2026-05-11)

**Decision:** RTL modules are verified in three layers: cocotb simulation
against golden vectors, hardware-in-the-loop with ARM-injected known IQ,
then OTA/cable loopback with real RF.

**Rationale:** Each layer catches different classes of bugs. Cocotb is fast
but misses timing/bus issues. HIL catches real hardware behavior. OTA
validates the full analog+digital chain. Skipping layers wastes time
debugging in the wrong domain.

**Consequence:** A module is not "done" until it passes HIL. Cocotb alone
is necessary but not sufficient. Sessions must plan for flash time.

---

## D6: No IIO/DMA framework — bare-metal DMA and static firmware tools (2026-05-11)

**Decision:** Strip out the ADI IIO DMA infrastructure and replace it with a
direct AXI DMA writing to a fixed DDR region. Firmware tools link statically
against `liblib80211.a` and reach hardware through `mmap(/dev/mem)` + sysfs.
No libiio dependency.

**Rationale:** IIO adds kernel-space complexity, buffer-management overhead,
and constrains our data flow; we need a simple ring buffer that fabric writes
and ARM reads, not a general-purpose streaming framework. Removing it also
frees non-trivial fabric resources. With IIO gone from the bitstream, libiio
buys nothing — only the AD9361 driver is needed, via sysfs attributes — and
static linking avoids glibc version drift and deployment complexity.

**Consequence:** We lose compatibility with standard IIO tools (`iio_readdev`
etc.) and ARM firmware manages the ring buffer directly. Tools are
self-contained ELF binaries deployed via SCP, with no shared-library
dependency beyond the Pluto's glibc. Real-time continuous decode is a fabric
responsibility, not a firmware one.

---

## D7: Target metric — EAPOL 4-way capture (2026-05-11)

**Decision:** Primary success metric is reliably capturing all 4 messages of
an EAPOL handshake on a target channel. Aspirational target: 80%+ success
rate, but real limits are unknown until the system is running.

**Rationale:** EAPOL capture requires: being on the right channel, detecting
frames quickly, decoding management frames at various legacy rates (typically
6 and 24 Mbps). This exercises the full system — STF detection, SIGNAL decode,
DATA decode, and ARM processing speed. It's concrete and measurable.

**Consequence:** Design choices should optimize for "don't miss frames" over
"decode everything perfectly." Better to decode SIGNAL for 100% of frames and
DATA for 80% than to attempt DATA for all and achieve 60%.

---

## D8: ARM as intelligent fallback, not just consumer (2026-05-11)

**Decision:** The ARM + lib80211 combination remains active even after fabric
decode is working. ARM uses the TX lookback buffer to chase frames the FPGA
missed or only partially decoded.

**Rationale:** Fabric handles the bulk (beacons, common management traffic)
at wire speed. But some frames will be at rates or formats fabric can't handle,
or fabric will occasionally miss frames under heavy load. ARM can use SIGNAL-only
tags (which include DDR offset) to rewind into the lookback buffer and decode
what fabric couldn't. This cooperative model — fabric sheds the common cases,
ARM handles the rest — may push capture rates higher than either could alone.

**Consequence:** The lookback buffer must be large enough to give ARM time
to catch up. 256 MB at 20 MSPS = ~1.6 seconds of history. ARM decode latency
must stay well under this to avoid losing data off the tail.

---

## D9: DDR layout — RX first, TX in separate bank group (2026-05-13)

**Decision:** Within the 160 MB reserved region (0x10000000-0x19FFFFFF):
128 MB RX ring buffer starting at 0x10000000, 32 MB TX region at 0x18000000.
TX is placed *after* RX so they land in different DDR bank groups.

**Rationale:** TX reads (HP2) and RX writes (HP0) hit the DDR controller
simultaneously during loopback and OTA capture. When both targets are in
the same DDR bank group (adjacent rows), the controller thrashes row-open/close
and latency spikes exceed the TX quad-buffer fill margin (~43 cycles), causing
drain stalls that corrupt the transmitted waveform. Separating by 128+ MB
places them in different bank groups, eliminating row conflicts. This matches
the airmon-sdr fix (TX at 0x18000000, RX at 0x10000000) which achieved 0
drain stalls with the same quad-buffer depth.

**Consequence:** The RX ring buffer now starts at the bottom of the reserved
region (0x10000000) and TX sits near the top (0x18000000). Both fit within
the existing 160 MB reserved-memory DTB allocation. No bitstream change
required — only firmware DDR_TX_BASE and DDR_RX_BASE macros.

**Amendment (2026-09-21):** TX was later expanded from 4 MB to 32 MB at the
same base (0x18000000) for HIL playback / stream mode. The 128 MB offset
between the RX and TX bases — the bank-group separation this decision is
about — is unchanged. Current values: `platform/styx/firmware/src/hal.h`
(`DDR_RX_SIZE` 0x08000000, `DDR_TX_SIZE` 0x02000000) and
`platform/styx/registers.md`.

---

## D10: Signal complexity ladder for DMA/RF validation (2026-05-13)

**Decision:** RF path validation uses a 6-level signal complexity ladder,
each with automated binary pass/fail. Levels are cumulative — failure at
level N blocks levels N+. The tool (`deimos_sigladder`) runs all levels in
<5 seconds and is included in regular validation.

**Levels:**

| Level | TX Signal         | RX Criterion                          | Proves                                |
|-------|-------------------|---------------------------------------|---------------------------------------|
| 1     | 100 kHz tone      | FFT peak at expected bin (±2)         | DDR→DAC→cable→ADC→DDR path alive      |
| 2     | 1 MHz tone        | Peak at bin (±1), >20 dB above floor  | Sample rate correct, no I/Q swap      |
| 3     | Chirp 0.5–9.5 MHz | Cross-correlation > 0.9 vs reference  | No sample drops, timing alignment     |
| 4     | Full preamble     | lib80211_sync_detect returns success  | STF detection works through hardware  |
| 5     | Full preamble     | rx_decode reaches SIGNAL field        | CFO/timing/channel estimation works   |
| 6     | 6 Mbps beacon     | Full decode, FCS OK, payload match    | End-to-end validated                  |

**Rationale:** When integrating custom DMA with an existing validated PHY
(lib80211), the failure domain is not the PHY itself but the signal path.
A graduated test isolates *where* the path breaks: L1-2 find basic
connectivity/configuration issues, L3 finds sample timing problems, L4-6
find subtle signal integrity issues. This avoids jumping straight to "it
doesn't decode" with no diagnostic narrowing.

**Consequence:** Any session that modifies DMA, ADC/DAC routing, clock
domains, or DDR layout must run `deimos_sigladder` after flash and report
the highest passing level. A regression (lower level than previous build)
blocks further work until diagnosed.

---

## D11: Rate 54 length limit — accepted hardware EVM (2026-05-14)

**Decision:** Rate 54 (64-QAM 3/4) fails at payload ≥180 bytes in cable
loopback. This is an accepted limitation of the analog path, not a decoder
bug.

**Evidence:**
- lib80211 software loopback decodes rate 54 at all sizes (80-800 bytes, FCS OK)
- Hardware loopback: ≤160 bytes passes ~99%, ≥180 bytes passes 0%
- Rate 48 (64-QAM 2/3, same modulation, weaker coding) passes at all sizes
- The failure cliff is independent of symbol count — it correlates with
  effective fill factor (less padding = less coding margin)
- No DMA overflow, no sample drops — analog EVM is the bottleneck

**Rationale:** The AD9361 DAC→cable→ADC path adds ~1-2 dB of systematic
EVM (likely from DAC quantization, PLL phase noise, or internal SFO between
TX/RX clock paths). Rate 48's extra coding gain (2/3 vs 3/4) absorbs this;
rate 54 cannot when the frame is densely packed. This matches lib80211 D3.

**Consequence:** Cable loopback tests use 80-byte payload (default) where
all 8 rates pass reliably. OTA management frames (beacons, EAPOL) are
transmitted at rates 6-24, well within the reliable range. Rate 54 long
frames are a "nice to have" optimization target, not a blocker.

---

## D12: FFT architecture — R2²SDF streaming pipeline (2026-05-17)

**Decision:** 64-point FFT uses R2²SDF (Radix-2² Single-path Delay Feedback)
streaming pipeline. Vertical decomposition: fft4_sdf → fft16_sdf → fft64_sdf.
Each sub-FFT independently tested. Combinational BF pairs with 4-stage
pipelined twiddle multipliers between levels.

**Implementation invariants:**
- 3 stage pairs (BF1+BF2), 2 pipelined twiddle multipliers, 4-stage pipelined
  multiply (input reg → DSP products → add/sub → round).
- Input 16-bit signed, natural order, 1 sample/clock; output 16-bit signed,
  bit-reversed from `fft64_sdf`, restored to natural order by `bit_rev6` in
  `decode_engine.v`.
- ~73-clock latency from first input to first output, then 64 consecutive
  outputs; output gating uses FILL_SKIP = 63 to skip pipeline fill.
- FFT feed is buffer-then-burst in `decode_engine.v`: it holds `fft_rst_n`
  low per transform, streams 64 samples from the feed mixer, then zero-fill
  drains to keep the FFT fed without gaps. (`fft_wrap.v` was deleted
  2026-06-22; the FFT is now the external `fft64_sdf` BD entity, and the
  wrapper's capture/reorder role moved into `decode_engine.v`.)
- Twiddle ROM is Q1.15, strides `[2, 1, 3, 0]`; one `gcnt` per module
  (fft4/fft16/fft64), each wrapping at its own N, addresses it directly.
- BF pair control: `sel = gcnt[MSB]`, `rot = ~gcnt[MSB] & gcnt[MSB-1]`.

**Rationale — why R2²SDF, not in-place iterative.** Three in-place approaches
were each tested and each fails structurally:

| Approach | Result | Why it fails |
|----------|--------|--------------|
| Single-port BRAM | 1920 cycles | Serialized reads blow the cycle budget |
| TDP BRAM, 2-cycle pipe | Garbage output | NB timing: DSP fires on the same edge as the BRAM read |
| Distributed RAM, 2-cycle pipe | 10,045 LUTs | Combinational reads generate MUX trees |

R2²SDF removes the memory problem: no random-access memory, only
shift-register delay lines (circular buffers) that Vivado maps efficiently to
distributed RAM (read-at-write-pointer, not arbitrary-address MUX trees).

**Rationale — why vertical decomposition, not a global counter.** A
single-`gcnt`-with-delayed-taps architecture fails at the second twiddle
multiplier: SDF feedback reordering makes the correct twiddle address
unrecoverable from a delayed global counter.

**Consequence / do not regress:**
- Do not attempt in-place iterative or a global-counter-with-delayed-taps
  design — both were tested and fail as above.
- Do not change the twiddle ROM contents (strides `[2, 1, 3, 0]` verified) or
  the BF pair control (verified bit-exact).
- If LUTs are needed later, promote the depth-32 and depth-16 delay lines to
  BRAM (`(* ram_style = "block" *)`) rather than re-opening the architecture.

---

## D13: CFO estimator requires full 16-iteration CORDIC (2026-06-09)

**Decision:** The coarse CFO estimator (`cfo_est.v`) must use the full
16-iteration, 32-bit `cordic_atan2` module. The compact 8-iteration
`cordic_atan2_sm` is not acceptable.

**Evidence:**
- Compact CORDIC saves 773 LUTs (15,760 → 14,987) — significant.
- CFO estimation error increases from ±2 to ±6 LSBs of phase_inc.
- 6 LSBs = ~1.8 kHz residual CFO after correction.
- Rates 6/9/12 (BPSK/QPSK): cable loopback 20/20. Pilot PLL absorbs residual.
- Rate 24 (16-QAM): cable loopback **0/20**. Pilot PLL cannot absorb residual
  at the tighter 16-QAM constellation spacing.
- HIL (no real CFO) passes at all rates with either CORDIC — confirms the
  failure is specifically about real-world CFO correction precision.
- Reverting to full CORDIC immediately restored rate 24 to 20/20.

**Rationale:** The pilot PLL corrects per-symbol residual phase, but it
tracks slowly (single-pole, constrained bandwidth to avoid noise amplification).
A 1.8 kHz residual CFO accumulates ~0.5° per symbol, which is within the noise
margin for BPSK/QPSK (90° between decision boundaries) but NOT for 16-QAM
(~26° between nearest neighbors). The coarse CFO estimate must be accurate
enough that the pilot PLL only handles slow drift and noise, not a systematic
frequency offset.

**Hard constraint:** Do not reduce cfo_est CORDIC precision below 16 iterations
or truncate its 32-bit accumulator inputs. If LUT savings are needed from CFO,
explore reducing the accumulation window (WINDOW parameter) or the delay line
depth instead — these trade detection range for resources, not accuracy.

---

## D14: AD9363 analog filter bandwidth = 28 MHz (2026-06-11)

**Decision:** Set AD9363 RX and TX analog bandwidth (`rf_bandwidth`) to 28 MHz
for all firmware tools, up from the previous 20 MHz. Sample rate remains 20 MSPS.

**Evidence:**
- Per-subcarrier EVM analysis (`diag_evm_decomposition`) showed a symmetric
  U-shaped EVM profile: edge subcarriers at -16 dB vs center at -19.8 dB
  (3.8 dB degradation).
- Symmetry rules out SFO (which would produce asymmetric slope). Quadratic
  fit (R²=0.60) vs linear fit (R²=0.00) confirms U-shape = filter roll-off.
- With `BANDWIDTH_HZ=20000000`: edge subcarriers at ±9.4 MHz sit at the
  3rd-order Butterworth 3dB point. Signal is attenuated 2-3 dB at band edges.
- Rate 36 (16-QAM, code rate 3/4) needs mean EVM < -19 dB. The edge
  degradation pulls the mean from -19.8 (passable) to -18.4 dB (fail).
- Changing to 28 MHz: rate 36 cable loopback went from 0/20 to 50/50.
  No regression at rates 6-24. Rates 48/54 improved (0% → 40-60%).

**Rationale:** The AD9363 `rf_bandwidth` register sets the 3dB corner of the
analog anti-aliasing/reconstruction filters (RX BB LPF: 3rd-order Butterworth,
per AD9361 Reference Manual UG-570). Setting this equal to the signal bandwidth
places the outermost OFDM data subcarriers (at ±26 × 312.5 kHz = ±8.125 MHz)
within the filter's transition band. Standard SDR practice is 1.2-1.5× the
signal bandwidth to ensure flat passband response across all active subcarriers.

28 MHz = 1.4× of 20 MHz — a conservative choice that:
- Ensures <0.5 dB attenuation at the outermost subcarriers
- Admits only 1.5 dB additional thermal noise (10·log₁₀(28/20))
- Does not alias in-band: with 20 MSPS, Nyquist is ±10 MHz; no transmitted
  energy exists above ±10 MHz in cable loopback or single-channel OTA
- Is consistent with the Analog Devices ADALM-PLUTO default configuration
  for OFDM applications (their gnuradio examples use 1.4× overspreading)

**Consequence:** All firmware tools now use 28 MHz as the default bandwidth.
`deimos_fabric_loopback` accepts `-b <MHz>` for experimentation. For OTA on
congested bands where adjacent-channel rejection matters, consider using a
digital FIR filter (AD9363 programmable FIR, currently bypassed) to provide
sharper out-of-band rejection without compromising in-band flatness.

---

## D15: Skip len=23 rate-6 frames unconditionally in classifier (2026-06-19)

**Decision:** The frame classifier in `deimos_rx` unconditionally
classifies rate=6, length=23 frames as `ht_vht` (never parsed),
regardless of fabric FCS result.

**Evidence:**
- These frames pass fabric FCS (100%) but ARM `lib80211_rx_decode_at_offset`
  cannot decode them (HT detection triggers, HT decode fails, legacy fallback
  fails).
- Length 23 is not a valid 802.11 frame: ACK/CTS=14, RTS/CF-End=20, Compressed
  BAR=24; the shortest valid frame is 28 bytes (24-byte header + 4-byte FCS).
- Length 23 is not a valid HT/VHT L-SIG length either (`(L+3) % 3 == 0` is
  required; `23 % 3 = 2`).
- Root cause: HT preamble bits (HT-SIG + HT-STF) produce a deterministic
  Viterbi output when the legacy decoder processes them as rate-1/2 data, and
  the CRC-32 of that output happens to match for the AP's fixed HT-SIG.

**Rationale:** The FCS pass is a coincidental CRC collision on structured
(non-random) input, not a legitimate frame. ARM decode wastes CPU and always
fails, and no valid frame can be 23 bytes, so skipping unconditionally has
zero impact on EAPOL capture or real legacy traffic.

**Consequence:** `deimos_rx_classify_frame()` in `deimos_rx.c` handles this
before any decode attempt.

---

## D16: Rates 48/54 cable loopback — accept analog EVM limit (2026-06-28)

**Decision:** Rates 48 and 54 via cable loopback are report-only (never gate)
in all test scripts. No further digital-side optimization will be attempted.

**Evidence:**
- EVM characterization (sim replay of hardware ADC captures): rate 48 mean
  -19.8 dB, rate 54 mean -17.6 dB post-EQ; clean 64-QAM needs < -25 dB.
- Cable loopback: ~85-95% (48) and ~90% (54) after the double-CFO-correction
  fix; the residual 5-15% is the true analog floor.
- HIL (digital injection): 100% for both rates — the fabric logic is correct.
- PM_WIDTH changes hurt: 12 + ÷4 regressed both rates and was reverted;
  PM_WIDTH=10 fails `test_error_correction`.

**Rationale:** The analog EVM floor (-18 to -22 dB) is set by AD9363 DAC
quantization noise and PLL phase noise through the cable path. 64-QAM needs
< -25 dB; the gap is ~2-3 dB and cannot be closed digitally. (The original
measurements read 4-6 dB because a double CFO-correction bug added ~40° of
pilot drift that only manifests with real crystal CFO, not in zero-CFO HIL.)

**Consequence:** Rates 48/54 should work better OTA, where real transmitters
have better EVM than the DAC→cable→ADC path. Test infrastructure treats them
as informational; cable loopback gates on rates 6-36 only.

---

## D17: Rate 54 + CFO — accept as analog-compounded limit (2026-06-28)

**Decision:** Rate 54 with ±5 kHz CFO is report-only in hil_test.sh layer 3.
No fix will be attempted.

**Rationale:** Rate 54 (64-QAM, code rate 3/4) has the tightest constellation
spacing and least coding redundancy of all rates. Even with the double-correction
bug fixed, rate 54 + ±5kHz CFO fails 100% in HIL (confirmed 2026-06-29). The
residual phase error from CFO quantization (cfo_est estimates
in integer phase_inc units = ±305 Hz granularity) exceeds what pilot_track can
absorb within 64-QAM 3/4's tight EVM budget. Rate 48 (code rate 2/3) passes at
±5 kHz because the extra redundancy absorbs the additional impairment.

**Consequence:** hil_test.sh layer 3 reports rate 54 + CFO results but never
gates on them. OTA performance at rate 54 depends on channel conditions.

---

## D18: Cable loopback — fixed manual RX gain 24 dB + level telemetry (2026-09-02)

**Decision:** Cable loopback tools (`deimos_fabric_loopback`,
`deimos_adc_capture`, `deimos_burst_loopback`, `pluto_loopback`,
`pluto_burst_loopback`) use fixed manual RX gain 24 dB. No AGC for
loopback. `session_start.sh` logs ADC level telemetry (peak/RMS/clipped
samples) as a `level` field in `logs/hardware.jsonl` and warns when
peak < 1000 (low signal) or peak ≥ 2040 (clipping).

**Rationale:** The AD9361 4-6 GHz gain table has discrete LNA transitions.
At gain 22 the LNA is in its lower-gain mode (`0x04`) with the mixer at
maximum compensation (`0x2D`) — a mixer-heavy point with a worse noise figure
than the LNA-heavy configuration at 23+. The 22→23 transition is the largest
single-step noise-figure change in the band, so 22 is a marginal operating
point for 64-QAM (48/54M). Gain 24 sits one step above the transition,
mid-range in the `0x24` band, at a peak of ~1280 — comfortable margin above
the 1000 low-signal floor and well below the 2048 clipping ceiling. Gain 25
also works but is closer to the next transition at 26; 23 is on the boundary.
(Gain 30, the original default, sat on the ADC full-scale cliff and clipped.)

| Gain | LNA | Mixer | Notes |
|------|-----|-------|-------|
| 22 | 0x04 | 0x2D | **Ceiling of 0x04 LNA range** — mixer at max |
| 23 | 0x24 | 0x20 | **LNA step up** — mixer resets low |
| 24 | 0x24 | 0x21 | |
| 25 | 0x24 | 0x22 | |

**Evidence** (measured 2026-09-02; ADC peak at ch149/TX atten 3.0 dB, and 48M
fabric loopback, 20 trials × 3 runs):

| Gain | Peak I/Q | Clipped | 48M loopback |
|------|----------|---------|--------------|
| 22 | 1108/1141 | 0 | 19/20, 20/20, 18/20 — dips below the 90% gate |
| 24 | 1282/1282 | 0 | 20/20, 19/20, 20/20 — stable above gate |
| 30 | 2048 (sat) | 14 | — |

At g=24 the full-rate sweep is 8/8 rates 20/20; at g=22 the 6-36M rates are
solid but 48M fluctuates 80-100% and 54M 90-100%.

**Consequence:**
- Gate runs at g=24. `-A` on `deimos_fabric_loopback` is kept for AGC
  experiments; `-L` on `deimos_adc_capture` emits level JSON and exits 0
  (telemetry, not a test).
- Avoid gain values at the ceiling of an LNA range (15, 22, 40).

---

## D19: Block design is the single source of truth; sim view is generated (2026-09-16)

**Decision:** The hardware netlist description (`fpga/project/system_bd.tcl`)
is the *only* authored description of the receiver pipeline wiring.
`fpga/rtl/rx_pipeline.v` and `fpga/rtl/rx_frontend.v` are **generated** from
it by `scripts/gen_sim_pipeline.py`. The generator parses the BD
(`create_bd_cell`, `ad_connect`, `connect_bd_net`) for the 147
module-to-module edges, reads port directions/widths from Verilator XML, and
emits the sim wrappers. The only hand-authored sim delta lives in
`scripts/pipeline_sim_view.py`: wrapper port lists, the mapping of BD
boundary endpoints to ports/literals, the net names tests read, and the
sim-only assigns/assertions. `sim.sh` Layer 0 runs
`gen_sim_pipeline.py --check` (regenerate + diff) before any test.

**Supersedes:** the previous D19 (*BD/RTL parity gate*, 2026-08-24), which
kept two hand-maintained netlists in sync by diffing them and could only
detect drift after the fact. The generated files carry a `DO NOT EDIT`
banner; the freshness check makes a hand edit a hard failure before tests run.

**Rationale:** Two hand-maintained descriptions of one netlist always drift.
The obvious fix — collapse the 22 BD cells into one pipeline module the BD
instantiates once — is rejected: each BD module cell gets its own
out-of-context synthesis run, and the 11 compute-heavy modules are
area-optimized (`fpga/Makefile:OOC_MODULES`), which is load-bearing for the
near-limit LUT fit (D21). Inverting the source of truth removes the second
description *without touching the hardware netlist, its OOC boundary, or the
bitstream* — the sim view simply cannot be out of sync with the design.

**Consequence / do not regress:**
- Do not hand-edit `fpga/rtl/rx_pipeline.v` or `fpga/rtl/rx_frontend.v`.
  Change `system_bd.tcl` (hardware) and regenerate; a hand edit fails Layer 0.
- Simulation-only intent goes in `scripts/pipeline_sim_view.py`:
  `BOUNDARY` (BD endpoint -> port/literal/net/unconn), `SIM_EDGES` (connections
  with no BD edge), `NET_NAMES` (names tests read), `EXTRA_DECL`/`EXTRA_BODY`.
- The two former allowlist entries are now explicit literals with reasons in
  `BOUNDARY`: `decode_engine/snap_mode -> 3'd0` (register file not modeled) and
  `stf_detect/enable -> 1'b1` (always enabled in test).
- Sim-only divergences with no BD edge are listed in `SIM_EDGES` with a
  reason (`chan_est/clip_cnt`, `psdu_packer/byte_count`, decode_engine status
  ports). Adding one requires a reason.
- A partial revert of `system_bd.tcl` that drops `ad_connect` lines now fails
  the freshness check until the sim view is regenerated — the 2026-08-23
  failure class (sim passes, hardware silently drops frames) is closed at the
  source instead of being policed by a diff.

---

## D20: Build identity — fingerprint register + bitstream sha256 (2026-08-24)

**Decision:** Two identities travel with every build. A source **fingerprint**
(git short hash) is embedded in a readable AXI register (`BUILD_ID`) at
synthesis time and read by firmware on boot and at session start. A result
identity — sha256 of `system_top.bit` — is written to
`build/fpga/bitstream.sha256` beside `build/fpga/build_info.json` (WNS/WHS,
LUT/FF/BRAM/DSP, place strategy, incremental state, host, timestamp); both are
written beside `fingerprint` in the local `build/fpga/` evidence tree, which is
private-dev-repo and never committed (AGENTS.md, "Build Artifacts").
`flash.sh` stamps the hash into U-Boot env
(`fw_setenv deimos_bitsha`); `validate.sh` and `session_start.sh` compare the
device value to the build sidecar. Every `logs/hardware.jsonl` entry carries a
`bitstream` field beside `fingerprint`.

**Rationale:** The fingerprint answers "which source tree is deployed?" and
catches stale flash, partial updates, and version mismatch without trusting
files on the device. It cannot answer "which netlist is running?": two
placements of one tree produce different netlists with the same fingerprint
(2026-08-23: fingerprint `8b5fd32d` built twice, WNS 0.204 vs 0.074). The
`BUILD_ID` register can only carry the source fingerprint, so the device-side
sha256 stamp closes the loop. A/B evidence is thus result-level: equal
fingerprint **and** equal sha256 means the same artifact (pre-identity log
entries read `"bitstream":"unknown"`).

**Consequence / do not regress:**
- Compute the fingerprint **before** project creation
  (`fpga/tcl/build.tcl:41-54`). A fingerprint computed afterward is invariant
  to deimos RTL, and once was: `system_bd.tcl` assigns `rtl_dir` at global
  scope and clobbered the caller's value, so Phase 2 hashed the styx tree
  twice — a 22-file manifest instead of 48 — and two builds with different
  `viterbi_k7.v` both recorded `0x93ca3e19`. The fix computes the value into
  uniquely named locals (`deimos_fp_styx_rtl_dir`, `deimos_fp_rtl_dir`) that
  no global assignment can reach; Phase 2 only applies it.
- BAT (build acceptance test) checks the fingerprint register first.
- `build_info.json`'s `host` field defaults to `unspecified` and is populated
  only when `DEIMOS_BUILD_HOST` is set — build evidence may be shared outside
  the repo, so it must not carry a real hostname.

---

## D21: Resource headroom and per-netlist fit tuning (2026-08-26)

**Decision:** The design is kept near the device's placement limit on purpose
— LUTs buy function, not idle headroom — but the two fit-tuning knobs are
treated as *per-netlist* settings, not settled values: `OPT_DIRECTIVE`
(`opt_design`; default `ExploreArea`, the load-bearing half) and
`PLACE_STRATEGY` (default `ExtraPostPlacementOpt`). Both are overridable from
`config.mk` or the command line and recorded in `build_info.json`, so a
build's recipe is recoverable. `styx::implement` exposes
`-opt_directive` / `-phys_opt` / `-route` parameters so this deimos-specific
fit requirement does not live hardcoded in the shared platform repo.

**Rationale:** A congested design (~96% pre-opt utilization) is sensitive to
tiny netlist perturbations — including the build-id constant itself, so even
comment-only edits that move the fingerprint can flip timing. Vivado is also
non-deterministic here: a fresh OOC synthesis can add ~1000 LUTs over an
incremental build of identical source. Cheap-to-sweep knobs are what keep
that recoverable.

**Rejected:**
- `control_set_opt` — Versal-only, useless on 7-series.
- `-no_lc` — counterproductive (synth LUT combining beats physopt combining).
- Shrinking the OOC module list — BD modules get their own synthesis runs
  regardless of the `styx::ooc_synth` list.

**Consequence / do not regress:**
- Keep `OPT_DIRECTIVE=ExploreArea` unless a sweep replaces it; that directive
  is the ~160-LUT half of the fit.
- Any RTL growth moves the design closer to placement failure: budget ~200
  LUTs of headroom and re-sweep the directives if timing breaks.
- If a fit fix would require editing a hardcoded constant in a shared styx
  proc, parameterize it instead.
- Area-driven reductions must carry their *validation context* forward: D24's
  correlator cut was justified on a wide search but silently broke the narrow
  acquisition window — see D29.

---

## D22: Acquisition and decode are separate engines (2026-07-01)

**Decision:** Frame acquisition and frame decode are independent modules
connected by a FIFO, not one FSM. `acquisition_ctrl.v` owns STF/LTF peak
search and CFO capture, and pushes a descriptor `{ltf_pos, phase_inc}` into
`frame_fifo.v` (DEPTH=4). `decode_engine.v` pops descriptors and decodes
through FCS/tag. There is no "pending decode" path and no replay path:
acquisition is never blocked by decode state.

Acquisition signals `pipeline_ack` to `stf_detect` as soon as a descriptor is
pushed (or the frame is rejected), so `stf_detect` rearms within ~40 samples
of `stf_end` rather than thousands of samples later when decode finishes.

**Rationale:** A single FSM made "acquire frame N+1 while decoding frame N" a
special case that had to be handled correctly under timing pressure — the
SIFS burst-drop failure (HIL at gap=320 decoded 1-3/10 frames while the same
RTL passed sim 10/10) came from that handoff. Splitting the engines makes the
SIFS case the *only* case: it becomes structurally impossible rather than
conditionally handled. That is why the bug closed without a signal-level root
cause — the machinery that produced it no longer exists.

**Rejected — hypotheses tested and eliminated before the refactor:**
`iq_valid` delivery gaps, correlator threshold flicker, BRAM read-during-write,
accumulator underflow, DSP truncation, STF holdoff, and rearm/latch behavior.
None localized the fault. The refactor removed the three remaining candidates at
once: the pending path's separate windowed-max search (window constants
sensitive to the correlator's 10-clock `cfo_mixer` delay on hardware), the
`playback_start` trigger that produced a garbage decode before the first real
STF, and the trigger-rejection interaction when frame N+1 arrived during frame
N's `S_TAG_OUT`/`S_DONE`.

**Evidence:** post-refactor, rate 6 and rate 24 at gap=320 are both 10/10 FCS
OK, the gap sweep 320/5000/20000 is 10/10, and a mixed-rate burst is 15/15.

**Consequence / do not regress:**
- Do not reintroduce a shared FSM, a pending path, or a replay path.
- Do not feed a synthetic start pulse (e.g. `playback_start`) into the frame
  trigger; acquisition must trigger on real STF detection only.
- `frame_fifo` full must back-pressure acquisition, not stall decode.
- Do not add a per-tag DDR offset queue. PSDU bytes live in fabric BRAM and
  `TAG_HI` carries `psdu_addr[13:0]`; the `offset_queue` in `rx_pipeline.v` is
  sim-wrapper-only (`rx_pipeline` is not in the block design).
- The tag FIFO is 16 entries (`tag_fifo_axi.v:72-73`); ARM must drain it.

---

## D23: Throughput budget — 400 clocks/symbol, budget-derived targets, clean drops (2026-07-05)

**Decision:** The real-time per-symbol budget is **400 fabric clocks**
(80 samples/symbol × 5 clocks/sample at 100 MHz fabric / 20 MSPS ADC). A
module's target is derived from that budget and the *current binding stage*
(`budget / work_per_symbol`), not from the module's best case — buy area for
a module only when it moves the binding stage. Rates over budget are allowed
to run, but saturation must produce **clean drops or `tag_abort`, never a
corrupt tag**: ARM must never see wrong data because the pipeline fell
behind. Current per-rate figures and the binding stage live in
`docs/throughput-analysis.md`.

**Rationale:** A clean drop is observable and safe; a silent overwrite is
neither. Buying buffer depth raises the frame count before saturation but
cannot remove the overrun — only reducing clk/sym can. Separating "correct or
absent" from "fast enough" lets the correctness property be fixed immediately
and the throughput work be scheduled independently. Headroom the pipeline
cannot use is worthless — an oversized module can fail placement for no gain
(D21).

**Clean-drop mechanism (in `decode_engine.v`):**
- `circ_buf` is 32,768 entries with 16-bit pointers (`[14:0]` index, MSB for
  wrap detection).
- `frame_overwritten` — if `data_age >= BUFFER_SIZE` at FIFO pop, the queued
  frame's IQ is already gone; abort instead of decoding garbage.
- `near_overwrite` / `write_ok` — a `GUARD_MARGIN` (4096) write guard blocks
  new IQ writes before they reach queued frame data; blocked samples drop and
  acquisition keeps running on live ADC.

**Rejected — concurrent-search Viterbi (Tasks 4/5).** Overlapping the
best-state search with traceback (Task 4) was a ~790-LUT delta that failed
placement, with almost none of it in the shared ACS; Task 5 was its timing
fallback. Against a budget-derived target (rate 54 allows ~1.85 clk/pair) the
serialized search at ~1.278 clk/pair already clears with ~30% margin.

**Consequence / do not regress:**
- Throughput status: rates 6-36 run real-time; 48/54 remain over budget
  (~36 clocks/sym) and are deferred (`docs/throughput-analysis.md`).
- Any new per-symbol latency must be measured against the 400-clock budget
  before commit (`test_latency_ratchet.py`).
- Do not remove the `frame_overwritten` abort or the `near_overwrite` write
  guard to reclaim LUTs — they are the clean-drop guarantee. Sustained SIFS
  bursts at an over-budget rate will drop frames; that is accepted behavior,
  not a bug to hide by suppressing the drop path.
- **Viterbi:** ships Task 3-only (traceback overlaps ACS; the 13-clock
  best-state search stays serialized in `S_FIND_BEST`, ~1.278 clk/pair). Do
  not implement Tasks 4 or 5. Do not add a `CONTINUOUS` flag; `MEM_DEPTH` is
  128 with a bare-decrement `circ_dec` (span `2*48+13 = 109`).
- **PM normalization:** no per-step subtract; correctness requires the PM max
  spread `< PM_HALF = 1024` (measured ~535 on clean input — a 2× margin, not
  10×).
- **Streaming-mode end-of-frame flush** must trace back from the best state,
  not state 0. 802.11 pad bits drive the encoder to a data-dependent end
  state; tracing from 0 corrupts the last ~15 bits of the final partial
  window (intermittent HIL layer-6b burst FCS failures).

---

## D24: LTF correlator — 4 engines / 16 taps (2026-06-23)

**Decision:** The sliding LTF cross-correlator uses **4 parallel complex MAC
engines over 16 taps**, producing one metric per ADC sample (4 taps/clock ×
4 stages + 1 metric clock = 5 clocks/sample, matching the 100 MHz / 20 MSPS
ratio). Reduced from 6 engines / 24 taps. Measured cost: 705 LUTs and 16
DSP48E1 (`build/fpga/utilization.rpt`). The T1 offset constant is
`T1_OFFSET = 19` (`acquisition_ctrl.v:87`).

**Why fewer taps are safe:** the correlator is not a detector — `stf_detect`
handles detection. The correlator is a *ruler*: it locates the precise T1
position inside a small search window after a frame is already confirmed to
exist. Inside a windowed-max search you do not need high SNR; you only need
the correct peak to be the maximum in the window.

**Evidence:** on a wide (80-sample) search over OTA and cable captures,
16 taps picks the *identical* peak sample as 24 taps. 12 taps does not — it
locks onto a secondary peak with a consistent −10 sample error, which is why
the reduction stopped at 16 rather than continuing down. Residual −4 to −12
narrow-window bias at 16 taps is correlation *peak shape* bias, not a
detection failure, and the `T1_OFFSET` constant compensates for it.

Timing margin: the 802.11 cyclic prefix is 16 samples (800 ns). A ±4 sample
error consumes 25% of CP, leaving 600 ns of guard — roughly 6-8× typical
indoor 5 GHz RMS delay spread (50-100 ns). Pilot tracking absorbs the
residual common phase.

**Rationale:** the correlator was the largest single reclaimable block during
the placement shortfall that also produced D21. Cutting it freed ~600-800
LUTs, ~10 control sets, and 4 DSP48E1 with no measurable change in peak
selection.

**Rejected — time-division:** one MAC at 1 tap/clock yields only 5 taps per
sample given the 5-clock budget. Covering 24 taps serially would emit a
metric every 5th sample, dropping peak resolution from 1 sample to 5 samples
(250 ns) — strictly worse than the ±4 sample bias of simply using fewer taps.
GNSS receivers can time-divide because their codes are 1023+ chips with
coherent accumulation over a long period; the 802.11 LTF is 64 samples, so
there is no long integration window to exploit.

**Consequence / do not regress:**
- Do not reduce below 16 taps without re-running the wide-window peak
  comparison. 12 taps is known bad (secondary peak lock).
- `T1_OFFSET` is tied to the tap count. Changing taps requires changing it.
- Supporting measurement data is in `docs/correlator-reduction-analysis.md`
  (cited from `ltf_correlator.v:9`).
- This is one half of D29: T1 correctness is joint across (taps, window,
  `T1_OFFSET`). The wide-window equivalence above does **not** transfer to a
  narrow window; re-validate jointly (sweep bench + OTA) on any change.

---

## D25: Depuncturer pattern state is per-frame, not persistent (2026-09-15)

**Decision:** The depuncturer's puncture-pattern position is reset once
per frame, at SIGNAL setup, via a dedicated `decode_engine.depunct_restart`
output. It is no longer tied to GND.

**Rule:** Cross-frame shared sequential state must be reset explicitly at
the frame boundary — the setup/transition that begins the new frame — and
must not share a reset with per-symbol state. Never rely on downstream drain
timing to clear it.

**Rationale:** The depuncturer emits one bit per clock, so at 54 Mbps it needs
432 clocks to drain a symbol the deinterleaver delivers in 144; its elastic
FIFO still holds a tail of the current frame's coded bits when the next frame
is popped, and that tail is residue past what FCS consumed. `decode_engine`
sets `code_rate_out` back to 0 on frame pop, so with persistent pattern state
the residue drained under rate-1/2 passthrough, which never advances `pat_pos`
— the position froze mid-group and every later frame in the burst decoded
against the wrong puncture positions. The old design only worked because the
traceback stall happened to drain the backlog before end-of-frame; the
invariant was never enforced.

This is a latent fragility, not a shipping regression: the stall masked it, and
removing the stall breaks rates in turn.

| Traceback stall | 36M six-frame burst | 54M six-frame burst |
|---|---|---|
| present (shipping) | 6/6 | 6/6 |
| partially removed | FAIL (phase-slip, frame 3) | 6/6 |
| removed | FAIL | FAIL (1/6) |

Single-variable A/B on one waveform at the `code_rate 2 → 0` transition:

| | backlog | `pat_pos` after | frame 2 FCS |
|---|---|---|---|
| stall present | 0 bits | 0 | OK |
| stall removed | 114 bits | 2 | FAIL |

Independent arithmetic agreed: 1728 of 1728 kept bits (nothing lost), 806
erasures against the 864 required — short by exactly the groups a `pat_pos` of
2 implies. The fix is required, not optional: any future change to chain timing
would re-break it.

**Rejected — stop resetting `code_rate_out` on frame pop.** It regressed frame
*detection* to 1 tag per burst: the SIGNAL field is always rate 1/2 and shares
this depuncturer, so holding the rate at 3/4 corrupts the next frame's SIGNAL.
One register cannot serve both the old frame's tail and the new frame's SIGNAL
— which is why the reset had to be a pattern restart, not a rate change.

**Rejected — a dedicated `frame_start` port.** Functionally equivalent, but
`symbol_start` already has exactly the needed semantics (discard state,
restart pattern); a second reset port would leave two inputs doing one job.

**Consequence / do not regress:**
- `symbol_start` must NOT be pulsed per symbol — a per-symbol reset flushes
  the elastic FIFO mid-stream and corrupts bit order. The name is historical;
  it means "discard and restart", and only a per-frame pulse is safe
  (`depuncturer.v:23`).
- The restart must fire before any of the new frame's bits reach the module;
  SIGNAL setup satisfies this, DATA config does not.
- Coverage: `test_hil_burst_replay` single-rate bursts at 36/48/54M. Mixed-rate
  scenarios are all code rate 1/2, where the depuncturer is a passthrough with
  no pattern position to strand, so they cannot catch this.
- A parity-based `PUNCTURE PHASE SLIP` assertion cannot detect this class (an
  even offset preserves G0/G1 alignment); the companion
  `PUNCTURE PATTERN CARRYOVER` assertion checks the position itself.

---

## D26: Streaming demapper — wide-word LLR transport (2026-09-16)

**Decision:** The demapper is a pure 3-stage streaming pipeline: no symbol
buffer, no capture/prime/emit FSM. Each `valid_in` subcarrier flows
input-register → LLR arithmetic → wide output at an exact 3-clock latency
(`wide_valid` = `valid_in` delayed 3), presenting **all `n_bpsc` LLRs of one
subcarrier** (lanes 0..5) per clock. The deinterleaver captures one wide word
per clock into a **`48×48b`** buffer and gathers 2 LLRs/clk through a
permutation ROM whose entry encodes the arrival position as
`(subcarrier << 3) | lane` (`scripts/gen_deint_enc.py`).

**Rationale:** The per-symbol tail had two serialized emit phases — demapper
`48 × step_bpsc` then deinterleaver `N_CBPS/2` — because deint capture could not
begin until the demapper's last output. All LLR candidates were already
computed in parallel, so the emit phase was pure serialization. Widening the
interface removes it: `demap` drops to 48 clocks at every rate and the two
stages pace-match 1:1.

**Design invariants:**
- Capture is always 48 words for every rate (`N_CBPS = 48 × n_bpsc`) — this is
  what makes demapper and deinterleaver pace-match.
- The ROM entry fits the existing 9-bit width: 6-bit subcarrier (0-47) + 3-bit
  lane (0-5). BPSK/QPSK/16-QAM reduce to shifts; 64-QAM (`n_bpsc=6`) does not,
  which is why the lane is precomputed rather than decoded arithmetically.
- Lanes `≥ n_bpsc` must be **defined** (never X) — an X lane once propagated
  into the shared soft path and regressed `test_adc_replay`.

**Consequence / do not regress:**
- Do not reintroduce the demapper serial emit, `sym_buf`, or the deinterleaver
  2-bit capture; the wide interface is the transport.
- The port widening must land in `system_bd.tcl`; the sim view
  (`rx_pipeline.v`) is generated from it (D19).
- `test_latency_ratchet` gates on the **feed-independent compute window**
  (`frontend_helpers.measure_compute_window`), not the `symbol_start` period:
  once the pipeline is faster than the live feed the period clamps near 400
  and hides further improvement.

**As-built record:** `docs/throughput-analysis.md` §C (lever C).

---

## D27: OTA uses AGC fast-attack; cable loopback uses manual gain (2026-09-19)

**Decision:** Over-the-air decode (`deimos_rx_dump`, and shesha's WiFi
producer) configures the AD9361 RX gain in `fast_attack` AGC mode, not manual
gain. OTA sets the gain *mode*; a fixed gain value is only for cable paths.

**Rationale:** OTA reception sees multiple transmitters at different distances
arriving within short intervals, so any fixed manual gain is wrong for part of
the traffic. `fast_attack` tracks per-frame. Measured A/B (2026-07-07, ch36):
manual 50 dB vs fast_attack — ~38 → ~127 frames/s, FCS-OK 44–49% → 53–64%,
EAPOL toggle 31/32 (96.9%). Manual gain also risks the AD9361 `slow_attack`
default silently rejecting sysfs gain writes (see
`docs/ad9361-gain-mode-gotcha.md`), which settles on noise and destroys decode.

**Consequence / do not regress:**
- OTA is AGC-only. `deimos_rx_dump` has no gain option (2026-09-19: the inert
  `-g` flag was removed rather than left as a no-op in AGC mode).
- Cable loopback tools (D16) keep manual 24 dB — fixed path, no near-far.
- Gain readback returns -999 under AGC (cosmetic).

**Rejected — manual fixed gain for OTA:** 3.3× lower frame rate and 44–49%
FCS-OK in the A/B above. **Rejected — `slow_attack`:** settles on noise during
WiFi's low duty cycle and degrades real frames.

## D28: Coarse CFO estimate is owned by the accepted acquisition (2026-09-20)

**Decision:** `acquisition_ctrl` arms the coarse CFO estimator itself: it emits
`cfo_start` on an *accepted* trigger, wired to `cfo_est.start`, and clears
`latched_phase_inc` on trigger accept. One estimate per accepted acquisition; a
descriptor's `phase_inc` is that frame's estimate, or 0 if no fresh estimate
arrived — never the previous frame's.

**Rationale:** Previously `cfo_est.start` was driven by raw
`stf_detect/frame_detect`, and the descriptor took the first `cfo_done` after an
accepted trigger while `latched_phase_inc` was never cleared at the frame
boundary. A spurious/re-trigger could start (and win) an estimate for a frame
that was not acquired, and a frame with no fresh estimate inherited the previous
frame's CFO. This is the frame-boundary rule of D25 applied to the coarse
estimate: shared cross-frame state is reset at the boundary that begins the new
frame, not left to luck.

**Consequence / do not regress:** CFO arming is bound to the same
single-outstanding acquisition decision that pushes the descriptor. Rejected
triggers (FIFO full, too-close, metric fail) never start an estimate.

**Not the EAPOL fix.** This was introduced as a candidate root cause for OTA
EAPOL M2/M4 loss and is **not** it. OTA A/B (ch36, 15 toggles, `-a`) shows the
loss unchanged within binomial noise (pre-fix ~13%, +latch-reset ~9%,
+ownership ~8%). Treat D28 as correctness hardening, not the loss fix. The live
loss investigation is in STATUS.md (`logs/m4/ota_ab/`).

**Fallback caveat:** on accept the latch is cleared to 0, so a frame whose
`cfo_done` is late gets *no* correction rather than the last estimate. With the
current timing (`cfo_done` lands during `S_WAIT_STF_END`) this is rare, but "no
correction" is not obviously safer than a same-transmitter carry — the part
most worth revisiting if estimator timing changes.

**Observability (same change set):** per-reason decode-abort counters and a
last-L-SIG/phase abort snapshot (`DIAG_ABORT_CNT/SIG/CTX`, regs 0x20–0x28), plus
a good-frame tag snapshot (`DIAG_TAG_SIG/CTX`, regs 0x2C/0x30). These are the
A/B backbone and are what falsified the stale-CFO story.

---

## D29: Timing is a reduced-gain estimator with geometric compensation (2026-09-27)

**Decision:** The receiver has no full-gain LTF fine-timing stage. `stf_detect`
is detect-only (delay-16 autocorrelation: `frame_detect` + a coarse `stf_end`);
T1 is resolved by the deliberately truncated `ltf_correlator` (D24: 16 taps)
plus an `acquisition_ctrl` search window and the fixed `T1_OFFSET=19`. The
**(tap count, window geometry, T1_OFFSET) triple is one architectural unit**,
validated jointly against field data — never pairwise.

**Why:** area pressure (D21) paid for the correlator's LUTs by cutting
processing gain (24→16 taps), and the lost gain is repaid in *geometry*: the
window rejects out-of-band peaks and `T1_OFFSET` absorbs the 16-tap peak-shape
bias. What makes that fragile is the correlator's partial-ambiguity sidelobes:
a channel-induced +34-sample lobe is 0.00× the true peak at 64 taps, 0.76× at
32, but **1.19× at 16** — so at 16 taps the window is the *only* thing that can
reject it.

**Evidence (the case that forced this):** the 2026-09 OTA AP-side EAPOL
(M1/M3) loss. D24 proved "16 taps picks the same peak as 24" on a *wide*
(80-sample) search; the real window was narrow and forward-only
(`[stf_end+2, stf_end+31]`), so the +34 lobe sat inside it while the true peak
— `stf_end` jitters −17..+9 around it — sat outside. Each was validated alone;
their joint envelope was never evaluated. Fixed by option B
(`docs/acquisition-window-fix.md`): an stf_end-anchored ping-pong trailing
argmax whose window floats `[se−32, se+20]`, reaching backward for the true
peak and stopping short of the lobe. OTA AP roles recovered to M1 20/20,
M3 19/20 (were 4/20, 7/20).

**Consequence / do not regress:**
- Changing taps, window bounds, or `T1_OFFSET` requires the joint sweep bench
  (`test_late_stf_end_selects_true_peak_not_late_lobe`) **and** an OTA EAPOL
  run — sim/HIL/cable loopback do not exercise the channel lobe.
- The way to restore gain is to *build* the fine stage, not tune geometry: the
  coarse windowed max plus a refinement pass re-correlating the top candidates
  against a full 24–64-tap LTF reference
  (`docs/correlator-reduction-analysis.md`, Option C). Option B is a geometry
  patch; Option C is the real fine stage.
- Generalizes D21/D24: a DSP function replaced by constants/geometry must carry
  the *validation context* of the function it replaced.

**Scope:** this is specifically about T1 estimation. D23 (clean drops) and D25
(frame-boundary state reset) stand independently.

---

## D30: OTA EAPOL is a merge gate for fingerprint-changing RTL (2026-09-27)

**Decision:** No change that moves the build fingerprint merges to main until
**both** gates are logged in `logs/hardware.jsonl` for that fingerprint:

1. `session_end.sh` — sim + HIL + cable **loopback** (cable connected);
2. `eapol_toggle_test.sh` — live **OTA** EAPOL handshake (antenna connected),
   event `eapol_toggle`.

Evidence is keyed on the **fingerprint**, not the commit, so docs, tests, and
firmware changes that don't move the fingerprint need neither gate. Confirm
with `scripts/merge_check.sh` before merging.

**Rationale:** the D29 AP-frame loss (M1/M3) passed sim, HIL, **and** cable
loopback for weeks and only failed on live OTA. The failure mechanism — a
channel-induced +34-sample secondary lobe — does not exist in a cable or
idealized channel: cable loopback cannot create it, HIL cannot, only an
over-the-air multipath channel can. Every lower layer was green while the
actual success metric (D7, EAPOL capture) was silently broken. Verification
evidence is only as strong as its ability to reproduce the field condition;
for the RF/PHY path, that is OTA.

**Cost / coordination:** the OTA setup (antenna) and the loopback setup
(TX→RX cable) are mutually exclusive, so each fingerprint-changing merge costs
one physical swap and a `make deploy` on each side (volatile rootfs). Accepted
as the price of not shipping a class of regression that sim/HIL/loopback cannot
observe. Scope to fingerprint changes so documentation/test/firmware merges
skip it.

**Enforcement:** prose plus a check, not a hard merge hook — a human
`git merge` bypasses git hooks, so the reviewer must run `merge_check.sh`
(and the pre-commit hook now points at it). This rule would have caught the
D29 loss at the D24 change, weeks before it surfaced.
