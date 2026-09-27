# Throughput Analysis: Lead-In Decomposition and Rate-24 Path

**Date:** 2026-08-27 (updated 2026-09-16)
**Commit:** a4cdc06 (original analysis); 2026-09-16 measured on `feat/streaming-demapper`
**Status:** Lever C implemented and hardware-validated; 48/54 deferred

> **Update 2026-09-16 (lever C implemented — `feat/streaming-demapper`).**
> The streaming demapper shipped. The demapper is now a pure 3-stage
> pipeline — no symbol buffer, no capture/prime/emit FSM — that emits one
> wide word (all `n_bpsc` LLRs) per subcarrier. The deinterleaver captures
> wide and stores `48×48b`, gathering via a `(subcarrier<<3)|lane`-encoded
> perm ROM (`scripts/gen_deint_enc.py`). Measured in sim (deterministic):
> per-symbol compute windows (`deint_done - symbol_start`) **314 / 338 / 386**
> clk/sym at rates 6 / 12 / 24 (periods 327 / 340 / 388); all under the
> 400-clock budget. `diag_decode_profile` shows `demap = 48` at every rate
> (was 48/96/144) and lead-in `demap@` 288 → 238 — the demapper emit phase
> is gone. Hardware (fingerprint `0x4a8f4734`): WNS +0.448, LUTs 15596
> (−74), BRAM 43 (net 0); HIL 80/80, cable loopback gate 6/6 rates 20/20.
> The lever-C estimates below (rate 24 ~386, BRAM-neutral, LUT-negative)
> were confirmed. 48/54 remain over budget (436 clk/sym each; see the
> corrected table below) and deferred.
> Line references in the lever-C discussion were re-derived against the
> as-built streaming demapper (section C).

> **Task 3-only Viterbi (D23).** The Viterbi
> now runs at **1.278 clk/pair** (was 3.31), which removes it as the
> coded-bit-path limiter at 48/54M. Measured `diag_throughput` this branch:
> `366 / 390 / 486 / 582` at 6 / 12 / 24 / 48 clk/sym (6-18 realtime,
> 24-54 over budget). The 24/36 tail is **unchanged** — it was never
> Viterbi-limited. The lever-C projection below is unchanged and now applies
> to a non-stalling Viterbi.

---

## Current Per-Symbol Latency

Measured via `diag_symbol_cycle` / `diag_throughput` (deterministic
compute windows; the `symbol_start` period is feed-jittery at low rates,
e.g. rate 6 Min 316 / Max 400, while the feed-independent compute window is
stable). Ratchets in `fpga/test/test_latency_ratchet.py`.

| Rate | Measured (clk/sym) | Budget | Delta | Status |
|------|--------------------|--------|-------|--------|
| 6    | 327                | 400    | −73   | Realtime |
| 9    | 316                | 400    | −84   | Realtime |
| 12   | 340                | 400    | −60   | Realtime |
| 18   | 340                | 400    | −60   | Realtime |
| 24   | 388                | 400    | −12   | Realtime |
| 36   | 388                | 400    | −12   | Realtime |
| 48   | 436                | 400    | +36   | Over budget |
| 54   | 436                | 400    | +36   | Over budget |

`clk/sym` is the `symbol_start` *period*. The feed-independent compute
window (`deint_done - symbol_start`) is 314/338/386 at rates 6/12/24 and 434
at 48/54. (Rates 48/54 were ~680 stall-limited before the Task 3-only
Viterbi and 582 after it; lever C then removed the serialized demapper tail
and they now sit at 436.)

The budget is 400 clocks = 80 samples × 5 fabric clocks/sample (100 MHz
fabric / 20 MSPS ADC). One OFDM symbol period.

Rate 24 is the target: common in 20 MHz legacy traffic (44% of decoded
frames per OTA measurement). Rates 48/54 are deferred — nice for
completeness but not practically required. 48 is reachable by lead-in
reduction (lever A); 54 additionally needs a ≥2 bit/clk depuncturer.

## Throughput Model (pre-lever-C; retained as the design record)

> **Historical.** The model below describes the design *before* lever C. The
> demapper emit term has since been deleted (streaming demapper, D26), so the
> "Total"/"Measured" columns are pre-lever-C; current numbers are in the
> table above and D26. The structural reasoning — why the two emit phases
> were serialized — still explains the old 486/582.

Per-symbol latency = lead-in + two serialized emit phases:

```
clk/sym = 294 + demapper_emit + deinterleaver_emit
                (48 × step_bpsc)   (NCBPS / 2)
```

| Rate | Mod | NCBPS | demap emit | deint emit | Total (model) | Measured |
|------|-----|-------|-----------|------------|---------------|----------|
| 6    | BPSK  | 48  | 48  | 24  | 294+72=366  | 366 ✓ |
| 12   | QPSK  | 96  | 48  | 48  | 294+96=390  | 390 ✓ |
| 24   | 16-QAM | 192 | 96 | 96  | 294+192=486 | 486 ✓ |
| 48   | 64-QAM | 288 | 144 | 144 | 294+288=582 | 582 ✓ |

Four exact hits. Rate 48 used to diverge (~680) because the old
3.31 clk/pair Viterbi was the binding constraint in the coded-bit path
(144 pairs × 3.31 = 477 clk/sym, and backpressure through `vit_fifo`
stalled the chain). The Task 3-only Viterbi (1.278 clk/pair) removes that
stall, so rate 48/54 now match the model; their residual overage is the
serialized tail, not the Viterbi.

**Both emit phases scaled, and they were serialized (pre-lever-C).** Two
independent ×2 penalties between rate 12 and rate 24, +48 clocks each:

- The demapper's LLR output bus was fixed at 2 lanes (`soft_out0`/
  `soft_out1`), so `step_bpsc = n_bpsc >> 1`: BPSK/QPSK = 1
  clk/subcarrier (48 total), 16-QAM = 2 (96), 64-QAM = 3 (144). QPSK was
  free because 2 bits fit the 2 lanes exactly. **The LLR arithmetic did
  not scale at all** — all six candidates were computed combinationally in
  parallel every clock, and stage 3 was a pure 2-wide byte MUX. Only the
  output port was serial. Lever C widened the port: as-built the demapper
  emits one subcarrier per clock on six lanes (`demapper.v:50-56`, stage 3
  `demapper.v:267-274`).
- The deinterleaver gathered at a fixed 2 bits/clk for all rates (BRAM
  dual-port ceiling), so emit = `NCBPS/2`. As-built the gather works on a
  `48×48b` word buffer with a `(word, lane)` perm encoding
  (`deinterleaver.v:73,283-360`).
- They did not overlap: `S_CAPTURE` exits only on `!valid_in`
  (`deinterleaver.v:328-342`), so deinterleaver emit could not begin until
  the demapper's last output. Its *capture* does overlap demapper emit —
  that overlap was already exploited. The emit-after-capture dependency is
  what made the two costs additive.

**To bring rate 24 under 400, remove the demapper emit phase (~100
clocks).** Done — lever C (D26). The tail was *not* already minimized —
it held 192 clocks at rate 24, and half of it was removable without
touching the lead-in.

> **Corrected 2026-09-03.** This section previously modelled the tail as
> `NCBPS/2 + 3 + 5` and concluded the fix had to come from the 286-clock
> lead-in. That formula omits the demapper emit phase entirely: it gives
> 286+104 = 390 for rate 24, not the measured 486. The old table's
> "Total" column silently used the correct +200 while its own formula
> column said +104 — a 96-clock internal contradiction. Lever C was
> correspondingly estimated at ~5 clocks instead of ~100.

## Lead-In Decomposition (286 clocks to first demap_valid)

Traced from decode_engine `S_FFT_FEED` entry to first `demap_valid`.
Note this is a *per-symbol* constant, not a once-per-frame cost — the
genuine per-frame costs (LTF FFT passes, `chan_est` S_FIND_MAX 128 clk
and S_COMPUTE ~3700 clk, SIGNAL symbol, N_SYM division) total ~4550
clocks and are absorbed by `circ_buf`.

| # | Stage | Clocks | Cumul. | Notes |
|---|-------|--------|--------|-------|
| 1 | DE FSM setup | 2 | 2 | rd_ptr set + fft_rst_n assert |
| 2 | CFO mixer | 10 | 12 | 5 compute + 5 delay-match |
| 3 | FFT64 SDF fill | 75 | 87 | 64 data + pipeline(12) + FILL_SKIP(63) − overlap |
| 4 | FFT streaming 64 bins | 63 | 150 | 1 bin/clock output |
| 5 | FFT bin register | 1 | 151 | decode_engine registered output |
| 6 | EQ capture→prime | 2 | 153 | S_CAPTURE end + S_PRIME BRAM setup |
| 7 | EQ process + pipeline | 58 | 211 | 48 data + 1 prime + 4 pilots + 5 pipe drain |
| 8 | Pilot track atan2 | 10 | 221 | 1 launch + 9 CORDIC iterations |
| 9 | Pilot track PLL + emit start | 2 | 223 | Update phase_acc, BRAM prefetch |
| 10 | Pilot track CORDIC rotate | 10 | 233 | Pipeline depth to first output |
| 11 | Pilot track emit remaining | 47 | 280 | 48 total symbols, 1/clock |
| 12 | Demapper gap + prime | 5 | 285 | Detect end, BRAM setup, first output |
| 13 | Transition fences | ~1 | 286 | FSM edge alignment |

### Structural Observations

- **FFT (stages 3-5): 139 clocks, 49% of lead-in.** Dominated by the
  64-sample SDF fill and FILL_SKIP=63 gating. The SDF architecture does
  need all 64 input samples before valid output — but note the FFT is
  **reset every DATA symbol** (`decode_engine.v:753-777`, with
  `system_bd.tcl:113` tying `gate_rst_n` to `fft_rst_n`), which is what
  forces the full FILL_SKIP refill. Every SDF delay line and counter is
  already `en`-gated by `din_valid` (`fft64_sdf.v:454, 489, 833, 892`),
  so pausing rather than resetting between symbols is a design choice,
  not an architectural constant. The pre-refactor `logs/latency_baseline.jsonl`
  entries (fft_wrap-era keys `fft_wrap_continuous: 204` vs
  `fft_wrap_live_mode: 457`, null from 2026-06-23 onward) put this at worth
  ~70 clocks; high risk (bin-phase alignment via `gcnt`, and `chan_est.v:34-35`
  depends on the inter-symbol bin gap). Unexamined, not disproven.

- **Demapper (stage 12): 5 clocks, 2%.** This counts only the gap-detect
  and BRAM prime. The 96-clock *emit* phase that follows is the single
  largest removable term in the whole budget — see the Throughput Model
  section and lever C.

- **Equalizer (stages 6-7): 60 clocks, 21%.** Capture-then-process:
  waits for all 64 bins, then sequentially processes 48 data + 4 pilots
  through a 5-stage multiply/shift pipeline. The 64-bin capture is
  overlapped with FFT streaming, so only the post-capture processing
  (58 clocks) is on the critical path.

- **Pilot track (stages 8-11): 69 clocks, 24%.** Collect-then-emit:
  waits for all 52 EQ outputs, computes atan2 (phase error from 4
  pilots), updates PLL, then emits all 48 data symbols through
  cordic_rotate. The serial atan2→PLL→emit chain is the dominant term.

- **Demapper (stage 12): 5 clocks, 2%.** Capture-then-emit with trivial
  prime overhead.

- **Fixed overhead (stages 1-2): 12 clocks, 4%.** CFO mixer pipeline
  includes 5 delay-match stages. **These are not vestigial** — see
  lever B for why removing them globally is a D22/D24 regression.

## Candidate Reductions

Ordered by estimated savings. None are committed. All savings figures are
**estimates from RTL cycle-counting**, not measurements — the only
measured numbers in this document are 366/390/486 from `diag_symbol_cycle`.

### C. Streaming demapper (~100 clocks) — closes the gap alone — IMPLEMENTED

> **Implemented 2026-09-16** (`feat/streaming-demapper`, D26). Measured rate 24
> 486 → 388 clk/sym (compute window 386). This section is retained as the
> design record. As-built: the demapper is a pure 3-stage pipeline emitting
> one subcarrier per clock on `soft_wide0..5` (`demapper.v:50-56,267-274`);
> `deinterleaver.sbuf` is `48×48b` (`deinterleaver.v:73`) and gathers by
> `enc[8:3]` word / `enc[2:0]` lane (`deinterleaver.v:283-360`).

**This was the recommended lever.** Before lever C: capture all 48 subcarriers
into `sym_buf`, detect the end-of-stream gap, 4-clock BRAM prime, then
emit over `48 × step_bpsc` clocks (96 at rate 24).

The LLR datapath was already fully parallel and per-subcarrier independent,
so driving it directly from the `demap_mux` output as subcarriers arrive from
`pilot_track` — and writing all `n_bpsc` LLRs as one wide word into the
deinterleaver buffer — removes the serialization. The gather encodes the
arrival position as `enc(p, n_bpsc) = ((p // n_bpsc) << 3) | (p % n_bpsc)`
— 6-bit subcarrier + 3-bit lane, exactly the existing 9-bit perm ROM width —
so the dual-port gather still reads 2 arbitrary bits/clk. The `word b>>2,
lane b&3` shortcut holds only for `n_bpsc ∈ {1,2,4}`; 64-QAM needs the
precomputed `enc`.

- Deletes the gap-detect + prime (4) and the whole emit phase (96)
- **Rate 24: 486 → ~386** (measured 388). Rate 12: 390 → ~342. Rate 6: 366 → ~318
- Change set: `demapper.v` (delete `sym_buf` and the
  `S_CAPTURE`/`S_PRIME0-2`/`S_EMIT`/`S_DRAIN1-2` FSM), `deinterleaver.v`
  (widen `sbuf` to one wide word per subcarrier), `system_bd.tcl`, and the
  regenerated sim view
- Resource: **BRAM-neutral, LUT-negative** (confirmed). `sbuf` stays 2304
  bits (288×8b → 48×48b), but the 48-bit width forces 2 RAMB18 instead of 1
  (+1), cancelled by deleting the demapper's `sym_buf` (1 RAMB18) and the
  prefetch/step FSM. The earlier **+2 to +4 RAMB18** estimate was wrong.
  Respects the D21 headroom constraint, unlike most alternatives
- Risk: **medium.** Touched two BRAM prefetch schemes and `deint_done`,
  which gates the FSM at `decode_engine.v:447`. LLR *values* are unchanged —
  only their transport — so bit-exactness was directly checkable against the
  prior output

**Cheaper subset (superseded):** widen the demapper output to `n_bpsc`
lanes while keeping the capture FSM. Emit 96 → 48, **rate 24 → ~438** — not
sufficient alone. It landed as the intermediate step; the final
implementation also deleted the capture phase to reach ~386.

### A. Streaming pilot track (~55 clocks)

Current: collect 52 → atan2(9) → PLL(1) → emit 48 through CORDIC(58).
Proposed: rotate data through CORDIC **as it arrives from EQ**, using
the previous symbol's `phase_acc`. Update PLL asynchronously for the
next symbol.

- Collapses stages 8-11 (69 clk) to the `cordic_rotate` depth (~11), and
  the 48 emit clocks merge into the EQ's existing 48 output clocks →
  **~55, not 70.** Rate 24: 486 → ~430. Insufficient alone
- Resource: ~neutral; may free the 48×32b `data_buf` (`pilot_track.v:137`)
- Risk: **high, and higher than previously stated.** `ALPHA_SHIFT = 1`
  (alpha = 0.5, "aggressive tracking for short frames",
  `pilot_track.v:68`) and `phase_acc <= 0` on SIGNAL
  (`pilot_track.v:310`) mean DATA1 would be rotated by *zero* phase
  instead of by its own measured pilots. For short frames (an ACK is 2-4
  DATA symbols) that is a large fraction of the frame. The
  "~0.5°/symbol at typical residual CFO" justification has no supporting
  measurement in this repo — characterize with `hil_test.sh -l 3` first

### B. Remove CFO mixer delay-match (~5 clocks) — scope carefully

`cfo_mixer.v:206-214` has 5 delay-match registers (total 10-cycle latency
= 5 compute + 5 match).

**The delay-match is not dead.** `cfo_mixer` is instantiated twice:
`system_bd.tcl:59` (`cfo_mixer_0` → `ltf_correlator_0`) and
`decode_engine.v:282` (`u_feed_mixer`). For the acquisition instance the
10-clock latency sets the alignment between `wr_ptr` and the correlator
metric, compensated by `T1_OFFSET = 19` (`acquisition_ctrl.v:87`).
D22 names the "10-clock `cfo_mixer` delay on
hardware" as a candidate mechanism behind the SIFS burst-drop bug, and
D24 ties `T1_OFFSET` to the tap count. Removing 5 stages shifts the FFT
window by 5 samples — 31% of the 16-sample cyclic prefix.

Viable **only** as a `DELAY_MATCH` parameter set to 0 on `u_feed_mixer`
alone, leaving the acquisition instance at 5. Global removal is a
D22/D24 regression.

### D. Pipelined equalizer (speculative, ~55 clocks)

Current: capture all 64 bins, then process sequentially. If the EQ could
process bins as they arrive from FFT, the sequential processing would
overlap with FFT streaming.

Risk: **high.** Bins arrive bit-reversed but output must be in
`data_bin_rom` order (`equalizer.v:68-86`), so out-of-order processing
needs a reorder buffer. Note that `pilot_track`'s `data_buf[data_idx_in]`
(`pilot_track.v:162`) *is* already exactly that buffer — which is why D
is cheap against today's collect-then-emit pilot_track, and why **D
conflicts with A** (see Summary).

### Summary

| Combination | Total savings | Rate 24 result | Feasibility |
|-------------|---------------|----------------|-------------|
| **C alone** | **~100**      | **~386**       | **Medium — recommended** |
| C + B (scoped) | ~105       | ~381           | Medium |
| C-subset (4 lanes) | ~48    | ~438           | Low risk, insufficient |
| A alone     | ~55           | ~430           | High risk |
| A + C-subset | ~103         | ~382           | High risk (A dominates) |
| D alone     | ~55           | ~431           | High risk |

**C alone clears the budget with 14 clocks of margin**, at neutral-or-
better LUT cost. If more margin is wanted, add B scoped to `u_feed_mixer`.

**Do not stack A and D.** Their savings do not add: D needs the reorder
buffer that exists only because pilot_track is collect-then-emit, and A
deletes exactly that collect phase (`pilot_track.v:336-345`). Doing both
means moving the reorder into the demapper's `sym_buf`, whose capture is
strictly sequential on `wr_count` (`demapper.v:449-465`). A+D is a third,
larger rewrite — not a composition of two levers. (An earlier version of
this table listed "A + D → ~358" as if it composed.)

### Measurement caveat

The budget is exactly 400 clocks = 80 samples × 5 clk, and one OFDM
symbol *is* 80 samples. Once the pipeline drops below 400, the decode
engine starts waiting on `num_avail >= 64+16` (`decode_engine.v:911`) and
enters `data_buf_wait` (`:891-902, 920-922`).

**`diag_symbol_cycle` will therefore report ~400, not ~386** — it becomes
ADC-limited, which is the goal. Do not read a plateau at 400 as "the fix
didn't work." True pipeline capability must be measured with a
faster-than-live feed, and `test_latency_ratchet.py` should assert
`<= 400` rather than an exact sub-400 number.

## What Is Already Confirmed

Do not spend effort re-deriving these — they were verified against the RTL.
All are off the rate-24 critical path; the depuncturer is the one that also
binds at rate 54 (see its entry).

- **Viterbi.** Task 3-only (D23): traceback overlaps ACS, the
  13-clock best-state search is serialized → **1.278 clk/pair** (measured,
  `test_streaming_throughput_ratchet`). At rate 48/54: 144/216 pairs →
  184/276 clk/sym, so the coded-bit path no longer binds — the streaming
  latency does (582 before lever C, **436** after). At rate 24: 96 pairs →
  123 clk/sym ≪ 388. Confirms D23. (Was 3.31 clk/pair before this branch;
  at rate 48 the old Viterbi needed 477 clk/sym and stalled the chain to
  ~680.)
- **Depuncturer — off the rate-24 path, but a hard floor at 54.** It emits
  1 bit/clk (`depuncturer.v:198-204`), so a punctured symbol costs
  `2 × NDBPS` clocks: 192 at rate 24 and 384 at 48, each below the 400
  budget, and it runs *behind* `deint_done`, hiding under the next symbol's
  lead-in. At rate 54 it needs **432** clocks, above the 400 budget, so
  there it is a genuine throughput wall, not hidden latency: no lead-in
  reduction puts 54 under 400 without widening this module to ≥2 bit/clk.
  The earlier "hides under the next symbol's lead-in" claim was true only
  while `2 × NDBPS ≤ lead-in`.
- **FIFO handshake stalls.** `depunct_full`/`vitf_full`
  (`rx_pipeline.v:597, 615, 631`) do not assert at rate 24 in steady
  state. The old 3.31 clk/pair Viterbi was the entire reason rate 48
  measured ~680 vs the model's 582; Task 3-only removed that stall and
  lever C then took it to **436**.
- **Equalizer `S_DONE` drain.** `eq_done` only clears `eq_rd_sel` and
  advances the FSM; the next FFT feed is gated on `deint_done`, not
  `eq_done` (`equalizer.v:436-448`). Shaving EQ pipeline stages saves zero.

## References

- D23 (Viterbi not the limiter) and D23 (400-clock budget, clean-drop
  policy) — `DECISIONS.md`
- Latency ratchets: `fpga/test/test_latency_ratchet.py`
- Throughput model validation: `logs/latency_baseline.jsonl`
- Per-symbol measurement: `diag_symbol_cycle` (`fpga/test/`)
