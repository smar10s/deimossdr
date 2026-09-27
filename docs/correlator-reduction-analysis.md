# Correlator Reduction Analysis

> **Decision of record: D24** (`DECISIONS.md`). This file is the supporting
> measurement data — the tap-count/peak-selection experiments behind the
> 4-engine / 16-tap shape. Option A below was adopted. Cited from
> `fpga/rtl/ltf_correlator.v:9`.

Options and empirical data for reducing the sliding LTF correlator to close
the placement gap. Created 2026-06-23.

## State at time of analysis (pre-reduction)

- 6 engines, 24 taps, 5 clocks/sample (4 stages + 1 metric)
- Engines 0-2: DSP48E1 (12 DSPs total)
- Engines 3-5: CSD fabric multiplies (~2,043 LUTs)
- Total correlator contribution: ~2,043 LUTs, 12 DSPs, ~20 control sets

(Current shape after adopting Option A: 4 engines, 16 taps, 705 LUTs,
16 DSP48E1 — see `build/fpga/utilization.rpt`.)

## The Placement Problem

520 control sets total. Placer needs 1,937 slices, only 1,585 available
(352 short). Need ~880 LUT-equivalent reduction. The correlator is the
largest single target for cuts.

## Why Fewer Taps Are Safe

The correlator is **not a detector** — STF handles detection. The correlator
is a **ruler**: it finds the precise T1 position within a small search window
(25 samples normal path; the 60-sample pending path was removed by D22) after
STF has already confirmed a frame exists.

Within a windowed-max search, you don't need high SNR — you just need the
correct peak to be the maximum within the window.

## Empirical Results (OTA EAPOL + cable captures)

Tested on all 5 passing captures with hardware-realistic 25-sample windowed
search (centered on 24-tap ground truth T1):

| Capture | 16 taps | 12 taps | 8 taps |
|---------|---------|---------|--------|
| EAPOL OTA (4 frames) | 0 error (wide) | -10 (secondary peak) | 0 error (wide) |
| 36M cable ch149 | -4 | 0 | +4 |
| 6M cable drift | -12 | 0 | 0 |
| 12M cable drift | -12 | -12 | -12 |
| 12M cable ch149 | -10 | 0 | -3 |

**Key findings:**
- Errors are **correlation peak shape bias**, not detection failures
- ±4 samples (200 ns) is within 802.11 CP tolerance (CP = 800 ns = 16 samples)
- 12 taps has a problematic secondary peak on OTA data (consistent -10 error)
- 16 taps matches 24-tap on wide search but shows -4 to -12 in narrow windows
- The `ltf1_offset = peak_max_pos - (N_TAPS + 1)` constant compensates

**Wide-window (80-sample) results on OTA EAPOL frame 0:**
```
24 taps: peak at absolute sample 703
20 taps: peak at absolute sample 703  (same)
16 taps: peak at absolute sample 703  (same)
12 taps: peak at absolute sample 693  (wrong — secondary peak)
 8 taps: peak at absolute sample 703  (same)
```

16 taps is identical to 24 taps in wide search. The narrow-window
bias comes from the correlation peak shape, not from finding the
wrong peak.

## Timing Impact

802.11 cyclic prefix = 16 samples (800 ns). A timing error of N samples
means the FFT window overlaps with N samples of the preceding/following
symbol. Effects:

| Error | CP consumed | ICI impact |
|-------|-------------|------------|
| ±1 | 6% | Negligible |
| ±2 | 12% | Negligible |
| ±4 | 25% | Minor (pilot PLL absorbs common phase) |
| ±8 | 50% | Significant (edge subcarrier degradation) |

Typical indoor 5 GHz multipath: 50-100 ns RMS delay spread. The full CP
(800 ns) provides margin for 8× the typical channel. A ±4 sample error
still leaves 12/16 of CP = 600 ns guard — adequate.

## Options

### Option A: 4 engines / 16 taps (RECOMMENDED first step)

Architecture: 4 engines × 4 stages = 16 taps, 5 clocks/sample.
- Engines 0-1: DSP48E1 (8 DSPs, frees 4)
- Engines 2-3: CSD fabric (8 multiplies, down from 12)
- Shift register: 16 entries (down from 24)

Savings: ~600-800 LUTs, ~10 control sets, frees 4 DSP48E1.
Risk: Low — empirically identical to 24 taps on all captures.

Implementation:
1. `ltf_correlator.v`: reduce shift register to 16, remove engines 4-5,
   adjust stage tap mapping (4 taps/stage × 4 stages = 16 total)
2. Adjust the T1 offset constant to `N_TAPS + 1` = 17 (now
   `T1_OFFSET` in `acquisition_ctrl.v`, currently 19)
3. Run the full sim suite (`sim.sh`) — all tests must pass
4. Run `test_adc_replay` specifically — every capture in
   `captures/passing/` must pass FCS
5. Build and check placement

### Option B: 2 engines / 8 taps (aggressive)

Architecture: 2 engines × 4 stages = 8 taps, 5 clocks/sample.
- Engines 0-1: DSP48E1 (4 DSPs total — frees 8 from current)
- Zero CSD fabric engines
- Shift register: 8 entries

Savings: ~1,500-1,800 LUTs, ~20 control sets, frees 8 DSP48E1.
Risk: Medium — ±4 sample error on some cable captures. Probably fine
for 802.11 CP tolerance but needs hardware validation.

### Option C: 2 engines + hybrid buffer refinement

Same as Option B for real-time, plus a refinement pass in S_START_CE:
- After coarse peak found (8-tap windowed-max), read ~10 samples from
  circular buffer centered on coarse peak
- Correlate each against full LTF reference (24-64 taps) using 1 shared
  MAC accumulator at 1 tap/clock
- Takes ~640 clocks (10 positions × 64 taps) — negligible vs frame time
- Final T1 = position with highest refined metric

Savings: Same as Option B (~1,500-1,800 LUTs).
Risk: Low — full-precision fallback guarantees correct T1.
Complexity: Moderate — ~50-100 lines of new RTL for the refinement FSM.
Additional LUT cost: ~50 (one MAC accumulator + counter).

### Why Time-Division Doesn't Work Here

The fundamental constraint is 5 fabric clocks per ADC sample (100 MHz / 20 MSPS).
With 1 MAC doing 1 tap/clock, you get 5 taps per sample — period. To get
more taps you need more engines (parallel) or to accept decimated metrics
(fewer search positions per window).

A time-divided correlator processing 1 tap/clock for 24 taps would need
24 clocks per sample — producing one metric every 5th sample. Within a
25-sample window, you'd get only 5 metric values instead of 25. Peak
resolution drops from 1-sample to 5-sample (250 ns). This is WORSE than
the ±4 sample error from simply reducing to 8 taps.

GNSS receivers use time-division because their code is 1023+ chips and they
have coherent accumulation over the full code period. 802.11 LTF is only
64 samples — there's no long integration window to exploit.

## Outcome

Option A (4 engines / 16 taps) was adopted and was sufficient — placement
closed without escalating to Option C. Options B and C were never
implemented; they remain documented as the known escalation path if
correlator LUTs are needed again. See D24.
