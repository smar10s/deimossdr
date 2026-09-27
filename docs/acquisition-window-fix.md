# Acquisition window fix — OTA AP-frame EAPOL loss

Status: **implemented, hardware-validated (HIL + cable loopback + OTA
EAPOL).** Branch `fix/eapol-capture`, flashed fingerprint `0x647f83bb`.
The bench is GREEN; see §10 for results. OTA AP-side roles M1/M3 recovered
to 100%/95% (was 20%/35%). Do **not** re-attempt option A.

Read the parent context in D22/D24/D28 first.

---

## 1. Symptom

OTA EAPOL: STA-side frames (M2/M4) decode ~95–100%; AP-side frames
(M1/M3) are lost. lib80211 decodes the same IQ — the failure is inside
the fabric acquisition path.

## 2. Mechanism (established)

- `ltf_correlator` (4-engine / 16-tap partial matched filter, D24) emits a
  metric per ADC sample. The true LTF1 peak is at absolute `ltf + 19`;
  `ltf_correlator`'s own fixed-point truncation is absorbed by
  `T1_OFFSET = 19` in `acquisition_ctrl`.
- `acquisition_ctrl` finds T1 by argmax over a **stf_end-anchored,
  forward-only** window `[stf_end+2, stf_end+31]`.
- But `stf_end` (a normalized-autocorrelation threshold crossing) jitters
  relative to the true peak: measured `stf_end − true_peak ∈ [−17, +9]`
  (38 frames, 5 captures). When the peak precedes `stf_end`, the window
  cannot reach it.
- A **channel-induced `+34`-sample lobe** can be ~1.2× stronger than the
  true peak at 16 taps. (`ideal.py`: a noise-free ideal preamble has *no*
  `+34` lobe at any tap count — it is created by the channel. `what34.py`:
  it is 1.19× at 16 taps, 0.76× at 32, 0.00× at 64 → a 16-tap/channel
  artifact.)
- So the window excludes the true peak and *includes* a stronger late lobe
  → wrong `desc_ltf_pos` → wrong channel estimate. `chan_est` averages
  LTF1+LTF2 (chan_est.v:11, :169-173); a gross mis-position makes them
  differ by the CFO phase and partially cancel → L-SIG EVM spread ~50° →
  `S_PARSE_SIGNAL` abort.

Hazard map pooled over 38 frames (`why.py`), offset from the true peak:
lobes at `+34` (14 frames), `+49..+56`, `+64` (LTF2, 46), `+98`; below
peak at `−3` (and `−39..−66`). **Safe zone ≈ `[−2, +33]`** (36 wide).
The current window reaches `[−15, +41]` rel truth — it overshoots into `+34`.

## 3. What is *not* the problem

- **Not CFO.** Controlled injection on an isolated frame: ±80 kHz → no
  shift; ±400 kHz → only `+4` samples (a `|CFO|` effect). The cross-frame
  CFO correlation was a confound.
- **Not amplitude.** ×0.2–1.0 → no shift.
- **Not a resolvable sidelobe.** LTF1 and LTF2 are the same 64 samples, so
  no matched-filter length distinguishes them. A longer/full matched filter
  would not fix peak *selection*. (This kills the "fine timing via lag-64 /
  64-tap" option — the DSP wall was never the deciding factor.)
- The offset is **intrinsic and reproducible per waveform** (isolated
  frames reproduce their full-window offset exactly) → it is a property of
  the received waveform (channel/noise/interference), i.e. environment-
  dependent, not a fixed artifact and not bounded by a simple parameter.

## 4. Why option A (frame_detect-anchored window) is rejected

A det-anchored forward window `[det+132, det+172]` scores 17/17 on the
continuously-running OTA captures and passes the cable/live tests. But it
**breaks the cold-priming geometry**, and that geometry is real on the HIL
merge gate:

- `stf_detect.soft_clear` is wired to `hil_ctrl.playback_start`
  (`fpga/project/system_bd.tcl:184`).
- `playback_start` is a one-shot edge at playback onset
  (`hil_ctrl.v:164`).
- `soft_clear` zeroes `sample_cnt` (`stf_detect.v:460`), restarting the
  **82-sample correlation-window priming**.
- `deimos_hil_inject` plays the vector verbatim from DDR — there is no
  synthetic lead-in in the tool or in `hil_regression.sh`.

So on every HIL injection the window primes *inside* the preamble:
`det→peak = 113` (measured on `legacy_6mbps_waveform.json`: `frame_detect`
@98, true peak @211). This is the Pluto's behaviour, not a bench artifact.
The three tests that went red under option A (`test_rx_frontend` clean,
`test_rx_pipeline`, `test_latency_ratchet`) encode this HIL geometry and
must stay.

The live ADC path is primed (the frontend docstring is right there), but
HIL is not — and HIL is the gate.

Feasibility of the third option (dual `[det+A, se+B]`): the feasible region
is `A = 119` exactly — one sample wide, zero margin (`dualboth.py`,
`pinA.py`). That is a coincidence, not a design; rejected.

## 5. Option B — se-anchored backward window with a ping-pong trailing argmax

Target window: **`[se−17, se+16]`** (golden peak is `se+13`; OTA peak is
`se−17..se+9`). Scored 38/38 on both OTA and cold HIL, with real margin:
**406 distinct `(A,B)` pairs** cover both geometries, `A ∈ [−35,−7]`,
`B ∈ [7,20]`.

You do **not** need a metric history array. Use a **two-register ping-pong
trailing argmax**:

- `cur`  = running argmax since the last block boundary, reset every `N` samples
- `prev` = final argmax of the previous completed block
- at `stf_end`, seed the forward argmax with `max(cur, prev)`, then keep
  sweeping forward to `se + B`

Cost: `~2 × (29-bit metric + 16-bit position) ≈ 90 FFs` and one comparator.
**No shift register, no argmax tree, no BRAM.** This is roughly half the
WIP patch's flops and removes the wide mux that likely drove its placement
failure — the WIP cost was **not** inherent to option B.

The effective lower edge floats between `se−2N` and `se−N` with block phase.
That is safe only if the whole float range scores — and it does
(`pingpong.py`), for `N=16`:

| N  | worst-phase OTA | float range |
|----|-----------------|-------------|
| 16 | 38/38           | [−32,−16]   |
| 18 | 38/38           | [−36,−18]   |
| 20 | 37/38           | [−40,−20]   |
| 24 | 34/38           | [−48,−24]   |

**Pick `N=16, B=20`.** `N=16` keeps the float range `[−32,−16]` inside the
safe `A ∈ [−35,−7]`, so block phase never needs to be reasoned about again.

`B=20` (not 16) is required to keep the existing
`test_gi2_extension_tolerance` GREEN: its `+15`-sample shift places the
synthetic peak at `stf_end+26`, and a `B=16` forward reach is all-noise
there → false metric-floor reject (`shift=+15: Expected descriptor, got 0`,
reproduced at `B=16`). `B=20` reaches enough of the GI2-extended rising edge
to acquire while still stopping short of the earliest measured `+34` lobe
(`stf_end+25`). A re-run of `pingpong.py` over the surviving 55-frame set
shows `B=12/16/20` **tie** at worst-phase 54/55 for `N=16`, so `B=20` costs
nothing on OTA data.

Implemented with the stored metric truncated to its **top 14 bits**
(`HIST_W=14`). The untruncated form **missed placement by 6 slices**
(1967 available vs 1973 required); truncation saves 30 FFs and the build
closes at WNS +0.169 ns. Ordering only needs the top bits — the low 15 are
noise for peak selection.

## 6. Bench / coverage

- Keep `test_late_stf_end_selects_true_peak_not_late_lobe`
  (`fpga/test/test_acquisition_ctrl.py`) — currently **RED** against the
  baseline RTL; it must go GREEN with option B. It drives the true peak
  *before* `stf_end`, plus a stronger `+34` lobe.
- Keep the existing golden/HIL-geometry tests (they cover cold priming).
- **Add a cold-priming bench case** (no lead-in, window primes inside the
  preamble) rather than removing the live pre-noise ones. Then both warm
  and cold geometries are covered in sim and a green `sim.sh` actually
  means something.

Caveat to close: `det→peak = 113` and `peak = se+13` were measured on one
golden vector. Option B's margin absorbs moderate error, but confirm those
two numbers across the HIL rate ladder before trusting the cold column.

## 7. Plan (executed — see §10 for the as-built deltas)

1. Implement option B in `acquisition_ctrl.v` (ping-pong trailing argmax,
   `N=16`, window `[se−32..−16 floating, se+B]`). Keep `T1_OFFSET = 19`.
   As built `B=20` (not 16) and the stored metric is truncated to 14 bits.
2. Make the RED bench GREEN; keep the golden/HIL tests; add the
   cold-priming case.
3. `./scripts/sim.sh` (full suite, all must pass).
4. **Placement check first** — the one real risk, and it is binary. If
   ~90 FFs still misses, report the slice delta and narrow the stored
   metric to its top ~14 bits (plenty for an argmax compare; ~halves it).
5. Build, verify timing MET, flash, `session_start.sh` fingerprint.
6. `./scripts/session_end.sh` (sim + HIL 80/80 + cable loopback).

## 8. Prior attempts — do not repeat

- **WIP patch** (prior session; not in the repo): `[se−8, se+24]` with a
  32-deep metric history + argmax tree. sim 50/50; missed placement by ~39
  slices. Do **not** reuse its implementation (see §5 for the cheap form).
- **`feat/ltf-fine-cfo`** (`bb96107` → `42b4f45` → `6baf5ce`): lag-64 LTF
  fine CFO. Shelved — premise (coarse-CFO bias) was false, the repro was a
  class-A acquisition collision. Module at
  `archive/ltf-fine-cfo/ltf_fine_cfo.v` (produces *phase*, not a timing
  metric).
- **D24**: correlator reduced 24→16 taps to fit. 16 taps is fine on a wide
  search; narrow-window behaviour is biased. The `+34` lobe is the 16-tap/
  channel artifact.
- **D28**: CFO ownership hardening — correctness, not the loss fix.
- **`8bbf931`**: STF HPF error-term width (real bug, *different* failure:
  `stf_end −64`). Exonerated from the AP loss by controlled A/B.

## 9. Evidence / reproduction

The root-cause work used session-local analysis scripts (Python scratch over
fabric metric dumps and lib80211 ground truth). Those scripts are **not
shipped**; the durable record is the numbers in §2–§5 plus the committed
regression bench (`test_late_stf_end_selects_true_peak_not_late_lobe`, which
sweeps the measured stf_end-jitter envelope). Real captures referenced during
the investigation are gitignored recordings and may not be present.

Shipped tooling to reproduce offline:

- `scripts/host/validate_window.c` — independent lib80211 ground truth
  (`ltf` per frame) for any raw capture.
- `fpga/test/diag_lsig_constellation.py` — replay a window; prints
  `stf_end` / `frame_detect` / descriptors / metric profile;
  `OTA_METRIC_DUMP=<file>` writes the metric dump used for the hazard map.
- `fpga/test/diag_ota_window_replay.py` — replay
  `captures/ota_window_capture.json` (gitignored) through `rx_frontend`.

## 10. Outcome (implemented)

`acquisition_ctrl.v`: two-register ping-pong trailing argmax (`BLOCK_LEN=16`),
seeded at `stf_end` with `max(cur, prev)`, forward sweep `SEARCH_POST=20`;
`HIST_W=14` metric truncation for placement. `T1_OFFSET=19` unchanged.

| Gate | Result |
|------|--------|
| `test_acquisition_ctrl` (11 tests) | PASS — bench `ltf_pos=231` = true peak |
| full `sim.sh` | 50/50 PASS |
| timing | MET, WNS +0.169 / WHS +0.024 |
| utilization | 15695 LUTs / 15063 FFs |
| flashed fingerprint | `0x647f83bb` (`c3b557d7…`), `make validate` MATCH |
| HIL regression | 80/80 |
| cable loopback 6/9/12/18/24/36 | 20/20 each (48M 82%, 54M 17% report-only) |
| OTA EAPOL handshake (20 toggles, ch36) | M1 20/20, M2 19/20, M3 19/20, M4 19/20 |

New bench: `test_cold_priming_geometry_finds_peak_after_stf_end` (no lead-in,
`det→peak=113`, `peak=se+13`) guards against the option-A det-anchor
regression. `test_late_stf_end_selects_true_peak_not_late_lobe` is GREEN.

**OTA confirmation (the actual success metric):** AP-side EAPOL roles
recovered — M1 20/20 (was 4/20 = 20%), M3 19/20 (was 7/20 = 35%); STA M2/M4
19/20. Total 3 missing / 80 (`eapol_toggle_test.sh`, event `eapol_toggle`; the
evidence log is local-only).
The AP-frame loss is fixed on air.

Caveat still open: `det→peak=113` / `peak=se+13` were measured on one golden
vector; confirm across the HIL rate ladder.
