# Q-Format Pipeline Budget

End-to-end fixed-point word-growth tracking for the deimos OFDM receiver.
Every wire has a documented Q-format. Updated when RTL changes bit-widths.

## Pipeline Summary

```
ADC → cfo_mixer → FFT → chan_est → equalizer → pilot_track → demapper → Viterbi
Q11.0   Q11.0    Q15.0    (H_inv)    Q9.6*       Q9.6*       Q7.0s      1-bit hard
12b     12b      16b      16b+5b     16b         16b         8b
```

*Q9.6 is the "intent" format — constellation points are scaled by 2^EQ_SCALE=64.
The demapper emits integer LLRs clamped to ±127 (two per clock, I and Q).

Note: `adc_sync.v` lives in `platform/styx/fpga/rtl/`, not `fpga/rtl/`.

## Stage Detail

### ADC Input (`adc_sync.v`)

| Property | Value |
|----------|-------|
| Width | 12-bit signed (I and Q) |
| Q-format | Q11.0 (integer, no fractional) |
| Range | [-2048, +2047] |
| Growth | None (clock-domain crossing only) |
| Saturation | None needed |

### CFO Mixer (`cfo_mixer.v`)

| Property | Value |
|----------|-------|
| Input | 12-bit signed Q11.0 |
| Sin/cos table | Two 1024×16 tables (cos, sin), Q1.14 (16384 = 1.0) |
| Multiply | 12 x 16 = 28-bit products |
| Accumulate | 28-bit sign-extended to 29-bit (complex add/sub) |
| Shift | >>14 (removes Q1.14 scale) → 15-bit |
| Output | 12-bit signed Q11.0, **saturated** from 15-bit |
| Overflow | Clamp to [-2048, +2047] |

### FFT (`fft64_sdf.v`)

| Property | Value |
|----------|-------|
| Input | 16-bit signed (12-bit ADC sign-extended to 16 — `decode_engine.v:309-310`) |
| Architecture | R2²SDF, 3 stages: 64→16→4 |
| Butterfly growth | +1 bit per butterfly (add/sub), +2 per stage pair |
| Twiddle multiply | 18-bit data x 16-bit Q0.15 twiddle = 35-bit |
| Twiddle rounding | +16384 (half-LSB) then >>15 → back to 16-bit |
| Final internal | 20-bit (fft16 output with 2 guard bits) |
| Output | 16-bit signed Q15.0, **saturated** from 20-bit |
| Overflow | Clamp to [-32768, +32767] |

Note: twiddle rounding at each stage prevents unchecked bit growth.
The 64-pt FFT has theoretical 6-bit growth (log2(64)); twiddle normalization
absorbs most of it, with the output saturation catching the remainder.

### Channel Estimation (`chan_est.v`)

| Property | Value |
|----------|-------|
| Input | 16-bit signed (FFT bins from two LTF symbols) |
| H computation | (LTF1 + LTF2) >> 1 → 16-bit signed, × LTF reference sign (`ltf_sign_rom`) |
| \|H\|^2 | 16x16 + 16x16 = 32-bit unsigned |
| Adaptive shift | `sv = 10 + bit_w[5:1]`, clamped to [15, 31] → reachable [15, 26] |
| Division | (|H_re| << shift_val) / |H|^2 via 32-cycle restoring divider |
| H_inv output | 16-bit signed, saturated at ±32767 |
| H_inv format | H_inv = conj(H) * 2^shift_val / |H|^2 |
| Negation | `sat_abs16()` — clamps -32768 to 32767 before division |

The shift_val encodes the implicit fractional scaling. Higher shift_val
means more precision (larger H_inv values for the same channel).

### Equalizer (`equalizer.v`)

| Property | Value |
|----------|-------|
| Input Y | 16-bit signed (FFT data bins) |
| Input H_inv | 16-bit signed (from chan_est) |
| Complex multiply | 16x16 = 32-bit products (4 DSP48E1) |
| Add/sub | 32-bit (rr-ii, ri+ir) — no overflow possible |
| EQ_SCALE | 6 (output retains 6 extra magnitude bits) |
| Effective shift | shift_val - 6 (minimum 0) |
| Rounding | +(1 << (eff_shift-1)) before >>> eff_shift |
| Output | 16-bit signed, **saturated** from 33-bit |
| Overflow counter | NOT BUILT — `re_sat_ovf`/`im_sat_ovf` exist as combinational wires only; no counter/register. Planned observability item (Batch 2) |

Expected output magnitudes (ideal channel). 802.11a scales each constellation
by K_MOD (1, 1/√2, 1/√10, 1/√42), so the innermost point lands at
`64 × K_MOD` — which is exactly the `norm` value the demapper is configured
with (`decode_engine.v:196-203`):

| Modulation | K_MOD | Constellation points | Output magnitudes | norm |
|------------|-------|---------------------|-------------------|------|
| BPSK | 1 | ±1 | ±64 | 64 |
| QPSK | 1/√2 | ±1 (each axis) | ±45 | 45 |
| 16-QAM | 1/√10 | ±1, ±3 | ±20, ±61 | 20 |
| 64-QAM | 1/√42 | ±1, ±3, ±5, ±7 | ±10, ±30, ±49, ±69 | 10 |

Do NOT "correct" these to the un-normalized values (±64/±192/±320/±448).
That table ignores K_MOD and contradicts the norm ROM. The same stale claim
is currently in the `equalizer.v:214` comment — the RTL comment is wrong,
not this table.

### Pilot Tracking (`pilot_track.v`)

| Property | Value |
|----------|-------|
| Input | 16-bit signed (from equalizer) |
| Phase estimate | cordic_atan2_sm on pilot average → 16-bit angle |
| PLL | alpha=0.5 (shift 1), phase_acc tracks residual CFO |
| Data rotation | cordic_rotate, 8 iterations |
| Internal width | 18-bit (16 + 2 guard bits for CORDIC) |
| Gain compensation | x39797 >>> 16 (= 1/K, applied after iteration 8) |
| Output | 16-bit signed, **saturated** from 18-bit |
| SIGNAL bypass | Passthrough (no rotation, no gain change) |

CORDIC gain: the rotation applies the ×39797>>16 compensation, so the
NET data-path gain is ~unity (measured 0.999992), NOT ~0.6073. BPSK
±64 stays ±64. The demapper's norm table (64/45/20/10,
decode_engine.v rate ROM) is built on unity gain — if anyone "fixes"
the RTL to match a 0.6073 doc claim, 16-QAM/64-QAM break. Do not
"correct" the compensation into an attenuation.

### Demapper (`demapper.v`)

| Property | Value |
|----------|-------|
| Input | 16-bit signed (constellation point) |
| Config | `norm[15:0]` — expected innermost constellation distance |
| Output | 8-bit signed LLR, **saturated** to [-127, +127] |

LLR scaling by modulation:

| Modulation | Amplification | Saturation function |
|------------|--------------|-------------------|
| BPSK | x1 (direct) | sat16: 16→8, clamp ±127 |
| QPSK | x1 (direct) | sat16: 16→8, clamp ±127 |
| 16-QAM | x2 | scale2_sat16 / scale2_sat17: x2 then clamp ±127 |
| 64-QAM | x4 | scale4_sat / scale4_sat16: x4 then clamp ±127 |

Every LLR path has explicit saturation. No wraparound possible.

## Saturation Points (complete list)

| Module | Location | From → To | Method |
|--------|----------|-----------|--------|
| cfo_mixer | output | 15b → 12b | Overflow detect + clamp |
| fft64_sdf | twiddle_64 stage 4 (`:678-683`) | 21b → 16b | Compare + mux |
| fft64_sdf | twiddle_16 stage 4 (`:1053-1058`) | 21b → 16b | Compare + mux |
| fft64_sdf | output | 20b → 16b | Compare + mux |
| chan_est | div_result | 32b → 16b | Compare + sign + clamp |
| chan_est | negation | -32768 → 32767 | sat_abs16() |
| equalizer | shifted output | 33b → 16b | Compare + mux |
| cordic_rotate | output | 18b → 16b | Compare + mux |
| demapper | all LLR paths | 16-19b → 8b | 5 saturation functions in use (`sat17` is defined but never called) |

## Overflow Exposure

The equalizer output is the one place where a deep channel fade could
overflow: a large `H_inv` multiplied by a moderate FFT output can exceed
16-bit range. It is saturated (`equalizer.v:252-264`), but the saturation
is **not counted** — `re_sat_ovf`/`im_sat_ovf` are combinational wires with
no counter behind them (see the equalizer table above). So clipping here is
silent in telemetry, not silent in data.

`chan_est`'s -32768 negation was safe by accident (unsigned storage holds
0x8000 = 32768 correctly) but relied on implicit behavior; it now uses
`sat_abs16()` explicitly.

Known latent issue: `equalizer.v:262,264` select the clamp direction from
`shifted_full_re[31]`, but that wire is 33-bit signed — the sign bit is
`[32]`. For `|shifted_full| ≥ 2^31` at `eff_shift=0` the clamp would go the
wrong way. Effectively unreachable, tracked in STATUS Known Issues.
