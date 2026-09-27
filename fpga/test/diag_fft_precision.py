"""
diag_fft_precision.py — Precision comparison: fft64_sdf hardware vs Python reference.

Uses REAL golden vector OFDM data (not synthetic tones) to characterize
the exact numerical error introduced by the streaming fft64_sdf when
processing back-to-back symbols.

Strategy:
  1. Load golden vector for rate 24 (where precision matters for 16-QAM).
  2. Extract LTF1 (64 samples), LTF2 (64), SIG (64), DATA1 (64), DATA2 (64).
  3. Feed all 5 frames at burst rate to fft64_sdf (continuous, no reset).
  4. Collect FFT output bins for each frame.
  5. Compare against Python reference_fft64_dif() applied to the same input.
  6. Also run in "reset per frame" mode (simulating decode_engine's per-symbol
     FFT reset).
  7. Report per-bin errors, max error, and whether the gap explains FCS failures.

This diagnoses whether fft64_sdf produces correct results in continuous mode
or whether there's systematic precision loss vs reset-per-frame operation.

Run:
  cd fpga/test && make diag_fft_precision
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

import json
import os
import math

VECTORS_DIR = os.path.join(os.path.dirname(__file__), "../../extern/lib80211/vectors")


def to_s16(v):
    v = int(v) & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


def to_s12(v):
    """Convert to 12-bit signed, clamped."""
    v = int(round(v))
    v = max(-2048, min(2047, v))
    return v


def reference_fft64_dif(x_re, x_im):
    """Radix-2 DIF FFT, 64-point, Q1.14 quantized twiddles, 16-bit signed.
    
    This matches the RTL's twiddle multiply precision (Q1.14 ROM values,
    round-to-nearest after multiply).
    """
    N = 64
    FRAC = 14
    SCALE = 1 << FRAC
    ROUND = 1 << (FRAC - 1)

    d_re = list(x_re)
    d_im = list(x_im)

    for s in range(6):
        block_size = N >> s
        half = block_size // 2
        num_blocks = 1 << s

        new_re = d_re[:]
        new_im = d_im[:]

        for blk in range(num_blocks):
            base = blk * block_size
            for j in range(half):
                idx_top = base + j
                idx_bot = base + j + half

                a_re = d_re[idx_top]
                a_im = d_im[idx_top]
                b_re = d_re[idx_bot]
                b_im = d_im[idx_bot]

                new_re[idx_top] = to_s16(a_re + b_re)
                new_im[idx_top] = to_s16(a_im + b_im)

                diff_re = to_s16(a_re - b_re)
                diff_im = to_s16(a_im - b_im)

                tw_exp = (j * num_blocks) % N

                if tw_exp == 0:
                    new_re[idx_bot] = diff_re
                    new_im[idx_bot] = diff_im
                elif tw_exp == N // 4:
                    new_re[idx_bot] = diff_im
                    new_im[idx_bot] = to_s16(-diff_re)
                elif tw_exp == N // 2:
                    new_re[idx_bot] = to_s16(-diff_re)
                    new_im[idx_bot] = to_s16(-diff_im)
                elif tw_exp == 3 * N // 4:
                    new_re[idx_bot] = to_s16(-diff_im)
                    new_im[idx_bot] = diff_re
                else:
                    angle = -2.0 * math.pi * tw_exp / N
                    wr = max(-32768, min(32767, int(round(math.cos(angle) * SCALE))))
                    wi = max(-32768, min(32767, int(round(math.sin(angle) * SCALE))))
                    prod_re = diff_re * wr - diff_im * wi
                    prod_im = diff_re * wi + diff_im * wr
                    new_re[idx_bot] = to_s16((prod_re + ROUND) >> FRAC)
                    new_im[idx_bot] = to_s16((prod_im + ROUND) >> FRAC)

        d_re = new_re
        d_im = new_im

    return d_re, d_im


def load_ofdm_symbols(rate_mbps):
    """Load golden vector and extract frame symbols at 12-bit quantization.
    
    Returns list of (re_list, im_list) for each 64-sample symbol:
      [LTF1, LTF2, SIG, DATA1, DATA2, ...]
    
    Samples are 12-bit signed then sign-extended to 16-bit for FFT input.
    """
    path = os.path.join(VECTORS_DIR, f'legacy_{rate_mbps}mbps_waveform.json')
    with open(path) as f:
        data = json.load(f)

    re_float = data['real']
    im_float = data['imag']

    # Scale to 12-bit range (matching pipeline test)
    peak = max(max(abs(x) for x in re_float), max(abs(x) for x in im_float))
    scale = 2047.0 / peak if peak > 0 else 1.0

    # Quantize to 12-bit signed
    re_q = [to_s12(r * scale) for r in re_float]
    im_q = [to_s12(i * scale) for i in im_float]

    # Frame structure for 802.11a:
    # STF: samples 0-159 (160 samples)
    # GI2: samples 160-191 (32 samples, includes guard interval)
    # LTF1: samples 192-255 (64 samples)
    # LTF2: samples 256-319 (64 samples)
    # SIG_CP: samples 320-335 (16 samples, cyclic prefix)
    # SIG: samples 336-399 (64 samples)
    # DATA1_CP: samples 400-415 (16 samples)
    # DATA1: samples 416-479 (64 samples)
    # DATA2_CP: samples 480-495 (16 samples)
    # DATA2: samples 496-559 (64 samples)

    symbols = []
    offsets = [192, 256, 336, 416, 496]  # LTF1, LTF2, SIG, DATA1, DATA2
    names = ['LTF1', 'LTF2', 'SIG', 'DATA1', 'DATA2']

    for start in offsets:
        if start + 64 <= len(re_q):
            s_re = re_q[start:start+64]
            s_im = im_q[start:start+64]
            # Sign-extend 12-bit to 16-bit for FFT input
            s_re_16 = [((v & 0xFFF) - 4096 if v & 0x800 else v & 0xFFF) for v in s_re]
            s_im_16 = [((v & 0xFFF) - 4096 if v & 0x800 else v & 0xFFF) for v in s_im]
            symbols.append((s_re_16, s_im_16))
    
    return symbols, names[:len(symbols)]


async def reset_dut(dut):
    """Reset fft64_sdf."""
    dut.rst_n.value = 0
    dut.din_valid.value = 0
    dut.din_re.value = 0
    dut.din_im.value = 0
    dut.i_idx.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_frame_burst(dut, re_list, im_list, idx_offset=0):
    """Feed 64 samples at burst rate (1/clock). Returns outputs collected."""
    outs_re = []
    outs_im = []
    for i in range(64):
        dut.din_valid.value = 1
        dut.i_idx.value = (idx_offset + i) & 0x3F
        dut.din_re.value = re_list[i] & 0xFFFF
        dut.din_im.value = im_list[i] & 0xFFFF
        await RisingEdge(dut.clk)
        if int(dut.dout_valid.value) == 1:
            outs_re.append(to_s16(int(dut.dout_re.value)))
            outs_im.append(to_s16(int(dut.dout_im.value)))
    dut.din_valid.value = 0
    return outs_re, outs_im


async def feed_zeros_burst(dut, count, idx_offset=0):
    """Feed zeros at burst rate. Returns outputs collected."""
    outs_re = []
    outs_im = []
    for i in range(count):
        dut.din_valid.value = 1
        dut.i_idx.value = (idx_offset + i) & 0x3F
        dut.din_re.value = 0
        dut.din_im.value = 0
        await RisingEdge(dut.clk)
        if int(dut.dout_valid.value) == 1:
            outs_re.append(to_s16(int(dut.dout_re.value)))
            outs_im.append(to_s16(int(dut.dout_im.value)))
    dut.din_valid.value = 0
    return outs_re, outs_im


async def drain_outputs(dut, max_clocks=200):
    """Drain any remaining outputs (no new inputs)."""
    outs_re = []
    outs_im = []
    for _ in range(max_clocks):
        await RisingEdge(dut.clk)
        if int(dut.dout_valid.value) == 1:
            outs_re.append(to_s16(int(dut.dout_re.value)))
            outs_im.append(to_s16(int(dut.dout_im.value)))
    return outs_re, outs_im


@cocotb.test()
async def diag_continuous_vs_reference(dut):
    """Compare fft64_sdf continuous output against Python DIF reference.
    
    Feeds LTF1 + LTF2 + SIG + DATA1 + DATA2 + flush (6 frames) continuously.
    Pipeline fill consumes first 63 outputs. Remaining outputs are frame-aligned.
    Compares each frame's 64 bins against reference_fft64_dif().
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    symbols, names = load_ofdm_symbols(24)
    dut._log.info(f"Loaded {len(symbols)} OFDM symbols for rate 24 Mbps")
    dut._log.info(f"Symbol names: {names}")

    # Feed all symbols + 1 flush frame at burst rate (continuous, no reset)
    all_re = []
    all_im = []
    for s_re, s_im in symbols:
        all_re.extend(s_re)
        all_im.extend(s_im)
    
    # Flush frame (zeros to push last symbol out)
    flush_re = [0] * 64
    flush_im = [0] * 64
    all_re.extend(flush_re)
    all_im.extend(flush_re)

    total_samples = len(all_re)
    
    # Feed everything at burst rate
    outputs_re = []
    outputs_im = []
    for i in range(total_samples):
        dut.din_valid.value = 1
        dut.i_idx.value = i & 0x3F
        dut.din_re.value = all_re[i] & 0xFFFF
        dut.din_im.value = all_im[i] & 0xFFFF
        await RisingEdge(dut.clk)
        if int(dut.dout_valid.value) == 1:
            outputs_re.append(to_s16(int(dut.dout_re.value)))
            outputs_im.append(to_s16(int(dut.dout_im.value)))

    # Drain remaining outputs
    dut.din_valid.value = 0
    for _ in range(100):
        await RisingEdge(dut.clk)
        if int(dut.dout_valid.value) == 1:
            outputs_re.append(to_s16(int(dut.dout_re.value)))
            outputs_im.append(to_s16(int(dut.dout_im.value)))

    total_outs = len(outputs_re)
    dut._log.info(f"Total outputs: {total_outs} (fed {total_samples} samples)")
    # Expected: total_samples - 63 (pipeline fill)
    expected_outs = total_samples - 63
    dut._log.info(f"Expected outputs: {expected_outs}")

    # Output alignment:
    # outputs[0:64]   = FFT(frame 0) = LTF1
    # outputs[64:128] = FFT(frame 1) = LTF2
    # outputs[128:192]= FFT(frame 2) = SIG
    # outputs[192:256]= FFT(frame 3) = DATA1
    # outputs[256:320]= FFT(frame 4) = DATA2

    dut._log.info("=" * 70)
    dut._log.info("CONTINUOUS MODE: Error vs Python DIF reference (per symbol)")
    dut._log.info("=" * 70)

    for sym_idx in range(min(len(symbols), (total_outs - 0) // 64)):
        start = sym_idx * 64
        if start + 64 > total_outs:
            break

        hw_re = outputs_re[start:start+64]
        hw_im = outputs_im[start:start+64]
        
        s_re, s_im = symbols[sym_idx]
        ref_re, ref_im = reference_fft64_dif(s_re, s_im)

        errors_re = [abs(hw_re[i] - ref_re[i]) for i in range(64)]
        errors_im = [abs(hw_im[i] - ref_im[i]) for i in range(64)]
        max_err = max(max(errors_re), max(errors_im))
        rms_err = math.sqrt(sum(e*e for e in errors_re + errors_im) / 128)
        
        # Find worst bins
        worst_bins = sorted(range(64), key=lambda i: max(errors_re[i], errors_im[i]), reverse=True)[:5]
        
        name = names[sym_idx] if sym_idx < len(names) else f"frame{sym_idx}"
        dut._log.info(f"  {name}: max_err={max_err}, rms_err={rms_err:.1f}, "
                      f"worst bins={worst_bins[:3]}")
        
        # Print details for worst bins
        if max_err > 5:
            for b in worst_bins[:3]:
                dut._log.info(f"    bin {b}: hw=({hw_re[b]},{hw_im[b]}) "
                              f"ref=({ref_re[b]},{ref_im[b]}) "
                              f"err=({errors_re[b]},{errors_im[b]})")

    # Now test with RESET between each frame (simulating decode_engine per-symbol reset)
    dut._log.info("-")
    dut._log.info("=" * 70)
    dut._log.info("RESET-PER-FRAME MODE: Error vs Python DIF reference")
    dut._log.info("=" * 70)

    for sym_idx, (s_re, s_im) in enumerate(symbols):
        await reset_dut(dut)
        
        # Feed 64 samples + 128 zeros (enough to drain all 64 output bins)
        outs_re = []
        outs_im = []
        
        # Data
        for i in range(64):
            dut.din_valid.value = 1
            dut.i_idx.value = i & 0x3F
            dut.din_re.value = s_re[i] & 0xFFFF
            dut.din_im.value = s_im[i] & 0xFFFF
            await RisingEdge(dut.clk)
            if int(dut.dout_valid.value) == 1:
                outs_re.append(to_s16(int(dut.dout_re.value)))
                outs_im.append(to_s16(int(dut.dout_im.value)))
        
        # Zeros to flush
        for i in range(128):
            dut.din_valid.value = 1
            dut.i_idx.value = (64 + i) & 0x3F
            dut.din_re.value = 0
            dut.din_im.value = 0
            await RisingEdge(dut.clk)
            if int(dut.dout_valid.value) == 1:
                outs_re.append(to_s16(int(dut.dout_re.value)))
                outs_im.append(to_s16(int(dut.dout_im.value)))
        
        dut.din_valid.value = 0
        
        # After reset + 64 data + 128 zeros = 192 valid inputs,
        # pipeline produces 192-63 = 129 outputs. First 64 = our frame.
        if len(outs_re) < 64:
            dut._log.warning(f"  {names[sym_idx]}: only {len(outs_re)} outputs (need 64)")
            continue

        hw_re = outs_re[:64]
        hw_im = outs_im[:64]
        ref_re, ref_im = reference_fft64_dif(s_re, s_im)

        errors_re = [abs(hw_re[i] - ref_re[i]) for i in range(64)]
        errors_im = [abs(hw_im[i] - ref_im[i]) for i in range(64)]
        max_err = max(max(errors_re), max(errors_im))
        rms_err = math.sqrt(sum(e*e for e in errors_re + errors_im) / 128)
        
        name = names[sym_idx] if sym_idx < len(names) else f"frame{sym_idx}"
        dut._log.info(f"  {name}: max_err={max_err}, rms_err={rms_err:.1f}")
        
        if max_err > 5:
            worst_bins = sorted(range(64), key=lambda i: max(errors_re[i], errors_im[i]), reverse=True)[:3]
            for b in worst_bins:
                dut._log.info(f"    bin {b}: hw=({hw_re[b]},{hw_im[b]}) "
                              f"ref=({ref_re[b]},{ref_im[b]}) "
                              f"err=({errors_re[b]},{errors_im[b]})")

    dut._log.info("-")
    dut._log.info("=" * 70)
    dut._log.info("CONCLUSION")
    dut._log.info("=" * 70)
    dut._log.info("If CONTINUOUS errors >> RESET errors for DATA symbols,")
    dut._log.info("the streaming pipeline has inter-symbol contamination.")
    dut._log.info("If both are similar, the issue is in the reference model")
    dut._log.info("(RTL twiddle quantization vs reference) — fix the reference.")
