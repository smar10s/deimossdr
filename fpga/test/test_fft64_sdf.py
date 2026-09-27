"""
Test 64-point R2²SDF FFT pipeline (fft64_sdf).

Tests:
1. DC input (all 100+j0) → bin 0 = 6400, bins 1..63 ≈ 0
2. Single tone at bin 8 → peak at bin 8 (after bit-reversal reordering)
3. LTF golden vector → compare against DIF fixed-point reference model
4. Linearity: 2× input → 2× output (±6 LSB)
5. Back-to-back: two different transforms, both correct
6. Pipeline latency: first dout_valid at expected clock
7. Cycle count: all 64 outputs in exactly 64 consecutive valid clocks
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


def bit_reverse(n, bits=6):
    result = 0
    for _ in range(bits):
        result = (result << 1) | (n & 1)
        n >>= 1
    return result


def reference_fft64_dif(x_re, x_im):
    """Radix-2 DIF FFT, 64-point, Q1.14 quantized twiddles, 16-bit signed.

    Input: x_re, x_im — 64 signed 16-bit integers, natural order.
    Output: out_re, out_im — 64 signed 16-bit integers, BIT-REVERSED order.
    """
    N = 64
    FRAC = 14
    SCALE = 1 << FRAC   # 16384
    ROUND = 1 << (FRAC - 1)  # 8192

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

                # Top output: a + b
                new_re[idx_top] = to_s16(a_re + b_re)
                new_im[idx_top] = to_s16(a_im + b_im)

                # Bottom output: (a - b) * W_N^(j * num_blocks)
                diff_re = to_s16(a_re - b_re)
                diff_im = to_s16(a_im - b_im)

                tw_exp = (j * num_blocks) % N

                if tw_exp == 0:
                    new_re[idx_bot] = diff_re
                    new_im[idx_bot] = diff_im
                elif tw_exp == N // 4:      # W^16 = -j
                    new_re[idx_bot] = diff_im
                    new_im[idx_bot] = to_s16(-diff_re)
                elif tw_exp == N // 2:      # W^32 = -1
                    new_re[idx_bot] = to_s16(-diff_re)
                    new_im[idx_bot] = to_s16(-diff_im)
                elif tw_exp == 3 * N // 4:  # W^48 = +j
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


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.gate_rst_n.value = 0
    dut.din_valid.value = 0
    dut.din_re.value = 0
    dut.din_im.value = 0
    dut.i_idx.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    dut.gate_rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def run_fft_sdf(dut, x_re, x_im):
    """Feed 64 samples + zeros, collect 64 outputs."""
    assert len(x_re) == 64 and len(x_im) == 64

    outputs_re = []
    outputs_im = []
    outputs_idx = []

    # Feed 64 data samples then zeros until we get 64 outputs
    for clk_num in range(200):
        dut.din_valid.value = 1
        dut.i_idx.value = clk_num & 0x3F  # 6-bit, wraps at 64
        if clk_num < 64:
            dut.din_re.value = x_re[clk_num] & 0xFFFF
            dut.din_im.value = x_im[clk_num] & 0xFFFF
        else:
            dut.din_re.value = 0
            dut.din_im.value = 0

        await RisingEdge(dut.clk)

        if int(dut.dout_valid.value) == 1:
            outputs_re.append(to_s16(int(dut.dout_re.value)))
            outputs_im.append(to_s16(int(dut.dout_im.value)))
            outputs_idx.append(int(dut.dout_idx.value))

        if len(outputs_re) == 64:
            break

    return outputs_re, outputs_im, outputs_idx


@cocotb.test()
async def test_dc_input(dut):
    """DC input (100+j0) → bin 0 = 6400, all others ≈ 0."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [100] * 64
    x_im = [0] * 64

    out_re, out_im, out_idx = await run_fft_sdf(dut, x_re, x_im)
    assert len(out_re) == 64, f"Expected 64 outputs, got {len(out_re)}"

    # Get reference (bit-reversed output order)
    ref_re, ref_im = reference_fft64_dif(x_re, x_im)

    # Convert both to natural order for comparison
    nat_out_re = [0] * 64
    nat_out_im = [0] * 64
    nat_ref_re = [0] * 64
    nat_ref_im = [0] * 64
    for i in range(64):
        nat_ref_re[bit_reverse(i)] = ref_re[i]
        nat_ref_im[bit_reverse(i)] = ref_im[i]
        nat_out_re[bit_reverse(i)] = out_re[i]
        nat_out_im[bit_reverse(i)] = out_im[i]

    dut._log.info(f"Bin 0: RTL=({nat_out_re[0]}, {nat_out_im[0]}), "
                  f"ref=({nat_ref_re[0]}, {nat_ref_im[0]})")

    # Bin 0 should be 6400, all others ~0
    assert abs(nat_out_re[0] - 6400) <= 3, f"Bin 0 re: {nat_out_re[0]}, expected 6400"
    assert abs(nat_out_im[0]) <= 3, f"Bin 0 im: {nat_out_im[0]}, expected 0"

    max_err = 0
    for i in range(64):
        err_re = abs(nat_out_re[i] - nat_ref_re[i])
        err_im = abs(nat_out_im[i] - nat_ref_im[i])
        max_err = max(max_err, err_re, err_im)

    dut._log.info(f"DC test max error vs DIF reference: {max_err}")
    assert max_err <= 3, f"DC test failed: max error {max_err} > 3"


@cocotb.test()
async def test_single_tone(dut):
    """Tone at bin 8 → peak at bin 8."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [int(round(500 * math.cos(2 * math.pi * 8 * n / 64))) for n in range(64)]
    x_im = [int(round(500 * math.sin(2 * math.pi * 8 * n / 64))) for n in range(64)]

    out_re, out_im, _ = await run_fft_sdf(dut, x_re, x_im)
    assert len(out_re) == 64

    ref_re, ref_im = reference_fft64_dif(x_re, x_im)

    # Convert to natural order
    nat_out_re = [0] * 64
    nat_out_im = [0] * 64
    for i in range(64):
        nat_out_re[bit_reverse(i)] = out_re[i]
        nat_out_im[bit_reverse(i)] = out_im[i]

    # Find peak
    magnitudes = [abs(complex(nat_out_re[i], nat_out_im[i])) for i in range(64)]
    peak_bin = magnitudes.index(max(magnitudes))
    dut._log.info(f"Tone test: peak at bin {peak_bin}, magnitude = {magnitudes[peak_bin]:.0f}")
    assert peak_bin == 8, f"Peak at bin {peak_bin}, expected 8"

    # Check error vs reference
    max_err = 0
    for i in range(64):
        nat_ref_re = ref_re[bit_reverse(i, 6) if False else i]  # ref is already bit-reversed
        # Actually ref output is in bit-reversed order, so ref[i] corresponds to natural bin bit_reverse(i)
        pass

    # Simpler: compare directly in output order (both RTL and ref are bit-reversed)
    for i in range(64):
        err_re = abs(out_re[i] - ref_re[i])
        err_im = abs(out_im[i] - ref_im[i])
        max_err = max(max_err, err_re, err_im)

    dut._log.info(f"Tone test max error vs DIF reference: {max_err}")
    assert max_err <= 10, f"Tone test failed: max error {max_err} > 10"


@cocotb.test()
async def test_ltf_golden(dut):
    """LTF golden vector — FFT matches DIF reference."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS_DIR, "annex_i1_ltf_time.json")) as f:
        ltf_data = json.load(f)

    ltf1 = ltf_data["samples"][33:97]
    assert len(ltf1) == 64

    SCALE = 8000
    x_re = [max(-32768, min(32767, int(round(s[0] * SCALE)))) for s in ltf1]
    x_im = [max(-32768, min(32767, int(round(s[1] * SCALE)))) for s in ltf1]

    out_re, out_im, _ = await run_fft_sdf(dut, x_re, x_im)
    assert len(out_re) == 64, f"Got {len(out_re)} outputs"

    ref_re, ref_im = reference_fft64_dif(x_re, x_im)

    max_err = 0
    total_err = 0
    for i in range(64):
        err_re = abs(out_re[i] - ref_re[i])
        err_im = abs(out_im[i] - ref_im[i])
        max_err = max(max_err, err_re, err_im)
        total_err += err_re + err_im

    dut._log.info(f"LTF test: max error = {max_err}, total error = {total_err}")
    assert max_err <= 10, f"LTF test failed: max error {max_err} > 10"


@cocotb.test()
async def test_linearity(dut):
    """2× input → 2× output (±6 LSB)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re_1x = [int(round(200 * math.cos(2*math.pi*5*n/64) +
                          100 * math.cos(2*math.pi*13*n/64))) for n in range(64)]
    x_im_1x = [int(round(200 * math.sin(2*math.pi*5*n/64) +
                          100 * math.sin(2*math.pi*13*n/64))) for n in range(64)]

    out1_re, out1_im, _ = await run_fft_sdf(dut, x_re_1x, x_im_1x)

    # Need to reset pipeline state for second transform
    await reset_dut(dut)

    x_re_2x = [v * 2 for v in x_re_1x]
    x_im_2x = [v * 2 for v in x_im_1x]

    out2_re, out2_im, _ = await run_fft_sdf(dut, x_re_2x, x_im_2x)

    max_err = 0
    for i in range(64):
        exp_re = to_s16(out1_re[i] * 2)
        exp_im = to_s16(out1_im[i] * 2)
        err_re = abs(out2_re[i] - exp_re)
        err_im = abs(out2_im[i] - exp_im)
        max_err = max(max_err, err_re, err_im)

    dut._log.info(f"Linearity test max error: {max_err}")
    assert max_err <= 6, f"Linearity test failed: max error {max_err} > 6"


@cocotb.test()
async def test_back_to_back(dut):
    """Two transforms back-to-back (reset between)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # First: tone at bin 3
    x1_re = [int(round(300 * math.cos(2*math.pi*3*n/64))) for n in range(64)]
    x1_im = [int(round(300 * math.sin(2*math.pi*3*n/64))) for n in range(64)]

    out1_re, out1_im, _ = await run_fft_sdf(dut, x1_re, x1_im)
    ref1_re, ref1_im = reference_fft64_dif(x1_re, x1_im)

    # Reset for second
    await reset_dut(dut)

    # Second: tone at bin 20
    x2_re = [int(round(400 * math.cos(2*math.pi*20*n/64))) for n in range(64)]
    x2_im = [int(round(400 * math.sin(2*math.pi*20*n/64))) for n in range(64)]

    out2_re, out2_im, _ = await run_fft_sdf(dut, x2_re, x2_im)
    ref2_re, ref2_im = reference_fft64_dif(x2_re, x2_im)

    # Verify first
    max_err1 = max(max(abs(out1_re[i] - ref1_re[i]) for i in range(64)),
                   max(abs(out1_im[i] - ref1_im[i]) for i in range(64)))
    dut._log.info(f"Back-to-back 1: max error = {max_err1}")
    assert max_err1 <= 10

    # Verify second
    max_err2 = max(max(abs(out2_re[i] - ref2_re[i]) for i in range(64)),
                   max(abs(out2_im[i] - ref2_im[i]) for i in range(64)))
    dut._log.info(f"Back-to-back 2: max error = {max_err2}")
    assert max_err2 <= 10

    # Check peaks in natural order
    nat1_re = [out1_re[bit_reverse(i)] for i in range(64)]
    nat1_im = [out1_im[bit_reverse(i)] for i in range(64)]
    mag1 = [abs(complex(nat1_re[i], nat1_im[i])) for i in range(64)]
    assert mag1.index(max(mag1)) == 3, f"First peak at {mag1.index(max(mag1))}"

    nat2_re = [out2_re[bit_reverse(i)] for i in range(64)]
    nat2_im = [out2_im[bit_reverse(i)] for i in range(64)]
    mag2 = [abs(complex(nat2_re[i], nat2_im[i])) for i in range(64)]
    assert mag2.index(max(mag2)) == 20, f"Second peak at {mag2.index(max(mag2))}"


@cocotb.test()
async def test_latency_and_count(dut):
    """First dout_valid at expected clock; 64 consecutive outputs."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [100] * 64
    x_im = [0] * 64

    first_valid_clk = None
    last_valid_clk = None
    output_count = 0

    for clk_num in range(200):
        dut.din_valid.value = 1
        dut.i_idx.value = clk_num & 0x3F
        if clk_num < 64:
            dut.din_re.value = x_re[clk_num] & 0xFFFF
            dut.din_im.value = x_im[clk_num] & 0xFFFF
        else:
            dut.din_re.value = 0
            dut.din_im.value = 0

        await RisingEdge(dut.clk)

        if int(dut.dout_valid.value) == 1:
            if first_valid_clk is None:
                first_valid_clk = clk_num
            last_valid_clk = clk_num
            output_count += 1

        if output_count == 64:
            break

    dut._log.info(f"First output at clock {first_valid_clk}, last at {last_valid_clk}")
    dut._log.info(f"Total outputs: {output_count}")
    dut._log.info(f"Output span: {last_valid_clk - first_valid_clk + 1} clocks")

    assert output_count == 64, f"Expected 64 outputs, got {output_count}"
    # Outputs should be consecutive (64 in exactly 64 clocks)
    assert last_valid_clk - first_valid_clk == 63, \
        f"Outputs not consecutive: span = {last_valid_clk - first_valid_clk + 1}"

    # Pipeline latency (LIMPIX31 port): ~69-80 clocks from first input
    # Allow generous tolerance for the implementation
    assert first_valid_clk >= 60 and first_valid_clk <= 90, \
        f"First output at clock {first_valid_clk}, expected ~69-80"


@cocotb.test()
async def test_back_to_back_short_reset(dut):
    """Two identical transforms with only 2-clock reset between (mimics decode_engine)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Use same amplitude as test_back_to_back (300)
    x_re = [int(round(300 * math.cos(2*math.pi*3*n/64))) for n in range(64)]
    x_im = [int(round(300 * math.sin(2*math.pi*3*n/64))) for n in range(64)]

    # First transform (with long reset from reset_dut)
    out1_re, out1_im, _ = await run_fft_sdf(dut, x_re, x_im)
    ref_re, ref_im = reference_fft64_dif(x_re, x_im)
    max_err1 = max(max(abs(out1_re[i] - ref_re[i]) for i in range(64)),
                   max(abs(out1_im[i] - ref_im[i]) for i in range(64)))
    dut._log.info(f"First transform: max error = {max_err1}")
    assert max_err1 <= 10, f"First transform failed: max_err={max_err1}"

    # SHORT reset: only 2 clocks (like decode_engine between LTF1 and LTF2)
    dut.rst_n.value = 0
    dut.gate_rst_n.value = 0
    dut.din_valid.value = 0
    await ClockCycles(dut.clk, 2)
    dut.rst_n.value = 1
    dut.gate_rst_n.value = 1
    # NO idle clocks after reset release — immediately start feeding

    # Second transform (same data)
    out2_re, out2_im, _ = await run_fft_sdf(dut, x_re, x_im)
    max_err2 = max(max(abs(out2_re[i] - ref_re[i]) for i in range(64)),
                   max(abs(out2_im[i] - ref_im[i]) for i in range(64)))
    dut._log.info(f"Second transform (2-clk reset): max error = {max_err2}")
    assert max_err2 <= 10, f"Second transform failed: max_err={max_err2}"
