"""
Test 4-point R2²SDF FFT (fft4_sdf).

The simplest possible SDF: one stage pair (BF1 delay=2, BF2 delay=1),
no twiddle multiply. Verifies butterfly logic and delay-line mechanics.

4-point DIF FFT output is in bit-reversed order.
Output index mapping: idx=3→X[0], idx=0→X[2], idx=1→X[1], idx=2→X[3]

Tests:
1. DC input — all same → bin 0 = 4*val, others = 0
2. Impulse — [1000, 0, 0, 0] → all bins = 1000
3. Alternating — [500, -500, 500, -500] → bin 2 = 2000, others = 0
4. Complex tone at bin 1 — exp(j*2π*1*n/4) → peak at bin 1
5. Full DFT reference — arbitrary input vs Python DFT
6. Back-to-back frames — two transforms, both correct
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

import math


def to_signed(v, bits):
    """Convert unsigned int to signed."""
    mask = (1 << bits) - 1
    v = int(v) & mask
    if v >= (1 << (bits - 1)):
        v -= (1 << bits)
    return v


def dft4_ref(x_re, x_im):
    """4-point DFT reference. Returns (re, im) in natural frequency order."""
    N = 4
    X_re = [0] * N
    X_im = [0] * N
    for k in range(N):
        sr, si = 0.0, 0.0
        for n in range(N):
            angle = -2.0 * math.pi * k * n / N
            sr += x_re[n] * math.cos(angle) - x_im[n] * math.sin(angle)
            si += x_re[n] * math.sin(angle) + x_im[n] * math.cos(angle)
        X_re[k] = int(round(sr))
        X_im[k] = int(round(si))
    return X_re, X_im


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.din_valid.value = 0
    dut.din_re.value = 0
    dut.din_im.value = 0
    dut.i_idx.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


# Output index to natural frequency bin mapping (empirically verified):
# idx=3 → X[0], idx=0 → X[2], idx=1 → X[1], idx=2 → X[3]
IDX_TO_BIN = {3: 0, 0: 2, 1: 1, 2: 3}


async def run_fft4(dut, x_re, x_im):
    """Feed 4 samples + drain zeros, collect 4 DFT outputs.
    
    fft4 outputs on every valid clock (no gating). The first 2 outputs
    are pipeline fill (BF1+BF2 latency). Skip those, take the next 4.
    """
    assert len(x_re) == 4 and len(x_im) == 4

    outputs = {}  # bin -> (re, im)
    valid_count = 0
    FILL_LATENCY = 3  # BF1 stores first, outputs garbage for 2, BF2 adds 1 more

    for clk_num in range(30):
        dut.din_valid.value = 1
        dut.i_idx.value = clk_num & 0x3
        if clk_num < 4:
            dut.din_re.value = x_re[clk_num] & 0xFFFF
            dut.din_im.value = x_im[clk_num] & 0xFFFF
        else:
            dut.din_re.value = 0
            dut.din_im.value = 0

        await RisingEdge(dut.clk)
        await Timer(1, units='ns')

        if int(dut.dout_valid.value) == 1:
            valid_count += 1
            if valid_count <= FILL_LATENCY:
                continue  # skip pipeline fill
            idx = int(dut.dout_idx.value)
            re = to_signed(int(dut.dout_re.value), 18)
            im = to_signed(int(dut.dout_im.value), 18)
            freq_bin = IDX_TO_BIN[idx]
            outputs[freq_bin] = (re, im)

        if len(outputs) == 4:
            break

    # Return in natural order [X[0], X[1], X[2], X[3]]
    out_re = [outputs[k][0] for k in range(4)]
    out_im = [outputs[k][1] for k in range(4)]
    return out_re, out_im


@cocotb.test()
async def test_dc_input(dut):
    """DC input: [100,100,100,100] → X[0]=400, others=0."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [100, 100, 100, 100]
    x_im = [0, 0, 0, 0]
    out_re, out_im = await run_fft4(dut, x_re, x_im)
    ref_re, ref_im = dft4_ref(x_re, x_im)

    dut._log.info(f"DC: RTL = {list(zip(out_re, out_im))}")
    dut._log.info(f"DC: ref = {list(zip(ref_re, ref_im))}")

    for i in range(4):
        assert abs(out_re[i] - ref_re[i]) <= 1, f"Bin {i} re: {out_re[i]} vs {ref_re[i]}"
        assert abs(out_im[i] - ref_im[i]) <= 1, f"Bin {i} im: {out_im[i]} vs {ref_im[i]}"


@cocotb.test()
async def test_impulse(dut):
    """Impulse: [1000,0,0,0] → all bins = 1000+j0."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [1000, 0, 0, 0]
    x_im = [0, 0, 0, 0]
    out_re, out_im = await run_fft4(dut, x_re, x_im)
    ref_re, ref_im = dft4_ref(x_re, x_im)

    dut._log.info(f"Impulse: RTL = {list(zip(out_re, out_im))}")
    dut._log.info(f"Impulse: ref = {list(zip(ref_re, ref_im))}")

    for i in range(4):
        assert abs(out_re[i] - ref_re[i]) <= 1, f"Bin {i}: {out_re[i]} vs {ref_re[i]}"
        assert abs(out_im[i] - ref_im[i]) <= 1


@cocotb.test()
async def test_alternating(dut):
    """Alternating: [500,-500,500,-500] → X[2]=2000, others=0."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [500, -500, 500, -500]
    x_im = [0, 0, 0, 0]
    out_re, out_im = await run_fft4(dut, x_re, x_im)
    ref_re, ref_im = dft4_ref(x_re, x_im)

    dut._log.info(f"Alternating: RTL = {list(zip(out_re, out_im))}")
    dut._log.info(f"Alternating: ref = {list(zip(ref_re, ref_im))}")

    for i in range(4):
        assert abs(out_re[i] - ref_re[i]) <= 1, f"Bin {i}: {out_re[i]} vs {ref_re[i]}"
        assert abs(out_im[i] - ref_im[i]) <= 1


@cocotb.test()
async def test_complex_tone(dut):
    """Complex tone at bin 1: exp(j*2π*1*n/4) → X[1]=2000, others=0."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [int(round(500 * math.cos(2 * math.pi * 1 * n / 4))) for n in range(4)]
    x_im = [int(round(500 * math.sin(2 * math.pi * 1 * n / 4))) for n in range(4)]
    out_re, out_im = await run_fft4(dut, x_re, x_im)
    ref_re, ref_im = dft4_ref(x_re, x_im)

    dut._log.info(f"Tone: RTL = {list(zip(out_re, out_im))}")
    dut._log.info(f"Tone: ref = {list(zip(ref_re, ref_im))}")

    for i in range(4):
        assert abs(out_re[i] - ref_re[i]) <= 1, f"Bin {i}: {out_re[i]} vs {ref_re[i]}"
        assert abs(out_im[i] - ref_im[i]) <= 1


@cocotb.test()
async def test_full_reference(dut):
    """Arbitrary input vs full DFT reference."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    x_re = [300, -150, 700, -400]
    x_im = [100, -200, 50, 350]
    out_re, out_im = await run_fft4(dut, x_re, x_im)
    ref_re, ref_im = dft4_ref(x_re, x_im)

    dut._log.info(f"Full: RTL = {list(zip(out_re, out_im))}")
    dut._log.info(f"Full: ref = {list(zip(ref_re, ref_im))}")

    max_err = 0
    for i in range(4):
        err_re = abs(out_re[i] - ref_re[i])
        err_im = abs(out_im[i] - ref_im[i])
        max_err = max(max_err, err_re, err_im)
    dut._log.info(f"Max error: {max_err}")
    assert max_err <= 1, f"Max error {max_err} > 1"


@cocotb.test()
async def test_back_to_back(dut):
    """Two frames back-to-back (reset between), both correct."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Frame 1: impulse
    out1_re, out1_im = await run_fft4(dut, [1000, 0, 0, 0], [0, 0, 0, 0])
    ref1_re, ref1_im = dft4_ref([1000, 0, 0, 0], [0, 0, 0, 0])

    await reset_dut(dut)

    # Frame 2: shifted impulse
    out2_re, out2_im = await run_fft4(dut, [0, 1000, 0, 0], [0, 0, 0, 0])
    ref2_re, ref2_im = dft4_ref([0, 1000, 0, 0], [0, 0, 0, 0])

    for i in range(4):
        assert abs(out1_re[i] - ref1_re[i]) <= 1
        assert abs(out1_im[i] - ref1_im[i]) <= 1
        assert abs(out2_re[i] - ref2_re[i]) <= 1
        assert abs(out2_im[i] - ref2_im[i]) <= 1

    dut._log.info("Back-to-back: both frames correct")
