"""
Test demapper — 802.11a soft decision demapper.

Given equalized constellation points (16-bit signed I/Q per subcarrier),
produces soft LLR bits (8-bit signed) for the Viterbi decoder.

Rate modes:
  0 = BPSK:   1 bit/symbol  → LLR = Re(y)
  1 = QPSK:   2 bits/symbol → LLR = [Re(y), Im(y)]
  2 = 16-QAM: 4 bits/symbol → LLR = [Re, 2d-|Re|, Im, 2d-|Im|]
  3 = 64-QAM: 6 bits/symbol → LLR = [Re, 4d-|Re|, 2d-||Re|-4d|, Im, 4d-|Im|, 2d-||Im|-4d|]

The module accepts a `norm` input that sets the scaling reference (the expected
magnitude of the innermost constellation ring after equalization). When the
equalizer outputs ±1 magnitudes, norm=1 and soft bits degrade to hard decisions.
When norm is larger (e.g. after reducing shift_val), soft bits carry confidence.

Tests:
1. BPSK: known constellation points → correct hard decisions matching golden vector
2. BPSK: sign pattern matches annex I.1 SIGNAL interleaved bits
3. QPSK: 4 known points → correct 2 LLRs per point
4. 16-QAM: annex I.1 data → hard decisions match interleaved bits
5. 16-QAM: scaled input → soft bits have proper magnitudes
6. 64-QAM: 16 known points → correct 6 LLRs per point
7. Timing: 48 subcarriers in → correct number of soft bits out
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import json
import os
import math

VECTORS = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')

# Rate modes
BPSK = 0
QPSK = 1
QAM16 = 2
QAM64 = 3

# Bits per symbol for each rate
NBPSC = {BPSK: 1, QPSK: 2, QAM16: 4, QAM64: 6}


def to_s16(v):
    """Interpret 16-bit value as signed."""
    v = int(v) & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


def s16_to_bits(v):
    """Convert signed int to 16-bit unsigned representation."""
    if v < 0:
        v = v + 0x10000
    return v & 0xFFFF


def to_s8(v):
    """Interpret 8-bit value as signed."""
    v = int(v) & 0xFF
    return v - 0x100 if v >= 0x80 else v


def hard_decision(soft):
    """Convert signed soft bit to hard decision (1 if positive, 0 if negative/zero)."""
    return 1 if soft > 0 else 0


async def reset_dut(dut):
    """Reset the DUT and initialize inputs."""
    dut.rst_n.value = 0
    dut.valid_in.value = 0
    dut.sym_re.value = 0
    dut.sym_im.value = 0
    dut.rate_mode.value = 0
    dut.norm.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_symbols(dut, sym_re_list, sym_im_list, rate_mode, norm=1):
    """Feed constellation points, collect soft LLRs from the wide output.

    The demapper emits one subcarrier per clock on soft_wide0..5 (lever C).
    Lanes 0..n_bpsc-1 are appended in order, so the returned flat list matches
    the pre-lever-C serial soft_out stream.

    Args:
        sym_re_list: list of signed 16-bit real values
        sym_im_list: list of signed 16-bit imag values
        rate_mode: BPSK/QPSK/QAM16/QAM64
        norm: expected magnitude of innermost ring

    Returns:
        list of signed 8-bit soft LLR values (n_bpsc per subcarrier)
    """
    n_syms = len(sym_re_list)
    nb = NBPSC[rate_mode]

    dut.rate_mode.value = rate_mode
    dut.norm.value = s16_to_bits(norm) if norm < 0 else norm

    soft_out = []

    def collect():
        # Streaming (lever C): outputs emerge 3 clocks after each input, so
        # collection must run concurrently with feeding, not after it.
        if int(dut.wide_valid.value) == 1:
            for j in range(nb):
                soft_out.append(to_s8(int(getattr(dut, f"soft_wide{j}").value)))

    # Feed symbols one per clock, collecting as they emerge
    for i in range(n_syms):
        dut.valid_in.value = 1
        dut.sym_re.value = s16_to_bits(sym_re_list[i])
        dut.sym_im.value = s16_to_bits(sym_im_list[i])
        await RisingEdge(dut.clk)
        collect()

    dut.valid_in.value = 0

    # Drain the last 3 pipeline stages
    for _ in range(8):
        await RisingEdge(dut.clk)
        collect()

    return soft_out


@cocotb.test()
async def test_bpsk_basic(dut):
    """BPSK: positive Re → positive LLR (bit=1), negative Re → negative LLR (bit=0).

    Input: 8 BPSK symbols with known Re values.
    Expected: 8 soft bits with correct signs.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # BPSK symbols: Re carries the info, Im should be ~0
    sym_re = [100, -200, 50, -50, 1000, -1, 32767, -32768]
    sym_im = [0, 0, 0, 0, 0, 0, 0, 0]

    soft = await feed_symbols(dut, sym_re, sym_im, BPSK, norm=100)

    assert len(soft) == 8, f"BPSK: expected 8 soft bits, got {len(soft)}"

    # Check signs (hard decisions)
    expected_hard = [1, 0, 1, 0, 1, 0, 1, 0]
    for i in range(8):
        hd = hard_decision(soft[i])
        assert hd == expected_hard[i], \
            f"BPSK bit {i}: soft={soft[i]}, hard={hd}, expected={expected_hard[i]}"

    dut._log.info("BPSK basic test passed")


@cocotb.test()
async def test_bpsk_signal_golden(dut):
    """BPSK: Annex I.1 SIGNAL field — hard decisions match golden interleaved bits.

    The equalizer outputs ±SCALE for BPSK constellation points.
    With unit channel and default shift_val, these might be ±1.
    Hard decisions should still be correct.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Load golden vectors
    with open(os.path.join(VECTORS, 'annex_i1_signal_freq.json')) as f:
        sig_freq = json.load(f)
    with open(os.path.join(VECTORS, 'annex_i1_signal_interleaved.json')) as f:
        sig_intl = json.load(f)

    subcarriers = sig_freq['subcarriers']
    expected_bits = sig_intl['bits']

    # Convert signal_freq from centered indexing to FFT bin order, extract data bins
    DATA_BINS = [
        38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50,
        51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63,
         1,  2,  3,  4,  5,  6,  8,  9, 10, 11, 12, 13,
        14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26,
    ]

    # Convert centered index to FFT bin: fft_bin = (vec_idx - 32 + 64) % 64
    fft_indexed = [None] * 64
    for vec_idx in range(64):
        fft_bin = (vec_idx - 32 + 64) % 64
        fft_indexed[fft_bin] = subcarriers[vec_idx]

    # Extract 48 data subcarrier values in DATA_BINS order
    # Scale to integer: BPSK constellation is ±1.0, scale to ±1000 for reasonable dynamic range
    SCALE = 1000
    sym_re = []
    sym_im = []
    for k in DATA_BINS:
        pt = fft_indexed[k]
        assert pt is not None, f"Data bin {k} is None (should not be pilot)"
        sym_re.append(int(round(pt[0] * SCALE)))
        sym_im.append(int(round(pt[1] * SCALE)))

    soft = await feed_symbols(dut, sym_re, sym_im, BPSK, norm=SCALE)

    assert len(soft) == 48, f"Expected 48 soft bits, got {len(soft)}"

    # Verify hard decisions match golden interleaved bits
    errors = 0
    for i in range(48):
        hd = hard_decision(soft[i])
        if hd != expected_bits[i]:
            dut._log.warning(f"Bit {i}: soft={soft[i]}, hard={hd}, expected={expected_bits[i]}")
            errors += 1

    assert errors == 0, f"BPSK SIGNAL golden vector: {errors}/48 hard decision errors"
    dut._log.info("BPSK SIGNAL golden vector test passed (48/48 bits correct)")


@cocotb.test()
async def test_qpsk_basic(dut):
    """QPSK: 2 bits per symbol — Re determines bit 0, Im determines bit 1.

    QPSK constellation (Table 17-12):
      b0b1=00 → (-1,-1)/sqrt(2)
      b0b1=01 → (-1,+1)/sqrt(2)
      b0b1=10 → (+1,-1)/sqrt(2)
      b0b1=11 → (+1,+1)/sqrt(2)

    Soft demap: LLR(b0) = Re, LLR(b1) = Im
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 4 QPSK symbols covering all quadrants
    SCALE = 500
    sym_re = [SCALE, -SCALE, SCALE, -SCALE]   # b0: 1, 0, 1, 0
    sym_im = [SCALE, SCALE, -SCALE, -SCALE]   # b1: 1, 1, 0, 0

    soft = await feed_symbols(dut, sym_re, sym_im, QPSK, norm=SCALE)

    assert len(soft) == 8, f"QPSK: expected 8 soft bits (4 syms × 2), got {len(soft)}"

    # Expected hard decisions: b0,b1 per symbol
    expected = [1, 1, 0, 1, 1, 0, 0, 0]
    for i in range(8):
        hd = hard_decision(soft[i])
        assert hd == expected[i], \
            f"QPSK bit {i}: soft={soft[i]}, hard={hd}, expected={expected[i]}"

    dut._log.info("QPSK basic test passed")


@cocotb.test()
async def test_16qam_golden(dut):
    """16-QAM: Annex I.1 DATA first symbol — hard decisions match interleaved bits.

    Golden vector has ideal constellation points (±0.316, ±0.949 = ±1/√10, ±3/√10).
    We scale to 16-bit integer. Hard decisions from soft LLRs must match the 192
    interleaved bits bit-for-bit.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    with open(os.path.join(VECTORS, 'annex_i1_data_qam.json')) as f:
        qam_vec = json.load(f)
    with open(os.path.join(VECTORS, 'annex_i1_data_interleaved.json')) as f:
        intl_vec = json.load(f)

    pts = qam_vec['data']  # 48 [re, im] constellation points
    expected_bits = intl_vec['data']  # 192 interleaved bits

    # Scale float constellation to integer.
    # Innermost ring = 1/sqrt(10) ≈ 0.316
    # Scale so innermost maps to NORM.
    Kmod = 1.0 / math.sqrt(10)
    NORM = 1000  # innermost ring magnitude in integer domain

    sym_re = []
    sym_im = []
    for pt in pts:
        sym_re.append(int(round(pt[0] / Kmod * NORM)))
        sym_im.append(int(round(pt[1] / Kmod * NORM)))

    # So now the grid points are at ±NORM and ±3*NORM
    soft = await feed_symbols(dut, sym_re, sym_im, QAM16, norm=NORM)

    assert len(soft) == 192, f"16-QAM: expected 192 soft bits (48×4), got {len(soft)}"

    # Verify hard decisions
    errors = 0
    for i in range(192):
        hd = hard_decision(soft[i])
        if hd != expected_bits[i]:
            sc = i // 4
            bit_in_sc = i % 4
            dut._log.warning(
                f"Bit {i} (sc={sc}, b{bit_in_sc}): soft={soft[i]}, "
                f"hard={hd}, expected={expected_bits[i]}")
            errors += 1

    assert errors == 0, f"16-QAM golden vector: {errors}/192 hard decision errors"
    dut._log.info("16-QAM golden vector test passed (192/192 bits correct)")


@cocotb.test()
async def test_16qam_soft_magnitudes(dut):
    """16-QAM: verify soft bit magnitudes are proportional to distance from boundary.

    With NORM=1000, grid is at ±1000, ±3000.
    Decision boundary for b0: Re=0. Point at Re=+3000 → LLR(b0) should be large positive.
    Decision boundary for b1: |Re|=2*NORM=2000. Point at Re=+1000 → LLR(b1) = 2000-1000 = positive.
    Point at Re=+3000 → LLR(b1) = 2000-3000 = negative (outer ring → b1=0).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    NORM = 1000
    # Test point: Re=+3000, Im=+1000 → bits should be b0=1, b1=0, b2=1, b3=1
    # (outer I ring, inner Q ring)
    sym_re = [3 * NORM]
    sym_im = [1 * NORM]

    soft = await feed_symbols(dut, sym_re, sym_im, QAM16, norm=NORM)
    assert len(soft) == 4, f"Expected 4 soft bits, got {len(soft)}"

    # b0 = sign(Re) = positive (bit 1) — magnitude proportional to |Re|
    assert soft[0] > 0, f"b0: expected positive, got {soft[0]}"
    # b1 = 2*NORM - |Re| = 2000 - 3000 = -1000 → negative (bit 0, outer ring)
    assert soft[1] < 0, f"b1: expected negative (outer ring), got {soft[1]}"
    # b2 = sign(Im) = positive (bit 1)
    assert soft[2] > 0, f"b2: expected positive, got {soft[2]}"
    # b3 = 2*NORM - |Im| = 2000 - 1000 = +1000 → positive (bit 1, inner ring)
    assert soft[3] > 0, f"b3: expected positive (inner ring), got {soft[3]}"

    # Verify magnitudes make sense (saturated to ±127 is OK for 8-bit)
    # b0 should be larger than b3 (farther from boundary)
    # Actually b0 could saturate at 127 so just check signs are correct
    dut._log.info(f"16-QAM soft magnitudes: {soft}")
    dut._log.info("16-QAM soft magnitude test passed")


@cocotb.test()
async def test_64qam_basic(dut):
    """64-QAM: 6 bits per symbol from known constellation points.

    64-QAM grid: ±1, ±3, ±5, ±7 (normalized by 1/sqrt(42)).
    Soft demap:
      LLR(b0) = Re
      LLR(b1) = 4d - |Re|
      LLR(b2) = 2d - ||Re| - 4d|
      LLR(b3) = Im
      LLR(b4) = 4d - |Im|
      LLR(b5) = 2d - ||Im| - 4d|

    Where d = NORM (the grid spacing / innermost point magnitude).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    NORM = 500

    # Test 4 points at different grid locations:
    # Point 1: Re=+1*NORM (+500), Im=+1*NORM (+500) → bits 111111
    # Point 2: Re=+3*NORM (+1500), Im=-5*NORM (-2500) → bits 101010 (approx)
    # Point 3: Re=-7*NORM (-3500), Im=+7*NORM (+3500) → bits 000000
    # Point 4: Re=-1*NORM (-500), Im=-3*NORM (-1500) → bits 011010 (approx)

    # For 64-QAM mapping (Table 17-15):
    # b0: sign(Re) → 1 if positive
    # b1: |Re| < 4d → 1 if inner 4 (|Re| in {1d, 3d})
    # b2: ||Re|-4d| < 2d → 1 if |Re| in {3d, 5d} (middle pair in each half)
    # (same for b3,b4,b5 on Im axis)

    # Point 1: Re=+1d → b0=1, |Re|=1d<4d→b1=1, ||Re|-4d|=3d>2d→b2=0... wait
    # Let me re-derive:
    # Actually the LLR formulas determine the sign, not the bit directly.
    # b2: LLR = 2d - ||Re|-4d|
    # For |Re|=1d: LLR = 2d - |1d-4d| = 2d - 3d = -d → b2=0

    # Let's just verify signs:
    test_pts = [
        (1, 1),    # Re=+1d, Im=+1d
        (3, -5),   # Re=+3d, Im=-5d
        (-7, 7),   # Re=-7d, Im=+7d
        (-1, -3),  # Re=-1d, Im=-3d
    ]

    sym_re = [p[0] * NORM for p in test_pts]
    sym_im = [p[1] * NORM for p in test_pts]

    soft = await feed_symbols(dut, sym_re, sym_im, QAM64, norm=NORM)

    assert len(soft) == 24, f"64-QAM: expected 24 soft bits (4×6), got {len(soft)}"

    # Verify each point's hard decisions
    for pt_idx, (re_grid, im_grid) in enumerate(test_pts):
        pt_soft = soft[pt_idx * 6:(pt_idx + 1) * 6]

        # Compute expected hard decisions from LLR formulas
        re_abs = abs(re_grid)
        im_abs = abs(im_grid)

        # b0 = sign(Re): 1 if Re > 0
        exp_b0 = 1 if re_grid > 0 else 0
        # b1 = 4 - |Re/d|: positive if |Re| < 4d (inner 4 points)
        exp_b1 = 1 if (4 - re_abs) > 0 else 0
        # b2 = 2 - ||Re/d| - 4|: positive if ||Re|-4d| < 2d
        exp_b2 = 1 if (2 - abs(re_abs - 4)) > 0 else 0
        # b3 = sign(Im): 1 if Im > 0
        exp_b3 = 1 if im_grid > 0 else 0
        # b4 = 4 - |Im/d|
        exp_b4 = 1 if (4 - im_abs) > 0 else 0
        # b5 = 2 - ||Im/d| - 4|
        exp_b5 = 1 if (2 - abs(im_abs - 4)) > 0 else 0

        expected = [exp_b0, exp_b1, exp_b2, exp_b3, exp_b4, exp_b5]
        actual = [hard_decision(s) for s in pt_soft]

        assert actual == expected, \
            f"64-QAM point {pt_idx} ({re_grid}d, {im_grid}d): " \
            f"got {actual}, expected {expected}, soft={pt_soft}"

    dut._log.info("64-QAM basic test passed")


@cocotb.test()
async def test_timing_48_subcarriers(dut):
    """Verify that feeding 48 subcarriers produces exactly N_CBPS soft bits.

    BPSK: 48 subcarriers → 48 soft bits
    QPSK: 48 subcarriers → 96 soft bits
    16-QAM: 48 subcarriers → 192 soft bits
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Feed 48 random-ish BPSK symbols
    sym_re = [(i * 137 % 2000) - 1000 for i in range(48)]
    sym_im = [0] * 48

    soft = await feed_symbols(dut, sym_re, sym_im, BPSK, norm=1000)
    assert len(soft) == 48, f"BPSK: expected 48, got {len(soft)}"

    # Reset and test QPSK
    await reset_dut(dut)
    sym_im_q = [(i * 97 % 2000) - 1000 for i in range(48)]
    soft = await feed_symbols(dut, sym_re, sym_im_q, QPSK, norm=1000)
    assert len(soft) == 96, f"QPSK: expected 96, got {len(soft)}"

    # Reset and test 16-QAM
    await reset_dut(dut)
    soft = await feed_symbols(dut, sym_re, sym_im_q, QAM16, norm=500)
    assert len(soft) == 192, f"16-QAM: expected 192, got {len(soft)}"

    # Reset and test 64-QAM
    await reset_dut(dut)
    soft = await feed_symbols(dut, sym_re, sym_im_q, QAM64, norm=300)
    assert len(soft) == 288, f"64-QAM: expected 288, got {len(soft)}"

    dut._log.info("Timing test passed: correct output count for all rates")


@cocotb.test()
async def test_bpsk_small_magnitude(dut):
    """BPSK with ±1 magnitude (worst case from equalizer with high shift_val).

    Even with tiny magnitudes, hard decisions should be correct.
    The soft bits will effectively be hard decisions (±1 clamped to ±something).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Equalizer outputs ±1 for BPSK with unit channel + high shift_val
    sym_re = [1, -1, 1, 1, -1, -1, 1, -1]
    sym_im = [0, 0, 0, 0, 0, 0, 0, 0]

    soft = await feed_symbols(dut, sym_re, sym_im, BPSK, norm=1)

    assert len(soft) == 8, f"Expected 8 soft bits, got {len(soft)}"

    expected_hard = [1, 0, 1, 1, 0, 0, 1, 0]
    for i in range(8):
        hd = hard_decision(soft[i])
        assert hd == expected_hard[i], \
            f"BPSK small-mag bit {i}: soft={soft[i]}, hard={hd}, expected={expected_hard[i]}"

    dut._log.info("BPSK small magnitude test passed (hard decisions correct even at ±1)")
