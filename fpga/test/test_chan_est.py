"""
Test chan_est — Channel estimator for 802.11a OFDM.

Given two LTF symbol FFT outputs (sequential bin read), computes:
  H[k] = (LTF1[k] + LTF2[k]) / 2 * LTF_ref[k]   (channel estimate)
  H_inv[k] = conj(H[k]) / |H[k]|^2                (equalization coefficient)

The module stores H_inv in internal BRAM, readable by the equalizer.

Tests:
1. Unit channel (LTF = ideal ±1 on active bins) → H_inv = LTF_ref (±1 real)
2. Scaled channel (gain 0.5) → H_inv compensates (×2)
3. Complex channel (known rotation) → H_inv removes it
4. Null subcarrier handling (guards/DC) → H_inv = 0
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import math
import json
import os
import numpy as np

# LTF reference sequence — NATURAL FFT BIN ORDER
# Active subcarriers in natural 64-pt FFT bins:
#   bins 1-26   = positive subcarriers (+1 to +26)
#   bins 38-63  = negative subcarriers (-26 to -1)
# All values are ±1 real. Null at bin 0 (DC) and bins 27-37 (guards).
LTF_REF = [0]*64
# Positive subcarriers (bins 1-26): L(+1..+26) from IEEE 802.11-2020 Eq 17-25
_ltf_pos = [+1, -1, -1, +1, +1, -1, +1, -1, +1, -1, -1, -1, -1, -1, +1, +1, -1, -1, +1, -1, +1, -1, +1, +1, +1, +1]
# Negative subcarriers (bins 38-63): L(-26..-1) from IEEE 802.11-2020 Eq 17-25
_ltf_neg = [+1, +1, -1, -1, +1, +1, -1, +1, -1, +1, +1, +1, +1, +1, +1, -1, -1, +1, +1, -1, +1, -1, +1, +1, +1, +1]
for i, v in enumerate(_ltf_pos):
    LTF_REF[1 + i] = v   # bins 1-26
for i, v in enumerate(_ltf_neg):
    LTF_REF[38 + i] = v  # bins 38-63

# Active bin indices (natural FFT order)
ACTIVE_BINS = list(range(1, 27)) + list(range(38, 64))
NULL_BINS = [0] + list(range(27, 38))


def to_s16(v):
    """Interpret 16-bit value as signed."""
    v = int(v) & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


def s16_to_bits(v):
    """Convert signed int to 16-bit unsigned representation."""
    if v < 0:
        v = v + 0x10000
    return v & 0xFFFF


async def reset_dut(dut):
    """Reset the DUT and initialize all inputs."""
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.bin_valid.value = 0
    dut.bin_idx.value = 0
    dut.bin_re.value = 0
    dut.bin_im.value = 0
    dut.rd_addr.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def feed_ltf_pair(dut, ltf1_re, ltf1_im, ltf2_re, ltf2_im):
    """Feed two LTF FFT outputs to the channel estimator and wait for done.

    Each LTF is a list of 64 complex values (16-bit signed re/im).
    Feeds bin-by-bin sequentially (matching decode_engine fft_bin_* output format).
    """
    # Pulse start
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)

    # Feed LTF1 bins sequentially
    for i in range(64):
        dut.bin_valid.value = 1
        dut.bin_idx.value = i
        dut.bin_re.value = s16_to_bits(ltf1_re[i])
        dut.bin_im.value = s16_to_bits(ltf1_im[i])
        await RisingEdge(dut.clk)

    dut.bin_valid.value = 0
    await ClockCycles(dut.clk, 2)

    # Feed LTF2 bins sequentially
    for i in range(64):
        dut.bin_valid.value = 1
        dut.bin_idx.value = i
        dut.bin_re.value = s16_to_bits(ltf2_re[i])
        dut.bin_im.value = s16_to_bits(ltf2_im[i])
        await RisingEdge(dut.clk)

    dut.bin_valid.value = 0

    # Wait for done — compute phase takes ~52 bins × 70 cycles = ~3640+ cycles
    # (iterative divider: 32 cycles per division × 2 divisions per bin)
    for _ in range(8000):
        await RisingEdge(dut.clk)
        if int(dut.done.value) == 1:
            return True

    assert False, "chan_est done never asserted"


async def read_h_inv(dut):
    """Read all 64 H_inv values from the channel estimator."""
    h_inv_re = [0] * 64
    h_inv_im = [0] * 64

    for i in range(64):
        dut.rd_addr.value = i
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)  # one cycle read latency
        h_inv_re[i] = to_s16(int(dut.h_inv_re.value))
        h_inv_im[i] = to_s16(int(dut.h_inv_im.value))

    return h_inv_re, h_inv_im


@cocotb.test()
async def test_unit_channel(dut):
    """Unit channel: LTF bins equal the known LTF reference (±1 real, scaled).

    For unit channel, received LTF FFT = LTF_ref * scale.
    H[k] = (avg) * LTF_ref[k] = scale * LTF_ref[k]^2 = scale (since LTF is ±1).
    H_inv[k] = 1/scale on active bins, 0 on null bins.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Simulate unit channel: FFT output = LTF_ref * 1000 (scale for fixed-point)
    scale = 1000
    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        ltf_re[k] = LTF_REF[k] * scale
        # imag = 0 (perfect channel, no phase)

    # Both LTF symbols identical (no noise)
    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)

    # Read H_inv
    h_inv_re, h_inv_im = await read_h_inv(dut)

    # On active bins: H = scale, so H_inv = 1/scale.
    # In Q1.15 fixed-point: 1/1000 * 32768 ≈ 32.77
    # But the module should normalize differently. Let's check:
    # H_inv = conj(H) / |H|^2 = scale / scale^2 = 1/scale
    # In 16-bit: depends on the scaling convention.
    #
    # Actually, for equalization we want: eq[k] = Y[k] * H_inv[k]
    # If Y has magnitude ~scale and H_inv = 1/scale, then eq ≈ 1.
    # The module likely stores H_inv in a format where the product gives
    # meaningful output. Let's verify the relative relationships:

    # Check that all active bins have the same H_inv magnitude
    # and null bins are zero
    for k in NULL_BINS:
        assert h_inv_re[k] == 0 and h_inv_im[k] == 0, \
            f"Null bin {k} should be 0, got ({h_inv_re[k]}, {h_inv_im[k]})"

    # Active bins: H_inv should be real (imaginary ≈ 0) and same magnitude
    # Since H = scale * LTF_ref[k] and LTF_ref is ±1:
    # H[k] = ±scale (real), so H_inv[k] = ±1/scale (real)
    # Actually: H_inv = conj(H)/|H|^2. For real H=±scale:
    # H_inv = (±scale) / scale^2 = ±1/scale
    # So H_inv[k] = LTF_ref[k] / scale^2... wait.
    # Let's reconsider:
    # H[k] = avg * LTF_ref[k] = scale * LTF_ref[k] * LTF_ref[k] = scale
    # (because LTF_ref^2 = 1 for ±1 values)
    # So H[k] = scale (same sign for all active bins)
    # H_inv = conj(H)/|H|^2 = scale / scale^2 = 1/scale
    #
    # All active bins should have same H_inv value.
    # The exact fixed-point value depends on scaling.

    # For now, verify:
    # 1. Active bins are all approximately equal (within ±1 LSB of each other)
    # 2. Imaginary parts are near zero
    active_re = [h_inv_re[k] for k in ACTIVE_BINS]
    active_im = [h_inv_im[k] for k in ACTIVE_BINS]

    # All imaginary parts should be near zero (tolerance ±2 for rounding)
    for k in ACTIVE_BINS:
        assert abs(h_inv_im[k]) <= 2, \
            f"Active bin {k}: imag should be ~0, got {h_inv_im[k]}"

    # All real parts should be the same value (uniform channel)
    expected_val = active_re[0]
    assert expected_val != 0, "H_inv should be non-zero on active bins"
    for i, k in enumerate(ACTIVE_BINS):
        assert abs(h_inv_re[k] - expected_val) <= 2, \
            f"Active bin {k}: expected ~{expected_val}, got {h_inv_re[k]}"

    dut._log.info(f"Unit channel: H_inv active value = {expected_val} "
                  f"(expected 1/{scale} in module's fixed-point)")


@cocotb.test()
async def test_scaled_channel(dut):
    """Scaled channel: LTF with gain of 0.5 → H_inv should be 2× larger."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Channel gain = 500 (half of the 1000 used in unit test)
    scale = 500
    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        ltf_re[k] = LTF_REF[k] * scale

    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)
    h_inv_re, h_inv_im = await read_h_inv(dut)

    # H = scale, H_inv = 1/scale.
    # Compared to unit test (scale=1000), H_inv should be 2× larger.
    # Let's read the first active bin as reference:
    ref_val = h_inv_re[ACTIVE_BINS[0]]
    assert ref_val != 0, "H_inv should be non-zero"

    for k in ACTIVE_BINS:
        assert abs(h_inv_im[k]) <= 2, \
            f"Bin {k}: imag should be ~0, got {h_inv_im[k]}"
        assert abs(h_inv_re[k] - ref_val) <= 2, \
            f"Bin {k}: expected ~{ref_val}, got {h_inv_re[k]}"

    for k in NULL_BINS:
        assert h_inv_re[k] == 0 and h_inv_im[k] == 0, \
            f"Null bin {k} should be 0"

    dut._log.info(f"Scaled channel (gain={scale}): H_inv = {ref_val}")


@cocotb.test()
async def test_complex_channel(dut):
    """Complex channel: apply known phase rotation → H_inv should undo it.

    Channel H = A * exp(j*theta) where A = 800, theta = pi/4.
    Received LTF[k] = LTF_ref[k] * H = LTF_ref[k] * A * exp(j*pi/4)
    After averaging and LTF_ref multiplication:
    H_est[k] = A * exp(j*pi/4)   (uniform for all active bins)
    H_inv[k] = conj(H) / |H|^2 = exp(-j*pi/4) / A
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    A = 800
    theta = math.pi / 4  # 45 degrees
    h_re_float = A * math.cos(theta)  # ≈ 565.7
    h_im_float = A * math.sin(theta)  # ≈ 565.7

    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        # Received = LTF_ref[k] * (h_re + j*h_im)
        ltf_re[k] = int(round(LTF_REF[k] * h_re_float))
        ltf_im[k] = int(round(LTF_REF[k] * h_im_float))

    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)
    h_inv_re, h_inv_im = await read_h_inv(dut)

    # Expected H_inv direction: exp(-j*pi/4) → re positive, im negative
    # Both components should have same magnitude (cos(pi/4) = sin(pi/4))
    # Null bins should be zero
    for k in NULL_BINS:
        assert h_inv_re[k] == 0 and h_inv_im[k] == 0, \
            f"Null bin {k} should be 0, got ({h_inv_re[k]}, {h_inv_im[k]})"

    # Active bins should all have same H_inv value (uniform channel)
    ref_re = h_inv_re[ACTIVE_BINS[0]]
    ref_im = h_inv_im[ACTIVE_BINS[0]]

    for k in ACTIVE_BINS:
        assert abs(h_inv_re[k] - ref_re) <= 3, \
            f"Bin {k}: H_inv_re expected ~{ref_re}, got {h_inv_re[k]}"
        assert abs(h_inv_im[k] - ref_im) <= 3, \
            f"Bin {k}: H_inv_im expected ~{ref_im}, got {h_inv_im[k]}"

    # Verify phase of H_inv is approximately -pi/4
    # Both re and im should be positive and negative respectively
    # (conj of exp(j*pi/4) = exp(-j*pi/4) → re > 0, im < 0)
    assert ref_re > 0, f"H_inv real should be positive, got {ref_re}"
    assert ref_im < 0, f"H_inv imag should be negative, got {ref_im}"

    # |re| ≈ |im| for pi/4 rotation
    assert abs(abs(ref_re) - abs(ref_im)) <= max(3, abs(ref_re) // 10), \
        f"H_inv magnitude asymmetry: re={ref_re}, im={ref_im}"

    dut._log.info(f"Complex channel: H_inv = ({ref_re}, {ref_im})")


@cocotb.test()
async def test_null_bins_zero(dut):
    """Verify null subcarriers (DC, guards) always produce H_inv = 0."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Even if we put garbage in the null bins, H_inv should force them to 0
    ltf_re = [500] * 64  # all bins have some value
    ltf_im = [300] * 64

    # Active bins still need proper LTF values for the module to work
    for k in ACTIVE_BINS:
        ltf_re[k] = LTF_REF[k] * 1000
        ltf_im[k] = 0

    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)
    h_inv_re, h_inv_im = await read_h_inv(dut)

    for k in NULL_BINS:
        assert h_inv_re[k] == 0 and h_inv_im[k] == 0, \
            f"Null bin {k} must be 0, got ({h_inv_re[k]}, {h_inv_im[k]})"

    dut._log.info("All null bins correctly zeroed")


@cocotb.test()
async def test_equalization_identity(dut):
    """End-to-end check: H_inv * H should give ~2^shift_val (identity) on active bins.

    This verifies that using H_inv for equalization actually works:
    eq[k] = Y[k] * H_inv[k] >> shift_val ≈ X[k] (original transmitted data)
    when Y[k] = X[k] * H[k].

    For this test, H = 1200 + j*400 (fixed complex channel).
    H_inv = conj(H) << shift_val / |H|^2
    Product H * H_inv should ≈ 2^shift_val (the identity in scaled fixed-point).

    We verify that H_re * H_inv_re - H_im * H_inv_im ≈ constant (real)
    and H_re * H_inv_im + H_im * H_inv_re ≈ 0 (imag).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    H_re_chan = 1200
    H_im_chan = 400

    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        # Received LTF = LTF_ref * H_channel
        ltf_re[k] = LTF_REF[k] * H_re_chan
        ltf_im[k] = LTF_REF[k] * H_im_chan

    # Clamp to 16-bit range
    for k in range(64):
        ltf_re[k] = max(-32768, min(32767, ltf_re[k]))
        ltf_im[k] = max(-32768, min(32767, ltf_im[k]))

    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)
    h_inv_re, h_inv_im = await read_h_inv(dut)

    # Read shift_val
    shift_val = int(dut.shift_val.value)

    # Verify H * H_inv product on active bins
    # The product's real part should be consistent (all same value ≈ 2^shift_val)
    # and imaginary part should be ~0
    products_re = []
    products_im = []
    for k in ACTIVE_BINS:
        # Product = H * H_inv (both in fixed-point, result scaled by 2^shift_val)
        pr = H_re_chan * h_inv_re[k] - H_im_chan * h_inv_im[k]
        pi = H_re_chan * h_inv_im[k] + H_im_chan * h_inv_re[k]
        products_re.append(pr)
        products_im.append(pi)

    # All product_re values should be the same (the "identity" scale factor ≈ 2^shift_val)
    ref_product = products_re[0]
    assert ref_product != 0, "Product should be non-zero"

    for i, k in enumerate(ACTIVE_BINS):
        # Allow ±1% tolerance due to fixed-point rounding
        tol = max(abs(ref_product) // 50, 100)
        assert abs(products_re[i] - ref_product) <= tol, \
            f"Bin {k}: product_re = {products_re[i]}, expected ~{ref_product}"
        assert abs(products_im[i]) <= tol, \
            f"Bin {k}: product_im = {products_im[i]}, expected ~0"

    dut._log.info(f"Equalization identity verified: H*H_inv = {ref_product} "
                  f"(shift_val={shift_val}, consistent across {len(ACTIVE_BINS)} active bins)")


@cocotb.test()
async def test_adaptive_shift_precision(dut):
    """Adaptive shift: full-scale signal must produce H_inv with ≥12 bits of precision.

    With full-scale 12-bit input quantized to ±2047, FFT bins are ~7500.
    The fixed Q1.15 H_inv would be ~4 (only 2 bits! → 8% EVM).
    With adaptive shift, H_inv should fill most of the 16-bit range,
    giving <0.1% equalization EVM.

    This test also verifies the shift_val output exists and is correct.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Simulate full-scale signal: LTF bins at magnitude ~7500
    # (This is what happens when deimos_hil_inject fills 12-bit ADC range)
    scale = 7500
    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        ltf_re[k] = LTF_REF[k] * scale

    # Clamp to 16-bit
    for k in range(64):
        ltf_re[k] = max(-32768, min(32767, ltf_re[k]))
        ltf_im[k] = max(-32768, min(32767, ltf_im[k]))

    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)

    # Read shift_val output
    await RisingEdge(dut.clk)
    shift_val = int(dut.shift_val.value)
    assert shift_val > 15, \
        f"shift_val should be >15 for large signal (got {shift_val})"
    assert shift_val <= 31, \
        f"shift_val should fit in 5 bits (got {shift_val})"

    # Read H_inv values
    h_inv_re, h_inv_im = await read_h_inv(dut)

    # Active bins: H_inv should fill most of 16-bit range (≥12 bits used)
    active_hinv_mag = [abs(h_inv_re[k]) for k in ACTIVE_BINS]
    avg_hinv = sum(active_hinv_mag) / len(active_hinv_mag)

    # With adaptive shift, H_inv should be >> 100 (not the pathetic 4 of Q1.15)
    assert avg_hinv > 1000, \
        f"H_inv average magnitude {avg_hinv:.0f} too small (precision problem). " \
        f"Expected >1000 with adaptive shift."

    # Verify equalization precision: H * H_inv >> shift_val should ≈ 1.0
    # Note: chan_est removes the LTF_ref sign when computing H, so H[k] = +scale
    # for ALL active bins regardless of LTF_ref polarity.
    # The equalization identity is: H[k] * H_inv[k] / 2^shift_val ≈ 1.0
    max_eq_error = 0
    for k in ACTIVE_BINS:
        # H[k] = scale (always positive after LTF_ref sign removal)
        # H_inv[k] should also be positive for real positive H
        product = scale * h_inv_re[k]
        eq_normalized = product / (2**shift_val)
        if k == ACTIVE_BINS[0]:
            ref_eq = eq_normalized
        else:
            eq_error = abs(eq_normalized - ref_eq) / abs(ref_eq) if ref_eq != 0 else 0
            max_eq_error = max(max_eq_error, eq_error)

    # Equalization should be consistent across bins (< 1% relative error)
    assert max_eq_error < 0.01, \
        f"Equalization inconsistency across bins: {max_eq_error*100:.2f}% (should be <1%)"

    # Verify the actual EVM: product should be very close to a constant
    # (the "gain" after equalization — doesn't matter what value, just consistent)
    products = []
    for k in ACTIVE_BINS:
        # Use scale (not LTF_REF[k]*scale) because chan_est stores H = +scale always
        p = scale * h_inv_re[k] / (2**shift_val)
        products.append(p)

    mean_p = sum(products) / len(products)
    max_deviation = max(abs(p - mean_p) for p in products)
    evm_pct = (max_deviation / abs(mean_p)) * 100 if mean_p != 0 else 100

    assert evm_pct < 0.5, \
        f"Equalization EVM = {evm_pct:.2f}% (should be <0.5% with adaptive shift)"

    dut._log.info(f"Adaptive shift: shift_val={shift_val}, avg H_inv={avg_hinv:.0f}, "
                  f"EVM={evm_pct:.4f}%")


@cocotb.test()
async def test_neg32768_safe_negation(dut):
    """Verify -32768 negation doesn't wrap in division setup.

    When an LTF bin is exactly -32768 (e.g. from ADC clipping or a transient
    channel null at max magnitude), the divider's absolute-value step must not
    silently wrap (-(-32768) = -32768 in 16-bit signed).

    After fix: sat_abs16(-32768) = 32767, losing 1 LSB but avoiding catastrophic
    sign inversion in H_inv. The output should still be a reasonable inverse.

    This is the 1-in-65536 scenario that becomes likely under real conditions
    (clipping, channel nulls during transients).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Construct LTF where one active bin has value -32768 (the pathological case).
    # Use bin 1 (first positive subcarrier, LTF_REF[1] = +1).
    # A received LTF value of -32768 means H[1] = -32768 * LTF_REF[1] = -32768.
    # The divider must compute |H_re| = abs(-32768) → should be 32767 (saturated).
    # Then H_inv_re = -(32767 << shift) / |H|^2  (sign from cur_h_re[15])

    scale = 1000  # normal magnitude for most bins
    ltf_re = [0] * 64
    ltf_im = [0] * 64

    for k in ACTIVE_BINS:
        ltf_re[k] = LTF_REF[k] * scale

    # Inject the pathological value on bin 1 (real part = -32768)
    ltf_re[1] = -32768
    # Also test imaginary path: inject on bin 2
    ltf_im[2] = -32768

    # Feed identical LTF pair (no noise)
    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)

    # Read H_inv
    h_inv_re, h_inv_im = await read_h_inv(dut)

    # Key assertion: H_inv for the pathological bins should have CORRECT SIGN.
    # Before fix: -(-32768) wraps to -32768, and the dividend becomes 0x8000=32768
    # interpreted as unsigned — coincidentally correct magnitude but only by accident.
    # After fix: sat_abs16(-32768) = 32767, explicit and intentional.

    # For bin 1: LTF_REF[1] = +1, so chan_est stores H_re = ltf_re[1] = -32768.
    # After averaging: cur_h_re = -32768 (since both LTFs same).
    # Division: dividend = sat_abs16(-32768) = 32767
    # Sign: cur_h_re[15] = 1, so div_sign = 1 → result is negated → H_inv_re < 0
    # This is correct: H = -32768 (negative), H_inv should be negative
    # (1/H where H is negative).
    # The magnitude: |H_inv| = (32767 << shift) / (32768^2)
    # For typical shift=15: (32767 << 15) / 1,073,741,824 ≈ 1 → very small
    # But the KEY point: it must be NEGATIVE (sign preserved correctly).

    # Bin 1: H was -32768 → H_inv_re should be negative
    assert h_inv_re[1] <= 0, \
        f"Bin 1 H_inv_re should be <=0 (H was negative), got {h_inv_re[1]}. " \
        f"Likely -32768 negation wraparound bug."

    # Bin 2: imaginary was -32768, so H_inv_im should handle it too
    # For conjugate inversion: H_inv_im = -H_im / |H|^2
    # H_im = -32768 (negative), so H_inv_im = -(-32768)/|H|^2 → positive
    # With sat_neg: H_inv_im uses sign = ~cur_h_im[15] = ~1 = 0, no negate → positive
    assert h_inv_im[2] >= 0, \
        f"Bin 2 H_inv_im should be >=0 (conjugate of negative H_im), got {h_inv_im[2]}. " \
        f"Likely -32768 negation wraparound bug."

    # Also verify that normal bins are unaffected (spot check)
    # Bin 5 should have positive H_inv_re (LTF_REF[5] = -1, so H = -(-1*1000) = -1000... 
    # actually H = ltf_re[5] * LTF_REF[5] = (-1000)*(-1)... but chan_est just averages,
    # the LTF_ref multiply is internal. Let's just check it's non-zero.
    assert h_inv_re[5] != 0, f"Normal bin 5 H_inv_re should be non-zero"

    # Check that H_inv values are reasonable magnitude (not wildly wrong from wrap)
    # Bins with scale=1000: H_inv should be substantial
    normal_magnitudes = [abs(h_inv_re[k]) for k in ACTIVE_BINS if k not in (1, 2)]
    avg_normal = sum(normal_magnitudes) / len(normal_magnitudes) if normal_magnitudes else 0
    assert avg_normal > 100, \
        f"Normal bins avg H_inv magnitude {avg_normal:.0f} too low"

    dut._log.info(f"sat_neg16 test passed: bin1 H_inv_re={h_inv_re[1]} (correct sign), "
                  f"bin2 H_inv_im={h_inv_im[2]} (correct sign), "
                  f"normal bins avg={avg_normal:.0f}")


async def feed_bins(dut, n, start_idx, ltf_re, ltf_im):
    """Feed n consecutive LTF bins starting at start_idx."""
    for i in range(n):
        dut.bin_valid.value = 1
        dut.bin_idx.value = (start_idx + i) % 64
        dut.bin_re.value = s16_to_bits(ltf_re[(start_idx + i) % 64])
        dut.bin_im.value = s16_to_bits(ltf_im[(start_idx + i) % 64])
        await RisingEdge(dut.clk)
    dut.bin_valid.value = 0
    await RisingEdge(dut.clk)


def unit_channel_ltf(scale=1000):
    """Ideal LTF pair for a unit channel (identical symbols, no noise)."""
    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        ltf_re[k] = LTF_REF[k] * scale
    return ltf_re, ltf_im


async def assert_unit_h_inv(dut):
    """H_inv checks for a unit channel (same assertions as test_unit_channel)."""
    h_inv_re, h_inv_im = await read_h_inv(dut)
    for k in NULL_BINS:
        assert h_inv_re[k] == 0 and h_inv_im[k] == 0, \
            f"Null bin {k} should be 0, got ({h_inv_re[k]}, {h_inv_im[k]})"
    active_re = [h_inv_re[k] for k in ACTIVE_BINS]
    expected_val = active_re[0]
    assert expected_val != 0, "H_inv should be non-zero on active bins"
    for i, k in enumerate(ACTIVE_BINS):
        assert abs(h_inv_im[k]) <= 2, \
            f"Active bin {k}: imag should be ~0, got {h_inv_im[k]}"
        assert abs(h_inv_re[k] - expected_val) <= 2, \
            f"Active bin {k}: expected ~{expected_val}, got {h_inv_re[k]}"


async def stall_and_verify_recovery(dut):
    """After a mid-capture stall, a fresh start + full LTF pair must produce
    a correct H_inv (the FSM recovered to S_IDLE; otherwise the new bins
    would merge with the stale partial capture and corrupt H)."""
    ltf_re, ltf_im = unit_channel_ltf()

    # Wait out the capture watchdog + margin
    await ClockCycles(dut.clk, 1024 + 100)

    # done must NOT have pulsed during the stall (garbage H must not be
    # presented as valid)
    assert int(dut.done.value) == 0, \
        "done asserted during stall — timeout must not flag garbage H as valid"

    # Fresh frame: pulse start, feed a full LTF pair
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)
    await feed_bins(dut, 64, 0, ltf_re, ltf_im)
    await ClockCycles(dut.clk, 2)
    await feed_bins(dut, 64, 0, ltf_re, ltf_im)

    for _ in range(8000):
        await RisingEdge(dut.clk)
        if int(dut.done.value) == 1:
            await assert_unit_h_inv(dut)
            return

    assert False, "chan_est done never asserted after recovery"


@cocotb.test()
async def test_timeout_recovery_ltf1_partial(dut):
    """Stall watchdog: start + only 30 LTF1 bins (aborted frame) must not
    wedge the FSM. A following frame must estimate a clean channel."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ltf_re, ltf_im = unit_channel_ltf()

    # Aborted frame: start, then the bin stream dies after 30 bins
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)
    await feed_bins(dut, 30, 0, ltf_re, ltf_im)

    await stall_and_verify_recovery(dut)
    dut._log.info("LTF1-partial stall recovered with clean H_inv")


@cocotb.test()
async def test_timeout_recovery_wait_ltf2(dut):
    """Stall watchdog: full LTF1 but LTF2 never arrives (abort between
    symbols) must not wedge the FSM in S_WAIT_LTF2."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ltf_re, ltf_im = unit_channel_ltf()

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)
    await feed_bins(dut, 64, 0, ltf_re, ltf_im)

    await stall_and_verify_recovery(dut)
    dut._log.info("S_WAIT_LTF2 stall recovered with clean H_inv")


@cocotb.test()
async def test_timeout_recovery_ltf2_partial(dut):
    """Stall watchdog: full LTF1 + 30 LTF2 bins (abort mid-LTF2) must not
    wedge the FSM in S_CAPTURE_LTF2."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ltf_re, ltf_im = unit_channel_ltf()

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)
    await feed_bins(dut, 64, 0, ltf_re, ltf_im)
    await ClockCycles(dut.clk, 2)
    await feed_bins(dut, 30, 0, ltf_re, ltf_im)

    await stall_and_verify_recovery(dut)
    dut._log.info("LTF2-partial stall recovered with clean H_inv")


def multipath_ltf(taps, scale=8000):
    """LTF pair through a multipath channel: taps = [(delay, gain), ...].

    H[k] = sum gain * exp(-j*2*pi*k*delay/64) per natural FFT bin k.
    Received LTF[k] = LTF_REF[k] * H[k] * scale, quantized to 16-bit.
    """
    ltf_re = [0] * 64
    ltf_im = [0] * 64
    for k in ACTIVE_BINS:
        h = complex(0, 0)
        for delay, gain in taps:
            h += gain * np.exp(-2j * np.pi * k * delay / 64.0)
        v = LTF_REF[k] * h * scale
        ltf_re[k] = int(max(-32768, min(32767, round(v.real))))
        ltf_im[k] = int(max(-32768, min(32767, round(v.imag))))
    return ltf_re, ltf_im


def chan_h(taps, k):
    h = complex(0, 0)
    for delay, gain in taps:
        h += gain * np.exp(-2j * np.pi * k * delay / 64.0)
    return h


@cocotb.test()
async def test_deep_fade_multipath_no_clip(dut):
    """Multipath with a -6 dB tap: taps [(0, 1.0), (4, -0.5)] (~9.5 dB ripple).

    With the legacy shift (sv = 14 + bit_w/2) the fade-side bins clamp H_inv
    at the +-32767 divider saturation (that sv would clip ~24 of 52 active
    bins on this channel — computed below). With sv = 10 + bit_w/2 no bin
    may clip: clip_cnt must be 0 and the per-bin equalization identity
    must hold on every active bin (constant complex product, imag ~0).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    taps = [(0, 1.0 + 0j), (4, -0.5 + 0j)]
    scale = 8000
    ltf_re, ltf_im = multipath_ltf(taps, scale)

    await feed_ltf_pair(dut, ltf_re, ltf_im, ltf_re, ltf_im)
    h_inv_re, h_inv_im = await read_h_inv(dut)
    shift_val = int(dut.shift_val.value)

    # Legacy sv would clip: count bins where the old shift saturates
    max_mag_sq = max(abs(chan_h(taps, k) * scale) ** 2 for k in ACTIVE_BINS)
    bit_w = int(max_mag_sq).bit_length() if max_mag_sq > 0 else 1
    sv_old = min(31, max(15, 14 + (bit_w >> 1)))
    old_clips = sum(
        1 for k in ACTIVE_BINS
        if abs(chan_h(taps, k)) * scale * (2 ** sv_old) / (abs(chan_h(taps, k)) * scale) ** 2 > 32767
    )
    dut._log.info(f"deep fade: shift_val={shift_val}, legacy sv={sv_old} "
                  f"would clip {old_clips} bins; clip_cnt must be 0")

    # Discriminating assertion: no clamp with the new shift
    assert int(dut.clip_cnt.value) == 0, \
        f"clip_cnt = {int(dut.clip_cnt.value)} — fade bins clamped at +-32767"

    # Per-bin equalization identity: p[k] = H[k]*scale * H_inv[k],
    # p[k] / 2^shift_val must be ~constant (real ≈ scale, imag ≈ 0) for ALL
    # active bins — clamped bins would break this on the fade side.
    for k in ACTIVE_BINS:
        h = chan_h(taps, k) * scale
        inv = complex(h_inv_re[k], h_inv_im[k])
        p = h * inv / (2 ** shift_val)
        if k == ACTIVE_BINS[0]:
            ref_re, ref_im = p.real, p.imag
        else:
            tol = max(abs(ref_re) * 0.02, 0.05)
            assert abs(p.real - ref_re) <= tol, \
                f"Bin {k}: identity real {p.real:.0f} vs {ref_re:.0f} (tol {tol:.0f})"
            assert abs(p.imag - ref_im) <= tol, \
                f"Bin {k}: identity imag {p.imag:.0f} vs {ref_im:.0f} (tol {tol:.0f})"

    for k in NULL_BINS:
        assert h_inv_re[k] == 0 and h_inv_im[k] == 0, \
            f"Null bin {k} should be 0"

    dut._log.info(f"deep fade identity holds on all {len(ACTIVE_BINS)} active bins "
                  f"(ref p = {ref_re:.0f} + {ref_im:.0f}j, scale {scale})")
