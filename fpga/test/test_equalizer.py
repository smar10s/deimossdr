"""
Test equalizer — OFDM frequency-domain equalizer for 802.11a.

Given FFT output bins (sequential, 64 complex values) and H_inv from chan_est,
complex-multiplies each subcarrier by H_inv to remove channel distortion,
then extracts the 48 data subcarriers in standard order.

Algorithm:
  eq[k] = Y[k] * H_inv[k] >> (shift_val - EQ_SCALE)
  where EQ_SCALE=6, so output is ±64 for BPSK (not ±1).
  Output: 48 data subcarriers in order (negative freq first, then positive)

Tests:
1. Unit channel (H_inv = 1+0j scaled) → output = input * 64 on data subcarriers
2. Known channel → equalized output matches golden QAM points * 64
3. Pilot extraction (bins 7, 21, 43, 57) works correctly
4. Data subcarrier ordering matches LIB80211_DATA_BINS
5. Golden vector: annex_i1_signal_freq_with_pilots → equalized = ±64 BPSK
6. Adaptive shift: HIL scenario → output ±64 (not ±1)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import math
import json
import os

# Data subcarrier FFT bin indices (same as LIB80211_DATA_BINS)
# Negative freq subcarriers (-26..-1) first, then positive (+1..+26)
# Skips pilots at bins 7, 21, 43, 57 and DC at bin 0
DATA_BINS = [
    38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63,
     1,  2,  3,  4,  5,  6,  8,  9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26,
]
assert len(DATA_BINS) == 48

# Pilot subcarrier FFT bins: sc +7, +21, -21, -7
PILOT_BINS = [7, 21, 43, 57]

# Active bins = 48 data + 4 pilots = 52 subcarriers
ACTIVE_BINS = sorted(DATA_BINS + PILOT_BINS)
assert len(ACTIVE_BINS) == 52

# Null bins: DC (0), lower guard (27-31 doesn't exist in 0-63 indexing...)
# Actually for 64-pt FFT with -32 to +31 mapping:
# bin index = (subcarrier + 64) % 64
# Null subcarriers: DC (bin 0), guards (bins 27-37 = subcarriers -5 to +5... no)
# Let me just define null as anything not in ACTIVE_BINS:
NULL_BINS = [k for k in range(64) if k not in ACTIVE_BINS]

# Q1.15 scale factor
Q15 = 32768

# EQ_SCALE: equalizer outputs at 2^EQ_SCALE times the normalized constellation.
# BPSK ±1 becomes ±64, 16-QAM ±1/±3 becomes ±64/±192, etc.
EQ_SCALE = 6
EQ_GAIN = 1 << EQ_SCALE  # 64


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
    """Reset the DUT and initialize inputs."""
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.fft_valid.value = 0
    dut.fft_bin.value = 0
    dut.fft_re.value = 0
    dut.fft_im.value = 0
    dut.hinv_re.value = 0
    dut.hinv_im.value = 0
    dut.shift_val.value = 15  # default: Q1.15 (backward compat)
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def run_equalization(dut, fft_re, fft_im, hinv_re, hinv_im):
    """Feed 64 FFT bins and H_inv values, collect equalized data subcarriers.

    Simulates chan_est BRAM behavior: drives hinv_re/hinv_im based on rd_addr
    output with 1-cycle latency (matching registered BRAM read).

    Args:
        fft_re, fft_im: lists of 64 signed 16-bit values (FFT output)
        hinv_re, hinv_im: lists of 64 signed 16-bit values (channel inverse)

    Returns:
        (data_re, data_im): lists of 48 equalized data subcarrier values (signed 16-bit)
        (pilot_re, pilot_im): lists of 4 equalized pilot values (signed 16-bit)
    """
    # Pulse start
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)

    # Feed 64 bins sequentially (CAPTURE phase)
    for i in range(64):
        dut.fft_valid.value = 1
        dut.fft_bin.value = i
        dut.fft_re.value = s16_to_bits(fft_re[i])
        dut.fft_im.value = s16_to_bits(fft_im[i])
        await RisingEdge(dut.clk)

    dut.fft_valid.value = 0

    # After capture, simulate BRAM-latency H_inv delivery.
    # Drive hinv_re/hinv_im based on rd_addr from PREVIOUS cycle (1-cycle latency).
    data_re = []
    data_im = []
    pilot_re = []
    pilot_im = []

    # Initial: drive H_inv[0] (rd_addr is 0 initially)
    last_rd_addr = int(dut.rd_addr.value) & 0x3F
    dut.hinv_re.value = s16_to_bits(hinv_re[last_rd_addr])
    dut.hinv_im.value = s16_to_bits(hinv_im[last_rd_addr])

    for _ in range(300):
        await RisingEdge(dut.clk)

        # After posedge: update hinv based on current rd_addr (will be sampled next posedge)
        cur_rd_addr = int(dut.rd_addr.value) & 0x3F
        dut.hinv_re.value = s16_to_bits(hinv_re[cur_rd_addr])
        dut.hinv_im.value = s16_to_bits(hinv_im[cur_rd_addr])

        if int(dut.data_valid.value) == 1:
            data_re.append(to_s16(int(dut.data_re.value)))
            data_im.append(to_s16(int(dut.data_im.value)))
        if int(dut.pilot_valid.value) == 1:
            pilot_re.append(to_s16(int(dut.pilot_re.value)))
            pilot_im.append(to_s16(int(dut.pilot_im.value)))
        if hasattr(dut, 'symbol_done') and int(dut.symbol_done.value) == 1:
            break

    return (data_re, data_im), (pilot_re, pilot_im)


@cocotb.test()
async def test_unit_channel(dut):
    """Unit channel (H_inv = 1.0 in Q15): output = input * EQ_GAIN on data subcarriers.

    H_inv[k] = 32768 + 0j (= 1.0 in Q1.15) on active bins, 0 on null bins.
    Input Y[k] = known values on data bins.
    Expected: equalized output = input * 64 (due to EQ_SCALE=6).
    With shift_val=15 (default), effective shift = 15 - 6 = 9.
    eq = Y * 32767 >> 9 ≈ Y * 64.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Create FFT input: known values on active bins (keep small to avoid overflow)
    fft_re = [0] * 64
    fft_im = [0] * 64
    for i, k in enumerate(DATA_BINS):
        # Use distinct values per subcarrier so we can verify ordering
        fft_re[k] = 50 + i * 2  # range: 50 to 144 (small enough that *64 fits 16-bit)
        fft_im[k] = -(25 + i)
    for k in PILOT_BINS:
        fft_re[k] = 100
        fft_im[k] = 0

    # H_inv = 1.0 (Q15 = 32767) on active bins, 0 elsewhere
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = Q15 - 1  # 32767 ≈ 1.0 in Q1.15
        hinv_im[k] = 0

    (data_re, data_im), (pilot_re, pilot_im) = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    # Verify 48 data subcarriers output
    assert len(data_re) == 48, f"Expected 48 data subcarriers, got {len(data_re)}"
    assert len(data_im) == 48

    # Each output should be input * 64 (with H_inv=1, eq = Y * 1 * 2^EQ_SCALE)
    # Effective shift = 15 - 6 = 9. eq = Y * 32767 >> 9.
    # For Y=50: eq = 50 * 32767 >> 9 = 1638350 >> 9 = 3199 ≈ 50*64 = 3200
    eff_shift = 15 - EQ_SCALE  # 9
    for i, k in enumerate(DATA_BINS):
        expected_re = (fft_re[k] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        expected_im = (fft_im[k] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        # Handle negative values for rounding
        if fft_im[k] < 0:
            expected_im = -((-fft_im[k]) * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        assert abs(data_re[i] - expected_re) <= 2, \
            f"Data subcarrier {i} (bin {k}): re expected {expected_re}, got {data_re[i]}"
        assert abs(data_im[i] - expected_im) <= 2, \
            f"Data subcarrier {i} (bin {k}): im expected {expected_im}, got {data_im[i]}"

    # Verify 4 pilots extracted
    assert len(pilot_re) == 4, f"Expected 4 pilots, got {len(pilot_re)}"
    for i, k in enumerate(PILOT_BINS):
        expected_pilot = (fft_re[k] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        assert abs(pilot_re[i] - expected_pilot) <= 2, \
            f"Pilot {i} (bin {k}): re expected {expected_pilot}, got {pilot_re[i]}"

    dut._log.info("Unit channel test passed: output ≈ input * 64")


@cocotb.test()
async def test_complex_channel_compensation(dut):
    """Known complex channel: H = 1.5*exp(j*pi/6) → H_inv compensates.

    After equalization: eq ≈ X * EQ_GAIN (the original transmitted symbols * 64).
    With shift_val=15 (Q1.15 H_inv), effective shift = 9.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Original transmitted symbols X (what we want to recover)
    # Use small BPSK-like values (will be multiplied by 64 in output)
    x_re = [0] * 64
    x_im = [0] * 64
    for i, k in enumerate(DATA_BINS):
        x_re[k] = 60 if (i % 2 == 0) else -60
        x_im[k] = 0

    # Channel: gain 1.5, phase 30 degrees
    gain = 1.5
    theta = math.pi / 6
    h_re_f = gain * math.cos(theta)  # ~1.299
    h_im_f = gain * math.sin(theta)  # ~0.75
    mag_sq = gain * gain  # 2.25

    # Received: Y = X * H (complex multiply)
    fft_re = [0] * 64
    fft_im = [0] * 64
    for k in ACTIVE_BINS:
        yr = x_re[k] * h_re_f - x_im[k] * h_im_f
        yi = x_re[k] * h_im_f + x_im[k] * h_re_f
        fft_re[k] = int(round(yr))
        fft_im[k] = int(round(yi))

    # H_inv = conj(H) / |H|^2, scaled by 2^15
    hinv_re_f = h_re_f * Q15 / mag_sq
    hinv_im_f = -h_im_f * Q15 / mag_sq

    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = int(round(hinv_re_f))
        hinv_im[k] = int(round(hinv_im_f))

    (data_re, data_im), _ = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48, f"Expected 48, got {len(data_re)}"

    # After equalization: eq ≈ X * EQ_GAIN = ±60 * 64 = ±3840
    # (with some fixed-point error from the channel compensation)
    eff_shift = 15 - EQ_SCALE
    for i, k in enumerate(DATA_BINS):
        # Expected: X * EQ_GAIN (approximately)
        expected_re = x_re[k] * EQ_GAIN
        expected_im = x_im[k] * EQ_GAIN
        # Allow ~5% tolerance for fixed-point quantization across complex multiply
        tol = max(abs(expected_re) // 15, 20)
        assert abs(data_re[i] - expected_re) <= tol, \
            f"Subcarrier {i} (bin {k}): eq_re={data_re[i]}, expected={expected_re} (tol={tol})"
        assert abs(data_im[i] - expected_im) <= tol, \
            f"Subcarrier {i} (bin {k}): eq_im={data_im[i]}, expected={expected_im} (tol={tol})"

    dut._log.info("Complex channel compensation passed")


@cocotb.test()
async def test_data_subcarrier_ordering(dut):
    """Verify data subcarriers are output in LIB80211_DATA_BINS order.

    Put unique values in each data bin. Verify output order matches
    DATA_BINS (negative freq first: bins 38-63 minus pilots, then
    positive freq: bins 1-26 minus pilots).
    Output is scaled by ~64 due to EQ_SCALE.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Put unique marker in each data bin (small values to avoid overflow at *64)
    fft_re = [0] * 64
    fft_im = [0] * 64
    for i, k in enumerate(DATA_BINS):
        fft_re[k] = 10 + i  # unique value per data subcarrier
        fft_im[k] = 20 + i

    # Also put values in pilot bins so they're distinguishable
    for i, k in enumerate(PILOT_BINS):
        fft_re[k] = 50 + i
        fft_im[k] = 60 + i

    # Unit H_inv on all active bins
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = Q15 - 1
        hinv_im[k] = 0

    (data_re, data_im), (pilot_re, pilot_im) = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48, f"Expected 48 data outputs, got {len(data_re)}"

    # Verify ordering: output[i] should correspond to DATA_BINS[i]
    # Output is input * ~64 (shift_val=15, eff_shift=9, eq = Y*32767>>9 ≈ Y*64)
    eff_shift = 15 - EQ_SCALE
    for i in range(48):
        expected_re = (fft_re[DATA_BINS[i]] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        expected_im = (fft_im[DATA_BINS[i]] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        assert abs(data_re[i] - expected_re) <= 2, \
            f"Output[{i}] (bin {DATA_BINS[i]}): re={data_re[i]}, expected={expected_re}"
        assert abs(data_im[i] - expected_im) <= 2, \
            f"Output[{i}] (bin {DATA_BINS[i]}): im={data_im[i]}, expected={expected_im}"

    # Verify pilot order: bins 7, 21, 43, 57
    assert len(pilot_re) == 4
    for i, k in enumerate(PILOT_BINS):
        expected_pilot = (fft_re[k] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        assert abs(pilot_re[i] - expected_pilot) <= 2, \
            f"Pilot[{i}] (bin {k}): re={pilot_re[i]}, expected={expected_pilot}"

    dut._log.info("Data subcarrier ordering verified (matches DATA_BINS)")


@cocotb.test()
async def test_null_bins_ignored(dut):
    """Null bins (DC, guards) should not appear in output."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Put large values in null bins — they should never appear in output
    fft_re = [9999] * 64
    fft_im = [8888] * 64

    # Put known small values in data/pilot bins
    for i, k in enumerate(DATA_BINS):
        fft_re[k] = 10
        fft_im[k] = 20
    for k in PILOT_BINS:
        fft_re[k] = 30
        fft_im[k] = 40

    # H_inv = 1 on active, 0 on null (as chan_est would produce)
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = Q15 - 1

    (data_re, data_im), _ = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48

    # Output should be ~10*64=640 (data bins), NOT 9999*64 (null bin values)
    eff_shift = 15 - EQ_SCALE
    expected_re = (10 * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
    expected_im = (20 * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
    for i in range(48):
        assert abs(data_re[i] - expected_re) <= 2, \
            f"Data[{i}]: got {data_re[i]}, expected ~{expected_re} (null bin leak?)"
        assert abs(data_im[i] - expected_im) <= 2, \
            f"Data[{i}]: got {data_im[i]}, expected ~{expected_im} (null bin leak?)"

    dut._log.info("Null bins correctly excluded from output")


@cocotb.test()
async def test_golden_vector_bpsk_signal(dut):
    """Golden vector: annex I.1 SIGNAL field (BPSK) through unit channel.

    For unit channel (H=1), the FFT output of the SIGNAL symbol IS the
    transmitted frequency-domain data. After equalization with H_inv=1,
    the 48 data subcarriers should be ±SCALE*EQ_GAIN (BPSK * 64).

    Uses annex_i1_signal_freq.json which has the expected BPSK values on
    data subcarriers (±1.0 real, 0 imaginary, null at pilot positions).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Load golden vector
    vec_dir = os.path.join(os.path.dirname(__file__), '../../extern/lib80211/vectors')
    with open(os.path.join(vec_dir, 'annex_i1_signal_freq_with_pilots.json')) as f:
        sig_with_pilots = json.load(f)

    # The vector uses subcarrier indexing -32 to +31 (array index 0 = sc -32)
    # Convert to FFT bin indexing: bin = (sc + 64) % 64
    # Array index i corresponds to sc = i - 32
    # FFT bin = (i - 32 + 64) % 64 = (i + 32) % 64
    subcarriers = sig_with_pilots['subcarriers']

    # Scale to 16-bit integer (BPSK ±1 → ±SCALE)
    # Keep SCALE small enough that SCALE*64 fits 16-bit signed (max 32767)
    SCALE = 400  # ±400 input → ±400*64 = ±25600 output (fits 16-bit)
    fft_re = [0] * 64
    fft_im = [0] * 64
    for i, sc in enumerate(subcarriers):
        if sc is None:
            continue
        bin_idx = (i + 32) % 64
        fft_re[bin_idx] = int(round(sc[0] * SCALE))
        fft_im[bin_idx] = int(round(sc[1] * SCALE))

    # Unit channel: H_inv = 1.0 on active bins
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = Q15 - 1

    (data_re, data_im), (pilot_re, pilot_im) = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48, f"Expected 48, got {len(data_re)}"

    # Load expected data subcarriers (without pilots)
    with open(os.path.join(vec_dir, 'annex_i1_signal_freq.json')) as f:
        sig_no_pilots = json.load(f)

    # Extract expected data subcarrier values in DATA_BINS order
    sig_subcarriers = sig_no_pilots['subcarriers']
    expected_data_re = []
    expected_data_im = []
    for k in DATA_BINS:
        # Convert bin index to subcarrier index for the vector
        sc = k if k < 32 else k - 64
        arr_idx = sc + 32
        val = sig_subcarriers[arr_idx]
        if val is None:
            expected_data_re.append(0)
            expected_data_im.append(0)
        else:
            expected_data_re.append(int(round(val[0] * SCALE)))
            expected_data_im.append(int(round(val[1] * SCALE)))

    # Verify each data subcarrier (output is input * ~64)
    eff_shift = 15 - EQ_SCALE
    for i in range(48):
        exp_re = (expected_data_re[i] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        exp_im = (expected_data_im[i] * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        # Handle negatives
        if expected_data_re[i] < 0:
            exp_re = -((-expected_data_re[i]) * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        if expected_data_im[i] < 0:
            exp_im = -((-expected_data_im[i]) * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
        tol = 3  # ±3 LSB for Q15 rounding
        assert abs(data_re[i] - exp_re) <= tol, \
            f"Data[{i}] (bin {DATA_BINS[i]}): re={data_re[i]}, expected={exp_re}"
        assert abs(data_im[i] - exp_im) <= tol, \
            f"Data[{i}] (bin {DATA_BINS[i]}): im={data_im[i]}, expected={exp_im}"

    # Verify all BPSK: real should be ±SCALE*64, imag should be ~0
    expected_mag = (SCALE * (Q15 - 1) + (1 << (eff_shift - 1))) >> eff_shift
    for i in range(48):
        assert abs(abs(data_re[i]) - expected_mag) <= 3, \
            f"Data[{i}]: BPSK magnitude {abs(data_re[i])}, expected {expected_mag}"
        assert abs(data_im[i]) <= 3, \
            f"Data[{i}]: BPSK imag {data_im[i]}, expected 0"

    # Verify pilots (should be ±SCALE*64 based on polarity)
    assert len(pilot_re) == 4
    dut._log.info(f"Pilots: {list(zip(pilot_re, pilot_im))}")

    dut._log.info(f"Golden vector BPSK SIGNAL passed: all 48 subcarriers at ±{expected_mag}")


@cocotb.test()
async def test_adaptive_shift_equalization(dut):
    """Adaptive shift: equalizer uses shift_val from chan_est (not fixed 15).

    Simulates the full-scale HIL scenario:
    - FFT output bins at magnitude ~7500 (from full 12-bit ADC input)
    - chan_est produces H_inv with shift_val=27 (adaptive, fills 14+ bits)
    - Equalizer computes eq = Y * H_inv >> (shift_val - EQ_SCALE)

    After equalization, constellation points should be at ±EQ_GAIN = ±64 (BPSK).
    This is the critical test: previously output was ±1 (unusable for demapper),
    now it's ±64 (strong soft decisions).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Simulate: channel H = 7500 (real, uniform — loopback scenario)
    # FFT output Y[k] = transmitted_symbol * H = ±1 * 7500 = ±7500
    # chan_est produced: H_inv ≈ 17895 with shift_val = 27
    # After eq: Y * H_inv >> (27-6) = ±7500 * 17895 >> 21 ≈ ±64

    H_mag = 7500
    shift_val = 27
    # H_inv = (H << shift) / |H|^2 = (7500 << 27) / 7500^2 = 2^27 / 7500 ≈ 17895
    hinv_val = (H_mag * (2**shift_val)) // (H_mag * H_mag)

    # FFT output: BPSK at ±H_mag on data bins, ±H_mag on pilots
    fft_re = [0] * 64
    fft_im = [0] * 64
    for i, k in enumerate(DATA_BINS):
        fft_re[k] = H_mag if (i % 2 == 0) else -H_mag
    for k in PILOT_BINS:
        fft_re[k] = H_mag

    # H_inv: uniform channel, same value for all active bins
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = hinv_val

    # Set shift_val
    dut.shift_val.value = shift_val

    (data_re, data_im), _ = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48, f"Expected 48, got {len(data_re)}"

    # After equalization: eq = Y * H_inv >> (shift_val - EQ_SCALE)
    # For Y = +7500: eq = 7500 * 17895 >> 21 = 134,212,500 >> 21 = 64
    eff_shift = shift_val - EQ_SCALE
    expected_positive = (H_mag * hinv_val + (1 << (eff_shift - 1))) >> eff_shift
    expected_negative = -(H_mag * hinv_val + (1 << (eff_shift - 1))) >> eff_shift

    for i in range(48):
        expected = expected_positive if (i % 2 == 0) else expected_negative
        # Allow small rounding tolerance
        assert abs(data_re[i] - expected) <= 2, \
            f"Data[{i}] (bin {DATA_BINS[i]}): got {data_re[i]}, expected {expected}"
        assert abs(data_im[i]) <= 2, \
            f"Data[{i}] imaginary: got {data_im[i]}, expected ~0"

    dut._log.info(f"Adaptive shift equalization passed: shift_val={shift_val}, "
                  f"H_inv={hinv_val}, output=±{expected_positive} (was ±1, now ±{EQ_GAIN})")


@cocotb.test()
async def test_symbol_done_after_pilots(dut):
    """symbol_done must not fire before the last pilot_valid pulse.

    Guards the equalizer drain invariant: "symbol_done ⇒ all 48 data +
    4 pilot outputs emitted". decode_engine gates eq_rd_sel on eq_done
    (which is this module's symbol_done), so an early symbol_done can
    desync the pipeline. A drain predicate that omits a pipeline stage
    (the pipe3a class of bug) is exactly the failure mode this catches —
    the value/count assertions elsewhere in this file cannot see a
    cycle-shift because their collector drains until it has 48+4 samples.

    Records the cycle index of the last pilot_valid pulse (pilot_idx==3,
    the last of the four pilots through the pipe) and the cycle index of
    the symbol_done pulse, and asserts strict ordering.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Marker values on active bins (contents irrelevant for ordering)
    fft_re = [0] * 64
    fft_im = [0] * 64
    for i, k in enumerate(DATA_BINS):
        fft_re[k] = 50 + i
        fft_im[k] = -(25 + i)
    for k in PILOT_BINS:
        fft_re[k] = 100
        fft_im[k] = 0

    # Unit channel
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = Q15 - 1
        hinv_im[k] = 0

    # Same feed sequence as run_equalization (pulse start, then 64 bins)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.clk)

    for i in range(64):
        dut.fft_valid.value = 1
        dut.fft_bin.value = i
        dut.fft_re.value = s16_to_bits(fft_re[i])
        dut.fft_im.value = s16_to_bits(fft_im[i])
        await RisingEdge(dut.clk)

    dut.fft_valid.value = 0

    # BRAM-latency H_inv delivery (same as run_equalization)
    last_rd_addr = int(dut.rd_addr.value) & 0x3F
    dut.hinv_re.value = s16_to_bits(hinv_re[last_rd_addr])
    dut.hinv_im.value = s16_to_bits(hinv_im[last_rd_addr])

    cycle = 0
    pilot_count = 0
    last_pilot_cycle = None
    done_cycle = None
    for _ in range(300):
        await RisingEdge(dut.clk)
        cycle += 1

        cur_rd_addr = int(dut.rd_addr.value) & 0x3F
        dut.hinv_re.value = s16_to_bits(hinv_re[cur_rd_addr])
        dut.hinv_im.value = s16_to_bits(hinv_im[cur_rd_addr])

        if int(dut.pilot_valid.value) == 1:
            pilot_count += 1
            if int(dut.pilot_idx.value) == 3:
                last_pilot_cycle = cycle
        if int(dut.symbol_done.value) == 1:
            done_cycle = cycle
            if pilot_count >= 4:
                break

    assert pilot_count == 4, f"Expected 4 pilot pulses, got {pilot_count}"
    assert last_pilot_cycle is not None, "last pilot (idx 3) never observed"
    assert done_cycle is not None, "symbol_done never observed"
    assert done_cycle > last_pilot_cycle, (
        f"symbol_done at cycle {done_cycle} does not follow the last "
        f"pilot_valid at cycle {last_pilot_cycle}: drain predicate fires "
        f"before the pipeline has emitted the final pilot (a pipeline "
        f"stage is missing from the S_DONE drain predicate)"
    )

    dut._log.info(f"symbol_done ordering OK: last pilot_valid at cycle "
                  f"{last_pilot_cycle}, symbol_done at cycle {done_cycle}")


@cocotb.test()
async def test_saturation_on_overflow(dut):
    """Equalizer output saturates instead of wrapping on extreme H_inv values.

    Simulates a deep channel fade where H_inv is very large (attempting to boost
    a nearly-zero subcarrier). The multiply result overflows 16-bit signed range.
    Without saturation, this wraps to wrong-sign garbage. With saturation, it
    clamps to ±32767.

    This is the "SimCity bug" scenario: rare in normal operation (requires a
    specific fade pattern), but guaranteed to occur given enough traffic.
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Scenario: deep fade where H is very small (e.g. 50) but FFT output is still
    # moderate (e.g. 200). chan_est produces large H_inv to compensate.
    # With shift_val=15: H_inv = (50 << 15) / (50^2) = 1638400/2500 = 655
    # eq = 200 * 655 = 131000 >> (15-6) = 131000 >> 9 = 255 — fine, no overflow.
    #
    # But with a deeper fade (H=10): H_inv = (10 << 15) / 100 = 3276
    # and FFT output = 5000 (from adjacent strong subcarrier leakage):
    # eq = 5000 * 3276 = 16,380,000 >> 9 = 31,992 — barely fits.
    #
    # Pathological case: H_inv = 32767 (max), Y = 32767:
    # eq_full = 32767 * 32767 = 1,073,676,289 (32-bit signed OK)
    # shifted = 1,073,676,289 >> 9 = 2,096,242 — way over 32767.
    # Without saturation: wraps. With saturation: clamps to 32767.

    shift_val = 15
    dut.shift_val.value = shift_val

    # Create pathological input: max magnitude on all data bins
    fft_re = [0] * 64
    fft_im = [0] * 64
    for k in DATA_BINS:
        fft_re[k] = 32767  # maximum positive
        fft_im[k] = 32767
    for k in PILOT_BINS:
        fft_re[k] = 32767
        fft_im[k] = 0

    # H_inv at maximum (simulating deep fade compensation attempt)
    hinv_re = [0] * 64
    hinv_im = [0] * 64
    for k in ACTIVE_BINS:
        hinv_re[k] = 32767  # max H_inv (deep fade)
        hinv_im[k] = 0

    (data_re, data_im), _ = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48, f"Expected 48, got {len(data_re)}"

    # Without saturation, eq = 32767*32767 = 1,073,676,289.
    # Complex: eq_full_re = rr - ii = 32767*32767 - 32767*0 = 1,073,676,289
    # eq_full_im = ri + ir = 32767*0 + 32767*32767 = 1,073,676,289
    # shifted = 1,073,676,289 >> 9 = 2,096,242 → OVERFLOWS 16-bit signed
    # Expected: saturates to +32767 (not wraps to some negative value)
    for i in range(48):
        assert data_re[i] == 32767, \
            f"Data[{i}] re: expected +32767 (saturated), got {data_re[i]} (wraparound?)"
        assert data_im[i] == 32767, \
            f"Data[{i}] im: expected +32767 (saturated), got {data_im[i]} (wraparound?)"

    dut._log.info("Saturation test passed (equalizer output saturates correctly)")

    # Now test negative saturation: large negative Y * large positive H_inv
    await reset_dut(dut)
    dut.shift_val.value = shift_val

    for k in DATA_BINS:
        fft_re[k] = -32768  # maximum negative
        fft_im[k] = -32768
    for k in PILOT_BINS:
        fft_re[k] = -32768
        fft_im[k] = 0

    (data_re, data_im), _ = await run_equalization(
        dut, fft_re, fft_im, hinv_re, hinv_im)

    assert len(data_re) == 48
    for i in range(48):
        assert data_re[i] == -32768, \
            f"Data[{i}] re: expected -32768 (saturated), got {data_re[i]}"
        assert data_im[i] == -32768, \
            f"Data[{i}] im: expected -32768 (saturated), got {data_im[i]}"

    dut._log.info("Negative saturation test passed")
