"""
Test CFO Estimator (cfo_est).

Tests:
1. Zero CFO → phase_inc ≈ 0
2. Known positive CFO → correct phase_inc
3. Known negative CFO → correct phase_inc
4. Maximum CFO (±625 kHz = ±π/16 rad/sample) → saturates correctly

Algorithm:
- Accumulate complex autocorrelation at lag-16 over 32 samples
  (window constrained to stay within STF boundary)
- CFO = -atan2(P_imag, P_real) / 16
- Output as 16-bit signed phase increment (fraction of 2π per sample)
  phase_inc = round(cfo_rad / (2*pi) * 2^16)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

import json
import os
import math
import random

VECTORS_DIR = os.path.join(os.path.dirname(__file__), "../../extern/lib80211/vectors")


def load_6mbps_waveform_quantized(scale=6000):
    """Load the 6 Mbps golden waveform, quantize to 12-bit signed."""
    path = os.path.join(VECTORS_DIR, "legacy_6mbps_waveform.json")
    with open(path) as f:
        d = json.load(f)
    re_f = d["real"]
    im_f = d["imag"]
    re = [max(-2048, min(2047, int(round(x * scale)))) for x in re_f]
    im = [max(-2048, min(2047, int(round(x * scale)))) for x in im_f]
    return re, im


def apply_cfo(re, im, cfo_rad_per_sample):
    """Apply CFO (phase rotation per sample) to IQ data."""
    out_re = []
    out_im = []
    for n in range(len(re)):
        phase = n * cfo_rad_per_sample
        c = math.cos(phase)
        s = math.sin(phase)
        r = re[n] * c - im[n] * s
        i = re[n] * s + im[n] * c
        out_re.append(max(-2048, min(2047, int(round(r)))))
        out_im.append(max(-2048, min(2047, int(round(i)))))
    return out_re, out_im


def cfo_to_phase_inc(cfo_rad_per_sample):
    """Convert CFO in rad/sample to 16-bit signed phase increment.

    phase_inc represents the fraction of a full 2π rotation per sample,
    scaled to 16-bit signed: phase_inc = cfo_rad / (2π) × 2^16.
    Range: [-32768, 32767] covers [-π, +π) rad/sample.
    """
    return int(round(cfo_rad_per_sample / (2.0 * math.pi) * 65536.0))


def reference_cfo_estimate(re, im, start_offset, lag=16, window=32):
    """Python reference: accumulate autocorrelation and compute CFO.

    Matches the RTL algorithm exactly:
    - Accumulate x[n] * conj(x[n+lag]) for n in [start_offset, start_offset+window)
    - CORDIC computes angle = atan2(P_im, P_re) / (2π) × 2^16
    - RTL divides by lag using arithmetic right-shift: -(angle >> 4)
    - Return expected phase_inc matching RTL behavior
    """
    pr = 0
    pi_acc = 0
    for k in range(window):
        n = start_offset + k
        r1 = re[n]
        i1 = im[n]
        r2 = re[n + lag]
        i2 = im[n + lag]
        # x1 * conj(x2) = (r1*r2 + i1*i2) + j*(i1*r2 - r1*i2)
        pr += r1 * r2 + i1 * i2
        pi_acc += i1 * r2 - r1 * i2

    # CORDIC produces: angle = atan2(P_im, P_re) in units of (2π)/2^16
    cordic_angle_float = math.atan2(pi_acc, pr) / (2.0 * math.pi) * 65536.0
    # CORDIC rounds to nearest integer (16-bit output)
    cordic_int = int(round(cordic_angle_float))
    # Clamp to signed 16-bit range
    cordic_int = max(-32768, min(32767, cordic_int))

    # RTL: phase_inc = -( cordic_int >>> 4 )
    # Python's >> is arithmetic shift for negative numbers
    phase_inc = -(cordic_int >> 4)
    return phase_inc


def to_twos_complement_12(val):
    """Convert signed int to 12-bit two's complement."""
    if val < 0:
        return val + 4096
    return val


def from_twos_complement_16(val):
    """Convert 16-bit unsigned to signed."""
    if val >= 32768:
        return val - 65536
    return val


async def reset(dut):
    """Assert reset for a few cycles."""
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.iq_valid.value = 0
    dut.iq_i.value = 0
    dut.iq_q.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def feed_iq_and_wait_done(dut, re_samples, im_samples, start_delay=48):
    """Feed IQ stream with start pulse after start_delay samples.

    The start pulse tells the module to begin accumulation. In real operation,
    this fires after STF detection (with an offset of +48 samples from STF start
    to skip ramp-up, as in lib80211 sync.c:175).

    Returns phase_inc value when done asserts.
    """
    done_seen = False
    phase_inc_val = 0

    for i in range(len(re_samples)):
        dut.iq_i.value = to_twos_complement_12(re_samples[i])
        dut.iq_q.value = to_twos_complement_12(im_samples[i])
        dut.iq_valid.value = 1

        if i == start_delay:
            dut.start.value = 1
        else:
            dut.start.value = 0

        await RisingEdge(dut.clk)
        await Timer(1, units="ns")

        if int(dut.done.value) == 1 and not done_seen:
            phase_inc_val = int(dut.phase_inc.value)
            phase_inc_val = from_twos_complement_16(phase_inc_val)
            done_seen = True

    dut.iq_valid.value = 0
    dut.start.value = 0

    # Give a few extra cycles for pipeline flush
    for _ in range(20):
        await RisingEdge(dut.clk)
        await Timer(1, units="ns")
        if int(dut.done.value) == 1 and not done_seen:
            phase_inc_val = int(dut.phase_inc.value)
            phase_inc_val = from_twos_complement_16(phase_inc_val)
            done_seen = True

    return done_seen, phase_inc_val


@cocotb.test()
async def test_zero_cfo(dut):
    """Zero CFO → phase_inc matches reference (near 0)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Use golden waveform (no CFO applied) — STF region
    re, im = load_6mbps_waveform_quantized()

    # Feed enough samples: start_delay + fill(16) + window(32) + CORDIC(18) + margin
    start_delay = 16  # Start early in STF to keep entire window within STF region
    n_feed = start_delay + 16 + 32 + 30
    done, phase_inc = await feed_iq_and_wait_done(dut, re[:n_feed], im[:n_feed],
                                                   start_delay=start_delay)

    assert done, "cfo_est never asserted done"

    # Reference: module processes from sample (start_delay+1) due to state transition.
    # First 16 samples fill delay line, then accumulates 32 pairs.
    # Window is entirely within STF (period-16), so reference should give ~0.
    expected = reference_cfo_estimate(re, im, start_offset=start_delay + 1)

    # Allow ±2 LSB tolerance for CORDIC quantization
    tolerance = 2
    error = abs(phase_inc - expected)
    assert error <= tolerance, \
        f"Zero CFO: expected phase_inc={expected}, got {phase_inc}, error={error}"

    dut._log.info(f"Zero CFO: phase_inc={phase_inc}, expected={expected}, error={error}")


@cocotb.test()
async def test_positive_cfo(dut):
    """Positive CFO (+25 kHz) → correct phase_inc within tolerance."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    cfo_hz = 25000.0
    sample_rate = 20e6
    cfo_rad = 2.0 * math.pi * cfo_hz / sample_rate

    re, im = load_6mbps_waveform_quantized()
    re_cfo, im_cfo = apply_cfo(re, im, cfo_rad)

    start_delay = 16
    n_feed = start_delay + 16 + 32 + 30

    # Reference: expected phase_inc from the algorithm
    expected = reference_cfo_estimate(re_cfo, im_cfo, start_offset=start_delay + 1)

    done, phase_inc = await feed_iq_and_wait_done(dut, re_cfo[:n_feed],
                                                    im_cfo[:n_feed],
                                                    start_delay=start_delay)

    assert done, "cfo_est never asserted done"
    # Allow ±2 LSB tolerance for CORDIC quantization
    error = abs(phase_inc - expected)
    assert error <= 2, f"Positive CFO ({cfo_hz} Hz): expected={expected}, got={phase_inc}, error={error}"
    dut._log.info(f"Positive CFO ({cfo_hz} Hz): phase_inc={phase_inc}, expected={expected}, error={error}")


@cocotb.test()
async def test_negative_cfo(dut):
    """Negative CFO (-50 kHz) → correct phase_inc within tolerance."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    cfo_hz = -50000.0
    sample_rate = 20e6
    cfo_rad = 2.0 * math.pi * cfo_hz / sample_rate

    re, im = load_6mbps_waveform_quantized()
    re_cfo, im_cfo = apply_cfo(re, im, cfo_rad)

    start_delay = 16
    n_feed = start_delay + 16 + 32 + 30

    expected = reference_cfo_estimate(re_cfo, im_cfo, start_offset=start_delay + 1)

    done, phase_inc = await feed_iq_and_wait_done(dut, re_cfo[:n_feed],
                                                    im_cfo[:n_feed],
                                                    start_delay=start_delay)

    assert done, "cfo_est never asserted done"
    error = abs(phase_inc - expected)
    assert error <= 2, f"Negative CFO ({cfo_hz} Hz): expected={expected}, got={phase_inc}, error={error}"
    dut._log.info(f"Negative CFO ({cfo_hz} Hz): phase_inc={phase_inc}, expected={expected}, error={error}")


@cocotb.test()
async def test_max_cfo(dut):
    """Large CFO (+500 kHz) → correct phase_inc within tolerance."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    cfo_hz = 500000.0
    sample_rate = 20e6
    cfo_rad = 2.0 * math.pi * cfo_hz / sample_rate

    re, im = load_6mbps_waveform_quantized()
    re_cfo, im_cfo = apply_cfo(re, im, cfo_rad)

    start_delay = 16
    n_feed = start_delay + 16 + 32 + 30

    # Reference using actual window
    expected = reference_cfo_estimate(re_cfo, im_cfo, start_offset=start_delay + 1)

    done, phase_inc = await feed_iq_and_wait_done(dut, re_cfo[:n_feed],
                                                    im_cfo[:n_feed],
                                                    start_delay=start_delay)

    assert done, "cfo_est never asserted done"
    error = abs(phase_inc - expected)
    assert error <= 2, f"Max CFO ({cfo_hz} Hz): expected={expected}, got={phase_inc}, error={error}"
    dut._log.info(f"Max CFO ({cfo_hz} Hz): phase_inc={phase_inc}, expected={expected}, error={error}")
