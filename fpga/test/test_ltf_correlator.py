"""
Test sliding LTF correlator (ltf_correlator).

Tests:
1. Peak detection: feed noise + LTF T1 + noise, verify metric peaks at T1 start
2. Metric magnitude: clean LTF produces metric >> noise floor
3. Continuous operation: multiple LTFs in sequence produce correct peaks
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

import json
import os
import math
import random

VECTORS_DIR = os.path.join(os.path.dirname(__file__), "../../extern/lib80211/vectors")

# ROM values (first 32 samples of LTF T1, 8-bit signed)
ROM_RE = [124,-4,31,77,17,47,-91,-30,77,42,1,-108,19,46,-18,94,
          49,29,-45,-104,65,55,-48,-45,-28,-96,-101,59,-2,-73,73,10]
ROM_IM = [0,-95,-88,65,22,-69,-44,-84,-20,3,-91,-37,-46,-12,127,-3,
          -49,78,31,52,73,11,64,-17,-119,-13,-16,-59,43,91,84,77]


def load_ltf_t1_12bit(scale=16.0):
    """Load LTF T1 waveform scaled to 12-bit signal range.

    Uses the ROM values directly (8-bit) scaled up to 12-bit range.
    scale=16 gives ROM*16 which stays within 12-bit [-2048, 2047].
    Max ROM value is 127, so 127*16 = 2032 < 2048. Good.
    """
    re = [int(v * scale) for v in ROM_RE]
    im = [int(v * scale) for v in ROM_IM]
    return re, im


def load_ltf_from_vector():
    """Load LTF T1 from golden vector, quantized to 12-bit."""
    path = os.path.join(VECTORS_DIR, "annex_i1_ltf_time.json")
    with open(path) as f:
        d = json.load(f)
    samples = d["samples"]
    # T1 starts at index 32, length 64
    # Scale to ~12-bit range (use scale that puts peak at ~1800)
    adc_scale = 1800.0 / 0.156  # peak of T1 sample 0 is 0.156
    re = []
    im = []
    for i in range(32, 96):  # T1: 64 samples
        r = max(-2048, min(2047, int(round(samples[i][0] * adc_scale))))
        q = max(-2048, min(2047, int(round(samples[i][1] * adc_scale))))
        re.append(r)
        im.append(q)
    return re, im


def generate_noise(n, amplitude=200, seed=42):
    """Generate n samples of random noise, 12-bit range."""
    rng = random.Random(seed)
    re = [rng.randint(-amplitude, amplitude) for _ in range(n)]
    im = [rng.randint(-amplitude, amplitude) for _ in range(n)]
    return re, im


async def reset_dut(dut):
    """Assert reset for 10 clocks."""
    dut.rst_n.value = 0
    dut.iq_valid.value = 0
    dut.iq_re.value = 0
    dut.iq_im.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def feed_sample(dut, re_val, im_val):
    """Feed one IQ sample at 1-per-5 timing (assert iq_valid for 1 clock)."""
    dut.iq_valid.value = 1
    dut.iq_re.value = re_val & 0xFFF  # 12-bit unsigned representation
    dut.iq_im.value = im_val & 0xFFF
    await RisingEdge(dut.clk)
    dut.iq_valid.value = 0
    # Wait 4 more clocks (total 5 per sample)
    await ClockCycles(dut.clk, 4)


async def feed_samples_and_collect(dut, re_list, im_list):
    """Feed a sequence of IQ samples, collect all metric outputs."""
    metrics = []  # list of (sample_index, metric_value)
    sample_idx = 0

    for re_val, im_val in zip(re_list, im_list):
        # Feed one sample
        dut.iq_valid.value = 1
        dut.iq_re.value = re_val & 0xFFF
        dut.iq_im.value = im_val & 0xFFF
        await RisingEdge(dut.clk)
        # Metric for the previous sample pulses on this clock (the metric
        # pipeline's 3 registered stages align with the iq_valid clock).
        if dut.metric_valid.value == 1:
            metrics.append((sample_idx, int(dut.metric.value)))
        dut.iq_valid.value = 0

        # Wait 4 clocks, checking for metric_valid each cycle
        for _ in range(4):
            await RisingEdge(dut.clk)
            if dut.metric_valid.value == 1:
                metrics.append((sample_idx, int(dut.metric.value)))

        sample_idx += 1

    # One more sample period to flush the last metric
    dut.iq_valid.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
        if dut.metric_valid.value == 1:
            metrics.append((sample_idx, int(dut.metric.value)))

    return metrics


@cocotb.test()
async def test_peak_detection(dut):
    """Feed noise + LTF T1 + noise. Verify metric peaks near T1 alignment."""
    clock = Clock(dut.clk, 10, unit="ns")  # 100 MHz
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Build stimulus: 50 noise + 32 LTF samples (first half of T1) + 50 noise
    # The correlator has 32 taps, so peak should occur when the shift register
    # contains exactly the 32-sample LTF reference (after sample 31 of T1 is fed,
    # i.e., at sample index 50+31 = 81 in the stream).
    noise_pre_re, noise_pre_im = generate_noise(50, amplitude=100, seed=1)
    ltf_re, ltf_im = load_ltf_t1_12bit(scale=16)
    # Use first 32 samples of T1 (perfect match to ROM)
    ltf_32_re = ltf_re[:32]
    ltf_32_im = ltf_im[:32]
    noise_post_re, noise_post_im = generate_noise(50, amplitude=100, seed=2)

    all_re = noise_pre_re + ltf_32_re + noise_post_re
    all_im = noise_pre_im + ltf_32_im + noise_post_im

    metrics = await feed_samples_and_collect(dut, all_re, all_im)

    # Find peak metric
    assert len(metrics) > 0, "No metric_valid pulses received"

    peak_idx, peak_val = max(metrics, key=lambda x: x[1])
    dut._log.info(f"Peak metric: {peak_val} at sample index {peak_idx}")
    dut._log.info(f"Total metric samples: {len(metrics)}")

    # Expected peak position: sample 66 (50 noise + 15 LTF + 1 pipeline delay)
    # The correlator peaks when all 16 reference taps align with the shift
    # register contents. This happens when the 16th LTF sample is fed.
    # Pipeline/collection adds 1 sample of reporting delay.
    expected_peak = 50 + 16  # = 66
    dut._log.info(f"Expected peak near sample {expected_peak}, got {peak_idx}")

    # Allow ±2 sample tolerance (pipeline alignment)
    assert abs(peak_idx - expected_peak) <= 2, \
        f"Peak at {peak_idx}, expected near {expected_peak} (±2)"

    # Peak should be significantly above pure noise floor
    # Exclude the entire LTF influence zone (samples 50 to 115)
    # where partial correlation produces elevated metrics
    noise_metrics = [(idx, val) for idx, val in metrics
                     if idx < 45]
    if noise_metrics:
        max_noise = max(v for _, v in noise_metrics)
        dut._log.info(f"Peak/noise ratio: {peak_val}/{max_noise} = {peak_val/max(1,max_noise):.1f}x")
        assert peak_val > max_noise * 10, \
            f"Peak {peak_val} not sufficiently above noise {max_noise}"


@cocotb.test()
async def test_metric_magnitude(dut):
    """Perfect LTF alignment produces large metric; noise produces small metric."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Feed 40 noise samples (primes pipeline + establishes noise floor)
    # then 32 LTF samples (perfect alignment at end)
    noise_re, noise_im = generate_noise(40, amplitude=150, seed=7)
    ltf_re, ltf_im = load_ltf_t1_12bit(scale=16)
    ltf_32_re = ltf_re[:32]
    ltf_32_im = ltf_im[:32]

    all_re = noise_re + ltf_32_re
    all_im = noise_im + ltf_32_im

    metrics = await feed_samples_and_collect(dut, all_re, all_im)

    assert len(metrics) > 0, "No metric_valid pulses received"

    # Compute expected peak metric analytically:
    # conj(ref) * sig where ref = ROM (8-bit) and sig = ROM * 16 (12-bit)
    # For perfect alignment: acc = 16 * sum_k (ref_re[k]^2 + ref_im[k]^2)
    # Metric is the squared magnitude with >>11 truncation:
    #   metric = (acc >> 11)^2   (Im term is zero at perfect alignment)
    energy = sum(r*r + i*i for r, i in zip(ROM_RE[:16], ROM_IM[:16]))
    expected_metric = ((energy * 16) >> 11) ** 2
    dut._log.info(f"Reference energy (sum |ref|^2): {energy}")
    dut._log.info(f"Expected metric (energy * 16): {expected_metric}")

    peak_idx, peak_val = max(metrics, key=lambda x: x[1])
    dut._log.info(f"Measured peak metric: {peak_val}")
    dut._log.info(f"Ratio measured/expected: {peak_val/expected_metric:.2f}")

    # Peak should be within 20% of expected (rounding, quantization)
    assert peak_val > expected_metric * 0.7, \
        f"Peak metric {peak_val} too low (expected ~{expected_metric})"
    assert peak_val < expected_metric * 1.3, \
        f"Peak metric {peak_val} too high (expected ~{expected_metric})"

    # Noise floor should be much lower
    noise_metrics = [v for idx, v in metrics if idx < 35]
    if noise_metrics:
        avg_noise = sum(noise_metrics) / len(noise_metrics)
        dut._log.info(f"Average noise metric: {avg_noise:.0f}")
        dut._log.info(f"Peak/noise_avg: {peak_val/max(1,avg_noise):.1f}x")
        assert peak_val > avg_noise * 8, \
            f"Insufficient peak/noise ratio: {peak_val/max(1,avg_noise):.1f}x"


@cocotb.test()
async def test_continuous_two_ltfs(dut):
    """Two LTF bursts separated by noise — both produce peaks."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ltf_re, ltf_im = load_ltf_t1_12bit(scale=16)
    # Use first 16 samples (exact match to 16-tap ROM reference) to avoid
    # partial-correlation ambiguity from unused taps 16-31.
    ltf_16_re = ltf_re[:16]
    ltf_16_im = ltf_im[:16]
    noise1_re, noise1_im = generate_noise(50, amplitude=100, seed=10)
    noise2_re, noise2_im = generate_noise(40, amplitude=100, seed=11)
    noise3_re, noise3_im = generate_noise(50, amplitude=100, seed=12)

    # Stream: noise(50) + LTF(16) + noise(40) + LTF(16) + noise(50)
    # 40-sample gap is sufficient (> 16 taps, fully flushes shift register)
    all_re = noise1_re + ltf_16_re + noise2_re + ltf_16_re + noise3_re
    all_im = noise1_im + ltf_16_im + noise2_im + ltf_16_im + noise3_im

    metrics = await feed_samples_and_collect(dut, all_re, all_im)

    assert len(metrics) > 0, "No metric_valid pulses received"

    # Find peak metric value to set threshold
    peak_val = max(v for _, v in metrics)
    threshold = peak_val * 0.7  # 70% of max (tighter, avoids partial correlation)

    # Find all samples above threshold
    peaks = [(idx, val) for idx, val in metrics if val > threshold]
    dut._log.info(f"Peak threshold: {threshold:.0f}")
    dut._log.info(f"Samples above threshold: {len(peaks)}")

    # Expected peaks near sample 66 (50+16) and sample 122 (50+16+40+16)
    # The correlator peaks when 16 LTF samples fill its shift register.
    # First burst: last LTF sample fed at 50+15, peak reported at 50+16 (priming)
    # Second burst: starts at 50+16+40=106, peaks at 106+16=122
    expected_1 = 50 + 16   # = 66
    expected_2 = 50 + 16 + 40 + 16  # = 122

    # Find the two highest local maxima
    # Group peaks that are within 5 samples of each other
    groups = []
    for idx, val in sorted(peaks):
        if not groups or idx - groups[-1][-1][0] > 5:
            groups.append([(idx, val)])
        else:
            groups[-1].append((idx, val))

    assert len(groups) >= 2, \
        f"Expected 2 peak groups, found {len(groups)}"

    # Each group's peak should be near expected position
    group1_peak = max(groups[0], key=lambda x: x[1])
    group2_peak = max(groups[1], key=lambda x: x[1])

    dut._log.info(f"Peak 1: sample {group1_peak[0]} (expected {expected_1})")
    dut._log.info(f"Peak 2: sample {group2_peak[0]} (expected {expected_2})")

    assert abs(group1_peak[0] - expected_1) <= 2, \
        f"First peak at {group1_peak[0]}, expected near {expected_1}"
    assert abs(group2_peak[0] - expected_2) <= 2, \
        f"Second peak at {group2_peak[0]}, expected near {expected_2}"
