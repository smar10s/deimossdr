"""
test_channel_model.py — Gate test: channel model scenarios with named impairment profiles.

WHAT THIS TESTS:
  The rx_frontend pipeline's ability to decode multi-frame streams through
  realistic channel impairments: near-far power differences, multipath,
  CFO, SFO, and mixed traffic patterns. Uses the py80211.channel composable
  model to generate coherent impaired streams.

WHY THIS EXISTS:
  The power-ratio diagnostic (diag_power_ratio) established that the fabric
  handles up to ~10 dB power ratio between consecutive frames. This gate test
  locks in conservative scenarios (well within that envelope) as permanent
  regression gates. If the STF detector, channel estimation, or pilot tracking
  regresses, these scenarios will break.

SCENARIOS:
  1. Near-far 6 dB: EAPOL handshake with AP/STA amplitude difference
  2. Indoor office: Mixed beacons+data through moderate multipath + CFO
  3. Mixed rates: Multi-rate burst through clean channel (cable-equivalent)
  4. EAPOL multipath: Same-power handshake through mild multipath + CFO

GATE CRITERIA:
  Each scenario specifies a minimum FCS pass rate and is run multiple trials.
  The test fails if any scenario drops below its threshold.

INTEGRATION:
  Gate test (test_*). Runs in sim.sh. DUT = rx_frontend.
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    quantize_12bit, reset_dut, s12_to_unsigned, SAMPLE_RATE,
)

# Add lib80211 for channel model
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.channel import (
    ChannelConfig, FrameSpec, generate_stream,
    eapol_handshake, beacon_data_mix,
)


# =========================================================
# Feed + Collect Helper
# =========================================================

async def feed_and_collect_tags(dut, samples, n_expected_frames, timeout_cycles=8_000_000):
    """Feed quantized IQ stream and collect tag outputs (multi-frame).

    Feeds at 1-per-5 clock rate. After all samples, feeds 8192 zeros to flush.
    Returns list of tag dicts in decode order.
    """
    n_samples = len(samples)
    post_zeros = 8192
    total_samples = n_samples + post_zeros

    tags = []
    sample_idx = 0
    valid_counter = 0

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed IQ at 1-per-5 clock rate
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < total_samples:
                # Post-stream: feed zeros to flush pipeline
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Check for tag output
        try:
            if int(dut.tag_valid.value) == 1:
                tag = {
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                }
                tags.append(tag)
                if len(tags) >= n_expected_frames:
                    break
        except (ValueError, AttributeError):
            pass

        # Progress logging
        if cycle > 0 and cycle % 2_000_000 == 0:
            dut._log.info(f"  cycle {cycle}: {len(tags)}/{n_expected_frames} tags, "
                          f"sample {sample_idx}/{n_samples}")

    return tags


# =========================================================
# Scenario Runner
# =========================================================

async def run_scenario(dut, name, frames, config, n_expected, min_fcs_ok,
                       trials=3, timeout_cycles=8_000_000):
    """Run a channel model scenario multiple trials and assert FCS threshold.

    Args:
        dut: cocotb DUT handle
        name: scenario name for logging
        frames: list of FrameSpec
        config: ChannelConfig
        n_expected: number of frames expected per trial
        min_fcs_ok: minimum FCS-OK frames required per trial
        trials: number of independent trials (different seeds)
        timeout_cycles: sim timeout per trial

    Raises:
        AssertionError if any trial fails to meet min_fcs_ok threshold.
    """
    dut._log.info(f"{'='*60}")
    dut._log.info(f"SCENARIO: {name}")
    dut._log.info(f"  Frames: {n_expected}, min FCS OK: {min_fcs_ok}, trials: {trials}")
    dut._log.info(f"  Channel: SNR={config.snr_db} dB, CFO={config.cfo_hz} Hz, "
                  f"SFO={config.sfo_ppm} ppm")
    if config.multipath_taps != [(0, 1.0 + 0j)]:
        dut._log.info(f"  Multipath: {config.multipath_taps}")
    dut._log.info(f"{'='*60}")

    trial_results = []

    for trial in range(trials):
        await reset_dut(dut)

        seed = hash(name) % 10000 + trial * 100
        iq_stream, metadata = generate_stream(frames, config, seed=seed)
        samples = quantize_12bit(iq_stream)

        dut._log.info(f"  Trial {trial+1}/{trials}: {len(samples)} samples "
                      f"({len(samples)/SAMPLE_RATE*1000:.1f} ms), seed={seed}")

        tags = await feed_and_collect_tags(dut, samples, n_expected,
                                           timeout_cycles=timeout_cycles)

        fcs_ok_count = sum(1 for t in tags if t['fcs_ok'])
        detected = len(tags)

        dut._log.info(f"    Detected: {detected}/{n_expected}, "
                      f"FCS OK: {fcs_ok_count}/{detected}")
        for i, t in enumerate(tags):
            fcs_str = 'OK' if t['fcs_ok'] else 'FAIL'
            dut._log.info(f"      Tag {i}: rate=0b{t['rate']:04b}, "
                          f"len={t['length']}, fcs={fcs_str}")

        trial_results.append({
            'detected': detected,
            'fcs_ok': fcs_ok_count,
            'tags': tags,
        })

        assert fcs_ok_count >= min_fcs_ok, \
            f"SCENARIO '{name}' trial {trial+1}: " \
            f"FCS OK {fcs_ok_count}/{detected} < threshold {min_fcs_ok}/{n_expected}. " \
            f"Tags: {[(t['length'], 'OK' if t['fcs_ok'] else 'FAIL') for t in tags]}"

    # Summary
    avg_fcs = sum(r['fcs_ok'] for r in trial_results) / trials
    dut._log.info(f"  PASS: {name} — avg FCS OK {avg_fcs:.1f}/{n_expected} "
                  f"(threshold {min_fcs_ok})")


# =========================================================
# Gate Tests
# =========================================================

@cocotb.test()
async def test_channel_near_far_6db(dut):
    """EAPOL handshake with 6 dB AP/STA power difference — well within 10 dB limit.

    AP frames at amplitude=1.0, STA frames at amplitude=0.5 (6 dB below).
    Mild impairments (no multipath, no AGC ramp) — isolates near-far effect.
    All 4 frames must decode (threshold = 100%).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    frames = eapol_handshake(ap_amplitude=1.0, sta_amplitude=0.5)

    config = ChannelConfig(
        snr_db=35.0,
        cfo_hz=1000.0,
        sfo_ppm=2.0,
    )

    await run_scenario(
        dut,
        name="near_far_6db",
        frames=frames,
        config=config,
        n_expected=4,
        min_fcs_ok=4,
        trials=3,
    )


@cocotb.test()
async def test_channel_indoor_office(dut):
    """Mixed beacons + data through indoor office channel (moderate impairments).

    2 beacons (rate 6, 350B) + 3 data (rate 24, 200B) with 2 dB power difference.
    Moderate multipath, CFO, and SFO. Threshold: 80% (4/5 minimum).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    frames = beacon_data_mix(
        n_beacons=2,
        n_data=3,
        beacon_amplitude=1.0,
        data_amplitude=0.8,
    )

    config = ChannelConfig(
        snr_db=28.0,
        cfo_hz=2000.0,
        sfo_ppm=3.0,
        multipath_taps=[(0, 1.0 + 0j), (4, -0.25 + 0.1j)],
    )

    await run_scenario(
        dut,
        name="indoor_office",
        frames=frames,
        config=config,
        n_expected=5,
        min_fcs_ok=4,
        trials=3,
    )


@cocotb.test()
async def test_channel_mixed_rates(dut):
    """Multi-rate burst through clean channel (cable-equivalent).

    Rate 6, 12, 24, 36 at 100B each, all amplitude=1.0, DIFS gaps.
    Clean channel (high SNR, low CFO/SFO, no multipath).
    All 4 must decode (threshold = 100%).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    from py80211.channel import DIFS_SAMPLES
    frames = [
        FrameSpec(rate_mbps=6,  psdu_len=100, amplitude=1.0, gap_samples=DIFS_SAMPLES),
        FrameSpec(rate_mbps=12, psdu_len=100, amplitude=1.0, gap_samples=DIFS_SAMPLES),
        FrameSpec(rate_mbps=24, psdu_len=100, amplitude=1.0, gap_samples=DIFS_SAMPLES),
        FrameSpec(rate_mbps=36, psdu_len=100, amplitude=1.0, gap_samples=DIFS_SAMPLES),
    ]

    config = ChannelConfig(
        snr_db=35.0,
        cfo_hz=500.0,
        sfo_ppm=1.0,
    )

    await run_scenario(
        dut,
        name="mixed_rates",
        frames=frames,
        config=config,
        n_expected=4,
        min_fcs_ok=4,
        trials=3,
    )


@cocotb.test()
async def test_channel_deep_fade_multipath(dut):
    """Rate 6 + 24 through strong multipath: taps [(0, 1.0), (4, -0.5)].

    The -6 dB tap produces ~9.5 dB of |H| ripple across the band. With the
    legacy chan_est shift (sv = 14 + bit_w/2), bins >6 dB below the strongest
    bin clamp H_inv at +-32767 — fade-side subcarriers are mis-equalized and
    24 Mbps frames FCS-fail. The sv=10 shift extends clamp-free range by
    +24 dB (see test_chan_est deep-fade unit test + clip counter).

    Frames: 1 beacon (rate 6, 200B) + 2 data (rate 24, 200B), DIFS gaps.
    Threshold: all 3 must FCS-OK.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    from py80211.channel import DIFS_SAMPLES
    frames = [
        FrameSpec(rate_mbps=6,  psdu_len=200, amplitude=1.0, gap_samples=DIFS_SAMPLES),
        FrameSpec(rate_mbps=24, psdu_len=200, amplitude=1.0, gap_samples=DIFS_SAMPLES),
        FrameSpec(rate_mbps=24, psdu_len=200, amplitude=1.0, gap_samples=DIFS_SAMPLES),
    ]

    config = ChannelConfig(
        snr_db=30.0,
        cfo_hz=1000.0,
        sfo_ppm=2.0,
        multipath_taps=[(0, 1.0 + 0j), (4, -0.5 + 0j)],
    )

    await run_scenario(
        dut,
        name="deep_fade_multipath",
        frames=frames,
        config=config,
        n_expected=3,
        min_fcs_ok=3,
        trials=3,
    )


@cocotb.test()
async def test_channel_eapol_multipath(dut):
    """EAPOL handshake through mild multipath (same power, moderate CFO).

    All frames at amplitude=1.0 (no near-far), but with moderate multipath
    and higher CFO. Tests channel estimation and pilot tracking under
    frequency-selective fading.
    Threshold: 75% (3/4 minimum).
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    frames = eapol_handshake(ap_amplitude=1.0, sta_amplitude=1.0)

    config = ChannelConfig(
        snr_db=30.0,
        cfo_hz=2500.0,
        sfo_ppm=4.0,
        multipath_taps=[(0, 1.0 + 0j), (3, -0.3 + 0.1j)],
    )

    await run_scenario(
        dut,
        name="eapol_multipath",
        frames=frames,
        config=config,
        n_expected=4,
        min_fcs_ok=3,
        trials=5,
    )
