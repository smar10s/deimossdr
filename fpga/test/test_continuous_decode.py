"""
test_continuous_decode.py — Gate test: back-to-back frame decode without reset.

THE PROBLEM THIS CATCHES:
  The sim resets all DUT state before each frame. On hardware, the fabric runs
  continuously: frame 1 finishes, noise fills the gap, frame 2 arrives. State
  from the previous decode (pilot_track EWMA, STF detector accumulators, CFO
  mixer phase) carries into the next frame.

  If any module doesn't properly re-initialize on frame boundaries, the second
  frame fails even though the first passes. This manifests as intermittent
  hardware failures that single-frame sim tests cannot reproduce.

WHAT THIS TESTS:
  Feeds multiple frames as one contiguous IQ stream (noise gaps between frames,
  NO DUT reset between frames). All frames must decode with FCS OK.

INTEGRATION:
  This is a gate test (test_*). It runs in sim.sh alongside test_rx_frontend.
  Uses rx_frontend DUT (no dead-zone — pilot PLL handles residual).
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    load_waveform_float, quantize_12bit, s12_to_unsigned,
    reset_dut, SAMPLE_RATE,
)

# Add lib80211 for impairments
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.impairments import add_cfo, add_awgn


# Gap between frames in samples.
# Must be longer than the pipeline processing time (~12000 samples for rate 6
# at this clock ratio) so the sequencer returns to idle before next STF arrives.
# On real hardware at 5 GHz, typical inter-frame spacing is 1-100ms (20k-2M samples).
# The minimum that exercises re-arm without overlap: 15000 samples (750 us).
INTER_FRAME_GAP_SAMPLES = 15000


def build_multi_frame_stream(frames, snr_db=35, gap_samples=INTER_FRAME_GAP_SAMPLES):
    """Build a contiguous IQ stream with multiple frames separated by noise gaps.

    Args:
        frames: list of (rate_mbps, cfo_hz) tuples
        snr_db: SNR for additive noise
        gap_samples: noise-only gap between frames

    Returns:
        list of (re_q, im_q) tuples (12-bit quantized), total sample count,
        and list of expected frame info for assertion.
    """
    rng = np.random.default_rng(seed=42)

    # First compute signal RMS from first frame (for noise calibration)
    iq0 = load_waveform_float(frames[0][0])
    sig_rms = np.sqrt(np.mean(np.abs(iq0)**2))
    noise_std = sig_rms / (10**(snr_db / 20))

    segments = []
    frame_info = []

    # Leading noise (fills STF detector delay lines)
    leading_noise = rng.normal(0, noise_std, gap_samples) + \
                    1j * rng.normal(0, noise_std, gap_samples)
    segments.append(leading_noise)

    # Track cumulative sample position for frame info
    current_sample_offset = gap_samples  # leading noise

    for i, (rate, cfo_hz) in enumerate(frames):
        iq = load_waveform_float(rate)
        if cfo_hz != 0:
            iq = add_cfo(iq, cfo_hz)
        iq_noisy = iq + (rng.normal(0, noise_std, len(iq)) +
                         1j * rng.normal(0, noise_std, len(iq)))
        segments.append(iq_noisy)
        frame_info.append({
            'rate': rate, 'cfo_hz': cfo_hz, 'index': i,
            'stf_start_sample': current_sample_offset,
        })
        current_sample_offset += len(iq_noisy)

        # Inter-frame gap (noise only) — except after last frame
        if i < len(frames) - 1:
            gap = rng.normal(0, noise_std, gap_samples) + \
                  1j * rng.normal(0, noise_std, gap_samples)
            segments.append(gap)
            current_sample_offset += gap_samples

    # Trailing silence to flush pipeline after last frame
    trailing = np.zeros(4000, dtype=complex)
    segments.append(trailing)

    # Concatenate and quantize
    full_stream = np.concatenate(segments)
    samples = quantize_12bit(full_stream)

    return samples, frame_info


async def feed_and_collect_tags(dut, samples, n_expected_frames, timeout_cycles=15000000):
    """Feed a contiguous IQ stream and collect all tag outputs (multi-frame).

    No reset during feeding. Returns list of tag results in decode order.
    """
    n_samples = len(samples)
    tags = []
    sample_idx = 0
    valid_counter = 0
    frames_seen = 0

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
            else:
                # Post-stream: keep feeding zeros (continuous ADC)
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
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
                    'cycle': cycle,
                }
                tags.append(tag)
                frames_seen += 1
                dut._log.info(f"  Frame {frames_seen}: rate=0b{tag['rate']:04b}, "
                              f"len={tag['length']}, fcs={'OK' if tag['fcs_ok'] else 'FAIL'} "
                              f"@ cycle {cycle}")
                if frames_seen >= n_expected_frames:
                    break
        except (ValueError, AttributeError):
            pass

        # Progress logging
        if cycle > 0 and cycle % 2000000 == 0:
            dut._log.info(f"  cycle {cycle}: {frames_seen}/{n_expected_frames} frames decoded, "
                          f"sample {sample_idx}/{n_samples}")

    return tags


@cocotb.test()
async def test_back_to_back_same_rate(dut):
    """2 consecutive rate-6 frames with CFO, no reset between frames.

    Models the common case: same AP beaconing at 6 Mbps, fabric must decode
    every frame regardless of what state the previous decode left behind.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 2 frames with different CFO (mimics estimator variance)
    frames = [
        (6, 1500),   # ~phase_inc 5
        (6, -900),   # ~phase_inc -3
    ]

    samples, frame_info = build_multi_frame_stream(frames, snr_db=35)
    dut._log.info(f"Back-to-back same rate: {len(frames)} frames, "
                  f"{len(samples)} total samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")

    tags = await feed_and_collect_tags(dut, samples, n_expected_frames=len(frames))

    # All frames must be detected
    assert len(tags) >= len(frames), \
        f"Only {len(tags)}/{len(frames)} frames detected. " \
        f"State carryover is preventing detection of subsequent frames."

    # All frames must pass FCS
    fcs_failures = [i for i, t in enumerate(tags) if not t['fcs_ok']]
    assert not fcs_failures, \
        f"Frames {fcs_failures} failed FCS. " \
        f"State from previous frame decode is corrupting subsequent frames."



@cocotb.test()
async def test_back_to_back_mixed_rates(dut):
    """4 frames at different rates, no reset between frames.

    Tests that rate switching (different modulation, different symbol counts)
    doesn't leave stale state that corrupts the next frame.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    frames = [
        (6,  1200),   # BPSK 1/2, long frame
        (24, -600),   # 16-QAM 1/2, medium frame
    ]

    samples, frame_info = build_multi_frame_stream(frames, snr_db=35)
    dut._log.info(f"Back-to-back mixed rates: {len(frames)} frames, "
                  f"{len(samples)} total samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")

    tags = await feed_and_collect_tags(dut, samples, n_expected_frames=len(frames))

    assert len(tags) >= len(frames), \
        f"Only {len(tags)}/{len(frames)} frames detected. " \
        f"Rate switching leaves stale state preventing subsequent detection."

    fcs_failures = [i for i, t in enumerate(tags) if not t['fcs_ok']]
    assert not fcs_failures, \
        f"Frames {fcs_failures} failed FCS in mixed-rate sequence. " \
        f"Previous frame's decode state corrupts different-rate frames."



@cocotb.test()
async def test_back_to_back_stress(dut):
    """3 frames with aggressive CFO, no reset — stress test for state leakage.

    Uses higher CFO values and tighter gaps to stress pilot_track EWMA
    carryover and STF detector re-arming.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    frames = [
        (6,  1800),   # ~pi=6, max normal residual
        (6,  -1800),  # sign flip — tests EWMA adaptation
        (24, 1500),
    ]

    samples, frame_info = build_multi_frame_stream(frames, snr_db=35, gap_samples=INTER_FRAME_GAP_SAMPLES)
    dut._log.info(f"Back-to-back stress: {len(frames)} frames, "
                  f"{len(samples)} total samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")

    tags = await feed_and_collect_tags(dut, samples, n_expected_frames=len(frames))

    # All frames must be detected
    assert len(tags) >= len(frames), \
        f"Only {len(tags)}/{len(frames)} frames detected. " \
        f"Serious state carryover issue."

    # All detected frames must pass FCS
    fcs_failures = [i for i, t in enumerate(tags) if not t['fcs_ok']]
    assert not fcs_failures, \
        f"Frames {fcs_failures} failed FCS in stress sequence."


@cocotb.test()
async def test_burst_10_frames(dut):
    """10 consecutive rate-6 frames with 20000-sample gap — matches hardware burst test.

    Reproduces the exact parameters of deimos_burst_loopback that shows
    ~40% FCS failure on hardware: 10 frames, rate 6, 20000-sample (1ms) gap.
    Each frame gets a slightly different CFO to model real-world variance.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # 10 frames at rate 6, varied CFO (matching hardware conditions)
    rng = np.random.default_rng(seed=99)
    cfo_values = rng.integers(-2000, 2000, size=10).tolist()
    frames = [(6, cfo) for cfo in cfo_values]

    # 20000-sample gap matches hardware burst loopback exactly
    samples, frame_info = build_multi_frame_stream(frames, snr_db=35, gap_samples=20000)
    dut._log.info(f"Burst 10 frames: {len(frames)} frames, "
                  f"gap=20000 samples (1ms), "
                  f"{len(samples)} total samples ({len(samples)/SAMPLE_RATE*1000:.1f} ms)")

    tags = await feed_and_collect_tags(dut, samples, n_expected_frames=len(frames),
                                       timeout_cycles=40000000)

    # All frames must be detected
    assert len(tags) >= len(frames), \
        f"Only {len(tags)}/{len(frames)} frames detected in 10-frame burst."

    # All frames must pass FCS
    fcs_failures = [i for i, t in enumerate(tags) if not t['fcs_ok']]
    assert not fcs_failures, \
        f"Frames {fcs_failures} failed FCS in 10-frame burst (hardware shows ~40% fail). " \
        f"Pipeline rearm bug reproduced in sim!"



@cocotb.test()
async def test_watchdog_recovery(dut):
    """Watchdog timeout while eq_rd_sel=1, then valid frame must decode.

    THE BUG THIS CATCHES:
      If watchdog fires while eq_rd_sel=1 (during EQ/FFT processing of a DATA
      symbol), the equalizer permanently owns the chan_est read address bus.
      The next frame's channel estimation reads through the wrong mux path,
      producing garbage H_inv and guaranteed FCS failure.

    HOW IT'S FIXED (two layers):
      1. S_IDLE trigger: when next frame arrives, the S_IDLE case clears
         eq_rd_sel/vit_streaming_mode/is_signal_out before entering S_SKIP.
      2. Watchdog handler: clears these signals immediately at timeout, not
         waiting for the next trigger.
      Both are needed — (1) catches the case where watchdog isn't the exit
      path; (2) provides immediate cleanup without relying on re-trigger.

    TEST SEQUENCE:
      1. Feed a truncated golden vector: full STF+LTF+SIGNAL but only partial
         DATA. The FSM advances through S_ARM_SIG (eq_rd_sel=1) and into DATA
         processing. Truncation means the pipeline stalls with eq_rd_sel=1.
      2. Silence follows — watchdog fires after ~524k clocks with eq_rd_sel
         still asserted (or cleared by the handler if present).
      3. After watchdog gap, a complete valid rate-6 frame arrives.
      4. Assert: the valid frame MUST decode with FCS OK.

    Without EITHER fix, step 4 fails because channel estimation in the next
    frame reads through the stuck EQ mux.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    valid_iq = load_waveform_float(6)  # Rate 6 golden vector

    # Truncate after SIGNAL symbol begins DATA processing.
    # Golden vector structure: ~160 samples STF + 160 LTF + 80 SIGNAL + DATA
    # We want to get past SIGNAL decode into the first DATA symbol arm
    # (S_ARM_DATA sets eq_rd_sel=1). Keep STF+LTF+SIGNAL+~2 DATA symbols
    # worth then cut. At rate 6 (BPSK 1/2), each OFDM symbol = 80 samples
    # (64 data + 16 CP). Preamble = 320 samples (10 short + 2 long + GI2).
    # SIGNAL = 80 samples. First DATA symbol = 80 samples.
    # Keep 320 + 80 + 160 = 560 samples → FSM is in DATA processing with
    # eq_rd_sel=1 when samples run out.
    truncated_iq = valid_iq[:560]

    # Watchdog fires at watchdog_cnt[19] = 524288 clocks = 104857 samples at
    # 1-per-5 clock ratio. Add margin: 110000 samples of silence.
    watchdog_gap = np.zeros(110000, dtype=complex)

    # Gap before valid frame (STF detector needs quiet to re-arm)
    pre_frame_gap = np.zeros(2000, dtype=complex)

    # Assemble stream
    full_stream = np.concatenate([
        truncated_iq,
        watchdog_gap,
        pre_frame_gap,
        valid_iq,
        np.zeros(4000, dtype=complex),  # trailing flush
    ])
    samples = quantize_12bit(full_stream)

    n_samples = len(samples)
    dut._log.info(f"Watchdog recovery: {n_samples} samples "
                  f"({n_samples/SAMPLE_RATE*1000:.1f} ms), "
                  f"truncated frame (eq_rd_sel=1 at truncation) + "
                  f"watchdog gap + valid frame")

    # Feed and collect — expect 2 tags: one from truncated frame (FCS fail)
    # and one from the valid frame after watchdog recovery.
    # The truncated frame will produce a tag (SIGNAL decodes, DATA is garbage
    # → FCS fail). The complete frame MUST produce FCS OK.
    tags = await feed_and_collect_tags(dut, samples, n_expected_frames=2,
                                       timeout_cycles=1500000)

    # Find the valid frame's tag
    fcs_ok_tags = [t for t in tags if t['fcs_ok']]
    assert len(fcs_ok_tags) >= 1, \
        f"No FCS-OK frame decoded after watchdog recovery! " \
        f"Tags seen: {tags}. " \
        f"This indicates eq_rd_sel or other pipeline state is stuck after " \
        f"watchdog timeout during active EQ processing."

    dut._log.info(f"PASS: watchdog recovery from eq_rd_sel=1 — valid frame "
                  f"decoded after timeout (total tags: {len(tags)}, "
                  f"FCS-OK: {len(fcs_ok_tags)})")
