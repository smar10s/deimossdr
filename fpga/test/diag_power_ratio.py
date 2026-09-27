"""
diag_power_ratio.py — Diagnostic: power-ratio characterization for near-far envelope.

WHAT THIS MEASURES:
  Sweeps the amplitude ratio between consecutive SIFS-spaced frames and reports
  the maximum ratio at which both frames decode (FCS OK). This characterizes
  the fabric's operating envelope for near-far scenarios (e.g., AP at -30 dBm
  vs STA at -60 dBm on the same channel).

WHY IT MATTERS:
  In real 5 GHz traffic, an AP's beacon arrives at one power level and a STA's
  EAPOL response arrives at a very different level. The AGC may still be settled
  for the loud frame when the quiet frame arrives. The receiver must decode both.
  This diagnostic finds the wall.

NOT A GATE TEST:
  This is a diagnostic (diag_*). It reports numbers. It never asserts.
  Run via: ./scripts/sim.sh diag_power_ratio

PARAMETERS:
  - Rates: 6, 12, 24, 36 (skip 48/54 — known EVM margin limitation)
  - Ratios: 0, 6, 10, 14, 18, 22, 26, 30 dB
  - Trials per (rate, ratio): 3
  - Channel: SNR=35 dB (high, to isolate power-ratio effect)
  - Gap: SIFS (320 samples = 16 us)
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
from py80211.channel import ChannelConfig, generate_stream, power_sweep_pair

# Sweep parameters
RATES = [6, 12, 24, 36]
RATIOS_DB = [0, 6, 10, 14, 18, 22, 26, 30]
TRIALS = 3
SNR_DB = 35.0


async def feed_and_collect_tags(dut, samples, n_expected_frames):
    """Feed quantized IQ stream and collect tag outputs (multi-frame).

    Feeds at 1-per-5 clock rate. After all samples, feeds 8192 zeros to flush.
    After flush completes, waits at most 50000 more cycles for final tag.
    Returns list of tag dicts in decode order.
    """
    n_samples = len(samples)
    post_zeros = 8192
    # Total cycles: all data at 1-per-5 + post-flush wait
    total_feed_cycles = (n_samples + post_zeros) * 5
    # After feed done, wait at most 50000 clocks for any remaining tag
    max_post_cycles = 50000
    timeout_cycles = total_feed_cycles + max_post_cycles

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
            elif sample_idx < n_samples + post_zeros:
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
                frames_seen += 1
                if frames_seen >= n_expected_frames:
                    break
        except (ValueError, AttributeError):
            pass

    return tags


@cocotb.test()
async def diag_power_ratio(dut):
    """Sweep power ratio between SIFS-spaced frame pairs across rates.

    Reports max decodable ratio per rate. Both frames must decode (FCS OK)
    for a trial to count as success.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    config = ChannelConfig(snr_db=SNR_DB)

    dut._log.info("=" * 60)
    dut._log.info("POWER RATIO CHARACTERIZATION")
    dut._log.info(f"  Rates: {RATES}")
    dut._log.info(f"  Ratios (dB): {RATIOS_DB}")
    dut._log.info(f"  Trials per point: {TRIALS}")
    dut._log.info(f"  Channel: SNR={SNR_DB} dB, no CFO/multipath (isolate power ratio)")
    dut._log.info(f"  Gap: SIFS (320 samples = 16 us)")
    dut._log.info("=" * 60)

    # Results: rate -> max_ratio_db where all trials passed
    results = {}

    for rate in RATES:
        dut._log.info(f"--- Rate {rate} Mbps ---")
        max_passing_ratio = -1

        for ratio_db in RATIOS_DB:
            pass_count = 0

            for trial in range(TRIALS):
                await reset_dut(dut)

                # Generate frame pair: loud then quiet, separated by SIFS
                seed = rate * 1000 + int(ratio_db * 10) + trial
                frames = power_sweep_pair(rate_mbps=rate, ratio_db=ratio_db)
                # First frame gap = 0 (starts immediately)
                frames[0].gap_samples = 0

                # Generate stream through channel model
                iq_stream, metadata = generate_stream(frames, config, seed=seed)

                # Quantize to 12-bit
                samples = quantize_12bit(iq_stream)

                # Feed and collect tags (expect 2 frames)
                tags = await feed_and_collect_tags(dut, samples, n_expected_frames=2)

                # Check: both frames decoded with FCS OK
                both_ok = (len(tags) >= 2 and
                           tags[0]['fcs_ok'] and
                           tags[1]['fcs_ok'])
                if both_ok:
                    pass_count += 1

            # Report per-ratio result
            status = f"{pass_count}/{TRIALS}"
            if pass_count == TRIALS:
                max_passing_ratio = ratio_db
                marker = "  PASS"
            elif pass_count > 0:
                marker = "  PARTIAL"
            else:
                marker = "  FAIL"
            dut._log.info(f"  ratio={ratio_db:2d} dB: {status}{marker}")

        results[rate] = max_passing_ratio
        dut._log.info(f"  => Max passing ratio: {max_passing_ratio} dB")

    # Summary
    dut._log.info("=" * 60)
    dut._log.info("SUMMARY: Maximum power ratio (both frames decode, all trials)")
    dut._log.info("-" * 60)
    for rate in RATES:
        ratio = results[rate]
        bar = "#" * (ratio // 2) if ratio >= 0 else "(none)"
        dut._log.info(f"  Rate {rate:2d} Mbps: {ratio:3d} dB  {bar}")
    dut._log.info("=" * 60)
    dut._log.info("(Diagnostic complete — no assertions)")
