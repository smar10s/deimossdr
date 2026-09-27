"""
diag_stf_clear_race.py — Characterize the playback_start/stf_clear race.

HIL hardware path: hil_ctrl playback_start -> stf_clear_or (OR logic cell)
-> stf_detect.clear. In sim, rx_frontend models stf_clear = playback_start
(the OR is a LUT in the BD; the sim view is generated, see D19).

If the clear lands mid-STF (hardware skew between the clear path and the
DDR/DMA first-sample path — placement + DRAM latency jitter), stf_detect's
accumulators and sample_cnt are zeroed. The remaining STF samples must
refill the 64-sample window: a clear at sample ~100 leaves ~60 STF samples,
the window refills into the LTF (different period) -> correlation drops ->
frame_detect never fires -> no tag, no snap trigger. Signature matches the
observed ~0.5% rate-6 HIL flake (no-tag, empty snap).

Report-only diag. Sweeps playback_start pulse offsets across a clean
rate-6 frame; logs frame_detect events and tag outcome per offset.
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

OFFSETS = [None, 0, 20, 40, 60, 80, 90, 100, 110, 120, 140, 160, 200, 300]


async def run_offset(dut, offset):
    """Feed one clean rate-6 frame; pulse playback_start at `offset` feed
    samples (None = no pulse). Returns (detect_count, tags)."""
    await reset_dut(dut)

    iq = load_waveform_float(6)
    samples = quantize_12bit(iq)

    leading = 3000
    trailing = 8000
    n_total = leading + len(samples) + trailing

    detect_count = 0
    tags = []
    sample_idx = 0
    valid_counter = 0
    pulsed = False

    for cycle in range(1_500_000):
        await RisingEdge(dut.clk)

        if valid_counter == 0:
            if sample_idx < n_total:
                if sample_idx < leading or sample_idx >= leading + len(samples):
                    dut.iq_valid_in.value = 1
                    dut.iq_i_in.value = 0
                    dut.iq_q_in.value = 0
                else:
                    re_q, im_q = samples[sample_idx - leading]
                    dut.iq_valid_in.value = 1
                    dut.iq_i_in.value = s12_to_unsigned(re_q)
                    dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # playback_start pulse at the requested frame-sample offset
        if (offset is not None and not pulsed
                and sample_idx == leading + offset):
            dut.playback_start.value = 1
            pulsed = True
        else:
            dut.playback_start.value = 0

        try:
            if int(dut.frame_detect.value) == 1:
                detect_count += 1
                dut._log.info(f"    frame_detect #{detect_count} @ feed sample {sample_idx}")
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.tag_valid.value) == 1:
                tag = {
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'sample': sample_idx,
                }
                tags.append(tag)
                dut._log.info(f"    tag: rate={tag['rate']} len={tag['length']} "
                              f"fcs={tag['fcs_ok']}")
                if len(tags) >= 1:
                    break
        except (ValueError, AttributeError):
            pass

    return detect_count, tags


@cocotb.test()
async def test_stf_clear_race_sweep(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    dut._log.info("=" * 60)
    dut._log.info("playback_start pulse offset sweep over a clean rate-6 frame")
    dut._log.info("=" * 60)

    for offset in OFFSETS:
        detect_count, tags = await run_offset(dut, offset)
        if tags:
            t = tags[0]
            outcome = f"tag fcs={'OK' if t['fcs_ok'] else 'FAIL'} len={t['length']}"
        else:
            outcome = "NO TAG"
        dut._log.info(f"  offset={str(offset):>4}: detects={detect_count} -> {outcome}")
