"""
diag_pending_vcd.py — Minimal pending trigger test for VCD capture.

Runs on Verilator (no hierarchical access). Just feeds 2-frame stream,
waits for 2 tags, exits. All debug info comes from the VCD dump.

Usage:
    make waves TARGET=diag_pending_vcd
    python scripts/vcd_query.py list <vcd> feed_valid
    python scripts/vcd_query.py extract <vcd> feed_valid rd_ptr state --start <T1> --end <T2>
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

LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.impairments import add_cfo, add_awgn


def build_tight_gap_stream(gap_samples=200, snr_db=40):
    """Build 2 rate-6 frames with a very tight gap (zero CFO, zero noise)."""
    iq1 = load_waveform_float(6)
    iq2 = load_waveform_float(6)
    iq1 = add_cfo(iq1, 0)
    iq2 = add_cfo(iq2, 0)

    leading = np.zeros(5000, dtype=complex)
    gap = np.zeros(gap_samples, dtype=complex)
    trailing = np.zeros(8000, dtype=complex)

    full = np.concatenate([leading, iq1, gap, iq2, trailing])
    samples = quantize_12bit(full)
    return samples


@cocotb.test()
async def diag_pending_vcd_run(dut):
    """Feed 2-frame stream, wait for 2 tags. VCD captures everything."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = build_tight_gap_stream(gap_samples=200)
    n_samples = len(samples)
    dut._log.info(f"Feeding {n_samples} samples (2 frames, gap=200)")

    tags = []
    sample_idx = 0
    valid_counter = 0
    tag_count = 0

    # Run until 2 tags received or timeout
    timeout_cycles = n_samples * 5 + 300000

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        # Feed IQ at 1-per-5
        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        # Collect tags (port-level only, Verilator compatible)
        try:
            if int(dut.tag_valid.value) == 1:
                tag_count += 1
                rate = int(dut.tag_rate.value)
                length = int(dut.tag_length.value)
                fcs_ok = int(dut.tag_fcs_ok.value)
                fcs_str = 'OK' if fcs_ok else 'FAIL'
                dut._log.info(f"  Tag {tag_count}: rate=0b{rate:04b}, len={length}, fcs={fcs_str} @ cycle {cycle}")
                if tag_count >= 2:
                    # Wait a few more cycles for VCD to capture post-tag state
                    for _ in range(100):
                        await RisingEdge(dut.clk)
                    break
        except (ValueError, AttributeError):
            pass

    dut._log.info(f"Done. {tag_count} tags collected.")
