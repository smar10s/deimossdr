"""
test_long_frame.py — Gate test for streaming decode of long frames.

Validates that the circular buffer with streaming writes correctly decodes
frames exceeding the old capture-then-process limit (48-99 DATA symbols).

This test ASSERTS on FCS pass. If it fails, the streaming buffer has a bug.

Vectors:
  - Rate 6, 300 bytes (101 DATA symbols) — exceeds old 99-symbol limit
  - Rate 24, 500 bytes (42 DATA symbols) — control (always fit)
  - Rate 6, 1500 bytes (501 DATA symbols) — stresses symbol_idx > 255
  - Rate 24, 1500 bytes (126 DATA symbols) — control (no idx wrap)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

import os
import json
import numpy as np

from frontend_helpers import (
    quantize_12bit, reset_dut, run_frontend_decode_live,
)

VECTORS_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')


def load_long_waveform(rate_mbps):
    """Load long waveform as complex float numpy array."""
    path = os.path.join(VECTORS_DIR, f'legacy_{rate_mbps}mbps_long_waveform.json')
    with open(path) as f:
        data = json.load(f)
    return np.array(data['real'], dtype=np.float64) + 1j * np.array(data['imag'], dtype=np.float64)


@cocotb.test()
async def test_rate6_long_frame(dut):
    """Rate 6, 300-byte PSDU (101 DATA symbols) — streaming buffer gate test."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_long_waveform(6)
    samples = quantize_12bit(iq)
    dut._log.info(f"Rate 6 long frame: {len(samples)} samples (300 bytes, 101 DATA symbols)")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    assert r['frame_detect_seen'], "STF detection did not fire"
    assert r['tag_valid'], "No tag output — pipeline did not complete"
    assert r['tag_fcs_ok'], (
        f"FCS FAILED for 300-byte rate 6 frame (101 symbols). "
        f"Streaming buffer overflow — consumer too slow or buffer too small."
    )
    dut._log.info(f"PASS: rate 6, 300 bytes, 101 symbols, FCS OK")


@cocotb.test()
async def test_rate24_long_frame(dut):
    """Rate 24, 500-byte PSDU (42 DATA symbols) — control test."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_long_waveform(24)
    samples = quantize_12bit(iq)
    dut._log.info(f"Rate 24 long frame: {len(samples)} samples (500 bytes, 42 DATA symbols)")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    assert r['frame_detect_seen'], "STF detection did not fire"
    assert r['tag_valid'], "No tag output — pipeline did not complete"
    assert r['tag_fcs_ok'], (
        f"FCS FAILED for 500-byte rate 24 frame (42 symbols). "
        f"This frame should always fit. Pipeline bug, not buffer overflow."
    )
    dut._log.info(f"PASS: rate 24, 500 bytes, 42 symbols, FCS OK")


def load_1500B_waveform(rate_mbps):
    """Load 1500-byte PSDU waveform as complex float numpy array."""
    path = os.path.join(VECTORS_DIR, f'legacy_{rate_mbps}mbps_1500B_waveform.json')
    with open(path) as f:
        data = json.load(f)
    return np.array(data['real'], dtype=np.float64) + 1j * np.array(data['imag'], dtype=np.float64)


@cocotb.test()
async def test_rate6_1500B_frame(dut):
    """Rate 6, 1500-byte PSDU (501 DATA symbols) — symbol_idx > 255 stress test.

    This frame wraps the 8-bit symbol index twice (past 255 and past 381).
    Without the 11-bit symbol_idx fix, DATA symbol 255 would be treated as
    SIGNAL (idx=0) and bypass pilot tracking, causing FCS failure.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_1500B_waveform(6)
    samples = quantize_12bit(iq)
    dut._log.info(f"Rate 6, 1500B frame: {len(samples)} samples (1500 bytes, 501 DATA symbols)")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    assert r['frame_detect_seen'], "STF detection did not fire"
    assert r['tag_valid'], "No tag output — pipeline did not complete"
    assert r['tag_fcs_ok'], (
        f"FCS FAILED for 1500-byte rate 6 frame (501 symbols). "
        f"symbol_idx wrap bug — DATA symbol 255 treated as SIGNAL?"
    )
    dut._log.info(f"PASS: rate 6, 1500 bytes, 501 symbols, FCS OK (symbol_idx > 255 safe)")


@cocotb.test()
async def test_rate24_1500B_frame(dut):
    """Rate 24, 1500-byte PSDU (126 DATA symbols) — control (no idx wrap).

    126 DATA symbols does not exceed 255, so this tests large-frame decode
    without exercising the index wrap. Serves as regression baseline.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_1500B_waveform(24)
    samples = quantize_12bit(iq)
    dut._log.info(f"Rate 24, 1500B frame: {len(samples)} samples (1500 bytes, 126 DATA symbols)")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    assert r['frame_detect_seen'], "STF detection did not fire"
    assert r['tag_valid'], "No tag output — pipeline did not complete"
    assert r['tag_fcs_ok'], (
        f"FCS FAILED for 1500-byte rate 24 frame (126 symbols). "
        f"Large frame decode bug (not symbol_idx related at this count)."
    )
    dut._log.info(f"PASS: rate 24, 1500 bytes, 126 symbols, FCS OK")
