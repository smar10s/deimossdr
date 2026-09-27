"""
diag_long_frame.py — Characterize buffer behavior for long frames.

Historical context: this diagnostic was written when the IQ buffer was 8192
entries in the old rx_top.v, which truncated frames with >99 DATA symbols
(SIGNAL decoded, DATA FCS failed). That limit is gone — the streaming
circular buffer in decode_engine.v is now 32768 entries, and back-pressure /
clean-drop behavior is described in D23.

The diagnostic is retained to characterize long-frame decode end to end. It
is a diagnostic, not a gate: it must never assert (see AGENTS.md, Tests vs
Diagnostics).

Run:
  SIM=verilator make -C fpga/test diag_long_frame
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

import os
import json
import numpy as np

from frontend_helpers import (
    quantize_12bit, reset_dut, run_frontend_decode_live,
)

VECTORS_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'vectors')


def load_long_waveform(rate_mbps):
    """Load long waveform (300-byte PSDU) as complex float numpy array."""
    path = os.path.join(VECTORS_DIR, f'legacy_{rate_mbps}mbps_long_waveform.json')
    with open(path) as f:
        data = json.load(f)
    return np.array(data['real'], dtype=np.float64) + 1j * np.array(data['imag'], dtype=np.float64)


@cocotb.test()
async def diag_rate6_long_frame(dut):
    """Rate 6, 300-byte PSDU (101 DATA symbols) — buffer overflow characterization.

    With 8192-sample buffer: expects SIGNAL decode OK, DATA FCS FAIL.
    After IQ FIFO: expects full decode, FCS PASS.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_long_waveform(6)
    samples = quantize_12bit(iq)
    dut._log.info(f"Long frame rate 6: {len(samples)} samples (300 bytes, 101 DATA symbols)")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    dut._log.info("=" * 60)
    dut._log.info("LONG FRAME CHARACTERIZATION (rate 6, 300 bytes, 101 symbols)")
    dut._log.info("=" * 60)
    dut._log.info(f"  frame_detect: {'YES' if r['frame_detect_seen'] else 'NO'}")
    dut._log.info(f"  stf_end:      {'YES' if r['stf_end_seen'] else 'NO'}")
    dut._log.info(f"  SIGNAL valid: {'YES' if r['signal_valid'] else 'NO'}")
    if r['signal_valid']:
        dut._log.info(f"  SIGNAL rate:  0b{r['parsed_rate']:04b}")
        dut._log.info(f"  SIGNAL len:   {r['parsed_length']}")
    dut._log.info(f"  tag_valid:    {'YES' if r['tag_valid'] else 'NO'}")
    if r['tag_valid']:
        dut._log.info(f"  tag_fcs_ok:   {r['tag_fcs_ok']}")
        dut._log.info(f"  tag_length:   {r['tag_length']}")
    dut._log.info(f"  total_cycles: {r['total_cycles']}")
    dut._log.info("-")

    if r['tag_valid'] and r['tag_fcs_ok']:
        dut._log.info("RESULT: PASS — long frame decoded successfully (FIFO working!)")
    elif r['tag_valid'] and not r['tag_fcs_ok']:
        dut._log.info("RESULT: EXPECTED FAIL — FCS failed (buffer overflow truncates frame)")
    elif r['signal_valid'] and not r['tag_valid']:
        dut._log.info("RESULT: EXPECTED FAIL — SIGNAL decoded but no tag output (stuck)")
    else:
        dut._log.info("RESULT: UNEXPECTED — no SIGNAL decode (check STF/LTF path)")

    dut._log.info("=" * 60)


@cocotb.test()
async def diag_rate24_long_frame(dut):
    """Rate 24, 500-byte PSDU — fewer DATA symbols (42), should fit in buffer.

    Rate 24: 16-QAM, code rate 1/2, N_DBPS = 96.
    500 bytes: ceil((16 + 500*8 + 6) / 96) = 42 DATA symbols.
    42 symbols × 80 = 3360 samples + 208 preamble = 3568 total. Well within 8192.

    This is a CONTROL: if this fails, the problem is NOT buffer overflow.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq = load_long_waveform(24)
    samples = quantize_12bit(iq)
    dut._log.info(f"Long frame rate 24: {len(samples)} samples (500 bytes, 42 DATA symbols)")

    r = await run_frontend_decode_live(dut, samples, pre_noise_samples=500)

    dut._log.info("=" * 60)
    dut._log.info("LONG FRAME CONTROL (rate 24, 500 bytes, 42 symbols)")
    dut._log.info("=" * 60)
    dut._log.info(f"  frame_detect: {'YES' if r['frame_detect_seen'] else 'NO'}")
    dut._log.info(f"  SIGNAL valid: {'YES' if r['signal_valid'] else 'NO'}")
    if r['signal_valid']:
        dut._log.info(f"  SIGNAL len:   {r['parsed_length']}")
    dut._log.info(f"  tag_valid:    {'YES' if r['tag_valid'] else 'NO'}")
    if r['tag_valid']:
        dut._log.info(f"  tag_fcs_ok:   {r['tag_fcs_ok']}")
        dut._log.info(f"  tag_length:   {r['tag_length']}")
    dut._log.info("-")

    if r['tag_valid'] and r['tag_fcs_ok']:
        dut._log.info("RESULT: PASS — rate 24 long frame fits in buffer (control passes)")
    else:
        dut._log.info("RESULT: FAIL — rate 24 long frame failed (NOT a buffer overflow issue)")
        dut._log.info("  This control should pass. Investigate other pipeline issues.")

    dut._log.info("=" * 60)
