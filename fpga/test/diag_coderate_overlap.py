"""
diag_coderate_overlap.py — Diagnostic: does code_rate_out change while the
depuncturer still holds undrained bits from the previous frame?

DIAGNOSTIC (never asserts).

Hypothesis under test: the depuncturer is the slowest stage in the chain. It
emits one bit per clock, so at 54 Mbps it needs 432 clocks per symbol while
the deinterleaver delivers that symbol in 144. Its elastic FIFO therefore
still holds a backlog when the frame ends.

decode_engine drives code_rate_out from its own frame sequencing, and resets
it to 0 when it pops the next frame descriptor. The depuncturer has no
frame_start input, so if that reset lands while a backlog is pending, the
remaining bits are emitted in rate-1/2 passthrough. Passthrough never touches
pat_pos, so the pattern position freezes mid-group and every later frame in
the burst decodes at the wrong puncture phase.

This logs every code_rate_out transition alongside the depuncturer's pending
bit count and pattern position. A nonzero backlog at a transition is the bug;
zero at every transition refutes the hypothesis.

Usage:
  make -C fpga/test diag_coderate_overlap BURST_RATE=54 [BURST_GAP=680]
  (env: BURST_RATE, BURST_GAP, BURST_IMPAIR, BURST_PAYLOADS)
"""
import os
import sys

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, s12_to_unsigned
from diag_hil_burst_replay import apply_hil_noise_floor

SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))

from gen_impaired_burst import build_burst, DIFS_SAMPLES  # noqa: E402
from py80211.impairments import apply_multipath, MULTIPATH_PRESETS  # noqa: E402

RATE = int(os.environ.get("BURST_RATE", "54"))
GAP = int(os.environ.get("BURST_GAP", str(DIFS_SAMPLES)))
IMPAIR = os.environ.get("BURST_IMPAIR", "1") == "1"
PAYLOADS = [int(x) for x in os.environ.get("BURST_PAYLOADS", "137,14").split(",")]


def build_waveform():
    specs = [(RATE, n) for n in PAYLOADS]
    burst_iq, _ = build_burst(specs, gap_samples=GAP)
    if IMPAIR:
        burst_iq = apply_multipath(burst_iq.copy(), MULTIPATH_PRESETS["moderate"])
    re = np.real(burst_iq).astype(np.float64)
    im = np.imag(burst_iq).astype(np.float64)
    peak = max(np.abs(re).max(), np.abs(im).max())
    scale = (2047.0 * 0.9) / peak if peak > 0 else 1.0
    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)
    samples = list(zip(re_q.tolist(), im_q.tolist()))
    return apply_hil_noise_floor(samples, noise_fraction=0.02, seed=1)


def _i(sig):
    try:
        return int(sig.value)
    except Exception:
        return -1


@cocotb.test()
async def coderate_overlap(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = build_waveform()
    n_samples = len(samples)
    dut._log.info(f"=== rate {RATE}M gap={GAP} payloads={PAYLOADS} ===")

    pipe = dut.u_rx_pipeline
    dp = pipe.u_depuncturer

    sample_idx = 0
    valid_counter = 0
    post_zero = 8192
    prev_cr = None
    n_tags = 0

    for _ in range((n_samples + post_zero) * 5 + 200000):
        await RisingEdge(dut.clk)

        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < n_samples + post_zero:
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0
        valid_counter = (valid_counter + 1) % 5

        cr = _i(pipe.code_rate_out)
        if prev_cr is not None and cr != prev_cr:
            wr = _i(dp.wr_entries)
            rd = _i(dp.rd_bits)
            backlog = (((wr << 1) & 0x1FF) - rd) & 0x1FF
            flag = "  <-- BACKLOG PENDING" if backlog else ""
            dut._log.info(
                f"  code_rate {prev_cr} -> {cr} @sample {sample_idx}: "
                f"depunct backlog={backlog} bits, pat_pos={_i(dp.pat_pos)}, "
                f"group_active={_i(dp.group_active)}{flag}"
            )
        prev_cr = cr

        if _i(dut.tag_valid) == 1:
            n_tags += 1
            ok = _i(dut.tag_fcs_ok)
            dut._log.info(f"  Frame {n_tags}: len={_i(dut.tag_length)} "
                          f"fcs={'OK' if ok else 'FAIL'}")

        if sample_idx >= n_samples + post_zero:
            break
