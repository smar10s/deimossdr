"""
diag_stage_counts.py — Diagnostic: per-frame bit counts at each decode stage
boundary during a multi-frame burst.

DIAGNOSTIC (never asserts).

Purpose: an A/B on the continuous-Viterbi burst regression showed the
continuous decoder emitting fewer depuncturer bits than the committed
baseline for the identical waveform, and leaving the depuncturer's pattern
position mid-group at the next frame_start. This counts valid pulses per
stage, segmented by vit_frame_start, so the stage where the count diverges
is visible directly.

Counted per segment: deinterleaver pairs, depuncturer bits, pairer pairs,
vit_fifo reads, Viterbi output bits, plus stall cycles (vitf_full) and the
depuncturer pattern position at each frame boundary.

Run against both RTL versions and diff the tables. The first stage whose
count differs is where the loss originates; every count upstream of it is
identical by construction.

Usage:
  make -C fpga/test diag_stage_counts BURST_RATE=54 [BURST_GAP=680]
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
async def stage_counts(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = build_waveform()
    n_samples = len(samples)
    dut._log.info(f"=== rate {RATE}M gap={GAP} payloads={PAYLOADS} ===")

    pipe = dut.u_rx_pipeline
    dp = pipe.u_depuncturer

    c = dict(deint=0, depunct=0, pair=0, vitrd=0, vitout=0, stall=0,
             dpwr=0, dprd=0, dpfull=0, kept=0, empty=0)

    def flush_segment(label):
        dut._log.info(
            f"  [{label}] deint={c['deint']} depunct={c['depunct']} "
            f"pair={c['pair']} vitrd={c['vitrd']} vitout={c['vitout']} "
            f"stall={c['stall']} | dp_writes={c['dpwr']} "
            f"dp_reads={c['dprd']} dp_full={c['dpfull']} "
            f"kept={c['kept']} erase={c['dprd'] - c['kept']} "
            f"dp_empty_cyc={c['empty']}"
        )
        for k in c:
            c[k] = 0

    sample_idx = 0
    valid_counter = 0
    post_zero = 8192
    prev_fs = 0
    prev_rd = None
    n_fs = 0
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

        c['deint'] += _i(pipe.deint_valid) == 1
        c['depunct'] += _i(pipe.depunct_valid) == 1
        c['pair'] += _i(pipe.pair_valid) == 1
        c['vitrd'] += _i(pipe.vitf_rd_valid) == 1
        c['vitout'] += _i(pipe.vit_valid) == 1
        c['stall'] += _i(pipe.vitf_full) == 1

        # Depuncturer elastic-FIFO activity. depunct_valid is HELD during a
        # downstream stall, so counting it overstates distinct bits; these
        # count actual FIFO traffic instead.
        c['dpfull'] += _i(dp.fifo_full) == 1
        if _i(dp.valid_in) == 1 and _i(dp.fifo_full) == 0:
            c['dpwr'] += 1
        if _i(pipe.depunct_valid) == 1 and _i(pipe.vitf_full) == 0:
            c['dprd'] += 1
        # Kept vs erasure, measured from the registered read pointer rather
        # than from soft_out == 0. A kept bit consumes a FIFO bit and so
        # advances rd_bits; an erasure does not. Testing soft_out for zero
        # would misclassify a genuine bit that quantized to 0x00 under noise
        # (rx_pipeline.v:626 documents that exact false-positive).
        rd_now = _i(dp.rd_bits)
        if prev_rd is not None and rd_now != prev_rd:
            c['kept'] += (rd_now - prev_rd) & 0x1FF
        prev_rd = rd_now
        # Cycles the depuncturer wanted to emit but its FIFO was starved.
        if _i(dp.active) == 1 and _i(dp.fifo_empty) == 1 and _i(pipe.vitf_full) == 0:
            c['empty'] += 1

        fs = _i(pipe.vit_frame_start)
        if fs == 1 and prev_fs == 0:
            n_fs += 1
            flush_segment(f"before frame_start #{n_fs}")
            dut._log.info(f"    at frame_start #{n_fs}: "
                          f"pat_pos={_i(dp.pat_pos)} active={_i(dp.active)} "
                          f"group_active={_i(dp.group_active)}")
        prev_fs = fs

        if _i(dut.tag_valid) == 1:
            n_tags += 1
            ok = _i(dut.tag_fcs_ok)
            dut._log.info(f"  Frame {n_tags}: len={_i(dut.tag_length)} "
                          f"fcs={'OK' if ok else 'FAIL'}")

        if sample_idx >= n_samples + post_zero:
            break

    flush_segment("tail")
