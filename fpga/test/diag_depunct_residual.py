"""
diag_depunct_residual.py — Diagnostic: report depuncturer residual state at
every vit_frame_start during a multi-frame burst.

DIAGNOSTIC (never asserts).

Hypothesis under test: soft_pairer and vit_fifo reset on vit_frame_start, but
the depuncturer has no frame_start input — its rd_bits / wr_entries / pat_pos
/ active state clears only on rst_n or symbol_start, and symbol_start is tied
to 1'b0 in rx_pipeline. If frame N leaves undrained bits in the elastic FIFO,
those bits are emitted into a freshly-reset pairer, shifting the puncture
phase by one bit for the rest of the burst.

The number that matters is FIFO occupancy at vit_frame_start. Zero means the
chain drained cleanly. Nonzero, especially odd, means leftover bits are about
to be paired against the new frame's stream.

That missing reset is equally true of the pre-change RTL, so this is an A/B
instrument, not a smoking gun: run it against the committed baseline and
against the continuous decoder, then compare the occupancy numbers.

Internal-signal reads are documented as unreliable (docs/debugging.md), so
treat this as a locator and confirm what it points at with VCD.

Usage:
  make -C fpga/test diag_depunct_residual BURST_RATE=54 [BURST_GAP=680]
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
async def depunct_residual(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = build_waveform()
    n_samples = len(samples)
    dut._log.info(f"=== rate {RATE}M gap={GAP} payloads={PAYLOADS}: "
                  f"{n_samples} samples ===")

    pipe = dut.u_rx_pipeline
    dp = pipe.u_depuncturer
    sp = pipe.u_soft_pairer

    sample_idx = 0
    valid_counter = 0
    post_zero = 8192
    prev_fs = 0
    n_tags = 0
    depunct_emits = 0

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

        if _i(dp.valid_out) == 1:
            depunct_emits += 1

        # vit_frame_start is the reset that soft_pairer/vit_fifo see but the
        # depuncturer does not. Report what the depuncturer still holds.
        fs = _i(pipe.vit_frame_start)
        if fs == 1 and prev_fs == 0:
            wr = _i(dp.wr_entries)
            rd = _i(dp.rd_bits)
            occ = (((wr << 1) & 0x1FF) - rd) & 0x1FF
            dut._log.info(
                f"  vit_frame_start @{sample_idx}: depunct occupancy={occ} "
                f"(wr_entries={wr} rd_bits={rd}) pat_pos={_i(dp.pat_pos)} "
                f"active={_i(dp.active)} group_active={_i(dp.group_active)} "
                f"have_first={_i(sp.have_first)} emits_so_far={depunct_emits}"
            )
        prev_fs = fs

        if _i(dut.tag_valid) == 1:
            n_tags += 1
            ok = _i(dut.tag_fcs_ok)
            dut._log.info(f"  Frame {n_tags}: len={_i(dut.tag_length)} "
                          f"fcs={'OK' if ok else 'FAIL'}")

        if sample_idx >= n_samples + post_zero:
            break

    dut._log.info(f"total depuncturer emits={depunct_emits}")
