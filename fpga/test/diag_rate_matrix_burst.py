"""
diag_rate_matrix_burst.py — Diagnostic: replay a 6-frame burst at a single
chosen rate and report per-frame FCS.

DIAGNOSTIC (never asserts).

Purpose: the hil_burst_replay regression fails only on the 54M scenarios and
passes on the mixed-rate scenario. Those two differ in TWO ways at once —
code rate (3/4 vs 1/2) and modulation order (64-QAM vs BPSK/QPSK/16-QAM).
This sweeps one rate at a time so the two factors separate:

    code rate 1/2 : rates 6, 12, 24
    code rate 3/4 : rates 9, 18, 36, 54
    code rate 2/3 : rate 48

If every 3/4 rate fails frames 2-6 and every 1/2 rate passes, the cause is
puncturing, not modulation. If only 48/54 fail, it is modulation or the
symbol-rate/throughput budget.

Usage:
  make -C fpga/test diag_rate_matrix_burst BURST_RATE=9
  (env: BURST_RATE = rate in Mbps, default 9; BURST_IMPAIR = 0 to disable
   the multipath impairment, isolating decode from channel effects)
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

RATE = int(os.environ.get("BURST_RATE", "9"))
IMPAIR = os.environ.get("BURST_IMPAIR", "1") == "1"
GAP = int(os.environ.get("BURST_GAP", str(DIFS_SAMPLES)))
NFRAMES = int(os.environ.get("BURST_FRAMES", "6"))

# Same payload length mix as FRAME_SPECS, so only the rate varies.
# BURST_PAYLOADS overrides the mix, so a single length can be replayed
# standalone to tell a per-frame failure from a cross-frame carryover.
PAYLOADS = [137, 14, 193, 100, 14, 371]
if os.environ.get("BURST_PAYLOADS"):
    PAYLOADS = [int(x) for x in os.environ["BURST_PAYLOADS"].split(",")]
    NFRAMES = int(os.environ.get("BURST_FRAMES", str(len(PAYLOADS))))


def build_waveform(rate, impair):
    specs = [(rate, n) for n in PAYLOADS[:NFRAMES]]
    burst_iq, expected = build_burst(specs, gap_samples=GAP)
    if impair:
        burst_iq = apply_multipath(burst_iq.copy(), MULTIPATH_PRESETS["moderate"])

    re = np.real(burst_iq).astype(np.float64)
    im = np.imag(burst_iq).astype(np.float64)
    peak = max(np.abs(re).max(), np.abs(im).max())
    scale = (2047.0 * 0.9) / peak if peak > 0 else 1.0
    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)

    samples = list(zip(re_q.tolist(), im_q.tolist()))
    samples = apply_hil_noise_floor(samples, noise_fraction=0.02, seed=1)
    return samples, expected


@cocotb.test()
async def rate_matrix_burst(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples, expected = build_waveform(RATE, IMPAIR)
    n_samples = len(samples)
    dut._log.info(f"=== rate {RATE}M burst, impair={IMPAIR}, gap={GAP}, "
                  f"frames={NFRAMES}: {n_samples} samples ===")

    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2000000

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)

        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < n_samples + post_zero_count:
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = 0
                dut.iq_q_in.value = 0
                sample_idx += 1
            else:
                dut.iq_valid_in.value = 0
        else:
            dut.iq_valid_in.value = 0

        valid_counter = (valid_counter + 1) % 5

        if int(dut.tag_valid.value) == 1:
            tag = {
                "rate": int(dut.tag_rate.value),
                "length": int(dut.tag_length.value),
                "fcs_ok": int(dut.tag_fcs_ok.value),
            }
            tags.append(tag)
            fcs = "OK" if tag["fcs_ok"] else "FAIL"
            dut._log.info(f"  Frame {len(tags)}: rate=0b{tag['rate']:04b} "
                          f"len={tag['length']} fcs={fcs}")

        if sample_idx >= n_samples + post_zero_count:
            break

    n_ok = sum(t["fcs_ok"] for t in tags)
    pattern = "".join("O" if t["fcs_ok"] else "X" for t in tags)
    dut._log.info(f"RESULT rate={RATE}M impair={IMPAIR} gap={GAP} "
                  f"frames={NFRAMES}: {len(tags)} tags, {n_ok} OK  "
                  f"pattern={pattern}")
