"""
diag_burst_hil_replay.py — Diagnostic: replay a deimos_burst_loopback
--dump-hil waveform through rx_frontend and report per-frame tag/FCS.

Usage:
  make -C fpga/test diag_burst_hil_replay DUMP=/path/to/dump.hil

Never asserts — diagnostics only (per AGENTS tests-vs-diagnostics rule).
"""
import os
import struct
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, s12_to_unsigned

DUMP = os.environ.get("DUMP", "")


def load_hil_dump(path):
    """Parse raw 32-bit words {8'b0, im[11:0], re[11:0]} into [(re, im), ...]."""
    with open(path, "rb") as f:
        raw = f.read()
    n_words = len(raw) // 4
    samples = []
    for i in range(n_words):
        w = struct.unpack("<I", raw[i * 4:i * 4 + 4])[0]
        re = w & 0xFFF
        im = (w >> 12) & 0xFFF
        if re & 0x800:
            re -= 0x1000
        if im & 0x800:
            im -= 0x1000
        samples.append((re, im))
    return samples


@cocotb.test()
async def diag_burst_hil_replay(dut):
    if not DUMP:
        cocotb.log.error("DUMP not set — pass DUMP=/path/to/file.hil")
        return
    cocotb.log.info(f"Replaying HIL dump: {DUMP}")
    samples = load_hil_dump(DUMP)
    n_samples = len(samples)
    cocotb.log.info(f"Loaded {n_samples} samples")

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 5000000

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
                "sample": sample_idx,
            }
            tags.append(tag)
            fcs_str = "OK" if tag["fcs_ok"] else "FAIL"
            cocotb.log.info(
                f"  Frame {len(tags)}: rate=0b{tag['rate']:04b}, "
                f"len={tag['length']}, fcs={fcs_str} @ sample ~{sample_idx}"
            )

        if sample_idx >= n_samples + post_zero_count:
            break

    fcs_ok = [t for t in tags if t["fcs_ok"]]
    fcs_fail = [t for t in tags if not t["fcs_ok"]]
    cocotb.log.info(f"REPLAY SUMMARY: {len(tags)} tags, "
                    f"{len(fcs_ok)} FCS OK, {len(fcs_fail)} FCS FAIL")
    for i, t in enumerate(tags):
        cocotb.log.info(f"  TAG[{i}]: rate={t['rate']:#06b} len={t['length']} "
                        f"fcs={'OK' if t['fcs_ok'] else 'FAIL'}")
