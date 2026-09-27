"""
diag_bit_path_dump.py — Replay a single-frame capture and dump the decode
bit path stage outputs for comparison against the TX reference chain.

DIAGNOSTIC (never asserts). Used to locate the first stage that diverges
from the reference bit stream for a frame that FCS-fails despite perfect
constellation (EVM -60 dB).

Stages dumped (during the first decoded frame):
  demap_soft   — demapper output (interleaved order)
  deint_soft   — deinterleaver output (punctured order)
  depunct_soft — depuncturer output (coded order + erasures)
  vit_bit      — viterbi decoded bits
  descr_bit    — descrambled bits

Output JSON written to BIT_DUMP_FILE (default /tmp/bit_dump.json):
  { demap: [[s8,...],...], deint: [...], depunct: [...],
    vit_bits: [...], descr_bits: [...] }
Soft values signed (positive = likely 0). Erasures are 0.

Usage:
  REPLAY_FILE=/tmp/simreplay/minrepro_f13.json BIT_DUMP_FILE=/tmp/bits.json \
    make sim SIM_BUILD=sim_build_rx_frontend TOPLEVEL=rx_frontend \
    VERILOG_SOURCES=... COCOTB_TEST_MODULES=diag_bit_path_dump
"""
import json
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, load_adc_capture, s12_to_unsigned


def _s8(v):
    try:
        x = int(v.value)
    except (ValueError, AttributeError):
        return None
    return x - 256 if x >= 128 else x


@cocotb.test()
async def dump_bit_path(dut):
    path = os.environ.get("REPLAY_FILE")
    out_path = os.environ.get("BIT_DUMP_FILE", "/tmp/bit_dump.json")
    if not path:
        dut._log.warning("REPLAY_FILE not set — skipping dump")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = load_adc_capture(path)
    n_samples = len(samples)
    dut._log.info(f"Replaying {path}: {n_samples} samples")

    demap = dut.u_rx_pipeline.u_demapper
    deint = dut.u_rx_pipeline.u_deinterleaver
    depunct = dut.u_rx_pipeline.u_depuncturer
    viterbi = dut.u_rx_pipeline.u_viterbi
    descr = dut.u_rx_pipeline.u_descrambler

    demap_syms = []
    deint_syms = []
    depunct_syms = []
    vit_bits = []
    descr_bits = []

    cur_demap = []
    cur_deint = []
    cur_depunct = []

    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2000000
    frame_done = False

    for cycle in range(timeout_cycles):
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

        if not frame_done:
            try:
                if int(demap.wide_valid.value) == 1:
                    nb = {0: 1, 1: 2, 2: 4, 3: 6}[int(demap.rate_mode.value)]
                    for j in range(nb):
                        v = _s8(getattr(demap, f"soft_wide{j}"))
                        if v is not None:
                            cur_demap.append(v)
                    if len(cur_demap) == nb * 48:
                        demap_syms.append(cur_demap)
                        cur_demap = []
            except (ValueError, AttributeError):
                pass
            try:
                if int(deint.valid_out.value) == 1:
                    v = _s8(deint.soft_out0)
                    if v is not None:
                        cur_deint.append(v)
                    v = _s8(deint.soft_out1)
                    if v is not None:
                        cur_deint.append(v)
                    if len(cur_deint) == 48:
                        deint_syms.append(cur_deint)
                        cur_deint = []
            except (ValueError, AttributeError):
                pass
            try:
                if int(depunct.valid_out.value) == 1:
                    v = _s8(depunct.soft_out)
                    if v is not None:
                        cur_depunct.append(v)
                        if len(cur_depunct) == 72:
                            depunct_syms.append(cur_depunct)
                            cur_depunct = []
            except (ValueError, AttributeError):
                pass
            try:
                if int(viterbi.valid_out.value) == 1:
                    vit_bits.append(int(viterbi.bit_out.value))
            except (ValueError, AttributeError):
                pass
            try:
                if int(descr.valid_out.value) == 1:
                    descr_bits.append(int(descr.bit_out.value))
            except (ValueError, AttributeError):
                pass
            try:
                if int(dut.tag_valid.value) == 1:
                    frame_done = True
            except (ValueError, AttributeError):
                pass

        if sample_idx >= n_samples + post_zero_count and frame_done:
            break

    out = {
        'demap': demap_syms,
        'deint': deint_syms,
        'depunct': depunct_syms,
        'vit_bits': vit_bits,
        'descr_bits': descr_bits,
    }
    with open(out_path, 'w') as f:
        json.dump(out, f)
    dut._log.info(f"dumped: demap_syms={len(demap_syms)} "
                  f"deint_syms={len(deint_syms)} depunct_syms={len(depunct_syms)} "
                  f"vit_bits={len(vit_bits)} descr_bits={len(descr_bits)}")
