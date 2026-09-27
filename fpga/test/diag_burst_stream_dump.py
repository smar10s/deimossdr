"""
diag_burst_stream_dump.py — Diagnostic: dump deinterleaver output stream
(valid + soft0/1), depuncturer output, pairer output, stall-chain state,
and vit_fifo flush state for frame N (default: frame 2, between tag 1
and tag 2).

Usage:
  make -C fpga/test diag_burst_stream_dump DUMP=... [FRAME=2]
Writes /tmp/opencode/stream_dump.txt (append mode off).

Works on pre-stall-chain RTL too (missing signals read as 0).
"""
import os
import struct

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, s12_to_unsigned

DUMP = os.environ.get("DUMP", "")
FRAME = int(os.environ.get("FRAME", "2"))
STOP_CYCLE = int(os.environ.get("STOP_CYCLE", "0"))
OUT = os.environ.get("OUT", "/tmp/opencode/stream_dump.txt")


def load_hil_dump(path):
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


def g(obj, name, default=0):
    try:
        return int(getattr(obj, name).value)
    except (AttributeError, ValueError):
        return default


@cocotb.test()
async def dump_stream(dut):
    if not DUMP:
        cocotb.log.error("DUMP not set")
        return
    samples = load_hil_dump(DUMP)
    n_samples = len(samples)
    cocotb.log.info(f"Loaded {n_samples} samples")

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    dep = dut.u_rx_pipeline.u_depuncturer
    deint = dut.u_rx_pipeline.u_deinterleaver
    pair = dut.u_rx_pipeline.u_soft_pairer
    pipe = dut.u_rx_pipeline
    vitf = dut.u_rx_pipeline.u_vit_fifo

    tags = 0
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 5000000

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    f = open(OUT, "w")
    f.write("cycle frame sym d_val d_s0 d_s1 d_state d_outc d_wrc "
            "p_val p_s p_pat p_wr p_rd p_act pa_val pa_s0 pa_s1 pa_hf "
            "vitf_full depf_full pstall dstall distall fflush ffull fidle fudone fflush_out "
            "frd_valid fbusy\n")

    sym_count = -1
    prev_done = 0

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

        d_val = g(deint, "valid_out")
        d_s0 = g(deint, "soft_out0")
        d_s1 = g(deint, "soft_out1")
        d_done = g(pipe, "deint_done")
        d_state = g(deint, "state")
        d_outc = g(deint, "out_count")
        d_wrc = g(deint, "wr_count")
        p_val = g(dep, "valid_out")
        p_s = g(dep, "soft_out")
        p_pat = g(dep, "pat_pos")
        p_wr = g(dep, "wr_entries")
        p_rd = g(dep, "rd_bits")
        p_act = g(dep, "active")
        pa_val = g(pair, "valid_out")
        pa_s0 = g(pair, "soft0")
        pa_s1 = g(pair, "soft1")
        pa_hf = g(pair, "have_first")
        vitf_full = g(pipe, "vitf_full")
        depf_full = g(pipe, "depunct_full")
        pair_stall = g(pair, "stall_in")
        dep_stall = g(dep, "stall_in")
        deint_stall = g(deint, "stall_in")
        fflush = g(vitf, "flush_in")
        ffull = g(vitf, "full")
        fidle = g(vitf, "drain_idle_cnt")
        fudone = g(vitf, "upstream_done")
        fflush_out = g(vitf, "flush_out")
        frd_valid = g(vitf, "rd_valid")
        fbusy = g(vitf, "vit_busy")

        if d_done and not prev_done:
            if tags >= FRAME - 1:
                sym_count += 1
        prev_done = d_done

        if tags == FRAME - 1:
            if d_val or p_val or pa_val or vitf_full or depf_full \
                    or fflush or fflush_out:
                f.write(f"{cycle} {tags} {sym_count} {d_val} {d_s0} {d_s1} "
                        f"{d_state} {d_outc} {d_wrc} {p_val} {p_s} {p_pat} "
                        f"{p_wr} {p_rd} {p_act} {pa_val} {pa_s0} {pa_s1} "
                        f"{pa_hf} {vitf_full} {depf_full} {pair_stall} {dep_stall} {deint_stall} "
                        f"{fflush} {ffull} "
                        f"{fidle} {fudone} {fflush_out} {frd_valid} "
                        f"{fbusy}\n")

        if int(dut.tag_valid.value) == 1:
            tags += 1
            cocotb.log.info(
                f"TAG[{tags}]: rate=0b{int(dut.tag_rate.value):04b} "
                f"fcs={'OK' if int(dut.tag_fcs_ok.value) else 'FAIL'}")
            if tags >= FRAME:
                break

        if sample_idx >= n_samples + post_zero_count:
            break

        if STOP_CYCLE and cycle >= STOP_CYCLE:
            cocotb.log.info(f"Stopped at cycle {cycle} (STOP_CYCLE)")
            break

    f.close()
    cocotb.log.info(f"Wrote {OUT}")
