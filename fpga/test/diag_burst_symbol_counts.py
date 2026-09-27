"""
diag_burst_symbol_counts.py — Diagnostic: per-symbol datapath counts during
HIL dump replay (deint_done-bounded intervals), covering the first N frames.

Usage:
  make -C fpga/test diag_burst_symbol_counts DUMP=... [TAGS=2]
"""
import os
import struct

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, s12_to_unsigned

DUMP = os.environ.get("DUMP", "")
TAGS_MAX = int(os.environ.get("TAGS", "2"))


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


@cocotb.test()
async def count_symbols(dut):
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

    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 5000000

    # per-interval counters, grouped by frame (frame = tags seen so far)
    cur = None
    rows = []

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

        try:
            d_vin = int(deint.valid_in.value)
            d_done = int(pipe.deint_done.value)
            d_wrc = int(deint.wr_count.value)
            d_state = int(deint.state.value)
            d_outc = int(deint.out_count.value)
            dep_val = int(dep.valid_out.value)
            dep_act = int(dep.active.value)
            dep_pat = int(dep.pat_pos.value)
            dep_wr = int(dep.wr_entries.value)
            dep_rd = int(dep.rd_bits.value)
            pair_val = int(pair.valid_out.value)
            skip = int(dep.valid_in.value) and int(dep.fifo_full.value)
            stall_deint = int(deint.stall_in.value)
            vitf_full = int(pipe.vitf_full.value)
            depf_full = int(pipe.depunct_full.value)
        except (AttributeError, ValueError) as e:
            cocotb.log.warning(f"signal access failed: {e}")
            break

        if cur is None:
            cur = {"n": 0, "frame": len(tags), "d_vin": 0, "d_wrc_at_done": 0,
                   "dep_val": 0, "pair_val": 0, "skip": 0, "stall_deint": 0,
                   "vitf_full": 0, "depf_full": 0,
                   "dep_pat_end": 0, "dep_act_end": 0,
                   "dep_wr_end": 0, "dep_rd_end": 0, "d_state_end": 0,
                   "d_outc_end": 0}

        cur["d_vin"] += d_vin
        cur["dep_val"] += dep_val
        cur["pair_val"] += pair_val
        cur["skip"] += skip
        cur["stall_deint"] += stall_deint
        cur["vitf_full"] += vitf_full
        cur["depf_full"] += depf_full
        cur["d_wrc_at_done"] = d_wrc
        cur["dep_pat_end"] = dep_pat
        cur["dep_act_end"] = dep_act
        cur["dep_wr_end"] = dep_wr
        cur["dep_rd_end"] = dep_rd
        cur["d_state_end"] = d_state
        cur["d_outc_end"] = d_outc

        if d_done:
            cur["n"] = len([r for r in rows if r["frame"] == len(tags)])
            rows.append(cur)
            cur = None

        if int(dut.tag_valid.value) == 1:
            tags.append({
                "rate": int(dut.tag_rate.value),
                "length": int(dut.tag_length.value),
                "fcs_ok": int(dut.tag_fcs_ok.value),
                "sample": sample_idx,
            })
            cocotb.log.info(
                f"  === TAG[{len(tags)}]: rate=0b{tags[-1]['rate']:04b} "
                f"len={tags[-1]['length']} "
                f"fcs={'OK' if tags[-1]['fcs_ok'] else 'FAIL'}"
            )
            if len(tags) >= TAGS_MAX:
                break

        if sample_idx >= n_samples + post_zero_count:
            break

    for r in rows:
        cocotb.log.info(
            f"  f{r['frame']} sym{r['n']}: d_vin={r['d_vin']} "
            f"dep_bits={r['dep_val']} pair_val={r['pair_val']} "
            f"deint_stall={r['stall_deint']} dep_skip={r['skip']} "
            f"vitf_full={r['vitf_full']} depf_full={r['depf_full']} "
            f"wrc_end={r['d_wrc_at_done']} pat_end={r['dep_pat_end']} "
            f"act_end={r['dep_act_end']} dstate={r['d_state_end']} "
            f"doutc={r['d_outc_end']} dep_wr={r['dep_wr_end']} "
            f"dep_rd={r['dep_rd_end']}"
        )
    cocotb.log.info(f"SUMMARY: {len(tags)} tags, "
                    f"{sum(1 for t in tags if t['fcs_ok'])} OK, "
                    f"{sum(1 for t in tags if not t['fcs_ok'])} FAIL")
