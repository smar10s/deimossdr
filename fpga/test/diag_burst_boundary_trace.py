"""
diag_burst_boundary_trace.py — Diagnostic: replay a HIL dump and log
frame-boundary signals (flush chain, stall chain) for the first N frames.

Usage:
  make -C fpga/test diag_burst_boundary_trace DUMP=... [TRACE_FRAMES=3]
  (env: DUMP = HIL dump path; TRACE_FRAMES = how many frames to trace)
"""
import os
import struct

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, s12_to_unsigned

DUMP = os.environ.get("DUMP", "")
TRACE_FRAMES = int(os.environ.get("TRACE_FRAMES", "3"))


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
async def trace_burst_boundary(dut):
    if not DUMP:
        cocotb.log.error("DUMP not set")
        return
    samples = load_hil_dump(DUMP)
    n_samples = len(samples)
    cocotb.log.info(f"Loaded {n_samples} samples, tracing first {TRACE_FRAMES} frames")

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    eng = dut.u_rx_pipeline.u_decode_engine
    fifo = dut.u_rx_pipeline.u_vit_fifo
    dep = dut.u_rx_pipeline.u_depuncturer
    deint = dut.u_rx_pipeline.u_deinterleaver
    vit = dut.u_rx_pipeline.u_viterbi

    bypass = os.environ.get("STALL_BYPASS", "0") == "1"
    if bypass:
        cocotb.log.info("STALL_BYPASS=1: forcing stall inputs low")
        dep.stall_in.value = 0
        deint.stall_in.value = 0
        dut.u_rx_pipeline.u_soft_pairer.stall_in.value = 0

    tags = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 5000000

    prev = {}
    trace_until_tag = TRACE_FRAMES

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

        if len(tags) < trace_until_tag:
            try:
                sigs = {
                    "state": int(eng.state.value),
                    "vit_flush": int(eng.vit_flush.value),
                    "flush_in": int(fifo.flush_in.value),
                    "flush_out": int(fifo.flush_out.value),
                    "flush_pending": int(fifo.flush_pending.value),
                    "upstream_done": int(fifo.upstream_done.value),
                    "drain_idle_cnt": int(fifo.drain_idle_cnt.value),
                    "empty": int(fifo.empty.value),
                    "full": int(fifo.full.value),
                    "vit_busy": int(vit.busy.value),
                    "dep_fifo_full": int(dep.fifo_full.value),
                    "dep_valid": int(dep.valid_out.value),
                    "deint_done": int(deint.deint_done.value),
                    "deint_state": int(deint.state.value),
                    "deint_valid": int(deint.valid_out.value),
                    "stall_dep": int(dep.stall_in.value),
                    "vit_state": int(vit.state.value),
                    "f_wr_ptr": int(fifo.wr_ptr.value),
                    "f_rd_ptr": int(fifo.rd_ptr.value),
                    "f_rd_valid": int(fifo.rd_valid.value),
                    "f_pf_valid": int(fifo.pf_valid.value),
                    "f_rd_inflight": int(fifo.rd_in_flight.value),
                    "f_overflow": int(fifo.overflow.value),
                    "vit_win_steps": int(vit.window_steps.value),
                    "vit_valid_in": int(vit.valid_in.value),
                    "pair_valid": int(dut.u_rx_pipeline.u_soft_pairer.valid_out.value),
                    "pat_pos": int(dep.pat_pos.value),
                    "code_rate": int(dep.code_rate.value),
                    "dep_active": int(dep.active.value),
                    "deint_wrc": int(deint.wr_count.value),
                    "deint_outc": int(deint.out_count.value),
                    "deint_lat": int(deint.lat_mode.value),
                    "deint_vin": int(deint.valid_in.value),
                }
            except (AttributeError, ValueError) as e:
                cocotb.log.warning(f"signal access failed: {e}")
                break

            changed = sigs != prev
            prev = sigs
            if changed:
                cocotb.log.info(
                    f"  [c{cycle} s~{sample_idx}] state={sigs['state']} "
                    f"vit_flush={sigs['vit_flush']} flush_in={sigs['flush_in']} "
                    f"flush_out={sigs['flush_out']} "
                    f"fl_pend={sigs['flush_pending']} up_done={sigs['upstream_done']} "
                    f"idle={sigs['drain_idle_cnt']} empty={sigs['empty']} "
                    f"full={sigs['full']} busy={sigs['vit_busy']} "
                    f"dep_full={sigs['dep_fifo_full']} dep_val={sigs['dep_valid']} "
                    f"dep_stall={sigs['stall_dep']} "
                    f"deint_done={sigs['deint_done']} deint_state={sigs['deint_state']} "
                    f"deint_val={sigs['deint_valid']} vit_state={sigs['vit_state']} "
                    f"wr={sigs['f_wr_ptr']} rd={sigs['f_rd_ptr']} "
                    f"rdv={sigs['f_rd_valid']} pfv={sigs['f_pf_valid']} "
                    f"inf={sigs['f_rd_inflight']} ovf={sigs['f_overflow']} "
                    f"winsteps={sigs['vit_win_steps']} vit_in={sigs['vit_valid_in']} "
                    f"pair={sigs['pair_valid']} pat_pos={sigs['pat_pos']} "
                    f"code_rate={sigs['code_rate']} dep_act={sigs['dep_active']} "
                    f"d_wrc={sigs['deint_wrc']} d_outc={sigs['deint_outc']} "
                    f"d_lat={sigs['deint_lat']} d_vin={sigs['deint_vin']}"
                )

        if int(dut.tag_valid.value) == 1:
            tag = {
                "rate": int(dut.tag_rate.value),
                "length": int(dut.tag_length.value),
                "fcs_ok": int(dut.tag_fcs_ok.value),
                "sample": sample_idx,
            }
            tags.append(tag)
            cocotb.log.info(
                f"  === TAG[{len(tags)}]: rate=0b{tag['rate']:04b} "
                f"len={tag['length']} fcs={'OK' if tag['fcs_ok'] else 'FAIL'} "
                f"@ sample ~{sample_idx}"
            )

        if sample_idx >= n_samples + post_zero_count:
            break

    cocotb.log.info(f"SUMMARY: {len(tags)} tags, "
                    f"{sum(1 for t in tags if t['fcs_ok'])} OK, "
                    f"{sum(1 for t in tags if not t['fcs_ok'])} FAIL")
