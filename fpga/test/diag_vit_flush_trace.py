"""
diag_vit_flush_trace.py — Timeline capture of the viterbi flush sequence.

DIAGNOSTIC (never asserts). Records (cycle, viterbi state/window_steps/
total_steps/flush_pending, vit_fifo rd/wr/flush/upstream_done, depuncturer
valid) for the tail of a single-frame replay, and writes the final window
to /tmp/flush_trace.json.

Usage:
  REPLAY_FILE=/tmp/simreplay/minrepro_f13.json \
    make sim ... COCOTB_TEST_MODULES=diag_vit_flush_trace
"""
import json
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import reset_dut, load_adc_capture, s12_to_unsigned


def _v(sig):
    try:
        return int(sig.value)
    except (ValueError, AttributeError):
        return None


@cocotb.test()
async def trace_vit_flush(dut):
    path = os.environ.get("REPLAY_FILE")
    out_path = os.environ.get("FLUSH_TRACE_FILE", "/tmp/flush_trace.json")
    if not path:
        dut._log.warning("REPLAY_FILE not set — skipping trace")
        return

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    samples = load_adc_capture(path)
    n_samples = len(samples)
    dut._log.info(f"Replaying {path}: {n_samples} samples")

    vit = dut.u_rx_pipeline.u_viterbi
    vitf = dut.u_rx_pipeline.u_vit_fifo
    dep = dut.u_rx_pipeline.u_depuncturer

    trace = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2000000
    tag_seen = False
    tag_cycle = None

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
            if int(dut.tag_valid.value) == 1 and not tag_seen:
                tag_seen = True
                tag_cycle = cycle
        except (ValueError, AttributeError):
            pass

        # record timeline after tag (or always, but keep bounded: ring buffer)
        row = {
            'cyc': cycle,
            'v_state': _v(vit.state),
            'v_ws': _v(vit.window_steps),
            'v_ts': _v(vit.total_steps),
            'v_flush_pending': _v(vit.flush_pending),
            'v_valid_out': _v(vit.valid_out),
            'v_bit_out': _v(vit.bit_out),
            'v_decode_len': _v(vit.decode_len),
            'v_output_idx': _v(vit.output_idx),
            'v_tb_remaining': _v(vit.tb_remaining),
            'v_tb_conv_depth': _v(vit.tb_conv_depth),
            'f_rd': _v(vitf.rd_ptr),
            'f_wr': _v(vitf.wr_ptr),
            'f_flush_pending': _v(vitf.flush_pending),
            'f_upstream_done': _v(vitf.upstream_done),
            'f_drain_idle': _v(vitf.drain_idle_cnt),
            'f_flush_out': _v(vitf.flush_out),
            'dep_valid': _v(dep.valid_out),
        }
        trace.append(row)
        if len(trace) > 200000:
            trace = trace[-200000:]

        if tag_seen and sample_idx >= n_samples + post_zero_count:
            break

    # trim to tag + 3000 cycles
    if tag_cycle is not None:
        trace = [r for r in trace if r['cyc'] <= tag_cycle + 3000]
    with open(out_path, 'w') as f:
        json.dump(trace, f)
    dut._log.info(f"wrote {len(trace)} rows to {out_path} (tag_cycle={tag_cycle})")
