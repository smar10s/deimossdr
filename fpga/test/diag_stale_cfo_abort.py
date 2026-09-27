"""
diag_stale_cfo_abort.py — Diagnostic: wrong per-frame CFO on a clean frame.

WHY THIS EXISTS:
  fix/eapol-capture root cause: an OTA EAPOL response can be decoded with the
  *preceding* AP frame's CFO, not its own, because acquisition_ctrl's
  latched_phase_inc is not cleared on trigger accept (and cfo_est.start is raw
  frame_detect, so an estimate can be missing / displaced). The unit test
  showed acquisition_ctrl DOES inherit the prior frame's phase when no fresh
  cfo_done arrives. This diagnostic closes the causal link: does applying a
  wrong per-frame CFO to an otherwise-clean frame produce an S_PARSE_SIGNAL
  abort with the OTA signature (length intact, rate/parity corrupted)?

WHAT THIS DOES:
  Drives a golden rate-6 waveform through rx_pipeline exactly like
  test_rx_pipeline, but pulses cfo_done_in at the calibrated sample with a
  deliberately wrong phase_inc_in (the "stale" value). Control run uses 0.
  Reports whether SIGNAL parsed, the tag, and the RTL abort snapshot.

  This is the faithful failure condition: in the stale-latch case the live
  front-end NCO and the descriptor latch both carry the prior frame's value,
  so injecting it at cfo_done reproduces both.

NOT A GATE TEST:
  Diagnostic (diag_*). Reports numbers, never asserts.
    ./scripts/sim.sh diag_stale_cfo_abort
"""

import os
import json

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

VECTORS_DIR = os.path.join(os.path.dirname(__file__), '..', '..',
                           'extern', 'lib80211', 'vectors')

STF_END_SAMPLE = 197
CFO_DONE_SAMPLE = 155

OTA_ABORT_SIG = 0x0013E1
OTA_PHASE_INC = (237, 251)

RATE_CODE_6 = 0b1011

# phase_inc values to test: 0 (control) then a sweep across the abort threshold.
PHASE_INCS = [0, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240,
              250, 260, 280]


def load_waveform(rate_mbps):
    path = os.path.join(VECTORS_DIR, f'legacy_{rate_mbps}mbps_waveform.json')
    with open(path) as f:
        data = json.load(f)
    re_float = data['real']
    im_float = data['imag']
    peak = max(max(abs(x) for x in re_float), max(abs(x) for x in im_float))
    if peak == 0:
        peak = 1.0
    scale = 2047.0 / peak
    out = []
    for r, i in zip(re_float, im_float):
        re_q = max(-2048, min(2047, int(round(r * scale))))
        im_q = max(-2048, min(2047, int(round(i * scale))))
        out.append((re_q, im_q))
    return out


def s12_to_unsigned(v):
    return (v + 4096) & 0xFFF if v < 0 else v & 0xFFF


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.frame_detect.value = 0
    dut.stf_end.value = 0
    dut.stf_end_skip.value = 1
    dut.ltf_skip.value = 192
    dut.iq_valid_in.value = 0
    dut.iq_re_in.value = 0
    dut.iq_im_in.value = 0
    dut.phase_inc_in.value = 0
    dut.cfo_done_in.value = 0
    dut.ddr_wr_ptr.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)


def _sign16(v):
    return v - 65536 if v >= 32768 else v


def _decode_sig_bits(v):
    return {
        'rate': v & 0x0F,
        'length': (v >> 5) & 0xFFF,
        'parity_ok': (bin(v & 0x3FFFF).count('1') % 2) == 0,
    }


async def run_one(dut, iq_samples, phase_inc):
    """Feed one frame, cfo_done at CFO_DONE_SAMPLE with `phase_inc`."""
    n = len(iq_samples)
    sample_idx = 0
    grace = 0
    signal_parsed = False
    parsed_rate = parsed_length = 0
    tag_seen = False
    tag_rate = tag_length = tag_fcs = 0

    # Pulse frame_detect to start acquisition_ctrl (as run_decode does).
    dut.frame_detect.value = 1
    await RisingEdge(dut.clk)
    dut.frame_detect.value = 0
    await RisingEdge(dut.clk)

    for cycle in range(2_000_000):
        await RisingEdge(dut.clk)

        if sample_idx < n and cycle % 5 == 0:
            re_q, im_q = iq_samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_re_in.value = s12_to_unsigned(re_q)
            dut.iq_im_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0
            if sample_idx >= n:
                grace += 1

        if sample_idx == STF_END_SAMPLE:
            dut.stf_end.value = 1
        elif sample_idx == STF_END_SAMPLE + 1:
            dut.stf_end.value = 0

        if sample_idx == CFO_DONE_SAMPLE:
            dut.cfo_done_in.value = 1
            dut.phase_inc_in.value = phase_inc & 0xFFFF
        elif sample_idx == CFO_DONE_SAMPLE + 1:
            dut.cfo_done_in.value = 0

        try:
            if int(dut.signal_valid.value) == 1 and not signal_parsed:
                signal_parsed = True
                parsed_rate = int(dut.parsed_rate.value)
                parsed_length = int(dut.parsed_length.value)
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.tag_valid.value) == 1:
                tag_seen = True
                tag_rate = int(dut.tag_rate.value)
                tag_length = int(dut.tag_length.value)
                tag_fcs = int(dut.tag_fcs_ok.value)
                break
        except (ValueError, AttributeError):
            pass

        try:
            if int(dut.seq_done.value) == 1 and not tag_seen:
                break
        except (ValueError, AttributeError):
            pass

        if sample_idx >= n and grace > 400_000:
            break

    cnts = int(dut.diag_abort_cnts.value)
    return {
        'signal_parsed': signal_parsed,
        'parsed_rate': parsed_rate,
        'parsed_length': parsed_length,
        'tag_seen': tag_seen,
        'tag_rate': tag_rate,
        'tag_length': tag_length,
        'tag_fcs': tag_fcs,
        'abort_sig': int(dut.diag_abort_sig.value) & 0xFFFFFF,
        'abort_ctx': _sign16(int(dut.diag_abort_ctx.value) & 0xFFFF),
        'abort_sig_cnt': cnts & 0xFF,
        'abort_rate_cnt': (cnts >> 8) & 0xFF,
    }


@cocotb.test()
async def diag_stale_cfo_abort(dut):
    Clock(dut.clk, 10, unit="ns").start()
    iq = load_waveform(6)

    dut._log.info("=" * 72)
    dut._log.info("STALE-CFO ABORT DIAGNOSTIC — clean rate-6 frame, wrong cfo_done")
    dut._log.info(f"  OTA target: sig=0x{OTA_ABORT_SIG:06X}, "
                  f"phase_inc={OTA_PHASE_INC[0]}..{OTA_PHASE_INC[1]}")
    dut._log.info("=" * 72)

    for pi in PHASE_INCS:
        await reset_dut(dut)
        r = await run_one(dut, iq, pi)
        d = _decode_sig_bits(r['abort_sig'])
        status = "TAG OK" if (r['tag_seen'] and r['tag_fcs']) else (
            "abort" if r['abort_sig_cnt'] > 0 or r['abort_rate_cnt'] > 0
            else "no-tag")
        match = "  <== OTA MATCH" if r['abort_sig'] == OTA_ABORT_SIG else ""
        dut._log.info(
            f"phase_inc={pi:5d}  -> {status:7s} "
            f"sig_parsed={r['signal_parsed']} "
            f"parsed=(rate=0b{r['parsed_rate']:04b},len={r['parsed_length']}) "
            f"tag=(rate=0b{r['tag_rate']:04b},len={r['tag_length']},fcs={r['tag_fcs']}) "
            f"abort_cnt(sig/rate)={r['abort_sig_cnt']}/{r['abort_rate_cnt']} "
            f"snap=0x{r['abort_sig']:06X}"
            f"(rate=0b{d['rate']:04b},len={d['length']},"
            f"parity={'ok' if d['parity_ok'] else 'BAD'}) "
            f"ctx={r['abort_ctx']:+d}{match}")

    dut._log.info("=" * 72)
    dut._log.info("(Diagnostic complete — no assertions)")
