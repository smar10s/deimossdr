"""
diag_ota_window_replay.py — Replay a captured OTA DDR window through the fabric.

WHY THIS EXISTS:
  OTA captures lose a handshake frame (the investigation targets the STA
  response M2). deimos_ota_capture records one long raw-DDR window around an
  M1-triggered handshake, plus the tag log the fabric produced for that window.
  This diag feeds the same window through the rx_frontend DUT and reports which
  frames the fabric tags on replay.

  The decision fork (see STATUS.md):
    - M2 not tagged OTA but tagged in sim  -> defect is UPSTREAM of the fabric
      (AGC/analog path); no RTL change fixes it.
    - M2 not tagged in sim either          -> defect is IN the fabric; this is
      the failing vector for offline iteration (systematic-debugging loop).
    - M2 tagged OTA                        -> no loss in this window.

  Pair with scripts/host/validate_window.c (independent lib80211 decode) to
  separate "fabric defect" from "IQ unrecoverable".

NOT A GATE TEST:
  Diagnostic (diag_*). Reports numbers, never asserts. Run via:
    OTA_WINDOW_JSON=/tmp/ddr_win_stimulus.json \
      ./scripts/sim.sh diag_ota_window_replay
  Default path: captures/ota_window_capture.json (skips if absent).
"""

import os
import json

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    reset_dut, load_adc_capture, s12_to_unsigned, SAMPLE_RATE,
)

DEFAULT_JSON = os.path.join(os.path.dirname(__file__), '..', '..',
                            'captures', 'ota_window_capture.json')


def _norm(tag):
    """Normalize a sim tag ({'length','fcs_ok'}) or sidecar tag ({'len','fcs'})."""
    length = int(tag.get('length', tag.get('len', 0)))
    fcs = bool(tag.get('fcs_ok', tag.get('fcs', False)))
    return length, fcs


def _present(tags, length):
    return any(l == length and f for l, f in map(_norm, tags))


@cocotb.test()
async def diag_ota_window_replay(dut):
    path = os.environ.get('OTA_WINDOW_JSON', DEFAULT_JSON)
    if not os.path.exists(path):
        cocotb.log.info(f"{path} not found — capture one with deimos_ota_capture first")
        return

    with open(path) as f:
        meta = json.load(f)
    samples = load_adc_capture(path)
    n_samples = len(samples)

    trigger_len = int(meta.get('trigger_len', 137))
    absent_len = int(meta.get('absent_len', 159))
    fabric_tags = meta.get('fabric_tags', [])

    dut._log.info("=" * 72)
    dut._log.info(f"OTA window replay: {path}")
    dut._log.info(f"  {n_samples} samples ({n_samples / SAMPLE_RATE * 1000:.1f} ms), "
                  f"channel={meta.get('channel')}, selected={meta.get('selected')}")
    dut._log.info(f"  OTA fabric tags: {len(fabric_tags)}")
    for t in fabric_tags:
        length, fcs = _norm(t)
        dut._log.info(f"    OTA tag: rate={t.get('rate')} len={length} fcs={fcs} "
                      f"offset={t.get('offset')}")
    dut._log.info("=" * 72)

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Feed the whole window continuously and collect ALL tags (as in
    # test_eapol_burst_continuous). Post-stream zeros flush the pipeline.
    tags = []
    aborts = []
    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2_000_000

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
            if int(dut.tag_valid.value) == 1:
                tag = {
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'sample': sample_idx,
                    # L-SIG bits + {ltf1_offset, phase_inc} at the last tag
                    # (D28 observability): what SIGNAL and timing/CFO the
                    # fabric used for this frame.
                    'tag_sig': int(dut.diag_tag_sig.value) & 0xFFFFFF,
                    'tag_ctx': int(dut.diag_tag_ctx.value) & 0xFFFFFFFF,
                }
                tags.append(tag)
                fcs_str = 'OK' if tag['fcs_ok'] else 'FAIL'
                ltf1 = tag['tag_ctx'] >> 16
                phinc = tag['tag_ctx'] & 0xFFFF
                dut._log.info(f"  SIM frame {len(tags)}: rate=0b{tag['rate']:04b}, "
                              f"len={tag['length']}, fcs={fcs_str} @ sample ~{sample_idx} "
                              f"tag_sig=0x{tag['tag_sig']:06x} "
                              f"ltf1={ltf1}({ltf1-32768 if ltf1>=32768 else ltf1}) "
                              f"phase_inc={phinc}({phinc-65536 if phinc>=32768 else phinc})")
            if int(dut.tag_abort.value) == 1:
                aborts.append({
                    'sample': sample_idx,
                    'sig': int(dut.diag_abort_sig.value) & 0xFFFFFF,
                    'ctx': int(dut.diag_abort_ctx.value),
                })
                dut._log.info(f"  SIM ABORT @ sample ~{sample_idx} "
                              f"abort_sig=0x{aborts[-1]['sig']:06x} ctx=0x{aborts[-1]['ctx']:08x}")
        except (ValueError, AttributeError):
            pass

        if sample_idx >= n_samples + post_zero_count:
            break

        if cycle > 0 and cycle % 2_000_000 == 0:
            dut._log.info(f"  cycle {cycle}: {len(tags)} sim tags, "
                          f"sample {sample_idx}/{n_samples}")

    # --- Report: OTA vs sim tag sets ---
    fcs_ok_sim = [t for t in tags if t['fcs_ok']]
    m1_sim = _present(fcs_ok_sim, trigger_len)
    m2_sim = _present(fcs_ok_sim, absent_len)
    m1_ota = _present(fabric_tags, trigger_len)
    m2_ota = _present(fabric_tags, absent_len)

    dut._log.info("--- OTA window replay summary ---")
    dut._log.info(f"  SIM tags: {len(tags)} ({len(fcs_ok_sim)} FCS OK)")
    try:
        acq = int(dut.diag_abort_cnts.value)
        dut._log.info(
            f"  acq: found={int(dut.diag_frames_found.value)} "
            f"rejected={int(dut.diag_frames_rejected.value)} "
            f"decode-abort={{sig={acq & 0xFF}, rate={(acq >> 8) & 0xFF}, "
            f"ow={(acq >> 16) & 0xFF}, wd={(acq >> 24) & 0xFF}}}")
    except (ValueError, AttributeError):
        pass
    dut._log.info(f"  tag_abort events: {len(aborts)}")
    for a in aborts[:20]:
        dut._log.info(f"    abort @ ~{a['sample']} sig=0x{a['sig']:06x} ctx=0x{a['ctx']:08x}")
    dut._log.info(f"  trigger (len {trigger_len}): OTA={m1_ota} SIM={m1_sim}")
    dut._log.info(f"  absent  (len {absent_len}): OTA={m2_ota} SIM={m2_sim}")

    if m2_ota:
        verdict = ("NO LOSS IN THIS WINDOW — the fabric tagged the absent frame "
                   "OTA; capture a different window.")
    elif m2_sim:
        verdict = ("UPSTREAM — the fabric decodes the absent frame on replay of "
                   "the same IQ, so the OTA miss was upstream of the fabric "
                   "(AGC/analog). No RTL change fixes this.")
    else:
        verdict = ("FABRIC — the fabric does not decode the absent frame on "
                   "replay either. This is the failing vector: iterate offline.")
    dut._log.info(f"  VERDICT: {verdict}")
    dut._log.info("=" * 72)
