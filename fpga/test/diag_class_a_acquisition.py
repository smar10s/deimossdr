"""
diag_class_a_acquisition.py — Measure class-A acquisition collision (sim-only).

WHY:
  OTA EAPOL captures lose the STA responses (M2/M4) that follow an AP frame at
  SIFS. The acquisition analysis (docs/acquisition-window-fix.md) established that
  the delta-160 repro is a class-A acquisition collision: the interferer's STF completes
  first, acquisition locks onto the interferer, and the target M2 is never
  acquired. Before changing RTL we measure whether a second, correctly-timed
  M2 acquisition is reachable, and which gate drops it.

WHAT THIS MEASURES (no RTL change):
  Runs the class-A geometry plus controls and reports the acquisition outcome
  from the RTL's *port-level* diagnostics — no internal-signal reads:
    - diag_frames_found / diag_frames_rejected : descriptors pushed / rejected
      by acquisition_ctrl (top-level rx_frontend ports).
    - tag / abort snapshots' ltf1_offset : which frame's LTF was acquired.
  Answers: how many acquisitions happen, and does any descriptor point at the
  target M2's LTF (m2_start + 192)?

  The internal "why" (stf_detect detected_latch / rearm_cnt, acquisition_ctrl
  state / trigger_distance at each frame_detect) is read from a VCD, per
  docs/debugging.md — cocotb internal-signal reads are unreliable.
    make -C fpga/test waves TARGET=diag_class_a_acquisition
    python scripts/vcd_query.py list waves/diag_class_a_acquisition_latest.vcd

NOT A GATE TEST: diagnostic (diag_*), reports numbers, never asserts.
  ./scripts/sim.sh diag_class_a_acquisition
  DIAG_CLASSA_FILTER=class_a ./scripts/sim.sh diag_class_a_acquisition
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from frontend_helpers import reset_dut, quantize_12bit
import diag_cochannel_overlap as D

FILTER = os.environ.get('DIAG_CLASSA_FILTER', '')

# Focused cases. delta is relative to M2's frame start (negative = interferer
# starts first). 'clean' has no interferer.
CASES = (
    ('class_a',  -160, D.INTF_CFO_HZ, 0.0),
    ('same_cfo', -160, 0.0,            0.0),
    ('clean',    None, None,           None),
)


def _compose_clean():
    """Target EAPOL pair alone (no interferer), same seeds/SNR as _compose."""
    m1 = D._frame(D.RATE, D.M1_PSDU, seed=11, scrambler_seed=7)
    m2 = D._frame(D.RATE, D.M2_PSDU, seed=22, scrambler_seed=23)
    target = np.concatenate([
        np.zeros(D.PRE_SAMPLES, np.complex64), m1,
        np.zeros(D.SIFS_SAMPLES, np.complex64), m2,
    ])
    m2_start = D.PRE_SAMPLES + len(m1) + D.SIFS_SAMPLES
    return D.add_awgn(target, D.SNR_DB, seed=99), m2_start


def _which(lt, m2_start, p):
    cands = [("M1", D.PRE_SAMPLES + 192), ("TARGET M2", m2_start + 192),
             ("INTERFERER", None if p is None else p + 192)]
    cands = [(n, v) for n, v in cands if v is not None]
    return min(cands, key=lambda c: abs(lt - c[1]))[0]


@cocotb.test()
async def diag_class_a_acquisition(dut):
    """Report acquisition outcome for class-A geometry vs controls."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    dut._log.info("=" * 72)
    dut._log.info("CLASS-A ACQUISITION MEASUREMENT — target M1->SIFS->M2 + interferer")
    dut._log.info(f"  target rate {D.RATE} M1={D.M1_PSDU + 4}B M2={D.M2_PSDU + 4}B, "
                  f"SNR {D.SNR_DB} dB")
    dut._log.info(f"  interferer rate {D.INTF_RATE} {D.INTF_PSDU + 4}B "
                  f"cfo={D.INTF_CFO_HZ:.0f}Hz, amp 0 dB, delta -160 (starts first)")
    dut._log.info("  port reads: diag_frames_found / diag_frames_rejected / "
                  "tag+abort ltf1_offset")
    dut._log.info("=" * 72)

    for name, delta, intf_cfo, amp_db in CASES:
        if FILTER and FILTER not in name:
            continue
        await reset_dut(dut)

        if delta is None:
            stream, m2_start = _compose_clean()
            p = None
        else:
            stream, m2_start, p = D._compose(delta, amp_db, intf_cfo)
        samples = quantize_12bit(stream)
        r = await D.feed_and_observe(dut, samples)

        tag_str = ", ".join(
            f"{D.RATE_NAME.get(t['rate'], hex(t['rate']))}/{t['length']}B/"
            f"{'OK' if t['fcs_ok'] else 'FCS!'}/pi={t['tag_phase']:+d}/"
            f"{_which(t['tag_ltf'], m2_start, p)}@ltf{t['tag_ltf']}"
            for t in r['tags']) or "(none)"

        abort_str = ""
        for a in r['aborts']:
            if not a['delta']['sig']:
                continue
            d = D._decode_sig_bits(a['sig_raw'])
            abort_str += (f" [sig=0x{a['sig_raw']:06X} "
                          f"rate=0b{d['rate']:04b} len={d['length']} "
                          f"par={d['parity_ok']} pi={a['ctx_phase_inc']:+d} "
                          f"{_which(a['ctx_ltf'], m2_start, p)}@ltf{a['ctx_ltf']}]")

        counts = {k: sum(a['delta'][k] for a in r['aborts'])
                  for k in ('sig', 'rate', 'ow', 'wd')}
        dut._log.info("-" * 72)
        dut._log.info(f"[{name}] m2_start={m2_start}"
                      + (f" intf_start={p} (target_ltf1={m2_start + 192}"
                         f" intf_ltf1={p + 192})" if p is not None else ""))
        dut._log.info(f"  descriptors: found={r['frames_found']} "
                      f"rejected={r['frames_rejected']}")
        dut._log.info(f"  tags: {tag_str}")
        dut._log.info(f"  aborts sig/rate/ow/wd = "
                      f"{counts['sig']}/{counts['rate']}/{counts['ow']}/{counts['wd']}"
                      f"{abort_str}")

    dut._log.info("=" * 72)
    dut._log.info("(Diagnostic complete — no assertions)")
