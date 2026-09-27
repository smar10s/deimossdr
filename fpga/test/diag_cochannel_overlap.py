"""
diag_cochannel_overlap.py — Diagnostic: co-channel interferer over target STF.

WHY THIS EXISTS:
  OTA EAPOL captures lose the STA responses (M2/M4) that follow an AP frame at
  SIFS. They abort in decode_engine.v at S_PARSE_SIGNAL with the L-SIG length
  intact while the RATE nibble and parity are corrupted (see
  docs/acquisition-window-fix.md).

  An earlier reading attributed this to the target M2's coarse CFO being biased
  toward the interferer's CFO, which motivated an LTF fine-CFO stage. That
  reading is wrong. The repro shows the failure is a two-part acquisition
  collision, not a CFO error:

    1. When the interferer's preamble precedes the target's by 0 < delta < 256
       samples, the interferer's STF completes first, frame_detect fires on it,
       and the target's trigger lands inside acquisition_ctrl's
       MIN_TRIGGER_DISTANCE (=256) window — so the target is never acquired.
    2. The descriptor then points at the *interferer's* LTF. The interferer's
       own coarse CFO is correct (measured from its clean STF), but its LTF
       collides with the target's STF, so its channel estimate fails and it
       aborts. The target M2 is simply lost.

  The abort's latched phase_inc therefore equals the interferer's real CFO
  (correct for the frame that was acquired), NOT a biased estimate on M2.

WHAT THIS MEASURES:
  A target EAPOL pair M1(AP,137B) -> SIFS -> M2(STA,159B), rate 6, CFO ~0, is
  summed with an independent interferer frame from a "second BSS" at a
  controllable CFO, power, and timing offset. The offset slides the interferer
  across M2's preamble. For each run it reads the RTL's own diagnostics
  (diag_abort_cnts / diag_abort_sig / diag_abort_ctx) and reports M2's fate.

  The interferer PSDU length is DELIBERATELY DISTINCT from the target M2's
  (104B vs 159B) so the abort's L-SIG length identifies which frame was
  acquired: len 159 => target M2, len 104 => interferer. (Both were 159B in the
  original repro, which is why the "OTA-shaped len 159" match was ambiguous —
  it was the interferer.)

  Classes of overlap:
    A — acquisition collision: interferer preamble 0 < delta < 256 before target
    B — merged preamble:       delta ~ 0 (both preambles coincide)
    C — STF-only overlap:      interferer tail overlaps the target STF but ends
                               before the target LTF (target still acquired,
                               coarse biased, LTF clean) — the only class fine
                               CFO can address.

NOT A GATE TEST:
  Diagnostic (diag_*). Reports numbers, never asserts. Run via:
    ./scripts/sim.sh diag_cochannel_overlap
  Narrow the sweep with:
    DIAG_COCHAN_FILTER=delta0 ./scripts/sim.sh diag_cochannel_overlap
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from frontend_helpers import quantize_12bit, reset_dut, s12_to_unsigned, SAMPLE_RATE

LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, os.path.abspath(LIB80211_PYTHON))
from py80211.gen_ofdm_frame import generate_frame
from py80211.impairments import add_cfo, add_awgn

# OTA reference: M2 is 159B at rate 6, L-SIG snapshot 0x0013E1, abort phase ~240-282.
OTA_ABORT_SIG = 0x0013E1
OTA_PHASE_INC = (240, 282)

RATE = 6
M1_PSDU = 133          # 137B incl FCS
M2_PSDU = 155          # 159B incl FCS
PRE_SAMPLES = 500      # silence before M1
SIFS_SAMPLES = 320     # 16 us at 20 MSPS
SNR_DB = 35.0

# Interferer: a same-channel frame from a second BSS at ~78 kHz (≈255 LSB).
INTF_CFO_HZ = 78000.0
INTF_RATE = 6
# PSDU length deliberately distinct from the target M2 (155 -> 159B) so the
# abort's L-SIG length identifies which frame was acquired. See module docstring.
INTF_PSDU = 100        # 104B (target M2 is 159B)

# Same-CFO control: an overlapping interferer at the TARGET's CFO (0) cannot
# bias the estimate. If M2 still aborts here, the failure is direct SIGNAL
# interference, not a biased estimate.
INTF_CFOS = (INTF_CFO_HZ, 0.0)

# Slide the interferer relative to M2's frame start (negative = starts first).
DELTAS = (-160, -128, -96, -64, -32, 0, 32, 64, 96, 128, 160)
# Interferer power sweep: maps the bias-vs-abort threshold. Filter with
# DIAG_COCHAN_FILTER (e.g. "delta-160") to keep a run focused.
INTF_AMP_DB = (0.0, -3.0, -6.0, -9.0, -12.0, -15.0)

FILTER = os.environ.get('DIAG_COCHAN_FILTER', '')


def _frame(rate_mbps, psdu_len, cfo_hz=0.0, seed=1, scrambler_seed=1):
    rng = np.random.default_rng(seed)
    psdu = bytes(rng.integers(0, 256, size=psdu_len, dtype=np.uint8))
    iq, _meta = generate_frame(rate_mbps, psdu, scrambler_seed)
    iq = np.asarray(iq, dtype=np.complex64)
    if cfo_hz:
        iq = add_cfo(iq, cfo_hz, SAMPLE_RATE)
    return iq


def _compose(delta, intf_amp_db, intf_cfo_hz):
    """Target EAPOL pair + interferer offset by `delta` from M2 start."""
    m1 = _frame(RATE, M1_PSDU, seed=11, scrambler_seed=7)
    m2 = _frame(RATE, M2_PSDU, seed=22, scrambler_seed=23)
    target = np.concatenate([
        np.zeros(PRE_SAMPLES, np.complex64), m1,
        np.zeros(SIFS_SAMPLES, np.complex64), m2,
    ])
    m2_start = PRE_SAMPLES + len(m1) + SIFS_SAMPLES

    intf = _frame(INTF_RATE, INTF_PSDU, cfo_hz=intf_cfo_hz,
                  seed=33, scrambler_seed=41) * (10.0 ** (intf_amp_db / 20.0))

    p = m2_start + delta
    assert p >= 0, delta
    total = max(len(target), p + len(intf))
    out = np.zeros(total, np.complex64)
    out[:len(target)] += target
    out[p:p + len(intf)] += intf
    out = add_awgn(out, SNR_DB, seed=99)
    return out, m2_start, p


def _sign16(v):
    return v - 65536 if v >= 32768 else v


def _unpack_abort_cnts(v):
    return {
        'sig': v & 0xFF,
        'rate': (v >> 8) & 0xFF,
        'ow': (v >> 16) & 0xFF,
        'wd': (v >> 24) & 0xFF,
    }


def _decode_sig_bits(v):
    return {
        'rate': v & 0x0F,
        'length': (v >> 5) & 0xFFF,
        'parity_ok': (bin(v & 0x3FFFF).count('1') % 2) == 0,
    }


RATE_NAME = {0b1011: '6M', 0b1111: '9M', 0b1010: '12M', 0b1110: '18M',
             0b1001: '24M', 0b1101: '36M', 0b1000: '48M', 0b1100: '54M'}


async def feed_and_observe(dut, samples):
    """Feed one stream at 1-per-5 clocks; collect tags and abort events."""
    n_samples = len(samples)
    post_zeros = 8192
    total_feed = n_samples + post_zeros

    tags = []
    aborts = []
    prev_cnts = _unpack_abort_cnts(int(dut.diag_abort_cnts.value))
    try:
        prev_ff = int(dut.diag_frames_found.value)
        prev_fr = int(dut.diag_frames_rejected.value)
    except (ValueError, AttributeError):
        prev_ff = prev_fr = 0
    sample_idx = 0
    valid_counter = 0

    for _cycle in range(total_feed * 5 + 100000):
        await RisingEdge(dut.clk)

        if valid_counter == 0:
            if sample_idx < n_samples:
                re_q, im_q = samples[sample_idx]
                dut.iq_valid_in.value = 1
                dut.iq_i_in.value = s12_to_unsigned(re_q)
                dut.iq_q_in.value = s12_to_unsigned(im_q)
                sample_idx += 1
            elif sample_idx < total_feed:
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
                tags.append({
                    'sample': sample_idx,
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'tag_sig': int(dut.diag_tag_sig.value) & 0xFFFFFF,
                    'tag_phase': _sign16(int(dut.diag_tag_ctx.value) & 0xFFFF),
                    'tag_ltf': (int(dut.diag_tag_ctx.value) >> 16) & 0xFFFF,
                })
        except (ValueError, AttributeError):
            pass

        try:
            cnts = _unpack_abort_cnts(int(dut.diag_abort_cnts.value))
            if cnts != prev_cnts:
                delta = {k: (cnts[k] - prev_cnts[k]) & 0xFF for k in cnts}
                aborts.append({
                    'sample': sample_idx,
                    'delta': delta,
                    'sig_raw': int(dut.diag_abort_sig.value) & 0xFFFFFF,
                    'ctx_phase_inc': _sign16(int(dut.diag_abort_ctx.value) & 0xFFFF),
                    'ctx_ltf': (int(dut.diag_abort_ctx.value) >> 16) & 0xFFFF,
                })
                prev_cnts = cnts
        except (ValueError, AttributeError):
            pass

        if sample_idx >= total_feed:
            await ClockCycles(dut.clk, 200)
            break

    try:
        frames_found = (int(dut.diag_frames_found.value) - prev_ff) & 0xFF
        frames_rejected = (int(dut.diag_frames_rejected.value) - prev_fr) & 0xFF
    except (ValueError, AttributeError):
        frames_found = frames_rejected = 0

    return {'tags': tags, 'aborts': aborts,
            'frames_found': frames_found, 'frames_rejected': frames_rejected}


@cocotb.test()
async def diag_cochannel_overlap(dut):
    """Slide a co-channel interferer across the target M2 preamble."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    dut._log.info("=" * 72)
    dut._log.info("CO-CHANNEL OVERLAP DIAGNOSTIC — target M1->SIFS->M2 + interferer")
    dut._log.info(f"  target rate {RATE} M1={M1_PSDU + 4}B M2={M2_PSDU + 4}B, "
                  f"SNR {SNR_DB} dB")
    dut._log.info(f"  interferer rate {INTF_RATE} {INTF_PSDU + 4}B "
                  f"cfo={INTF_CFO_HZ:.0f}Hz (~{INTF_CFO_HZ / (SAMPLE_RATE / 65536):.0f} LSB)")
    dut._log.info(f"  OTA reference: sig=0x{OTA_ABORT_SIG:06X} "
                  f"phase_inc={OTA_PHASE_INC[0]}..{OTA_PHASE_INC[1]}")
    dut._log.info("=" * 72)

    reproduced = []       # target M2 acquired (len 159) — the class-C shape
    intf_acquired = []    # interferer acquired (len 104) — the class-A shape

    for intf_cfo in INTF_CFOS:
        cfo_tag = "same" if intf_cfo == 0.0 else "off"
        for amp_db in INTF_AMP_DB:
            for delta in DELTAS:
                name = f"cfo{cfo_tag}_delta{delta:+d}_amp{amp_db:+.0f}dB"
                if FILTER and FILTER not in name:
                    continue
                await reset_dut(dut)

                stream, m2_start, p = _compose(delta, amp_db, intf_cfo)
                samples = quantize_12bit(stream)
                r = await feed_and_observe(dut, samples)

                target_ltf1 = m2_start + 192
                intf_ltf1 = p + 192
                m1_ltf1 = PRE_SAMPLES + 192

                def _which(lt):
                    cands = [("M1", m1_ltf1), ("TARGET M2", target_ltf1),
                             ("INTERFERER", intf_ltf1)]
                    return min(cands, key=lambda c: abs(lt - c[1]))[0]

                tag_str = ", ".join(
                    f"{RATE_NAME.get(t['rate'], hex(t['rate']))}/{t['length']}B/"
                    f"{'OK' if t['fcs_ok'] else 'FCS!'}/pi={t['tag_phase']:+d}/"
                    f"{_which(t['tag_ltf'])}@ltf{t['tag_ltf']}"
                    for t in r['tags']) or "(none)"

                sig_aborts = [a for a in r['aborts'] if a['delta']['sig']]
                abort_str = ""
                flag = ""
                for a in sig_aborts:
                    d = _decode_sig_bits(a['sig_raw'])
                    who = _which(a['ctx_ltf'])
                    abort_str += (f" [sig=0x{a['sig_raw']:06X} "
                                  f"rate=0b{d['rate']:04b} len={d['length']} "
                                  f"par={d['parity_ok']} pi={a['ctx_phase_inc']:+d} "
                                  f"ltf={a['ctx_ltf']}]")
                    flag = f"  <-- acquired={who}"
                    if who == "TARGET M2":
                        reproduced.append((name, a['sig_raw'], a['ctx_phase_inc'], a['ctx_ltf']))
                    else:
                        intf_acquired.append((name, a['sig_raw'], a['ctx_phase_inc'], a['ctx_ltf']))

                counts = {k: sum(a['delta'][k] for a in r['aborts'])
                          for k in ('sig', 'rate', 'ow', 'wd')}
                dut._log.info("-" * 72)
                dut._log.info(f"[{name}] intf_start={p} (m2_start={m2_start}) "
                              f"target_ltf1={target_ltf1} intf_ltf1={intf_ltf1}")
                dut._log.info(f"  tags: {tag_str}")
                dut._log.info(f"  aborts sig/rate/ow/wd = "
                              f"{counts['sig']}/{counts['rate']}/{counts['ow']}/{counts['wd']}"
                              f"{abort_str}{flag}")

    dut._log.info("=" * 72)
    if reproduced:
        dut._log.info(f"TARGET M2 acquired+aborted in {len(reproduced)} combo(s):")
        for name, sig, pi, lt in reproduced:
            dut._log.info(f"  {name}: sig=0x{sig:06X} ctx_pi={pi:+d} ltf={lt}")
    else:
        dut._log.info("No target-M2-acquired abort in the swept combos.")
    if intf_acquired:
        dut._log.info(f"INTERFERER acquired+aborted in {len(intf_acquired)} combo(s):")
        for name, sig, pi, lt in intf_acquired:
            dut._log.info(f"  {name}: sig=0x{sig:06X} ctx_pi={pi:+d} ltf={lt}")
    else:
        dut._log.info("No interferer-acquired abort in the swept combos.")
    dut._log.info("(Diagnostic complete — no assertions)")
