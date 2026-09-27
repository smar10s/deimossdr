"""
diag_sifs_cfo_step.py — Diagnostic: per-frame CFO step across a SIFS pair.

WHY THIS EXISTS:
  OTA EAPOL captures lose ~5% of frames per handshake. The lost frames are the
  STA responses (M2/M4) that follow an AP frame at SIFS. They abort in
  decode_engine.v at S_PARSE_SIGNAL: the L-SIG length field decodes correctly
  while the RATE nibble and parity are corrupted, reproducibly (0x0013E1 twice),
  with latched frame_phase_inc ~251-263 LSB. Every "missing" frame still has its
  peer ACK, so it was received fine over the air (see D28 and
  docs/acquisition-window-fix.md).

WORKING HYPOTHESIS (falsified — see RESULT):
  The second frame of a SIFS pair comes from a different transmitter with a
  different crystal, so its CFO differs from the first frame's. Acquisition
  re-estimates per frame, but something in that re-estimate path corrupts the
  first OFDM symbol (SIGNAL) when the CFO step is large.

  Existing tests cover the pieces separately:
    - test_cfo_resilience: residual CFO on a SINGLE frame, small phase_inc.
    - diag_power_ratio: SIFS pairs with amplitude difference, zero CFO.
    - test_system_traffic: SIFS pairs with per-frame CFO, but only ~1 kHz.
  None drives a LARGE per-frame CFO step across SIFS.

RESULT (2026-09-20): the hypothesis does NOT hold. Full sweep — cfo_sta
  20–100 kHz, STA amp −10 dB, SNR 20 dB, 2/3-tap multipath, SFO, phase noise —
  produces 0 SIG aborts in every combo; M2 decodes with its correct coarse
  estimate each time (phase_inc up to +330). A clean per-transmitter CFO step
  is not sufficient to reproduce the OTA L-SIG corruption. The surviving
  mechanism must involve time-overlapping co-channel energy during the target's
  STF/acquisition, which this generator does not model (frames are placed
  back-to-back, never summed).

WHAT THIS MEASURES:
  A two-frame M1(AP, 137B) → SIFS → M2(STA, 159B) stream at rate 6, with an
  independent CFO and amplitude per frame. After each feed it reads the RTL's
  own diagnostic snapshot (diag_abort_cnts / diag_abort_sig / diag_abort_ctx)
  and reports the abort reason and the raw 24-bit L-SIG bits for frame 2. If the
  L-SIG snapshot matches the OTA evidence (rate nibble corrupted, length intact),
  the hardware failure is reproduced in sim.

NOT A GATE TEST:
  Diagnostic (diag_*). Reports numbers, never asserts. Run via:
    ./scripts/sim.sh diag_sifs_cfo_step
"""

import os
import sys

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from frontend_helpers import quantize_12bit, reset_dut, s12_to_unsigned, SAMPLE_RATE

# Add lib80211 for the composable channel model
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, LIB80211_PYTHON)
from py80211.channel import ChannelConfig, FrameSpec, generate_stream, SIFS_SAMPLES, DIFS_SAMPLES

# OTA reference: M2 is 159B at rate 6, L-SIG snapshot 0x0013E1.
OTA_ABORT_SIG = 0x0013E1
OTA_PHASE_INC = (251, 263)

# Two-frame EAPOL pair: M1 (AP) then M2 (STA), both rate 6, SIFS-spaced.
M1_PSDU = 133   # 137B incl FCS
M2_PSDU = 155   # 159B incl FCS
RATE = 6
SNR_DB = 35.0

MILD_MP = [(0, 1.0 + 0j), (3, -0.3 + 0.1j)]
MOD_MP = [(0, 1.0 + 0j), (5, -0.4 + 0.3j), (12, 0.2 - 0.1j)]

# Optional name-substring filter for quick focused runs:
#   DIAG_COMBO_FILTER=sta_cfo ./scripts/sim.sh diag_sifs_cfo_step
COMBO_FILTER = os.environ.get('DIAG_COMBO_FILTER', '')


def _combo(name, cfo_ap=0, cfo_sta=0, ap_amp=1.0, sta_amp=1.0,
           gap=SIFS_SAMPLES, snr=SNR_DB, sfo=0.0, mp=None, pn=0.0):
    return {
        'name': name, 'cfo_ap': cfo_ap, 'cfo_sta': cfo_sta,
        'ap_amp': ap_amp, 'sta_amp': sta_amp, 'gap': gap,
        'snr': snr, 'sfo': sfo, 'mp': mp, 'pn': pn,
    }


# Clean per-frame CFO/amplitude sweep, then realistic combined impairments.
COMBOS = [
    _combo("baseline_clean"),
    _combo("amp_only_10db", sta_amp=0.316),
    _combo("sta_cfo_78k", cfo_sta=78000),
    _combo("ap_cfo_78k", cfo_ap=78000),
    _combo("same_cfo_78k", cfo_ap=78000, cfo_sta=78000),
    _combo("step_neg78k_pos78k", cfo_ap=-78000, cfo_sta=78000),
    _combo("step_pos78k_neg78k", cfo_ap=78000, cfo_sta=-78000),
    _combo("step_78k_amp10", cfo_sta=78000, sta_amp=0.316),
    _combo("step_77k_amp10", cfo_sta=77000, sta_amp=0.316),
    _combo("step_40k", cfo_sta=40000),
    _combo("step_20k", cfo_sta=20000),
    _combo("sta_cfo_100k", cfo_sta=100000),
    _combo("sta_cfo_78k_difs", cfo_sta=78000, gap=DIFS_SAMPLES),
    # Combined impairments approaching OTA conditions.
    _combo("mp_mild", cfo_sta=78000, sta_amp=0.316, snr=30, mp=MILD_MP),
    _combo("mp_mod", cfo_sta=78000, sta_amp=0.316, snr=25, mp=MOD_MP),
    _combo("sfo_10ppm", cfo_sta=78000, sta_amp=0.316, snr=30, sfo=10.0),
    _combo("phase_noise", cfo_sta=78000, sta_amp=0.316, snr=30, pn=0.02),
    _combo("snr_20", cfo_sta=78000, sta_amp=0.316, snr=20),
    _combo("ota_combined", cfo_sta=78000, sta_amp=0.316, snr=25,
           sfo=5.0, mp=MILD_MP, pn=0.01),
    _combo("ota_combined_mod", cfo_sta=78000, sta_amp=0.316, snr=22,
           sfo=8.0, mp=MOD_MP, pn=0.015),
]


def _sign16(v):
    return v - 65536 if v >= 32768 else v


def _unpack_abort_cnts(v):
    return {
        'sig':  v & 0xFF,
        'rate': (v >> 8) & 0xFF,
        'ow':   (v >> 16) & 0xFF,
        'wd':   (v >> 24) & 0xFF,
    }


def _decode_sig_bits(v):
    """Decode a 24-bit L-SIG snapshot the way decode_engine.v does."""
    rate_nibble = v & 0x0F
    reserved = (v >> 4) & 0x1
    length = (v >> 5) & 0xFFF
    tail = (v >> 18) & 0x3F
    parity_ok = (bin(v & 0x3FFFF).count('1') % 2) == 0
    return {
        'rate': rate_nibble,
        'reserved': reserved,
        'length': length,
        'tail': tail,
        'parity_ok': parity_ok,
    }


RATE_NAME = {0b1011: '6M', 0b1111: '9M', 0b1010: '12M', 0b1110: '18M',
             0b1001: '24M', 0b1101: '36M', 0b1000: '48M', 0b1100: '54M'}


async def feed_and_observe(dut, samples):
    """Feed one stream at 1-per-5 clocks; collect tags, CFO estimates, aborts.

    Reads the RTL diagnostic counters every cycle and records an event whenever
    any abort counter increments, capturing the matching L-SIG snapshot and
    latched phase_inc. This mirrors the on-device -a diagnostic output.
    """
    n_samples = len(samples)
    post_zeros = 8192
    total_feed = n_samples + post_zeros

    tags = []
    cfo_events = []
    aborts = []
    descs = []

    prev_fd = 0
    prev_cfo_done = 0
    prev_cnts = _unpack_abort_cnts(int(dut.diag_abort_cnts.value))
    sample_idx = 0
    valid_counter = 0

    for cycle in range(total_feed * 5 + 100000):
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

        # frame_detect rising edge (per-frame trigger)
        try:
            fd = int(dut.frame_detect.value)
            if fd == 1 and prev_fd == 0:
                cfo_events.append({'kind': 'frame_detect', 'sample': sample_idx})
            prev_fd = fd
        except (ValueError, AttributeError):
            pass

        # cfo_done pulse: record the per-frame coarse estimate
        try:
            cd = int(dut.cfo_done.value)
            if cd == 1 and prev_cfo_done == 0:
                cfo_events.append({
                    'kind': 'cfo_done',
                    'sample': sample_idx,
                    'phase_inc': _sign16(int(dut.phase_inc.value)),
                })
            prev_cfo_done = cd
        except (ValueError, AttributeError):
            pass

        # acquisition descriptor: T1 position (ltf_pos) + latched CFO
        try:
            if int(dut.u_rx_pipeline.u_acquisition_ctrl.desc_valid.value) == 1:
                descs.append({
                    'sample': sample_idx,
                    'ltf_pos': int(dut.u_rx_pipeline.u_acquisition_ctrl.desc_ltf_pos.value),
                    'phase_inc': _sign16(
                        int(dut.u_rx_pipeline.u_acquisition_ctrl.desc_phase_inc.value)),
                    'max_metric': int(dut.u_rx_pipeline.u_acquisition_ctrl.max_metric.value),
                })
        except (ValueError, AttributeError):
            pass

        # tag output
        try:
            if int(dut.tag_valid.value) == 1:
                tags.append({
                    'rate': int(dut.tag_rate.value),
                    'length': int(dut.tag_length.value),
                    'fcs_ok': int(dut.tag_fcs_ok.value),
                    'tag_sig': int(dut.diag_tag_sig.value) & 0xFFFFFF,
                    'tag_phase': _sign16(int(dut.diag_tag_ctx.value) & 0xFFFF),
                })
        except (ValueError, AttributeError):
            pass

        # abort counter increments
        try:
            cnts = _unpack_abort_cnts(int(dut.diag_abort_cnts.value))
            if cnts != prev_cnts:
                delta = {k: (cnts[k] - prev_cnts[k]) & 0xFF for k in cnts}
                aborts.append({
                    'sample': sample_idx,
                    'delta': delta,
                    'sig_raw': int(dut.diag_abort_sig.value),
                    'ctx_phase_inc': _sign16(int(dut.diag_abort_ctx.value) & 0xFFFF),
                })
                prev_cnts = cnts
        except (ValueError, AttributeError):
            pass

        if sample_idx >= total_feed:
            await ClockCycles(dut.clk, 200)
            break

    return {'tags': tags, 'cfo_events': cfo_events, 'aborts': aborts, 'descs': descs}


@cocotb.test()
async def diag_sifs_cfo_step(dut):
    """Sweep per-frame CFO step / amplitude across a SIFS EAPOL pair."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    dut._log.info("=" * 68)
    dut._log.info("SIFS CFO-STEP DIAGNOSTIC — M1(AP) -> SIFS -> M2(STA), rate 6")
    dut._log.info(f"  M1 {M1_PSDU + 4}B, M2 {M2_PSDU + 4}B, SNR {SNR_DB} dB")
    dut._log.info(f"  OTA abort reference: sig=0x{OTA_ABORT_SIG:06X}, "
                  f"phase_inc={OTA_PHASE_INC[0]}..{OTA_PHASE_INC[1]}")
    dut._log.info("=" * 68)

    for c in COMBOS:
        name = c['name']
        if COMBO_FILTER and COMBO_FILTER not in name:
            continue
        cfo_ap, cfo_sta = c['cfo_ap'], c['cfo_sta']
        ap_amp, sta_amp, gap = c['ap_amp'], c['sta_amp'], c['gap']
        await reset_dut(dut)

        config = ChannelConfig(
            snr_db=c['snr'],
            cfo_hz=0.0,
            sfo_ppm=c['sfo'],
            multipath_taps=c['mp'] if c['mp'] else [(0, 1.0 + 0j)],
            phase_noise_strength=c['pn'],
        )

        frames = [
            FrameSpec(rate_mbps=RATE, psdu_len=M1_PSDU, amplitude=ap_amp,
                      cfo_hz=float(cfo_ap), gap_samples=500),
            FrameSpec(rate_mbps=RATE, psdu_len=M2_PSDU, amplitude=sta_amp,
                      cfo_hz=float(cfo_sta), gap_samples=int(gap)),
        ]
        seed = abs(hash(name)) % 100000
        stream, meta = generate_stream(frames, config, seed=seed)
        samples = quantize_12bit(stream)

        r = await feed_and_observe(dut, samples)

        # Final hardware-level counters for this combo
        final_cnts = _unpack_abort_cnts(int(dut.diag_abort_cnts.value))
        final_sig = int(dut.diag_abort_sig.value)
        final_ctx = _sign16(int(dut.diag_abort_ctx.value) & 0xFFFF)

        cfo_str = " ".join(
            f"pi={e['phase_inc']:+d}" for e in r['cfo_events'] if e['kind'] == 'cfo_done')
        tag_str = ", ".join(
            f"{RATE_NAME.get(t['rate'], hex(t['rate']))}/{t['length']}B/"
            f"{'OK' if t['fcs_ok'] else 'FCS!'}"
            f"/sig=0x{t['tag_sig']:06X}/pi={t['tag_phase']:+d}"
            for t in r['tags']) or "(none)"

        imp = []
        if c['snr'] != SNR_DB:
            imp.append(f"snr={c['snr']}")
        if c['sfo']:
            imp.append(f"sfo={c['sfo']}ppm")
        if c['mp']:
            imp.append(f"mp={len(c['mp'])}tap")
        if c['pn']:
            imp.append(f"pn={c['pn']}")
        imp_str = (" [" + ",".join(imp) + "]") if imp else ""

        dut._log.info("-" * 68)
        dut._log.info(f"[{name}]{imp_str} AP_cfo={cfo_ap:+d} STA_cfo={cfo_sta:+d} "
                      f"ap_amp={ap_amp} sta_amp={sta_amp} gap={gap}")
        dut._log.info(f"  detected: {len(r['tags'])}  tags: {tag_str}")
        dut._log.info(f"  cfo_done: {cfo_str or '(none)'}")
        dut._log.info(f"  abort counts (sig/rate/ow/wd): "
                      f"{final_cnts['sig']}/{final_cnts['rate']}/"
                      f"{final_cnts['ow']}/{final_cnts['wd']}")

        if final_cnts['sig'] > 0:
            d = _decode_sig_bits(final_sig)
            match = "  <== MATCHES OTA" if final_sig == OTA_ABORT_SIG else ""
            dut._log.info(
                f"  last SIG snapshot: 0x{final_sig:06X} "
                f"rate=0b{d['rate']:04b}({RATE_NAME.get(d['rate'], '?')}) "
                f"len={d['length']} parity={'ok' if d['parity_ok'] else 'BAD'} "
                f"tail={d['tail']} phase_inc={final_ctx}{match}")

        # Per-abort trace (first few)
        for a in r['aborts'][:4]:
            d = _decode_sig_bits(a['sig_raw'])
            dut._log.info(
                f"    abort @sample~{a['sample']} dSig={a['delta']['sig']} "
                f"dRate={a['delta']['rate']} dOw={a['delta']['ow']} "
                f"dWd={a['delta']['wd']} sig=0x{a['sig_raw']:06X} "
                f"(rate=0b{d['rate']:04b} len={d['length']} "
                f"parity={'ok' if d['parity_ok'] else 'BAD'}) "
                f"pi={a['ctx_phase_inc']:+d}")

        # Acquisition descriptors: T1 position bias vs the true STF offset.
        for i, d in enumerate(r['descs']):
            exp = meta[i]['stf_offset'] if i < len(meta) else -1
            dut._log.info(
                f"    acq[{i}] ltf_pos={d['ltf_pos']} "
                f"(ltf_pos-stf={d['ltf_pos'] - exp:+d}) "
                f"phase_inc={d['phase_inc']:+d} max_metric={d['max_metric']}")

    dut._log.info("=" * 68)
    dut._log.info("(Diagnostic complete — no assertions)")
