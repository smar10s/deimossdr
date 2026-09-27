"""
diag_lsig_constellation.py — DIAGNOSTIC (never asserts).

Replays an OTA DDR window (the failing M2 vector) through rx_frontend and
captures, per frame:
  - the equalizer output for the SIGNAL (L-SIG) symbol (48 subcarriers), and
  - the channel estimate H_inv read per FFT bin.

so the L-SIG constellation and channel estimate of a frame the fabric
mis-decodes (M2) can be compared against frames it decodes correctly.

WHY:
  `diag_ota_window_replay` localized the M2 loss to L-SIG mis-decode
  (tag_sig rate 0b1010/len156 instead of 0b1011/len159). This diag splits the
  mechanism:
    - L-SIG clean (imag ~ 0) but wrong bits -> L-SIG demod/parse bug.
    - L-SIG scattered / |H_inv| phase ramping across bins -> channel
      estimate / LTF timing / FFT-window problem upstream.

Run:
  OTA_WINDOW_JSON=/tmp/trim17500.json ./scripts/sim.sh diag_lsig_constellation
  (optional OTA_SLICE=start:end to slice the stimulus before feeding)
"""

import os
import json

import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import (
    reset_dut, load_adc_capture, s12_to_unsigned, SAMPLE_RATE,
)

DEFAULT_JSON = os.path.join(os.path.dirname(__file__), '..', '..',
                            'captures', 'ota_window_capture.json')


def _s16(v):
    v = int(v) & 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _bpsk_residuals(vals):
    """Residual phase from the nearest BPSK (real-axis) point, wrapped to
    [-90, 90] deg. Returns (cpe_deg, spread_deg)."""
    arr = np.array(vals)
    ph = np.angle(arr)
    ref = np.where(np.cos(ph) >= 0, 0.0, np.pi)
    res = ph - ref
    res = (res + np.pi / 2) % np.pi - np.pi / 2
    return float(np.degrees(np.mean(res))), float(np.degrees(np.std(res)))


@cocotb.test()
async def diag_lsig_constellation(dut):
    path = os.environ.get('OTA_WINDOW_JSON', DEFAULT_JSON)
    if not os.path.exists(path):
        cocotb.log.info(f"{path} not found — capture one first")
        return

    with open(path) as f:
        meta = json.load(f)
    samples = load_adc_capture(path)
    sl = os.environ.get('OTA_SLICE', '')
    if sl:
        a, b = (int(x) for x in sl.split(':'))
        samples = samples[a:b]
    n_samples = len(samples)

    eq = dut.u_rx_pipeline.u_equalizer
    de = dut.u_rx_pipeline.u_decode_engine
    acq = dut.u_rx_pipeline.u_acquisition_ctrl
    corr = dut.u_rx_pipeline.u_ltf_corr
    stf = dut.u_stf_detect
    desc_events = []      # (sample_idx, desc_ltf_pos, phase_inc, anchor_stf_end)
    stf_ends = []         # sample_idx of stf_end pulses
    frame_detects = []    # sample_idx of frame_detect pulses (port)
    corr_metric = {}      # sample_idx -> metric
    stf_diag = {}         # sample_idx -> (threshold_met, detected_latch, window_full, persist_cnt)
    last_stf_end = None

    dut._log.info("=" * 72)
    dut._log.info(f"L-SIG constellation: {path} ({n_samples} samples)")
    dut._log.info("=" * 72)

    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    symbols = []          # {'frame','sym','data':{idx:(re,im)}}
    cur = None
    frame_idx = -1

    sample_idx = 0
    valid_counter = 0
    post_zero_count = 8192
    timeout_cycles = (n_samples + post_zero_count) * 5 + 2_000_000

    for cycle in range(timeout_cycles):
        await RisingEdge(dut.clk)

        try:
            if int(de.symbol_start_out.value) == 1:
                sym = int(de.symbol_idx_out.value)
                if sym == 0:
                    frame_idx += 1
                cur = {'frame': frame_idx, 'sym': sym, 'data': {}}
                symbols.append(cur)
            if int(eq.data_valid.value) == 1 and cur is not None:
                cur['data'][int(eq.data_idx.value)] = (
                    _s16(eq.data_re.value), _s16(eq.data_im.value))
            do = int(stf.diag_out.value)
            stf_diag[sample_idx] = (
                (do >> 31) & 1, (do >> 30) & 1, (do >> 29) & 1, (do >> 24) & 0x1F)
            if int(stf.frame_detect.value) == 1:
                frame_detects.append(sample_idx)
            if int(stf.stf_end.value) == 1:
                stf_ends.append(sample_idx)
                last_stf_end = sample_idx
            if int(corr.metric_valid.value) == 1:
                corr_metric[sample_idx] = int(corr.metric.value)
            if int(acq.desc_valid.value) == 1:
                desc_events.append((
                    sample_idx,
                    int(acq.desc_ltf_pos.value),
                    _s16(acq.desc_phase_inc.value),
                    last_stf_end))
        except (ValueError, AttributeError):
            pass

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

        if sample_idx >= n_samples + post_zero_count:
            break

    dut._log.info("--- per-frame SIGNAL (L-SIG) constellation ---")
    for s in symbols:
        if s['sym'] != 0 or not s['data']:
            continue
        vals = [complex(re, im) for _, (re, im) in sorted(s['data'].items())]
        if len(vals) < 8:
            dut._log.info(f"  frame {s['frame']}: only {len(vals)} subcarriers")
            continue
        arr = np.array(vals)
        cpe, spread = _bpsk_residuals(vals)
        dut._log.info(
            f"  frame {s['frame']}: n={len(vals)} mean|v|={np.abs(arr).mean():6.1f} "
            f"cpe={cpe:+7.1f}deg spread={spread:5.1f}deg")
        dut._log.info(
            "      first 16: " +
            " ".join(f"{int(v.real):+d}{int(v.imag):+d}j" for v in vals[:16]))

    dut._log.info("--- acquisition descriptors (LTF position used) ---")
    for i, (ss, pos, phinc, se) in enumerate(desc_events):
        anchor = "" if se is None else f" anchor_stf_end={se} (pos-anchor={pos - se})"
        dut._log.info(f"  desc {i}: sample_idx={ss} desc_ltf_pos={pos} "
                      f"phase_inc={phinc}{anchor}")

    dut._log.info("--- frame_detect / stf_end timing ---")
    # stf_end should land ~150 samples after frame_detect (STF is 160 samples;
    # detection fires ~60 in, then the 64-sample correlation window slides out).
    # An early stf_end is the signature of a corrupted STF autocorrelation.
    for se in stf_ends[:40]:
        prev = [fd for fd in frame_detects if fd <= se]
        fd = prev[-1] if prev else None
        dut._log.info(f"  stf_end@{se}: last_frame_detect={fd} "
                      f"(stf_end-frame_detect={None if fd is None else se - fd})")

    dut._log.info("--- STF detector diag_out around each stf_end (T=threshold_met,"
                  " D=detected, W=window_full /persist) ---")
    for se in stf_ends[:40]:
        dut._log.info(f"  stf_end@{se}:")
        row = []
        for s in range(se - 40, se + 24):
            if s not in stf_diag:
                continue
            thr, det, win, pc = stf_diag[s]
            row.append(f"{s}:{'T' if thr else '.'}{'D' if det else '.'}"
                       f"{'W' if win else '.'}/{pc:02d}")
        for j in range(0, len(row), 8):
            dut._log.info("      " + " ".join(row[j:j + 8]))

    dut._log.info("--- LTF correlator peak vs stf_end window ---")
    # The peak search window is [stf_end+2, stf_end+31] and T1 = argmax - 19.
    for i, se in enumerate(stf_ends[:40]):
        w = [(s, corr_metric[s]) for s in range(se, se + 34) if s in corr_metric]
        if not w:
            continue
        # global max in a wider region, to see if the true peak is outside
        wide = [(s, corr_metric[s]) for s in range(se - 4, se + 80) if s in corr_metric]
        wmax = max(w, key=lambda kv: kv[1])
        gmax = max(wide, key=lambda kv: kv[1])
        inside = (wmax == gmax)
        dut._log.info(
            f"  stf_end@{se}: win_max@{wmax[0]}={wmax[1]} T1={wmax[0]-19} "
            f"wide_max@{gmax[0]}={gmax[1]} "
            f"{'IN' if inside else 'OUT-of-window'}")

    # Raw metric profile around each stf_end: shows whether the correlator peak
    # is a sharp single-sample max or a plateau (relevant if stf_end ever jitters).
    dut._log.info("--- LTF correlator metric profile (se-8 .. se+96) ---")
    lo = int(os.environ.get('OTA_PROFILE_LO', '-8'))
    hi = int(os.environ.get('OTA_PROFILE_HI', '96'))
    for se in stf_ends[:40]:
        dut._log.info(f"  stf_end@{se} (window=[{se+2},{se+31}]):")
        row = []
        for s in range(se + lo, se + hi + 1):
            if s not in corr_metric:
                continue
            inwin = se + 2 <= s <= se + 31
            row.append(f"{s}{'*' if inwin else ' '}:{corr_metric[s]}")
        for j in range(0, len(row), 6):
            dut._log.info("      " + "  ".join(row[j:j + 6]))

    dut._log.info("=" * 72)
