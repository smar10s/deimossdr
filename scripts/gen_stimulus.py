#!/usr/bin/env python3
"""Generate deterministic stimulus vectors for deimos regression gates.

Replaces sensitive OTA captures (real AP beacons, real 4-way EAPOL
handshake with key material) with seeded synthetic IQ streams that
reproduce the observable structure of the originals.

  eapol_4way_burst.json (stimulus/passing/)
    - 15 frames: 14 decodable legacy (all 6 Mbps, rate_bits 0b1011) plus
      1 HT/VHT stand-in whose L-SIG is corrupted -> STF triggers, decode
      aborts, NO tag. This matches the observed behavior of the original
      OTA capture (sim replay: 14 tags, 0 FCS-fail).
    - Frame lengths: 137/14/20/14/159/14/371/23/1498/193/14/20/14/137
      (EAPOL M1-M4 = 137/159/193/137; len-23 = D15 HT/VHT CRC-collision
      case; 371 = beacon-sized; 1498 = large data).
    - Gap schedule reproduced from the original capture's STF offsets:
      EAPOL M1->M2 ~7.2 ms, M2->M3 ~42.8 ms, M3->M4 ~1.23 ms (NOT SIFS),
      BUT M2 arrives ~19.5 us after an ACK and M4 ~21.3 us after an ACK,
      and every data frame's ACK arrives within ~21-24 us. The mixed
      structure IS the test: near-SIFS traffic coexists with ms-scale
      EAPOL gaps. DO NOT "normalize" these gaps to uniform SIFS/DIFS.

  eapol_m1.json ... eapol_m4.json (stimulus/passing/)
    - Single-frame EAPOL extracts (6 Mbps, L-SIG lengths 137/159/193/137)
      with fake locally-administered MACs and seeded-random key material.

Regeneration is deterministic: '--verify' reproduces byte-identical files.
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', 'extern',
                               'lib80211', 'python')
sys.path.insert(0, os.path.abspath(LIB80211_PYTHON))

from py80211.gen_ofdm_frame import generate_frame, RATE_TABLE
from py80211.impairments import (
    add_agc_ramp, add_awgn, add_cfo, add_dc_offset, add_phase_noise,
    add_sfo, apply_multipath,
)

SAMPLE_RATE = 20_000_000
PREAMBLE_SAMPLES = 400      # STF 160 + LTF 160 + L-SIG 80 at 20 MSPS
L_SIG_DATA_START = 336      # 320 (STF+LTF) + 16 (GI)
L_SIG_DATA_END = 400
SEED = 1337

# Fake locally-administered MACs (IEEE example range). Never real hardware.
AP_MAC = bytes.fromhex('020000000001')
STA_MAC = bytes.fromhex('020000000002')

RATE6_BITS = RATE_TABLE[6]['rate_bits']  # 0b1011

# Channel-wide impairments (mild OTA-like; must decode 14/14 in sim AND
# pass HIL replay with fcs_pass>=1, fcs_fail=0, psdu_fail=0).
CHANNEL = {
    'multipath': [(0, 1.0 + 0j), (3, -0.25 + 0.1j)],
    'cfo_hz': 800.0,
    'sfo_ppm': 2.0,
    'phase_noise': 0.004,
    'dc_i': 0.01,
    'dc_q': 0.008,
    'snr_db': 30.0,
    'noise_floor_dbfs': -55.0,
    'agc_settle': 128,
    'agc_init_db': -15.0,
}


# ---------------------------------------------------------------------------
# Payload builders (all seeded; no real credentials)
# ---------------------------------------------------------------------------

def build_eapol(da, sa, bssid, seq, payload_len, rng):
    """Structurally EAPOL-like PSDU: 802.11 hdr + LLC/SNAP + 802.1X body.

    payload_len excludes FCS (L-SIG length = payload_len + 4).
    Key material is seeded random bytes.
    """
    hdr = struct.pack('<HH', 0x8802, 0x003c) + da + sa + bssid \
        + struct.pack('<H', seq)
    llc = bytes.fromhex('aaaa03000000888e')
    eapol_hdr = bytes.fromhex('020300')  # v2, Key, length filled next
    body_target = payload_len - len(hdr) - len(llc) - len(eapol_hdr) - 2
    if body_target < 0:
        raise ValueError(f'payload_len {payload_len} too small for EAPOL hdr')
    eapol_len = struct.pack('>H', body_target)
    body = rng.integers(0, 256, size=body_target, dtype=np.uint8).tobytes()
    return hdr + llc + eapol_hdr + eapol_len + body


def build_ack14(ra):
    """ACK: FC(0x00D4) + dur + RA = 10 B payload (+4 FCS = L-SIG 14)."""
    return struct.pack('<HH', 0x00D4, 0) + ra


def build_ba20(ra, ta):
    """BlockAck: FC(0x00B4) + dur + RA + TA = 16 B payload (+4 = 20)."""
    return struct.pack('<HH', 0x00B4, 0x016e) + ra + ta


def build_beacon367(bssid, rng):
    """Beacon-sized (367 B payload = L-SIG 371) with fake SSID."""
    fc = struct.pack('<HH', 0x0080, 0)
    hdr = fc + b'\xff' * 6 + bssid + bssid + struct.pack('<H', 0)
    fixed = struct.pack('<QHH', 0, 100, 0x0431)
    ssid_ie = bytes([0, len('lib80211-stims')]) + b'lib80211-stims'
    rates_ie = bytes([1, 6]) + bytes.fromhex('0c1218243048606c')
    ds_ie = bytes([3, 1, 36])
    tim_ie = bytes([5, 4, 0, 1, 0, 0])
    ies = ssid_ie + rates_ie + ds_ie + tim_ie
    base = hdr + fixed + ies
    pad = rng.integers(0, 256, size=367 - len(base),
                       dtype=np.uint8).tobytes()
    return base + pad


def build_qos_null19(ra, ta, rng):
    """QoS-Null (len-23 case from D15): FC(0x0054) + dur + RA + TA + 3 B."""
    return struct.pack('<HH', 0x0054, 0x00b0) + ra + ta \
        + rng.integers(0, 256, size=3, dtype=np.uint8).tobytes()


def build_data1494(da, sa, bssid, rng):
    """Large data frame (1494 B payload = L-SIG 1498)."""
    hdr = struct.pack('<HH', 0x8802, 0x0060) + da + sa + bssid \
        + struct.pack('<H', 0)
    return hdr + rng.integers(0, 256, size=1494 - len(hdr),
                              dtype=np.uint8).tobytes()


# ---------------------------------------------------------------------------
# Burst definition (structure measured from the original OTA capture)
# ---------------------------------------------------------------------------

# Payloads in decode order: EAPOL M1-M4 = 137/159/193/137, len-23 = D15
# HT/VHT CRC-collision case, 371 = beacon-sized, 1498 = large data.
# STF_DELTAS[i] = samples between STF of frame i and STF of frame i+1,
# measured from the original capture's stf_offsets
# [500, 4981, 142764, 144134, 145298, 150349, 413943, 627835, 630794,
#  1001167, 1007203, 1023232, 1024601, 1025800].
# This preserves the real gap structure WITHOUT interpreting it:
#   EAPOL M1->M2 ~7.2 ms, M2->M3 ~42.8 ms, M3->M4 ~1.23 ms (NOT SIFS),
#   but the ACK chains and M2/M4 ride whatever tight spacing the
#   original channel had. DO NOT "normalize" to uniform SIFS/DIFS.
STF_DELTAS = [4481, 137783, 1370, 1164, 5051, 263594, 213892, 2959,
              370373, 6036, 16029, 1369, 1199]
BURST_LEAD = 500
# The HT/VHT stand-in sits inside the 370373-sample delta between the
# 1498 B frame and M3 (the original's undecoded 15th signal lives in
# that window).
ABORT_GAP_INTO_BIG_GAP = 165000


def build_burst_specs(rng):
    """Return list of (payload, amplitude, cfo_hz, kind)."""
    m1 = build_eapol(STA_MAC, AP_MAC, AP_MAC, 0, 133, rng)
    m2 = build_eapol(AP_MAC, STA_MAC, AP_MAC, 1, 155, rng)
    m3 = build_eapol(STA_MAC, AP_MAC, AP_MAC, 2, 189, rng)
    m4 = build_eapol(AP_MAC, STA_MAC, AP_MAC, 3, 133, rng)
    ack_ap = build_ack14(STA_MAC)   # AP acking STA
    ack_sta = build_ack14(AP_MAC)   # STA acking AP
    ba_ap = build_ba20(STA_MAC, AP_MAC)
    ba_sta = build_ba20(AP_MAC, STA_MAC)
    beacon = build_beacon367(AP_MAC, rng)
    qos23 = build_qos_null19(STA_MAC, AP_MAC, rng)
    data = build_data1494(STA_MAC, AP_MAC, AP_MAC, rng)

    return [
        (m1,      1.0,  800.0,  'eapol_m1'),
        (ack_sta, 1.0,  800.0,  'ack'),
        (ba_ap,   1.0,  800.0,  'blockack'),
        (ack_ap,  0.55, -500.0, 'ack'),
        (m2,      0.55, -500.0, 'eapol_m2'),
        (ack_sta, 1.0,  800.0,  'ack'),
        (beacon,  1.0,  800.0,  'beacon'),
        (qos23,   0.9,  800.0,  'qos_null_23'),
        (data,    0.9,  800.0,  'data_1498'),
        # HT/VHT stand-in inserted mid-gap after data_1498 (see assemble)
        (m3,      1.0,  800.0,  'eapol_m3'),
        (ack_sta, 1.0,  800.0,  'ack'),
        (ba_ap,   1.0,  800.0,  'blockack'),
        (ack_ap,  0.55, -500.0, 'ack'),
        (m4,      0.55, -500.0, 'eapol_m4'),
    ]


# ---------------------------------------------------------------------------
# Assembly (mirrors py80211.channel.generate_stream effect order so the
# abort frame's L-SIG can be corrupted BEFORE multipath smears it)
# ---------------------------------------------------------------------------

def _noise(n, rng, dbfs):
    amp = 10.0 ** (dbfs / 20.0)
    return amp * (rng.standard_normal(n) + 1j * rng.standard_normal(n)) \
        / np.sqrt(2.0)


def assemble_burst(rng):
    """Generate the 15-frame burst.

    Returns (iq, stf_offsets, expected_frames, abort_frames).
    """
    specs = build_burst_specs(rng)
    segments = []
    offsets = []
    expected = []
    abort = []
    current = 0
    prev_dur = 0

    # Abort stand-in: valid STF/LTF, L-SIG zeroed -> invalid rate 0b0000
    # -> decode abort -> NO tag (matches original OTA behavior).
    abort_payload = rng.integers(0, 256, size=52, dtype=np.uint8).tobytes()
    abort_iq, _ = generate_frame(6, abort_payload, int(rng.integers(1, 127)))
    abort_iq = abort_iq.copy()
    abort_iq[L_SIG_DATA_START:L_SIG_DATA_END] = 0.0 + 0.0j
    abort_dur = PREAMBLE_SAMPLES + int(np.ceil(56 * 8 * 20 / 6))

    for idx, (payload, amp, cfo, kind) in enumerate(specs):
        if idx == 0:
            gap = BURST_LEAD
        else:
            gap = STF_DELTAS[idx - 1] - prev_dur
        if kind == 'eapol_m3':
            # M3's delta already consumed the abort stand-in: subtract it
            gap -= ABORT_GAP_INTO_BIG_GAP + abort_dur
        if gap > 0:
            segments.append(_noise(gap, rng, CHANNEL['noise_floor_dbfs'])
                            .astype(np.complex64))
            current += gap
        iq, meta = generate_frame(6, payload, int(rng.integers(1, 127)))
        iq = iq.astype(np.complex64) * amp
        if cfo != 0.0:
            iq = add_cfo(iq, cfo, SAMPLE_RATE)
        if CHANNEL['agc_settle'] > 0:
            iq = add_agc_ramp(iq, settle_samples=CHANNEL['agc_settle'],
                              initial_gain_db=CHANNEL['agc_init_db'])
        offsets.append(current)
        expected.append({
            'rate_code': int(RATE6_BITS),
            'length': int(meta['psdu_length']),
            'psdu_hex': meta['psdu_payload'].hex(),
        })
        segments.append(iq)
        current += len(iq)
        prev_dur = len(iq)
        # place the abort stand-in immediately after the 1498 B frame
        if kind == 'data_1498':
            segments.append(_noise(ABORT_GAP_INTO_BIG_GAP, rng,
                                   CHANNEL['noise_floor_dbfs'])
                            .astype(np.complex64))
            current += ABORT_GAP_INTO_BIG_GAP
            abort.append({'stf_offset': current, 'type': 'ht_vht_lsig_abort',
                          'description': 'HT/VHT stand-in: STF+LTF valid, '
                                         'L-SIG corrupted -> abort, no tag'})
            segments.append(abort_iq)
            current += abort_dur

    stream = np.concatenate(segments).astype(np.complex64)

    # Channel-wide effects, physically correct order (channel.py reference)
    stream = apply_multipath(stream, CHANNEL['multipath'])
    stream = add_cfo(stream, CHANNEL['cfo_hz'], SAMPLE_RATE)
    ratio = 1.0 + CHANNEL['sfo_ppm'] * 1e-6
    stream = add_sfo(stream, CHANNEL['sfo_ppm'], SAMPLE_RATE)
    offsets = [int(o * ratio) for o in offsets]
    abort = [{'stf_offset': int(a['stf_offset'] * ratio),
              'type': a['type'], 'description': a['description']}
             for a in abort]
    stream = add_phase_noise(stream, strength=CHANNEL['phase_noise'],
                             sample_rate=SAMPLE_RATE,
                             seed=int(rng.integers(0, 2 ** 31)))
    stream = add_dc_offset(stream, CHANNEL['dc_i'], CHANNEL['dc_q'])
    stream = add_awgn(stream, CHANNEL['snr_db'],
                      seed=int(rng.integers(0, 2 ** 31)))

    return stream.astype(np.complex64), offsets, expected, abort


# ---------------------------------------------------------------------------
# Single-frame EAPOL extracts
# ---------------------------------------------------------------------------

def assemble_single(payload_len, rng, seq):
    """One EAPOL frame, 500 samples of lead/trail noise.

    Returns (stream, expected_frames, stf_offsets).
    """
    if seq % 2 == 0:
        payload = build_eapol(STA_MAC, AP_MAC, AP_MAC, seq, payload_len, rng)
    else:
        payload = build_eapol(AP_MAC, STA_MAC, AP_MAC, seq, payload_len, rng)
    iq, meta = generate_frame(6, payload, int(rng.integers(1, 127)))
    iq = iq.astype(np.complex64)
    lead = _noise(500, rng, CHANNEL['noise_floor_dbfs']).astype(np.complex64)
    trail = _noise(500, rng, CHANNEL['noise_floor_dbfs']).astype(np.complex64)
    stream = np.concatenate([lead, iq, trail]).astype(np.complex64)
    stream = add_cfo(stream, 300.0, SAMPLE_RATE)
    stream = add_awgn(stream, CHANNEL['snr_db'],
                      seed=int(rng.integers(0, 2 ** 31)))
    return (stream.astype(np.complex64),
            [{'rate_code': int(RATE6_BITS),
              'length': int(meta['psdu_length']),
              'psdu_hex': meta['psdu_payload'].hex()}],
            [500])


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def serialize(iq, description, source, rate_mbps, n_frames,
              stf_offsets, expected_frames, abort_frames=None):
    """Quantize IQ to 12-bit and build the full JSON dict (schema mirrors
    the removed captures: real/imag int arrays, stf_offsets,
    expected_frames, scale_factor, peak_amplitude)."""
    re = np.real(iq).astype(np.float64)
    im = np.imag(iq).astype(np.float64)
    peak = max(np.max(np.abs(re)), np.max(np.abs(im)))
    scale = (2047.0 * 0.9) / peak if peak > 0 else 1.0
    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)

    data = {
        'description': description,
        'source': source,
        'rate_mbps': rate_mbps,
        'n_frames': n_frames,
        'n_samples': len(iq),
        'stf_offsets': [int(o) for o in stf_offsets],
        'scale_factor': float(scale),
        'peak_amplitude': float(peak),
        'real': re_q.tolist(),
        'imag': im_q.tolist(),
        'expected_frames': expected_frames,
    }
    if abort_frames is not None:
        data['abort_frames'] = abort_frames
    return data


def generate_all():
    """Generate all stimulus files as JSON dicts. Returns
    {filename: json_data}."""
    outputs = {}

    rng = np.random.default_rng(SEED)
    iq, offs, expected, abort = assemble_burst(rng)
    outputs['eapol_4way_burst.json'] = serialize(
        iq,
        'Synthetic 4-way EAPOL handshake burst: 14 decodable legacy '
        'frames (6 Mbps) + 1 HT/VHT stand-in (L-SIG abort, no tag). '
        'Gap schedule reproduced from OTA structure: EAPOLs ms-apart, '
        'ACK chains near-SIFS.',
        'synthetic: scripts/gen_stimulus.py seed=%d' % SEED,
        6, 14, offs, expected, abort)

    singles = [
        ('eapol_m1.json', 133, 0,
         'EAPOL M1-shaped frame (6 Mbps, L-SIG 137), fake key material'),
        ('eapol_m2.json', 155, 1,
         'EAPOL M2-shaped frame (6 Mbps, L-SIG 159), fake key material'),
        ('eapol_m3.json', 189, 2,
         'EAPOL M3-shaped frame (6 Mbps, L-SIG 193), fake key material'),
        ('eapol_m4.json', 133, 3,
         'EAPOL M4-shaped frame (6 Mbps, L-SIG 137), fake key material'),
    ]
    for fname, plen, seq, desc in singles:
        iq, expected, offs = assemble_single(plen, rng, seq)
        outputs[fname] = serialize(
            iq, desc, 'synthetic: scripts/gen_stimulus.py seed=%d' % SEED,
            6, 1, offs, expected)

    return outputs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out-dir',
                    default=os.path.join(os.path.dirname(__file__), '..',
                                         'stimulus', 'passing'),
                    help='output directory (default: stimulus/passing)')
    ap.add_argument('--verify', action='store_true',
                    help='regenerate in memory and byte-compare against '
                         'committed files')
    args = ap.parse_args()

    if not args.verify:
        print('Generating stimulus files (seed=%d):' % SEED)
        out_dir = os.path.abspath(args.out_dir)
        os.makedirs(out_dir, exist_ok=True)
        for fname, data in generate_all().items():
            path = os.path.join(out_dir, fname)
            with open(path, 'w') as f:
                json.dump(data, f)
            print(f'  {fname}: {data["n_samples"]} samples, '
                  f'{os.path.getsize(path) / 1e6:.2f} MB')
        return 0

    # --verify: regenerate and compare field-by-field
    fresh = generate_all()
    mismatches = []
    for fname, d in fresh.items():
        path = os.path.join(args.out_dir, fname)
        if not os.path.exists(path):
            mismatches.append(f'{fname}: missing on disk')
            continue
        with open(path) as f:
            on_disk = json.load(f)
        for key in ('real', 'imag', 'stf_offsets', 'expected_frames',
                    'abort_frames', 'n_frames', 'rate_mbps'):
            if on_disk.get(key) != d.get(key):
                mismatches.append(f'{fname}: {key} differs')
    if mismatches:
        print('VERIFY FAILED:')
        for m in mismatches:
            print('  ' + m)
        return 1
    print(f'VERIFY OK: {len(fresh)} files byte-identical')
    return 0


if __name__ == '__main__':
    sys.exit(main())
