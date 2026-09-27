#!/usr/bin/env python3
"""Generate impaired burst vectors for HIL layer 7.

Produces JSON files compatible with deimos_burst_loopback --file --hil.
Uses lib80211 frame generation + impairment models.

Scenarios model real-world degradations that the pipeline must handle
for OTA decode. Each produces a multi-frame burst (6 frames, mixed rates,
DIFS-spaced) with specified impairments applied.

Usage:
    scripts/gen_impaired_burst.py --output /tmp/hil_impaired/
    scripts/gen_impaired_burst.py --scenario combined --output /tmp/

Scenarios:
    sfo_only            — 10 ppm SFO (typical crystal offset)
    multipath_mild      — 2-tap (LOS + 1 reflection at -10 dB, 3 samples)
    multipath_mod       — 3-tap moderate multipath
    awgn_25db           — 25 dB SNR (realistic indoor 5 GHz)
    combined            — CFO + SFO + multipath + AWGN (realistic OTA)
    multipath_mod_54m   — same as multipath_mod, all frames at 54 Mbps
    combined_54m        — same as combined, all frames at 54 Mbps
"""

import argparse
import json
import os
import sys

import numpy as np

# Add lib80211 to path
LIB80211_PYTHON = os.path.join(os.path.dirname(__file__), '..', 'extern', 'lib80211', 'python')
sys.path.insert(0, os.path.abspath(LIB80211_PYTHON))

from py80211.gen_ofdm_frame import generate_frame, RATE_TABLE
from py80211.impairments import add_cfo, add_sfo, add_awgn, apply_multipath, MULTIPATH_PRESETS


# ============================================================================
# Burst builder
# ============================================================================

DIFS_SAMPLES = 680      # 34 μs at 20 MSPS
PRE_PAD = 2000          # 100 μs silence before burst
POST_PAD = 2000         # 100 μs silence after burst

# Frame specs: (rate_mbps, payload_bytes)
FRAME_SPECS = [
    (6, 137),    # EAPOL M1 size
    (24, 14),    # ACK
    (6, 193),    # EAPOL M3 size
    (12, 100),   # misc management
    (24, 14),    # ACK
    (6, 371),    # beacon-sized
]

# 54M-only variant: same length mix, all at 54 Mbps (64-QAM 3/4) so the
# scenario exercises the highest-rate decode path under impairment.
# The standard layer-8 specs never touch 48/54 — this closes that gap.
FRAME_SPECS_54M = [
    (54, 137),
    (54, 14),
    (54, 193),
    (54, 100),
    (54, 14),
    (54, 371),
]

# Rate code mapping: rate_mbps -> 4-bit OFDM rate code
RATE_CODES = {r: info["rate_bits"] for r, info in RATE_TABLE.items()}


def build_burst(specs, gap_samples=DIFS_SAMPLES, seed=42):
    """Generate a multi-frame burst at baseband (complex IQ).

    Returns:
        (burst_iq, expected_frames) where expected_frames is a list of
        {rate_code, length, psdu_hex} dicts for --verify-psdu support.
    """
    rng = np.random.default_rng(seed)
    frames = []
    expected_frames = []
    for rate, payload_len in specs:
        payload = rng.integers(0, 256, size=payload_len, dtype=np.uint8).tobytes()
        iq, meta = generate_frame(rate, payload, scrambler_seed=rng.integers(1, 127))
        frames.append(iq)

        # Record expected frame info for PSDU verification
        # psdu_payload is the MAC payload (excluding FCS)
        # SIGNAL LENGTH = psdu_length (which includes 4-byte FCS)
        psdu_payload = meta["psdu_payload"]
        expected_frames.append({
            "rate_code": int(meta["rate_bits"]),
            "length": int(meta["psdu_length"]),
            "psdu_hex": psdu_payload.hex(),
        })

    # Concatenate with gaps
    total = PRE_PAD + sum(len(f) for f in frames) + gap_samples * (len(frames) - 1) + POST_PAD
    burst = np.zeros(total, dtype=complex)

    offset = PRE_PAD
    for i, frame_iq in enumerate(frames):
        burst[offset:offset + len(frame_iq)] = frame_iq
        offset += len(frame_iq)
        if i < len(frames) - 1:
            offset += gap_samples

    return burst, expected_frames


# ============================================================================
# Scenario definitions
# ============================================================================

SCENARIOS = {
    "sfo_only": {
        "description": "10 ppm SFO (typical crystal offset between AP and receiver)",
        "specs": FRAME_SPECS,
        "impairments": lambda iq: add_sfo(iq, ppm=10.0),
    },
    "multipath_mild": {
        "description": "2-tap multipath: LOS + reflection at -10 dB, 3 samples delay",
        "specs": FRAME_SPECS,
        "impairments": lambda iq: apply_multipath(iq, [(0, 1.0 + 0j), (3, -0.3 + 0.1j)]),
    },
    "multipath_mod": {
        "description": "3-tap moderate multipath (indoor 5 GHz)",
        "specs": FRAME_SPECS,
        "impairments": lambda iq: apply_multipath(iq, MULTIPATH_PRESETS["moderate"]),
    },
    "awgn_25db": {
        "description": "25 dB SNR AWGN (realistic indoor signal level)",
        "specs": FRAME_SPECS,
        "impairments": lambda iq: add_awgn(iq, snr_db=25.0, seed=77),
    },
    "combined": {
        "description": "CFO 3kHz + SFO 8ppm + mild multipath + 25dB AWGN",
        "specs": FRAME_SPECS,
        "impairments": lambda iq: add_awgn(
            apply_multipath(
                add_sfo(add_cfo(iq, cfo_hz=3000.0), ppm=8.0),
                [(0, 1.0 + 0j), (3, -0.3 + 0.1j)]
            ),
            snr_db=25.0, seed=99
        ),
    },
    "multipath_mod_54m": {
        "description": "3-tap moderate multipath at 54 Mbps (64-QAM 3/4)",
        "specs": FRAME_SPECS_54M,
        "impairments": lambda iq: apply_multipath(iq, MULTIPATH_PRESETS["moderate"]),
    },
    "combined_54m": {
        "description": "CFO 3kHz + SFO 8ppm + mild multipath + 30dB AWGN at 54 Mbps",
        "specs": FRAME_SPECS_54M,
        "impairments": lambda iq: add_awgn(
            apply_multipath(
                add_sfo(add_cfo(iq, cfo_hz=3000.0), ppm=8.0),
                [(0, 1.0 + 0j), (3, -0.3 + 0.1j)]
            ),
            snr_db=30.0, seed=99
        ),
    },
}


# ============================================================================
# Output
# ============================================================================

def quantize_to_json(iq_complex, output_path, description="",
                     expected_frames=None):
    """Quantize complex IQ to 12-bit and write as JSON."""
    re = np.real(iq_complex).astype(np.float64)
    im = np.imag(iq_complex).astype(np.float64)

    # Scale to 90% of 12-bit range
    peak = max(np.max(np.abs(re)), np.max(np.abs(im)))
    if peak > 0:
        scale = (2047.0 * 0.9) / peak
    else:
        scale = 1.0

    re_q = np.clip(np.round(re * scale), -2048, 2047).astype(int)
    im_q = np.clip(np.round(im * scale), -2048, 2047).astype(int)

    data = {
        "description": description,
        "n_samples": len(iq_complex),
        "peak_amplitude": float(peak),
        "scale_factor": float(scale),
        "real": re_q.tolist(),
        "imag": im_q.tolist(),
    }

    if expected_frames is not None:
        data["expected_frames"] = expected_frames

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f)

    size_mb = os.path.getsize(output_path) / 1024 / 1024
    print(f"  {os.path.basename(output_path)}: {len(iq_complex)} samples, "
          f"peak={peak:.3f}, {size_mb:.1f} MB")


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', '-o', default='/tmp/hil_impaired',
                        help='Output directory for JSON vectors')
    parser.add_argument('--scenario', '-s', default=None,
                        help='Single scenario to generate (default: all)')
    parser.add_argument('--list', action='store_true',
                        help='List available scenarios')
    args = parser.parse_args()

    if args.list:
        for name, cfg in SCENARIOS.items():
            print(f"  {name:20s} — {cfg['description']}")
        return

    scenarios = {args.scenario: SCENARIOS[args.scenario]} if args.scenario else SCENARIOS

    print(f"Generating {len(scenarios)} impaired burst vector(s)...")
    for name, cfg in scenarios.items():
        print(f"\n  Scenario: {name}")
        print(f"    {cfg['description']}")
        burst_clean, expected_frames = build_burst(cfg["specs"])
        print(f"    Clean burst: {len(burst_clean)} samples ({len(burst_clean)/20000:.1f} ms), "
              f"{len(cfg['specs'])} frames")
        burst_impaired = cfg['impairments'](burst_clean.copy())
        output_path = os.path.join(args.output, f"{name}.json")
        quantize_to_json(burst_impaired, output_path,
                         description=f"HIL layer 8: {cfg['description']}",
                         expected_frames=expected_frames)

    print(f"\nDone. Vectors in: {args.output}/")


if __name__ == '__main__':
    main()
