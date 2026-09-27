#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""ddr_capture_to_stimulus.py — Convert a deimos_ota_capture .bin window to the
sim replay JSON shape consumed by fpga/test/frontend_helpers.load_adc_capture.

ONE-OFF diagnostic helper (M2-loss investigation). Delete with the capture tool.

Input:
  <prefix>.bin   raw packed DDR words (window_samples x uint32, little-endian)
  <prefix>.json  sidecar written by deimos_ota_capture (geometry + tag log)

Output (default <prefix>_stimulus.json):
  {"real":[...],"imag":[...], ...metadata..., "fabric_tags":[...]}

The floats are normalized exactly like deimos_adc_capture (raw_12bit / 2047);
load_adc_capture rescales to 80% dynamic range, matching the other captures.

Usage:
  scripts/host/ddr_capture_to_stimulus.py /tmp/ddr_win.bin
  scripts/host/ddr_capture_to_stimulus.py /tmp/ddr_win.bin -o /tmp/win.json
"""

import argparse
import json
import os
import sys

import numpy as np


def sign_extend_12(v):
    """Signed 12-bit -> int16 (values are 0..4095)."""
    v = v.astype(np.int32)
    return np.where(v & 0x800, v - 0x1000, v).astype(np.int16)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bin", help="path to <prefix>.bin from deimos_ota_capture")
    ap.add_argument("-s", "--sidecar", help="sidecar JSON (default: <bin>.json)")
    ap.add_argument("-o", "--output", help="output JSON (default: <prefix>_stimulus.json)")
    ap.add_argument("--scale", type=float, default=2047.0,
                    help="normalization divisor (default: 2047.0)")
    args = ap.parse_args()

    sidecar = args.sidecar or os.path.splitext(args.bin)[0] + ".json"
    if not os.path.exists(sidecar):
        sys.exit(f"ERROR: sidecar not found: {sidecar}")
    with open(sidecar) as f:
        meta = json.load(f)

    words = np.fromfile(args.bin, dtype="<u4")
    n = int(meta.get("window_samples", len(words)))
    if n <= 0:
        sys.exit("ERROR: sidecar has no window_samples")
    if len(words) < n:
        sys.exit(f"ERROR: {args.bin} has {len(words)} words, sidecar wants {n}")
    words = words[:n]

    real = sign_extend_12(words & 0xFFF).astype(np.float64) / args.scale
    imag = sign_extend_12((words >> 12) & 0xFFF).astype(np.float64) / args.scale

    out = {
        "description": "OTA DDR window capture (deimos_ota_capture)",
        "source": os.path.basename(args.bin),
        "channel": meta.get("channel"),
        "sample_rate_hz": meta.get("sample_rate_hz"),
        "window_samples": n,
        "window_start": meta.get("window_start"),
        "pre_samples": meta.get("pre_samples"),
        "trigger_len": meta.get("trigger_len"),
        "absent_len": meta.get("absent_len"),
        "trigger_present": meta.get("trigger_present"),
        "absent_present": meta.get("absent_present"),
        "selected": meta.get("selected"),
        "scale": args.scale,
        "n_frames": len(meta.get("tags", [])),
        "fabric_tags": meta.get("tags", []),
        "real": [round(float(x), 6) for x in real],
        "imag": [round(float(x), 6) for x in imag],
    }

    output = args.output or (os.path.splitext(args.bin)[0] + "_stimulus.json")
    with open(output, "w") as f:
        json.dump(out, f)
    print(f"wrote {output} ({n} samples, {len(out['fabric_tags'])} fabric tags)",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
