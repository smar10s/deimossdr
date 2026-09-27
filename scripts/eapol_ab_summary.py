#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# eapol_ab_summary.py — EAPOL 4-way handshake completeness from a
# deimos_rx_dump JSONL capture.
#
# The toggle test's failure mode is losing the STA responses (M2/M4) that
# follow an AP frame at SIFS. This reports, per handshake role, how many of
# the expected frames were captured, so an A/B run has an unambiguous metric
# instead of an eyeballed EAPOL count.
#
# Roles (target BSS): M1 = 137B/AP, M2 = 159B/STA, M3 = 193B/AP, M4 = 137B/STA.
# EAPOL is identified by the LLC/SNAP OUI 88:8e in the PSDU.
#
# AP is inferred from the modal BSSID among rate-6 EAPOL frames. STA is
# inferred from the modal SA among 159-byte frames (M2), unless passed in.
#
# Usage:
#   scripts/eapol_ab_summary.py <capture.jsonl> [expected_per_role] [sta_mac]
#   scripts/eapol_ab_summary.py <capture.jsonl> [expected_per_role] [sta_mac] --json
#
# --json emits one JSON object (per-role counts + rates) on stdout instead of
# the human report, for gating/logging by eapol_toggle_test.sh.
#
# Exit status is always 0 (report only).

import json
import sys
from collections import Counter

EAPOL_SNAP = b"\xaa\xaa\x03\x00\x00\x00\x88\x8e"
ROLES = [("M1", 137, "AP"), ("M2", 159, "STA"),
         ("M3", 193, "AP"), ("M4", 137, "STA")]


def is_eapol(f):
    if f.get("is_eapol"):
        return True
    psdu = f.get("psdu", "")
    if len(psdu) < 32:
        return False
    try:
        raw = bytes.fromhex(psdu[:200])
    except ValueError:
        return False
    return EAPOL_SNAP in raw


def load(path):
    out = []
    with open(path, errors="ignore") as fh:
        for line in fh:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def main():
    as_json = "--json" in sys.argv[1:]
    args = [a for a in sys.argv[1:] if a != "--json"]
    if not args:
        print(__doc__)
        return 0
    path = args[0]
    expected = int(args[1]) if len(args) > 1 else 0
    sta_arg = args[2].lower() if len(args) > 2 else None

    frames = load(path)
    eap = [f for f in frames if f.get("rate") == 6 and is_eapol(f)]

    bssids = Counter(f["bssid"] for f in eap if f.get("bssid"))
    ap = bssids.most_common(1)[0][0] if bssids else None
    sta = sta_arg
    if sta is None:
        m2 = Counter(f["sa"] for f in eap
                     if f.get("len") == 159 and f.get("sa"))
        sta = m2.most_common(1)[0][0] if m2 else None

    roles = {}
    for name, ln, side in ROLES:
        mac = ap if side == "AP" else sta
        c = sum(1 for f in eap if f.get("len") == ln and f.get("sa") == mac)
        roles[name] = {
            "len": ln, "side": side, "count": c,
            "rate": (c * 100 // expected) if expected > 0 else None,
        }

    if as_json:
        sta_rates = [roles[n]["rate"] for n in ("M2", "M4")
                     if roles[n]["rate"] is not None]
        print(json.dumps({
            "path": path,
            "frames": len(frames),
            "rate6_eapol": len(eap),
            "ap": ap,
            "sta": sta,
            "expected": expected,
            "roles": roles,
            "sta_roles": {n: roles[n]["rate"] for n in ("M2", "M4")},
            "sta_min_rate": min(sta_rates) if sta_rates else None,
        }))
        return 0

    print(f"{path}")
    print(f"  frames={len(frames)}  rate6_EAPOL={len(eap)}  "
          f"AP={ap}  STA={sta}  expected={expected}/role")
    if expected <= 0:
        print("  (pass expected_per_role for a loss breakdown)")
        return 0

    total = 0
    for name, ln, side in ROLES:
        c = roles[name]["count"]
        loss = max(0, expected - c)
        total += loss
        print(f"    {name} {ln:>3}B/{side:<3}: {c:3d}/{expected}  "
              f"loss={loss:2d} ({loss * 100 // expected:3d}%)")
    print(f"  TOTAL missing = {total} / {4 * expected}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
