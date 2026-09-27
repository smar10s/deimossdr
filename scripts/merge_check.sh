#!/bin/bash
# SPDX-License-Identifier: MIT
# scripts/merge_check.sh — verify merge evidence exists for the current build.
#
# Merge gate (AGENTS.md "The Merge Rule", D30): a change that moves the build
# fingerprint must have BOTH gates logged in logs/hardware.jsonl for that
# fingerprint before it merges to main:
#
#   session_end   — sim + HIL + cable loopback   (./scripts/session_end.sh)
#   eapol_toggle  — live OTA EAPOL handshake     (./scripts/eapol_toggle_test.sh)
#
# Evidence is keyed on the fingerprint, not the commit, so docs/tests/firmware
# changes that don't move the fingerprint need neither gate.
#
# Exit 0 if both gates exist for build/fpga/fingerprint, 1 otherwise.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG="$PROJECT_DIR/logs/hardware.jsonl"
FP_FILE="$PROJECT_DIR/build/fpga/fingerprint"

if [ ! -f "$FP_FILE" ]; then
    echo "merge_check: $FP_FILE missing — build first (make bitstream)." >&2
    exit 2
fi
FP="$(cat "$FP_FILE")"

if [ ! -f "$LOG" ]; then
    echo "merge_check: $LOG missing — no hardware evidence for $FP." >&2
    exit 1
fi

if FP="$FP" python3 - "$LOG" <<'PY'
import json, os, sys

log = sys.argv[1]
fp = os.environ["FP"].lower()
if fp.startswith("0x"):
    fp = fp[2:]


def norm(x):
    x = str(x).lower()
    return x[2:] if x.startswith("0x") else x


need = {"session_end": False, "eapol_toggle": False}
for line in open(log):
    line = line.strip()
    if not line:
        continue
    try:
        e = json.loads(line)
    except json.JSONDecodeError:
        continue
    if norm(e.get("fingerprint", "")) != fp:
        continue
    if e.get("event") == "session_end":
        if (e.get("sim") == "PASS" and e.get("hil")
                and e.get("loopback") and not e.get("blocked", False)):
            need["session_end"] = True
    elif e.get("event") == "eapol_toggle":
        if e.get("result") == "PASS":
            need["eapol_toggle"] = True

ok = all(need.values())
for gate in ("session_end", "eapol_toggle"):
    print(f"  [{'OK' if need[gate] else 'MISSING'}] {gate}")
sys.exit(0 if ok else 1)
PY
then
    echo "merge_check: PASS — both gates logged for $FP"
    exit 0
else
    echo ""
    echo "merge_check: FAIL — OTA and/or loopback evidence missing for $FP." >&2
    echo "  loopback: cable connected -> ./scripts/session_end.sh" >&2
    echo "  ota:      antenna connected -> ./scripts/eapol_toggle_test.sh -n 20" >&2
    echo "  See AGENTS.md 'The Merge Rule' and DECISIONS.md D30." >&2
    exit 1
fi
