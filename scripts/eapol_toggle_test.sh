#!/bin/bash
# scripts/eapol_toggle_test.sh — Automated EAPOL capture via wifi toggle
#
# Toggles Mac wifi off/on to trigger 4-way EAPOL handshake, while
# deimos_rx_dump streams fabric-decoded frames as JSONL over SSH.
#
# Prerequisites:
#   - Pluto connected and reachable at PLUTO_IP (default 192.168.2.1)
#   - Firmware deployed (make deploy)
#   - Antenna connected (NOT cable loopback)
#   - Mac wifi configured to auto-join a known network on the target channel
#
# Usage:
#   ./scripts/eapol_toggle_test.sh                 # ch36, 15 toggles, STA gate 80%
#   ./scripts/eapol_toggle_test.sh -c 149 -n 20    # ch149, 20 toggles
#   ./scripts/eapol_toggle_test.sh -t 2            # 4x sensitive STF threshold
#   ./scripts/eapol_toggle_test.sh -g 60           # gate each STA role at 60%
#
# Output:
#   - Per-frame JSONL to stdout (for piping/analysis)
#   - Summary to stderr (EAPOL count, class breakdown, FCS stats)
#   - One JSON entry appended to logs/hardware.jsonl (event "eapol_toggle")
#
# Gate: each STA role (M2/M4) must reach >= -g% of N_TOGGLES. Exits nonzero
# and logs blocked:true on failure. AP roles are reported, not gated (today's
# OTA loss is AP-side; the STA gate is the regression metric).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/lib.sh"

WIFI_IFACE="en0"

# Defaults
CHANNEL=36
N_TOGGLES=15
STF_THRESH=0
STA_GATE_PCT=80
POST_TOGGLE_SEC=5
INTER_TOGGLE_SEC=2
ACQ_DIAG=""

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "  -c channel     WiFi channel (default: $CHANNEL)"
    echo "  -n toggles     Number of wifi off/on cycles (default: $N_TOGGLES)"
    echo "  -t threshold   STF sensitivity 0-7 (default: $STF_THRESH)"
    echo "  -g pct         Per-role STA capture gate, 0-100 (default: $STA_GATE_PCT)"
    echo "  -a             Acquisition diag (-a): per-tag found/rej/phase and"
    echo "                 timestamped acq/abort events on stderr (A/B analysis)"
    echo "  -h             Show this help"
    echo ""
    echo "Each toggle triggers a 4-way EAPOL handshake (4 frames)."
    echo "Expected EAPOL count: 4 x N_TOGGLES = $((4 * N_TOGGLES))"
}

while getopts "c:n:t:g:ah" opt; do
    case $opt in
        c) CHANNEL=$OPTARG ;;
        n) N_TOGGLES=$OPTARG ;;
        t) STF_THRESH=$OPTARG ;;
        g) STA_GATE_PCT=$OPTARG ;;
        a) ACQ_DIAG="-a" ;;
        h) usage; exit 0 ;;
        *) usage; exit 1 ;;
    esac
done

case "$STA_GATE_PCT" in
    ''|*[!0-9]*) echo "ERROR: -g must be an integer 0-100" >&2; exit 2 ;;
esac
[ "$STA_GATE_PCT" -le 100 ] || { echo "ERROR: -g must be <= 100" >&2; exit 2; }

# Sanity checks
preflight

# Check wifi is currently on
WIFI_STATE=$(networksetup -getairportpower "$WIFI_IFACE" 2>/dev/null | awk '{print $NF}')
if [ "$WIFI_STATE" != "On" ]; then
    echo "WARNING: WiFi is currently off. Turning on..." >&2
    networksetup -setairportpower "$WIFI_IFACE" on
    sleep 5
fi

# Total capture duration: 2s settle + N * (1s off + POST_TOGGLE_SEC wait + INTER_TOGGLE_SEC gap) + 3s tail
DURATION=$((2 + N_TOGGLES * (1 + POST_TOGGLE_SEC) + (N_TOGGLES - 1) * INTER_TOGGLE_SEC + 3))

echo "=== EAPOL Toggle Test ===" >&2
echo "  Channel:    $CHANNEL" >&2
echo "  Toggles:    $N_TOGGLES" >&2
echo "  STF thresh: $STF_THRESH" >&2
echo "  Duration:   ${DURATION}s" >&2
echo "  Expected:   $((4 * N_TOGGLES)) EAPOL frames" >&2
echo "" >&2

# Temp file for captured frames
TMPFILE=$(mktemp /tmp/eapol_toggle_XXXXXX.jsonl)
trap 'rm -f "$TMPFILE"; pluto_ssh "killall deimos_rx_dump 2>/dev/null" 2>/dev/null || true; networksetup -setairportpower "$WIFI_IFACE" on 2>/dev/null || true' EXIT

# Start deimos_rx_dump on Pluto via SSH, capture JSONL output locally
echo "Starting deimos_rx_dump on ch${CHANNEL} (${DURATION}s)..." >&2
pluto_ssh "deimos_rx_dump -c $CHANNEL -t $STF_THRESH $ACQ_DIAG -d $DURATION" > "$TMPFILE" &
DUMP_SSH_PID=$!

# Let radio settle
sleep 2

# Toggle wifi N times
for i in $(seq 1 "$N_TOGGLES"); do
    echo "  Toggle $i/$N_TOGGLES: wifi OFF..." >&2
    networksetup -setairportpower "$WIFI_IFACE" off 2>/dev/null || true
    sleep 1

    echo "  Toggle $i/$N_TOGGLES: wifi ON (waiting ${POST_TOGGLE_SEC}s for handshake)..." >&2
    networksetup -setairportpower "$WIFI_IFACE" on 2>/dev/null || true
    sleep "$POST_TOGGLE_SEC"

    if [ "$i" -lt "$N_TOGGLES" ]; then
        sleep "$INTER_TOGGLE_SEC"
    fi
done

# Wait for SSH to finish (deimos_rx_dump exits after duration)
echo "" >&2
echo "Waiting for capture to complete..." >&2
wait "$DUMP_SSH_PID" 2>/dev/null || true

# Ensure wifi is back on
networksetup -setairportpower "$WIFI_IFACE" on 2>/dev/null || true

# Output raw JSONL to stdout (for piping)
cat "$TMPFILE"

# Analyze results
echo "" >&2
echo "============================================================" >&2
echo "EAPOL TOGGLE TEST RESULTS" >&2
echo "============================================================" >&2

python3 -c "
import json, sys

frames = []
with open('$TMPFILE') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            frames.append(json.loads(line))
        except json.JSONDecodeError:
            pass

if not frames:
    print('  NO FRAMES CAPTURED', file=sys.stderr)
    sys.exit(1)

total = len(frames)
fcs_ok = sum(1 for f in frames if f.get('fcs'))
fcs_fail = total - fcs_ok

# Class breakdown
by_class = {}
for f in frames:
    cls = f.get('class', 'unknown')
    by_class[cls] = by_class.get(cls, 0) + 1

# EAPOL detection (from C tool's is_eapol field, or PSDU byte search fallback)
eapol_frames = [f for f in frames if f.get('is_eapol')]

for f in frames:
    if f in eapol_frames:
        continue
    psdu = f.get('psdu', '')
    if len(psdu) >= 32:
        raw = bytes.fromhex(psdu[:120])
        for i in range(len(raw) - 7):
            if (raw[i] == 0xAA and raw[i+1] == 0xAA and raw[i+2] == 0x03 and
                raw[i+3] == 0x00 and raw[i+4] == 0x00 and raw[i+5] == 0x00 and
                raw[i+6] == 0x88 and raw[i+7] == 0x8E):
                eapol_frames.append(f)
                break

# Report
print(f'  Total frames:    {total}', file=sys.stderr)
print(f'  FCS OK:          {fcs_ok}', file=sys.stderr)
print(f'  FCS fail:        {fcs_fail}', file=sys.stderr)
if fcs_ok > 0:
    print(f'  FCS pass rate:   {fcs_ok*100//total}%', file=sys.stderr)
print('', file=sys.stderr)

print('  --- Class Breakdown ---', file=sys.stderr)
for cls, count in sorted(by_class.items(), key=lambda x: -x[1]):
    print(f'    {cls:10s}: {count}', file=sys.stderr)

print('', file=sys.stderr)
print(f'  EAPOL detected:  {len(eapol_frames)} / ${N_TOGGLES}x4 = $((4 * N_TOGGLES)) expected', file=sys.stderr)

if eapol_frames:
    print('', file=sys.stderr)
    print('  --- EAPOL Frames ---', file=sys.stderr)
    for f in eapol_frames:
        ts_val = f.get('ts', 0)
        print(f'    ts={ts_val:.3f} rate={f.get(\"rate\",0)} '
              f'len={f.get(\"len\",0)} bssid={f.get(\"bssid\",\"?\")} '
              f'sa={f.get(\"sa\",\"?\")} da={f.get(\"da\",\"?\")}',
              file=sys.stderr)

# Verdict
print('', file=sys.stderr)
print('============================================================', file=sys.stderr)
if len(eapol_frames) >= 2:
    print(f'  VERDICT: PASS ({len(eapol_frames)} EAPOL captured)', file=sys.stderr)
elif len(eapol_frames) > 0:
    print(f'  VERDICT: PARTIAL ({len(eapol_frames)} EAPOL — expected more)', file=sys.stderr)
else:
    print(f'  VERDICT: FAIL (no EAPOL captured)', file=sys.stderr)
    if fcs_ok == 0:
        print(f'  HINT: No FCS-OK frames — check antenna/channel', file=sys.stderr)
    elif by_class.get('mgmt', 0) == 0:
        print(f'  HINT: No mgmt frames decoded — check decode mode and classifier', file=sys.stderr)
    else:
        print(f'  HINT: Mgmt decoded but no EAPOL — timing/channel mismatch?', file=sys.stderr)
print(f'============================================================', file=sys.stderr)
" 2>&2 || true

# --- 4-way handshake completeness + per-role STA gate ---------------------
# The toggle test's failure mode is losing the STA responses (M2/M4); each STA
# role is gated. AP roles are reported but not gated — today's OTA loss is
# AP-side, so gating AP would fail by construction. AP/STA MACs are inferred
# from the capture (STA MAC passed from en0 when available).
STA_MAC=$(ifconfig "$WIFI_IFACE" 2>/dev/null | awk '/ether/{print $2; exit}')
echo "" >&2
AB_JSON=$(python3 "$SCRIPT_DIR/eapol_ab_summary.py" "$TMPFILE" "$N_TOGGLES" "$STA_MAC" --json)
python3 "$SCRIPT_DIR/eapol_ab_summary.py" "$TMPFILE" "$N_TOGGLES" "$STA_MAC" >&2

M1_RATE=$(json_get "$AB_JSON" "roles.M1.rate")
M2_RATE=$(json_get "$AB_JSON" "roles.M2.rate")
M3_RATE=$(json_get "$AB_JSON" "roles.M3.rate")
M4_RATE=$(json_get "$AB_JSON" "roles.M4.rate")

STA_FAIL=0
if [ "$M2_RATE" -lt "$STA_GATE_PCT" ]; then
    STA_FAIL=1
    echo "  GATE FAIL: STA role M2 at ${M2_RATE}% < ${STA_GATE_PCT}%" >&2
fi
if [ "$M4_RATE" -lt "$STA_GATE_PCT" ]; then
    STA_FAIL=1
    echo "  GATE FAIL: STA role M4 at ${M4_RATE}% < ${STA_GATE_PCT}%" >&2
fi

if [ "$STA_FAIL" -eq 0 ]; then
    RESULT="PASS"; BLOCKED="false"
    echo "  VERDICT: PASS — STA M2=${M2_RATE}% M4=${M4_RATE}% (gate ${STA_GATE_PCT}%)" >&2
else
    RESULT="FAIL"; BLOCKED="true"
    echo "  VERDICT: FAIL — STA capture below the ${STA_GATE_PCT}% gate" >&2
fi

hwlog "eapol_toggle" "\"toggles\":$N_TOGGLES,\"expected_per_role\":$N_TOGGLES,\"sta_gate\":$STA_GATE_PCT,\"m1_rate\":$M1_RATE,\"m2_rate\":$M2_RATE,\"m3_rate\":$M3_RATE,\"m4_rate\":$M4_RATE,\"result\":\"$RESULT\",\"blocked\":$BLOCKED"
echo "  Results logged to: logs/hardware.jsonl (event eapol_toggle)" >&2

[ "$BLOCKED" = "false" ] || exit 1
exit 0
