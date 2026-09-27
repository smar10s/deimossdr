#!/bin/bash
# scripts/session_end.sh — Pre-merge regression gate
#
# Run this before declaring work done or requesting merge review.
# Exits nonzero if any gate fails. The agent MUST NOT claim
# "ready for merge" if this script fails.
#
# Gates:
#   1. sim.sh — full gate suite, ALL tests must pass
#   2. Uncommitted RTL warning (changes must be committed to be gated)
#   3. HIL regression — 80/80 (fabric logic on hardware)
#   4. Cable loopback — absolute floor (>=80% on gate rates) AND
#      regression ratchet vs the most recent green session_end for this
#      bitstream in logs/hardware.jsonl (never worse by >2 trials/rate)
#
# Results are appended to logs/hardware.jsonl (persistent, append-only).
#
# Measured wall time: sim (~6.5 min warm) + ~1.5 min hardware. AGENT TIMEOUT:
# 900000 ms (15 min) when invoking from OpenCode.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/lib.sh"

echo "=== Session End Gate Check ==="
echo ""

# ---- Step 1: Sim gate ----
echo "1. Running full sim suite (sim.sh)..."
echo ""
if ! "$SCRIPT_DIR/sim.sh"; then
    echo ""
    echo "BLOCKED: sim tests failed. Fix before claiming done."
    hwlog "session_end" "\"sim\":\"FAIL\",\"blocked\":true"
    exit 1
fi
SIM_STATUS="PASS"
echo ""

# ---- Step 2: Check for uncommitted RTL ----
echo "2. Checking working tree..."
RTL_DIRTY=$(git -C "$PROJECT_DIR" diff --name-only -- fpga/rtl/ 2>/dev/null || true)
if [ -n "$RTL_DIRTY" ]; then
    echo "  WARNING: Uncommitted RTL changes:"
    echo "$RTL_DIRTY" | sed 's/^/    /'
    echo "  These changes have NOT been through the sim gate in their committed form."
    echo ""
fi
echo ""

# ---- Step 3: HIL regression ----
echo "3. HIL regression (10 trials/rate)"

if ! pluto_ssh "true" 2>/dev/null; then
    result "FAIL" "Cannot reach Pluto at $PLUTO_IP — skipping hardware gates"
    echo ""
    echo "BLOCKED: hardware unreachable. Cannot validate."
    hwlog "session_end" "\"sim\":\"PASS\",\"hil\":\"unreachable\",\"blocked\":true"
    exit 1
fi

HIL_RESULT=$("$SCRIPT_DIR/hil_regression.sh" -n 10 --json 2>/dev/null)
HIL_EXIT=$?
HIL_STATUS="unknown"

if [ $HIL_EXIT -eq 0 ]; then
    result "PASS" "HIL 80/80 (10 trials × 8 rates)"
    HIL_STATUS="80/80"
else
    result "FAIL" "HIL regression FAILED"
    HIL_STATUS="fail"
    echo ""
    echo "BLOCKED: HIL must pass 80/80 before merge."
    hwlog "session_end" "\"sim\":\"PASS\",\"hil\":\"${HIL_STATUS}\",\"blocked\":true"
    exit 1
fi

echo ""

# ---- Step 4: Cable loopback vs regression ratchet ----
LOOPBACK_TRIALS=20
REPORT_TRIALS=100
echo "4. Cable loopback (gate: $LOOPBACK_TRIALS trials, report: $REPORT_TRIALS trials)"
echo "   Gate rates: 6/9/12/18/24/36 | Report-only: 48/54"

BASELINE=$(gate_baseline_for_bitstream)
if [ "$BASELINE" = "{}" ]; then
    echo "  NOTE: no prior green session_end for this bitstream in hardware.jsonl."
    echo "  Gating on the absolute floor (>=80%) only; this run becomes the baseline."
else
    echo "  Ratchet baseline (last green session_end, this bitstream): $BASELINE"
fi

loopback_gate "$LOOPBACK_TRIALS" "$REPORT_TRIALS" "$BASELINE"

echo ""

# ---- Append to hardware log ----
if [ $GATE_FAIL -gt 0 ]; then
    hwlog "session_end" "\"sim\":\"PASS\",\"hil\":\"${HIL_STATUS}\",\"loopback\":{${LB_LOG}},\"trials\":${LOOPBACK_TRIALS},\"report_trials\":${REPORT_TRIALS},\"blocked\":true,\"reason\":\"loopback_regression\""
    echo "  Results logged to: logs/hardware.jsonl"
    result_summary
    echo "BLOCKED: Fix failures before claiming done."
    exit 1
fi

hwlog "session_end" "\"sim\":\"PASS\",\"hil\":\"${HIL_STATUS}\",\"loopback\":{${LB_LOG}},\"trials\":${LOOPBACK_TRIALS},\"report_trials\":${REPORT_TRIALS},\"blocked\":false"
echo "  Results logged to: logs/hardware.jsonl"

result_summary
echo "ALL GATES PASS. Ready for merge review."
