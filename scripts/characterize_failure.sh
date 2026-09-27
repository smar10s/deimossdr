#!/bin/bash
# scripts/characterize_failure.sh — Capture + replay + EVM analysis in one command
#
# MANDATORY TOOL: Run this BEFORE proposing any RTL change for a failing rate.
# This script either captures a new ADC sample from hardware OR replays an
# existing capture, and produces per-symbol EVM analysis that shows exactly
# WHY the rate fails.
#
# Usage:
#   ./scripts/characterize_failure.sh <rate> [capture|replay <file>]
#
# Examples:
#   # Capture from hardware and analyze (Pluto must be connected):
#   ./scripts/characterize_failure.sh 12 capture
#
#   # Replay existing capture:
#   ./scripts/characterize_failure.sh 12 replay captures/passing/12m_cable_ch149_session23.json
#
#   # Auto: use existing capture if available, else prompt:
#   ./scripts/characterize_failure.sh 12
#
# Output:
#   - Per-symbol EVM (dB) showing where errors accumulate
#   - CPE and slope tracking behavior
#   - Diagnosis: drift vs noise vs timing
#   - Capture saved to captures/ for future regression use
#
# After fixing the rate in sim, move the capture to captures/passing/:
#   mv captures/12m_cable_ch149_sessionNN.json captures/passing/12m_cable_ch149.json
#   # It becomes a permanent regression gate via test_adc_replay
#
# WALL TIME: ~30-60 seconds (sim replay), ~10 seconds (ADC capture)
# AGENT TIMEOUT: 300000 ms (5 min)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CAPTURES_DIR="$PROJECT_DIR/captures"
FPGA_TEST="$PROJECT_DIR/fpga/test"
VENV="$PROJECT_DIR/.venv"

source "$SCRIPT_DIR/lib.sh"
export PATH="$VENV/bin:$PATH"

usage() {
    echo "Usage: $0 <rate_mbps> [capture|replay <file>]"
    echo ""
    echo "Rates: 6 9 12 18 24 36 48 54"
    echo ""
    echo "Modes:"
    echo "  capture       — Capture ADC from Pluto, then replay in sim"
    echo "  replay <file> — Replay existing capture in sim"
    echo "  (none)        — Auto: find existing capture or prompt"
    exit 1
}

if [ $# -lt 1 ]; then
    usage
fi

RATE="$1"
MODE="${2:-auto}"
CAPTURE_FILE="${3:-}"

# Validate rate
case "$RATE" in
    6|9|12|18|24|36|48|54) ;;
    *) echo "ERROR: Invalid rate '$RATE'. Must be 6|9|12|18|24|36|48|54"; exit 1 ;;
esac

mkdir -p "$CAPTURES_DIR"

# Determine capture file
if [ "$MODE" = "capture" ]; then
    # Capture from hardware
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    CAPTURE_FILE="$CAPTURES_DIR/cable_${RATE}m_${TIMESTAMP}.json"
    echo "=== Capturing ADC at rate ${RATE}M ==="
    echo "  Output: $CAPTURE_FILE"

    preflight

    # Run ADC capture (transmit at specified rate, capture RX)
    echo "  Running deimos_adc_capture -r $RATE ..."
    pluto_ssh "deimos_adc_capture -r $RATE -o /tmp/capture.json" || {
        echo "ERROR: ADC capture failed on Pluto"
        exit 1
    }

    # Copy back (use cat pipe — Pluto busybox lacks sftp-server for scp)
    pluto_ssh "cat /tmp/capture.json" > "$CAPTURE_FILE" || {
        echo "ERROR: Failed to copy capture from Pluto"
        exit 1
    }

    echo "  Captured: $(python3 -c "import json; d=json.load(open('$CAPTURE_FILE')); print(f'{len(d[\"real\"])} samples')")"

elif [ "$MODE" = "replay" ]; then
    if [ -z "$CAPTURE_FILE" ]; then
        echo "ERROR: replay mode requires a file path"
        usage
    fi
    if [ ! -f "$CAPTURE_FILE" ]; then
        echo "ERROR: Capture file not found: $CAPTURE_FILE"
        exit 1
    fi

elif [ "$MODE" = "auto" ]; then
    # Find existing capture for this rate
    EXISTING=$(find "$CAPTURES_DIR" \( -name "*${RATE}m*" -o -name "*${RATE}mbps*" \) -print | sort -r | head -1)
    if [ -n "$EXISTING" ]; then
        CAPTURE_FILE="$EXISTING"
        echo "=== Auto: found existing capture ==="
        echo "  Using: $CAPTURE_FILE"
    else
        echo "=== No existing capture for rate ${RATE}M ==="
        echo "  Available captures:"
        ls "$CAPTURES_DIR"/*.json 2>/dev/null | sed 's/^/    /' || echo "    (none)"
        echo ""
        echo "  To capture from hardware: $0 $RATE capture"
        echo "  To replay a file: $0 $RATE replay <path>"
        exit 1
    fi
else
    usage
fi

echo ""
echo "=== Replaying in sim with EVM analysis ==="
echo "  File: $CAPTURE_FILE"
echo "  Rate: ${RATE}M"
echo "  CFO threshold: 64 (live mode)"
echo ""

# Run EVM analysis diagnostic via the Makefile target (handles VERILOG_SOURCES correctly)
export ADC_CAPTURE_FILE="$(cd "$(dirname "$CAPTURE_FILE")" && pwd)/$(basename "$CAPTURE_FILE")"
export CFO_THRESHOLD=64

make -C "$FPGA_TEST" diag_evm 2>&1 | tee /tmp/evm_analysis_${RATE}m.log

echo ""
echo "=== Analysis complete ==="
echo "  Full log: /tmp/evm_analysis_${RATE}m.log"
echo ""
echo "Next steps:"
echo "  - If PASS: move to captures/passing/ (becomes regression gate)"
echo "    mv $CAPTURE_FILE captures/passing/${RATE}m_cable_$(date +%Y%m%d).json"
echo ""
echo "  - If FAIL with 'progressive degradation': implement H update"
echo "  - If FAIL with 'uniformly high EVM': check H estimation / timing"
echo "  - If FAIL with 'isolated bad symbol': check FFT/timing glitch"
echo ""
echo "  Re-run after RTL fix: $0 $RATE replay $CAPTURE_FILE"
