#!/bin/bash
# scripts/session_start.sh — Establish known hardware state before a session
#
# Confirms:
#   1. Pluto reachable, firmware tools deployed
#   2. Flashed fingerprint + bitstream sha match build/fpga/
#   3. RF path is clean (ARM decode, pluto_loopback 20 trials/rate) — the
#      guard that keeps "the analog path" from being blamed for RTL bugs
#   4. HIL regression 80/80 (fabric logic on hardware)
#   5. Cable loopback baseline on gate rates 6/9/12/18/24/36 (20 trials/rate)
#      + report-only 48/54 (100 trials/rate)
#
# Results are appended to logs/hardware.jsonl. The regression ratchet in
# session_end.sh derives its baseline from that log — no separate file.
#
# Measured wall time: ~75s. AGENT TIMEOUT: 300000 ms (5 min).
#
# Usage:
#   ./scripts/session_start.sh            # full check (~75s)
#   ./scripts/session_start.sh --quick    # skip loopback (identity + HIL only)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/lib.sh"

QUICK=false
[[ "${1:-}" == "--quick" ]] && QUICK=true

echo "=== Session Start: Hardware State Check ==="
echo ""

# ---- Step 1: Connectivity + firmware deploy ----
echo "1. Connectivity & firmware deploy"
if ! pluto_ssh "true" 2>/dev/null; then
    result "FAIL" "Cannot reach Pluto at $PLUTO_IP"
    echo ""
    echo "ABORT: Pluto not reachable. Check USB connection."
    exit 2
fi
result "PASS" "Pluto reachable at $PLUTO_IP"

echo "  Deploying firmware tools..."
if make -C "$PROJECT_DIR" deploy >/dev/null 2>&1; then
    result "PASS" "Firmware deployed"
else
    if pluto_ssh "which deimos_hil_inject && which deimos_fabric_loopback" >/dev/null 2>&1; then
        result "WARN" "Deploy failed but tools already present on device"
    else
        result "FAIL" "Firmware deploy failed and tools missing (run 'make firmware' first?)"
    fi
fi
echo ""

# ---- Step 2: Fingerprint + bitstream identity ----
# retry_ssh guards the post-SCP USB transient (observed: one-off ssh flake
# right after deploy used to produce a false "Cannot read fingerprint").
echo "2. Build fingerprint"
BUILD_ID_ADDR="0x43C00000"
if ! RAW=$(retry_ssh 3 "devmem $BUILD_ID_ADDR 32"); then
    result "FAIL" "Cannot read fingerprint register (3 attempts)"
else
    FINGERPRINT=$(echo "$RAW" | tr -d '[:space:]' | sed 's/0[xX]//' | tr '[:upper:]' '[:lower:]')
    echo "  Hardware fingerprint: 0x$FINGERPRINT"
    if [ -f "$PROJECT_DIR/build/fpga/fingerprint" ]; then
        FP_FILE="$PROJECT_DIR/build/fpga/fingerprint"
        EXPECTED=$(cat "$FP_FILE" | tr '[:upper:]' '[:lower:]' | sed 's/0[xX]//' | tr -d '[:space:]')
        echo "  Expected (build/fpga/fingerprint): 0x$EXPECTED"
        if [ "$FINGERPRINT" = "$EXPECTED" ]; then
            result "PASS" "Fingerprint matches build/fpga/"
        else
            result "WARN" "Fingerprint MISMATCH — flashed build differs from build/"
            echo "         Check STATUS.md for what should be flashed."
        fi
    else
        result "WARN" "No build/fpga/fingerprint file — cannot verify"
    fi
fi

echo ""
echo "2b. Bitstream identity"
identity
BITSTREAM="$ID_BITSTREAM"
if [ -f "$PROJECT_DIR/build/fpga/bitstream.sha256" ]; then
    EXPECTED_BSHA=$(cat "$PROJECT_DIR/build/fpga/bitstream.sha256" | tr -d '[:space:]')
    if DEV_BSHA=$(retry_ssh 3 "fw_printenv -n deimos_bitsha 2>/dev/null"); then
        DEV_BSHA=$(echo "$DEV_BSHA" | tr -d '[:space:]')
        BITSTREAM="$DEV_BSHA"
        if [ "$DEV_BSHA" = "$EXPECTED_BSHA" ]; then
            result "PASS" "Device bitstream sha256 matches build/fpga/ ($DEV_BSHA)"
        else
            result "WARN" "Bitstream MISMATCH — flashed netlist differs from build/fpga/system_top.bit"
            echo "         The fingerprint proves the SOURCE tree; this proves the netlist."
        fi
    else
        result "WARN" "deimos_bitsha not readable on device (pre-identity build?)"
    fi
else
    result "WARN" "No build/fpga/bitstream.sha256 — build predates identity tracking"
fi
echo ""

# ---- Step 3: RF path validation (ARM decode) ----
echo "3. RF path (pluto_loopback — ARM decode, 20 trials/rate)"
echo "   This proves the analog chain is clean. Failures here = cable/hardware problem."

ARM_JSON=$(pluto_ssh "pluto_loopback -n 20" 2>/dev/null; true)
[ -z "$ARM_JSON" ] && ARM_JSON='{}'

# Evaluate per-rate with a threshold: 18/20 = pass (1-2 drops are noise)
ARM_THRESH=18
ARM_RATES_OK=0
ARM_RATES_TOTAL=0
ARM_DETAIL=""
ARM_FAILURES=""
ARM_LOG=""
for rate in 6 9 12 18 24 36 48 54; do
    RATE_PASS=$(json_rate_pass "$ARM_JSON" "$rate")
    RATE_PASS=${RATE_PASS:-0}
    ARM_RATES_TOTAL=$((ARM_RATES_TOTAL + 1))
    if [ "$RATE_PASS" -ge "$ARM_THRESH" ]; then
        ARM_RATES_OK=$((ARM_RATES_OK + 1))
    else
        ARM_FAILURES="${ARM_FAILURES}    Rate ${rate}M: ${RATE_PASS}/20\n"
    fi
    if [ "$RATE_PASS" -lt 20 ]; then
        ARM_DETAIL="${ARM_DETAIL}    Rate ${rate}M: ${RATE_PASS}/20\n"
    fi
    ARM_LOG="${ARM_LOG}\"${rate}\":${RATE_PASS},"
done
ARM_LOG="${ARM_LOG%,}"

if [ "$ARM_RATES_OK" -eq "$ARM_RATES_TOTAL" ]; then
    if [ -n "$ARM_DETAIL" ]; then
        echo "  Minor drops (within threshold):"
        echo -e "$ARM_DETAIL"
    fi
    result "PASS" "ARM decode: ${ARM_RATES_OK}/${ARM_RATES_TOTAL} rates >= ${ARM_THRESH}/20"
else
    echo "  Rates below ${ARM_THRESH}/20 threshold:"
    echo -e "$ARM_FAILURES"
    if [ "$ARM_RATES_OK" -ge $(( ARM_RATES_TOTAL - 1 )) ]; then
        result "WARN" "ARM decode: ${ARM_RATES_OK}/${ARM_RATES_TOTAL} rates pass (minor RF issue)"
    else
        result "FAIL" "ARM decode: ${ARM_RATES_OK}/${ARM_RATES_TOTAL} rates pass — RF path problem!"
        echo "         Cable/connector/hardware issue. Do NOT investigate fabric until this passes."
    fi
fi

echo ""

# ---- Step 4: HIL regression ----
echo "4. HIL regression (10 trials/rate)"

HIL_RESULT=$("$SCRIPT_DIR/hil_regression.sh" -n 10 --json 2>/dev/null)
HIL_EXIT=$?
HIL_STATUS="unknown"

if [ $HIL_EXIT -eq 0 ]; then
    result "PASS" "HIL 80/80 (10 trials × 8 rates)"
    HIL_STATUS="80/80"
elif [ $HIL_EXIT -eq 2 ]; then
    result "FAIL" "HIL infrastructure error (tool missing or Pluto unreachable)"
    HIL_STATUS="error"
else
    echo "  $HIL_RESULT" | grep -i "fail" | head -5
    result "FAIL" "HIL regression FAILED — baseline broken"
    HIL_STATUS="fail"
fi

echo ""

# ---- Step 5: Cable loopback (skip if --quick) ----
LOOPBACK_TRIALS=20
REPORT_TRIALS=100
LEVEL_LOG="{}"

if [ "$QUICK" = true ]; then
    echo "5. Cable loopback — SKIPPED (--quick mode)"
    LB_LOG="\"skipped\":true"
else
    echo "5. Cable loopback (gate: $LOOPBACK_TRIALS trials, report: $REPORT_TRIALS trials)"
    echo "   Gate rates: 6/9/12/18/24/36 (FAIL if <80%)"
    echo "   Report-only: 48/54 (logged, never fail — known EVM margin issue)"
    loopback_gate "$LOOPBACK_TRIALS" "$REPORT_TRIALS"

    # Level telemetry (report-only): ADC peak/RMS/clipping at the gate's
    # operating point so cold/hot LO-cal drift is visible in the log
    # instead of surfacing as an unexplained loopback failure. D18.
    LEVEL_JSON=$(pluto_ssh "deimos_adc_capture -L" 2>/dev/null; true)
    [ -z "$LEVEL_JSON" ] && LEVEL_JSON='{}'
    LEVEL_PEAK=$(json_get "$LEVEL_JSON" "peak_i")
    LEVEL_RMS=$(json_get "$LEVEL_JSON" "rms_i")
    LEVEL_CLIP=$(json_get "$LEVEL_JSON" "clipped")
    LEVEL_LOG="{\"peak_i\":${LEVEL_PEAK:--999},\"rms_i\":${LEVEL_RMS:--1},\"clipped\":${LEVEL_CLIP:--1}}"
    echo ""
    echo "  Level telemetry (peak/RMS/clipped): $LEVEL_LOG"
    ABS_PEAK=${LEVEL_PEAK#-}
    if [ "$ABS_PEAK" -ge 2040 ] 2>/dev/null; then
        result "WARN" "ADC at/near full scale (peak ${LEVEL_PEAK}/2047) — gain cliff risk, see D18"
    elif [ "$ABS_PEAK" -lt 1000 ] 2>/dev/null; then
        result "WARN" "ADC low signal (peak ${LEVEL_PEAK}/2047, healthy ≥1200) — gain table edge or hardware issue, see D18"
    fi
fi

echo ""

# ---- Append to hardware log ----
hwlog "session_start" "\"hil\":\"${HIL_STATUS}\",\"arm\":{${ARM_LOG}},\"loopback\":{${LB_LOG}},\"level\":${LEVEL_LOG},\"trials\":${LOOPBACK_TRIALS},\"report_trials\":${REPORT_TRIALS}"
echo "  Results logged to: logs/hardware.jsonl"

result_summary

if [ $GATE_FAIL -gt 0 ]; then
    echo "ACTION: Investigate failures before starting RTL work."
    echo "The baseline is NOT confirmed. Do not assume prior results hold."
    exit 1
elif [ $GATE_WARN -gt 0 ]; then
    echo "ACTION: Warnings present. Verify expectations before proceeding."
    exit 0
else
    echo "Baseline confirmed. Ready to begin session work."
    exit 0
fi
