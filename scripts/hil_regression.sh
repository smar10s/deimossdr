#!/bin/bash
# hil_regression.sh — HIL inject regression gate
#
# Runs 10 trials per rate (8 rates, 80 total). Exits nonzero if any
# trial fails. This is the mandatory gate before accepting any RTL change.
#
# Transport errors (SSH auth glitches, transient connection drops) are
# retried up to MAX_SSH_RETRIES times and do NOT count as device FAILs —
# at ~0.6% occurrence they would otherwise contaminate every failure
# count. Empty output with a successful SSH exit is a device-side
# failure (snap timeout / no tag) and counts as a FAIL. If transport
# errors accumulate (>= 3), the run aborts as infrastructure failure.
#
# Usage:
#   ./scripts/hil_regression.sh           # default: 10 trials, all rates
#   ./scripts/hil_regression.sh -n 5      # 5 trials per rate (quick check)
#   ./scripts/hil_regression.sh -r 6      # single rate
#   ./scripts/hil_regression.sh --json    # JSON output for CI
#
# Exit codes:
#   0 = all pass
#   1 = one or more failures
#   2 = infrastructure error (Pluto unreachable, tool missing, etc.)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/lib.sh"

RATES="6mbps 9mbps 12mbps 18mbps 24mbps 36mbps 48mbps 54mbps"
TRIALS=10
JSON_OUTPUT=false
SINGLE_RATE=""
MAX_SSH_RETRIES=3

while [[ $# -gt 0 ]]; do
    case $1 in
        -n) TRIALS="$2"; shift 2 ;;
        -r) SINGLE_RATE="$2"; shift 2 ;;
        --json) JSON_OUTPUT=true; shift ;;
        -h|--help)
            echo "Usage: $0 [-n trials] [-r rate] [--json]"
            echo "  -n trials   Trials per rate (default: 10)"
            echo "  -r rate     Single rate to test (e.g., 6, 24)"
            echo "  --json      Output JSON summary"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 2 ;;
    esac
done

if [[ -n "$SINGLE_RATE" ]]; then
    RATES="${SINGLE_RATE}mbps"
fi

# --- Preflight checks ---

preflight

if ! pluto_ssh "which deimos_hil_inject" >/dev/null 2>&1; then
    echo "ERROR: deimos_hil_inject not found on Pluto (run 'make deploy')" >&2
    exit 2
fi

# Ensure snap mode 0 (state/FCS decode result) — other modes produce
# data that the parse_fcs_ok function cannot interpret correctly.
pluto_ssh "devmem 0x7C50001C 32 0" 2>/dev/null

# parse_fcs_ok comes from lib.sh

# --- Run tests ---

run_trial() {
    # One injection with transport-error retry. Prints snap words on
    # success (device-side failures produce empty output with rc=0,
    # which parse_fcs_ok reads as a device FAIL). Returns nonzero only
    # if every attempt hit a transport error.
    local rate=$1 output
    output=$(retry_ssh "$MAX_SSH_RETRIES" "deimos_hil_inject -q $rate") || return 1
    printf '%s' "$output"
    return 0
}

echo "HIL Inject Regression — $(date)" >&2
echo "Trials/rate: $TRIALS, Rates: $RATES" >&2
echo "" >&2

total_pass=0
total_tests=0
transport_errors=0
all_pass=true
json_rates="["

if ! $JSON_OUTPUT; then
    printf "%-8s | Pass/Total | Status\n" "Rate" >&2
    printf "%s\n" "---------|------------|-------" >&2
fi

for rate in $RATES; do
    pass=0
    for ((i=1; i<=TRIALS; i++)); do
        if result=$(run_trial "$rate"); then
            fcs=$(printf '%s' "$result" | parse_fcs_ok)
            if [ "$fcs" -eq 1 ]; then
                pass=$((pass + 1))
            fi
        else
            transport_errors=$((transport_errors + 1))
        fi
        sleep 0.2
    done
    total_pass=$((total_pass + pass))
    total_tests=$((total_tests + TRIALS))

    status="PASS"
    if [ "$pass" -ne "$TRIALS" ]; then
        status="FAIL"
        all_pass=false
    fi

    if ! $JSON_OUTPUT; then
        printf "%-8s | %d/%d      | %s\n" "$rate" "$pass" "$TRIALS" "$status" >&2
    fi

    # Build JSON array element
    rate_num="${rate%mbps}"
    json_rates="$json_rates{\"rate\":$rate_num,\"pass\":$pass,\"trials\":$TRIALS},"
done

json_rates="${json_rates%,}]"

echo "" >&2
echo "TOTAL: $total_pass/$total_tests" >&2

if [ "$transport_errors" -gt 0 ]; then
    echo "TRANSPORT ERRORS: $transport_errors (retried, not counted)" >&2
fi

if $JSON_OUTPUT; then
    echo "{\"total_pass\":$total_pass,\"total_trials\":$total_tests,\"transport_errors\":$transport_errors,\"all_pass\":$all_pass,\"rates\":$json_rates}"
fi

if [ "$transport_errors" -ge 3 ]; then
    echo "RESULT: INFRASTRUCTURE FAILURE — $transport_errors transport errors (Pluto/SSH unstable)" >&2
    exit 2
fi

if $all_pass; then
    echo "RESULT: ALL PASS ✓" >&2
    exit 0
else
    echo "RESULT: REGRESSION DETECTED — DO NOT DEPLOY" >&2
    exit 1
fi
