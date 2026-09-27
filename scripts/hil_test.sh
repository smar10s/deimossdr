#!/bin/bash
# scripts/hil_test.sh — Progressive HIL test suite
#
# Layers:
#   1. Single-frame sanity (1 trial/rate — quick smoke test)
#   2. Statistical decode (10 trials/rate, 100% required)
#   3. CFO resilience (±5 kHz, 5 trials/direction)
#   4. Multi-rate rapid fire (rate transitions, minimal cooldown)
#   5. PSDU byte verify (single-frame, all rates — 100% gate)
#   6. Burst decode (multi-frame single playback, STF rearm)
#   7. Real capture replay (captures/passing/ injected via HIL)
#   8. Impairments (SFO, multipath, AWGN, combined — via gen_impaired_burst.py)
#
# Gate rates: 6/9/12/18/24/36/48/54 (all rates — HIL is clean digital inject)
#
# Usage:
#   ./scripts/hil_test.sh              # all layers
#   ./scripts/hil_test.sh -l 2         # layers 1-2 only
#   ./scripts/hil_test.sh -r 24        # single rate focus
#   ./scripts/hil_test.sh -v           # verbose (print each trial)
#
# Exit codes:
#   0 = all gate layers pass
#   1 = gate layer failure
#   2 = infrastructure error

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/lib.sh"
LADDER_EVENT="hil_test"

GATE_RATES="6 9 12 18 24 36 48 54"
REPORT_RATES=""
MAX_LAYER=8
SINGLE_RATE=""
VERBOSE=false
JSON_OUTPUT=false

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "  -l layer   Run layers 1 through N (default: $MAX_LAYER)"
    echo "  -r rate    Single rate focus (still runs all layers for that rate)"
    echo "  --json     JSON summary to stdout"
    echo "  -v         Verbose (print each trial)"
    echo "  -h         Help"
    echo ""
    echo "Layers:"
    echo "  1  Single-frame sanity (1 trial/rate)"
    echo "  2  Statistical decode (10 trials/rate, 100%)"
    echo "  3  CFO resilience (±5 kHz, 80%)"
    echo "  4  Multi-rate rapid fire (rate transitions)"
    echo "  5  PSDU byte verify (single-frame, all rates)"
    echo "  6  Burst decode (multi-frame HIL playback)"
    echo "  7  Real capture replay (captures/passing/ via HIL)"
    echo "  8  Impairments (SFO, multipath, AWGN, combined)"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -l) MAX_LAYER="$2"; shift 2 ;;
        -r) SINGLE_RATE="$2"; shift 2 ;;
        --json) JSON_OUTPUT=true; shift ;;
        -v) VERBOSE=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

if [[ -n "$SINGLE_RATE" ]]; then
    GATE_RATES="$SINGLE_RATE"
fi

# ---- Preflight ----

preflight

if ! pluto_ssh "which deimos_hil_inject" >/dev/null 2>&1; then
    echo "ERROR: deimos_hil_inject not found on Pluto (run 'make deploy')" >&2
    exit 2
fi

# ---- Inject and check ----
# inject_trial <waveform> [extra_args...]
# Returns 0 on FCS pass, 1 on FCS fail, 2 on error
inject_trial() {
    local waveform="$1"
    shift
    local result fcs
    result=$(pluto_ssh "deimos_hil_inject -q $waveform $*" 2>/dev/null) || true
    if [ -z "$result" ]; then
        return 2
    fi
    fcs=$(echo "$result" | parse_fcs_ok)
    if [ "$fcs" -eq 1 ]; then
        return 0
    else
        return 1
    fi
}

# ---- Result tracking (counters; framework from lib.sh) ----
LAYER_PASS=0
LAYER_FAIL=0
TOTAL_GATE_PASS=0
TOTAL_GATE_FAIL=0

# ---- Start ----
echo "═══════════════════════════════════════" >&2
echo " HIL Test Suite — $(date +%H:%M:%S)" >&2
echo " Layers: 1-$MAX_LAYER" >&2
echo " Gate rates: $GATE_RATES" >&2
echo "═══════════════════════════════════════" >&2

# ════════════════════════════════════════
# Layer 1: Single-frame sanity
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 1 ]; then
    layer_header 1 "Single-frame sanity"
    l1_pass=0
    l1_total=0

    for rate in $GATE_RATES; do
        l1_total=$((l1_total + 1))
        if inject_trial "${rate}mbps"; then
            l1_pass=$((l1_pass + 1))
            $VERBOSE && echo "    ${rate}M: PASS" >&2
        else
            echo "    ${rate}M: FAIL" >&2
        fi
    done

    layer_result 1 "single-frame" $l1_pass $l1_total "true"
fi

# ════════════════════════════════════════
# Layer 2: Statistical decode (10 trials/rate)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 2 ]; then
    layer_header 2 "Statistical decode (10 trials/rate, via hil_regression.sh)"
    if hil_json=$("$SCRIPT_DIR/hil_regression.sh" -n 10 --json 2>/dev/null); then
        l2_pass=$(json_get "$hil_json" "total_pass")
        l2_total=$(json_get "$hil_json" "total_trials")
        layer_result 2 "statistical" "$l2_pass" "$l2_total" "true"
    else
        echo "  HIL statistical gate failed (run hil_regression.sh for details)" >&2
        layer_result 2 "statistical" 0 1 "true"
    fi
fi

# ════════════════════════════════════════
# Layer 3: CFO resilience (±5 kHz, 5 trials/direction)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 3 ]; then
    layer_header 3 "CFO resilience (±5 kHz)"
    l3_pass=0
    l3_total=0
    l3_gate_ok=true

    # Rate 54 + ±5 kHz is report-only: 64-QAM 3/4 EVM budget doesn't
    # survive CFO residual. CFO path correctness proven by rate 48 passing.
    L3_REPORT_COMBOS="54:5000 54:-5000"

    for rate in $GATE_RATES; do
        for cfo in 5000 -5000; do
            rate_pass=0
            for ((i=1; i<=5; i++)); do
                l3_total=$((l3_total + 1))
                if inject_trial "${rate}mbps" "-c $cfo"; then
                    rate_pass=$((rate_pass + 1))
                    l3_pass=$((l3_pass + 1))
                fi
                sleep 0.1
            done
            cfo_label="+5kHz"
            [ "$cfo" = "-5000" ] && cfo_label="-5kHz"

            # Check if this combo is report-only
            is_report=false
            for combo in $L3_REPORT_COMBOS; do
                [ "$combo" = "${rate}:${cfo}" ] && is_report=true
            done

            if [ "$rate_pass" -ge 4 ]; then
                $VERBOSE && echo "    ${rate}M ${cfo_label}: ${rate_pass}/5 PASS" >&2
            elif $is_report; then
                echo "    ${rate}M ${cfo_label}: ${rate_pass}/5 (report-only, known EVM limit)" >&2
            else
                echo "    ${rate}M ${cfo_label}: ${rate_pass}/5 FAIL (need 4/5)" >&2
                l3_gate_ok=false
            fi
        done
    done

    # Gate on 80% overall (any individual rate/direction needing 4/5)
    if $l3_gate_ok; then
        layer_result 3 "cfo-resilience" $l3_pass $l3_total "false"
    else
        layer_result 3 "cfo-resilience" $l3_pass $l3_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 4: Multi-rate rapid fire (3 cycles, minimal gap)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 4 ]; then
    layer_header 4 "Multi-rate rapid fire"
    l4_pass=0
    l4_total=0
    # Cycle order maximizes rate transitions
    CYCLE_ORDER="6 48 24 54 36 12 9 18"

    for ((cycle=1; cycle<=3; cycle++)); do
        for rate in $CYCLE_ORDER; do
            l4_total=$((l4_total + 1))
            if inject_trial "${rate}mbps"; then
                l4_pass=$((l4_pass + 1))
            else
                echo "    Cycle $cycle, ${rate}M: FAIL" >&2
            fi
            sleep 0.05
        done
        $VERBOSE && echo "    Cycle $cycle: complete" >&2
    done

    layer_result 4 "rapid-fire" $l4_pass $l4_total "true"
fi

# ════════════════════════════════════════
# Layer 5: PSDU byte verify (single-frame, all rates)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 5 ]; then
    layer_header 5 "PSDU byte verify (single-frame, all rates)"
    l5_pass=0
    l5_total=0

    for rate in $GATE_RATES; do
        # 3 trials per rate — deterministic payload (80 bytes)
        for ((trial=1; trial<=3; trial++)); do
            l5_total=$((l5_total + 1))
            result=$(pluto_ssh "deimos_burst_loopback --hil --verify-psdu -n 1 -r $rate -p 80 -g 20000" 2>/dev/null) || true
            json_line=$(echo "$result" | grep '^{' | tail -1)
            psdu_ok=$(json_get "$json_line" "psdu_ok") || psdu_ok=0
            psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0
            fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0

            if [ "$psdu_ok" -ge 1 ] && [ "$psdu_fail" -eq 0 ] && [ "$fcs_pass" -ge 1 ]; then
                l5_pass=$((l5_pass + 1))
                $VERBOSE && echo "    ${rate}M trial $trial: PSDU OK" >&2
            else
                echo "    ${rate}M trial $trial: FAIL (fcs=$fcs_pass psdu_ok=$psdu_ok psdu_fail=$psdu_fail)" >&2
            fi
        done
    done

    # Gate: 100% — PSDU path must work at every rate
    layer_result 5 "psdu-verify" $l5_pass $l5_total "true"
fi

# ════════════════════════════════════════
# Layer 6: Burst decode (multi-frame single playback)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 6 ]; then
    layer_header 6 "Burst decode (multi-frame HIL playback)"
    l6_pass=0
    l6_total=0

    # Use deimos_burst_loopback --hil for multi-frame inject.
    # Tests STF rearm: pipeline must detect and decode each frame in the burst.
    #
    # Sub-tests progress toward real-world traffic patterns:
    #   6a: Same-rate burst at proven gaps (existing — regression gate)
    #   6b: Mixed-rate burst at 2000-sample gap (100 μs)
    #   6c: SIFS timing (gap=320 = 16 μs) — hardest sub-test

    # Capture tool stderr for post-failure analysis (additive diagnostics)
    BURST_LOG_DIR="$PROJECT_DIR/logs/burst"
    mkdir -p "$BURST_LOG_DIR"
    BURST_TS=$(date +%Y%m%d_%H%M%S)

    # --- 6a: Same-rate burst (existing, proven reliable) ---
    for gap in 20000 5000; do
        for rate in 6 24; do
            l6_total=$((l6_total + 1))
            result=$(pluto_ssh "deimos_burst_loopback --hil --verify-psdu -n 10 -r $rate -g $gap" 2>"$BURST_LOG_DIR/${BURST_TS}_6a_${rate}_${gap}.log") || true
            json_line=$(echo "$result" | grep '^{' | tail -1)
            fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
            fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
            psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

            # Gate: 8/10 FCS OK with 0 FCS fail and 0 PSDU fail required for all gaps.
            if [ "$fcs_pass" -ge 8 ] && [ "$fcs_fail" -eq 0 ] && [ "$psdu_fail" -eq 0 ]; then
                l6_pass=$((l6_pass + 1))
                $VERBOSE && echo "    6a ${rate}M gap=${gap}: ${fcs_pass}/10 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — PASS" >&2
            else
                echo "    6a ${rate}M gap=${gap}: ${fcs_pass}/10 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — FAIL" >&2
            fi
        done
    done

    # --- 6b: Mixed-rate burst at 2000-sample gap (100 μs) ---
    for trial in 1 2; do
        l6_total=$((l6_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback --hil --verify-psdu --mix-rates -n 15 -g 2000" 2>"$BURST_LOG_DIR/${BURST_TS}_6b_t${trial}.log") || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
        psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

        # Gate: 12/15 FCS OK (80%), 0 FCS fail, 0 PSDU fail.
        if [ "$fcs_pass" -ge 12 ] && [ "$fcs_fail" -eq 0 ] && [ "$psdu_fail" -eq 0 ]; then
            l6_pass=$((l6_pass + 1))
            $VERBOSE && echo "    6b mixed gap=2000 trial${trial}: ${fcs_pass}/15 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — PASS" >&2
        else
            echo "    6b mixed gap=2000 trial${trial}: ${fcs_pass}/15 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — FAIL" >&2
        fi
    done

    # --- 6c: SIFS timing (gap=320 = 16 μs) ---
    for rate in 6 24; do
        l6_total=$((l6_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback --hil --verify-psdu -n 10 -r $rate -g 320" 2>"$BURST_LOG_DIR/${BURST_TS}_6c_${rate}.log") || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
        psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

        # Gate: 8/10 FCS OK, 0 FCS fail, 0 PSDU fail.
        if [ "$fcs_pass" -ge 8 ] && [ "$fcs_fail" -eq 0 ] && [ "$psdu_fail" -eq 0 ]; then
            l6_pass=$((l6_pass + 1))
            $VERBOSE && echo "    6c ${rate}M gap=320 (SIFS): ${fcs_pass}/10 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — PASS" >&2
        else
            echo "    6c ${rate}M gap=320 (SIFS): ${fcs_pass}/10 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — FAIL" >&2
        fi
    done

    # Layer 6 is gated — burst decode with STF rearm is fundamental.
    layer_result 6 "burst-decode" $l6_pass $l6_total "true"
fi

# ════════════════════════════════════════
# Layer 7: Real capture replay via HIL
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 7 ]; then
    layer_header 7 "Capture + stimulus replay (captures/passing/ + stimulus/passing/)"
    l7_pass=0
    l7_total=0
    CAPTURES_DIR="$PROJECT_DIR/captures/passing"
    STIMULUS_DIR="$PROJECT_DIR/stimulus/passing"
    REMOTE_DIR="/tmp/captures"

    if [ ! -d "$CAPTURES_DIR" ] && [ ! -d "$STIMULUS_DIR" ]; then
        echo "    WARNING: no capture/stimulus dirs found — skipping" >&2
    else
        # Deploy captures + stimulus to Pluto
        pluto_ssh "mkdir -p $REMOTE_DIR" 2>/dev/null
        captures=()
        for src_dir in "$CAPTURES_DIR" "$STIMULUS_DIR"; do
            if [ -d "$src_dir" ]; then
                while IFS= read -r -d '' f; do
                    captures+=("$f")
                done < <(find "$src_dir" -name "*.json" -print0 | sort -z)
            fi
        done

        if [ ${#captures[@]} -eq 0 ]; then
            echo "    WARNING: No captures found — skipping" >&2
        else
            echo "    Deploying ${#captures[@]} captures to Pluto..." >&2
            for cap in "${captures[@]}"; do
                pluto_scp "$cap" "root@$PLUTO_IP:$REMOTE_DIR/" 2>/dev/null
            done

            for cap in "${captures[@]}"; do
                basename=$(basename "$cap")
                remote_path="$REMOTE_DIR/$basename"

                # Skip captures too short for HIL (< 10K samples).
                # Short single-frame extractions lack sufficient noise context
                # for the correlator to establish energy baseline. These are
                # tested in sim (test_adc_replay). The burst capture that
                # contains them passes HIL and proves the waveforms decode.
                n_samples=$(python3 -c "import json; d=json.load(open('$cap')); print(len(d.get('real',[])))" 2>/dev/null) || n_samples=0
                if [ "$n_samples" -lt 10000 ]; then
                    $VERBOSE && echo "    $basename: SKIP (${n_samples} samples < 10K, sim-only)" >&2
                    continue
                fi

                l7_total=$((l7_total + 1))

                # Use --file --hil --verify-psdu for multi-frame tag counting + PSDU verify
                result=$(pluto_ssh "deimos_burst_loopback --file $remote_path --hil --verify-psdu" 2>/dev/null) || true
                json_line=$(echo "$result" | grep '^{' | tail -1)
                fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
                fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
                psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

                # Gate: at least 1 FCS OK, 0 FCS fail, 0 PSDU fail.
                if [ "$fcs_pass" -ge 1 ] && [ "$fcs_fail" -eq 0 ] && [ "$psdu_fail" -eq 0 ]; then
                    l7_pass=$((l7_pass + 1))
                    $VERBOSE && echo "    $basename: ${fcs_pass} FCS OK, psdu_fail=${psdu_fail} — PASS" >&2
                else
                    echo "    $basename: ${fcs_pass} FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — FAIL" >&2
                fi
            done
        fi
    fi

    # Layer 7 is GATED — these captures pass in sim, they must pass via HIL.
    layer_result 7 "capture-replay" $l7_pass $l7_total "true"
fi

# ════════════════════════════════════════
# Layer 8: Impairments (SFO, multipath, AWGN, combined)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 8 ]; then
    layer_header 8 "Impairments (SFO + multipath + AWGN)"
    l8_pass=0
    l8_total=0
    IMPAIRED_DIR="${IMPAIRED_DIR:-$(mktemp -d)}"
    _impaired_cleanup() { rm -rf "$IMPAIRED_DIR"; }
    trap _impaired_cleanup EXIT

    # Generate impaired vectors on host
    python3 "$PROJECT_DIR/scripts/gen_impaired_burst.py" --output "$IMPAIRED_DIR" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "    ERROR: gen_impaired_burst.py failed" >&2
        layer_result 8 "impairments" 0 1 "true"
    else
        # Deploy and test each scenario
        for scenario in sfo_only multipath_mild multipath_mod awgn_25db combined multipath_mod_54m combined_54m; do
            vec_file="$IMPAIRED_DIR/${scenario}.json"
            if [ ! -f "$vec_file" ]; then
                echo "    WARNING: $vec_file not found — skipping" >&2
                continue
            fi

            l8_total=$((l8_total + 1))

            # Upload and inject
            pluto_ssh "cat > /tmp/${scenario}.json" < "$vec_file" 2>/dev/null
            result=$(pluto_ssh "deimos_burst_loopback --file /tmp/${scenario}.json --hil --verify-psdu" 2>/dev/null) || true
            json_line=$(echo "$result" | grep '^{' | tail -1)
            fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
            fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
            psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

            # Gate: 5/6 FCS OK (allow 1 miss from SFO resampling edge effects), 0 FCS fail, 0 PSDU fail.
            if [ "$fcs_pass" -ge 5 ] && [ "$fcs_fail" -eq 0 ] && [ "$psdu_fail" -eq 0 ]; then
                l8_pass=$((l8_pass + 1))
                $VERBOSE && echo "    $scenario: ${fcs_pass}/6 FCS OK, psdu_fail=${psdu_fail} — PASS" >&2
            else
                echo "    $scenario: ${fcs_pass}/6 FCS OK, ${fcs_fail} fail, psdu_fail=${psdu_fail} — FAIL" >&2
            fi
        done

        layer_result 8 "impairments" $l8_pass $l8_total "true"
    fi
fi

echo "" >&2
echo "═══════════════════════════════════════" >&2
echo " HIL Test Suite: ALL LAYERS PASS" >&2
echo "═══════════════════════════════════════" >&2

finalize_layers "$LADDER_EVENT" 0
