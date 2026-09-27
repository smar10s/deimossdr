#!/bin/bash
# scripts/loopback_test.sh — Progressive cable loopback test suite
#
# Layers:
#   1. Single-frame RF decode (10 trials/rate, fabric FCS)
#   2. Burst detection (20 frames, tag FIFO coherence)
#   3. Rate mixing (20 mixed-rate frames in burst)
#   4. Capture replay through cable (captures/passing/ via DAC→cable→ADC)
#   5. Tight spacing / SIFS (320-sample gap, rearm + pending trigger)
#   6. PSDU verify (fabric BRAM readback)
#   7. EAPOL burst (SIFS timing + PSDU verify)
#
# Gate rates: 6/9/12/18/24/36
# Report-only: 48/54 (logged, never fail the script)
#
# Usage:
#   ./scripts/loopback_test.sh              # all layers
#   ./scripts/loopback_test.sh -l 3         # layers 1-3 only
#   ./scripts/loopback_test.sh -r 24        # single rate (layer 1 only)
#   ./scripts/loopback_test.sh -v           # verbose
#
# Exit codes:
#   0 = all gate layers pass
#   1 = gate layer failure
#   2 = infrastructure error

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/lib.sh"
LADDER_EVENT="loopback_test"

# Override lib.sh GATE_RATES for loopback_test — it has its own set
GATE_RATES="6 9 12 18 24 36"
REPORT_RATES="48 54"
MAX_LAYER=7
SINGLE_RATE=""
VERBOSE=false
JSON_OUTPUT=false

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "  -l layer   Run layers 1 through N (default: $MAX_LAYER)"
    echo "  -r rate    Single rate focus (layer 1 only)"
    echo "  --json     JSON summary to stdout"
    echo "  -v         Verbose"
    echo "  -h         Help"
    echo ""
    echo "Layers:"
    echo "  1  Single-frame RF decode (10 trials/rate)"
    echo "  2  Burst detection (20 frames, tag FIFO)"
    echo "  3  Rate mixing (20 mixed-rate burst)"
    echo "  4  Capture replay through cable"
    echo "  5  Tight spacing / SIFS (320-sample gap)"
    echo "  6  PSDU verify (fabric BRAM readback)"
    echo "  7  EAPOL burst (SIFS timing + PSDU verify)"
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

if ! pluto_ssh "which deimos_fabric_loopback && which deimos_burst_loopback" >/dev/null 2>&1; then
    echo "ERROR: loopback tools not found on Pluto (run 'make deploy')" >&2
    exit 2
fi

# ---- Result tracking (counters; framework from lib.sh) ----

# ---- Start ----
echo "═══════════════════════════════════════" >&2
echo " Loopback Test Suite — $(date +%H:%M:%S)" >&2
echo " Layers: 1-$MAX_LAYER" >&2
echo " Gate rates: $GATE_RATES" >&2
echo "═══════════════════════════════════════" >&2

# ════════════════════════════════════════
# Layer 1: Single-frame RF decode
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 1 ]; then
    layer_header 1 "Single-frame RF decode (10 trials/rate)"
    l1_pass=0
    l1_total=0
    l1_gate_ok=true

    for rate in $GATE_RATES; do
        l1_total=$((l1_total + 1))
        result=$(pluto_ssh "deimos_fabric_loopback -r $rate -n 10" 2>/dev/null) || true
        rate_pass=$(json_rate_pass "$result" "$rate") || rate_pass=0

        # Threshold: 9/10 for all gate rates
        thresh=9

        if [ "$rate_pass" -ge "$thresh" ]; then
            l1_pass=$((l1_pass + 1))
            $VERBOSE && echo "    ${rate}M: ${rate_pass}/10 PASS" >&2
        else
            echo "    ${rate}M: ${rate_pass}/10 FAIL (need ${thresh}/10)" >&2
            l1_gate_ok=false
        fi
        sleep 0.5
    done

    if $l1_gate_ok; then
        layer_result 1 "single-frame" $l1_pass $l1_total "false"
    else
        layer_result 1 "single-frame" $l1_pass $l1_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 2: Burst detection (20 frames, 2000-sample gap)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 2 ]; then
    layer_header 2 "Burst detection (20 frames, 250μs gap)"
    l2_pass=0
    l2_total=0
    l2_gate_ok=true

    for rate in 6 24; do
        for ((run=1; run<=3; run++)); do
            l2_total=$((l2_total + 1))
            result=$(pluto_ssh "deimos_burst_loopback -n 20 -r $rate -g 5000" 2>/dev/null) || true
            # Extract JSON line (last line with {)
            json_line=$(echo "$result" | grep '^{' | tail -1)
            fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
            fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0

            # Threshold: fcs_pass >= 18 AND fcs_fail == 0
            if [ "$fcs_pass" -ge 18 ] && [ "$fcs_fail" -eq 0 ]; then
                l2_pass=$((l2_pass + 1))
                $VERBOSE && echo "    ${rate}M run $run: ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — PASS" >&2
            else
                echo "    ${rate}M run $run: ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — FAIL" >&2
                l2_gate_ok=false
            fi
        done
    done

    if $l2_gate_ok; then
        layer_result 2 "burst-detect" $l2_pass $l2_total "false"
    else
        layer_result 2 "burst-detect" $l2_pass $l2_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 3: Rate mixing (20 mixed-rate frames)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 3 ]; then
    layer_header 3 "Rate mixing (20 mixed-rate frames)"
    l3_pass=0
    l3_total=0
    l3_gate_ok=true

    for ((run=1; run<=3; run++)); do
        l3_total=$((l3_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback -n 20 --mix-rates -g 5000" 2>/dev/null) || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0

        # Threshold: fcs_pass >= 16 AND fcs_fail <= 2
        # Rate 36 has ~5%/frame FCS fail in burst mode through cable (DAC/PA
        # sustained TX issue — single-frame is 50/50, burst degrades). Allow
        # 1-2 fails per 20-frame mixed burst that includes rate 36.
        if [ "$fcs_pass" -ge 16 ] && [ "$fcs_fail" -le 2 ]; then
            l3_pass=$((l3_pass + 1))
            $VERBOSE && echo "    Run $run: ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — PASS" >&2
        else
            echo "    Run $run: ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — FAIL" >&2
            l3_gate_ok=false
        fi
    done

    if $l3_gate_ok; then
        layer_result 3 "rate-mixing" $l3_pass $l3_total "false"
    else
        layer_result 3 "rate-mixing" $l3_pass $l3_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 4: Capture replay through cable
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 4 ]; then
    layer_header 4 "Capture replay through cable"
    l4_pass=0
    l4_total=0
    l4_gate_ok=true
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
                basename_cap=$(basename "$cap")
                l4_total=$((l4_total + 1))
                remote_path="$REMOTE_DIR/$basename_cap"

                result=$(pluto_ssh "deimos_burst_loopback --file $remote_path" 2>/dev/null) || true
                json_line=$(echo "$result" | grep '^{' | tail -1)
                fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
                stf_alive=$(json_get "$json_line" "stf_alive") || stf_alive="False"

                # Gate: cable-origin captures must decode (fcs_pass > 0)
                # Report-only: OTA/EAPOL captures (known requantization
                # issues) and 48/54M cable captures (D16/D17: analog EVM
                # limit — never gate at these rates).
                is_cable=false
                [[ "$basename_cap" == *_cable_* ]] && is_cable=true
                is_report_rate=false
                [[ "$basename_cap" == 48m_* || "$basename_cap" == 54m_* ]] && is_report_rate=true

                if [ "$fcs_pass" -gt 0 ]; then
                    l4_pass=$((l4_pass + 1))
                    $VERBOSE && echo "    $basename_cap: ${fcs_pass} FCS OK — PASS" >&2
                elif $is_cable && ! $is_report_rate; then
                    echo "    $basename_cap: ${fcs_pass} FCS OK — FAIL (cable capture must decode)" >&2
                    l4_gate_ok=false
                elif $is_report_rate; then
                    echo "    $basename_cap: ${fcs_pass} FCS OK (report-only, 48/54M EVM limit D16)" >&2
                else
                    echo "    $basename_cap: ${fcs_pass} FCS OK (report-only, OTA capture)" >&2
                fi
            done
        fi
    fi

    if $l4_gate_ok; then
        layer_result 4 "capture-replay" $l4_pass $l4_total "false"
    else
        layer_result 4 "capture-replay" $l4_pass $l4_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 5: Tight spacing / SIFS (320-sample gap)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 5 ]; then
    layer_header 5 "Tight spacing / SIFS (320-sample gap)"
    l5_pass=0
    l5_total=0
    l5_gate_ok=true

    for ((run=1; run<=3; run++)); do
        l5_total=$((l5_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback -n 20 -r 6 -g 320" 2>/dev/null) || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0

        # Threshold: fcs_pass >= 15 AND fcs_fail == 0
        if [ "$fcs_pass" -ge 15 ] && [ "$fcs_fail" -eq 0 ]; then
            l5_pass=$((l5_pass + 1))
            $VERBOSE && echo "    Run $run (rate 6): ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — PASS" >&2
        else
            echo "    Run $run (rate 6): ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — FAIL" >&2
            l5_gate_ok=false
        fi
    done

    # Also test rate 24 at SIFS timing
    for ((run=1; run<=3; run++)); do
        l5_total=$((l5_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback -n 20 -r 24 -g 320" 2>/dev/null) || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0

        # Threshold: fcs_pass >= 15 AND fcs_fail == 0
        if [ "$fcs_pass" -ge 15 ] && [ "$fcs_fail" -eq 0 ]; then
            l5_pass=$((l5_pass + 1))
            $VERBOSE && echo "    Run $run (rate 24): ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — PASS" >&2
        else
            echo "    Run $run (rate 24): ${fcs_pass}/20 FCS OK, ${fcs_fail} fail — FAIL" >&2
            l5_gate_ok=false
        fi
    done

    if $l5_gate_ok; then
        layer_result 5 "sifs-timing" $l5_pass $l5_total "false"
    else
        layer_result 5 "sifs-timing" $l5_pass $l5_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 6: PSDU verify (fabric BRAM readback)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 6 ]; then
    layer_header 6 "PSDU verify (fabric BRAM readback)"
    l6_pass=0
    l6_total=0
    l6_gate_ok=true

    for ((run=1; run<=3; run++)); do
        l6_total=$((l6_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback -n 20 --verify-psdu" 2>/dev/null) || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
        psdu_ok=$(json_get "$json_line" "psdu_ok") || psdu_ok=0
        psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

        # Gate: fcs_pass >= 18, psdu_fail == 0, fcs_fail == 0
        if [ "$fcs_pass" -ge 18 ] && [ "$psdu_fail" -eq 0 ] && [ "$fcs_fail" -eq 0 ]; then
            l6_pass=$((l6_pass + 1))
            $VERBOSE && echo "    Run $run: FCS ${fcs_pass}/20, PSDU ${psdu_ok} ok / ${psdu_fail} fail — PASS" >&2
        else
            echo "    Run $run: FCS ${fcs_pass}/20, PSDU ${psdu_ok} ok / ${psdu_fail} fail, fcs_fail=${fcs_fail} — FAIL" >&2
            l6_gate_ok=false
        fi
    done

    if $l6_gate_ok; then
        layer_result 6 "psdu-verify" $l6_pass $l6_total "false"
    else
        layer_result 6 "psdu-verify" $l6_pass $l6_total "true"
    fi
fi

# ════════════════════════════════════════
# Layer 7: EAPOL burst (SIFS timing + PSDU verify)
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 7 ]; then
    layer_header 7 "EAPOL burst (SIFS timing + PSDU verify)"
    l7_pass=0
    l7_total=0
    l7_gate_ok=true

    for ((run=1; run<=3; run++)); do
        l7_total=$((l7_total + 1))
        result=$(pluto_ssh "deimos_burst_loopback -n 20 -g 320 --eapol --verify-psdu" 2>/dev/null) || true
        json_line=$(echo "$result" | grep '^{' | tail -1)
        fcs_pass=$(json_get "$json_line" "fcs_pass") || fcs_pass=0
        fcs_fail=$(json_get "$json_line" "fcs_fail") || fcs_fail=0
        eapol_matched=$(json_get "$json_line" "eapol_matched") || eapol_matched=0
        psdu_ok=$(json_get "$json_line" "psdu_ok") || psdu_ok=0
        psdu_fail=$(json_get "$json_line" "psdu_fail") || psdu_fail=0

        # Gate: fcs_pass >= 15, eapol_matched >= 3, psdu_fail == 0, fcs_fail == 0
        if [ "$fcs_pass" -ge 15 ] && [ "$eapol_matched" -ge 3 ] && [ "$psdu_fail" -eq 0 ] && [ "$fcs_fail" -eq 0 ]; then
            l7_pass=$((l7_pass + 1))
            $VERBOSE && echo "    Run $run: FCS ${fcs_pass}/20, EAPOL ${eapol_matched}/4, PSDU ${psdu_ok} ok / ${psdu_fail} fail — PASS" >&2
        else
            echo "    Run $run: FCS ${fcs_pass}/20, EAPOL ${eapol_matched}/4, PSDU ${psdu_ok} ok / ${psdu_fail} fail, fcs_fail=${fcs_fail} — FAIL" >&2
            l7_gate_ok=false
        fi
    done

    if $l7_gate_ok; then
        layer_result 7 "eapol-psdu-burst" $l7_pass $l7_total "false"
    else
        layer_result 7 "eapol-psdu-burst" $l7_pass $l7_total "true"
    fi
fi

# ════════════════════════════════════════
# Report-only: rates 48/54
# ════════════════════════════════════════

if [ "$MAX_LAYER" -ge 1 ] && [ -z "$SINGLE_RATE" ]; then
    echo "" >&2
    echo "── Report-only rates (48/54) ──" >&2

    # LO PLL self-heal (2026-08-19 evidence):
    # Rates 48/54 sit at the analog EVM cliff (D11/D16): 64-QAM 3/4 needs
    # < -25 dB EVM, the DAC→cable→ADC path delivers -18 to -22 dB. The
    # TX/RX LO PLL at 5.745 GHz (ch149) can land in a marginal state
    # (spur / marginal lock) that survives repeated radio re-init at the
    # SAME frequency. Observed 2026-08-19: 54M fabric loopback collapsed
    # to 0-4/20 for ~40 min across many tool invocations (loopback_test
    # and standalone), while ARM decode stayed 19/20, fabric 48M 20/20,
    # and HIL 54M passed — i.e. RF-level, 54M-only, persistent state,
    # not an RTL problem. One retune to ch36 and back to ch149 restored
    # 20/20 immediately. The burst layers above run ~30 radio re-init
    # cycles, so the report rates are the most exposed to this state.
    # Do one away-trip retune (single trial, full radio re-init) before
    # measuring so a marginal LO state is not misread as a regression.
    # The trip back to ch149 happens implicitly: each report-rate run
    # re-inits the radio at its own channel (default 149).
    SELFHEAL_CHANNEL=36
    echo "  [INFO] LO retune self-heal: ch149 → ch${SELFHEAL_CHANNEL} (1 trial, then report rates re-init at ch149)" >&2
    pluto_ssh "deimos_fabric_loopback -c $SELFHEAL_CHANNEL -r 6 -n 1" >/dev/null 2>&1 || true

    for rate in $REPORT_RATES; do
        result=$(pluto_ssh "deimos_fabric_loopback -r $rate -n 20" 2>/dev/null) || true
        rate_pass=$(json_rate_pass "$result" "$rate") || rate_pass=0
        echo "    ${rate}M: ${rate_pass}/20 (report-only)" >&2
    done
fi

echo "" >&2
echo "═══════════════════════════════════════" >&2
echo " Loopback Test Suite: ALL LAYERS PASS" >&2
echo "═══════════════════════════════════════" >&2

finalize_layers "$LADDER_EVENT" 0
