#!/bin/bash
# scripts/lib.sh — shared test-infrastructure helpers
#
# Sourced (not executed) by session/gate/test scripts. The caller must
# define PROJECT_DIR before sourcing. Self-test: bash scripts/lib.sh selftest
#
# Provides:
#   pluto_ssh / pluto_scp     — remote command/file helpers (PLUTO_IP/PLUTO_PASS)
#   preflight                 — sshpass + Pluto reachability (exit 2 on infra)
#   retry_ssh                 — remote command with transport retries
#   identity                  — sets ID_BRANCH ID_COMMIT ID_FINGERPRINT ID_BITSTREAM
#   hwlog <event> <extra>     — append uniform entry to logs/hardware.jsonl
#   json_get <json> <path>    — python3 JSON path extractor (e.g. "rates.0.pass")
#   json_rate_pass <json> <r> — pass count for rate r in a {"rates":[...]} doc
#   result / result_summary   — PASS/WARN/FAIL counter framework
#   layer_header / layer_result / finalize_layers — ladder suite framework
#   parse_fcs_ok              — snap-buffer FCS word parser (HIL)
#   loopback_gate             — gate-rate + report-rate loopback runner
#   gate_baseline_for_bitstream — regression-ratchet baseline from hardware.jsonl

: "${PLUTO_IP:=192.168.2.1}"
: "${PLUTO_PASS:=analog}"

SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5)
SSH_CMD=(sshpass -p "$PLUTO_PASS" ssh "${SSH_OPTS[@]}" "root@$PLUTO_IP")
SCP_CMD=(sshpass -p "$PLUTO_PASS" scp -O "${SSH_OPTS[@]}")

pluto_ssh() { "${SSH_CMD[@]}" "$@"; }
pluto_scp() { "${SCP_CMD[@]}" "$@"; }

preflight() {
    command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed" >&2; exit 2; }
    pluto_ssh "true" 2>/dev/null || { echo "ERROR: Cannot reach Pluto at $PLUTO_IP" >&2; exit 2; }
}

# retry_ssh <attempts> <remote cmd...> — retries on SSH-level failure only.
# Prints stdout on first success. Returns 0 on success, 1 if all failed.
retry_ssh() {
    local max=$1 output attempt
    shift
    for ((attempt=1; attempt<=max; attempt++)); do
        output=$(pluto_ssh "$@" 2>/dev/null) && { printf '%s' "$output"; return 0; }
        sleep 1
    done
    return 1
}

identity() {
    ID_BRANCH=$(git -C "${PROJECT_DIR:-.}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
    ID_COMMIT=$(git -C "${PROJECT_DIR:-.}" rev-parse --short HEAD 2>/dev/null || echo unknown)
    ID_FINGERPRINT=$( (cat "$PROJECT_DIR/build/fpga/fingerprint" 2>/dev/null || echo unknown) | tr '[:upper:]' '[:lower:]' | sed 's/0[xX]//' | tr -d '[:space:]' )
    ID_BITSTREAM=$(cat "$PROJECT_DIR/build/fpga/bitstream.sha256" 2>/dev/null | tr -d '[:space:]' || true)
    [ -n "$ID_BITSTREAM" ] && [ "$ID_BITSTREAM" != "unavailable" ] || ID_BITSTREAM="unknown"
}

# hwlog <event> <extra-json-fields> — append uniform entry to hardware.jsonl.
# hardware.jsonl is append-only, machine-written; agents never edit it.
hwlog() {
    local event=$1 extra=${2:-} ts entry
    identity
    ts=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    entry="{\"timestamp\":\"${ts}\",\"event\":\"${event}\",\"branch\":\"${ID_BRANCH}\",\"commit\":\"${ID_COMMIT}\",\"fingerprint\":\"${ID_FINGERPRINT}\",\"bitstream\":\"${ID_BITSTREAM}\""
    [ -n "$extra" ] && entry="${entry},${extra}"
    entry="${entry}}"
    mkdir -p "$PROJECT_DIR/logs"
    echo "$entry" >> "$PROJECT_DIR/logs/hardware.jsonl"
}

# json_get <json-string> <dotted-path> — dict keys, list indices.
# Prints nothing and exits 1 when the path is absent, so both the
# `|| var=0` default idiom and `-n` tests work.
json_get() {
    echo "$1" | python3 -c '
import json,sys
d=json.load(sys.stdin)
for k in sys.argv[1].split("."):
    if isinstance(d,list): d=d[int(k)]
    elif isinstance(d,dict) and k in d: d=d[k]
    else: sys.exit(1)
print(d)' "$2" 2>/dev/null
}

# json_rate_pass <json-string> <rate> — pass count from {"rates":[...]} docs
# (pluto_loopback, deimos_fabric_loopback both use this shape).
json_rate_pass() {
    echo "$1" | python3 -c '
import json,sys
d=json.load(sys.stdin)
for r in d.get("rates",[]):
    if r.get("rate")==int(sys.argv[1]): print(r.get("pass",0)); break' "$2" 2>/dev/null
}

# ---- PASS/WARN/FAIL counter framework ----
GATE_PASS=0; GATE_WARN=0; GATE_FAIL=0
result() {
    local status=$1 msg=$2
    if [ "$status" = "PASS" ]; then echo "  [PASS] $msg"; GATE_PASS=$((GATE_PASS+1))
    elif [ "$status" = "FAIL" ]; then echo "  [FAIL] $msg"; GATE_FAIL=$((GATE_FAIL+1))
    else echo "  [WARN] $msg"; GATE_WARN=$((GATE_WARN+1)); fi
}
result_summary() {
    echo ""
    echo "========================================"
    echo "SUMMARY"
    echo "  PASS: $GATE_PASS"
    echo "  WARN: $GATE_WARN"
    echo "  FAIL: $GATE_FAIL"
    echo "========================================"
    echo ""
}

# ---- Ladder suite framework (hil_test.sh / loopback_test.sh) ----
# Caller sets LADDER_EVENT (hwlog event name), MAX_LAYER, JSON_OUTPUT.
JSON_LAYERS="["
layer_header() {
    local num=$1 name=$2
    echo "" >&2
    echo "═══ Layer $num: $name ═══" >&2
}
layer_result() {
    local num=$1 name=$2 pass=$3 total=$4 gate=$5
    local status="PASS"
    if [ "$pass" -ne "$total" ]; then
        if [ "$gate" = "true" ]; then status="FAIL"; else status="INFO"; fi
    fi
    echo "  Layer $num: $pass/$total $status" >&2
    JSON_LAYERS="$JSON_LAYERS{\"layer\":$num,\"name\":\"$name\",\"pass\":$pass,\"total\":$total,\"status\":\"$status\"},"
    if [ "$status" = "FAIL" ]; then
        echo "" >&2
        echo "GATE FAILURE at layer $num ($name)" >&2
        echo "Fix this layer before proceeding to higher layers." >&2
        finalize_layers "$LADDER_EVENT" 1
    fi
}
finalize_layers() {
    local event=$1 exit_code=${2:-0} result_str
    JSON_LAYERS="${JSON_LAYERS%,}]"
    if [ "$exit_code" -eq 0 ]; then result_str="PASS"; else result_str="FAIL"; fi
    hwlog "$event" "\"layers_run\":${MAX_LAYER:-0},\"result\":\"${result_str}\",\"layers\":${JSON_LAYERS}"
    echo "  Results logged to: logs/hardware.jsonl" >&2
    if ${JSON_OUTPUT:-false}; then
        echo "{\"layers\":$JSON_LAYERS,\"exit\":$exit_code}"
    fi
    exit "$exit_code"
}

# ---- HIL snap-buffer FCS parser ----
parse_fcs_ok() {
    local fcs_ok=0 found=0 word state
    while IFS= read -r line; do
        [[ "$line" =~ ^[0-9a-fA-F]{8}$ ]] || continue
        word=$((16#$line))
        state=$(( (word >> 27) & 0x0F ))
        # decode_engine states: S_TAG_OUT=12, S_DONE=13
        if [ "$state" -eq 12 ] || [ "$state" -eq 13 ]; then
            fcs_ok=$(( (word >> 26) & 0x01 ))
            found=1
        fi
    done
    if [ "$found" -eq 1 ]; then echo "$fcs_ok"; else echo "0"; fi
}

# ---- Cable loopback gate (session_start + session_end) ----
GATE_RATES="6 9 12 18 24 36"
REPORT_RATES="48 54"

# loopback_gate <trials> <report_trials> [baseline-json]
# Gate rates: absolute floor >=80% (PASS at trials/trials, WARN at >=80%).
# With baseline (ratchet, session_end): >=baseline PASS, -1..-2 WARN, worse FAIL.
# Report rates 48/54: logged, never gated (known EVM margin, D16).
# Fills LB_LOG with the per-rate JSON fragment for the hwlog entry.
loopback_gate() {
    local trials=$1 report_trials=$2 baseline=${3:-} rate json pass pct base_pass diff
    LB_LOG=""
    for rate in $GATE_RATES; do
        json=$(pluto_ssh "deimos_fabric_loopback -r $rate -n $trials" 2>/dev/null; true)
        [ -z "$json" ] && json='{}'
        pass=$(json_rate_pass "$json" "$rate"); pass=${pass:-0}
        pct=$((pass * 100 / trials))
        base_pass=$(json_get "$baseline" "$rate")
        if [ "$pass" -lt $(( trials * 80 / 100 )) ]; then
            result "FAIL" "Rate ${rate}M: ${pass}/${trials} (${pct}%) — below 80% floor"
        elif [ -n "$base_pass" ] && [ "$pass" -lt "$base_pass" ]; then
            diff=$((base_pass - pass))
            if [ "$diff" -gt 2 ]; then
                result "FAIL" "Rate ${rate}M: ${pass}/${trials} REGRESSED (baseline ${base_pass}/${trials}, -${diff})"
            else
                result "WARN" "Rate ${rate}M: ${pass}/${trials} (baseline ${base_pass}/${trials}, -${diff})"
            fi
        elif [ "$pass" -eq "$trials" ]; then
            result "PASS" "Rate ${rate}M: ${pass}/${trials} (${pct}%)"
        else
            result "WARN" "Rate ${rate}M: ${pass}/${trials} (${pct}%)"
        fi
        LB_LOG="${LB_LOG}\"${rate}\":${pass},"
    done
    echo ""
    echo "  Report-only rates (48/54 — known EVM margin, not gated, $report_trials trials):"
    for rate in $REPORT_RATES; do
        json=$(pluto_ssh "deimos_fabric_loopback -r $rate -n $report_trials" 2>/dev/null; true)
        [ -z "$json" ] && json='{}'
        pass=$(json_rate_pass "$json" "$rate"); pass=${pass:-0}
        pct=$((pass * 100 / report_trials))
        echo "  [INFO] Rate ${rate}M: ${pass}/${report_trials} (${pct}%)"
        LB_LOG="${LB_LOG}\"${rate}\":${pass},"
    done
    LB_LOG="${LB_LOG%,}"
}

# gate_baseline_for_bitstream [bsha] — regression-ratchet baseline: the
# loopback results of the most recent non-blocked session_end entry in
# logs/hardware.jsonl for this bitstream sha. Prints {} when none exists
# (first green run of a new bitstream → absolute floor only).
gate_baseline_for_bitstream() {
    local bsha=${1:-}
    if [ -z "$bsha" ]; then identity; bsha=$ID_BITSTREAM; fi
    python3 - "$PROJECT_DIR/logs/hardware.jsonl" "$bsha" <<'PY'
import json,sys
path, bsha = sys.argv[1], sys.argv[2]
best = None
try:
    with open(path) as f:
        for line in f:
            try: d = json.loads(line)
            except ValueError: continue
            if d.get("event") != "session_end": continue
            if d.get("bitstream") != bsha: continue
            if d.get("blocked") in (True, "true"): continue
            lb = d.get("loopback")
            if isinstance(lb, dict): best = lb
except FileNotFoundError:
    pass
print(json.dumps(best if best else {}))
PY
}

# ---- Self-test (pure helpers only; no hardware access) ----
selftest() {
    local fails=0 out tmpdir
    out=$(json_get '{"rates":[{"pass":20}]}' "rates.0.pass")
    [ "$out" = "20" ] || { echo "FAIL json_get dict path"; fails=$((fails+1)); }
    out=$(json_get '{"a":{"b":[1,2]}}' "a.b.1")
    [ "$out" = "2" ] || { echo "FAIL json_get list index"; fails=$((fails+1)); }
    out=$(json_rate_pass '{"rates":[{"rate":6,"pass":19},{"rate":9,"pass":7}]}' 6)
    [ "$out" = "19" ] || { echo "FAIL json_rate_pass"; fails=$((fails+1)); }
    out=$(printf '6c000000\n' | parse_fcs_ok)
    [ "$out" = "1" ] || { echo "FAIL parse_fcs_ok tag word"; fails=$((fails+1)); }
    out=$(printf '00000000\n' | parse_fcs_ok)
    [ "$out" = "0" ] || { echo "FAIL parse_fcs_ok non-tag word"; fails=$((fails+1)); }
    tmpdir=$(mktemp -d)
    mkdir -p "$tmpdir/logs"
    printf '%s\n' \
      '{"event":"session_end","bitstream":"abc","blocked":false,"loopback":{"6":20}}' \
      '{"event":"session_end","bitstream":"abc","blocked":true,"loopback":{"6":3}}' \
      '{"event":"session_start","bitstream":"abc","loopback":{"6":20}}' \
      '{"event":"session_end","bitstream":"xyz","blocked":false,"loopback":{"6":5}}' \
      > "$tmpdir/logs/hardware.jsonl"
    out=$(PROJECT_DIR="$tmpdir" bash -c 'source "$1"; gate_baseline_for_bitstream abc' _ "$PWD/scripts/lib.sh")
    [ "$out" = '{"6": 20}' ] || { echo "FAIL baseline: blocked/session_start/other-bitsha skipped (got: $out)"; fails=$((fails+1)); }
    out=$(PROJECT_DIR="$tmpdir" bash -c 'source "$1"; gate_baseline_for_bitstream nope' _ "$PWD/scripts/lib.sh")
    [ "$out" = '{}' ] || { echo "FAIL baseline: no match returns {}"; fails=$((fails+1)); }
    rm -rf "$tmpdir"
    if [ "$fails" -eq 0 ]; then echo "lib.sh selftest: ALL PASS"; else echo "lib.sh selftest: $fails FAIL"; exit 1; fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    case "${1:-}" in
        selftest) selftest ;;
        *) echo "usage: bash scripts/lib.sh selftest" >&2; exit 2 ;;
    esac
fi
