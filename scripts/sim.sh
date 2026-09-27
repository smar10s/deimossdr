#!/bin/bash
# scripts/sim.sh — Run cocotb FPGA simulation tests
#
# EXPECTED WALL TIME:
#   Full suite (50 test targets): ~4 min (warm builds), ~7 min (cold)
#   --fast mode: skips test_adc_replay (~1 min saved)
#   Single target: <5s (warm), <15s (cold compile)
#
# AGENT TIMEOUT: use 600000 ms (10 min) when invoking from OpenCode
#
# Usage:
#   ./scripts/sim.sh                     # run ALL gate tests (must all pass)
#   ./scripts/sim.sh --fast              # skip slow tests (for iteration)
#   ./scripts/sim.sh test_rx_pipeline    # run one test target
#   ./scripts/sim.sh diag_rx_frontend    # run diagnostics (reports only, never fails)
#   ./scripts/sim.sh --list              # list available targets
#
# TEST vs DIAGNOSTIC distinction:
#   test_*  = Gate tests. MUST pass. Failure = broken code. Exit nonzero on failure.
#   diag_*  = Diagnostic tools. Report results. Never assert. Always exit 0.
#
# The default (no args) runs ONLY test_* targets — the full gate suite.
# Diagnostics are run manually when investigating specific behavior.
#
# Simulator:
#   Always uses Verilator (3-6x faster than Icarus, compiled sim).
#   This wrapper forces verilator; for Icarus diagnostics call
#   `make -C fpga/test SIM=icarus <target>` directly.
#   Verilator builds are cached in sim_build_* dirs; only recompiles on RTL change.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV="$PROJECT_DIR/.venv"
FPGA_TEST="$PROJECT_DIR/fpga/test"

# Verify prerequisites
if ! command -v verilator &>/dev/null; then
    echo "ERROR: verilator not found. Install with: brew install verilator"
    exit 1
fi

if [ ! -f "$VENV/bin/cocotb-config" ]; then
    echo "ERROR: cocotb not installed. Run:"
    echo "  python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt"
    exit 1
fi

export PATH="$VENV/bin:$PATH"
export SIM=verilator

if [ "${1:-}" = "--list" ]; then
    echo "Gate tests (must pass):"
    grep -oE '^\.PHONY: test_[a-z0-9_]+' "$FPGA_TEST/Makefile" | sed 's/\.PHONY: /  /' | sort
    echo ""
    echo "Diagnostics (report only):"
    grep -oE '^\.PHONY: diag_[a-z0-9_]+' "$FPGA_TEST/Makefile" | sed 's/\.PHONY: /  /' | sort
    echo ""
    echo "Slow tests (skipped with --fast):"
    echo "  test_adc_replay (~2 min)"
    exit 0
fi

# --fast mode: skip test_adc_replay (~1 min) for quick iteration
FAST_MODE=0
if [ "${1:-}" = "--fast" ]; then
    FAST_MODE=1
    shift
fi
# Slow targets (dominated by long sim time, not compilation)
SLOW_TARGETS="test_adc_replay"

TARGET="${1:-}"

if [ -z "$TARGET" ]; then
    START_TIME=$(date +%s)
    echo "Running all gate tests (test_* targets)..."
    if [ $FAST_MODE -eq 1 ]; then
        echo "FAST MODE: skipping slow tests ($SLOW_TARGETS)"
    fi
    echo "Diagnostics (diag_*) are NOT included — run them explicitly."
    echo ""

    # Layer 0 pre-gate: the sim pipeline view must be exactly what the block
    # design (the single source of truth, D19) generates. ~30s.
    echo "=== gen_sim_pipeline --check ==="
    if ! python3 "$SCRIPT_DIR/gen_sim_pipeline.py" --check \
            --bd "$PROJECT_DIR/fpga/project/system_bd.tcl" \
            --out-dir "$PROJECT_DIR/fpga/rtl"; then
        echo "Generated sim view is STALE vs system_bd.tcl (see D19)."
        echo "Regenerate: python3 scripts/gen_sim_pipeline.py --bd fpga/project/system_bd.tcl --out-dir fpga/rtl"
        exit 1
    fi
    echo ""

    # Group targets by SIM_BUILD to maximize cache reuse.
    # Tests sharing a build dir run consecutively (compile once, run many).
    # Order: fast targets first, heavy targets last (fail-fast behavior).
    TARGETS=($(grep -oE '^\.PHONY: test_[a-z0-9_]+' "$FPGA_TEST/Makefile" | sed 's/\.PHONY: //' | sort))
    PASSED=0
    FAILED=0
    SKIPPED=0
    FAILED_LIST=()

    for t in "${TARGETS[@]}"; do
        # Skip slow targets in fast mode
        if [ $FAST_MODE -eq 1 ]; then
            for slow in $SLOW_TARGETS; do
                if [ "$t" = "$slow" ]; then
                    SKIPPED=$((SKIPPED + 1))
                    continue 2
                fi
            done
        fi

        echo "=== $t ==="
        T_START=$(date +%s)
        LOGFILE=$(mktemp)
        if make -C "$FPGA_TEST" "$t" >"$LOGFILE" 2>&1; then
            PASSED=$((PASSED + 1))
        else
            FAILED=$((FAILED + 1))
            FAILED_LIST+=("$t")
        fi
        tail -5 "$LOGFILE"
        rm -f "$LOGFILE"
        T_END=$(date +%s)
        echo "  (${t}: $((T_END - T_START))s)"

        # Save per-target coverage data (Verilator writes coverage.dat to CWD)
        if [ -f "$FPGA_TEST/coverage.dat" ]; then
            mv "$FPGA_TEST/coverage.dat" "$FPGA_TEST/coverage_${t}.dat" 2>/dev/null || true
        fi
        echo ""
    done

    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))

    echo "========================================"
    echo "GATE SUMMARY: $PASSED passed, $FAILED failed, $SKIPPED skipped"
    echo "WALL TIME: ${ELAPSED}s ($(( ELAPSED / 60 ))m $(( ELAPSED % 60 ))s)"
    if [ $FAILED -gt 0 ]; then
        echo "FAILED: ${FAILED_LIST[*]}"
        echo ""
        echo "Gate tests MUST pass before any commit. Fix the code or revert."
        exit 1
    fi
    if [ $SKIPPED -gt 0 ]; then
        echo ""
        echo "NOTE: $SKIPPED slow tests skipped (--fast mode)."
        echo "Run without --fast before merge to validate full suite."
    fi
    echo "ALL GATE TESTS PASS"

    # Coverage report (only if VERILATOR_COVERAGE=1 was passed to enable instrumentation)
    if [ "${VERILATOR_COVERAGE:-0}" = "1" ] && command -v verilator_coverage &>/dev/null; then
        echo ""
        echo "=== Coverage Report ==="
        COVERAGE_FILES=$(find "$FPGA_TEST" -maxdepth 1 -name "coverage_*.dat" 2>/dev/null)
        if [ -n "$COVERAGE_FILES" ]; then
            mkdir -p "$FPGA_TEST/coverage"
            verilator_coverage --annotate "$FPGA_TEST/coverage" $COVERAGE_FILES 2>/dev/null || true
            verilator_coverage --rank $COVERAGE_FILES 2>/dev/null | tail -10 || true
            echo "Annotated source: fpga/test/coverage/"
        fi
    fi

    exit 0
fi

# Running a specific target
echo "Running: $TARGET"

# Diagnostics never fail the script (they report, not assert)
if [[ "$TARGET" == diag_* ]]; then
    make -C "$FPGA_TEST" "$TARGET" 2>&1 || true
    echo ""
    echo "(Diagnostic complete — results above are informational only)"
    exit 0
fi

# Test targets: propagate exit code (failure = broken code)
exec make -C "$FPGA_TEST" "$TARGET"
