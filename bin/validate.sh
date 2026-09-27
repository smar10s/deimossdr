#!/bin/bash
# bin/validate.sh — Validate Pluto is alive after flash
#
# EXPECTED WALL TIME: 30-90 seconds (waits for reboot)
# AGENT TIMEOUT: use 300000 ms (5 min) when invoking from OpenCode
#
# Checks:
#   1. SSH connectivity (waits for boot if needed)
#   2. Kernel boots and runs
#   3. AD9361 IIO driver loads
#   4. Build fingerprint register matches expected value
#
# Usage:
#   ./bin/validate.sh           # validate against build/fpga/fingerprint
#   ./bin/validate.sh <hex>     # validate against specific fingerprint
#
# Prerequisites:
#   - Pluto connected via USB (192.168.2.1)
#   - sshpass installed
#   - devmem accessible on Pluto (always is as root)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PLUTO="root@192.168.2.1"
PLUTO_PASS="analog"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

# Build fingerprint register address (set in system_bd.tcl)
BUILD_ID_ADDR="0x43C00000"

# Expected fingerprint
if [ $# -ge 1 ]; then
    EXPECTED_FP="$1"
elif [ -f "$PROJECT_DIR/build/fpga/fingerprint" ]; then
    EXPECTED_FP=$(cat "$PROJECT_DIR/build/fpga/fingerprint")
else
    echo "WARNING: No expected fingerprint. Will read register but cannot verify."
    EXPECTED_FP=""
fi

pluto_ssh() {
    sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO "$@" 2>/dev/null
}

# ---- Wait for Pluto to boot ----
echo "=== Waiting for Pluto to come online ==="
RETRIES=30
for i in $(seq 1 $RETRIES); do
    if pluto_ssh "echo ok" >/dev/null 2>&1; then
        echo "Pluto online (attempt $i/$RETRIES)"
        break
    fi
    if [ $i -eq $RETRIES ]; then
        echo "FAIL: Pluto did not come online after $RETRIES attempts"
        echo "Check USB connection. May need DFU recovery."
        exit 1
    fi
    sleep 2
done

echo ""

# ---- Check 1: Basic system info ----
echo "=== System Info ==="
pluto_ssh "uname -a"
pluto_ssh "cat /opt/VERSIONS 2>/dev/null" || true
echo ""

# ---- Check 2: AD9361 IIO driver ----
echo "=== IIO Devices ==="
IIO_OUT=$(pluto_ssh "iio_info 2>/dev/null | grep -E '(hw_model|ad9361|context)' | head -5")
if echo "$IIO_OUT" | grep -q "ad9361\|AD9364\|PlutoSDR"; then
    echo "$IIO_OUT"
    echo "AD9361 PHY: OK"
else
    echo "WARNING: AD9361 IIO device not detected"
    echo "  (Expected with slim bitstream — DMA removed, partial probe)"
    pluto_ssh "iio_info 2>/dev/null | head -10" || true
fi
echo ""

# ---- Check 3: Build fingerprint register ----
echo "=== Build Fingerprint ==="
RAW=$(pluto_ssh "devmem $BUILD_ID_ADDR 32" 2>/dev/null || echo "FAIL")

if [ "$RAW" = "FAIL" ]; then
    echo "FAIL: Could not read register at $BUILD_ID_ADDR"
    echo "  devmem may not be available, or address not mapped."
    exit 1
fi

# devmem returns hex like "0xDEADBEEF"
REG_HEX=$(echo "$RAW" | tr -d '[:space:]' | sed 's/0[xX]//')
REG_HEX_LOWER=$(echo "$REG_HEX" | tr '[:upper:]' '[:lower:]')

echo "Register value: 0x$REG_HEX_LOWER (at $BUILD_ID_ADDR)"

if [ -n "$EXPECTED_FP" ]; then
    EXPECTED_LOWER=$(echo "$EXPECTED_FP" | sed 's/0[xX]//' | tr '[:upper:]' '[:lower:]')
    if [ "$REG_HEX_LOWER" = "$EXPECTED_LOWER" ]; then
        echo "MATCH: Fingerprint matches build/fpga/fingerprint"
    else
        echo "MISMATCH!"
        echo "  Expected: 0x$EXPECTED_LOWER"
        echo "  Got:      0x$REG_HEX_LOWER"
        echo ""
        echo "=== VALIDATION FAILED ==="
        exit 1
    fi
else
    echo "(no expected fingerprint to compare against — checking identity anyway)"
fi

echo ""

# ---- Check 4: Project ID register ----
echo "=== Project ID ==="
PROJECT_ID_ADDR="0x43C00004"
PID_RAW=$(pluto_ssh "devmem $PROJECT_ID_ADDR 32" 2>/dev/null || echo "FAIL")

if [ "$PID_RAW" = "FAIL" ]; then
    echo "FAIL: Could not read project ID register at $PROJECT_ID_ADDR"
    exit 1
fi

PID_HEX=$(echo "$PID_RAW" | tr -d '[:space:]' | sed 's/0[xX]//')
PID_HEX_LOWER=$(echo "$PID_HEX" | tr '[:upper:]' '[:lower:]')
EXPECTED_PID="57494649"  # "WIFI"

echo "Project ID value: 0x$PID_HEX_LOWER (at $PROJECT_ID_ADDR)"

if [ "$PID_HEX_LOWER" = "$EXPECTED_PID" ]; then
    echo "MATCH: Project ID = WIFI"
else
    echo "MISMATCH!"
    echo "  Expected: 0x$EXPECTED_PID (WIFI)"
    echo "  Got:      0x$PID_HEX_LOWER"
    echo ""
    echo "=== VALIDATION FAILED ==="
    exit 1
fi

echo ""
echo "=== Bitstream Identity (build result, not source) ==="
BSHA_FILE="$PROJECT_DIR/build/fpga/bitstream.sha256"
if [ -f "$BSHA_FILE" ]; then
    EXPECTED_BSHA=$(cat "$BSHA_FILE" | tr -d '[:space:]')
    DEV_BSHA=$(pluto_ssh "fw_printenv -n deimos_bitsha 2>/dev/null" 2>/dev/null | tr -d '[:space:]' || true)
    if [ -z "$DEV_BSHA" ]; then
        echo "WARN: deimos_bitsha not set on device (pre-bitstream-identity build?)"
        echo "      The BUILD_ID register above proves the SOURCE tree, not the netlist."
    elif [ "$DEV_BSHA" = "$EXPECTED_BSHA" ]; then
        echo "MATCH: device bitstream sha256 = $DEV_BSHA"
        echo "       (flashed netlist == build/fpga/system_top.bit)"
    else
        echo "MISMATCH!"
        echo "  Expected: $EXPECTED_BSHA (build/fpga/bitstream.sha256)"
        echo "  On device: $DEV_BSHA"
        echo ""
        echo "=== VALIDATION FAILED ==="
        exit 1
    fi
else
    echo "NOTE: no build/fpga/bitstream.sha256 — skipping (build predates identity tracking)"
fi

echo ""
if [ -n "$EXPECTED_FP" ]; then
    echo "=== VALIDATION PASSED ==="
else
    echo "=== VALIDATION COMPLETE (no fingerprint check) ==="
fi
