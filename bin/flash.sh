#!/bin/bash
# bin/flash.sh — Flash pluto.frm to PlutoSDR via mtd3
#
# Uploads firmware to Pluto over USB-network, erases flash, writes,
# and reboots. Does NOT use DFU — this is the safe mtd3 update path.
#
# EXPECTED WALL TIME: 4-5 minutes (flash_erase is the bottleneck)
# AGENT TIMEOUT: use 1200000 ms (20 min) when invoking from OpenCode
#
# Usage:
#   ./bin/flash.sh              # flash build/fpga/pluto.frm
#   ./bin/flash.sh path/to.frm  # flash specific file
#
# WARNING: flash_erase takes ~4-5 minutes. Be patient.
# The script will print progress. Do not interrupt.
#
# Prerequisites:
#   - Pluto connected via USB (192.168.2.1 reachable)
#   - sshpass installed (brew install sshpass / hudochenkov/sshpass)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FRM="${1:-$PROJECT_DIR/build/fpga/pluto.frm}"
PLUTO="root@192.168.2.1"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

# md5sum is Linux, `md5` is macOS. Normalize to a bare lowercase hash so the
# trailer check works on either host.
md5_of_stdin() {
    if command -v md5sum >/dev/null 2>&1; then
        md5sum | cut -d ' ' -f 1
    elif command -v md5 >/dev/null 2>&1; then
        md5 -q
    else
        echo "ERROR: neither md5sum nor md5 found on PATH" >&2
        return 1
    fi
}

if [ ! -f "$FRM" ]; then
    echo "ERROR: $FRM not found"
    echo "Run 'make bitstream' first, or specify path to .frm"
    exit 1
fi

echo "=== Firmware: $FRM ($(du -h "$FRM" | cut -f1)) ==="

# ---- Verify checksum ----
TOTAL=$(wc -c < "$FRM" | tr -d ' ')
ITB_SIZE=$((TOTAL - 33))
MD5_EXPECTED=$(tail -c 33 "$FRM" | tr -d '\n')
MD5_ACTUAL=$(dd if="$FRM" bs=1 count=$ITB_SIZE 2>/dev/null | md5_of_stdin)

if [ "$MD5_EXPECTED" != "$MD5_ACTUAL" ]; then
    echo "CHECKSUM MISMATCH — aborting"
    echo "  Expected: $MD5_EXPECTED"
    echo "  Actual:   $MD5_ACTUAL"
    exit 1
fi
echo "Checksum verified OK"

# ---- Check Pluto connectivity ----
echo ""
echo "=== Checking Pluto at 192.168.2.1 ==="
if ! sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO "echo ok" 2>/dev/null; then
    echo "ERROR: Cannot reach Pluto at 192.168.2.1"
    echo "Check USB connection and try again."
    exit 1
fi
echo "Pluto reachable."

# ---- Record pre-flash state ----
echo ""
echo "=== Pre-flash state ==="
sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO \
    "cat /opt/VERSIONS 2>/dev/null; echo ''; uname -r" 2>/dev/null || true

# ---- Upload ----
echo ""
echo "=== Uploading firmware ($(du -h "$FRM" | cut -f1)) ==="
sshpass -p "$PLUTO_PASS" scp -O $SSH_OPTS "$FRM" "$PLUTO:/tmp/pluto.frm"
echo "Upload complete."

# ---- Flash ----
echo ""
echo "=== Flashing (flash_erase takes ~4-5 min) ==="
sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO '
    TOTAL=$(wc -c < /tmp/pluto.frm)
    ITB_SIZE=$((TOTAL - 33))
    dd if=/tmp/pluto.frm of=/tmp/fw.itb bs=1 count=$ITB_SIZE 2>/dev/null
    echo "ITB size: $ITB_SIZE bytes"
    echo "Erasing mtd3 (this is slow)..."
    flash_erase /dev/mtd3 0 0
    echo "Writing to mtd3..."
    dd if=/tmp/fw.itb of=/dev/mtdblock3 bs=64k
    fw_setenv fit_size $(printf "%X" $ITB_SIZE)
    sync
    rm /tmp/pluto.frm /tmp/fw.itb
    echo "FLASH COMPLETE"
'

# ---- Stamp bitstream identity into U-Boot env ----
# The BUILD_ID register carries the SOURCE fingerprint, which cannot
# distinguish two placements of the same tree. The bitstream sha256
# (build/fpga/bitstream.sha256, written by build.tcl) identifies the
# build RESULT. Stamped here so validate.sh can prove which build is
# actually running.
BSHA_FILE="$PROJECT_DIR/build/fpga/bitstream.sha256"
if [ -f "$BSHA_FILE" ]; then
    BSHA=$(cat "$BSHA_FILE" | tr -d '[:space:]')
    if [ -n "$BSHA" ] && [ "$BSHA" != "unavailable" ]; then
        echo "=== Stamping bitstream identity: $BSHA ==="
        sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO \
            "fw_setenv deimos_bitsha $BSHA" 2>/dev/null \
            || echo "WARN: fw_setenv deimos_bitsha failed (env may be read-only)"
    fi
else
    echo "NOTE: no build/fpga/bitstream.sha256 — bitstream identity not stamped"
fi

# ---- Reboot ----
echo ""
echo "=== Rebooting Pluto ==="
sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO "device_reboot reset" 2>/dev/null || true

echo ""
echo "Pluto is rebooting (~20 seconds)."
echo "Validate with: ./bin/validate.sh"
