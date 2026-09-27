#!/bin/bash
# SPDX-License-Identifier: MIT
# packaging/package.sh — Package bitstream + kernel + rootfs into pluto.frm
#
# Follows styx's packaging pattern:
#   1. Stage files into plutosdr-fw/build/
#   2. Compile and apply device tree overlay (disable ADI DMA, reserve DDR)
#   3. Build FIT image with upstream scripts/pluto.its
#   4. Append MD5 trailer → pluto.frm
#
# Usage: packaging/package.sh <output_dir>
#
# Requires:
#   - Bitstream at <output_dir>/system_top.bit
#   - Built plutosdr-fw at platform/styx/extern/plutosdr-fw/
#   - dtc and fdtoverlay on PATH (or in plutosdr-fw buildroot host tools)
#
# Produces:
#   - <output_dir>/pluto.frm

set -euo pipefail

OUTPUT_DIR="${1:?Usage: $0 <output_dir>}"
case "$OUTPUT_DIR" in
    /*) ;;
    *) OUTPUT_DIR="$(pwd)/$OUTPUT_DIR" ;;
esac
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
STYX_DIR="$PROJECT_DIR/platform/styx"
FW_DIR="$STYX_DIR/extern/plutosdr-fw"
OVERLAY_SRC="$SCRIPT_DIR/deimos-pluto.dtso"

# md5sum is Linux, `md5` is macOS. Normalize to a bare lowercase hash so the
# pluto.frm trailer is identical regardless of build host.
md5_of_file() {
    if command -v md5sum >/dev/null 2>&1; then
        md5sum "$1" | cut -d ' ' -f 1
    elif command -v md5 >/dev/null 2>&1; then
        md5 -q "$1"
    else
        echo "ERROR: neither md5sum nor md5 found on PATH" >&2
        return 1
    fi
}

# ---- Validate inputs ----
if [ ! -f "$OUTPUT_DIR/system_top.bit" ]; then
    echo "ERROR: No bitstream at $OUTPUT_DIR/system_top.bit"
    exit 1
fi

if [ ! -f "$FW_DIR/buildroot/output/images/rootfs.cpio.gz" ]; then
    echo "ERROR: plutosdr-fw not built. Run 'make setup' first."
    exit 1
fi

if [ ! -f "$FW_DIR/linux/arch/arm/boot/zImage" ]; then
    echo "ERROR: No zImage at $FW_DIR/linux/arch/arm/boot/zImage"
    exit 1
fi

if [ ! -f "$FW_DIR/scripts/pluto.its" ]; then
    echo "ERROR: Upstream ITS not found at $FW_DIR/scripts/pluto.its"
    exit 1
fi

if [ ! -f "$OVERLAY_SRC" ]; then
    echo "ERROR: Device tree overlay not found at $OVERLAY_SRC"
    exit 1
fi

# ---- Locate tools ----
MKIMAGE="$FW_DIR/u-boot-xlnx/tools/mkimage"
if [ ! -x "$MKIMAGE" ]; then
    MKIMAGE="$FW_DIR/buildroot/output/host/bin/mkimage"
fi
if [ ! -x "$MKIMAGE" ]; then
    echo "ERROR: mkimage not found"
    echo "  Checked: $FW_DIR/u-boot-xlnx/tools/mkimage"
    echo "  Checked: $FW_DIR/buildroot/output/host/bin/mkimage"
    exit 1
fi

DTC="$FW_DIR/buildroot/output/host/bin/dtc"
if [ ! -x "$DTC" ]; then
    DTC=$(command -v dtc) || { echo "ERROR: dtc not found"; exit 1; }
fi

FDTOVERLAY_BIN="$FW_DIR/buildroot/output/host/bin/fdtoverlay"
if [ ! -x "$FDTOVERLAY_BIN" ]; then
    FDTOVERLAY_BIN=$(command -v fdtoverlay) || { echo "ERROR: fdtoverlay not found"; exit 1; }
fi

export PATH="$FW_DIR/buildroot/output/host/bin:$PATH"

echo "=== Packaging pluto.frm ==="
echo "  mkimage:    $MKIMAGE"
echo "  dtc:        $DTC"
echo "  fdtoverlay: $FDTOVERLAY_BIN"
echo "  overlay:    $OVERLAY_SRC"

# ---- Stage files into plutosdr-fw/build/ ----
BUILD_DIR="$FW_DIR/build"
mkdir -p "$BUILD_DIR"

echo ""
echo "Staging files into $BUILD_DIR/"

cp "$OUTPUT_DIR/system_top.bit" "$BUILD_DIR/system_top.bit"
echo "  system_top.bit"

cp "$FW_DIR/linux/arch/arm/boot/zImage" "$BUILD_DIR/zImage"
echo "  zImage"

cp "$FW_DIR/buildroot/output/images/rootfs.cpio.gz" "$BUILD_DIR/rootfs.cpio.gz"
echo "  rootfs.cpio.gz"

# ---- Brand rootfs with deimos motd + issue ----
# The source rootfs.cpio.gz has files owned by root:root. We must preserve
# that ownership through the extract/modify/repack cycle. Running as non-root
# user would otherwise stamp all files with the build user's UID,
# which can prevent Linux from booting (wrong ownership on /etc/*).
#
# Solution: fakeroot wraps the entire cycle, faking uid/gid 0 for all
# file operations so cpio records root ownership in the archive.
ROOTFS_DIR="$SCRIPT_DIR/rootfs"
BRAND_TMP=$(mktemp -d)
trap "rm -rf $BRAND_TMP" EXIT

echo ""
echo "Branding rootfs with deimos overlays..."

if ! command -v fakeroot >/dev/null 2>&1; then
    echo "ERROR: fakeroot not found (needed to preserve root:root in rootfs)."
    echo "  Linux:  apt install fakeroot"
    echo "  macOS:  no maintained Homebrew formula — build the rootfs on Linux"
    echo "          or inside a container. See README prerequisites."
    exit 1
fi

fakeroot -- bash -ec "
    cd '$BRAND_TMP'
    zcat '$BUILD_DIR/rootfs.cpio.gz' | cpio -idm --quiet
    cp '$ROOTFS_DIR/motd' etc/motd
    cp '$ROOTFS_DIR/issue' etc/issue
    find . | cpio -o -H newc --quiet | gzip > '$BUILD_DIR/rootfs.cpio.gz'
"
echo "  rootfs branded (motd + issue)"

# ---- Compile device tree overlay ----
echo ""
echo "Compiling device tree overlay..."
"$DTC" -q -@ -I dts -O dtb -o "$BUILD_DIR/deimos-pluto.dtbo" "$OVERLAY_SRC"
echo "  deimos-pluto.dtbo"

# ---- Apply overlay to stock DTBs ----
echo ""
echo "Patching device trees..."

DTB_SRC_DIR="$FW_DIR/linux/arch/arm/boot/dts"

# plutosdr-fw may have moved DTBs to build/ after kernel build
if [ ! -f "$DTB_SRC_DIR/zynq-pluto-sdr.dtb" ]; then
    DTB_SRC_DIR="$FW_DIR/build"
fi

for dtb_name in zynq-pluto-sdr.dtb zynq-pluto-sdr-revb.dtb zynq-pluto-sdr-revc.dtb; do
    src="$DTB_SRC_DIR/$dtb_name"
    if [ ! -f "$src" ]; then
        echo "  WARNING: $dtb_name not found (skipping)"
        continue
    fi

    "$FDTOVERLAY_BIN" -i "$src" -o "$BUILD_DIR/$dtb_name" "$BUILD_DIR/deimos-pluto.dtbo"
    echo "  $dtb_name patched"
done

# ---- Build FIT image using upstream ITS ----
echo ""
echo "Building FIT image..."

cd "$FW_DIR"
"$MKIMAGE" -f scripts/pluto.its build/pluto.itb

# ---- Create pluto.frm (ITB + MD5 trailer) ----
md5_of_file build/pluto.itb > build/pluto.frm.md5
cat build/pluto.itb build/pluto.frm.md5 > build/pluto.frm

cp build/pluto.frm "$OUTPUT_DIR/pluto.frm"

SIZE=$(du -h "$OUTPUT_DIR/pluto.frm" | cut -f1)
echo ""
echo "=== Packaged: $OUTPUT_DIR/pluto.frm ($SIZE) ==="
