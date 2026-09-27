#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate docs/registers.md from RTL register-map headers + BD base addresses.

The source of truth is the RTL, in two parts:

  1. Base addresses come from `ad_cpu_interconnect <hexaddr> <instance>` lines
     in the block-design tcl (deimos BD, plus the styx BD it includes).
  2. Register offsets/names/descriptions come from the register-map table in
     each module's header comment block (the comment lines before `module`).

Nothing here is hand-maintained, so the map cannot drift from the RTL without
`--check` failing.

Usage:
    python3 scripts/gen_registers.py                    # write to stdout
    python3 scripts/gen_registers.py -o docs/registers.md
    python3 scripts/gen_registers.py --check            # exit 1 on drift

Adding a peripheral: add its BD instance name and RTL path to INSTANCE_RTL
below. If a mapped instance has no parseable register table, generation fails
loudly rather than silently emitting an empty section.
"""

import argparse
import difflib
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# BD tcl files scanned for `ad_cpu_interconnect <addr> <instance>`.
BD_FILES = [
    REPO / "fpga/project/system_bd.tcl",
    REPO / "platform/styx/fpga/project/system_bd.tcl",
]

# BD instance name -> (RTL file, human label). Only these appear in the doc;
# AXI infrastructure (interconnects, IIC, SPI, ad9361) is deliberately omitted.
INSTANCE_RTL = {
    "deimos_regs_0":    ("fpga/rtl/deimos_regs_axi.v",             "Deimos receiver control/status"),
    "tag_fifo_axi_0":   ("fpga/rtl/tag_fifo_axi.v",                "Tag FIFO + PSDU BRAM readback"),
    "hil_ctrl_0":       ("platform/styx/fpga/rtl/hil_ctrl.v",      "HIL test controller (IQ playback)"),
    "snap_axi_0":       ("platform/styx/fpga/rtl/snap_axi.v",      "Debug snap capture buffer"),
    "iq_dma_rx_0":      ("platform/styx/fpga/rtl/iq_dma_rx.v",     "IQ RX DMA to DDR"),
    "iq_dma_tx_0":      ("platform/styx/fpga/rtl/iq_dma_tx.v",     "IQ TX DMA from DDR"),
    "axi_build_id_0":   ("platform/styx/fpga/rtl/axi_build_id.v",  "Build fingerprint (read-only)"),
}

# `ad_cpu_interconnect 0x7C520000 deimos_regs_0`
RE_INTERCONNECT = re.compile(
    r"^\s*ad_cpu_interconnect\s+0[xX]([0-9A-Fa-f]+)\s+(\S+)", re.MULTILINE
)

# Register lines inside a header comment. Tolerates the four styles in use:
#   //   0x00  NAME    RW  description
#   //   0x00 NAME     description
#   //   0x14-0x38     --  Reserved ...
#   //   Offset 0x00: NAME — description
RE_REG = re.compile(
    r"""^//\s+                              # comment lead
        (?:Offset\s+)?                      # optional "Offset" prefix
        0[xX](?P<off>[0-9A-Fa-f]+)          # offset
        (?:\s*[-\u2013]\s*0[xX](?P<off_end>[0-9A-Fa-f]+))?   # optional range
        \s*[:\s]\s*                         # separator
        (?P<rest>.*?)\s*$""",
    re.VERBOSE,
)

# Access qualifier as a standalone token following the name.
RE_ACCESS = re.compile(r"^(RW|RO|R/W|R|W|--)\b\s*", re.IGNORECASE)


def header_comment(text: str) -> list[str]:
    """Return the comment lines preceding the first `module` declaration."""
    out = []
    for line in text.splitlines():
        if re.match(r"^\s*module\b", line):
            break
        out.append(line)
    return out


def parse_registers(path: Path) -> list[dict]:
    """Extract register entries from a module's header comment block."""
    regs = []
    for line in header_comment(path.read_text()):
        m = RE_REG.match(line.strip())
        if not m:
            continue
        rest = m.group("rest").strip()
        if not rest:
            continue

        # Split leading NAME (or a `--` reserved marker) off the description.
        parts = rest.split(None, 1)
        name = parts[0].rstrip(":")
        desc = parts[1].strip() if len(parts) > 1 else ""

        # A reserved row has no real name.
        if name in ("--", "-"):
            name, desc = "(reserved)", rest.lstrip("- ").strip()

        # Pull out an access qualifier if the next token is one.
        access = ""
        am = RE_ACCESS.match(desc)
        if am:
            access = am.group(1).upper().replace("R/W", "RW")
            desc = desc[am.end():].strip()

        desc = desc.lstrip("\u2014-").strip()

        off = int(m.group("off"), 16)
        off_end = int(m.group("off_end"), 16) if m.group("off_end") else None
        regs.append(
            {"off": off, "off_end": off_end, "name": name, "access": access, "desc": desc}
        )
    return regs


def collect_bases() -> dict[str, int]:
    bases = {}
    for bd in BD_FILES:
        if not bd.exists():
            sys.exit(f"error: BD file not found: {bd}")
        for addr, inst in RE_INTERCONNECT.findall(bd.read_text()):
            bases[inst] = int(addr, 16)
    return bases


def render() -> str:
    bases = collect_bases()

    blocks = []
    for inst, (rel, label) in sorted(INSTANCE_RTL.items(), key=lambda kv: kv[0]):
        if inst not in bases:
            sys.exit(
                f"error: instance '{inst}' is in INSTANCE_RTL but has no "
                f"ad_cpu_interconnect line in the BD. Remove it from "
                f"INSTANCE_RTL or fix the BD."
            )
        path = REPO / rel
        if not path.exists():
            sys.exit(f"error: RTL file not found: {rel} (mapped from {inst})")
        regs = parse_registers(path)
        if not regs:
            sys.exit(
                f"error: no register table parsed from {rel}. Expected register "
                f"lines like '//   0x00  NAME  RW  description' in the header "
                f"comment block above `module`."
            )
        blocks.append((inst, rel, label, bases[inst], regs))

    out = []
    w = out.append

    w("# Register Map")
    w("")
    w("<!-- GENERATED FILE — DO NOT EDIT BY HAND.")
    w("     Regenerate: python3 scripts/gen_registers.py -o docs/registers.md")
    w("     Verify:     python3 scripts/gen_registers.py --check")
    w("     Source of truth: RTL header comment blocks + ad_cpu_interconnect")
    w("     base addresses in the block-design tcl. Edit the RTL, not this. -->")
    w("")
    w("All registers are 32-bit and word-aligned. Offsets are relative to the")
    w("peripheral base address.")
    w("")

    w("## Address Map")
    w("")
    w("| Base | Peripheral | BD instance | Source |")
    w("|------|-----------|-------------|--------|")
    for inst, rel, label, base, _ in sorted(blocks, key=lambda b: b[3]):
        w(f"| `0x{base:08X}` | {label} | `{inst}` | `{rel}` |")
    w("")

    for inst, rel, label, base, regs in sorted(blocks, key=lambda b: b[3]):
        w(f"## {label} — `0x{base:08X}`")
        w("")
        w(f"Instance `{inst}`, defined in `{rel}`.")
        w("")
        w("| Offset | Absolute | Name | Access | Description |")
        w("|--------|----------|------|--------|-------------|")
        for r in regs:
            if r["off_end"] is not None:
                off = f"`0x{r['off']:02X}`-`0x{r['off_end']:02X}`"
                absolute = f"`0x{base + r['off']:08X}`-`0x{base + r['off_end']:08X}`"
            else:
                off = f"`0x{r['off']:02X}`"
                absolute = f"`0x{base + r['off']:08X}`"
            desc = r["desc"].replace("|", "\\|")
            w(f"| {off} | {absolute} | `{r['name']}` | {r['access'] or '—'} | {desc} |")
        w("")

    return "\n".join(out) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-o", "--output", type=Path, help="write to file instead of stdout")
    ap.add_argument(
        "--check",
        action="store_true",
        help="compare against docs/registers.md; exit 1 on drift",
    )
    args = ap.parse_args()

    text = render()

    if args.check:
        target = REPO / "docs/registers.md"
        if not target.exists():
            print(f"FAIL: {target} does not exist", file=sys.stderr)
            return 1
        current = target.read_text()
        if current != text:
            print("FAIL: docs/registers.md is out of sync with the RTL.", file=sys.stderr)
            print("", file=sys.stderr)
            diff = difflib.unified_diff(
                current.splitlines(keepends=True),
                text.splitlines(keepends=True),
                fromfile="docs/registers.md (committed)",
                tofile="generated from RTL",
            )
            sys.stderr.writelines(diff)
            print(
                "\nRegenerate: python3 scripts/gen_registers.py -o docs/registers.md",
                file=sys.stderr,
            )
            return 1
        print("OK: docs/registers.md matches the RTL.")
        return 0

    if args.output:
        args.output.write_text(text)
        print(f"wrote {args.output}", file=sys.stderr)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
