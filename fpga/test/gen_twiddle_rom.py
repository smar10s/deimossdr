#!/usr/bin/env python3
"""Generate and verify the R2^2 SDF twiddle ROM.

WHAT THE ROM IS
---------------
A stage-pair twiddle ROM for size N has N entries, addressed by the enclosing
module's sample counter. It is divided into four equal groups of N/4 entries.
Group g uses stride s[g]: entry i of that group is

    W_N^(s[g] * i mod N)

so the four-element stride list is the complete ROM definition. Values are
Q1.15: round(32767 * exp(-j * 2*pi * phase / N)).

VERIFIED STRIDES
----------------
    N=16  -> [2, 1, 3, 0]   fft16_sdf, fpga/rtl/fft64_sdf.v
    N=64  -> [2, 1, 3, 0]   fft64_sdf, fpga/rtl/fft64_sdf.v

The list is self-similar at both sizes. Only the group size and the phase
modulus change with N.

WHY NOT A CLOSED FORM
---------------------
The obvious formula

    phase(c) = (c & (N/4 - 1)) * (c >> log2(N/4))

gives strides [0, 1, 2, 3] and is wrong. It assumes the butterfly pair emits
data in block-sequential order. The combinational BF pair with delay feedback
reorders the groups. Do not derive the table; verify it or recover it by sweep.

VERIFICATION
------------
--verify parses the twiddle case tables out of fpga/rtl and regenerates them
from the strides. The comparison is exact: every Q1.15 entry must match.

--recover runs the per-group sweep that discovered the table. The four groups
occupy disjoint counter ranges, so each stride can be searched independently:
4*N evaluations, not N^4. The score is the worst LSB error against the RTL
table. This proves the stride structure of the emitted table. It does not
re-litigate hardware behavior; that evidence lives in the HIL and OTA logs.

N=128 IS NOT VERIFIED
---------------------
The self-similar candidate is again [2, 1, 3, 0]: four groups of 32, exponent
modulus 128. 128 = 2 * 4^3, not a power of four, so the sub-FFT chain changes.
Bring it up bottom-up and mind these traps:

  - Verify fft2, then fft8, then fft32, then fft128. Start each level with an
    identity ROM, then an impulse, then a shifted impulse.
  - Try [2, 1, 3, 0] before sweeping for a new table.
  - Match the ROM address delay to the BF2 input register. fft16_sdf addresses
    with gcnt_d1 because its BF2 input is registered; fft64_sdf addresses with
    gcnt because it is not. Check the new pipeline, do not copy either.
  - FILL_SKIP and the output bit-reversal width are empirical. N=128 needs a
    7-bit reversal; re-derive the fill skip instead of copying N=64's 63.

`--print 128` emits the candidate ROM. Once fft128 RTL exists, `--recover 128`
reads the real structure out of it, then `--verify 128` locks it down.

USAGE
-----
    python3 gen_twiddle_rom.py 16 --print
    python3 gen_twiddle_rom.py 128 --print
    python3 gen_twiddle_rom.py 16 --print --addr gcnt_d1
    python3 gen_twiddle_rom.py --verify
    python3 gen_twiddle_rom.py 128 --recover
    python3 gen_twiddle_rom.py 128 --verify --strides 2,1,3,0
"""

import argparse
import math
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RTL_DIR = REPO_ROOT / "fpga" / "rtl"

# Strides verified against the RTL and cocotb tests.
KNOWN_STRIDES = {
    16: (2, 1, 3, 0),
    64: (2, 1, 3, 0),
}

# Self-similar extrapolations. None of these are verified.
CANDIDATE_STRIDES = {
    8: (2, 1, 3, 0),
    32: (2, 1, 3, 0),
    128: (2, 1, 3, 0),
}

TW_ENTRY_RE = re.compile(
    r"(?P<aw>\d+)'d\s*(?P<idx>\d+)\s*:\s*begin\s+"
    r"tw_rom_re\s*=\s*(?P<rs>-?)16'sd(?P<re>\d+)\s*;\s*"
    r"tw_rom_im\s*=\s*(?P<is>-?)16'sd(?P<im>\d+)\s*;\s*end"
)


def w_n(phase, N):
    """W_N^phase in Q1.15, rounded the same way the RTL tables were made."""
    angle = -2.0 * math.pi * phase / N
    return int(round(32767 * math.cos(angle))), int(round(32767 * math.sin(angle)))


def parse_strides(text):
    strides = tuple(int(part) for part in text.split(","))
    if len(strides) != 4:
        raise SystemExit("Need exactly four comma-separated strides")
    return strides


def gen_table(N, strides):
    if N < 4 or (N & (N - 1)):
        raise SystemExit(f"N must be a power of two >= 4, got {N}")
    if len(strides) != 4:
        raise SystemExit("Need exactly four strides, one per group")
    if any(not 0 <= s < N for s in strides):
        raise SystemExit(f"Every stride must be in 0..{N - 1}, got {list(strides)}")

    quarter = N // 4
    return {
        c: w_n((strides[c // quarter] * (c % quarter)) % N, N)
        for c in range(N)
    }


def parse_rtl_tables(paths):
    """Return {N: (path, {idx: (re, im)})} for every twiddle case table."""
    found = {}
    for path in paths:
        text = path.read_text()
        groups = {}
        for m in TW_ENTRY_RE.finditer(text):
            aw = int(m["aw"])
            idx = int(m["idx"])
            re = int(m["re"]) * (-1 if m["rs"] else 1)
            im = int(m["im"]) * (-1 if m["is"] else 1)
            groups.setdefault(aw, {})[idx] = (re, im)

        for aw, entries in groups.items():
            N = 1 << aw
            missing = sorted(set(range(N)) - set(entries))
            if missing:
                raise SystemExit(
                    f"{path}: N={N} table is incomplete, missing {missing[:8]}"
                )
            if N in found and found[N][1] != entries:
                raise SystemExit(
                    f"{path}: conflicting N={N} tables in "
                    f"{found[N][0]} and {path}"
                )
            found[N] = (path, entries)
    return found


def print_verilog(N, strides, addr):
    table = gen_table(N, strides)
    quarter = N // 4
    idx_bits = int(math.log2(N))

    if N in KNOWN_STRIDES and strides == KNOWN_STRIDES[N]:
        status = "VERIFIED against the RTL and cocotb"
    elif N in CANDIDATE_STRIDES and strides == CANDIDATE_STRIDES[N]:
        status = "CANDIDATE, unverified"
    else:
        status = "CUSTOM, unverified"

    print(f"// Twiddle ROM for N={N}, R2^2 SDF stage pair")
    print(f"// Generated by gen_twiddle_rom.py from strides {list(strides)}")
    print(f"// STATUS: {status}")
    print(f"// Entry i of group g is W_{N}^(stride[g] * i).")
    print(f"always @(*) begin")
    print(f"    case ({addr}[{idx_bits - 1}:0])")
    for c in range(N):
        phase = (strides[c // quarter] * (c % quarter)) % N
        re, im = table[c]
        re_str = f"-16'sd{-re}" if re < 0 else f" 16'sd{re}"
        im_str = f"-16'sd{-im}" if im < 0 else f" 16'sd{im}"
        print(
            f"        {idx_bits}'d{c}: begin tw_rom_re = {re_str:>10}; "
            f"tw_rom_im = {im_str:>10}; end // W^{phase}"
        )
    print(f"    endcase")
    print(f"end")


def verify_one(N, path, entries, strides):
    expected = gen_table(N, strides)
    bad = [
        (i, entries[i], expected[i])
        for i in range(N)
        if entries[i] != expected[i]
    ]
    if bad:
        print(f"N={N:<3} FAIL  {path.name}  strides={list(strides)}")
        for i, got, want in bad[:8]:
            print(f"  entry {i}: rtl={got} generated={want}")
        if len(bad) > 8:
            print(f"  ... {len(bad) - 8} more")
        return False
    print(f"N={N:<3} OK    {path.name}  strides={list(strides)}  ({N} entries)")
    return True


def recover_one(N, path, entries):
    quarter = N // 4
    strides = []
    ok = True

    for group in range(4):
        scored = []
        for stride in range(N):
            err = 0
            for i in range(quarter):
                re, im = w_n((stride * i) % N, N)
                rtl_re, rtl_im = entries[group * quarter + i]
                err = max(err, abs(re - rtl_re), abs(im - rtl_im))
            scored.append((err, stride))
        scored.sort()
        best_err, best_stride = scored[0]
        runner_err, runner_stride = scored[1]
        strides.append(best_stride)
        ambiguity = "  (TIE)" if runner_err == best_err else ""
        print(
            f"  group {group}: stride {best_stride}  err={best_err}  "
            f"next={runner_stride} err={runner_err}{ambiguity}"
        )
        ok = ok and best_err == 0 and runner_err > best_err

    strides = tuple(strides)
    print(f"N={N:<3} {path.name}  recovered strides={list(strides)}")
    if not ok:
        print(f"N={N} does not match a per-group stride model", file=sys.stderr)
        return None
    return strides


def resolve_strides(N, explicit):
    if explicit is not None:
        return parse_strides(explicit)
    if N in KNOWN_STRIDES:
        return KNOWN_STRIDES[N]
    if N in CANDIDATE_STRIDES:
        return CANDIDATE_STRIDES[N]
    raise SystemExit(
        f"No known or candidate strides for N={N}; pass --strides S0,S1,S2,S3"
    )


def rtl_paths(explicit):
    if explicit:
        return [Path(p) for p in explicit]
    paths = sorted(RTL_DIR.glob("*.v"))
    if not paths:
        raise SystemExit(f"No RTL files found under {RTL_DIR}")
    return paths


def main():
    parser = argparse.ArgumentParser(
        description="Generate and verify the R2^2 SDF twiddle ROM")
    parser.add_argument("N", type=int, nargs="?",
                        help="FFT size (power of two >= 4)")
    parser.add_argument("--print", dest="emit", action="store_true",
                        help="emit the Verilog case table (default with N)")
    parser.add_argument("--verify", action="store_true",
                        help="compare the RTL table against the strides")
    parser.add_argument("--recover", action="store_true",
                        help="recover strides from the RTL table by sweep")
    parser.add_argument("--strides", default=None,
                        help='strides per group, e.g. "2,1,3,0"')
    parser.add_argument("--addr", default="gcnt",
                        help="counter expression used in the emitted case "
                             "(default: gcnt)")
    parser.add_argument("--rtl", action="append",
                        help="RTL file to scan (repeatable; default: fpga/rtl/*.v)")
    args = parser.parse_args()

    if not args.emit and not args.verify and not args.recover:
        if args.N is None:
            args.verify = True
        else:
            args.emit = True

    if args.emit:
        if args.N is None:
            raise SystemExit("--print needs an N")
        strides = resolve_strides(args.N, args.strides)
        print_verilog(args.N, strides, args.addr)
        return 0

    tables = parse_rtl_tables(rtl_paths(args.rtl))
    if not tables:
        raise SystemExit("No twiddle case tables found in the RTL")

    if args.N is not None:
        if args.N not in tables:
            raise SystemExit(f"No N={args.N} twiddle table found in the RTL")
        selected = {args.N: tables[args.N]}
    else:
        selected = tables

    if args.recover:
        failed = False
        for N in sorted(selected):
            path, entries = selected[N]
            print(f"N={N}  {path}")
            if recover_one(N, path, entries) is None:
                failed = True
        return 1 if failed else 0

    failed = False
    for N in sorted(selected):
        path, entries = selected[N]
        if args.strides is not None:
            strides = parse_strides(args.strides)
        elif N in KNOWN_STRIDES:
            strides = KNOWN_STRIDES[N]
        elif N in CANDIDATE_STRIDES:
            strides = CANDIDATE_STRIDES[N]
        else:
            print(f"N={N:<3} SKIP  {path.name}  (no expected strides; "
                  f"use --recover {N})")
            continue
        if not verify_one(N, path, entries, strides):
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
