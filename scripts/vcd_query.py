#!/usr/bin/env python3
"""
vcd_query.py — Extract and analyze signals from VCD trace files.

Standard debug tool for the streaming pipeline phase. Generates VCD with:
    WAVES=1 make -C fpga/test test_rx_pipeline TESTCASE=test_rate6_fcs

Then query:
    # List all signals matching a pattern
    python scripts/vcd_query.py list dump.vcd pilot_track

    # Extract signal transitions in a time range
    python scripts/vcd_query.py extract dump.vcd signal1 signal2 --start 5000 --end 10000

    # Compare a signal across two symbol periods (by symbol_idx)
    python scripts/vcd_query.py compare-symbols dump.vcd --good 2 --bad 3

    # Tabular per-symbol timing report (symbol_start, data_valid edges)
    python scripts/vcd_query.py symbol-timing dump.vcd
"""

import sys
import os
import time
import argparse
from pathlib import Path

from vcdvcd import VCDVCD


# Freshness check: warn if VCD is older than 10 minutes
VCD_MAX_AGE_SECONDS = 600


def check_vcd_freshness(vcd_path):
    """Warn if VCD file appears stale (older than 10 minutes)."""
    try:
        mtime = os.path.getmtime(vcd_path)
        age = time.time() - mtime
        if age > VCD_MAX_AGE_SECONDS:
            mins = int(age / 60)
            print(f"WARNING: VCD is {mins} min old — possibly STALE.", file=sys.stderr)
            print(f"  File: {vcd_path}", file=sys.stderr)
            print("  Run: make waves TARGET=<target> to regenerate", file=sys.stderr)
            print(f"  (set VCD_NO_FRESHNESS_CHECK=1 to suppress)", file=sys.stderr)
            print("", file=sys.stderr)
    except OSError:
        pass


def find_signals(vcd, pattern):
    """Find all signal names matching a substring pattern."""
    pattern_lower = pattern.lower()
    matches = []
    for sig in sorted(vcd.signals):
        if pattern_lower in sig.lower():
            matches.append(sig)
    return matches


def cmd_list(args):
    """List signals matching a pattern."""
    if not os.environ.get('VCD_NO_FRESHNESS_CHECK'):
        check_vcd_freshness(args.vcd_file)
    vcd = VCDVCD(args.vcd_file, signals=[], store_tvs=False)
    pattern = args.pattern if args.pattern else ''
    matches = find_signals(vcd, pattern)
    print(f"Found {len(matches)} signals matching '{pattern}':")
    for sig in matches:
        print(f"  {sig}")


def get_signal_tv(vcd, signal_name):
    """Get time-value pairs for a signal, handling hierarchical lookup."""
    # Try exact match first
    if signal_name in vcd:
        return vcd[signal_name].tv
    # Try suffix match
    for sig in vcd.signals:
        if sig.endswith('.' + signal_name) or sig.endswith('.' + signal_name + '['):
            return vcd[sig].tv
    # Try substring
    matches = find_signals(vcd, signal_name)
    if len(matches) == 1:
        return vcd[matches[0]].tv
    elif len(matches) > 1:
        print(f"  Ambiguous signal '{signal_name}', matches:", file=sys.stderr)
        for m in matches[:10]:
            print(f"    {m}", file=sys.stderr)
        sys.exit(1)
    else:
        print(f"  Signal '{signal_name}' not found", file=sys.stderr)
        sys.exit(1)


def value_at_time(tv_pairs, t):
    """Get the value of a signal at time t (last transition <= t)."""
    val = '0'
    for time, v in tv_pairs:
        if time > t:
            break
        val = v
    return val


def parse_vcd_value(v):
    """Parse a VCD value string to int. Returns None for x/z."""
    if 'x' in v.lower() or 'z' in v.lower():
        return None
    # Binary string or single bit
    try:
        return int(v, 2)
    except ValueError:
        try:
            return int(v)
        except ValueError:
            return None


def cmd_extract(args):
    """Extract signal transitions in a time range."""
    if not os.environ.get('VCD_NO_FRESHNESS_CHECK'):
        check_vcd_freshness(args.vcd_file)
    signals_to_load = args.signals
    vcd = VCDVCD(args.vcd_file)

    start_t = args.start if args.start else 0
    end_t = args.end if args.end else float('inf')

    for sig_name in signals_to_load:
        matches = find_signals(vcd, sig_name)
        if not matches:
            print(f"Signal '{sig_name}' not found")
            continue

        for full_name in matches[:3]:  # Limit to 3 matches per pattern
            tv = vcd[full_name].tv
            print(f"\n--- {full_name} ---")
            count = 0
            for time, val in tv:
                if time < start_t:
                    continue
                if time > end_t:
                    break
                int_val = parse_vcd_value(val)
                val_str = f"{val} (={int_val})" if int_val is not None else val
                print(f"  t={time:>8}: {val_str}")
                count += 1
                if count > 500:
                    print(f"  ... (truncated, {len(tv)} total transitions)")
                    break


def cmd_symbol_timing(args):
    """Per-symbol timing report: when does each symbol start and how long
    does data_valid_out stay high?"""
    if not os.environ.get('VCD_NO_FRESHNESS_CHECK'):
        check_vcd_freshness(args.vcd_file)
    vcd = VCDVCD(args.vcd_file)

    # Find key signals
    sym_start_matches = find_signals(vcd, 'symbol_start_out')
    sym_idx_matches = find_signals(vcd, 'symbol_idx_out')
    pt_valid_matches = find_signals(vcd, 'pt_data_valid')

    if not sym_start_matches:
        # Try alternate naming (pipeline level)
        sym_start_matches = find_signals(vcd, 'symbol_start')
    if not pt_valid_matches:
        pt_valid_matches = find_signals(vcd, 'data_valid_out')

    if not sym_start_matches or not sym_idx_matches:
        print("Could not find symbol_start/symbol_idx signals")
        print("Available signals with 'symbol':")
        for s in find_signals(vcd, 'symbol'):
            print(f"  {s}")
        return

    sym_start_tv = vcd[sym_start_matches[0]].tv
    sym_idx_tv = vcd[sym_idx_matches[0]].tv

    pt_valid_tv = None
    if pt_valid_matches:
        pt_valid_tv = vcd[pt_valid_matches[0]].tv

    # Find symbol_start rising edges and the corresponding symbol_idx
    print(f"Signal: {sym_start_matches[0]}")
    print(f"Signal: {sym_idx_matches[0]}")
    if pt_valid_matches:
        print(f"Signal: {pt_valid_matches[0]}")
    print()
    print(f"{'Sym':>4} {'Start_t':>10} {'Idx':>4} {'Valid_rise':>11} "
          f"{'Valid_fall':>11} {'Valid_dur':>10} {'Gap_from_prev':>14}")
    print("-" * 80)

    # Collect symbol starts (rising edges of symbol_start)
    symbols = []
    prev_val = '0'
    for time, val in sym_start_tv:
        int_val = parse_vcd_value(val)
        if int_val == 1 and parse_vcd_value(prev_val) == 0:
            # Rising edge — what's the symbol idx at this time?
            idx_val = value_at_time(sym_idx_tv, time)
            idx_int = parse_vcd_value(idx_val)
            symbols.append((time, idx_int))
        prev_val = val

    # For each symbol, find the corresponding data_valid_out window
    prev_fall = None
    for sym_time, sym_idx in symbols:
        valid_rise = None
        valid_fall = None

        if pt_valid_tv:
            # Find first rising edge of data_valid_out after sym_time
            prev_v = '0'
            for time, val in pt_valid_tv:
                if time < sym_time:
                    prev_v = val
                    continue
                int_val = parse_vcd_value(val)
                prev_int = parse_vcd_value(prev_v)
                if int_val == 1 and prev_int == 0 and valid_rise is None:
                    valid_rise = time
                elif int_val == 0 and prev_int == 1 and valid_rise is not None:
                    valid_fall = time
                    break
                prev_v = val

        dur = (valid_fall - valid_rise) if (valid_rise and valid_fall) else None
        gap = (valid_rise - prev_fall) if (valid_rise and prev_fall) else None

        dur_str = str(dur) if dur else '-'
        gap_str = str(gap) if gap else '-'
        rise_str = str(valid_rise) if valid_rise else '-'
        fall_str = str(valid_fall) if valid_fall else '-'

        print(f"{sym_idx:>4} {sym_time:>10} {sym_idx:>4} {rise_str:>11} "
              f"{fall_str:>11} {dur_str:>10} {gap_str:>14}")

        if valid_fall:
            prev_fall = valid_fall


def cmd_compare_symbols(args):
    """Compare signal behavior between two symbols (good vs bad).

    Extracts all pilot_track-related signals during the processing window
    of each symbol and shows them side-by-side.
    """
    if not os.environ.get('VCD_NO_FRESHNESS_CHECK'):
        check_vcd_freshness(args.vcd_file)
    vcd = VCDVCD(args.vcd_file)

    # Find symbol_start rising edges to determine time windows
    sym_start_matches = find_signals(vcd, 'symbol_start_out')
    sym_idx_matches = find_signals(vcd, 'symbol_idx_out')

    if not sym_start_matches or not sym_idx_matches:
        sym_start_matches = find_signals(vcd, 'symbol_start')
        sym_idx_matches = find_signals(vcd, 'symbol_idx')

    if not sym_start_matches or not sym_idx_matches:
        print("Cannot find symbol timing signals")
        return

    sym_start_tv = vcd[sym_start_matches[0]].tv
    sym_idx_tv = vcd[sym_idx_matches[0]].tv

    # Build symbol time windows: [start_of_sym_N, start_of_sym_N+1)
    symbols = []
    prev_val = '0'
    for time, val in sym_start_tv:
        int_val = parse_vcd_value(val)
        if int_val == 1 and parse_vcd_value(prev_val) == 0:
            idx_val = value_at_time(sym_idx_tv, time)
            idx_int = parse_vcd_value(idx_val)
            symbols.append((time, idx_int))
        prev_val = val

    # Find windows for good and bad symbols
    good_idx = args.good
    bad_idx = args.bad

    def find_window(sym_idx):
        for i, (t, idx) in enumerate(symbols):
            if idx == sym_idx:
                end_t = symbols[i + 1][0] if i + 1 < len(symbols) else t + 5000
                return (t, end_t)
        return None

    good_win = find_window(good_idx)
    bad_win = find_window(bad_idx)

    if not good_win:
        print(f"Symbol idx {good_idx} not found. Available: {[s[1] for s in symbols]}")
        return
    if not bad_win:
        print(f"Symbol idx {bad_idx} not found. Available: {[s[1] for s in symbols]}")
        return

    print(f"Good symbol (idx={good_idx}): t={good_win[0]} to {good_win[1]} "
          f"(dur={good_win[1]-good_win[0]})")
    print(f"Bad symbol  (idx={bad_idx}):  t={bad_win[0]} to {bad_win[1]} "
          f"(dur={bad_win[1]-bad_win[0]})")
    print()

    # Extract key signals in both windows
    key_patterns = [
        'pilot_valid', 'data_valid_in', 'data_valid_out',
        'symbol_done', 'start_pending', 'emit_done',
        'eq_data_valid', 'pt_data_valid',
    ]

    for pattern in key_patterns:
        matches = find_signals(vcd, pattern)
        if not matches:
            continue
        sig_name = matches[0]
        tv = vcd[sig_name].tv

        def count_pulses(window):
            """Count rising edges in window."""
            start, end = window
            count = 0
            prev_v = parse_vcd_value(value_at_time(tv, start))
            for time, val in tv:
                if time <= start:
                    continue
                if time > end:
                    break
                int_v = parse_vcd_value(val)
                if int_v == 1 and prev_v == 0:
                    count += 1
                prev_v = int_v
            return count

        def first_rise(window):
            """Time of first rising edge in window."""
            start, end = window
            prev_v = parse_vcd_value(value_at_time(tv, start))
            for time, val in tv:
                if time <= start:
                    continue
                if time > end:
                    break
                int_v = parse_vcd_value(val)
                if int_v == 1 and prev_v == 0:
                    return time - start  # relative to window start
                prev_v = int_v
            return None

        def last_fall(window):
            """Time of last falling edge in window (relative)."""
            start, end = window
            prev_v = parse_vcd_value(value_at_time(tv, start))
            last = None
            for time, val in tv:
                if time <= start:
                    continue
                if time > end:
                    break
                int_v = parse_vcd_value(val)
                if int_v == 0 and prev_v == 1:
                    last = time - start
                prev_v = int_v
            return last

        good_pulses = count_pulses(good_win)
        bad_pulses = count_pulses(bad_win)
        good_first = first_rise(good_win)
        bad_first = first_rise(bad_win)
        good_last = last_fall(good_win)
        bad_last = last_fall(bad_win)

        mismatch = " ← DIFFERENT" if good_pulses != bad_pulses else ""
        short_name = sig_name.split('.')[-1]
        print(f"  {short_name:20s}  good: {good_pulses:>3} pulses, "
              f"first@+{good_first}, last_fall@+{good_last}  |  "
              f"bad: {bad_pulses:>3} pulses, first@+{bad_first}, "
              f"last_fall@+{bad_last}{mismatch}")


def main():
    parser = argparse.ArgumentParser(
        description='VCD signal query tool for deimos debug',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    subparsers = parser.add_subparsers(dest='command')

    # list
    p_list = subparsers.add_parser('list', help='List signals matching pattern')
    p_list.add_argument('vcd_file', help='Path to VCD file')
    p_list.add_argument('pattern', nargs='?', default='', help='Filter pattern')

    # extract
    p_extract = subparsers.add_parser('extract', help='Extract signal transitions')
    p_extract.add_argument('vcd_file', help='Path to VCD file')
    p_extract.add_argument('signals', nargs='+', help='Signal name patterns')
    p_extract.add_argument('--start', type=int, help='Start time (ns)')
    p_extract.add_argument('--end', type=int, help='End time (ns)')

    # symbol-timing
    p_timing = subparsers.add_parser('symbol-timing',
                                     help='Per-symbol timing report')
    p_timing.add_argument('vcd_file', help='Path to VCD file')

    # compare-symbols
    p_compare = subparsers.add_parser('compare-symbols',
                                      help='Compare good vs bad symbol')
    p_compare.add_argument('vcd_file', help='Path to VCD file')
    p_compare.add_argument('--good', type=int, required=True,
                           help='Good symbol index')
    p_compare.add_argument('--bad', type=int, required=True,
                           help='Bad symbol index')

    args = parser.parse_args()

    if args.command == 'list':
        cmd_list(args)
    elif args.command == 'extract':
        cmd_extract(args)
    elif args.command == 'symbol-timing':
        cmd_symbol_timing(args)
    elif args.command == 'compare-symbols':
        cmd_compare_symbols(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
