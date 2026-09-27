"""
diag_decode_profile.py — Per-module decode-chain activity profile.

Measures where the per-symbol cycle budget goes during a golden-vector
frame decode.

Method (corrected 2026-09-16):
  * A symbol period is closed ONLY by a `symbol_start` pulse. The window
    still open at `tag_valid` is drain-to-end-of-frame, not a period, and is
    never counted. It is LONGER than a symbol at every rate (~529 clk at
    48M, ~965 at 54M), so a "shorter than a symbol" guard cannot catch it;
    averaging it in once inflated 48/54 clk/sym to 455/568 (see D23).
  * Module cycles are attributed by `decode_engine` symbol index, not by
    wall-clock window. The Viterbi and its `vit_fifo` are decoupled from the
    `symbol_start` cadence, so their activity spills across symbol
    boundaries; per-window attribution clipped that spill at `tag_valid`
    and understated `depct`/`pair`/`vitB`/`vitV` on the last DATA symbols —
    badly at 48/54, which have only 3-5 DATA symbols. After `tag_valid` the
    loop keeps sampling until activity goes idle, so the decoupled tail is
    counted, then divides by the DATA-symbol count. SIGNAL (index 0) is
    excluded.

Columns:
  demap      u_demapper.wide_valid      (1 subcarrier/cycle)
  deint      u_deinterleaver.valid_out  (1 wide word/cycle)
  depunct    u_depuncturer.valid_out    (1 bit/cycle; 2*NDBPS per symbol)
  pair       u_soft_pairer.valid_out    (1 pair/cycle)
  vit_busy   u_viterbi.busy             (find-best + trace + output)
  vit_valid  u_viterbi.valid_out        (decoded bit/cycle)
  any        union of the above
  resid      period minus any_active

Diagnostic — reports only, never asserts, always exits 0.

Run: ./scripts/sim.sh diag_decode_profile
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from frontend_helpers import load_waveform_float, quantize_12bit, \
    s12_to_unsigned, reset_dut

RATES = [6, 9, 12, 18, 24, 36, 48, 54]

# Expected depuncturer valid_out cycles per symbol = 2 * NDBPS (it restores
# the rate-1/2 mother code at 1 bit/clk). Printed as a self-check.
EXPECTED_DEPUNCT = {6: 48, 9: 72, 12: 96, 18: 144,
                    24: 192, 36: 288, 48: 384, 54: 432}

# After tag_valid, keep sampling until the decoupled tail (Viterbi / vit_fifo
# / depuncturer) drains. Bounded by a cycle cap and an idle threshold.
DRAIN_CYCLES = 4000
IDLE_LIMIT = 256

MODULES = ("demap", "deint", "depunct", "pair", "vit_busy", "vit_valid")


def _i(sig):
    try:
        return int(sig.value)
    except (ValueError, AttributeError):
        return 0


async def profile_frame(dut, rate, max_cycles=500000):
    """Feed one golden frame; return per-rate means, or None on no decode.

    Returned dict: n (DATA symbols), total (mean period over closed DATA
    symbols -- the final DATA symbol has no closing pulse, so it contributes
    module counts but not a period), resid, first_demap (mean demap@), and
    one key per module.
    """
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    iq_float = load_waveform_float(rate)
    iq_samples = quantize_12bit(iq_float)
    n_samples = len(iq_samples)

    sample_idx = 0
    VALID_SPACING = 5

    pipe = dut.u_rx_pipeline
    dem = pipe.u_demapper
    dei = pipe.u_deinterleaver
    dep = pipe.u_depuncturer
    sp = pipe.u_soft_pairer
    vit = pipe.u_viterbi
    eng = pipe.u_decode_engine

    def _new():
        d = {m: 0 for m in MODULES}
        d["any"] = 0
        d["total"] = 0
        d["first_demap"] = -1
        return d

    per_idx = {}
    starts = []          # (cycle, symbol_idx) per symbol_start pulse
    cur_idx = -1
    tag_seen = False
    drain = 0
    idle = 0

    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)

        if sample_idx < n_samples and (cycle % VALID_SPACING) == 0:
            re_q, im_q = iq_samples[sample_idx]
            dut.iq_valid_in.value = 1
            dut.iq_i_in.value = s12_to_unsigned(re_q)
            dut.iq_q_in.value = s12_to_unsigned(im_q)
            sample_idx += 1
        else:
            dut.iq_valid_in.value = 0

        try:
            if int(eng.symbol_start_out.value):
                cur_idx = int(eng.symbol_idx_out.value)
                starts.append((cycle, cur_idx))
                per_idx.setdefault(cur_idx, _new())
                idle = 0
        except (ValueError, AttributeError):
            pass

        try:
            vals = {
                "demap": _i(dem.wide_valid),
                "deint": _i(dei.valid_out),
                "depunct": _i(dep.valid_out),
                "pair": _i(sp.valid_out),
                "vit_busy": _i(vit.busy),
                "vit_valid": _i(vit.valid_out),
            }
        except (ValueError, AttributeError):
            vals = {m: 0 for m in MODULES}
        any_active = any(vals.values())

        if cur_idx >= 0:
            d = per_idx[cur_idx]
            if not tag_seen:
                d["total"] += 1
            for m in MODULES:
                d[m] += vals[m]
            d["any"] += 1 if any_active else 0
            if vals["demap"] and d["first_demap"] < 0:
                d["first_demap"] = d["total"]

        idle = 0 if any_active else idle + 1

        if not tag_seen:
            try:
                tag_seen = _i(dut.tag_valid) == 1
            except (ValueError, AttributeError):
                pass
        else:
            drain += 1
            if drain >= DRAIN_CYCLES or idle >= IDLE_LIMIT:
                break

    data_idx = [i for _, i in starts if i >= 1]
    if not data_idx:
        return None

    # Periods from consecutive symbol_start pulses; the SIGNAL->DATA1 delta
    # is excluded. The final DATA symbol has no closing pulse, so it
    # contributes module counts but not a period.
    periods = {i0: c1 - c0 for (c0, i0), (c1, i1) in zip(starts, starts[1:])}
    closed = [i for i in data_idx if i in periods]

    n = len(data_idx)
    res = {"n": n}
    for m in MODULES:
        res[m] = sum(per_idx[i][m] for i in data_idx) / n
    # total/any/resid over the closed subset, so they are consistent.
    if closed:
        res["total"] = sum(periods[i] for i in closed) / len(closed)
        res["any"] = sum(per_idx[i]["any"] for i in closed) / len(closed)
    else:
        res["total"] = float("nan")
        res["any"] = float("nan")
    res["resid"] = res["total"] - res["any"]
    fd = [per_idx[i]["first_demap"] for i in data_idx
          if per_idx[i]["first_demap"] >= 0]
    res["first_demap"] = sum(fd) / len(fd) if fd else 0.0
    return res


@cocotb.test()
async def test_decode_profile(dut):
    """Profile all rates and print the budget decomposition."""
    print("\n" + "=" * 96)
    print(" DECODE-CHAIN ACTIVITY PROFILE (clocks/DATA symbol, golden vectors)")
    print("=" * 96)
    print(f" {'Rate':>4} | {'total':>5} | {'demap':>5} | {'deint':>5} | "
          f"{'depct':>5} | {'pair':>5} | {'vitB':>5} | {'vitV':>5} | "
          f"{'any':>5} | {'resid':>5} | {'demap@':>6} | {'dct_xp':>6}")
    print("-" * 96)

    for rate in RATES:
        res = await profile_frame(dut, rate)
        if res is None:
            print(f" {rate:>4}M | insufficient symbol windows "
                  f"— decode failed?")
            continue
        print(f" {rate:>4}M | {res['total']:>5.0f} | {res['demap']:>5.0f} | "
              f"{res['deint']:>5.0f} | {res['depunct']:>5.0f} | "
              f"{res['pair']:>5.0f} | {res['vit_busy']:>5.0f} | "
              f"{res['vit_valid']:>5.0f} | {res['any']:>5.0f} | "
              f"{res['resid']:>5.0f} | {res['first_demap']:>6.1f} | "
              f"{EXPECTED_DEPUNCT[rate]:>6}")

    print("-" * 96)
    print(" total  — mean period between symbol_start pulses (closed windows only)")
    print(" demap/deint/depunct/pair — valid_out cycles per DATA symbol")
    print(" vitB   — viterbi busy cycles (overlaps chain via vit_fifo)")
    print(" vitV   — viterbi decoded-bit cycles")
    print(" any    — cycles with >=1 module active (chain + viterbi)")
    print(" resid  — cycles with no module active: FFT/EQ/pilot/sched")
    print(" demap@ — cycles from symbol_start to first demap output")
    print(" dct_xp — expected depuncturer cycles = 2*NDBPS (self-check)")
    print("=" * 96 + "\n")
