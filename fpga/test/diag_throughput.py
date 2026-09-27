"""
diag_throughput.py — Per-rate pipeline throughput measurement.

Measures average DATA-symbol cycle count for all 8 legacy OFDM rates
via golden-vector replay at live timing (1 sample per 5 clocks,
100 MHz fabric / 20 MSPS ADC).

Wire-speed budget: 400 clocks/symbol. Rates at or below 400 stream in
real time (the pipeline keeps up with the ADC). Above 400, the decode
engine stalls on backpressure and the rate is NOT realtime streaming.

Diagnostic — reports only, never asserts, always exits 0.

Run: ./scripts/sim.sh diag_throughput
"""

import cocotb

from frontend_helpers import measure_symbol_cycle

RATES = [6, 9, 12, 18, 24, 36, 48, 54]
BUDGET = 400  # clocks/symbol (wire speed)


@cocotb.test()
async def test_throughput_all_rates(dut):
    """Measure clocks/symbol for every rate and report realtime status."""
    print("\n" + "=" * 64)
    print(" PER-RATE PIPELINE THROUGHPUT (golden vectors, live timing)")
    print("=" * 64)
    print(f" {'Rate':>5} | {'clocks/sym':>10} | {'min':>5} | {'max':>5} | "
          f"{'headroom':>8} | verdict")
    print("-" * 64)

    realtime = []
    stalled = []
    for rate in RATES:
        avg, deltas, decoded = await measure_symbol_cycle(dut, rate)
        if avg is None or not decoded:
            print(f" {rate:>4}M | {'no decode':>10} | {'—':>5} | {'—':>5} | "
                  f"{'—':>8} | UNMEASURED")
            continue
        headroom = BUDGET - avg
        verdict = "REALTIME" if avg <= BUDGET else "STALLED"
        print(f" {rate:>4}M | {avg:>10.1f} | {min(deltas):>5} | "
              f"{max(deltas):>5} | {headroom:>+7.1f} | {verdict}")
        if avg <= BUDGET:
            realtime.append(rate)
        else:
            stalled.append(rate)

    print("-" * 64)
    print(f" Budget: {BUDGET} clocks/symbol (wire speed)")
    print(f" Realtime streaming: {realtime if realtime else 'none'}")
    print(f" Stalled:           {stalled if stalled else 'none'}")
    print("=" * 64 + "\n")
