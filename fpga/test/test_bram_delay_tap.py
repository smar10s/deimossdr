"""Verify bram_delay_tap timing matches a shift register reference.

The BRAM circular buffer matches a shift register ONLY after the buffer
has been fully populated (DEPTH writes). Before that, reads from positions
not yet written will return stale data (zero from initialization or wrapped
writes). This is acceptable because stf_detect gates accumulation:
- tap0/tap1 (offsets 0, 16) are only used after sample_cnt >= 18
- tap2/tap3 (offsets 64, 80) are only used after sample_cnt >= 82

This test verifies correct behavior after DEPTH writes (steady state).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


@cocotb.test()
async def test_tap_steady_state_1in5(dut):
    """After buffer is full (81+ writes), taps match shift register exactly."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.rst_n.value = 0
    dut.clear.value = 0
    dut.iq_valid.value = 0
    dut.din.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Reference shift register
    dl = [0] * 81

    errors = 0
    for i in range(150):
        sample = i + 1

        dut.iq_valid.value = 1
        dut.din.value = sample
        await RisingEdge(dut.clk)
        dut.iq_valid.value = 0
        await ClockCycles(dut.clk, 4)
        await Timer(1, units="ns")  # settle

        # Update reference BEFORE checking — after 4 idle clocks the BRAM
        # taps reflect the write that just happened, so dl[0] = current sample.
        dl = [sample] + dl[:-1]

        t0 = int(dut.tap0_out.value)
        t1 = int(dut.tap1_out.value)
        t2 = int(dut.tap2_out.value)
        t3 = int(dut.tap3_out.value)

        # Reference (after shift: dl[0] = current sample)
        ref_tap0 = dl[0]
        ref_tap1 = dl[16]
        ref_tap2 = dl[64]
        ref_tap3 = dl[80]

        # Only check after buffer is fully populated
        if i >= 81:
            if t0 != ref_tap0 or t1 != ref_tap1 or t2 != ref_tap2 or t3 != ref_tap3:
                if errors < 10:
                    dut._log.error(
                        f"Sample {i}: tap0={t0} (exp {ref_tap0}), "
                        f"tap1={t1} (exp {ref_tap1}), "
                        f"tap2={t2} (exp {ref_tap2}), "
                        f"tap3={t3} (exp {ref_tap3})"
                    )
                errors += 1

    assert errors == 0, f"Tap mismatch in {errors} samples (steady state)"


@cocotb.test()
async def test_tap_steady_state_backtoback(dut):
    """Back-to-back iq_valid, verify taps after buffer fills.

    With back-to-back (iq_valid every clock), tap_out read at the rising edge
    reflects the state from the PREVIOUS clock's read address, which was
    computed from the wr_ptr BEFORE the previous write. So taps are effectively
    2 writes behind. The reference model accounts for this by comparing against
    dl[1] (one extra step behind compared to 1-in-5 mode).
    """
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.rst_n.value = 0
    dut.clear.value = 0
    dut.iq_valid.value = 0
    dut.din.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    dl = [0] * 81
    errors = 0

    for i in range(150):
        sample = i + 1

        dut.iq_valid.value = 1
        dut.din.value = sample
        await RisingEdge(dut.clk)
        await Timer(1, units="ns")

        t0 = int(dut.tap0_out.value)
        t1 = int(dut.tap1_out.value)
        t2 = int(dut.tap2_out.value)
        t3 = int(dut.tap3_out.value)

        # In back-to-back mode, tap outputs reflect the read address from the
        # PREVIOUS clock (BRAM 1-clock read latency). That address was based on
        # wr_ptr from the clock before THAT. So taps are 1 sample behind what
        # 1-in-5 mode gives. Reference: dl[0] = previous sample (before shift).
        ref_tap0 = dl[0]
        ref_tap1 = dl[16]
        ref_tap2 = dl[64]
        ref_tap3 = dl[80]

        # Only verify after buffer has wrapped + BRAM latency settled
        if i >= 82:
            if t0 != ref_tap0 or t1 != ref_tap1 or t2 != ref_tap2 or t3 != ref_tap3:
                if errors < 10:
                    dut._log.error(
                        f"Sample {i}: tap0={t0} (exp {ref_tap0}), "
                        f"tap1={t1} (exp {ref_tap1}), "
                        f"tap2={t2} (exp {ref_tap2}), "
                        f"tap3={t3} (exp {ref_tap3})"
                    )
                errors += 1

        dl = [sample] + dl[:-1]

    assert errors == 0, f"Tap mismatch in {errors} samples (steady state, back-to-back)"


@cocotb.test()
async def test_tap0_early_fill(dut):
    """Verify tap0 (offset 0) returns current sample from write 2 onward (1-in-5)."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.rst_n.value = 0
    dut.clear.value = 0
    dut.iq_valid.value = 0
    dut.din.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    errors = 0

    for i in range(100):
        sample = i + 1

        dut.iq_valid.value = 1
        dut.din.value = sample
        await RisingEdge(dut.clk)
        dut.iq_valid.value = 0
        await ClockCycles(dut.clk, 4)
        await Timer(1, units="ns")

        t0 = int(dut.tap0_out.value)

        # With 1-in-5 spacing, after 4 idle clocks, tap0 reflects the
        # sample just written in this iteration (the BRAM read settles).
        if i >= 1:
            expected = sample
            if t0 != expected:
                if errors < 5:
                    dut._log.error(f"i={i}: tap0={t0}, expected {expected}")
                errors += 1

    assert errors == 0, f"tap0 mismatch in {errors} samples"
