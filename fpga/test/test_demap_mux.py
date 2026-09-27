"""
Test demap_mux — Demapper input mux (SIGNAL bypass for pilot_track).

Selects equalizer output during the SIGNAL symbol (is_signal=1, zero-latency
path) and pilot_track-corrected output for DATA symbols (is_signal=0).

Regression guard: the original implementation selected on symbol_idx==0,
which re-asserts the SIGNAL path when the DATA symbol counter wraps
(256th DATA symbol; reached at 6M >= 763 B). Selecting on is_signal
(decoder FSM state, not the counter) cannot wrap. The long-stream test
drives the DATA select for >256 cycles to pin this class of bug.

Tests:
1. SIGNAL symbol: selects equalizer path (valid + data)
2. DATA symbol: selects pilot_track path (valid + data)
3. Valid passthrough: output valid follows the selected path only
4. Long-frame wrap regression: 300-cycle DATA stream never selects eq path
"""

import cocotb
from cocotb.triggers import Timer


def s16(v):
    """Interpret 16-bit value as signed."""
    v = int(v) & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


def u16(v):
    """Convert signed int to 16-bit unsigned representation."""
    if v < 0:
        v += 0x10000
    return v & 0xFFFF


def drive(dut, is_signal, eq_valid, eq_re, eq_im, pt_valid, pt_re, pt_im):
    dut.is_signal.value = is_signal
    dut.eq_data_valid.value = eq_valid
    dut.eq_data_re.value = u16(eq_re)
    dut.eq_data_im.value = u16(eq_im)
    dut.pt_data_valid.value = pt_valid
    dut.pt_data_re.value = u16(pt_re)
    dut.pt_data_im.value = u16(pt_im)


def read(dut):
    return (
        int(dut.valid_out.value),
        s16(int(dut.re_out.value)),
        s16(int(dut.im_out.value)),
    )


@cocotb.test()
async def test_signal_selects_eq_path(dut):
    """is_signal=1: output follows equalizer path, both paths valid."""
    drive(dut, is_signal=1, eq_valid=1, eq_re=1234, eq_im=-5678,
          pt_valid=1, pt_re=-9999, pt_im=8888)
    await Timer(1, units="ns")
    v, re, im = read(dut)
    assert v == 1, f"SIGNAL: expected valid, got {v}"
    assert re == 1234, f"SIGNAL: expected eq re=1234, got {re}"
    assert im == -5678, f"SIGNAL: expected eq im=-5678, got {im}"

    dut._log.info("SIGNAL symbol selects equalizer path")


@cocotb.test()
async def test_data_selects_pt_path(dut):
    """is_signal=0: output follows pilot_track path, both paths valid."""
    drive(dut, is_signal=0, eq_valid=1, eq_re=1234, eq_im=-5678,
          pt_valid=1, pt_re=-9999, pt_im=8888)
    await Timer(1, units="ns")
    v, re, im = read(dut)
    assert v == 1, f"DATA: expected valid, got {v}"
    assert re == -9999, f"DATA: expected pt re=-9999, got {re}"
    assert im == 8888, f"DATA: expected pt im=8888, got {im}"

    dut._log.info("DATA symbol selects pilot_track path")


@cocotb.test()
async def test_valid_follows_selected_path(dut):
    """Output valid must come from the selected path only."""
    # is_signal=1, eq invalid but pt valid -> output invalid
    drive(dut, is_signal=1, eq_valid=0, eq_re=0, eq_im=0,
          pt_valid=1, pt_re=1111, pt_im=2222)
    await Timer(1, units="ns")
    v, _, _ = read(dut)
    assert v == 0, f"SIGNAL with eq_valid=0: expected invalid, got {v}"

    # is_signal=0, pt invalid but eq valid -> output invalid
    drive(dut, is_signal=0, eq_valid=1, eq_re=3333, eq_im=4444,
          pt_valid=0, pt_re=0, pt_im=0)
    await Timer(1, units="ns")
    v, _, _ = read(dut)
    assert v == 0, f"DATA with pt_valid=0: expected invalid, got {v}"

    dut._log.info("valid passthrough follows selected path")


@cocotb.test()
async def test_long_data_stream_no_signal_bypass(dut):
    """300-cycle DATA stream: output always follows pt path (wrap regression).

    Original bug: symbol_idx==0 select re-asserted the SIGNAL path when the
    8-bit DATA symbol counter wrapped at the 256th DATA symbol. is_signal
    (FSM-derived) stays 0 for the entire DATA burst; pin that here.
    """
    for i in range(300):
        drive(dut, is_signal=0, eq_valid=1, eq_re=0x4000, eq_im=-0x4000,
              pt_valid=1, pt_re=i - 150, pt_im=-(i - 150))
        await Timer(1, units="ns")
        v, re, im = read(dut)
        assert v == 1, f"cycle {i}: expected valid DATA output"
        assert re == i - 150, \
            f"cycle {i}: eq path selected during DATA symbol (re={re}, want {i - 150})"
        assert im == -(i - 150), \
            f"cycle {i}: eq path selected during DATA symbol (im={im}, want {-(i - 150)})"

    dut._log.info("300-cycle DATA stream never bypasses pilot_track")
