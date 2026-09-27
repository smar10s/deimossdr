"""
Test HIL controller: test mode mux + DDR playback FSM.

Tests:
1. Normal mode: ADC input passes through to output
2. Test mode: playback from simulated DDR
3. Playback done flag asserts after all samples
4. Test mode mux switches cleanly
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

# Import shared AXI-Lite driver
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from axi_lite_driver import AxiLiteMaster

# Register offsets
REG_CONTROL    = 0x00
REG_STATUS     = 0x04
REG_DDR_BASE   = 0x08
REG_PLAY_COUNT = 0x0C
REG_PLAY_PTR   = 0x10


class AXI3ReadSlave:
    """Simulates DDR memory for AXI3 read transactions."""

    def __init__(self, dut, memory=None):
        self.dut = dut
        self.memory = memory or {}

    def write_iq_samples(self, base_addr, samples):
        """Write IQ samples into simulated DDR.
        samples: list of (re, im) tuples, 12-bit signed values.
        Packing: 32-bit word = {8'b0, im[11:0], re[11:0]}
        64-bit beat = {word_odd, word_even}
        """
        for i in range(0, len(samples), 2):
            re0, im0 = samples[i]
            word_even = ((im0 & 0xFFF) << 12) | (re0 & 0xFFF)

            if i + 1 < len(samples):
                re1, im1 = samples[i + 1]
                word_odd = ((im1 & 0xFFF) << 12) | (re1 & 0xFFF)
            else:
                word_odd = 0

            addr = base_addr + (i // 2) * 8
            self.memory[addr] = (word_odd << 32) | (word_even & 0xFFFFFFFF)

    async def run(self):
        """Service AXI3 read requests (respects RREADY)."""
        dut = self.dut
        while True:
            await RisingEdge(dut.clk)
            if int(dut.m_axi_arvalid.value) == 1:
                addr = int(dut.m_axi_araddr.value)
                burst_len = int(dut.m_axi_arlen.value) + 1

                # Accept address
                dut.m_axi_arready.value = 1
                await RisingEdge(dut.clk)
                dut.m_axi_arready.value = 0

                # Send data beats, respecting RREADY
                for i in range(burst_len):
                    beat_addr = addr + i * 8
                    data = self.memory.get(beat_addr, 0)
                    dut.m_axi_rdata.value = data
                    dut.m_axi_rvalid.value = 1
                    dut.m_axi_rlast.value = 1 if (i == burst_len - 1) else 0
                    dut.m_axi_rid.value = 0
                    dut.m_axi_rresp.value = 0
                    # Wait for RREADY handshake
                    while True:
                        await RisingEdge(dut.clk)
                        if int(dut.m_axi_rready.value) == 1:
                            break

                dut.m_axi_rvalid.value = 0
                dut.m_axi_rlast.value = 0
            else:
                dut.m_axi_arready.value = 0


async def reset(dut):
    dut.rst.value = 1
    dut.adc_valid.value = 0
    dut.adc_re.value = 0
    dut.adc_im.value = 0
    dut.m_axi_arready.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rid.value = 0
    dut.m_axi_rresp.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def axi_read_val(axi, addr):
    """Read and return just the data value (discard rresp)."""
    data, _ = await axi.read(addr)
    return data


@cocotb.test()
async def test_normal_mode_passthrough(dut):
    """In normal mode (test_mode=0), ADC input passes to output."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    # Mux is combinational — drive ADC and check output same cycle
    test_samples = [(100, -200), (500, 300), (-1000, 2047), (-2048, 0)]

    for re_val, im_val in test_samples:
        dut.adc_re.value = re_val & 0xFFF
        dut.adc_im.value = im_val & 0xFFF
        dut.adc_valid.value = 1
        await RisingEdge(dut.clk)
        await Timer(1, units="ns")  # Let combinational settle

        assert int(dut.iq_valid.value) == 1
        out_re = int(dut.iq_re.value)
        out_im = int(dut.iq_im.value)
        if out_re >= 2048: out_re -= 4096
        if out_im >= 2048: out_im -= 4096
        exp_re = re_val if abs(re_val) < 2048 else (re_val & 0xFFF) - 4096 * (re_val >= 2048)
        exp_im = im_val if abs(im_val) < 2048 else (im_val & 0xFFF) - 4096 * (im_val >= 2048)
        # Handle 12-bit signed wrap
        if re_val == -2048: exp_re = -2048
        if im_val == -200: exp_im = -200
        assert out_re == re_val, f"re: got {out_re}, expected {re_val}"
        assert out_im == im_val, f"im: got {out_im}, expected {im_val}"

    dut.adc_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, units="ns")
    assert int(dut.iq_valid.value) == 0


@cocotb.test()
async def test_playback_basic(dut):
    """Test mode plays back IQ from simulated DDR and asserts done."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    axi = AxiLiteMaster(dut, "s_axi_", dut.clk)

    ddr_base = 0x10000000
    n_samples = 32

    samples = [(i * 10, -(i * 10)) for i in range(n_samples)]

    mem_slave = AXI3ReadSlave(dut)
    mem_slave.write_iq_samples(ddr_base, samples)
    cocotb.start_soon(mem_slave.run())

    # Configure
    await axi.write(REG_DDR_BASE, ddr_base)
    await axi.write(REG_PLAY_COUNT, n_samples)
    await axi.write(REG_CONTROL, 0x03)  # test_mode=1, trigger=1

    # Wait for playback to complete
    for _ in range(1000):
        await RisingEdge(dut.clk)
        status = await axi_read_val(axi, REG_STATUS)
        if status & 0x02:  # playback_done
            break
    else:
        assert False, "Playback did not complete within timeout"

    ptr = await axi_read_val(axi, REG_PLAY_PTR)
    assert ptr == n_samples, f"play_ptr={ptr}, expected {n_samples}"


@cocotb.test()
async def test_playback_captures_correct_samples(dut):
    """Verify actual IQ values output during playback match DDR content."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    axi = AxiLiteMaster(dut, "s_axi_", dut.clk)

    ddr_base = 0x10000000
    n_samples = 64

    samples = [(i * 5 - 100, 2000 - i * 30) for i in range(n_samples)]

    mem_slave = AXI3ReadSlave(dut)
    mem_slave.write_iq_samples(ddr_base, samples)
    cocotb.start_soon(mem_slave.run())

    await axi.write(REG_DDR_BASE, ddr_base)
    await axi.write(REG_PLAY_COUNT, n_samples)
    await axi.write(REG_CONTROL, 0x03)

    # Capture output samples
    captured = []
    for _ in range(n_samples * 10):
        await RisingEdge(dut.clk)
        await Timer(1, units="ns")
        if int(dut.iq_valid.value) == 1:
            re_out = int(dut.iq_re.value)
            im_out = int(dut.iq_im.value)
            if re_out >= 2048: re_out -= 4096
            if im_out >= 2048: im_out -= 4096
            captured.append((re_out, im_out))
        if len(captured) >= n_samples:
            break

    assert len(captured) == n_samples, \
        f"Captured {len(captured)} samples, expected {n_samples}"

    # Compare
    for i, ((got_re, got_im), (exp_re, exp_im)) in enumerate(zip(captured, samples)):
        exp_re_12 = exp_re & 0xFFF
        if exp_re_12 >= 2048: exp_re_12 -= 4096
        exp_im_12 = exp_im & 0xFFF
        if exp_im_12 >= 2048: exp_im_12 -= 4096

        assert got_re == exp_re_12, \
            f"Sample {i} re: got {got_re}, expected {exp_re_12}"
        assert got_im == exp_im_12, \
            f"Sample {i} im: got {got_im}, expected {exp_im_12}"


@cocotb.test()
async def test_mux_switches_to_adc_when_disabled(dut):
    """After playback, disabling test_mode returns to ADC path."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset(dut)

    axi = AxiLiteMaster(dut, "s_axi_", dut.clk)

    # Enable test mode (no trigger — so no playback valid)
    await axi.write(REG_CONTROL, 0x01)
    await ClockCycles(dut.clk, 2)

    # Drive ADC — should NOT appear (test_mode=1, playback not running)
    dut.adc_re.value = 0x123
    dut.adc_im.value = 0x456
    dut.adc_valid.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, units="ns")
    assert int(dut.iq_valid.value) == 0, \
        "In test mode with no playback, iq_valid should be 0"

    # Switch back to normal mode
    await axi.write(REG_CONTROL, 0x00)
    await RisingEdge(dut.clk)
    await Timer(1, units="ns")

    assert int(dut.iq_valid.value) == 1, \
        "After disabling test mode, ADC should pass through"

