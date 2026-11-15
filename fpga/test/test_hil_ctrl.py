# SPDX-License-Identifier: MIT
"""Cocotb tests for hil_ctrl.v — Hardware-in-the-Loop test controller.

Tests:
    1. Normal mode passthrough — test_mode=0: ADC passes through
    2. Playback basic — load DDR, strobe, verify output
    3. Playback cadence — /5, ~1.6, and irregular strobe patterns
    4. Playback done — after play_count samples, playback_done asserts
    5. Fill gate — playback waits for the FIFO to fill
    6. Underrun — a starved FIFO is counted and can be cleared
    7. Mux boundary — test_mode takes effect only on a strobe
    8. ADC valid counter — ADC_CNT increments in live and test mode
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster
from axi3_mocks import Axi3ReadSlave

# Register offsets
REG_CONTROL    = 0x00
REG_STATUS     = 0x04
REG_DDR_BASE   = 0x08
REG_PLAY_COUNT = 0x0C
REG_PLAY_PTR   = 0x10
REG_UNDERRUN   = 0x14
# 0x18–0x38: reserved for downstream extension
REG_ADC_CNT    = 0x3C

# DDR base address
DDR_BASE = 0x10000000

# RTL fill threshold (mirror of hil_ctrl localparam)
FILL_LEVEL = 192


class StrobeDriver:
    """Drive adc_valid with a repeating cycle-gap pattern.

    A gap is the number of clock cycles from one valid pulse to the next.
    A gap of 1 gives back-to-back valid cycles.
    """

    def __init__(self, dut, pattern):
        self.dut = dut
        self.pattern = pattern
        self._running = False
        self._task = None

    async def _run(self):
        idx = 0
        while self._running:
            gap = self.pattern[idx % len(self.pattern)]
            self.dut.adc_valid.value = 0
            for _ in range(max(gap - 1, 0)):
                await RisingEdge(self.dut.clk)
            self.dut.adc_valid.value = 1
            await RisingEdge(self.dut.clk)
            idx += 1
        self.dut.adc_valid.value = 0

    def start(self):
        self._running = True
        self._task = cocotb.start_soon(self._run())

    async def stop(self):
        self._running = False
        if self._task is not None:
            await self._task
        self.dut.adc_valid.value = 0


async def setup(dut, rvalid_delay=0):
    """Start clock, reset, create AXI-Lite + AXI3 read slave."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst.value = 1
    dut.adc_valid.value = 0
    dut.adc_re.value = 0
    dut.adc_im.value = 0
    # AXI-Lite
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    # AXI3 read slave
    dut.m_axi_arready.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rid.value = 0

    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    slave = Axi3ReadSlave(dut, prefix="m_axi_", clk=dut.clk,
                          rvalid_delay=rvalid_delay)
    cocotb.start_soon(slave.run())

    return axi, slave


async def trigger_playback(axi, ddr_base, play_count):
    """Configure and trigger a playback."""
    await axi.write(REG_DDR_BASE, ddr_base)
    await axi.write(REG_PLAY_COUNT, play_count)
    # Set test_mode=1, trigger=1 (bit[1] W1S)
    await axi.write(REG_CONTROL, 0x03)


async def pulse_adc(dut, re, im):
    """Drive one live ADC strobe."""
    dut.adc_valid.value = 1
    dut.adc_re.value = re & 0xFFF
    dut.adc_im.value = im & 0xFFF
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.adc_valid.value = 0


async def collect_iq_output(dut, count, timeout_cycles=20000):
    """Collect IQ output samples. Returns list of (re, im) signed 12-bit."""
    results = []
    cycles = 0
    while len(results) < count and cycles < timeout_cycles:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.iq_valid.value):
            re = int(dut.iq_re.value)
            im = int(dut.iq_im.value)
            if re >= 0x800:
                re -= 0x1000
            if im >= 0x800:
                im -= 0x1000
            results.append((re, im))
        cycles += 1
    return results


async def wait_for_output(dut, timeout_cycles=20000):
    """Wait until the first valid output sample. Returns True on success."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.iq_valid.value):
            return True
    return False


async def wait_test_mode(dut, timeout_cycles=100):
    """Wait for the registered mux select to latch test mode."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.test_mode_out.value):
            return
    assert False, "test_mode did not latch on a strobe"


def to_signed_12(val):
    """Convert unsigned 12-bit to signed."""
    val = val & 0xFFF
    return val - 0x1000 if val >= 0x800 else val


@cocotb.test()
async def test_normal_mode_passthrough(dut):
    """test_mode=0: adc_valid/re/im pass through to iq_valid/re/im."""
    axi, slave = await setup(dut)

    test_data = [(100, -200), (500, 1000), (-1000, 2047)]

    for re, im in test_data:
        dut.adc_valid.value = 1
        dut.adc_re.value = re & 0xFFF
        dut.adc_im.value = im & 0xFFF
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        assert int(dut.iq_valid.value) == 1, "iq_valid not asserted in passthrough"
        got_re = to_signed_12(int(dut.iq_re.value))
        got_im = to_signed_12(int(dut.iq_im.value))
        assert got_re == re, f"re passthrough: expected {re}, got {got_re}"
        assert got_im == im, f"im passthrough: expected {im}, got {got_im}"

    dut.adc_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.iq_valid.value) == 0, "iq_valid stuck high"


async def playback_with_pattern(dut, pattern):
    """Play a short vector at a strobe pattern and verify every sample."""
    axi, slave = await setup(dut)

    play_count = 32
    samples = [(i * 10, -(i * 10)) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    strobe = StrobeDriver(dut, pattern)
    strobe.start()
    await trigger_playback(axi, DDR_BASE, play_count)
    await wait_test_mode(dut)

    results = await collect_iq_output(dut, play_count)
    await strobe.stop()

    assert len(results) == play_count, \
        f"Got {len(results)} samples, expected {play_count}"

    for i, (exp_re, exp_im) in enumerate(samples):
        got_re, got_im = results[i]
        assert got_re == exp_re, f"Sample {i} re: expected {exp_re}, got {got_re}"
        assert got_im == exp_im, f"Sample {i} im: expected {exp_im}, got {got_im}"


@cocotb.test()
async def test_playback_20msps_cadence(dut):
    """Playback follows a 1-in-5 strobe (20 MSPS into 100 MHz)."""
    await playback_with_pattern(dut, [5])


@cocotb.test()
async def test_playback_61msps_cadence(dut):
    """Playback follows an average 1.6-cycle strobe (61.44 MSPS)."""
    await playback_with_pattern(dut, [2, 1, 2, 2, 1])


@cocotb.test()
async def test_playback_irregular_cadence(dut):
    """Playback follows an irregular strobe pattern without shifting."""
    await playback_with_pattern(dut, [3, 1, 7, 2, 1, 4])


@cocotb.test()
async def test_playback_stops_with_strobe(dut):
    """Playback does not advance without an ADC strobe."""
    axi, slave = await setup(dut)

    play_count = 32
    samples = [(i, i) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    strobe = StrobeDriver(dut, [2])
    strobe.start()
    await trigger_playback(axi, DDR_BASE, play_count)
    await wait_test_mode(dut)

    results = await collect_iq_output(dut, 8)
    assert len(results) == 8, f"Got {len(results)} samples before stopping"

    await strobe.stop()
    for _ in range(100):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.iq_valid.value):
            results.append((0, 0))

    assert len(results) == 8, \
        f"Output advanced without a strobe ({len(results)} samples)"


@cocotb.test()
async def test_playback_done(dut):
    """After play_count samples output, playback_done asserts."""
    axi, slave = await setup(dut)

    play_count = 32
    samples = [(i, i) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    strobe = StrobeDriver(dut, [5])
    strobe.start()
    await trigger_playback(axi, DDR_BASE, play_count)
    await wait_test_mode(dut)

    results = await collect_iq_output(dut, play_count)
    assert len(results) == play_count
    await strobe.stop()

    for _ in range(20):
        await RisingEdge(dut.clk)

    status, _ = await axi.read(REG_STATUS)
    done = (status >> 1) & 1
    assert done == 1, f"playback_done not set: STATUS=0x{status:08X}"


@cocotb.test()
async def test_playback_fill_gate(dut):
    """Playback waits for FILL_LEVEL samples before the first output."""
    axi, slave = await setup(dut, rvalid_delay=20)

    play_count = 512
    samples = [(i, -i) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    strobe = StrobeDriver(dut, [5])
    strobe.start()
    await trigger_playback(axi, DDR_BASE, play_count)
    await wait_test_mode(dut)

    for _ in range(1000):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        assert int(dut.iq_valid.value) == 0, \
            "Playback started before the FIFO filled"

    underrun, _ = await axi.read(REG_UNDERRUN)
    assert underrun == 0, f"Underrun during fill: {underrun}"

    assert await wait_for_output(dut, timeout_cycles=5000), \
        "Playback did not start after the FIFO filled"
    await strobe.stop()


@cocotb.test()
async def test_playback_underrun(dut):
    """A starved FIFO increments the sticky underrun counter."""
    axi, slave = await setup(dut, rvalid_delay=4)

    play_count = 512
    samples = [(i, -i) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    strobe = StrobeDriver(dut, [1])
    strobe.start()
    await trigger_playback(axi, DDR_BASE, play_count)
    await wait_test_mode(dut)

    assert await wait_for_output(dut, timeout_cycles=10000), \
        "Playback did not start"

    slave.stall_rvalid = True
    for _ in range(1500):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

    await strobe.stop()
    underrun, _ = await axi.read(REG_UNDERRUN)
    assert underrun > 0, "Underrun counter did not increment"

    await axi.write(REG_UNDERRUN, 1)
    underrun, _ = await axi.read(REG_UNDERRUN)
    assert underrun == 0, f"Underrun counter did not clear: {underrun}"


@cocotb.test()
async def test_mux_waits_for_strobe(dut):
    """test_mode takes effect only after an ADC strobe."""
    axi, slave = await setup(dut)

    await pulse_adc(dut, 111, 222)
    assert int(dut.iq_valid.value) == 1
    assert int(dut.test_mode_out.value) == 0

    await axi.write(REG_CONTROL, 0x01)
    assert int(dut.test_mode_out.value) == 0, \
        "test_mode changed without a strobe"

    await pulse_adc(dut, 333, 444)
    assert int(dut.test_mode_out.value) == 1, \
        "test_mode did not latch on a strobe"
    assert int(dut.iq_valid.value) == 0, \
        "ADC leaked through after test mode latched"

    await axi.write(REG_CONTROL, 0x00)
    assert int(dut.test_mode_out.value) == 1, \
        "test_mode cleared without a strobe"

    await pulse_adc(dut, 555, 666)
    assert int(dut.test_mode_out.value) == 0, \
        "test_mode did not clear on a strobe"

    await pulse_adc(dut, 777, 888)
    assert int(dut.iq_valid.value) == 1, "ADC did not return after switch"
    assert to_signed_12(int(dut.iq_re.value)) == 777


@cocotb.test()
async def test_adc_valid_counter(dut):
    """In live mode, verify ADC_CNT register increments on each adc_valid."""
    axi, slave = await setup(dut)

    cnt_before, _ = await axi.read(REG_ADC_CNT)

    num_pulses = 20
    for _ in range(num_pulses):
        dut.adc_valid.value = 1
        dut.adc_re.value = 0
        dut.adc_im.value = 0
        await RisingEdge(dut.clk)
    dut.adc_valid.value = 0

    for _ in range(3):
        await RisingEdge(dut.clk)

    cnt_after, _ = await axi.read(REG_ADC_CNT)

    delta = cnt_after - cnt_before
    assert delta == num_pulses, \
        f"ADC_CNT delta = {delta}, expected {num_pulses}"


@cocotb.test()
async def test_adc_valid_counter_in_test_mode(dut):
    """ADC_CNT keeps counting while test mode is selected."""
    axi, slave = await setup(dut)

    await axi.write(REG_CONTROL, 0x01)
    await pulse_adc(dut, 0, 0)
    assert int(dut.test_mode_out.value) == 1

    cnt_before, _ = await axi.read(REG_ADC_CNT)

    num_pulses = 20
    for _ in range(num_pulses):
        await pulse_adc(dut, 0, 0)

    cnt_after, _ = await axi.read(REG_ADC_CNT)

    delta = cnt_after - cnt_before
    assert delta == num_pulses, \
        f"ADC_CNT delta in test mode = {delta}, expected {num_pulses}"


@cocotb.test()
async def test_read_while_rvalid_high(dut):
    """Back-to-back AR transactions: second AR must not be lost when RVALID is high.

    Regression test for the hil_ctrl AXI-Lite read lockup bug: without the
    ~s_axi_rvalid guard on ARREADY, a second AR accepted while RVALID is
    still asserted gets consumed but never serviced, deadlocking the bus.
    """
    axi, slave = await setup(dut)

    await axi.write(REG_DDR_BASE, 0xCAFEBABE)
    await axi.write(REG_PLAY_COUNT, 42)

    results = []
    addrs = [REG_DDR_BASE, REG_PLAY_COUNT, REG_DDR_BASE, REG_PLAY_COUNT]

    for addr in addrs:
        val, _ = await axi.read(addr)
        results.append(val)

    assert results[0] == 0xCAFEBABE, f"Read 0: expected 0xCAFEBABE, got 0x{results[0]:08X}"
    assert results[1] == 42, f"Read 1: expected 42, got {results[1]}"
    assert results[2] == 0xCAFEBABE, f"Read 2: expected 0xCAFEBABE, got 0x{results[2]:08X}"
    assert results[3] == 42, f"Read 3: expected 42, got {results[3]}"
