"""Cocotb tests for iq_dma_rx — continuous IQ DMA writer."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from axi_lite_driver import AxiLiteMaster

# Register offsets
REG_CONTROL      = 0x00
REG_STATUS       = 0x04
REG_DDR_BASE     = 0x08
REG_WR_PTR       = 0x0C
REG_SAMPLE_COUNT = 0x10
REG_WRAP_COUNT   = 0x14

DDR_BASE_ADDR = 0x2000_0000


# =========================================================
# AXI3 Write Slave Mock
# =========================================================
class Axi3WriteSlave:
    """Mock AXI3 write slave that captures burst writes into a dict."""

    def __init__(self, dut, prefix="m_axi_", clk=None, wready_delay=0):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix
        self.mem = {}  # byte_addr -> byte value
        self.bursts = []  # list of (base_addr, [beat_data, ...])
        self.wready_delay = wready_delay  # cycles to stall WREADY
        self.stall_wready = False  # if True, never assert WREADY

    def _sig(self, name):
        return getattr(self.dut, self._p + name)

    def read_word(self, byte_addr):
        """Read a 32-bit word from the memory model (little-endian)."""
        val = 0
        for i in range(4):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    def read_dword(self, byte_addr):
        """Read a 64-bit doubleword from the memory model (little-endian)."""
        val = 0
        for i in range(8):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    async def run(self):
        """Main loop — run as cocotb.start_soon(slave.run())."""
        # Initialize slave-driven signals
        self._sig("awready").value = 0
        self._sig("wready").value = 0
        self._sig("bresp").value = 0
        self._sig("bvalid").value = 0

        while True:
            # Wait for AWVALID (handle X during reset)
            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._sig("awvalid").value) == 1:
                        break
                except ValueError:
                    pass  # Signal is X/Z during reset

            # Capture address
            base_addr = int(self._sig("awaddr").value)
            awlen = int(self._sig("awlen").value)  # number of beats - 1
            burst_len = awlen + 1

            # Accept AW
            self._sig("awready").value = 1
            await RisingEdge(self.clk)
            self._sig("awready").value = 0

            # Receive W beats
            beat_data = []
            for beat_idx in range(burst_len):
                # Apply WREADY delay / stall
                if self.stall_wready:
                    # Never assert WREADY — infinite stall
                    while True:
                        await RisingEdge(self.clk)
                        if not self.stall_wready:
                            break
                elif self.wready_delay > 0:
                    for _ in range(self.wready_delay):
                        await RisingEdge(self.clk)

                self._sig("wready").value = 1

                # Wait for WVALID
                while True:
                    await RisingEdge(self.clk)
                    if int(self._sig("wvalid").value) == 1:
                        break

                data = int(self._sig("wdata").value)
                wlast = int(self._sig("wlast").value)
                beat_data.append(data)

                # Store in memory (little-endian, 8 bytes per beat)
                byte_addr = base_addr + beat_idx * 8
                for i in range(8):
                    self.mem[byte_addr + i] = (data >> (8 * i)) & 0xFF

                self._sig("wready").value = 0

                # Check WLAST on final beat
                if beat_idx == burst_len - 1:
                    assert wlast == 1, f"Expected WLAST on beat {beat_idx}"

            self.bursts.append((base_addr, beat_data))

            # Send B response
            self._sig("bresp").value = 0  # OKAY
            self._sig("bvalid").value = 1

            # Wait for BREADY
            while True:
                await RisingEdge(self.clk)
                if int(self._sig("bready").value) == 1:
                    break

            self._sig("bvalid").value = 0


# =========================================================
# IQ Sample Feeder
# =========================================================
async def feed_iq(dut, samples, clk_period_ratio=5):
    """Feed IQ samples. Each sample is (re, im) tuple, 12-bit unsigned.

    clk_period_ratio=5 means valid_in fires every 5th clock (20 MSPS into 100 MHz).
    """
    for re_val, im_val in samples:
        dut.re_in.value = re_val & 0xFFF
        dut.im_in.value = im_val & 0xFFF
        dut.valid_in.value = 1
        await RisingEdge(dut.clk)
        dut.valid_in.value = 0
        for _ in range(clk_period_ratio - 1):
            await RisingEdge(dut.clk)


# =========================================================
# Common setup / reset
# =========================================================
async def reset(dut):
    """Drive rst=1 for 5 cycles, deassert, wait 3 cycles."""
    dut.re_in.value = 0
    dut.im_in.value = 0
    dut.valid_in.value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 3)


async def setup(dut):
    """Start clock, reset, return (axi_lite_master, axi3_slave)."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    await axi._init_signals()

    slave = Axi3WriteSlave(dut, prefix="m_axi_", clk=dut.clk)
    cocotb.start_soon(slave.run())

    await reset(dut)
    return axi, slave


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_basic_burst(dut):
    """Enable DMA, feed 32 samples, verify 1 AXI burst at correct address."""
    axi, slave = await setup(dut)

    # Configure DDR base and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)  # enable

    # Feed 32 samples: (re=i, im=i+100) for i in 0..31
    samples = [(i, i + 100) for i in range(32)]
    await feed_iq(dut, samples)

    # Wait for burst to complete
    await ClockCycles(dut.clk, 200)

    # Verify exactly 1 burst occurred
    assert len(slave.bursts) == 1, f"Expected 1 burst, got {len(slave.bursts)}"

    # Verify burst address
    burst_addr, beat_data = slave.bursts[0]
    assert burst_addr == DDR_BASE_ADDR, \
        f"Expected burst at 0x{DDR_BASE_ADDR:08x}, got 0x{burst_addr:08x}"

    # Verify burst length (16 beats)
    assert len(beat_data) == 16, f"Expected 16 beats, got {len(beat_data)}"

    # Verify data packing for first beat (samples 0 and 1)
    # Even sample (idx 0): re=0, im=100 -> {8'b0, im[11:0], re[11:0]} = 0x00_064_000
    # Odd sample (idx 1):  re=1, im=101 -> {8'b0, im[11:0], re[11:0]} = 0x00_065_001
    # 64-bit beat: {odd_word, even_word}
    even_word = (100 << 12) | 0  # 0x00064000
    odd_word = (101 << 12) | 1   # 0x00065001
    expected_beat0 = (odd_word << 32) | even_word
    assert beat_data[0] == expected_beat0, \
        f"Beat 0: expected 0x{expected_beat0:016x}, got 0x{beat_data[0]:016x}"


@cocotb.test()
async def test_continuous_wrap(dut):
    """Feed 64 samples (2 bursts), verify addresses and wr_ptr."""
    # This test assumes DDR_BUF_SAMPLES > 64 (no wrap during 2 bursts)
    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    if buf_samples <= 64:
        cocotb.log.info(f"Skipping: DDR_BUF_SAMPLES={buf_samples} (need >64 for linear test)")
        return

    axi, slave = await setup(dut)

    # Configure DDR base and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)  # enable

    # Feed 64 samples
    samples = [(i, i + 200) for i in range(64)]
    await feed_iq(dut, samples)

    # Wait for both bursts to complete
    await ClockCycles(dut.clk, 400)

    # Verify 2 bursts
    assert len(slave.bursts) == 2, f"Expected 2 bursts, got {len(slave.bursts)}"

    # First burst at DDR_BASE + 0
    assert slave.bursts[0][0] == DDR_BASE_ADDR, \
        f"Burst 0 addr: expected 0x{DDR_BASE_ADDR:08x}, got 0x{slave.bursts[0][0]:08x}"

    # Second burst at DDR_BASE + 128 (32 samples * 4 bytes each)
    expected_addr1 = DDR_BASE_ADDR + 128
    assert slave.bursts[1][0] == expected_addr1, \
        f"Burst 1 addr: expected 0x{expected_addr1:08x}, got 0x{slave.bursts[1][0]:08x}"

    # Verify wr_ptr = 64
    wr_ptr, _ = await axi.read(REG_WR_PTR)
    assert wr_ptr == 64, f"Expected wr_ptr=64, got {wr_ptr}"


@cocotb.test()
async def test_sample_packing(dut):
    """Feed known I/Q values, verify 64-bit beat packing format."""
    axi, slave = await setup(dut)

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Feed 32 samples with distinctive values
    # re = 0xABC (2748), im = 0x123 (291) for even samples
    # re = 0xDEF (3567), im = 0x456 (1110) for odd samples
    samples = []
    for i in range(16):
        samples.append((0xABC, 0x123))  # even
        samples.append((0xDEF, 0x456))  # odd
    await feed_iq(dut, samples)

    # Wait for burst
    await ClockCycles(dut.clk, 200)

    assert len(slave.bursts) >= 1, "No burst received"

    # Check every beat in the burst
    _, beat_data = slave.bursts[0]
    for beat_idx in range(16):
        beat = beat_data[beat_idx]
        # Lower 32 bits: even sample = {8'b0, im_even[11:0], re_even[11:0]}
        low32 = beat & 0xFFFFFFFF
        expected_low = (0x123 << 12) | 0xABC  # 0x00123ABC
        assert low32 == expected_low, \
            f"Beat {beat_idx} low32: expected 0x{expected_low:08x}, got 0x{low32:08x}"

        # Upper 32 bits: odd sample = {8'b0, im_odd[11:0], re_odd[11:0]}
        high32 = (beat >> 32) & 0xFFFFFFFF
        expected_high = (0x456 << 12) | 0xDEF  # 0x00456DEF
        assert high32 == expected_high, \
            f"Beat {beat_idx} high32: expected 0x{expected_high:08x}, got 0x{high32:08x}"


@cocotb.test()
async def test_enable_disable(dut):
    """DMA disabled: no bursts. Enable: burst fires."""
    axi, slave = await setup(dut)

    # Configure DDR base but do NOT enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)

    # Feed 32 samples with DMA disabled
    samples = [(i, i) for i in range(32)]
    await feed_iq(dut, samples)
    await ClockCycles(dut.clk, 200)

    # No bursts should have fired
    assert len(slave.bursts) == 0, \
        f"Expected 0 bursts while disabled, got {len(slave.bursts)}"

    # Now enable
    await axi.write(REG_CONTROL, 1)

    # Feed 32 more samples
    await feed_iq(dut, samples)
    await ClockCycles(dut.clk, 200)

    # Should have 1 burst now
    assert len(slave.bursts) == 1, \
        f"Expected 1 burst after enable, got {len(slave.bursts)}"


@cocotb.test()
async def test_overflow_count(dut):
    """Stall AXI WREADY, feed samples until overflow_count increments."""
    axi, slave = await setup(dut)

    # Configure and enable with stalled WREADY
    slave.stall_wready = True
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Feed 32 samples to fill first buffer half (triggers burst attempt)
    samples = [(i, i) for i in range(32)]
    await feed_iq(dut, samples)

    # Feed 32 more samples to fill second buffer half
    # Now both halves are occupied and burst is stalled
    await feed_iq(dut, samples)

    # Feed more samples — these should overflow
    overflow_samples = [(i, i) for i in range(64)]
    await feed_iq(dut, overflow_samples)

    # Check STATUS for overflow_count > 0
    status, _ = await axi.read(REG_STATUS)
    overflow_count = (status >> 16) & 0xFFFF
    assert overflow_count > 0, \
        f"Expected overflow_count > 0, got {overflow_count} (STATUS=0x{status:08x})"


@cocotb.test()
async def test_sample_count_wrap(dut):
    """Verify WRAP_COUNT register exists and reads 0 after reset."""
    axi, slave = await setup(dut)

    # Read WRAP_COUNT register — should be 0 after reset
    wrap_count, _ = await axi.read(REG_WRAP_COUNT)
    assert wrap_count == 0, \
        f"Expected wrap_count=0 after reset, got {wrap_count}"

    # Feed some samples and verify wrap_count stays 0 (nowhere near 4G)
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    samples = [(i, i + 50) for i in range(32)]
    await feed_iq(dut, samples)
    await ClockCycles(dut.clk, 200)

    wrap_count, _ = await axi.read(REG_WRAP_COUNT)
    assert wrap_count == 0, \
        f"Expected wrap_count=0 after 32 samples, got {wrap_count}"


@cocotb.test()
async def test_no_burst_without_ddr_base(dut):
    """DMA enabled without DDR_BASE written: no bursts. After writing DDR_BASE: burst fires."""
    axi, slave = await setup(dut)

    # Enable DMA without writing DDR_BASE first
    await axi.write(REG_CONTROL, 1)

    # Feed 64 samples (more than enough for 2 bursts normally)
    samples = [(i, i + 10) for i in range(64)]
    await feed_iq(dut, samples)
    await ClockCycles(dut.clk, 200)

    # No bursts should have fired — base_configured is 0
    assert len(slave.bursts) == 0, \
        f"Expected 0 bursts without DDR_BASE configured, got {len(slave.bursts)}"

    # Now write DDR_BASE — this sets base_configured
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)

    # Feed 32 more samples — should produce a burst now
    samples = [(i, i + 20) for i in range(32)]
    await feed_iq(dut, samples)
    await ClockCycles(dut.clk, 200)

    # Should have exactly 1 burst
    assert len(slave.bursts) == 1, \
        f"Expected 1 burst after DDR_BASE configured, got {len(slave.bursts)}"

    # Verify burst address is correct
    burst_addr, _ = slave.bursts[0]
    assert burst_addr == DDR_BASE_ADDR, \
        f"Expected burst at 0x{DDR_BASE_ADDR:08x}, got 0x{burst_addr:08x}"


@cocotb.test()
async def test_ddr_wrap_boundary(dut):
    """With DDR_BUF_SAMPLES=64, verify write pointer wraps after 2 bursts.

    Requires parameter override: -Piq_dma_rx.DDR_BUF_SAMPLES=64
    Run via: make test_iq_dma_rx_wrap
    """
    # Skip if DDR_BUF_SAMPLES is the default large value (not overridden)
    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    if buf_samples > 128:
        cocotb.log.info(f"Skipping: DDR_BUF_SAMPLES={buf_samples} (need <=128 for wrap test)")
        return

    axi, slave = await setup(dut)

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Feed 96 samples (3 bursts worth: 32 samples per burst)
    samples = [(i, i + 50) for i in range(96)]
    await feed_iq(dut, samples)

    # Wait for all bursts to complete
    await ClockCycles(dut.clk, 600)

    # Expect 3 bursts total
    assert len(slave.bursts) == 3, \
        f"Expected 3 bursts, got {len(slave.bursts)}"

    # First burst at DDR_BASE + 0
    assert slave.bursts[0][0] == DDR_BASE_ADDR, \
        f"Burst 0 addr: expected 0x{DDR_BASE_ADDR:08x}, got 0x{slave.bursts[0][0]:08x}"

    # Second burst at DDR_BASE + 128 (32 samples * 4 bytes each)
    expected_addr1 = DDR_BASE_ADDR + 128
    assert slave.bursts[1][0] == expected_addr1, \
        f"Burst 1 addr: expected 0x{expected_addr1:08x}, got 0x{slave.bursts[1][0]:08x}"

    # Third burst wraps back to DDR_BASE + 0
    assert slave.bursts[2][0] == DDR_BASE_ADDR, \
        f"Burst 2 addr: expected 0x{DDR_BASE_ADDR:08x} (wrap), got 0x{slave.bursts[2][0]:08x}"

    # Verify wr_ptr = 32 (one burst past wrap)
    wr_ptr, _ = await axi.read(REG_WR_PTR)
    assert wr_ptr == 32, f"Expected wr_ptr=32 after wrap, got {wr_ptr}"


@cocotb.test()
async def test_slow_wready_no_overflow(dut):
    """Slow WREADY (2-cycle delay) does not cause overflow with double buffering."""
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    await axi._init_signals()

    # Create slave with wready_delay=2
    slave = Axi3WriteSlave(dut, prefix="m_axi_", clk=dut.clk, wready_delay=2)
    cocotb.start_soon(slave.run())

    await reset(dut)

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Feed 64 samples (2 bursts)
    samples = [(i, i + 300) for i in range(64)]
    await feed_iq(dut, samples)

    # Wait for both bursts to complete (extra time for WREADY delays)
    await ClockCycles(dut.clk, 800)

    # Both bursts should complete successfully
    assert len(slave.bursts) == 2, \
        f"Expected 2 bursts, got {len(slave.bursts)}"

    # Verify correct data in first burst (spot check beat 0)
    _, beat_data = slave.bursts[0]
    assert len(beat_data) == 16, f"Expected 16 beats, got {len(beat_data)}"

    # Beat 0: samples 0 and 1
    even_word = (300 << 12) | 0  # re=0, im=300
    odd_word = (301 << 12) | 1   # re=1, im=301
    expected_beat0 = (odd_word << 32) | even_word
    assert beat_data[0] == expected_beat0, \
        f"Beat 0: expected 0x{expected_beat0:016x}, got 0x{beat_data[0]:016x}"

    # Verify overflow_count remains 0
    status, _ = await axi.read(REG_STATUS)
    overflow_count = (status >> 16) & 0xFFFF
    assert overflow_count == 0, \
        f"Expected overflow_count=0, got {overflow_count} (STATUS=0x{status:08x})"


@cocotb.test()
async def test_sample_cnt_out(dut):
    """Verify sample_cnt_out tracks samples accurately (unlike burst-quantized wr_ptr).

    sample_cnt_out = reg_sample_count[24:0], increments on every valid_in.
    This is the source-of-truth for DDR ring buffer position — it advances
    one sample at a time, while ddr_wr_ptr_out jumps by 32 on burst completion.
    """
    axi, slave = await setup(dut)

    # Before enable: counter should be 0
    assert int(dut.sample_cnt_out.value) == 0, \
        f"sample_cnt_out should be 0 after reset, got {int(dut.sample_cnt_out.value)}"

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Feed 10 samples — less than a full burst (32)
    samples = [(i, i + 50) for i in range(10)]
    await feed_iq(dut, samples)
    await ClockCycles(dut.clk, 5)

    # sample_cnt_out should be 10 (advances per sample)
    cnt = int(dut.sample_cnt_out.value)
    assert cnt == 10, f"After 10 samples: sample_cnt_out={cnt}, expected 10"

    # ddr_wr_ptr_out should still be 0 (no burst completed yet)
    wr_ptr = int(dut.ddr_wr_ptr_out.value)
    assert wr_ptr == 0, f"After 10 samples: ddr_wr_ptr_out={wr_ptr}, expected 0 (no burst yet)"

    # Feed 22 more samples to complete a burst (total 32)
    samples2 = [(i + 10, i + 60) for i in range(22)]
    await feed_iq(dut, samples2)
    await ClockCycles(dut.clk, 200)  # wait for burst to complete

    # sample_cnt_out should be 32
    cnt = int(dut.sample_cnt_out.value)
    assert cnt == 32, f"After 32 samples: sample_cnt_out={cnt}, expected 32"

    # ddr_wr_ptr_out should now be 32 (burst completed)
    wr_ptr = int(dut.ddr_wr_ptr_out.value)
    assert wr_ptr == 32, f"After burst: ddr_wr_ptr_out={wr_ptr}, expected 32"

    # Feed 5 more samples (mid-burst)
    samples3 = [(i + 32, i + 82) for i in range(5)]
    await feed_iq(dut, samples3)
    await ClockCycles(dut.clk, 5)

    # sample_cnt_out should be 37 (sample-accurate)
    cnt = int(dut.sample_cnt_out.value)
    assert cnt == 37, f"After 37 samples: sample_cnt_out={cnt}, expected 37"

    # ddr_wr_ptr_out should still be 32 (second burst not complete)
    wr_ptr = int(dut.ddr_wr_ptr_out.value)
    assert wr_ptr == 32, f"Mid-burst: ddr_wr_ptr_out={wr_ptr}, expected 32 (no advance)"

    # Verify via register too
    sample_count_reg, _ = await axi.read(REG_SAMPLE_COUNT)
    assert sample_count_reg == 37, \
        f"REG_SAMPLE_COUNT={sample_count_reg}, expected 37"
    assert (sample_count_reg & 0x1FFFFFF) == cnt, \
        f"sample_cnt_out ({cnt}) != reg_sample_count[24:0] ({sample_count_reg & 0x1FFFFFF})"


@cocotb.test()
async def test_long_burst_data_integrity(dut):
    """Feed a large continuous stream and verify every sample reaches DDR correctly.

    Reproducer for Known Issue #9: 36th frame in a burst has no signal in DDR.
    At gap=10000, frame~7500 samples: 36 frames = ~630,000 samples = ~19,688 bursts.
    Start with 1200 bursts (38,400 samples) as a faster first check, then scale up.

    Requires large DDR_BUF_SAMPLES (default). Skipped when overridden to small
    values (e.g. DDR_BUF_SAMPLES=64 in the wrap test variant).
    """
    axi, slave = await setup(dut)

    # Skip when DDR buffer is too small — wrapping makes linear address checks invalid
    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    if buf_samples < 38400:
        cocotb.log.info(f"Skipping: DDR_BUF_SAMPLES={buf_samples} (need >=38400 for linear integrity test)")
        return

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Feed a large number of samples — enough for many bursts.
    # Use distinctive pattern: re = sample_idx & 0xFFF, im = (sample_idx >> 4) & 0xFFF
    # This ensures we can verify data identity per-sample.
    N_SAMPLES = 38400  # 1200 bursts of 32 samples
    BATCH_SIZE = 320   # Feed in batches to allow burst FSM to run

    samples_fed = 0
    for batch_start in range(0, N_SAMPLES, BATCH_SIZE):
        batch_end = min(batch_start + BATCH_SIZE, N_SAMPLES)
        batch = []
        for i in range(batch_start, batch_end):
            re_val = i & 0xFFF
            im_val = (i >> 4) & 0xFFF
            batch.append((re_val, im_val))
        await feed_iq(dut, batch)
        samples_fed += len(batch)

    # Wait for final burst to complete
    await ClockCycles(dut.clk, 500)

    # Verify burst count
    expected_bursts = N_SAMPLES // 32
    assert len(slave.bursts) == expected_bursts, \
        f"Expected {expected_bursts} bursts, got {len(slave.bursts)}"

    # Verify every sample in DDR matches what was fed
    errors = []
    for burst_idx, (addr, beat_data) in enumerate(slave.bursts):
        # Expected address
        sample_offset = burst_idx * 32
        expected_addr = DDR_BASE_ADDR + sample_offset * 4
        if addr != expected_addr:
            errors.append(f"Burst {burst_idx}: addr 0x{addr:08x} != expected 0x{expected_addr:08x}")
            continue

        # Check each beat (2 samples per beat, 16 beats per burst)
        for beat_idx in range(16):
            beat = beat_data[beat_idx]
            even_idx = sample_offset + beat_idx * 2
            odd_idx = even_idx + 1

            # Expected even sample
            even_re = even_idx & 0xFFF
            even_im = (even_idx >> 4) & 0xFFF
            expected_even = (even_im << 12) | even_re

            # Expected odd sample
            odd_re = odd_idx & 0xFFF
            odd_im = (odd_idx >> 4) & 0xFFF
            expected_odd = (odd_im << 12) | odd_re

            # 64-bit beat: {odd_word_with_padding, even_word_with_padding}
            expected_beat = (expected_odd << 32) | expected_even
            if beat != expected_beat:
                actual_even = beat & 0xFFFFFFFF
                actual_odd = (beat >> 32) & 0xFFFFFFFF
                errors.append(
                    f"Burst {burst_idx} beat {beat_idx} (samples {even_idx},{odd_idx}): "
                    f"got 0x{beat:016x}, expected 0x{expected_beat:016x} "
                    f"(even: 0x{actual_even:08x} vs 0x{expected_even:08x}, "
                    f"odd: 0x{actual_odd:08x} vs 0x{expected_odd:08x})")

    if errors:
        # Report first 10 errors
        msg = f"Data integrity failures ({len(errors)} total):\n"
        for e in errors[:10]:
            msg += f"  {e}\n"
        assert False, msg

    # Verify no overflow
    status, _ = await axi.read(REG_STATUS)
    overflow_count = (status >> 16) & 0xFFFF
    assert overflow_count == 0, f"Unexpected overflow: {overflow_count}"

    cocotb.log.info(f"PASS: {N_SAMPLES} samples ({expected_bursts} bursts) all correct")


@cocotb.test()
async def test_burst_mode_frame_pattern(dut):
    """Simulate the hardware burst-loopback pattern: frames with gaps.

    Reproducer for Known Issue #9: at gap=10000 samples between frames,
    the 36th FCS-ok frame's DDR region contains zeros.

    This test feeds a pattern mimicking cable loopback: "frame" regions
    with non-zero signal followed by "gap" regions with zeros (silence).
    After all samples are DMA'd, verify that every "frame" region in DDR
    contains non-zero data.

    Using shorter frames (1000 samples) and gaps (2000 samples) to keep
    sim time reasonable. 50 frames × 3000 = 150,000 samples = 4,688 bursts.
    """
    axi, slave = await setup(dut)

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    FRAME_SAMPLES = 1000   # Shorter than real (7500) but tests same DMA path
    GAP_SAMPLES = 2000     # Gap between frames (silence/zeros)
    N_FRAMES = 50          # More than 36 to catch the bug
    BATCH_SIZE = 500       # Feed in batches

    # Build the full sample sequence, tracking where each frame starts
    frame_regions = []  # (start_sample, end_sample) for each frame
    all_samples = []
    sample_idx = 0

    for frame_num in range(N_FRAMES):
        frame_start = sample_idx
        # Frame region: distinctive non-zero signal
        for i in range(FRAME_SAMPLES):
            # Use frame_num in the pattern so we can identify which frame
            re_val = ((frame_num + 1) * 100 + i) & 0xFFF
            im_val = ((frame_num + 1) * 50 + i) & 0xFFF
            all_samples.append((re_val, im_val))
            sample_idx += 1
        frame_end = sample_idx
        frame_regions.append((frame_start, frame_end))

        # Gap region: zeros (ADC noise floor / silence between frames)
        for i in range(GAP_SAMPLES):
            all_samples.append((0, 0))
            sample_idx += 1

    total_samples = len(all_samples)
    cocotb.log.info(f"Feeding {total_samples} samples ({N_FRAMES} frames × "
                    f"{FRAME_SAMPLES} + {GAP_SAMPLES} gap)")

    # Feed all samples
    for batch_start in range(0, total_samples, BATCH_SIZE):
        batch_end = min(batch_start + BATCH_SIZE, total_samples)
        batch = all_samples[batch_start:batch_end]
        await feed_iq(dut, batch)

    # Wait for final burst to complete
    await ClockCycles(dut.clk, 500)

    # Verify burst count
    expected_bursts = total_samples // 32
    actual_bursts = len(slave.bursts)
    assert actual_bursts == expected_bursts, \
        f"Expected {expected_bursts} bursts, got {actual_bursts}"

    # For each frame region, verify data is non-zero in DDR
    zero_frames = []
    for frame_num, (frame_start, frame_end) in enumerate(frame_regions):
        # Check a few samples in the middle of each frame
        check_positions = [
            frame_start + FRAME_SAMPLES // 4,
            frame_start + FRAME_SAMPLES // 2,
            frame_start + 3 * FRAME_SAMPLES // 4,
        ]
        frame_has_signal = False
        for pos in check_positions:
            # Find which burst and beat this sample is in
            burst_idx = pos // 32
            beat_within_burst = (pos % 32) // 2
            is_odd = pos % 2

            if burst_idx >= len(slave.bursts):
                continue

            _, beat_data = slave.bursts[burst_idx]
            beat = beat_data[beat_within_burst]

            if is_odd:
                word = (beat >> 32) & 0xFFFFFFFF
            else:
                word = beat & 0xFFFFFFFF

            if word != 0:
                frame_has_signal = True
                break

        if not frame_has_signal:
            zero_frames.append(frame_num)

    if zero_frames:
        msg = (f"Frames with zero/missing DDR data: {zero_frames}\n"
               f"(frame numbers are 0-indexed, total frames={N_FRAMES})")
        # Log surrounding frame data for first zero frame
        if zero_frames:
            f = zero_frames[0]
            fs, fe = frame_regions[f]
            burst_idx = fs // 32
            msg += f"\n  Frame {f}: samples {fs}-{fe}, starts at burst {burst_idx}"
        assert False, msg

    # Verify no overflow
    status, _ = await axi.read(REG_STATUS)
    overflow_count = (status >> 16) & 0xFFFF
    assert overflow_count == 0, f"Unexpected overflow: {overflow_count}"

    cocotb.log.info(f"PASS: All {N_FRAMES} frame regions have non-zero data in DDR")


@cocotb.test()
async def test_burst_mode_with_backpressure(dut):
    """Same as frame pattern test but with realistic AXI back-pressure.

    The Zynq HP0 port can stall WREADY for several cycles when the DDR
    controller is busy. This makes the burst FSM take longer, potentially
    causing the fill side to accumulate more before the burst completes.
    Test with wready_delay=3 (aggressive back-pressure).
    """
    clock = Clock(dut.clk, 10, unit='ns')
    cocotb.start_soon(clock.start())

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    await axi._init_signals()

    # Create slave with realistic back-pressure
    slave = Axi3WriteSlave(dut, prefix="m_axi_", clk=dut.clk, wready_delay=3)
    cocotb.start_soon(slave.run())

    await reset(dut)

    # Configure and enable
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 1)

    # Use hardware-scale parameters: 40 frames, gap=10000, frame=7500
    # Total: 40 × 17500 = 700,000 samples = 21,875 bursts
    # With wready_delay=3, each burst takes ~80 cycles (16 beats × 5 cycles/beat)
    # That's still faster than fill (32 samples × 5 clocks = 160 clocks)
    # So no overflow expected — but tests the sustained operation.
    #
    # Actually, this would take too long. Use scaled parameters:
    # 40 frames × (2000 frame + 5000 gap) = 280,000 samples = 8,750 bursts
    FRAME_SAMPLES = 2000
    GAP_SAMPLES = 5000
    N_FRAMES = 40
    BATCH_SIZE = 500

    frame_regions = []
    all_samples = []
    sample_idx = 0

    for frame_num in range(N_FRAMES):
        frame_start = sample_idx
        for i in range(FRAME_SAMPLES):
            re_val = ((frame_num + 1) * 77 + i) & 0xFFF
            im_val = ((frame_num + 1) * 33 + i) & 0xFFF
            all_samples.append((re_val, im_val))
            sample_idx += 1
        frame_end = sample_idx
        frame_regions.append((frame_start, frame_end))

        for i in range(GAP_SAMPLES):
            all_samples.append((0, 0))
            sample_idx += 1

    total_samples = len(all_samples)
    cocotb.log.info(f"Feeding {total_samples} samples with wready_delay=3 "
                    f"({N_FRAMES} frames)")

    # Feed all samples
    for batch_start in range(0, total_samples, BATCH_SIZE):
        batch_end = min(batch_start + BATCH_SIZE, total_samples)
        batch = all_samples[batch_start:batch_end]
        await feed_iq(dut, batch)

    # Wait for final burst to complete (longer due to back-pressure)
    await ClockCycles(dut.clk, 2000)

    # Verify burst count
    expected_bursts = total_samples // 32
    actual_bursts = len(slave.bursts)
    assert actual_bursts == expected_bursts, \
        f"Expected {expected_bursts} bursts, got {actual_bursts}"

    # Check each frame region for non-zero data
    zero_frames = []
    for frame_num, (frame_start, frame_end) in enumerate(frame_regions):
        check_positions = [
            frame_start + FRAME_SAMPLES // 4,
            frame_start + FRAME_SAMPLES // 2,
            frame_start + 3 * FRAME_SAMPLES // 4,
        ]
        frame_has_signal = False
        for pos in check_positions:
            burst_idx = pos // 32
            beat_within_burst = (pos % 32) // 2
            is_odd = pos % 2

            if burst_idx >= len(slave.bursts):
                continue

            _, beat_data = slave.bursts[burst_idx]
            beat = beat_data[beat_within_burst]

            if is_odd:
                word = (beat >> 32) & 0xFFFFFFFF
            else:
                word = beat & 0xFFFFFFFF

            if word != 0:
                frame_has_signal = True
                break

        if not frame_has_signal:
            zero_frames.append(frame_num)

    if zero_frames:
        assert False, (f"Frames with zero DDR data (backpressure test): {zero_frames}")

    # Check overflow
    status, _ = await axi.read(REG_STATUS)
    overflow_count = (status >> 16) & 0xFFFF
    assert overflow_count == 0, f"Overflow with backpressure: {overflow_count}"

    cocotb.log.info(f"PASS: All {N_FRAMES} frames OK with wready_delay=3")
