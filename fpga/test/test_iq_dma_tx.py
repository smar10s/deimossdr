"""Cocotb tests for iq_dma_tx — TX DMA reader with cyclic stop support."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import random

from axi_lite_driver import AxiLiteMaster

# Register offsets (AXI-Lite slave)
REG_CONTROL  = 0x00
REG_STATUS   = 0x04
REG_DDR_BASE = 0x08
REG_TX_COUNT = 0x0C
REG_TX_PTR   = 0x10

# Control bits
CTRL_ENABLE  = (1 << 0)
CTRL_TRIGGER = (1 << 1)
CTRL_CYCLIC  = (1 << 2)

# Status bits
STATUS_ACTIVE = (1 << 0)
STATUS_DONE   = (1 << 1)

DDR_BASE_ADDR = 0x1000_0000
SAMPLES_PER_BURST = 32  # 16 beats * 2 samples/beat


# =========================================================
# AXI3 Read Slave Mock — serves DDR read bursts
# =========================================================
class Axi3ReadSlave:
    """Mock AXI3 read slave that serves data from a pre-loaded memory."""

    def __init__(self, dut, prefix="m_axi_", clk=None, rvalid_delay=0,
                 random_delay_range=None):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix
        self.mem = {}  # byte_addr -> byte
        self.bursts_served = 0
        self.rvalid_delay = rvalid_delay
        # If set, (min, max) range for random per-beat delay (overrides rvalid_delay)
        self.random_delay_range = random_delay_range
        self._running = True

    def stop(self):
        """Stop the slave coroutine."""
        self._running = False

    def _sig(self, name):
        return getattr(self.dut, self._p + name)

    def load_samples(self, base_addr, samples):
        """Load IQ samples into the mock memory.

        Each sample is (re, im) tuple, packed as {8'b0, im[11:0], re[11:0]}.
        Two samples per 64-bit beat (even at low address, odd at high).
        """
        for i, (re_val, im_val) in enumerate(samples):
            word = ((im_val & 0xFFF) << 12) | (re_val & 0xFFF)
            byte_addr = base_addr + i * 4
            for b in range(4):
                self.mem[byte_addr + b] = (word >> (8 * b)) & 0xFF

    def _read_dword(self, byte_addr):
        """Read 64-bit from memory (little-endian)."""
        val = 0
        for i in range(8):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    async def run(self):
        """Main loop — run as cocotb.start_soon(slave.run())."""
        self._sig("arready").value = 0
        self._sig("rdata").value = 0
        self._sig("rresp").value = 0
        self._sig("rlast").value = 0
        self._sig("rvalid").value = 0

        while self._running:
            # Wait for ARVALID
            while self._running:
                await RisingEdge(self.clk)
                try:
                    if int(self._sig("arvalid").value) == 1:
                        break
                except ValueError:
                    pass
            if not self._running:
                break

            base_addr = int(self._sig("araddr").value)
            arlen = int(self._sig("arlen").value)
            burst_len = arlen + 1

            # Accept AR
            self._sig("arready").value = 1
            await RisingEdge(self.clk)
            self._sig("arready").value = 0

            if not self._running:
                break

            # Serve R beats
            for beat_idx in range(burst_len):
                if not self._running:
                    self._sig("rvalid").value = 0
                    break

                # Optional delay before asserting RVALID
                if self.random_delay_range is not None:
                    delay = random.randint(*self.random_delay_range)
                else:
                    delay = self.rvalid_delay
                for _ in range(delay):
                    if not self._running:
                        break
                    await RisingEdge(self.clk)

                if not self._running:
                    self._sig("rvalid").value = 0
                    break

                byte_addr = base_addr + beat_idx * 8
                data = self._read_dword(byte_addr)

                self._sig("rdata").value = data
                self._sig("rresp").value = 0
                self._sig("rlast").value = 1 if beat_idx == burst_len - 1 else 0
                self._sig("rvalid").value = 1

                # Wait for RREADY handshake
                while True:
                    await RisingEdge(self.clk)
                    if not self._running:
                        self._sig("rvalid").value = 0
                        break
                    try:
                        if int(self._sig("rready").value) == 1:
                            break
                    except ValueError:
                        pass

                self._sig("rvalid").value = 0
                self._sig("rlast").value = 0

            self.bursts_served += 1


# =========================================================
# AXI3 Read Slave Mock — 2 Outstanding, overlap-aware timing
# =========================================================
class Axi3ReadSlave2OS:
    """Mock AXI3 read slave supporting 2 outstanding reads with DDR
    row-activate modeling.

    Models a DDR controller that can accept a second AR while still
    serving data for the first.  The row-activate for the second
    address begins at AR acceptance time.  When it's time to serve
    the second burst's data, remaining latency = max(0,
    row_activate_delay - (now - acceptance_time)).

    This is critical for testing the speculative wrap AR optimization:
    the wrap AR is issued early (during last burst data), so by the
    time the last burst finishes, the wrap burst data is partially
    or fully ready.

    Uses simulation time (ns) divided by clock period to compute
    elapsed cycles, avoiding multi-coroutine counter conflicts.
    """

    CLK_PERIOD_NS = 50  # l_clk ~20 MHz

    def __init__(self, dut, prefix="m_axi_", clk=None,
                 normal_latency=1,
                 row_activate_latency=20,
                 contention_addr_fn=None,
                 per_beat_delay=0):
        """
        Args:
            normal_latency: cycles from AR accept to first RVALID for
                addresses that don't cross a row boundary.
            row_activate_latency: cycles from AR accept to first RVALID
                for addresses that DO cross a row boundary (contention).
            contention_addr_fn: callable(addr) -> bool.  Returns True if
                the address requires row-activate latency.  If None, all
                addresses use normal_latency.
            per_beat_delay: extra cycles between R beats (models DDR
                bandwidth limitations).  0 = back-to-back.
        """
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix
        self.mem = {}
        self.bursts_served = 0
        self.normal_latency = normal_latency
        self.row_activate_latency = row_activate_latency
        self.contention_addr_fn = contention_addr_fn
        self.per_beat_delay = per_beat_delay
        self._running = True
        # AR FIFO: list of (addr, burst_len, accept_time_ns, has_contention)
        self._ar_fifo = []

    def stop(self):
        self._running = False

    def _sig(self, name):
        return getattr(self.dut, self._p + name)

    def _now_cycles(self):
        """Current simulation time in l_clk cycles."""
        from cocotb.utils import get_sim_time
        return int(get_sim_time('ns') // self.CLK_PERIOD_NS)

    def load_samples(self, base_addr, samples):
        """Load IQ samples (same format as Axi3ReadSlave)."""
        for i, (re_val, im_val) in enumerate(samples):
            word = ((im_val & 0xFFF) << 12) | (re_val & 0xFFF)
            byte_addr = base_addr + i * 4
            for b in range(4):
                self.mem[byte_addr + b] = (word >> (8 * b)) & 0xFF

    def _read_dword(self, byte_addr):
        val = 0
        for i in range(8):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    async def _ar_acceptor(self):
        """Accept AR requests into the FIFO (max depth 2)."""
        self._sig("arready").value = 0
        while self._running:
            await RisingEdge(self.clk)
            if not self._running:
                break
            try:
                arvalid = int(self._sig("arvalid").value)
            except ValueError:
                continue

            if arvalid and len(self._ar_fifo) < 2:
                addr = int(self._sig("araddr").value)
                arlen = int(self._sig("arlen").value)
                burst_len = arlen + 1

                # Accept: assert arready for one cycle
                self._sig("arready").value = 1
                await RisingEdge(self.clk)
                self._sig("arready").value = 0

                accept_cycles = self._now_cycles()
                has_contention = (self.contention_addr_fn is not None and
                                  self.contention_addr_fn(addr))
                self._ar_fifo.append((addr, burst_len,
                                      accept_cycles, has_contention))

    async def _r_server(self):
        """Serve R data from the AR FIFO with latency modeling."""
        self._sig("rdata").value = 0
        self._sig("rresp").value = 0
        self._sig("rlast").value = 0
        self._sig("rvalid").value = 0

        while self._running:
            # Wait for something in the FIFO
            while self._running and len(self._ar_fifo) == 0:
                await RisingEdge(self.clk)

            if not self._running:
                break

            # Pop oldest entry
            addr, burst_len, accept_cycles, has_contention = self._ar_fifo.pop(0)

            # Compute remaining latency
            if has_contention:
                target_latency = self.row_activate_latency
            else:
                target_latency = self.normal_latency

            elapsed = self._now_cycles() - accept_cycles
            remaining = max(0, target_latency - elapsed)

            # Wait remaining latency
            for _ in range(remaining):
                if not self._running:
                    break
                await RisingEdge(self.clk)

            if not self._running:
                break

            # Serve R beats
            for beat_idx in range(burst_len):
                if not self._running:
                    self._sig("rvalid").value = 0
                    break

                # Per-beat delay (after first beat)
                if beat_idx > 0:
                    for _ in range(self.per_beat_delay):
                        if not self._running:
                            break
                        await RisingEdge(self.clk)

                if not self._running:
                    self._sig("rvalid").value = 0
                    break

                byte_addr = addr + beat_idx * 8
                data = self._read_dword(byte_addr)

                self._sig("rdata").value = data
                self._sig("rresp").value = 0
                self._sig("rlast").value = 1 if beat_idx == burst_len - 1 else 0
                self._sig("rvalid").value = 1

                # Wait for RREADY handshake
                while True:
                    await RisingEdge(self.clk)
                    if not self._running:
                        self._sig("rvalid").value = 0
                        break
                    try:
                        if int(self._sig("rready").value) == 1:
                            break
                    except ValueError:
                        pass

                self._sig("rvalid").value = 0
                self._sig("rlast").value = 0

            self.bursts_served += 1

    async def run(self):
        """Main entry — launches AR acceptor and R server as parallel tasks."""
        self._sig("arready").value = 0
        self._sig("rdata").value = 0
        self._sig("rresp").value = 0
        self._sig("rlast").value = 0
        self._sig("rvalid").value = 0

        # Run both coroutines concurrently
        cocotb.start_soon(self._ar_acceptor())
        await self._r_server()


# =========================================================
# Helpers
# =========================================================
_active_slave = None


async def reset(dut):
    """Drive rst=1 for 5 l_clk cycles, deassert, wait 3 cycles."""
    dut.rst.value = 1
    dut.s_axi_aresetn.value = 0  # Reset both domains simultaneously
    dut.dac_valid.value = 1  # Always high in 1R1T
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    dut.s_axi_aresetn.value = 1
    await ClockCycles(dut.clk, 3)


async def reset_axi(dut):
    """No-op: both domains now reset together in reset(). Kept for compatibility."""
    await ClockCycles(dut.s_axi_aclk, 3)


async def setup(dut):
    """Start both clocks, reset, return (axi_lite, read_slave)."""
    global _active_slave
    if _active_slave is not None:
        _active_slave.stop()

    # l_clk (~20 MHz = 50 ns period)
    clk_l = Clock(dut.clk, 50, unit='ns')
    cocotb.start_soon(clk_l.start())

    # sys_cpu_clk (100 MHz = 10 ns period)
    clk_sys = Clock(dut.s_axi_aclk, 10, unit='ns')
    cocotb.start_soon(clk_sys.start())

    # AXI-Lite master (on sys_cpu_clk)
    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.s_axi_aclk)
    await axi._init_signals()

    # AXI3 read slave (on l_clk)
    slave = Axi3ReadSlave(dut, prefix="m_axi_", clk=dut.clk)
    cocotb.start_soon(slave.run())
    _active_slave = slave

    # Reset both domains
    dut.dac_valid.value = 1
    await reset(dut)
    await reset_axi(dut)

    return axi, slave


async def setup_with_delays(dut, rvalid_delay=0, random_delay_range=None):
    """Start clocks, reset, return (axi_lite, read_slave) with configurable DDR latency."""
    global _active_slave
    if _active_slave is not None:
        _active_slave.stop()

    clk_l = Clock(dut.clk, 50, unit='ns')
    cocotb.start_soon(clk_l.start())

    clk_sys = Clock(dut.s_axi_aclk, 10, unit='ns')
    cocotb.start_soon(clk_sys.start())

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.s_axi_aclk)
    await axi._init_signals()

    slave = Axi3ReadSlave(dut, prefix="m_axi_", clk=dut.clk,
                          rvalid_delay=rvalid_delay,
                          random_delay_range=random_delay_range)
    cocotb.start_soon(slave.run())
    _active_slave = slave

    dut.dac_valid.value = 1
    await reset(dut)
    await reset_axi(dut)

    # Extra settling time
    await ClockCycles(dut.clk, 20)

    return axi, slave


async def trigger_tx(axi, n_samples, cyclic=False, ddr_base=DDR_BASE_ADDR):
    """Configure and trigger a TX transmission."""
    await axi.write(REG_DDR_BASE, ddr_base)
    await axi.write(REG_TX_COUNT, n_samples)
    ctrl = CTRL_ENABLE | CTRL_TRIGGER
    if cyclic:
        ctrl |= CTRL_CYCLIC
    await axi.write(REG_CONTROL, ctrl)


async def wait_done(axi, timeout_cycles=5000):
    """Poll STATUS until tx_done=1, return True. Timeout returns False."""
    for _ in range(timeout_cycles):
        status, _ = await axi.read(REG_STATUS)
        if status & STATUS_DONE:
            return True
        await RisingEdge(axi.clk)
    return False


async def stop_tx(axi):
    """Stop TX: write enable=0 + trigger=1 to latch disable."""
    await axi.write(REG_CONTROL, CTRL_TRIGGER)  # enable=0, trigger=1


# =========================================================
# Collect DAC output samples
# =========================================================
class DacCapture:
    """Captures re_out/im_out every l_clk cycle when valid_out is asserted.

    Also tracks gaps: consecutive cycles where valid_out=0 while capture
    is running.  A gap at the start (before first valid) is NOT counted.
    """

    def __init__(self, dut):
        self.dut = dut
        self.samples = []  # list of (re, im) — 16-bit left-justified
        self.gaps = []     # list of gap lengths (cycles without valid_out)
        self._running = False
        self._seen_first_valid = False
        self._current_gap = 0

    async def run(self, max_samples=10000):
        self._running = True
        while self._running and len(self.samples) < max_samples:
            await RisingEdge(self.dut.clk)
            try:
                if int(self.dut.valid_out.value) == 1:
                    re_val = int(self.dut.re_out.value)
                    im_val = int(self.dut.im_out.value)
                    self.samples.append((re_val, im_val))
                    if self._seen_first_valid and self._current_gap > 0:
                        self.gaps.append(self._current_gap)
                    self._seen_first_valid = True
                    self._current_gap = 0
                else:
                    if self._seen_first_valid:
                        self._current_gap += 1
            except ValueError:
                pass

    def stop(self):
        self._running = False

    @property
    def total_gap_cycles(self):
        """Total cycles of gap (valid_out=0) during active output."""
        return sum(self.gaps)

    @property
    def max_gap(self):
        """Longest single gap in cycles."""
        return max(self.gaps) if self.gaps else 0


# =========================================================
# Tests
# =========================================================

@cocotb.test()
async def test_oneshot_basic(dut):
    """One-shot TX: verify correct number of output samples and tx_done."""
    axi, slave = await setup(dut)

    n_samples = 64  # 2 bursts worth
    samples = [(i & 0xFFF, (i + 100) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    # Start DAC capture
    cap = DacCapture(dut)
    cocotb.start_soon(cap.run())

    # Trigger one-shot TX
    await trigger_tx(axi, n_samples, cyclic=False)

    # Wait for completion
    done = await wait_done(axi, timeout_cycles=2000)
    assert done, "TX did not complete (tx_done not set)"

    # Allow drain to finish outputting
    await ClockCycles(dut.clk, 50)
    cap.stop()

    # Verify we got exactly n_samples
    assert len(cap.samples) == n_samples, \
        f"Expected {n_samples} output samples, got {len(cap.samples)}"


@cocotb.test()
async def test_oneshot_data_integrity(dut):
    """Verify output samples match DDR content (12-bit << 4 format)."""
    axi, slave = await setup(dut)

    n_samples = 32  # 1 burst
    samples = [(i * 100 & 0xFFF, (i * 50 + 7) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run())

    await trigger_tx(axi, n_samples, cyclic=False)
    done = await wait_done(axi, timeout_cycles=2000)
    assert done, "TX did not complete"

    await ClockCycles(dut.clk, 50)
    cap.stop()

    assert len(cap.samples) == n_samples, \
        f"Expected {n_samples} samples, got {len(cap.samples)}"

    # Verify each sample (12-bit value left-shifted by 4)
    for i, (re_out, im_out) in enumerate(cap.samples):
        expected_re = (samples[i][0] & 0xFFF) << 4
        expected_im = (samples[i][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} re: expected 0x{expected_re:04x}, got 0x{re_out:04x}"
        assert im_out == expected_im, \
            f"Sample {i} im: expected 0x{expected_im:04x}, got 0x{im_out:04x}"


@cocotb.test()
async def test_oneshot_restart(dut):
    """One-shot TX can be triggered again after completion."""
    axi, slave = await setup(dut)

    n_samples = 32
    samples = [(i & 0xFFF, (i + 50) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    # First TX
    cap = DacCapture(dut)
    cocotb.start_soon(cap.run())

    await trigger_tx(axi, n_samples, cyclic=False)
    done = await wait_done(axi, timeout_cycles=2000)
    assert done, "First TX did not complete"
    await ClockCycles(dut.clk, 20)
    cap.stop()

    first_count = len(cap.samples)
    assert first_count == n_samples, \
        f"First TX: expected {n_samples}, got {first_count}"

    # Second TX (re-trigger) — verify completion via tx_done and TX_PTR
    await ClockCycles(dut.s_axi_aclk, 10)

    await trigger_tx(axi, n_samples, cyclic=False)
    done = await wait_done(axi, timeout_cycles=5000)
    assert done, "Second TX did not complete"

    # Verify TX_PTR indicates all samples were transmitted
    tx_ptr, _ = await axi.read(REG_TX_PTR)
    assert tx_ptr == n_samples, \
        f"Second TX: TX_PTR expected {n_samples}, got {tx_ptr}"


@cocotb.test()
async def test_cyclic_runs_multiple_iterations(dut):
    """Cyclic TX outputs more than one iteration of samples."""
    axi, slave = await setup(dut)

    n_samples = 32  # 1 burst per iteration
    samples = [(i & 0xFFF, (i + 10) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run(max_samples=200))

    await trigger_tx(axi, n_samples, cyclic=True)

    # Let it run for several iterations
    await ClockCycles(dut.clk, 800)
    cap.stop()

    # Should have more than 1 iteration worth of samples
    assert len(cap.samples) >= n_samples * 2, \
        f"Expected at least {n_samples * 2} cyclic samples, got {len(cap.samples)}"

    # Verify STATUS shows active (not done)
    status, _ = await axi.read(REG_STATUS)
    # In cyclic mode, tx_done should NOT be set
    assert not (status & STATUS_DONE), \
        f"Cyclic TX should not set tx_done, STATUS=0x{status:08x}"


@cocotb.test()
async def test_cyclic_stop(dut):
    """Cyclic TX can be stopped cleanly via enable=0 + trigger."""
    axi, slave = await setup(dut)

    n_samples = 32
    samples = [(i & 0xFFF, (i + 20) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run(max_samples=5000))

    # Start cyclic TX
    await trigger_tx(axi, n_samples, cyclic=True)

    # Let it run for a few iterations
    await ClockCycles(dut.clk, 500)

    # Verify it's running
    status, _ = await axi.read(REG_STATUS)
    assert status & STATUS_ACTIVE, \
        f"Expected active before stop, STATUS=0x{status:08x}"

    # Stop: write enable=0 + trigger to latch disable
    await stop_tx(axi)

    # Wait for tx_done
    done = await wait_done(axi, timeout_cycles=3000)
    assert done, "Cyclic TX did not stop after disable+trigger"

    # Record sample count at stop
    count_at_stop = len(cap.samples)

    # Wait more — no new samples should appear
    await ClockCycles(dut.clk, 200)
    cap.stop()

    assert len(cap.samples) == count_at_stop, \
        f"Samples continued after stop: {count_at_stop} -> {len(cap.samples)}"

    # Verify total samples is a multiple of n_samples
    # (drain finishes current iteration before stopping)
    assert count_at_stop % n_samples == 0, \
        f"Expected sample count to be multiple of {n_samples}, got {count_at_stop}"


@cocotb.test()
async def test_cyclic_stop_then_restart(dut):
    """After stopping cyclic TX, a new trigger starts fresh."""
    axi, slave = await setup(dut)

    n_samples = 32
    samples = [(42, 84)] * n_samples  # constant pattern
    slave.load_samples(DDR_BASE_ADDR, samples)

    # Start cyclic
    await trigger_tx(axi, n_samples, cyclic=True)
    await ClockCycles(dut.clk, 300)

    # Stop
    await stop_tx(axi)
    done = await wait_done(axi, timeout_cycles=3000)
    assert done, "First cyclic TX did not stop"

    await ClockCycles(dut.clk, 50)

    # Restart as one-shot — verify via tx_done and TX_PTR
    await ClockCycles(dut.s_axi_aclk, 10)

    await trigger_tx(axi, n_samples, cyclic=False)
    done = await wait_done(axi, timeout_cycles=5000)
    assert done, "One-shot TX after cyclic stop did not complete"

    # Verify TX_PTR indicates full transmission
    tx_ptr, _ = await axi.read(REG_TX_PTR)
    assert tx_ptr == n_samples, \
        f"Expected TX_PTR={n_samples} after restart, got {tx_ptr}"


# =========================================================
# Latency-jitter tests — exercise D_WAIT (underrun recovery)
# =========================================================

@cocotb.test()
async def test_oneshot_with_fixed_ddr_latency(dut):
    """One-shot TX under fixed DDR latency (2 cycles/beat) — no stale samples.

    With 2 cycles of delay per beat across a 16-beat burst, each burst
    takes 32 extra cycles. The 3-segment fill margin (~43 cycles) should
    still absorb this, but it stress-tests the boundary.
    """
    axi, slave = await setup_with_delays(dut, rvalid_delay=2)

    n_samples = 128  # 4 bursts — crosses multiple segment boundaries
    samples = [(i & 0xFFF, (i * 3 + 17) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run())

    await trigger_tx(axi, n_samples, cyclic=False)
    done = await wait_done(axi, timeout_cycles=10000)
    assert done, "TX did not complete with fixed latency"

    await ClockCycles(dut.clk, 50)
    cap.stop()

    assert len(cap.samples) == n_samples, \
        f"Expected {n_samples} samples, got {len(cap.samples)}"

    # Verify data integrity — every sample must match, no stale repeats
    for i, (re_out, im_out) in enumerate(cap.samples):
        expected_re = (samples[i][0] & 0xFFF) << 4
        expected_im = (samples[i][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} re: expected 0x{expected_re:04x}, got 0x{re_out:04x} (stale data?)"
        assert im_out == expected_im, \
            f"Sample {i} im: expected 0x{expected_im:04x}, got 0x{im_out:04x} (stale data?)"


@cocotb.test()
async def test_oneshot_with_random_ddr_jitter(dut):
    """One-shot TX under random DDR latency jitter (0-4 cycles/beat).

    Simulates realistic DDR controller behavior: variable response times
    due to row conflicts, refresh cycles, and port contention. The drain
    FSM must produce correct output with zero bubbles despite fill
    starvation periods that trigger D_WAIT.
    """
    random.seed(0xDEAD_BEEF)  # Deterministic for reproducibility
    axi, slave = await setup_with_delays(dut, random_delay_range=(0, 4))

    n_samples = 256  # 8 bursts — sufficient to trigger multiple D_WAIT entries
    samples = [(i & 0xFFF, ((i * 7) + 33) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run())

    await trigger_tx(axi, n_samples, cyclic=False)
    done = await wait_done(axi, timeout_cycles=10000)
    assert done, "TX did not complete with random jitter"

    await ClockCycles(dut.clk, 50)
    cap.stop()

    assert len(cap.samples) == n_samples, \
        f"Expected {n_samples} samples, got {len(cap.samples)}"

    # Full data integrity check — detects any stale/repeated/corrupted samples
    for i, (re_out, im_out) in enumerate(cap.samples):
        expected_re = (samples[i][0] & 0xFFF) << 4
        expected_im = (samples[i][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} re: expected 0x{expected_re:04x}, got 0x{re_out:04x}"
        assert im_out == expected_im, \
            f"Sample {i} im: expected 0x{expected_im:04x}, got 0x{im_out:04x}"


@cocotb.test()
async def test_cyclic_with_heavy_ddr_jitter(dut):
    """Cyclic TX under heavy DDR jitter (0-6 cycles/beat).

    With heavy jitter, fill may be outpaced by drain during the iteration
    itself (not just at wrap).  This test verifies:
    1. Data integrity is maintained across iteration boundaries.
    2. The system recovers correctly from any D_WAIT stalls.
    3. At least 2 complete iterations are output correctly.
    """
    random.seed(0xCAFE_BABE)
    axi, slave = await setup_with_delays(dut, random_delay_range=(0, 6))

    n_samples = 64  # 2 bursts per iteration — tight margin
    samples = [(i & 0xFFF, ((i * 11) + 5) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run(max_samples=5000))

    await trigger_tx(axi, n_samples, cyclic=True)

    # Let it run for multiple iterations (with heavy jitter, iterations are slower)
    await ClockCycles(dut.clk, 5000)

    # Stop cleanly
    await stop_tx(axi)
    done = await wait_done(axi, timeout_cycles=10000)
    assert done, "Cyclic TX did not stop under heavy jitter"

    await ClockCycles(dut.clk, 200)
    cap.stop()

    # Must have completed at least 2 full iterations
    assert len(cap.samples) >= n_samples * 2, \
        f"Expected at least {n_samples * 2} samples, got {len(cap.samples)}"

    # Verify data integrity across all captured iterations.
    full_iters = len(cap.samples) // n_samples
    assert full_iters >= 2, \
        f"Expected at least 2 full iterations, got {full_iters}"

    # Check all full iterations for data integrity
    for i in range(full_iters * n_samples):
        re_out, im_out = cap.samples[i]
        idx = i % n_samples
        expected_re = (samples[idx][0] & 0xFFF) << 4
        expected_im = (samples[idx][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) re: " \
            f"expected 0x{expected_re:04x}, got 0x{re_out:04x}"
        assert im_out == expected_im, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) im: " \
            f"expected 0x{expected_im:04x}, got 0x{im_out:04x}"

    # Report gap stats for diagnostic purposes (not a hard assertion
    # under extreme jitter — the goal is data integrity, not zero bubbles
    # when fill is fundamentally outpaced)
    if cap.total_gap_cycles > 0:
        dut._log.info(f"INFO: {len(cap.gaps)} D_WAIT gaps ({cap.total_gap_cycles} total cycles, "
                      f"max {cap.max_gap}) — expected under heavy jitter with 2-burst iterations")


@cocotb.test()
async def test_cyclic_zero_bubble_at_iteration_boundary(dut):
    """Verify zero-bubble output specifically at cyclic iteration boundaries.

    Uses a larger waveform (256 samples = 8 bursts = 8 segments) with LOW
    jitter (0-1 cycles/beat).  Under these conditions the fill FSM easily
    stays 7 segments ahead of drain.  The only potential stall is at the
    cyclic iteration boundary where the old design would pause in F_WAIT.

    The pre-fill optimization in F_SWAP should eliminate that stall,
    producing zero gaps in valid_out across many iterations.
    """
    random.seed(0x1234_5678)
    axi, slave = await setup_with_delays(dut, random_delay_range=(0, 1))

    n_samples = 256  # 8 bursts per iteration — fills all 8 segments
    samples = [(i & 0xFFF, ((i * 5) + 99) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run(max_samples=5000))

    await trigger_tx(axi, n_samples, cyclic=True)

    # Run for enough time to get 5+ iterations
    # 256 samples/iter * 5 iters = 1280 samples minimum
    await ClockCycles(dut.clk, 5000)

    await stop_tx(axi)
    done = await wait_done(axi, timeout_cycles=10000)
    assert done, "Cyclic TX did not stop"

    await ClockCycles(dut.clk, 200)
    cap.stop()

    full_iters = len(cap.samples) // n_samples
    assert full_iters >= 3, \
        f"Expected at least 3 full iterations, got {full_iters} " \
        f"({len(cap.samples)} samples total)"

    # CRITICAL: Zero bubbles at iteration boundary with low jitter
    assert cap.total_gap_cycles == 0, \
        f"BUBBLE DETECTED at iteration boundary: {len(cap.gaps)} gaps " \
        f"totaling {cap.total_gap_cycles} cycles (max: {cap.max_gap}). " \
        f"Pre-fill optimization failed."

    # Verify data integrity — especially at iteration boundary samples
    for i in range(full_iters * n_samples):
        re_out, im_out = cap.samples[i]
        idx = i % n_samples
        expected_re = (samples[idx][0] & 0xFFF) << 4
        expected_im = (samples[idx][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) re: " \
            f"expected 0x{expected_re:04x}, got 0x{re_out:04x}"
        assert im_out == expected_im, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) im: " \
            f"expected 0x{expected_im:04x}, got 0x{im_out:04x}"

    dut._log.info(f"PASS: {full_iters} iterations, 0 bubbles, "
                  f"{full_iters * n_samples} samples verified")


@cocotb.test()
async def test_cyclic_extreme_jitter_data_integrity(dut):
    """Stress test: cyclic TX with extreme DDR jitter (0-10 cycles/beat).

    This deliberately pushes beyond what ANY finite buffer can guarantee
    zero-bubble under (fill is fundamentally outpaced).  Verifies:
    1. Data integrity is maintained — every output sample is correct.
    2. System doesn't deadlock or corrupt state.
    3. Multiple iterations complete successfully.

    D_WAIT stalls ARE expected here — the test validates graceful recovery.
    """
    random.seed(0xDEAD_FACE)
    axi, slave = await setup_with_delays(dut, random_delay_range=(0, 10))

    n_samples = 64  # 2 bursts per iteration
    samples = [(i & 0xFFF, ((i * 13) + 7) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(DDR_BASE_ADDR, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run(max_samples=3000))

    await trigger_tx(axi, n_samples, cyclic=True)

    # Extra time for extreme jitter
    await ClockCycles(dut.clk, 10000)

    await stop_tx(axi)
    done = await wait_done(axi, timeout_cycles=10000)
    assert done, "Cyclic TX did not stop under extreme jitter"

    await ClockCycles(dut.clk, 200)
    cap.stop()

    full_iters = len(cap.samples) // n_samples
    assert full_iters >= 3, \
        f"Expected at least 3 full iterations, got {full_iters}"

    # Data integrity — every sample that was output must be correct
    for i in range(full_iters * n_samples):
        re_out, im_out = cap.samples[i]
        idx = i % n_samples
        expected_re = (samples[idx][0] & 0xFFF) << 4
        expected_im = (samples[idx][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) re: " \
            f"expected 0x{expected_re:04x}, got 0x{re_out:04x}"
        assert im_out == expected_im, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) im: " \
            f"expected 0x{expected_im:04x}, got 0x{im_out:04x}"

    # Report D_WAIT stats (expected under extreme jitter)
    dut._log.info(f"PASS: {full_iters} iterations, data integrity verified. "
                  f"D_WAIT gaps: {len(cap.gaps)} ({cap.total_gap_cycles} total cycles, "
                  f"max {cap.max_gap}) — expected under 0-10 cycle jitter")


# =========================================================
# DDR contention test — models the real hardware failure mode
# =========================================================

async def setup_with_2os_slave(dut, normal_latency=1, row_activate_latency=20,
                               contention_addr_fn=None, per_beat_delay=0):
    """Start clocks, reset, return (axi_lite, 2OS_read_slave)."""
    global _active_slave
    if _active_slave is not None:
        _active_slave.stop()

    clk_l = Clock(dut.clk, 50, unit='ns')
    cocotb.start_soon(clk_l.start())

    clk_sys = Clock(dut.s_axi_aclk, 10, unit='ns')
    cocotb.start_soon(clk_sys.start())

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.s_axi_aclk)
    await axi._init_signals()

    slave = Axi3ReadSlave2OS(dut, prefix="m_axi_", clk=dut.clk,
                             normal_latency=normal_latency,
                             row_activate_latency=row_activate_latency,
                             contention_addr_fn=contention_addr_fn,
                             per_beat_delay=per_beat_delay)
    cocotb.start_soon(slave.run())
    _active_slave = slave

    dut.dac_valid.value = 1
    await reset(dut)
    await reset_axi(dut)

    await ClockCycles(dut.clk, 20)

    return axi, slave


@cocotb.test()
async def test_cyclic_contention_at_wrap(dut):
    """Cyclic TX under DDR row-activate contention at iteration boundary.

    Models the real hardware failure mode: wrap to DDR_BASE at iteration
    boundary incurs a row-activate penalty (DDR controller must open new
    row after serving end-of-waveform addresses in a different row).

    Configuration: 128 samples (4 bursts = 4 segments per iteration).
    Normal latency: 1 cycle.  Wrap penalty: 55 cycles.

    Measured behavior WITHOUT speculative AR: 6-cycle gap at each wrap
    boundary (fill barely loses the race, drain enters D_WAIT).

    With speculative AR (issued during last burst's F_RD_DATA, giving
    ~15 cycles of head start on row-activate), the penalty is absorbed
    and zero bubbles should occur.
    """
    n_samples = 128  # 4 bursts per iteration
    ddr_base = DDR_BASE_ADDR

    def is_wrap_addr(addr):
        """Returns True for the first burst address of the waveform (wrap)."""
        return addr == ddr_base

    axi, slave = await setup_with_2os_slave(
        dut,
        normal_latency=1,           # fast for sequential mid-waveform reads
        row_activate_latency=55,    # penalty for wrap to start
        contention_addr_fn=is_wrap_addr,
        per_beat_delay=0            # back-to-back beats once started
    )

    samples = [(i & 0xFFF, ((i * 5) + 99) & 0xFFF) for i in range(n_samples)]
    slave.load_samples(ddr_base, samples)

    cap = DacCapture(dut)
    cocotb.start_soon(cap.run(max_samples=5000))

    await trigger_tx(axi, n_samples, cyclic=True)

    # Run for 10+ iterations
    await ClockCycles(dut.clk, 5000)

    await stop_tx(axi)
    done = await wait_done(axi, timeout_cycles=10000)
    assert done, "Cyclic TX did not stop under contention"

    await ClockCycles(dut.clk, 200)
    cap.stop()

    full_iters = len(cap.samples) // n_samples
    assert full_iters >= 5, \
        f"Expected at least 5 full iterations, got {full_iters} " \
        f"({len(cap.samples)} samples total)"

    # CRITICAL: Zero bubbles — the speculative AR must hide the 40-cycle penalty
    assert cap.total_gap_cycles == 0, \
        f"BUBBLE at wrap boundary: {len(cap.gaps)} gaps totaling " \
        f"{cap.total_gap_cycles} cycles (max: {cap.max_gap}). " \
        f"Speculative wrap AR did not eliminate contention stall."

    # Data integrity across all iterations
    for i in range(full_iters * n_samples):
        re_out, im_out = cap.samples[i]
        idx = i % n_samples
        expected_re = (samples[idx][0] & 0xFFF) << 4
        expected_im = (samples[idx][1] & 0xFFF) << 4
        assert re_out == expected_re, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) re: " \
            f"expected 0x{expected_re:04x}, got 0x{re_out:04x}"
        assert im_out == expected_im, \
            f"Sample {i} (iter {i // n_samples}, pos {idx}) im: " \
            f"expected 0x{expected_im:04x}, got 0x{im_out:04x}"

    dut._log.info(f"PASS: {full_iters} iterations, 0 bubbles under "
                  f"55-cycle contention at wrap. Speculative AR working.")
