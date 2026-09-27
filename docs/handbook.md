# Operational Handbook

Build, flash, debug, and recover the DeimosSDR platform. Reference
material for day-to-day development — command tables, access procedures,
and environment requirements.

For verification protocol, see `docs/verification.md`.
For agent session rules, see `AGENTS.md`.

## Build Commands

| Command | Wall time | Notes |
|---------|-----------|-------|
| `make bitstream` | 8-15 min | Vivado synthesis + implementation |
| `make firmware` | ~2 min | ARM cross-compile (Docker) |
| `make deploy` | ~30s | SCP tools to device |
| `make package` | ~30s | Assemble `pluto.frm` |
| `make flash` | 4-5 min | Write `pluto.frm` to device |
| `make validate` | ~5s | Confirm flashed build matches your local build's fingerprint |
| `make waves TARGET=...` | <60s | VCD waveforms for a sim target |
| `make hooks` | instant | Install git hooks from `scripts/hooks/` |

| Script | Wall time | Notes |
|--------|-----------|-------|
| `sim.sh` (full suite) | ~7 min | All gate tests, no hardware needed |
| `sim.sh` (one target) | <30s | Single test |
| `gen_sim_pipeline.py --check` | ~30s | Generated sim view matches BD (Layer 0) |
| `gen_registers.py --check` | <5s | Verify register map matches RTL |
| `session_start.sh` | ~75s | Connectivity, deploy, fingerprint, HIL, loopback |
| `session_end.sh` | ~8 min | Full sim + HIL + loopback gate |
| `hil_regression.sh -n 10` | ~50s | 10 trials x 8 rates |
| `loopback_test.sh -l 1` | ~2 min | Layer 1 loopback |
| `deimos_fabric_loopback` (-n 20) | ~2s/rate | Per-rate cable loopback |
| `characterize_failure.sh` | ~30s | ADC capture + EVM analysis |

## PlutoSDR Access

```bash
sshpass -p analog ssh -o StrictHostKeyChecking=no root@192.168.2.1
```

Host key changes on every reflash. Default credentials: `root` / `analog`.

The Pluto rootfs is **volatile**: `make deploy` writes the on-device tools
(`deimos_hil_inject`, `deimos_fabric_loopback`, …) into `/usr/bin`, which
lives in RAM. **Any cold boot loses them — not just a flash.** Power
cycles, antenna/cable swaps that drop power, and DFU recovery all count.
Run `make deploy` after every cold boot and before any HIL or loopback —
otherwise HIL/loopback run against stale or missing binaries.

The device appears at `192.168.2.1` over USB-Ethernet. If the device
does not respond to ping after flash, see DFU Recovery below.

## DFU Recovery

Use when a flash produces a non-booting Pluto (no ping at 192.168.2.1).

**Enter DFU mode:** Press the DFU button (small hole next to USB port)
with a toothpick. Hold until the LED stops blinking, then release. The
Pluto appears as a USB DFU device (verify with `dfu-util -l`). If DFU
is not detected, power-cycle while holding the button, or try a
different USB cable/port.

**Download stock firmware** (v0.38):
```bash
curl -L -o /tmp/pluto-recovery.zip \
  "https://github.com/analogdevicesinc/plutosdr-fw/releases/download/v0.38/plutosdr-fw-v0.38.zip"
unzip -o /tmp/pluto-recovery.zip pluto.frm -d /tmp/pluto-recovery/
```

**Flash via DFU:**
```bash
dfu-util -D /tmp/pluto-recovery/pluto.frm -a firmware.dfu
```

**Verify recovery** (~60 seconds after flash):
```bash
ping -c 1 192.168.2.1
sshpass -p analog ssh -o StrictHostKeyChecking=no root@192.168.2.1 cat /opt/VERSIONS
# Expected: device-fw v0.38
```

**DFU partitions** (USB ID 0456:b674):

| Alt | Name | Purpose |
|-----|------|---------|
| 0 | boot.dfu | U-Boot SPL + bootloader |
| 1 | firmware.dfu | pluto.frm (bitstream + kernel + rootfs) |
| 2 | uboot-extra-env.dfu | Extra U-Boot environment |
| 3 | uboot-env.dfu | Primary U-Boot environment |
| 4 | spare.dfu | Spare/unused |

Only flash `firmware.dfu` for recovery. Do NOT flash other partitions
without explicit instruction — erasing `uboot-env` can permanently
brick the device.

## Register Map

`docs/registers.md` is generated from RTL header comments. Do not
hand-edit it.

```bash
python3 scripts/gen_registers.py -o docs/registers.md   # regenerate
python3 scripts/gen_registers.py --check                 # verify, exit 1 on drift
```

To change the register map, edit the source-of-truth RTL header comment
block. To add a new peripheral, add its BD instance and RTL path to
`INSTANCE_RTL` in the script.

## Sim View Generation

`fpga/rtl/rx_pipeline.v` and `fpga/rtl/rx_frontend.v` are generated from the
block design (D19). Do not hand-edit them.

| Script | Role |
|--------|------|
| `scripts/bd_graph.py` | Parse `system_bd.tcl` → cells + `ad_connect`/`connect_bd_net` edges |
| `scripts/verilog_ports.py` | Verilator XML → per-module port directions/widths |
| `scripts/pipeline_sim_view.py` | Hand-authored sim delta: wrapper ports, `BOUNDARY`, `SIM_EDGES`, `NET_NAMES`, `EXTRA_*` |
| `scripts/gen_sim_pipeline.py` | Emit both wrappers; `--check` is sim Layer 0 |

To change pipeline wiring, edit `fpga/project/system_bd.tcl` (or the sim
delta in `pipeline_sim_view.py`) and regenerate:

```bash
python3 scripts/gen_sim_pipeline.py --bd fpga/project/system_bd.tcl --out-dir fpga/rtl
python3 scripts/gen_sim_pipeline.py --check --bd fpga/project/system_bd.tcl --out-dir fpga/rtl
```

The generated wrappers must preserve the instance hierarchy tests read
(`dut.u_rx_pipeline.u_<module>`) and the `rx_pipeline`-level net names in
`NET_NAMES`. `sim.sh` Layer 0 (`--check`) regenerates the view and diffs it,
so any drift fails before tests run. The block design, its per-module OOC
synthesis runs (`fpga/Makefile:OOC_MODULES`), and `fpga/tcl/*` are not touched
by generation (D19, D21).

## Golden Vectors

`extern/lib80211/vectors/` — source of truth for PHY correctness.
Consumed by cocotb tests via the lib80211 submodule.

## lib80211 Boundary

Git submodule at `extern/lib80211/`. Dual purpose: golden vectors for
RTL verification, and ARM reference decode for test-setup validation.
Bug fixes go upstream first, then bump the submodule pin.

ARM decode results (via `pluto_loopback`, `pluto_sigladder`) validate
the RF test setup. They prove the analog/DMA path works. They are not
fabric validation — see `docs/verification.md` for what counts.

## Environment

| Tool | Version | Source |
|------|---------|--------|
| Python | 3.11 | `.venv` (direnv) |
| cocotb | 2.0.1 | pip (`requirements.txt`) |
| verilator | 5.044 | homebrew |
| iverilog | 12.0 | homebrew (not used by `sim.sh`, which forces verilator; only via `make -C fpga/test SIM=icarus`) |
| Vivado | 2025.2 | Linux (WebPACK-eligible) |
| Docker | any | ARM cross-compile toolchain |

Python is pinned to 3.11. That is a legacy pin, not a cocotb constraint:
cocotb 2.0.1 declares support through 3.13. Bumping to 3.12 would let
`py80211` install as a package (its `pyproject.toml` requires `>=3.12`)
instead of relying on the `sys.path` injection in
`fpga/test/frontend_helpers.py`; revalidate the gate suite first
(see STATUS.md next steps).
