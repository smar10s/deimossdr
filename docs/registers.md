# Register Map

<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Regenerate: python3 scripts/gen_registers.py -o docs/registers.md
     Verify:     python3 scripts/gen_registers.py --check
     Source of truth: RTL header comment blocks + ad_cpu_interconnect
     base addresses in the block-design tcl. Edit the RTL, not this. -->

All registers are 32-bit and word-aligned. Offsets are relative to the
peripheral base address.

## Address Map

| Base | Peripheral | BD instance | Source |
|------|-----------|-------------|--------|
| `0x43C00000` | Build fingerprint (read-only) | `axi_build_id_0` | `platform/styx/fpga/rtl/axi_build_id.v` |
| `0x7C4B0000` | IQ RX DMA to DDR | `iq_dma_rx_0` | `platform/styx/fpga/rtl/iq_dma_rx.v` |
| `0x7C4D0000` | IQ TX DMA from DDR | `iq_dma_tx_0` | `platform/styx/fpga/rtl/iq_dma_tx.v` |
| `0x7C4E0000` | Debug snap capture buffer | `snap_axi_0` | `platform/styx/fpga/rtl/snap_axi.v` |
| `0x7C500000` | HIL test controller (IQ playback) | `hil_ctrl_0` | `platform/styx/fpga/rtl/hil_ctrl.v` |
| `0x7C510000` | Tag FIFO + PSDU BRAM readback | `tag_fifo_axi_0` | `fpga/rtl/tag_fifo_axi.v` |
| `0x7C520000` | Deimos receiver control/status | `deimos_regs_0` | `fpga/rtl/deimos_regs_axi.v` |

## Build fingerprint (read-only) — `0x43C00000`

Instance `axi_build_id_0`, defined in `platform/styx/fpga/rtl/axi_build_id.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x43C00000` | `BUILD_ID` | — | content-addressed source fingerprint |
| `0x04` | `0x43C00004` | `PROJECT_ID` | — | ASCII project magic (e.g. "STYX" = 0x53545958) |

## IQ RX DMA to DDR — `0x7C4B0000`

Instance `iq_dma_rx_0`, defined in `platform/styx/fpga/rtl/iq_dma_rx.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x7C4B0000` | `CONTROL` | RW | [0]=enable |
| `0x04` | `0x7C4B0004` | `STATUS` | R | [0]=active, [31:16]=overflow_count |
| `0x08` | `0x7C4B0008` | `DDR_BASE` | RW | Physical DDR base address |
| `0x0C` | `0x7C4B000C` | `WR_PTR` | R | Current write pointer (sample index, 25-bit) |
| `0x10` | `0x7C4B0010` | `SAMPLE_COUNT` | R | Total samples written (32-bit, wraps) |

## IQ TX DMA from DDR — `0x7C4D0000`

Instance `iq_dma_tx_0`, defined in `platform/styx/fpga/rtl/iq_dma_tx.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x7C4D0000` | `CONTROL` | RW | [0]=enable, [1]=trigger (W1S), [2]=cyclic, [3]=stream |
| `0x04` | `0x7C4D0004` | `STATUS` | R | [0]=active, [1]=tx_done |
| `0x08` | `0x7C4D0008` | `DDR_BASE` | RW | Physical DDR base of TX buffer |
| `0x0C` | `0x7C4D000C` | `TX_COUNT` | RW | Total samples in buffer (one-shot/cyclic); buffer size (stream) |
| `0x10` | `0x7C4D0010` | `TX_PTR` | R | Current read pointer (sample index, unsynchronized) |
| `0x14` | `0x7C4D0014` | `WR_PTR` | RW | ARM write cursor (stream mode: fill FSM reads up to here) |
| `0x18` | `0x7C4D0018` | `RD_PTR` | R | FPGA read position (stream mode: ARM may overwrite past here) |

## Debug snap capture buffer — `0x7C4E0000`

Instance `snap_axi_0`, defined in `platform/styx/fpga/rtl/snap_axi.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x7C4E0000` | `CONTROL` | RW | [0]=arm (W1 rearms), [1]=sw_trigger (W1S, self-clr), [2]=circular_en |
| `0x04` | `0x7C4E0004` | `STATUS` | RO | [0]=captured, [1]=armed, [25:16]=trig_pos |
| `0x08` | `0x7C4E0008` | `TRIG_CYCLE` | RO | [31:0] cycle counter at trigger |
| `0x0C` | `0x7C4E000C` | `RD_ADDR` | RW | [9:0] read address |
| `0x10` | `0x7C4E0010` | `RD_DATA` | RO | [31:0] data at RD_ADDR (2-cycle BRAM latency hidden by AXI) |

## HIL test controller (IQ playback) — `0x7C500000`

Instance `hil_ctrl_0`, defined in `platform/styx/fpga/rtl/hil_ctrl.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x7C500000` | `CONTROL` | RW | [0]=test_mode, [1]=trigger (W1S) |
| `0x04` | `0x7C500004` | `STATUS` | RO | [0]=playback_active, [1]=playback_done |
| `0x08` | `0x7C500008` | `DDR_BASE` | RW | Physical DDR base of test waveform |
| `0x0C` | `0x7C50000C` | `PLAY_COUNT` | RW | Number of IQ samples to play back |
| `0x10` | `0x7C500010` | `PLAY_PTR` | RO | Current playback position (sample index) |
| `0x14`-`0x38` | `0x7C500014`-`0x7C500038` | `(reserved)` | — | Reserved for downstream extension |
| `0x3C` | `0x7C50003C` | `ADC_CNT` | RO | [31:0]=adc_valid pulse counter (live mode only) |

## Tag FIFO + PSDU BRAM readback — `0x7C510000`

Instance `tag_fifo_axi_0`, defined in `fpga/rtl/tag_fifo_axi.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x7C510000` | `STATUS` | — | [0]=tag_empty, [1]=tag_full, [15:8]=tag_count |
| `0x04` | `0x7C510004` | `TAG_LO` | — | Read (no pop): tag metadata (fcs, rate, length) |
| `0x08` | `0x7C510008` | `TAG_HI` | — | Read-pop: BRAM base address, advances tag FIFO, sets PSDU cursor |
| `0x0C` | `0x7C51000C` | `CONTROL` | — | [0]=flush (W1S) |
| `0x10` | `0x7C510010` | `FRAME_CNT` | — | Total frames written (32-bit) |
| `0x14` | `0x7C510014` | `DROP_CNT` | — | Frames dropped (tag full or BRAM full) (32-bit) |
| `0x28` | `0x7C510028` | `PSDU_DATA` | — | Read: byte at cursor, advances cursor |

## Deimos receiver control/status — `0x7C520000`

Instance `deimos_regs_0`, defined in `fpga/rtl/deimos_regs_axi.v`.

| Offset | Absolute | Name | Access | Description |
|--------|----------|------|--------|-------------|
| `0x00` | `0x7C520000` | `STF_THRESH` | RW | [7:0]=stf_detect threshold. Default 0. |
| `0x04` | `0x7C520004` | `STF_ENABLE` | RW | [0]=stf_detect enable. Default 0 (held in reset). |
| `0x08` | `0x7C520008` | `DIAG_ACQ` | RO | [15:8]=diag_frames_found, [7:0]=diag_frames_rejected. |
| `0x0C` | `0x7C52000C` | `SNAP_MODE` | RW | [2:0]=snap observation point selector. Default 0. |
| `0x10` | `0x7C520010` | `DIAG_DROP_CNT` | RO | RESERVED — unimplemented (decode_engine counter |
| `0x14` | `0x7C520014` | `DIAG_CLIP_CNT` | RO | RESERVED — unimplemented (chan_est counter |
| `0x18` | `0x7C520018` | `PHASE_INC` | RO | [15:0]=cfo_est phase_inc (sign-extended). Read-only. |
| `0x1C` | `0x7C52001C` | `VERSION` | RO | Firmware ID (parameter). Read-only. |
| `0x20` | `0x7C520020` | `DIAG_ABORT_CNT` | RO | [31:24]=watchdog, [23:16]=overwritten, |
| `0x24` | `0x7C520024` | `DIAG_ABORT_SIG` | RO | [23:0]=L-SIG bits at the last SIG-parse abort. |
| `0x28` | `0x7C520028` | `DIAG_ABORT_CTX` | RO | [31:16]=ltf1_offset, [15:0]=latched frame_phase_inc at abort. |
| `0x2C` | `0x7C52002C` | `DIAG_TAG_SIG` | RO | [23:0]=L-SIG bits at the last good tag-out. |
| `0x30` | `0x7C520030` | `DIAG_TAG_CTX` | RO | [31:16]=ltf1_offset, [15:0]=latched frame_phase_inc at tag. |

