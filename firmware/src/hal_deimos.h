// SPDX-License-Identifier: MIT
#ifndef DEIMOS_HAL_DEIMOS_H
#define DEIMOS_HAL_DEIMOS_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Deimos maps into styx accelerator slots:
 *   ACCEL_A (0x7C510000) → tag FIFO + PSDU BRAM
 *   ACCEL_B (0x7C520000) → pipeline control (deimos_regs_axi)
 * -------------------------------------------------------------------------- */

/* Backward-compat aliases — existing deimos code uses these names */
#define REG_TAG_FIFO_BASE      REG_ACCEL_A_BASE
#define REG_DEIMOS_REGS_BASE   REG_ACCEL_B_BASE

/* Tag FIFO registers (ACCEL_A) */
#define REG_TAG_FIFO_STATUS    (REG_TAG_FIFO_BASE + 0x00)
#define REG_TAG_FIFO_TAG_LO    (REG_TAG_FIFO_BASE + 0x04)
#define REG_TAG_FIFO_TAG_HI    (REG_TAG_FIFO_BASE + 0x08)
#define REG_TAG_FIFO_CONTROL   (REG_TAG_FIFO_BASE + 0x0C)
#define REG_TAG_FIFO_FRAME_CNT (REG_TAG_FIFO_BASE + 0x10)
#define REG_TAG_FIFO_DROP_CNT  (REG_TAG_FIFO_BASE + 0x14)
#define REG_PSDU_DATA          (REG_TAG_FIFO_BASE + 0x28)

/* Tag FIFO status bits */
#define TAG_STATUS_EMPTY       (1 << 0)
#define TAG_STATUS_FULL        (1 << 1)
#define TAG_STATUS_COUNT(s)    (((s) >> 8) & 0xFF)

/* TAG_LO field extraction: {8'b0, fcs_ok, rate[3:0], 3'b0, length[11:0], 4'b0} */
#define TAG_LO_FCS_OK(t)       (((t) >> 23) & 0x01)
#define TAG_LO_RATE(t)         (((t) >> 19) & 0x0F)
#define TAG_LO_LENGTH(t)       (((t) >> 4) & 0xFFF)

/* TAG_HI field extraction: {18'b0, psdu_addr[13:0]} */
#define TAG_HI_PSDU_ADDR(t)    ((t) & 0x3FFF)

/* Tag FIFO control bits */
#define TAG_CTRL_FLUSH         (1 << 0)

/* Pipeline control registers (ACCEL_B — deimos_regs_axi) */
#define REG_DEIMOS_STF_THRESH  (REG_DEIMOS_REGS_BASE + 0x00)
#define REG_DEIMOS_STF_ENABLE  (REG_DEIMOS_REGS_BASE + 0x04)
#define REG_DEIMOS_DIAG_ACQ    (REG_DEIMOS_REGS_BASE + 0x08)  /* RO: [15:8]=found, [7:0]=rejected */
#define REG_DEIMOS_SNAP_MODE   (REG_DEIMOS_REGS_BASE + 0x0C)
#define REG_DEIMOS_DIAG_DROP   (REG_DEIMOS_REGS_BASE + 0x10)  /* RO: [15:0]=IQ write_ok drops */
#define REG_DEIMOS_DIAG_CLIP   (REG_DEIMOS_REGS_BASE + 0x14)  /* RO: [15:0]=chan_est clip events */
#define REG_DEIMOS_PHASE_INC   (REG_DEIMOS_REGS_BASE + 0x18)
#define REG_DEIMOS_VERSION     (REG_DEIMOS_REGS_BASE + 0x1C)
#define REG_DEIMOS_DIAG_ABORT_CNT (REG_DEIMOS_REGS_BASE + 0x20) /* RO: {wd,ow,rate,sig} */
#define REG_DEIMOS_DIAG_ABORT_SIG (REG_DEIMOS_REGS_BASE + 0x24) /* RO: [23:0]=L-SIG bits */
#define REG_DEIMOS_DIAG_ABORT_CTX (REG_DEIMOS_REGS_BASE + 0x28) /* RO: [15:0]=latched phase_inc */
#define REG_DEIMOS_DIAG_TAG_SIG   (REG_DEIMOS_REGS_BASE + 0x2C) /* RO: [23:0]=L-SIG bits at last good tag */
#define REG_DEIMOS_DIAG_TAG_CTX   (REG_DEIMOS_REGS_BASE + 0x30) /* RO: [15:0]=latched phase_inc at that tag */

/* Diagnostic register field extraction */
#define DIAG_ACQ_FOUND(d)      (((d) >> 8) & 0xFF)
#define DIAG_ACQ_REJECTED(d)   ((d) & 0xFF)

#define DIAG_ABORT_SIG(d)      ((d) & 0xFF)
#define DIAG_ABORT_RATE(d)     (((d) >> 8) & 0xFF)
#define DIAG_ABORT_OW(d)       (((d) >> 16) & 0xFF)
#define DIAG_ABORT_WD(d)       (((d) >> 24) & 0xFF)
#define DIAG_ABORT_SIGBITS(d)  ((d) & 0xFFFFFF)
#define DIAG_ABORT_PHASE(d)    ((int16_t)((d) & 0xFFFF))

#define DIAG_TAG_SIGBITS(d)    ((d) & 0xFFFFFF)
#define DIAG_TAG_PHASE(d)      ((int16_t)((d) & 0xFFFF))

/* --------------------------------------------------------------------------
 * HIL playback buffer (DDR)
 *
 * Single definition shared by every tool that drives hil_ctrl playback.
 * Offset 2 MB into the 32 MB TX region (DDR_TX_BASE = 0x18000000) so the
 * HIL buffer does NOT alias the DMA TX buffer, which dma_tx_load() fills
 * from offset 0. deimos_burst_loopback previously defined this as
 * 0x18000000 (exactly DDR_TX_BASE) while deimos_hil_inject used
 * 0x18200000 — the two tools disagreed, and in burst_loopback a
 * dma_tx_load() and an HIL load targeted the same memory.
 *
 * 4 MB window => 1 Mi samples at 4 bytes/sample (packed 12-bit I/Q).
 * Spans 0x18200000-0x18600000, well inside the mapped TX region.
 * -------------------------------------------------------------------------- */
#define DEIMOS_HIL_DDR_BASE     0x18200000
#define DEIMOS_HIL_MAX_SAMPLES  (0x00400000 / 4)

#endif /* DEIMOS_HAL_DEIMOS_H */
