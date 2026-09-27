# SPDX-License-Identifier: MIT
# Deimos composition CDC timing constraints.
#
# The deimos BD composes styx infrastructure + the receiver pipeline under
# system_i, clocked by PS7 FCLKs. The styx dma_timing.xdc references
# standalone top-level ports (s_axi_aclk, clk) that do not exist in this
# composition, so its set_clock_groups is a no-op and several of its false
# paths can't match. This file re-declares the CDC intent for the deimos
# hierarchy using the actual clock names from system_constr.xdc:
#
#   rx_clk      16.27ns  AD9361 lane clock (rx_clk_in port)
#   clk_fpga_0  10ns     PS7 FCLK0 — fabric/AXI
#   clk_fpga_1   5ns     PS7 FCLK1
#   (spi0_clk create_clock in system_constr.xdc is a no-op — no EMIOSPI0 pin)
#
# Crossing inventory (all synchronizer chains in RTL):
#   iq_dma_tx: wr_ptr (2-FF), rd_ptr (gray), dac_miss (gray), trigger
#              toggle (3-FF), config regs (static), status (2-FF)
#   adc_cdc:   async FIFO gray pointers

# ---- Async clock group declaration ----
# All PL clock domains are mutually asynchronous (no phase relationship
# guarantees between PS7 FCLKs and the AD9361 lane clock).
set_clock_groups -asynchronous \
    -group [get_clocks rx_clk] \
    -group [get_clocks clk_fpga_0] \
    -group [get_clocks clk_fpga_1]

# ---- adc_cdc async FIFO (rx_clk -> clk_fpga_0) ----
# Gray-code pointer synchronizers
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/wr_gray_reg[*]}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/wr_gray_sync1_reg[*]}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/rd_gray_reg[*]}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/rd_gray_sync1_reg[*]}]

# ---- iq_dma_tx trigger CDC (clk_fpga_0 -> rx_clk) ----
# Toggle-based CDC: trigger_toggle_axi -> trig_sync1/2/3
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/trigger_toggle_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/trig_sync1*}]

# Config registers: static during TX, sampled on trigger edge
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_enable_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/lcl_enable*}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_cyclic_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/lcl_cyclic*}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_stream_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/lcl_stream*}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_ddr_base_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/lcl_ddr_base*}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_tx_count_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/lcl_tx_count*}]

# Status readback (rx_clk -> clk_fpga_0): 2-FF synchronizers
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_active*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_active_sync1*}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_done*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_done_sync1*}]

# tx_ptr: best-effort cross-domain read (multi-bit, no synchronizer)
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_ptr*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/s_axi_rdata*}]

# ---- iq_dma_tx WR_PTR CDC (clk_fpga_0 -> rx_clk) ----
# 2-stage synchronizer: reg_wr_ptr_axi -> wr_ptr_sync1/2
# NOTE: styx dma_timing.xdc references wr_ptr_gray_* names (gray-code
# conversion started upstream but the RTL at this submodule revision still
# uses the raw 2-FF chain). Constraint matches the actual RTL.
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_wr_ptr_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/wr_ptr_sync1*}]

# ---- iq_dma_tx RD_PTR CDC (rx_clk -> clk_fpga_0) ----
# Gray-code synchronizer: rd_ptr_gray_reg -> rd_ptr_gray_sync1/2
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/rd_ptr_gray_reg*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/rd_ptr_gray_sync1*}]

# ---- iq_dma_tx DAC_VALID_MISS CDC (rx_clk -> clk_fpga_0) ----
# Gray-code synchronizer: dac_miss_gray_reg -> dac_miss_gray_sync1/2
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/dac_miss_gray_reg*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/dac_miss_gray_sync1*}]
