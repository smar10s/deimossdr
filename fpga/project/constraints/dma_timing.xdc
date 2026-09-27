# SPDX-License-Identifier: MIT
# Styx custom timing constraints for IQ DMA clock domain crossings.

# ---- Async clock group declaration ----
# Removes all cross-domain paths from timing analysis between
# s_axi_aclk (~100 MHz) and clk/l_clk (~20 MHz).  Individual
# false paths below handle the specific CDC crossings.
set_clock_groups -asynchronous \
    -group [get_clocks -of_objects [get_ports s_axi_aclk]] \
    -group [get_clocks -of_objects [get_ports clk]]

# ---- Async FIFO CDC (adc_sync / u_iq_fifo) ----
# The async FIFO crosses IQ data from rx_clk (l_clk, ~20 MHz) to
# clk_fpga_0 (sys_cpu_clk, ~100 MHz). Gray-code pointer synchronizers
# and stable-before-read FIFO memory don't need single-cycle timing.

# Gray-code pointer synchronizers
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/wr_gray_reg[*]}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/wr_gray_sync1_reg[*]}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/rd_gray_reg[*]}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/rd_gray_sync1_reg[*]}]

# FIFO distributed RAM: written on rx_clk, read on clk_fpga_0.
# Data is guaranteed stable by the time the pointer update propagates.
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *u_iq_fifo/mem_reg*}]

# ---- iq_dma_tx trigger CDC (s_axi_aclk -> l_clk) ----
# Toggle-based CDC: trigger_toggle_axi (s_axi_aclk) -> trig_sync1/2/3 (l_clk).
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/trigger_toggle_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/trig_sync1*}]

# Config registers (s_axi_aclk -> l_clk): static during TX, sampled on trigger edge.
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

# Status readback (l_clk -> s_axi_aclk): 2-FF synchronizers.
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_active*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_active_sync1*}]
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_done*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_done_sync1*}]

# tx_ptr: best-effort cross-domain read (multi-bit, no synchronizer).
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/tx_ptr*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/s_axi_rdata*}]

# ---- iq_dma_tx WR_PTR CDC (s_axi_aclk -> l_clk) ----
# 2-stage synchronizer: reg_wr_ptr_axi -> wr_ptr_sync1/2
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/reg_wr_ptr_axi*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/wr_ptr_sync1*}]

# ---- iq_dma_tx RD_PTR CDC (l_clk -> s_axi_aclk) ----
# Gray-code synchronizer: rd_ptr_gray_reg -> rd_ptr_gray_sync1/2
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/rd_ptr_gray_reg*}] \
               -to   [get_cells -hierarchical -filter {NAME =~ *iq_dma_tx_0/inst/rd_ptr_gray_sync1*}]
