# ==========================================================================
# system_bd.tcl — Deimos block design (receiver pipeline on Styx platform)
#
# Composition: sources styx platform BD (infrastructure), then adds the
# 802.11 receiver pipeline connected to hil_ctrl IQ outputs.
#
# NOTE: This file is sourced inside a BD design context. Project creation
# and IP catalog setup happens in system_project.tcl before BD creation.
# ==========================================================================

# ---- Source styx platform infrastructure ----
# This creates: PS7, AD9361, DMA (RX/TX), adc_sync CDC, hil_ctrl, snap_axi,
# axi_build_id, and all clock/reset/interconnect infrastructure.
set styx_project_dir [file normalize [file dirname [info script]]/../../platform/styx/fpga/project]
source $styx_project_dir/system_bd.tcl

# ==========================================================================
# Deimos Receiver Control Registers
# ==========================================================================

create_bd_cell -type module -reference deimos_regs_axi deimos_regs_0

ad_connect sys_cpu_clk deimos_regs_0/s_axi_aclk
ad_connect sys_cpu_resetn deimos_regs_0/s_axi_aresetn

ad_cpu_interconnect 0x7C520000 deimos_regs_0
delete_bd_objs [get_bd_addr_segs sys_ps7/Data/SEG_data_deimos_regs_0]
create_bd_addr_seg -range 0x1000 -offset 0x7C520000 \
    [get_bd_addr_spaces sys_ps7/Data] \
    [get_bd_addr_segs deimos_regs_0/s_axi/reg0] \
    SEG_data_deimos_regs_0

# ==========================================================================
# Receiver Pipeline
# ==========================================================================

# ---- STF Detector ----
create_bd_cell -type module -reference stf_detect stf_detect_0

ad_connect sys_cpu_clk stf_detect_0/clk
ad_connect sys_cpu_resetn stf_detect_0/rst_n
ad_connect hil_ctrl_0/iq_valid stf_detect_0/iq_valid
ad_connect hil_ctrl_0/iq_re stf_detect_0/iq_i
ad_connect hil_ctrl_0/iq_im stf_detect_0/iq_q
ad_connect deimos_regs_0/stf_threshold_out stf_detect_0/threshold
ad_connect deimos_regs_0/stf_enable_out stf_detect_0/enable

# ---- CFO Estimator ----
create_bd_cell -type module -reference cfo_est cfo_est_0

ad_connect sys_cpu_clk cfo_est_0/clk
ad_connect sys_cpu_resetn cfo_est_0/rst_n
# CFO estimation is armed by the acquisition FSM (one estimate per accepted
# trigger) — wired in the control block below, once acquisition_ctrl_0 exists.
ad_connect hil_ctrl_0/iq_valid cfo_est_0/iq_valid
ad_connect hil_ctrl_0/iq_re cfo_est_0/iq_i
ad_connect hil_ctrl_0/iq_im cfo_est_0/iq_q

# ---- CFO Mixer ----
create_bd_cell -type module -reference cfo_mixer cfo_mixer_0

ad_connect sys_cpu_clk cfo_mixer_0/clk
ad_connect sys_cpu_resetn cfo_mixer_0/rst_n
ad_ip_instance xlconstant mixer_enable_const
ad_ip_parameter mixer_enable_const CONFIG.CONST_WIDTH 1
ad_ip_parameter mixer_enable_const CONFIG.CONST_VAL 1
ad_connect mixer_enable_const/dout cfo_mixer_0/enable
ad_ip_instance xlconstant mixer_phase_reset_const
ad_ip_parameter mixer_phase_reset_const CONFIG.CONST_WIDTH 1
ad_ip_parameter mixer_phase_reset_const CONFIG.CONST_VAL 0
ad_connect mixer_phase_reset_const/dout cfo_mixer_0/phase_reset
ad_connect cfo_est_0/phase_inc cfo_mixer_0/phase_inc
ad_connect hil_ctrl_0/iq_valid cfo_mixer_0/iq_valid_in
ad_connect hil_ctrl_0/iq_re cfo_mixer_0/iq_i_in
ad_connect hil_ctrl_0/iq_im cfo_mixer_0/iq_q_in

# ---- Decode Engine ----
create_bd_cell -type module -reference decode_engine decode_engine_0

ad_connect sys_cpu_clk decode_engine_0/clk
ad_connect sys_cpu_resetn decode_engine_0/rst_n

# ---- Acquisition Controller ----
create_bd_cell -type module -reference acquisition_ctrl acquisition_ctrl_0

ad_connect sys_cpu_clk acquisition_ctrl_0/clk
ad_connect sys_cpu_resetn acquisition_ctrl_0/rst_n

# ---- Frame FIFO ----
create_bd_cell -type module -reference frame_fifo frame_fifo_0

ad_connect sys_cpu_clk frame_fifo_0/clk
ad_connect sys_cpu_resetn frame_fifo_0/rst_n
ad_ip_instance xlconstant fifo_flush_const
ad_ip_parameter fifo_flush_const CONFIG.CONST_WIDTH 1
ad_ip_parameter fifo_flush_const CONFIG.CONST_VAL 0
ad_connect fifo_flush_const/dout frame_fifo_0/flush

# FIFO wiring: acquisition → frame_fifo → decode_engine
ad_connect acquisition_ctrl_0/desc_valid frame_fifo_0/push
ad_connect acquisition_ctrl_0/desc_ltf_pos frame_fifo_0/push_ltf_pos
ad_connect acquisition_ctrl_0/desc_phase_inc frame_fifo_0/push_phase_inc
ad_connect frame_fifo_0/full acquisition_ctrl_0/fifo_full
ad_connect frame_fifo_0/pop_ltf_pos decode_engine_0/pop_ltf_pos
ad_connect frame_fifo_0/pop_phase_inc decode_engine_0/pop_phase_inc
ad_connect frame_fifo_0/empty decode_engine_0/fifo_empty
ad_connect decode_engine_0/fifo_pop frame_fifo_0/pop

# ---- FFT64 SDF ----
create_bd_cell -type module -reference fft64_sdf fft64_sdf_0

ad_connect sys_cpu_clk fft64_sdf_0/clk
ad_connect decode_engine_0/fft_rst_n_out fft64_sdf_0/rst_n
ad_connect decode_engine_0/fft_rst_n_out fft64_sdf_0/gate_rst_n
ad_connect decode_engine_0/fft_din_valid_out fft64_sdf_0/din_valid
ad_connect decode_engine_0/fft_din_re_out fft64_sdf_0/din_re
ad_connect decode_engine_0/fft_din_im_out fft64_sdf_0/din_im
ad_connect fft64_sdf_0/dout_valid decode_engine_0/fft_dout_valid_in
ad_connect fft64_sdf_0/dout_re decode_engine_0/fft_dout_re_in
ad_connect fft64_sdf_0/dout_im decode_engine_0/fft_dout_im_in
ad_connect fft64_sdf_0/dout_idx decode_engine_0/fft_dout_idx_in
ad_ip_instance xlconstant fft_idx_zero
ad_ip_parameter fft_idx_zero CONFIG.CONST_WIDTH 6
ad_ip_parameter fft_idx_zero CONFIG.CONST_VAL 0
ad_connect fft_idx_zero/dout fft64_sdf_0/i_idx

# ---- LTF Correlator ----
create_bd_cell -type module -reference ltf_correlator ltf_correlator_0

ad_connect sys_cpu_clk ltf_correlator_0/clk
ad_connect sys_cpu_resetn ltf_correlator_0/rst_n
ad_connect cfo_mixer_0/iq_valid_out ltf_correlator_0/iq_valid
ad_connect cfo_mixer_0/iq_i_out ltf_correlator_0/iq_re
ad_connect cfo_mixer_0/iq_q_out ltf_correlator_0/iq_im

connect_bd_net [get_bd_pins ltf_correlator_0/metric_valid] [get_bd_pins acquisition_ctrl_0/corr_metric_valid]
connect_bd_net [get_bd_pins ltf_correlator_0/metric] [get_bd_pins acquisition_ctrl_0/corr_metric]

# ---- LTF Peak Detector ----
create_bd_cell -type module -reference ltf_peak_detect ltf_peak_detect_0

ad_connect sys_cpu_clk ltf_peak_detect_0/clk
ad_connect sys_cpu_resetn ltf_peak_detect_0/rst_n
ad_connect decode_engine_0/corr_enable_out ltf_peak_detect_0/enable
connect_bd_net [get_bd_pins ltf_correlator_0/metric_valid] [get_bd_pins ltf_peak_detect_0/metric_valid]
connect_bd_net [get_bd_pins ltf_correlator_0/metric] [get_bd_pins ltf_peak_detect_0/metric]
ad_connect decode_engine_0/corr_sample_pos_out ltf_peak_detect_0/sample_pos
ad_connect decode_engine_0/ltf_peak_ack ltf_peak_detect_0/ack
ad_connect ltf_peak_detect_0/peak_found decode_engine_0/ltf_peak_found_in
ad_connect ltf_peak_detect_0/peak_sample_pos decode_engine_0/ltf_peak_pos_in
ad_connect ltf_peak_detect_0/peak_metric decode_engine_0/ltf_peak_metric_in

# ---- Trigger + control wiring ----
ad_connect stf_detect_0/frame_detect acquisition_ctrl_0/frame_detect
ad_connect stf_detect_0/stf_end acquisition_ctrl_0/stf_end
ad_connect cfo_est_0/phase_inc acquisition_ctrl_0/phase_inc
ad_connect cfo_est_0/done acquisition_ctrl_0/cfo_done
# CFO estimation is armed by the acquisition FSM on an ACCEPTED trigger (one
# estimate per acquired frame), not by raw stf_detect/frame_detect. This binds
# the estimate to the same single-outstanding decision that pushes the
# descriptor, so a spurious/re-trigger cannot displace or mis-associate it.
ad_connect acquisition_ctrl_0/cfo_start cfo_est_0/start

# CFO phase_inc readback to deimos_regs (diagnostic)
ad_connect cfo_est_0/phase_inc deimos_regs_0/phase_inc_in

# Diagnostic counter readbacks to deimos_regs
ad_connect acquisition_ctrl_0/diag_frames_found deimos_regs_0/diag_frames_found_in
ad_connect acquisition_ctrl_0/diag_frames_rejected deimos_regs_0/diag_frames_rejected_in
ad_connect decode_engine_0/diag_drop_cnt deimos_regs_0/diag_drop_cnt_in
# Abort-reason counters + last SIG-parse abort snapshot (0x20/0x24/0x28)
ad_connect decode_engine_0/diag_abort_cnts deimos_regs_0/diag_abort_cnt_in
ad_connect decode_engine_0/diag_abort_sig deimos_regs_0/diag_abort_sig_in
ad_connect decode_engine_0/diag_abort_ctx deimos_regs_0/diag_abort_ctx_in
# Good-frame (tag-out) snapshot for the good-vs-abort A/B (0x2C/0x30)
ad_connect decode_engine_0/diag_tag_sig deimos_regs_0/diag_tag_sig_in
ad_connect decode_engine_0/diag_tag_ctx deimos_regs_0/diag_tag_ctx_in
# chan_est_0/clip_cnt wired below (after chan_est_0 creation)

# STF clear: split into unconditional (watchdog) + gated (playback_start).
# soft_clear is ignored inside stf_detect when stf_end_armed — prevents the
# HIL re-arm race that drops ~0.5% of rate-6 frames at the STF level.
ad_connect decode_engine_0/watchdog_reset stf_detect_0/clear
ad_connect hil_ctrl_0/playback_start stf_detect_0/soft_clear

# Pipeline acknowledgment
ad_connect acquisition_ctrl_0/pipeline_ack stf_detect_0/pipeline_ack
ad_connect decode_engine_0/corr_sample_pos_out acquisition_ctrl_0/wr_ptr

# SNAP_MODE register
ad_connect deimos_regs_0/snap_mode_out decode_engine_0/snap_mode

# Live IQ input to decode_engine
ad_connect hil_ctrl_0/iq_valid decode_engine_0/iq_valid_in
ad_connect hil_ctrl_0/iq_re decode_engine_0/iq_re_in
ad_connect hil_ctrl_0/iq_im decode_engine_0/iq_im_in

# ---- Channel Estimator ----
create_bd_cell -type module -reference chan_est chan_est_0

ad_connect sys_cpu_clk chan_est_0/clk
ad_connect sys_cpu_resetn chan_est_0/rst_n
ad_connect decode_engine_0/fft_bin_valid chan_est_0/bin_valid
ad_connect decode_engine_0/fft_bin_idx chan_est_0/bin_idx
ad_connect decode_engine_0/fft_bin_re chan_est_0/bin_re
ad_connect decode_engine_0/fft_bin_im chan_est_0/bin_im
ad_connect decode_engine_0/chan_est_start chan_est_0/start
ad_connect chan_est_0/done decode_engine_0/chan_est_done
# clip_cnt wiring deferred until LUT headroom available (D21 constraint:
# chan_est not OOC, connecting clip_cnt adds ~16 LUTs + control set).
# Register address reserved; firmware reads 0 until implemented.
ad_ip_instance xlconstant diag_clip_zero
ad_ip_parameter diag_clip_zero CONFIG.CONST_WIDTH 16
ad_ip_parameter diag_clip_zero CONFIG.CONST_VAL 0
ad_connect diag_clip_zero/dout deimos_regs_0/diag_clip_cnt_in

# ---- Equalizer ----
create_bd_cell -type module -reference equalizer equalizer_0

ad_connect sys_cpu_clk equalizer_0/clk
ad_connect sys_cpu_resetn equalizer_0/rst_n
ad_connect decode_engine_0/fft_bin_valid equalizer_0/fft_valid
ad_connect decode_engine_0/fft_bin_idx equalizer_0/fft_bin
ad_connect decode_engine_0/fft_bin_re equalizer_0/fft_re
ad_connect decode_engine_0/fft_bin_im equalizer_0/fft_im
ad_connect chan_est_0/h_inv_re equalizer_0/hinv_re
ad_connect chan_est_0/h_inv_im equalizer_0/hinv_im
ad_connect chan_est_0/shift_val equalizer_0/shift_val
ad_connect decode_engine_0/eq_start equalizer_0/start
ad_connect equalizer_0/symbol_done decode_engine_0/eq_done
ad_connect equalizer_0/data_valid decode_engine_0/eq_data_valid
ad_connect equalizer_0/data_re decode_engine_0/eq_data_re
ad_connect equalizer_0/data_im decode_engine_0/eq_data_im
ad_connect equalizer_0/pilot_valid decode_engine_0/eq_pilot_valid
ad_connect equalizer_0/pilot_re decode_engine_0/eq_pilot_re
ad_connect equalizer_0/pilot_im decode_engine_0/eq_pilot_im
ad_connect equalizer_0/rd_addr decode_engine_0/eq_rd_addr
ad_connect decode_engine_0/ce_rd_addr chan_est_0/rd_addr
ad_connect chan_est_0/h_inv_re decode_engine_0/ce_h_inv_re
ad_connect chan_est_0/h_inv_im decode_engine_0/ce_h_inv_im

# ---- Pilot Tracking ----
create_bd_cell -type module -reference pilot_track pilot_track_0

ad_connect sys_cpu_clk pilot_track_0/clk
ad_connect sys_cpu_resetn pilot_track_0/rst_n
ad_connect decode_engine_0/symbol_idx_out pilot_track_0/symbol_idx
ad_connect decode_engine_0/symbol_start_out pilot_track_0/symbol_start
ad_connect decode_engine_0/is_signal_out pilot_track_0/is_signal
ad_connect equalizer_0/pilot_valid pilot_track_0/pilot_valid
ad_connect equalizer_0/pilot_re pilot_track_0/pilot_re
ad_connect equalizer_0/pilot_im pilot_track_0/pilot_im
ad_connect equalizer_0/pilot_idx pilot_track_0/pilot_idx
ad_connect equalizer_0/data_valid pilot_track_0/data_valid_in
ad_connect equalizer_0/data_re pilot_track_0/data_re_in
ad_connect equalizer_0/data_im pilot_track_0/data_im_in
ad_connect equalizer_0/data_idx pilot_track_0/data_idx_in

# ---- Demapper Input Mux ----
create_bd_cell -type module -reference demap_mux demap_mux_0

ad_connect decode_engine_0/is_signal_out demap_mux_0/is_signal
ad_connect equalizer_0/data_valid demap_mux_0/eq_data_valid
ad_connect equalizer_0/data_re demap_mux_0/eq_data_re
ad_connect equalizer_0/data_im demap_mux_0/eq_data_im
ad_connect pilot_track_0/data_valid_out demap_mux_0/pt_data_valid
ad_connect pilot_track_0/data_re_out demap_mux_0/pt_data_re
ad_connect pilot_track_0/data_im_out demap_mux_0/pt_data_im

# ---- Demapper ----
create_bd_cell -type module -reference demapper demapper_0

ad_connect sys_cpu_clk demapper_0/clk
ad_connect sys_cpu_resetn demapper_0/rst_n
ad_connect demap_mux_0/valid_out demapper_0/valid_in
ad_connect demap_mux_0/re_out demapper_0/sym_re
ad_connect demap_mux_0/im_out demapper_0/sym_im
ad_connect decode_engine_0/rate_mode_out demapper_0/rate_mode
ad_connect decode_engine_0/norm_out demapper_0/norm

# ---- Deinterleaver ----
create_bd_cell -type module -reference deinterleaver deinterleaver_0

ad_connect sys_cpu_clk deinterleaver_0/clk
ad_connect sys_cpu_resetn deinterleaver_0/rst_n
ad_connect demapper_0/wide_valid deinterleaver_0/valid_in
ad_connect demapper_0/soft_wide0 deinterleaver_0/soft_in0
ad_connect demapper_0/soft_wide1 deinterleaver_0/soft_in1
ad_connect demapper_0/soft_wide2 deinterleaver_0/soft_in2
ad_connect demapper_0/soft_wide3 deinterleaver_0/soft_in3
ad_connect demapper_0/soft_wide4 deinterleaver_0/soft_in4
ad_connect demapper_0/soft_wide5 deinterleaver_0/soft_in5
ad_connect decode_engine_0/rate_mode_out deinterleaver_0/rate_mode

# ---- Depuncturer ----
create_bd_cell -type module -reference depuncturer depuncturer_0

ad_connect sys_cpu_clk depuncturer_0/clk
ad_connect sys_cpu_resetn depuncturer_0/rst_n
ad_connect deinterleaver_0/valid_out depuncturer_0/valid_in
ad_connect deinterleaver_0/soft_out0 depuncturer_0/soft_in0
ad_connect deinterleaver_0/soft_out1 depuncturer_0/soft_in1
ad_connect decode_engine_0/code_rate_out depuncturer_0/code_rate
ad_connect decode_engine_0/depunct_restart depuncturer_0/symbol_start
ad_connect depuncturer_0/valid_out decode_engine_0/depunct_valid
ad_connect deinterleaver_0/deint_done decode_engine_0/deint_done

# ---- Soft Pairer + Viterbi ----
create_bd_cell -type module -reference soft_pairer soft_pairer_0

ad_connect sys_cpu_clk soft_pairer_0/clk
ad_connect sys_cpu_resetn soft_pairer_0/rst_n
ad_connect decode_engine_0/vit_frame_start soft_pairer_0/frame_start
ad_connect depuncturer_0/valid_out soft_pairer_0/valid_in
ad_connect depuncturer_0/soft_out soft_pairer_0/soft_in

create_bd_cell -type module -reference viterbi_k7 viterbi_k7_0

ad_connect sys_cpu_clk viterbi_k7_0/clk
ad_connect sys_cpu_resetn viterbi_k7_0/rst_n
ad_connect decode_engine_0/vit_frame_start viterbi_k7_0/frame_start
ad_connect decode_engine_0/vit_streaming_mode viterbi_k7_0/streaming_mode

# Vit FIFO (backpressure absorber)
create_bd_cell -type module -reference vit_fifo vit_fifo_0

ad_connect sys_cpu_clk vit_fifo_0/clk
ad_connect sys_cpu_resetn vit_fifo_0/rst_n
ad_connect decode_engine_0/vit_frame_start vit_fifo_0/frame_start
ad_connect soft_pairer_0/valid_out vit_fifo_0/wr_valid
ad_connect soft_pairer_0/soft0 vit_fifo_0/wr_soft0
ad_connect soft_pairer_0/soft1 vit_fifo_0/wr_soft1
ad_connect vit_fifo_0/rd_valid viterbi_k7_0/valid_in
ad_connect vit_fifo_0/rd_soft0 viterbi_k7_0/soft0
ad_connect vit_fifo_0/rd_soft1 viterbi_k7_0/soft1
ad_connect viterbi_k7_0/busy vit_fifo_0/vit_busy
ad_connect decode_engine_0/vit_flush vit_fifo_0/flush_in
ad_connect vit_fifo_0/flush_out viterbi_k7_0/flush
ad_connect vit_fifo_0/rd_valid decode_engine_0/diag_vit_valid
ad_connect vit_fifo_0/rd_soft0 decode_engine_0/diag_vit_soft0
ad_connect vit_fifo_0/rd_soft1 decode_engine_0/diag_vit_soft1
ad_connect vit_fifo_0/overflow decode_engine_0/diag_vit_overflow

# Backpressure (lever 2a-prime): vit_fifo full stalls soft_pairer and
# depuncturer; depuncturer pair FIFO full stalls the deinterleaver.
ad_connect vit_fifo_0/full soft_pairer_0/stall_in
ad_connect vit_fifo_0/full depuncturer_0/stall_in
ad_connect depuncturer_0/fifo_full deinterleaver_0/stall_in

# Pilot track diagnostics
ad_connect pilot_track_0/diag_phase_acc decode_engine_0/diag_phase_acc
ad_connect pilot_track_0/diag_atan2_x decode_engine_0/diag_atan2_x
ad_connect pilot_track_0/diag_atan2_y decode_engine_0/diag_atan2_y
ad_connect pilot_track_0/diag_state decode_engine_0/diag_pt_state
ad_connect pilot_track_0/diag_pilot_received decode_engine_0/diag_pt_pilot_received
ad_connect pilot_track_0/diag_data_count decode_engine_0/diag_pt_data_count

# Viterbi output to decode_engine
ad_connect viterbi_k7_0/valid_out decode_engine_0/vit_valid
ad_connect viterbi_k7_0/bit_out decode_engine_0/vit_bit

# ---- Descrambler ----
create_bd_cell -type module -reference descrambler descrambler_0

ad_connect sys_cpu_clk descrambler_0/clk
ad_connect sys_cpu_resetn descrambler_0/rst_n
# fcs_frame_start (not vit_frame_start): descrambler state reset must
# align with fcs_check/psdu_packer — see rx_pipeline.v u_descrambler.
ad_connect decode_engine_0/fcs_frame_start descrambler_0/frame_start
ad_connect viterbi_k7_0/valid_out descrambler_0/valid_in
ad_connect viterbi_k7_0/bit_out descrambler_0/bit_in
ad_connect descrambler_0/valid_out decode_engine_0/descr_valid
ad_connect descrambler_0/bit_out decode_engine_0/descr_bit

# ---- FCS Check ----
create_bd_cell -type module -reference fcs_check fcs_check_0

ad_connect sys_cpu_clk fcs_check_0/clk
ad_connect sys_cpu_resetn fcs_check_0/rst_n
ad_connect decode_engine_0/fcs_frame_start fcs_check_0/frame_start
ad_connect descrambler_0/valid_out fcs_check_0/valid_in
ad_connect descrambler_0/bit_out fcs_check_0/bit_in
ad_connect decode_engine_0/psdu_len_out fcs_check_0/psdu_len
ad_connect fcs_check_0/fcs_valid decode_engine_0/fcs_valid_in
ad_connect fcs_check_0/fcs_fail decode_engine_0/fcs_fail_in

# ---- PSDU Packer ----
create_bd_cell -type module -reference psdu_packer psdu_packer_0

ad_connect sys_cpu_clk psdu_packer_0/clk
ad_connect sys_cpu_resetn psdu_packer_0/rst_n
connect_bd_net -net [get_bd_nets -of_objects [get_bd_pins descrambler_0/valid_out]] \
    [get_bd_pins psdu_packer_0/valid_in]
connect_bd_net -net [get_bd_nets -of_objects [get_bd_pins descrambler_0/bit_out]] \
    [get_bd_pins psdu_packer_0/bit_in]
connect_bd_net -net [get_bd_nets -of_objects [get_bd_pins decode_engine_0/fcs_frame_start]] \
    [get_bd_pins psdu_packer_0/frame_start]
connect_bd_net -net [get_bd_nets -of_objects [get_bd_pins decode_engine_0/psdu_len_out]] \
    [get_bd_pins psdu_packer_0/psdu_len]

# ---- Tag FIFO ----
create_bd_cell -type module -reference tag_fifo_axi tag_fifo_axi_0

ad_connect sys_cpu_clk tag_fifo_axi_0/clk
ad_connect sys_rst_inv/Res tag_fifo_axi_0/rst

ad_connect decode_engine_0/tag_valid tag_fifo_axi_0/tag_wr_valid
ad_connect decode_engine_0/tag_rate tag_fifo_axi_0/tag_wr_rate
ad_connect decode_engine_0/tag_length tag_fifo_axi_0/tag_wr_length
ad_connect decode_engine_0/tag_fcs_ok tag_fifo_axi_0/tag_wr_fcs_ok
ad_connect decode_engine_0/tag_abort tag_fifo_axi_0/tag_abort_in

ad_connect iq_dma_rx_0/ddr_wr_ptr_out decode_engine_0/ddr_wr_ptr

ad_cpu_interconnect 0x7C510000 tag_fifo_axi_0
delete_bd_objs [get_bd_addr_segs sys_ps7/Data/SEG_data_tag_fifo_axi_0]
create_bd_addr_seg -range 0x1000 -offset 0x7C510000 \
    [get_bd_addr_spaces sys_ps7/Data] \
    [get_bd_addr_segs tag_fifo_axi_0/s_axi/reg0] \
    SEG_data_tag_fifo_axi_0

# PSDU packer → tag_fifo_axi
ad_connect psdu_packer_0/byte_out tag_fifo_axi_0/psdu_byte_in
ad_connect psdu_packer_0/byte_valid tag_fifo_axi_0/psdu_byte_valid
ad_connect psdu_packer_0/frame_start_out tag_fifo_axi_0/psdu_frame_start
ad_connect psdu_packer_0/frame_done tag_fifo_axi_0/psdu_frame_done

# ---- Reconnect snap probe (override styx default connection) ----
# Styx connected snap to raw HIL output (hil_ctrl_0/iq_* nets). We reconnect
# snap to decode_engine observation. disconnect_bd_net removes only the snap
# pins from shared nets; the receiver pipeline share stays intact.
disconnect_bd_net [get_bd_nets -of_objects [get_bd_pins snap_axi_0/sample_data]] [get_bd_pins snap_axi_0/sample_data]
disconnect_bd_net [get_bd_nets -of_objects [get_bd_pins snap_axi_0/sample_valid]] [get_bd_pins snap_axi_0/sample_valid]
disconnect_bd_net [get_bd_nets -of_objects [get_bd_pins snap_axi_0/ext_trig]] [get_bd_pins snap_axi_0/ext_trig]

ad_connect decode_engine_0/snap_data snap_axi_0/sample_data
ad_connect decode_engine_0/snap_valid snap_axi_0/sample_valid
ad_connect decode_engine_0/snap_trig snap_axi_0/ext_trig
