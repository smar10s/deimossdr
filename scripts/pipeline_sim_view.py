# SPDX-License-Identifier: MIT
# pipeline_sim_view.py — declarative spec for generating the simulation view of
# the receiver pipeline from the block design (see DECISIONS.md D19).
#
# The block design (fpga/project/system_bd.tcl) is the single authored source
# of truth for the receiver pipeline wiring. scripts/gen_sim_pipeline.py reads
# it and emits fpga/rtl/rx_pipeline.v and fpga/rtl/rx_frontend.v. Everything in
# this file is the *simulation harness* delta: the wrapper port lists, the
# mapping of BD boundary endpoints to wrapper ports or literals, the net names
# tests read, and the sim-only assigns/assertions. The 147 module-to-module
# edges are NOT listed here — they come from the BD.

PIPE_CELLS = [
    "cfo_mixer_0", "decode_engine_0", "acquisition_ctrl_0", "frame_fifo_0",
    "fft64_sdf_0", "ltf_correlator_0", "ltf_peak_detect_0", "chan_est_0",
    "equalizer_0", "pilot_track_0", "demap_mux_0", "demapper_0",
    "deinterleaver_0", "depuncturer_0", "soft_pairer_0", "viterbi_k7_0",
    "vit_fifo_0", "descrambler_0", "fcs_check_0", "psdu_packer_0",
]
FE_CELLS = ["stf_detect_0", "cfo_est_0"]

# Generated wrapper modules. rx_frontend instantiates the rx_pipeline child.
CELLS = {"rx_pipeline": PIPE_CELLS, "rx_frontend": FE_CELLS}
CHILD = {"rx_frontend": "rx_pipeline"}
CHILD_CELLS = {"rx_frontend": PIPE_CELLS}

# Ordered wrapper port lists (name, direction, width). Transcribed from the
# pre-inversion module headers; tests drive/read these by name.
PORTS = {
    "rx_pipeline": [
        ("clk", "input", 1), ("rst_n", "input", 1),
        ("frame_detect", "input", 1), ("stf_end", "input", 1),
        ("stf_end_skip", "input", 16), ("ltf_skip", "input", 16),
        ("iq_valid_in", "input", 1), ("iq_re_in", "input", 12),
        ("iq_im_in", "input", 12), ("phase_inc_in", "input", 16),
        ("cfo_done_in", "input", 1), ("ddr_wr_ptr", "input", 25),
        ("tag_valid", "output", 1), ("tag_rate", "output", 4),
        ("tag_length", "output", 12), ("tag_fcs_ok", "output", 1),
        ("seq_done", "output", 1), ("signal_valid", "output", 1),
        ("parsed_rate", "output", 4), ("parsed_length", "output", 12),
        ("pipeline_ack", "output", 1), ("watchdog_reset", "output", 1),
        ("state", "output", 4), ("fcs_valid", "output", 1),
        ("fcs_fail", "output", 1), ("tag_abort", "output", 1),
        ("diag_frames_found", "output", 8), ("diag_frames_rejected", "output", 8),
        ("diag_drop_cnt", "output", 16), ("diag_clip_cnt", "output", 16),
        ("psdu_byte_valid", "output", 1), ("psdu_byte_out", "output", 8),
        ("psdu_frame_done", "output", 1), ("psdu_byte_count", "output", 12),
    ],
    "rx_frontend": [
        ("clk", "input", 1), ("rst_n", "input", 1),
        ("iq_valid_in", "input", 1), ("iq_i_in", "input", 12),
        ("iq_q_in", "input", 12), ("playback_start", "input", 1),
        ("stf_end_skip", "input", 16), ("ltf_skip", "input", 16),
        ("stf_threshold", "input", 8),
        ("tag_valid", "output", 1), ("tag_rate", "output", 4),
        ("tag_length", "output", 12), ("tag_fcs_ok", "output", 1),
        ("frame_detect", "output", 1), ("frame_offset", "output", 25),
        ("stf_end", "output", 1), ("cfo_done", "output", 1),
        ("phase_inc", "output", 16), ("seq_done", "output", 1),
        ("signal_valid", "output", 1), ("parsed_rate", "output", 4),
        ("parsed_length", "output", 12), ("state", "output", 4),
        ("fcs_valid", "output", 1), ("fcs_fail", "output", 1),
        ("tag_abort", "output", 1), ("diag_frames_found", "output", 8),
        ("diag_frames_rejected", "output", 8), ("diag_drop_cnt", "output", 16),
        ("diag_clip_cnt", "output", 16), ("psdu_byte_valid", "output", 1),
        ("psdu_byte_out", "output", 8), ("psdu_frame_done", "output", 1),
        ("psdu_byte_count", "output", 12),
    ],
}

CLK_PORTS = {"sys_cpu_clk": "clk", "sys_cpu_resetn": "rst_n"}

# BD cell -> simulation instance name (the hierarchy tests read).
INST_NAMES = {
    "stf_detect_0": "u_stf_detect", "cfo_est_0": "u_cfo_est",
    "cfo_mixer_0": "u_cfo_mixer", "decode_engine_0": "u_decode_engine",
    "acquisition_ctrl_0": "u_acquisition_ctrl", "frame_fifo_0": "u_frame_fifo",
    "fft64_sdf_0": "u_fft", "ltf_correlator_0": "u_ltf_corr",
    "ltf_peak_detect_0": "u_ltf_peak", "chan_est_0": "u_chan_est",
    "equalizer_0": "u_equalizer", "pilot_track_0": "u_pilot_track",
    "demap_mux_0": "u_demap_mux", "demapper_0": "u_demapper",
    "deinterleaver_0": "u_deinterleaver", "depuncturer_0": "u_depuncturer",
    "soft_pairer_0": "u_soft_pairer", "viterbi_k7_0": "u_viterbi",
    "vit_fifo_0": "u_vit_fifo", "descrambler_0": "u_descrambler",
    "fcs_check_0": "u_fcs_check", "psdu_packer_0": "u_psdu_packer",
}

# BD boundary endpoint -> external resolution.
#   ("port", name)  connect directly to wrapper port `name`
#   ("net",  name)  connect to an internal net (declared by the emitter)
#   ("lit",  expr)  tie to a Verilog literal
#   ("unconn", None) leave unconnected
BOUNDARY = {
    "rx_pipeline": {
        ("cfo_mixer_0", "phase_inc"): ("port", "phase_inc_in"),
        ("cfo_mixer_0", "iq_valid_in"): ("port", "iq_valid_in"),
        ("cfo_mixer_0", "iq_i_in"): ("port", "iq_re_in"),
        ("cfo_mixer_0", "iq_q_in"): ("port", "iq_im_in"),
        ("decode_engine_0", "iq_valid_in"): ("port", "iq_valid_in"),
        ("decode_engine_0", "iq_re_in"): ("port", "iq_re_in"),
        ("decode_engine_0", "iq_im_in"): ("port", "iq_im_in"),
        ("decode_engine_0", "ddr_wr_ptr"): ("port", "ddr_wr_ptr"),
        ("acquisition_ctrl_0", "frame_detect"): ("port", "frame_detect"),
        ("acquisition_ctrl_0", "stf_end"): ("port", "stf_end"),
        ("acquisition_ctrl_0", "phase_inc"): ("port", "phase_inc_in"),
        ("acquisition_ctrl_0", "cfo_done"): ("port", "cfo_done_in"),
        ("acquisition_ctrl_0", "diag_frames_found"): ("port", "diag_frames_found"),
        ("acquisition_ctrl_0", "diag_frames_rejected"): ("port", "diag_frames_rejected"),
        ("acquisition_ctrl_0", "pipeline_ack"): ("port", "pipeline_ack"),
        ("decode_engine_0", "diag_drop_cnt"): ("port", "diag_drop_cnt"),
        ("decode_engine_0", "watchdog_reset"): ("port", "watchdog_reset"),
        # BD drives snap_mode from the deimos_regs register. Sim does not model
        # the register file; snap observation is a debug feature, not decode
        # path. Tied to 0 (pre-inversion allowlist entry, now explicit).
        ("decode_engine_0", "snap_mode"): ("lit", "3'd0"),
        ("decode_engine_0", "tag_valid"): ("port", "tag_valid"),
        ("decode_engine_0", "tag_rate"): ("port", "tag_rate"),
        ("decode_engine_0", "tag_length"): ("port", "tag_length"),
        ("decode_engine_0", "tag_fcs_ok"): ("port", "tag_fcs_ok"),
        ("decode_engine_0", "tag_abort"): ("port", "tag_abort"),
        ("decode_engine_0", "snap_data"): ("net", "snap_data"),
        ("decode_engine_0", "snap_valid"): ("net", "snap_valid"),
        ("decode_engine_0", "snap_trig"): ("net", "snap_trig"),
        ("psdu_packer_0", "byte_out"): ("port", "psdu_byte_out"),
        ("psdu_packer_0", "byte_valid"): ("port", "psdu_byte_valid"),
        # BD routes frame_start_out to tag_fifo_axi (platform, not modeled in
        # sim). Sim observes the PSDU path via byte_valid/frame_done/byte_count.
        ("psdu_packer_0", "frame_start_out"): ("unconn", None),
        ("psdu_packer_0", "frame_done"): ("port", "psdu_frame_done"),
    },
    "rx_frontend": {
        ("stf_detect_0", "iq_valid"): ("port", "iq_valid_in"),
        ("stf_detect_0", "iq_i"): ("port", "iq_i_in"),
        ("stf_detect_0", "iq_q"): ("port", "iq_q_in"),
        ("stf_detect_0", "threshold"): ("port", "stf_threshold"),
        # BD drives enable from deimos_regs. Sim ties high (always enabled);
        # pre-inversion allowlist entry, now explicit.
        ("stf_detect_0", "enable"): ("lit", "1'b1"),
        ("stf_detect_0", "soft_clear"): ("port", "playback_start"),
        ("stf_detect_0", "clear"): ("net", "watchdog_reset_w"),
        ("stf_detect_0", "pipeline_ack"): ("net", "acq_pipeline_ack"),
        ("stf_detect_0", "stf_end"): ("net", "stf_end_w"),
        ("stf_detect_0", "frame_offset"): ("port", "frame_offset"),
        ("cfo_est_0", "iq_valid"): ("port", "iq_valid_in"),
        ("cfo_est_0", "iq_i"): ("port", "iq_i_in"),
        ("cfo_est_0", "iq_q"): ("port", "iq_q_in"),
        ("cfo_est_0", "phase_inc"): ("net", "phase_inc_w"),
        ("cfo_est_0", "done"): ("net", "cfo_done_w"),
    },
}

# Sim-only connections with no BD edge (documented divergences).
SIM_EDGES = {
    "rx_pipeline": {
        # BD defers chan_est/clip_cnt (D21 LUT budget, system_bd.tcl:196-198);
        # sim observes it for diagnostics.
        ("chan_est_0", "clip_cnt"): ("port", "diag_clip_cnt"),
        # psdu byte_count is not consumed by tag_fifo_axi; exposed for tests.
        ("psdu_packer_0", "byte_count"): ("port", "psdu_byte_count"),
        # decode_engine status ports feed deimos_regs in BD; exposed as ports
        # for cocotb. BD does not wire them (no readback).
        ("decode_engine_0", "seq_done"): ("port", "seq_done"),
        ("decode_engine_0", "signal_valid"): ("port", "signal_valid"),
        ("decode_engine_0", "parsed_rate"): ("port", "parsed_rate"),
        ("decode_engine_0", "parsed_length"): ("port", "parsed_length"),
    },
    "rx_frontend": {},
}

# Internal net names preserved for tests and /* verilator public */.
NET_NAMES = {
    ("deinterleaver_0", "deint_done"): "deint_done",
    ("vit_fifo_0", "full"): "vitf_full",
    ("depuncturer_0", "fifo_full"): "depunct_full",
    ("deinterleaver_0", "valid_out"): "deint_valid",
    ("depuncturer_0", "valid_out"): "depunct_valid",
    ("soft_pairer_0", "valid_out"): "pair_valid",
    ("viterbi_k7_0", "valid_out"): "vit_valid",
    ("decode_engine_0", "vit_frame_start"): "vit_frame_start",
    ("decode_engine_0", "code_rate_out"): "code_rate_out",
    ("equalizer_0", "data_valid"): "eq_data_valid",
    ("equalizer_0", "data_re"): "eq_data_re",
    ("equalizer_0", "data_im"): "eq_data_im",
    ("equalizer_0", "pilot_valid"): "eq_pilot_valid",
    ("equalizer_0", "pilot_re"): "eq_pilot_re",
    ("equalizer_0", "pilot_im"): "eq_pilot_im",
    ("pilot_track_0", "data_valid_out"): "pt_data_valid",
    ("pilot_track_0", "data_re_out"): "pt_data_re",
    ("pilot_track_0", "data_im_out"): "pt_data_im",
    ("decode_engine_0", "fft_bin_valid"): "fft_bin_valid",
    ("decode_engine_0", "fft_bin_idx"): "fft_bin_idx",
    ("decode_engine_0", "fft_bin_re"): "fft_bin_re",
    ("decode_engine_0", "fft_bin_im"): "fft_bin_im",
    ("fcs_check_0", "fcs_valid"): "fcs_valid_w",
    ("fcs_check_0", "fcs_fail"): "fcs_fail_w",
    ("decode_engine_0", "vit_streaming_mode"): "vit_streaming_mode",
    # rx_frontend
    ("stf_detect_0", "frame_detect"): "frame_detect_w",
}
PUBLIC = {"deint_done", "vitf_full", "depunct_full"}

# rx_frontend child (rx_pipeline) port -> FE expression. Only exceptions to
# "same-named FE port"; everything else auto-connects to the FE port of the
# same name.
CHILD_CONN = {
    "ddr_wr_ptr": "sim_ddr_wr_ptr",
    "frame_detect": "frame_detect_w",
    "stf_end": "stf_end_w",
    "phase_inc_in": "phase_inc_w",
    "cfo_done_in": "cfo_done_w",
    "watchdog_reset": "watchdog_reset_w",
    "pipeline_ack": "acq_pipeline_ack",
    "iq_re_in": "iq_i_in",
    "iq_im_in": "iq_q_in",
}

EXTRA_DECL = {
    "rx_pipeline": "",
    "rx_frontend": """
    // Free-running sample counter models the DDR write pointer in the absence
    // of iq_dma_rx (rx_frontend.v:96-105 pre-inversion).
    reg [24:0] sim_ddr_wr_ptr;
    always @(posedge clk) begin
        if (!rst_n)
            sim_ddr_wr_ptr <= 0;
        else if (iq_valid_in)
            sim_ddr_wr_ptr <= sim_ddr_wr_ptr + 1;
    end
""",
}

EXTRA_BODY = {
    "rx_pipeline": """
    // 4-bit FSM state exported from decode_engine snap_data for tests
    // (rx_pipeline.v:393 pre-inversion).
    assign state = snap_data[30:27];

    // fcs_check outputs are read by decode_engine and exposed for tests.
    assign fcs_valid = fcs_valid_w;
    assign fcs_fail  = fcs_fail_w;

`ifdef SIM
    // Puncture-phase assertions (rx_pipeline.v:641-682 pre-inversion).
    always @(posedge clk) begin
        if (rst_n && depunct_valid && !vitf_full &&
            code_rate_out != 2'd0 && !u_soft_pairer.have_first &&
            u_depuncturer.emit_par !== 1'b0)
            $error("[rx_pipeline] PUNCTURE PHASE SLIP: soft_pairer latched an odd-parity coded bit as G0 at %0t; a bit was dropped or duplicated between depuncturer and soft_pairer", $time);
    end
    always @(posedge clk) begin
        if (rst_n && vit_frame_start && vit_streaming_mode &&
            u_depuncturer.pat_pos !== 3'd0)
            $error("[rx_pipeline] PUNCTURE PATTERN CARRYOVER: DATA phase started with pat_pos=%0d (expected 0) at %0t; the previous frame left the depuncturer mid-group",
                   u_depuncturer.pat_pos, $time);
    end
`endif
""",
    "rx_frontend": """
    assign frame_detect = frame_detect_w;
    assign stf_end      = stf_end_w;
    assign cfo_done     = cfo_done_w;
    assign phase_inc    = phase_inc_w;
""",
}
