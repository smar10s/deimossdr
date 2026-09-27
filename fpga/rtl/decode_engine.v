// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/decode_engine.v — Decode-only sequencer (FIFO-fed, no acquisition)
//
// Refactored from the old monolithic rx_top: all acquisition/pending logic removed.
// Reads frame descriptors from frame_fifo, decodes through FCS/tag.
// The circular buffer and FFT feed logic are identical to the proven rx_top.
//
// States removed: S_SKIP, S_CAPTURE, S_START_CE (acquisition handles these)
// Logic removed: pending path, trigger handling, stf_end consumption,
//                inline CFO latching, windowed-max peak finding
//
// New S_IDLE: pops from FIFO when not empty → loads ltf_pos/phase_inc →
// resets pipeline → starts channel estimation → S_FFT_FEED.
//
// Architecture: raw IQ written to circular buffer continuously.
// Decode engine reads from buffer via rd_ptr during S_FFT_FEED.
// CFO correction applied during feed via internal cfo_mixer instance.

module decode_engine (
    input  wire        clk,
    input  wire        rst_n,

    // Snap mode (observation point selector)
    input  wire [2:0]  snap_mode,

    // Live IQ input (raw, pre-mixer — written to circular buffer)
    input  wire        iq_valid_in,
    input  wire [11:0] iq_re_in,
    input  wire [11:0] iq_im_in,

    // Frame descriptor FIFO interface (from frame_fifo)
    input  wire        fifo_empty,
    input  wire [15:0] pop_ltf_pos,
    input  wire [15:0] pop_phase_inc,
    output reg         fifo_pop,

    // FFT interface (external fft64_sdf connected via BD)
    output wire        fft_din_valid_out,
    output wire signed [15:0] fft_din_re_out,
    output wire signed [15:0] fft_din_im_out,
    output wire        fft_rst_n_out,
    input  wire        fft_dout_valid_in,
    input  wire signed [15:0] fft_dout_re_in,
    input  wire signed [15:0] fft_dout_im_in,
    input  wire [5:0]  fft_dout_idx_in,

    // FFT bin output (registered, to chan_est and equalizer)
    output wire        fft_bin_valid,
    output wire [5:0]  fft_bin_idx,
    output wire signed [15:0] fft_bin_re,
    output wire signed [15:0] fft_bin_im,

    // LTF correlator interface (peak detect — diagnostic only in decode)
    input  wire        ltf_peak_found_in,
    input  wire [15:0] ltf_peak_pos_in,
    input  wire [28:0] ltf_peak_metric_in,
    output reg         ltf_peak_ack,
    output wire [15:0] corr_sample_pos_out,  // wr_ptr for acquisition_ctrl
    output wire        corr_enable_out,      // always-on for free-running correlator

    // chan_est control
    output reg         chan_est_start,

    // chan_est read port
    output reg  [5:0]  ce_rd_addr,
    input  wire signed [15:0] ce_h_inv_re,
    input  wire signed [15:0] ce_h_inv_im,

    // chan_est status
    input  wire        chan_est_done,

    // Equalizer control
    output reg         eq_start,
    input  wire        eq_done,

    // Equalizer rd_addr mux
    input  wire [5:0]  eq_rd_addr,
    output reg         eq_rd_sel,

    // Equalizer data (for snap capture)
    input  wire        eq_data_valid,
    input  wire signed [15:0] eq_data_re,
    input  wire signed [15:0] eq_data_im,
    input  wire        eq_pilot_valid,
    input  wire signed [15:0] eq_pilot_re,
    input  wire signed [15:0] eq_pilot_im,

    // Deinterleaver emit-complete detection (per-symbol advance signal)
    input  wire        deint_done,
    input  wire        depunct_valid,

    // Viterbi decoded output
    output reg         vit_frame_start,
    output reg         vit_flush,
    output reg         vit_streaming_mode,
    input  wire        vit_valid,
    input  wire        vit_bit,

    // Descrambler output (for DATA — descrambled bits feed FCS)
    input  wire        descr_valid,
    input  wire        descr_bit,

    // FCS check interface
    output reg         fcs_frame_start,
    input  wire        fcs_valid_in,
    input  wire        fcs_fail_in,

    // Pipeline configuration outputs (dynamic, driven by SIGNAL decode)
    output reg  [1:0]  rate_mode_out,
    output reg  [1:0]  code_rate_out,
    // Depuncturer restart: one pulse per frame, at SIGNAL setup. Discards
    // the previous frame's undrained coded-bit tail and restarts the
    // puncture pattern at position 0. See the depuncturer instantiation in
    // rx_pipeline.v for why the tail must be dropped rather than decoded.
    output reg         depunct_restart,
    output reg  [15:0] norm_out,
    output reg  [11:0] psdu_len_out,

    // SIGNAL field parse result
    output reg         signal_valid,
    output reg  [3:0]  parsed_rate,
    output reg  [11:0] parsed_length,

    // DDR ring buffer write pointer (unused in this module — legacy snap
    // plumbing retained for rx_pipeline wiring compatibility)
    input  wire [24:0] ddr_wr_ptr,

    // Tag output (frame metadata for ARM)
    output reg         tag_valid,
    output reg  [3:0]  tag_rate,
    output reg  [11:0] tag_length,
    output reg         tag_fcs_ok,

    // Snap interface
    output wire        snap_valid,
    output wire [31:0] snap_data,
    output reg         snap_trig,

    // Diagnostic inputs
    input  wire        diag_vit_valid,
    input  wire [7:0]  diag_vit_soft0,
    input  wire [7:0]  diag_vit_soft1,
    input  wire        diag_vit_overflow,
    input  wire signed [15:0] diag_phase_acc,
    input  wire [15:0] diag_atan2_x,
    input  wire [15:0] diag_atan2_y,
    input  wire [2:0]  diag_pt_state,
    input  wire [3:0]  diag_pt_pilot_received,
    input  wire [5:0]  diag_pt_data_count,

    // Sequence status
    output reg         seq_done,
    output reg         tag_abort,
    output wire        watchdog_reset,

    // Diagnostic counter: IQ samples dropped (write_ok false)
    output wire [15:0] diag_drop_cnt,

    // Abort diagnostics: per-reason counters + last SIG-parse abort snapshot
    output wire [31:0] diag_abort_cnts,  // {wd, ow, rate, sig} 8-bit counters
    output wire [31:0] diag_abort_sig,   // {8'b0, sig_bits[23:0]} at last SIG abort
    output wire [31:0] diag_abort_ctx,   // {ltf1_offset[15:0], frame_phase_inc[15:0]} at last SIG abort

    // Good-frame (tag-out) snapshot: A/B baseline against the abort snapshot.
    output wire [31:0] diag_tag_sig,     // {8'b0, sig_bits[23:0]} at last tag
    output wire [31:0] diag_tag_ctx,     // {ltf1_offset[15:0], frame_phase_inc[15:0]} at last tag

    // Pilot tracking interface
    output reg  [7:0]  symbol_idx_out,
    output reg         symbol_start_out,
    output reg         is_signal_out
);

    // =========================================================
    // Snap: observation mux — selected by snap_mode register
    // =========================================================
    reg [31:0] snap_data_mux;
    always @(*) begin
        case (snap_mode)
            3'd0: snap_data_mux = {1'b0, state, fcs_result_ok, parsed_rate, tag_fcs_ok,
                                    9'b0, parsed_length};
            3'd1: snap_data_mux = {29'b0, ltf_peak_found_in, 2'b0};
            3'd2: snap_data_mux = {iq_valid_in, 7'b0, iq_re_in, iq_im_in};
            3'd3: snap_data_mux = {diag_atan2_x, diag_atan2_y};
            3'd4: snap_data_mux = {diag_heartbeat, diag_accepted_cnt,
                                    diag_abort_cnt, 8'd0};
            3'd5: snap_data_mux = {31'b0, diag_vit_overflow};
            default: snap_data_mux = 32'd0;
        endcase
    end
    assign snap_valid = 1'b1;
    assign snap_data  = snap_data_mux;

    // =========================================================
    // Configuration
    // =========================================================

    // =========================================================
    // Rate table ROM (rate_code → {rate_mode, code_rate, n_dbps, norm})
    // =========================================================
    reg [1:0]  tbl_rate_mode;
    reg [1:0]  tbl_code_rate;
    reg [7:0]  tbl_n_dbps;
    reg [15:0] tbl_norm;
    reg        tbl_valid;

    always @(*) begin
        tbl_valid = 1;
        case (parsed_rate)
            4'b1011: begin tbl_rate_mode = 0; tbl_code_rate = 0; tbl_n_dbps = 24;  tbl_norm = 64; end // 6 Mbps
            4'b1111: begin tbl_rate_mode = 0; tbl_code_rate = 2; tbl_n_dbps = 36;  tbl_norm = 64; end // 9 Mbps
            4'b1010: begin tbl_rate_mode = 1; tbl_code_rate = 0; tbl_n_dbps = 48;  tbl_norm = 45; end // 12 Mbps
            4'b1110: begin tbl_rate_mode = 1; tbl_code_rate = 2; tbl_n_dbps = 72;  tbl_norm = 45; end // 18 Mbps
            4'b1001: begin tbl_rate_mode = 2; tbl_code_rate = 0; tbl_n_dbps = 96;  tbl_norm = 20; end // 24 Mbps
            4'b1101: begin tbl_rate_mode = 2; tbl_code_rate = 2; tbl_n_dbps = 144; tbl_norm = 20; end // 36 Mbps
            4'b1000: begin tbl_rate_mode = 3; tbl_code_rate = 1; tbl_n_dbps = 192; tbl_norm = 10; end // 48 Mbps
            4'b1100: begin tbl_rate_mode = 3; tbl_code_rate = 2; tbl_n_dbps = 216; tbl_norm = 10; end // 54 Mbps
            default: begin tbl_rate_mode = 0; tbl_code_rate = 0; tbl_n_dbps = 24;  tbl_norm = 64; tbl_valid = 0; end
        endcase
    end

    // =========================================================
    // IQ Circular Buffer (32768 × 24 bits — streaming decode)
    // =========================================================
    localparam BUFFER_SIZE   = 16'd32768;
    localparam GUARD_MARGIN  = 16'd4096;

    (* ram_style = "block" *)
    reg [23:0] circ_buf [0:BUFFER_SIZE-1];
    reg [15:0] wr_ptr;
    reg [15:0] rd_ptr;
    reg [7:0]  fill_wait_cnt;  // IQ samples counted while waiting for buffer fill

    // Producer/consumer tracking
    wire [15:0] num_avail = wr_ptr - rd_ptr;
    wire buf_full = (wr_ptr[15] != rd_ptr[15]) && (wr_ptr[14:0] == rd_ptr[14:0]);

    // Frame data protection: prevent writes near queued frame data
    wire [15:0] data_age = wr_ptr - pop_ltf_pos;
    wire frame_overwritten = !fifo_empty && (data_age >= BUFFER_SIZE);
    wire near_overwrite = !fifo_empty && !frame_overwritten &&
                          (data_age >= (BUFFER_SIZE - GUARD_MARGIN));
    wire write_ok = iq_valid_in && !buf_full && !near_overwrite;

    // Diagnostic: IQ drop counter placeholder — output tied to zero.
    // The actual counter is deferred until LUT headroom is available
    // (D21 constraint: 7 slices over at 89% utilization). The register
    // address and firmware interface exist; firmware reads 0 until the
    // counter is implemented.
    assign diag_drop_cnt = 16'd0;

    assign diag_abort_cnts = {abort_wd_cnt, abort_ow_cnt, abort_rate_cnt, abort_sig_cnt};
    assign diag_abort_sig = abort_sig_snap;
    assign diag_abort_ctx = abort_ctx_snap;

    assign diag_tag_sig = tag_sig_snap;
    assign diag_tag_ctx = tag_ctx_snap;

    // Registered S_IDLE decision terms: the data_age carry chain
    // (wr_ptr - pop_ltf_pos) plus its compares would otherwise stretch
    // from the frame_fifo pop_ltf_pos input through the fill_wait_cnt
    // CE (10 logic levels, timing-critical). These are polled every
    // cycle in S_IDLE, so one cycle of latency is harmless. write_ok's
    // near_overwrite guard stays combinational — the producer cannot
    // be delayed. Both registers are gated by !fifo_empty: while the
    // FIFO is empty pop_ltf_pos is stale and data_age is garbage, and
    // a stale-1 register would pop/abort a freshly arrived frame before
    // its IQ is in the buffer.
    reg frame_overwritten_r;
    reg fill_ready_r;
    always @(posedge clk) begin
        if (!rst_n) begin
            frame_overwritten_r <= 0;
            fill_ready_r        <= 0;
        end else begin
            frame_overwritten_r <= !fifo_empty && (data_age >= BUFFER_SIZE);
            fill_ready_r        <= !fifo_empty && (data_age >= 16'd208);
        end
    end

    // Buffer read (1-cycle latency — BRAM registered output)
    reg [23:0] buf_rd_data;
    always @(posedge clk) begin
        buf_rd_data <= circ_buf[rd_ptr[14:0]];
    end

    wire [11:0] buf_re = buf_rd_data[23:12];
    wire [11:0] buf_im = buf_rd_data[11:0];

    // =========================================================
    // Per-frame CFO latch + feed-path mixer
    // =========================================================
    reg [15:0] frame_phase_inc;
    reg        mixer_phase_reset;

    // Feed-path mixer instance
    wire        feed_mixer_valid_out;
    wire [11:0] feed_mixer_re_out;
    wire [11:0] feed_mixer_im_out;

    wire feeding = (state == S_FFT_FEED && !fft_draining && fft_rst_n);

    cfo_mixer u_feed_mixer (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (1'b1),
        .phase_reset  (mixer_phase_reset),
        .phase_inc    (frame_phase_inc),
        .iq_valid_in  (feeding & feed_valid),
        .iq_i_in      (buf_re),
        .iq_q_in      (buf_im),
        .iq_valid_out (feed_mixer_valid_out),
        .iq_i_out     (feed_mixer_re_out),
        .iq_q_out     (feed_mixer_im_out)
    );

    // =========================================================
    // FFT64 SDF — external BD entity, connected via ports
    // =========================================================
    reg        fft_rst_n;
    reg        fft_draining;
    reg [6:0]  fft_data_cnt;
    reg [5:0]  fft_out_cnt;
    reg [1:0]  fft_phase;

    wire fft_symbol_complete = fft_dout_valid_in && fft_rst_n && (fft_out_cnt == 6'd63);
    wire fft_in_active = (state == S_FFT_FEED) && fft_rst_n && !fft_symbol_complete;
    wire fft_din_valid = fft_in_active ?
                         (fft_draining ? 1'b1 : feed_mixer_valid_out) : 1'b0;
    wire signed [15:0] fft_din_re = fft_draining ? 16'sd0 : {{4{feed_mixer_re_out[11]}}, feed_mixer_re_out};
    wire signed [15:0] fft_din_im = fft_draining ? 16'sd0 : {{4{feed_mixer_im_out[11]}}, feed_mixer_im_out};

    assign fft_din_valid_out = fft_din_valid;
    assign fft_din_re_out    = fft_din_re;
    assign fft_din_im_out    = fft_din_im;
    assign fft_rst_n_out     = fft_rst_n;

    // Bit-reversal for natural-order bin index
    function [5:0] bit_rev6;
        input [5:0] x;
        bit_rev6 = {x[0], x[1], x[2], x[3], x[4], x[5]};
    endfunction

    // Registered bin output
    reg        fft_bin_valid_r;
    reg [5:0]  fft_bin_idx_r;
    reg signed [15:0] fft_bin_re_r;
    reg signed [15:0] fft_bin_im_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            fft_bin_valid_r <= 0;
            fft_bin_idx_r   <= 0;
            fft_bin_re_r    <= 0;
            fft_bin_im_r    <= 0;
        end else begin
            fft_bin_valid_r <= 0;
            if (fft_dout_valid_in && state == S_FFT_FEED && fft_rst_n) begin
                fft_bin_valid_r <= 1'b1;
                fft_bin_idx_r   <= bit_rev6(fft_out_cnt);
                fft_bin_re_r    <= fft_dout_re_in;
                fft_bin_im_r    <= fft_dout_im_in;
            end
        end
    end

    assign fft_bin_valid = fft_bin_valid_r;
    assign fft_bin_idx   = fft_bin_idx_r;
    assign fft_bin_re    = fft_bin_re_r;
    assign fft_bin_im    = fft_bin_im_r;

    // Export wr_ptr for acquisition_ctrl
    assign corr_sample_pos_out = wr_ptr;
    assign corr_enable_out     = 1'b1;

    // LTF T1 offset (loaded from FIFO)
    reg [15:0] ltf1_offset;

    // =========================================================
    // FSM States (13 states, 4 bits)
    // Removed: S_SKIP, S_CAPTURE, S_START_CE (acquisition handles these)
    // =========================================================
    localparam S_IDLE           = 4'd0;
    localparam S_FFT_FEED       = 4'd1;
    localparam S_WAIT_CE        = 4'd2;
    localparam S_EQ_SIG         = 4'd3;
    localparam S_WAIT_DEMAP_SIG = 4'd4;
    localparam S_WAIT_VIT_SIG   = 4'd5;
    localparam S_PARSE_SIGNAL   = 4'd6;
    localparam S_CONFIG_DATA    = 4'd7;
    localparam S_WAIT_DIV       = 4'd8;
    localparam S_EQ_DATA        = 4'd9;
    localparam S_WAIT_DEMAP_D   = 4'd10;
    localparam S_WAIT_FCS       = 4'd11;
    localparam S_TAG_OUT        = 4'd12;
    localparam S_DONE           = 4'd13;

    reg [3:0]  state;
    reg [6:0]  feed_cnt;
    reg        feed_valid;
    reg        feed_primed;
    reg        data_buf_wait;

    // SIGNAL field capture (24 bits from Viterbi)
    reg [23:0] sig_bits;
    reg [4:0]  sig_bit_cnt;
    reg        sig_capture_en;

    // DATA symbol tracking
    reg [10:0] n_sym;
    reg [10:0] sym_cnt;

    // FCS result
    reg        fcs_result_ok;
    reg        fcs_result_fail;
    reg [19:0] fcs_timeout_cnt;

    // Global watchdog
    reg [19:0] watchdog_cnt;

    // Diagnostic counters
    reg [7:0] diag_accepted_cnt;
    reg [7:0] diag_abort_cnt;
    reg [7:0] diag_heartbeat;

    // Abort diagnostics (per-reason counters + last SIG abort snapshot)
    reg [7:0]  abort_sig_cnt;
    reg [7:0]  abort_rate_cnt;
    reg [7:0]  abort_ow_cnt;
    reg [7:0]  abort_wd_cnt;
    reg [31:0] abort_sig_snap;
    reg [31:0] abort_ctx_snap;

    // Good-frame (tag-out) snapshot (see diag_tag_sig/diag_tag_ctx)
    reg [31:0] tag_sig_snap;
    reg [31:0] tag_ctx_snap;

    // Watchdog reset output.
    // Registered: the net crosses the BD hierarchy into stf_clear_or →
    // stf_detect.clear → rearm logic (8 LUT levels + a long route on the
    // stf_clear fanout, timing-critical). The watchdog acts on a ~10ms
    // timescale, so one extra cycle of latency is immaterial, and the
    // state-decode + counter-bit logic leaves the crossing path.
    reg watchdog_reset_r;
    always @(posedge clk) begin
        if (!rst_n)
            watchdog_reset_r <= 0;
        else
            watchdog_reset_r <= (state != S_IDLE) && watchdog_cnt[19];
    end
    assign watchdog_reset = watchdog_reset_r;

    // Symbol-complete detection: advance on deinterleaver emit complete
    // (lever 2a-prime).
    //
    // deint_done fires ~NCBPS/2 clocks before the depuncturer finishes
    // draining the symbol. The early advance overlaps that drain with the
    // next symbol's FFT/EQ lead-in, hiding it (spec §4: 6M ~361, 12M ~385,
    // 24M ~481 clk/sym). Safety: the stall chain from vit_fifo.full back
    // through depuncturer to the deinterleaver throttles production to the
    // Viterbi's sustainable rate (~0.26-0.30 pairs/clk) at 48/54M where the
    // drain tail (~240 clk) exceeds the lead-in (~227 clk) — no overflow,
    // no loss; those rates remain Viterbi-limited and over budget.
    //
    // NOTE: depunct_done is deliberately NOT OR'd in. deint_done(N) always
    // precedes depunct_done(N), and at 48/54M the depunct drain (~240 clk)
    // outlasts the lead-in (~227 clk), so depunct_done(N) lands in
    // S_WAIT_DEMAP_D(N+1). OR-ing it double-triggers the FSM: symbol_start
    // for N+2 fires early, force-resetting pilot_track mid-emit of N+1
    // (truncated demap stream, deinterleaver mispermute, FCS fail —
    // measured 2026-08-22, rates 48/54 only). Advancing on deint_done
    // alone fires exactly once per symbol and is strictly earliest.
    wire       pipeline_sym_done = deint_done;

    // rd_addr mux
    reg [5:0] local_rd_addr;
    always @(*) begin
        if (eq_rd_sel)
            ce_rd_addr = eq_rd_addr;
        else
            ce_rd_addr = local_rd_addr;
    end

    // =========================================================
    // SIGNAL field parity check
    // =========================================================
    wire sig_parity_ok;
    reg  sig_parity_computed;
    always @(*) begin
        sig_parity_computed = ^sig_bits[17:0];
    end
    assign sig_parity_ok = (sig_parity_computed == 1'b0);

    // =========================================================
    // N_SYM calculation: 16-cycle binary long-division
    // =========================================================
    reg [15:0] div_remainder;
    reg [15:0] div_dividend;
    reg [7:0]  div_n_dbps;
    reg [10:0] div_quotient;
    reg [3:0]  div_step;
    reg        div_active;
    reg        div_done;

    // =========================================================
    // Main FSM
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            wr_ptr          <= 0;
            rd_ptr          <= 0;
            fill_wait_cnt   <= 0;
            feed_cnt        <= 0;
            feed_valid      <= 0;
            feed_primed     <= 0;
            data_buf_wait   <= 0;
            fft_rst_n       <= 0;
            fft_draining    <= 0;
            fft_data_cnt    <= 0;
            fft_out_cnt     <= 0;
            fft_phase       <= 0;
            chan_est_start  <= 0;
            eq_start        <= 0;
            eq_rd_sel       <= 0;
            seq_done        <= 0;
            snap_trig       <= 0;
            local_rd_addr   <= 0;
            vit_frame_start <= 0;
            vit_flush       <= 0;
            vit_streaming_mode <= 0;
            depunct_restart <= 0;
            sig_bits        <= 0;
            sig_bit_cnt     <= 0;
            sig_capture_en  <= 0;
            signal_valid    <= 0;
            parsed_rate     <= 0;
            parsed_length   <= 0;
            tag_valid       <= 0;
            tag_abort       <= 0;
            tag_rate        <= 0;
            tag_length      <= 0;
            tag_fcs_ok      <= 0;
            rate_mode_out   <= 0;
            code_rate_out   <= 0;
            norm_out        <= 64;
            psdu_len_out    <= 0;
            n_sym           <= 0;
            sym_cnt         <= 0;
            fcs_frame_start <= 0;
            fcs_result_ok   <= 0;
            fcs_result_fail <= 0;
            div_active      <= 0;
            div_done        <= 0;
            div_quotient    <= 0;
            div_remainder   <= 0;
            div_dividend    <= 0;
            div_n_dbps      <= 0;
            div_step        <= 0;
            symbol_idx_out  <= 0;
            symbol_start_out <= 0;
            is_signal_out   <= 0;
            ltf1_offset     <= 0;
            frame_phase_inc <= 0;
            mixer_phase_reset <= 0;
            ltf_peak_ack    <= 0;
            fifo_pop        <= 0;
            diag_accepted_cnt <= 0;
            diag_abort_cnt  <= 0;
            diag_heartbeat  <= 0;
            abort_sig_cnt   <= 0;
            abort_rate_cnt  <= 0;
            abort_ow_cnt    <= 0;
            abort_wd_cnt    <= 0;
            abort_sig_snap  <= 0;
            abort_ctx_snap  <= 0;
            tag_sig_snap    <= 0;
            tag_ctx_snap    <= 0;
            fcs_timeout_cnt <= 0;
            watchdog_cnt    <= 0;
        end else begin
            // Default: deassert pulses
            diag_heartbeat <= diag_heartbeat + 1;
            chan_est_start  <= 0;
            eq_start        <= 0;
            seq_done        <= 0;
            feed_valid      <= 0;
            snap_trig       <= 0;
            vit_frame_start <= 0;
            vit_flush       <= 0;
            depunct_restart <= 0;
            signal_valid    <= 0;
            tag_valid       <= 0;
            tag_abort       <= 0;
            fcs_frame_start <= 0;
            div_done        <= 0;
            symbol_start_out <= 0;
            ltf_peak_ack    <= 0;
            fifo_pop        <= 0;
            mixer_phase_reset <= 0;

            // Binary long-division step (runs while div_active, 16 cycles)
            if (div_active) begin
                if (div_step == 4'd15) begin
                    if ({div_remainder[14:0], div_dividend[15]} >= {8'b0, div_n_dbps}) begin
                        div_quotient <= {div_quotient[9:0], 1'b1};
                    end else begin
                        div_quotient <= {div_quotient[9:0], 1'b0};
                    end
                    div_active <= 0;
                    div_done   <= 1;
                end else begin
                    if ({div_remainder[14:0], div_dividend[15]} >= {8'b0, div_n_dbps}) begin
                        div_remainder <= {div_remainder[14:0], div_dividend[15]} - {8'b0, div_n_dbps};
                        div_quotient  <= {div_quotient[9:0], 1'b1};
                    end else begin
                        div_remainder <= {div_remainder[14:0], div_dividend[15]};
                        div_quotient  <= {div_quotient[9:0], 1'b0};
                    end
                    div_dividend <= {div_dividend[14:0], 1'b0};
                    div_step     <= div_step + 1;
                end
            end

            // Capture SIGNAL bits from Viterbi output
            if (sig_capture_en && vit_valid) begin
                sig_bits[sig_bit_cnt] <= vit_bit;
                sig_bit_cnt <= sig_bit_cnt + 1;
            end

            // Monitor FCS result
            if (fcs_valid_in) begin
                fcs_result_ok <= 1;
            end
            if (fcs_fail_in) begin
                fcs_result_fail <= 1;
            end

            // Global watchdog: abort to IDLE if stuck > ~10ms
            if (state != S_IDLE) begin
                if (watchdog_cnt[19]) begin
                    state          <= S_IDLE;
                    tag_abort      <= 1;
                    diag_abort_cnt <= diag_abort_cnt + 1;
                    abort_wd_cnt   <= abort_wd_cnt + 1;
                    eq_rd_sel          <= 0;
                    vit_streaming_mode <= 0;
                    is_signal_out      <= 0;
                    eq_start           <= 0;
                    fft_rst_n          <= 0;
                    fft_draining       <= 0;
                end else begin
                    watchdog_cnt <= watchdog_cnt + 1;
                end
            end

            // Circular buffer producer: write continuously when valid.
            // Must never stop writing — acquisition_ctrl uses wr_ptr for
            // position reporting, and pending frames need their IQ in buffer.
            if (write_ok) begin
                circ_buf[wr_ptr[14:0]] <= {iq_re_in, iq_im_in};
                wr_ptr <= wr_ptr + 1;
            end

            // Dismiss stale peak detector peaks (diagnostic)
            if (ltf_peak_found_in && state != S_IDLE) begin
                ltf_peak_ack <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    watchdog_cnt <= 0;
                    // Count IQ samples while waiting to pop — fill guard fallback.
                    // After 208 samples, buffer is guaranteed to contain the frame
                    // data regardless of wr_ptr wrapping (16-bit wraps at 65536).
                    if (!fifo_empty && iq_valid_in && fill_wait_cnt < 8'd255) begin
                        fill_wait_cnt <= fill_wait_cnt + 1;
                    end
                    // Drain buffer when idle with nothing pending: advance rd_ptr
                    // to follow wr_ptr so the circular buffer never fills up
                    // between frames. Without this, buf_full freezes wr_ptr,
                    // which deadlocks the fill guard (wr_ptr - pop_ltf_pos < 208
                    // forever) when inter-frame gaps exceed buffer capacity.
                    if (fifo_empty && !fifo_pop) begin
                        rd_ptr <= wr_ptr;
                        fill_wait_cnt <= 0;
                    end
                    if (!fifo_empty && !fifo_pop) begin
                        if (frame_overwritten_r) begin
                            // Retroactive: queued frame's IQ data has been
                            // overwritten while waiting in FIFO. Abort cleanly.
                            fifo_pop <= 1;
                            tag_abort <= 1;
                            diag_abort_cnt <= diag_abort_cnt + 1;
                            abort_ow_cnt   <= abort_ow_cnt + 1;
                            // Stay in S_IDLE — check next queued frame
                        end else if (fill_ready_r || fill_wait_cnt >= 8'd208) begin
                            // Pop frame descriptor from FIFO
                            fifo_pop          <= 1;
                            fill_wait_cnt     <= 0;
                            ltf1_offset       <= pop_ltf_pos;
                            frame_phase_inc   <= pop_phase_inc;
                            mixer_phase_reset <= 1'b1;
                            // Reset decode state for new frame
                            sig_bits          <= 0;
                            sig_bit_cnt       <= 0;
                            sig_capture_en    <= 0;
                            fcs_result_ok     <= 0;
                            fcs_result_fail   <= 0;
                            eq_rd_sel         <= 0;
                            vit_streaming_mode <= 0;
                            is_signal_out     <= 0;
                            rate_mode_out     <= 0;
                            code_rate_out     <= 0;
                            norm_out          <= 64;
                            diag_accepted_cnt <= diag_accepted_cnt + 1;
                            // Start channel estimation — go straight to FFT feed
                            chan_est_start    <= 1;
                            rd_ptr            <= pop_ltf_pos;
                            feed_cnt          <= 0;
                            feed_primed       <= 0;
                            fft_phase         <= 2'd0;
                            fft_data_cnt      <= 0;
                            fft_out_cnt       <= 0;
                            fft_draining      <= 0;
                            fft_rst_n         <= 0;
                            state             <= S_FFT_FEED;
                        end
                    end
                end

                // =========================================================
                // S_FFT_FEED — Unified FFT feed state
                // fft_phase: 0=LTF1, 1=LTF2, 2=SIG, 3=DATA
                // =========================================================
                S_FFT_FEED: begin
                    feed_valid <= 0;

                    // Count FFT output bins
                    if (fft_dout_valid_in && fft_rst_n) begin
                        fft_out_cnt <= fft_out_cnt + 1;
                    end

                    // Count mixer data outputs
                    if (feed_mixer_valid_out && !fft_draining) begin
                        fft_data_cnt <= fft_data_cnt + 1;
                        if (fft_data_cnt == 7'd63) begin
                            fft_draining <= 1'b1;
                        end
                    end

                    // Phase setup (first clock of each symbol — fft_rst_n is 0)
                    if (!fft_rst_n && !feed_primed) begin
                        feed_primed <= 1;
                        case (fft_phase)
                            2'd0: begin  // LTF1
                                rd_ptr <= ltf1_offset;
                            end
                            2'd1: begin  // LTF2
                                 rd_ptr <= ltf1_offset + 16'd64;
                            end
                            2'd2: begin  // SIG
                                 rd_ptr <= ltf1_offset + 16'd144;
                                eq_start        <= 1;
                                eq_rd_sel       <= 1;
                                symbol_start_out <= 1;
                                symbol_idx_out   <= 8'd0;
                                is_signal_out    <= 1;
                                vit_frame_start <= 1;
                                vit_streaming_mode <= 0;
                                // Discard the previous frame's undrained
                                // coded-bit tail and restart the puncture
                                // pattern. Safe here: no bit of this frame
                                // has reached the depuncturer yet.
                                depunct_restart <= 1;
                                sig_capture_en  <= 1;
                                sig_bit_cnt     <= 0;
                            end
                            2'd3: begin  // DATA
                                eq_start        <= 1;
                                eq_rd_sel       <= 1;
                                symbol_start_out <= 1;
                                symbol_idx_out   <= sym_cnt[7:0] + 8'd1;
                                is_signal_out    <= 0;
                            end
                        endcase
                    end else if (!fft_rst_n && feed_primed) begin
                        fft_rst_n <= 1'b1;
                    end else if (feed_cnt < 64) begin
                        feed_valid <= 1;
                        feed_cnt   <= feed_cnt + 1;
                        rd_ptr     <= rd_ptr + 1;
                    end

                    // Symbol done: all 64 output bins received
                    if (fft_dout_valid_in && fft_rst_n && fft_out_cnt == 6'd63) begin
                        case (fft_phase)
                            2'd0: begin
                                fft_phase    <= 2'd1;
                                fft_data_cnt <= 0;
                                fft_out_cnt  <= 0;
                                fft_draining <= 0;
                                fft_rst_n    <= 0;
                                feed_cnt     <= 0;
                                feed_primed  <= 0;
                                feed_valid   <= 0;
                            end
                            2'd1: begin
                                fft_rst_n <= 0;
                                state <= S_WAIT_CE;
                            end
                            2'd2: begin
                                fft_rst_n <= 0;
                                state <= S_EQ_SIG;
                            end
                            2'd3: begin
                                fft_rst_n <= 0;
                                state <= S_EQ_DATA;
                            end
                        endcase
                    end
                end

                S_WAIT_CE: begin
                    if (chan_est_done) begin
                        fft_phase    <= 2'd2;
                        fft_data_cnt <= 0;
                        fft_out_cnt  <= 0;
                        fft_draining <= 0;
                        fft_rst_n    <= 0;
                        feed_cnt     <= 0;
                        feed_primed  <= 0;
                        state        <= S_FFT_FEED;
                    end
                end

                // ---- SIGNAL symbol processing ----
                S_EQ_SIG: begin
                    if (eq_done) begin
                        eq_rd_sel <= 0;
                        state     <= S_WAIT_DEMAP_SIG;
                    end
                end

                S_WAIT_DEMAP_SIG: begin
                    if (pipeline_sym_done) begin
                        vit_flush <= 1;
                        state     <= S_WAIT_VIT_SIG;
                    end
                end

                S_WAIT_VIT_SIG: begin
                    if (sig_bit_cnt >= 24) begin
                        sig_capture_en <= 0;
                        state          <= S_PARSE_SIGNAL;
                    end
                end

                // ---- SIGNAL field parsing ----
                S_PARSE_SIGNAL: begin
                    parsed_rate   <= sig_bits[3:0];
                    parsed_length <= sig_bits[16:5];

                    if (!sig_parity_ok || sig_bits[23:18] != 6'b0 || sig_bits[16:5] < 12'd4) begin
                        signal_valid <= 1;
                        seq_done     <= 1;
                        tag_abort    <= 1;
                        diag_abort_cnt <= diag_abort_cnt + 1;
                        abort_sig_cnt  <= abort_sig_cnt + 1;
                        abort_sig_snap <= {8'b0, sig_bits[23:0]};
                        abort_ctx_snap <= {ltf1_offset, frame_phase_inc};
                        state        <= S_IDLE;
                    end else begin
                        state <= S_CONFIG_DATA;
                    end
                end

                S_CONFIG_DATA: begin
                    if (!tbl_valid) begin
                        signal_valid <= 1;
                        seq_done     <= 1;
                        tag_abort    <= 1;
                        diag_abort_cnt <= diag_abort_cnt + 1;
                        abort_rate_cnt <= abort_rate_cnt + 1;
                        state        <= S_IDLE;
                    end else begin
                        signal_valid  <= 1;
                        rate_mode_out <= tbl_rate_mode;
                        code_rate_out <= tbl_code_rate;
                        norm_out      <= tbl_norm;
                        psdu_len_out  <= parsed_length;

                        div_dividend  <= 16'd21 + ({4'b0, parsed_length} << 3) + {8'b0, tbl_n_dbps};
                        div_remainder <= 16'd0;
                        div_n_dbps    <= tbl_n_dbps;
                        div_quotient  <= 0;
                        div_step      <= 0;
                        div_active    <= 1;

                        sym_cnt       <= 0;
                        rd_ptr        <= ltf1_offset + 16'd224;

                        fcs_frame_start <= 1;

                        vit_frame_start <= 1;
                        vit_streaming_mode <= 1;

                        state <= S_WAIT_DIV;
                    end
                end

                S_WAIT_DIV: begin
                    if (div_done) begin
                        n_sym <= div_quotient;
                    end
                    if ((div_done || !div_active) && num_avail >= 16'd64) begin
                        fft_phase    <= 2'd3;
                        fft_data_cnt <= 0;
                        fft_out_cnt  <= 0;
                        fft_draining <= 0;
                        fft_rst_n    <= 0;
                        feed_cnt     <= 0;
                        feed_primed  <= 0;
                        data_buf_wait <= 0;
                        state        <= S_FFT_FEED;
                    end
                end

                // ---- DATA symbol processing ----
                S_EQ_DATA: begin
                    if (eq_done) begin
                        eq_rd_sel <= 0;
                        state     <= S_WAIT_DEMAP_D;
                    end
                end

                S_WAIT_DEMAP_D: begin
                    if (data_buf_wait) begin
                        if (num_avail >= 16'd64) begin
                            data_buf_wait <= 0;
                            fft_phase    <= 2'd3;
                            fft_data_cnt <= 0;
                            fft_out_cnt  <= 0;
                            fft_draining <= 0;
                            fft_rst_n    <= 0;
                            feed_cnt     <= 0;
                            feed_primed  <= 0;
                            state        <= S_FFT_FEED;
                        end
                    end else if (pipeline_sym_done) begin
                        sym_cnt <= sym_cnt + 1;
                        rd_ptr  <= rd_ptr + 16'd16;
                        if (sym_cnt + 1 >= n_sym) begin
                            vit_flush <= 1;
                            fcs_timeout_cnt <= 0;
                            state     <= S_WAIT_FCS;
                        end else begin
                            if (num_avail >= 16'd64 + 16'd16) begin
                                fft_phase    <= 2'd3;
                                fft_data_cnt <= 0;
                                fft_out_cnt  <= 0;
                                fft_draining <= 0;
                                fft_rst_n    <= 0;
                                feed_cnt     <= 0;
                                feed_primed  <= 0;
                                state        <= S_FFT_FEED;
                            end else begin
                                data_buf_wait <= 1;
                            end
                        end
                    end
                end

                S_WAIT_FCS: begin
                    if (fcs_result_ok || fcs_result_fail) begin
                        state <= S_TAG_OUT;
                    end else if (fcs_timeout_cnt[19]) begin
                        fcs_result_fail <= 1;
                        state <= S_TAG_OUT;
                    end else begin
                        fcs_timeout_cnt <= fcs_timeout_cnt + 1;
                    end
                end

                S_TAG_OUT: begin
                    tag_valid  <= 1;
                    tag_rate   <= parsed_rate;
                    tag_length <= parsed_length;
                    tag_fcs_ok <= fcs_result_ok;
                    snap_trig  <= 1;
                    // Good-frame snapshot for the hardware A/B (good vs abort).
                    // sig_bits/frame_phase_inc still hold this frame's values.
                    tag_sig_snap <= {8'b0, sig_bits[23:0]};
                    tag_ctx_snap <= {ltf1_offset, frame_phase_inc};
                    state      <= S_DONE;
                end

                S_DONE: begin
                    seq_done   <= 1;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================
    // Simulation assertions (not synthesized)
    // =========================================================
    `ifdef SIM
    always @(posedge clk) begin
        if (state > S_DONE)
            $error("[decode_engine] ILLEGAL STATE: %0d", state);
        // fft_out_cnt is [5:0] — cannot exceed 63; assertion removed (CMPCONST)
        if (sym_cnt > n_sym && state != S_IDLE && n_sym != 0)
            $error("[decode_engine] sym_cnt (%0d) > n_sym (%0d)", sym_cnt, n_sym);
    end
    `endif

endmodule
