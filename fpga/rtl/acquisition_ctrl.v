// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/acquisition_ctrl.v — Standalone frame acquisition engine
//
// Watches the LTF correlator metric after each frame_detect and finds the
// T1 start position using an stf_end-anchored windowed-max search. Pushes
// frame descriptors {ltf_pos, phase_inc} to a FIFO for the decode engine.
//
// This module runs independently of the decode pipeline. It is never blocked
// by decode state, so back-to-back frames at SIFS timing are acquired without
// the jitter sensitivity of the old inline pending path.
//
// Peak-finding strategy (option B, docs/acquisition-window-fix.md):
// stf_end jitters relative to the true LTF1 metric peak by as much as -17
// samples, and a channel-induced +34-sample lobe can be stronger than the
// true peak. A forward-only window anchored at stf_end therefore misses the
// true peak and selects the lobe. Instead, keep a trailing argmax over the
// ~2*BLOCK_LEN samples preceding stf_end via a two-register ping-pong
// (cur = running argmax of the current block, prev = final argmax of the
// previous block), seed the search with max(cur, prev) at stf_end, then keep
// sweeping forward to stf_end + SEARCH_POST. The effective window floats
// between [stf_end - 2*BLOCK_LEN, stf_end + SEARCH_POST] with block phase;
// this covers the measured peak range while stopping short of the +34 lobe.
// The max position marks the T1 rising edge (strict > keeps the first
// occurrence, so the rising edge wins over the plateau).
//
// Rearm: signals pipeline_ack to stf_detect as soon as a descriptor is pushed
// (or a frame is rejected). This allows stf_detect to rearm within ~40 samples
// of stf_end, not thousands of samples later when decode finishes.

module acquisition_ctrl (
    input  wire        clk,
    input  wire        rst_n,

    // Trigger input (from stf_detect)
    input  wire        frame_detect,

    // STF end (from stf_detect — marks STF/GI2 boundary)
    input  wire        stf_end,

    // Correlator metric (from ltf_correlator, continuous 1-per-sample)
    input  wire        corr_metric_valid,
    input  wire [28:0] corr_metric,

    // Buffer write pointer (from decode_engine, for T1 position)
    input  wire [15:0] wr_ptr,

    // CFO estimate (from cfo_est)
    input  wire        cfo_done,
    input  wire [15:0] phase_inc,

    // Frame descriptor output (to frame_fifo)
    output reg         desc_valid,       // pulse: descriptor ready
    output reg  [15:0] desc_ltf_pos,     // T1 start position in circular buffer
    output reg  [15:0] desc_phase_inc,   // CFO correction for this frame

    // Backpressure: FIFO full (triggers rejected)
    input  wire        fifo_full,

    // Pipeline acknowledgment (to stf_detect — enables rearm)
    output reg         pipeline_ack,

    // Estimator arming (to cfo_est.start): one pulse per ACCEPTED trigger.
    // Ties the CFO estimate to the same single-outstanding acquisition
    // decision that produces the descriptor, so raw frame_detect re-triggers
    // cannot start/displace an estimate for a frame that was not acquired.
    output reg         cfo_start,

    // Diagnostic
    output reg  [7:0]  diag_frames_found,   // descriptors pushed
    output reg  [7:0]  diag_frames_rejected // rejected (fifo full or metric fail)
);

    // =========================================================
    // Parameters
    // =========================================================

    // Minimum samples between frame_detect triggers to suppress STF tail
    // re-detection. 256 samples = 12.8 μs — safely past the STF-to-data
    // transition, well within SIFS (320 samples = 16 μs).
    localparam [8:0] MIN_TRIGGER_DISTANCE = 9'd256;

    // Trailing argmax block length (N) and forward search reach (B).
    // BLOCK_LEN=16 makes the effective lower edge float in
    // [stf_end-32, stf_end-16], inside the measured safe backward region
    // A in [-35,-7]. SEARCH_POST=20 reaches the GI2-extended peak while
    // stopping short of the +34 lobe (nearest at stf_end+25 for the worst
    // measured stf_end jitter). See docs/acquisition-window-fix.md §5.
    localparam [4:0] BLOCK_LEN    = 5'd16;
    localparam [5:0] SEARCH_POST  = 6'd20;

    // Stored metric width for the trailing argmax. The 29-bit squared
    // magnitude only needs to be *ordered*, not measured, so keep the top
    // 14 bits (the low 15 are noise as far as peak selection goes). This
    // saves 30 FFs in the two history registers and was required to close
    // placement (the untruncated form missed by 6 slices). The forward
    // compare and the metric-floor test still use the full-width metric.
    localparam [4:0] HIST_W     = 5'd14;
    localparam [4:0] HIST_SHIFT = 5'd15;   // 29 - 14

    // Minimum metric to accept a peak (rejects noise-only windows).
    // Squared-magnitude metric (|acc|>>11)^2: 16384 = (262144>>11)^2 preserves
    // the exact accept/reject boundary of the old L1 metric floor 262144.
    localparam [28:0] METRIC_FLOOR = 29'h004000;  // 16384

    // T1 offset correction: peak_pos - 19 = T1 start
    // (16 taps + 1 wr_ptr post-increment + 2 correlator pipeline delay.
    // The metric's registered square pipeline shifts the pulse within the
    // 5-clock sample period (onto the next iq_valid clock); wr_ptr updates
    // only on sample boundaries, so the captured position is unchanged.)
    localparam [4:0] T1_OFFSET = 5'd19;

    // S_WAIT_STF_END timeout: if stf_end never arrives (watchdog/playback
    // clear resets stf_detect's stf_end_armed mid-window), give up and
    // return to IDLE so the next frame's trigger can be accepted.
    // 1024 clocks = ~205 samples: far beyond legitimate stf_end latency
    // (~80-120 samples after trigger), well under SIFS (320 samples), so
    // at most one frame is lost per wedge.
    localparam [10:0] STF_END_TIMEOUT = 11'd1024;

    // =========================================================
    // State Machine
    // =========================================================
    localparam [1:0] S_IDLE         = 2'd0;
    localparam [1:0] S_WAIT_STF_END = 2'd1;
    localparam [1:0] S_SEARCH       = 2'd2;

    reg [1:0] state;

    // =========================================================
    // Internal Registers
    // =========================================================
    reg [8:0]  trigger_distance;   // samples since last frame_detect (saturates)
    reg        cfo_latched;        // first-wins CFO latch
    reg [15:0] latched_phase_inc;  // stored CFO estimate

    // Peak search state
    reg [5:0]  search_cnt;         // forward samples since stf_end
    reg [28:0] max_metric;
    reg [15:0] max_pos;

    // Trailing argmax ping-pong (active from trigger accept until stf_end):
    //   cur  = running argmax of the block currently accumulating
    //   prev = final argmax of the previous completed block
    // max(cur, prev) covers the trailing [BLOCK_LEN, 2*BLOCK_LEN] samples.
    reg [4:0]        blk_cnt;      // sample index within current block
    reg [HIST_W-1:0] cur_metric;   // top HIST_W bits of the metric
    reg [15:0]       cur_pos;
    reg [HIST_W-1:0] prev_metric;
    reg [15:0]       prev_pos;

    // S_WAIT_STF_END timeout counter
    reg [10:0] stf_end_cnt;

    // =========================================================
    // Trigger Distance Counter
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            trigger_distance <= 9'd511;  // saturated at reset (allow first trigger)
        end else if (frame_detect && state == S_IDLE) begin
            trigger_distance <= 0;
        end else if (corr_metric_valid && trigger_distance < 9'd511) begin
            trigger_distance <= trigger_distance + 1;
        end
    end

    // =========================================================
    // Trailing argmax ping-pong
    // =========================================================
    // Restarted on each accepted trigger (so pre-frame metrics, e.g. a prior
    // frame's plateau, can never leak into the seed) and ticked once per
    // correlator metric while waiting for stf_end.
    always @(posedge clk) begin
        if (!rst_n) begin
            blk_cnt     <= 5'd0;
            cur_metric  <= {HIST_W{1'b0}};
            cur_pos     <= 16'd0;
            prev_metric <= {HIST_W{1'b0}};
            prev_pos    <= 16'd0;
        end else if (frame_detect && state == S_IDLE &&
                     !fifo_full && trigger_distance >= MIN_TRIGGER_DISTANCE) begin
            blk_cnt     <= 5'd0;
            cur_metric  <= {HIST_W{1'b0}};
            cur_pos     <= 16'd0;
            prev_metric <= {HIST_W{1'b0}};
            prev_pos    <= 16'd0;
        end else if (state == S_WAIT_STF_END && corr_metric_valid) begin
            if (blk_cnt == BLOCK_LEN - 1) begin
                // Block complete: promote the winning sample to prev and
                // start a fresh block.
                if (corr_metric[28:HIST_SHIFT] > cur_metric) begin
                    prev_metric <= corr_metric[28:HIST_SHIFT];
                    prev_pos    <= wr_ptr;
                end else begin
                    prev_metric <= cur_metric;
                    prev_pos    <= cur_pos;
                end
                blk_cnt    <= 5'd0;
                cur_metric <= {HIST_W{1'b0}};
                cur_pos    <= 16'd0;
            end else begin
                blk_cnt <= blk_cnt + 1'b1;
                if (corr_metric[28:HIST_SHIFT] > cur_metric) begin
                    cur_metric <= corr_metric[28:HIST_SHIFT];
                    cur_pos    <= wr_ptr;
                end
            end
        end
    end

    // =========================================================
    // Main FSM
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            desc_valid         <= 0;
            pipeline_ack       <= 0;
            cfo_start          <= 0;
            cfo_latched        <= 0;
            latched_phase_inc  <= 0;
            search_cnt         <= 0;
            max_metric         <= 0;
            max_pos            <= 0;
            stf_end_cnt        <= 0;
            diag_frames_found  <= 0;
            diag_frames_rejected <= 0;
        end else begin
            // Default: deassert pulses
            desc_valid   <= 0;
            pipeline_ack <= 0;
            cfo_start    <= 0;

            case (state)
                S_IDLE: begin
                    if (frame_detect && !fifo_full &&
                        trigger_distance >= MIN_TRIGGER_DISTANCE) begin
                        // Accept trigger — begin acquisition
                        state         <= S_WAIT_STF_END;
                        cfo_latched   <= 0;
                        // Arm the CFO estimator for exactly this acquisition.
                        cfo_start     <= 1;
                        // Clear the stored estimate too: if no fresh cfo_done
                        // arrives before this frame's descriptor is pushed, it
                        // must default to "no correction" (0), never inherit
                        // the previous frame's CFO (OTA EAPOL M2/M4 loss).
                        latched_phase_inc <= 0;
                        search_cnt    <= 0;
                        max_metric    <= 0;
                        max_pos       <= 0;
                        stf_end_cnt   <= 0;
                    end else if (frame_detect) begin
                        // Reject: FIFO full or too close to previous trigger
                        diag_frames_rejected <= diag_frames_rejected + 1;
                        pipeline_ack <= 1;  // rearm stf_detect anyway
                    end
                end

                S_WAIT_STF_END: begin
                    // Latch first CFO estimate (first-wins)
                    if (cfo_done && !cfo_latched) begin
                        // Dead-zone filter: ±1 LSB is CORDIC noise (~305 Hz)
                        latched_phase_inc <= (phase_inc == 16'h0001 || phase_inc == 16'hFFFF)
                                              ? 16'd0 : phase_inc;
                        cfo_latched <= 1;
                    end

                    if (stf_end) begin
                        // Seed the forward search with the trailing argmax so
                        // a peak that precedes stf_end can still win. Prefer
                        // the more recent block on ties.
                        state      <= S_SEARCH;
                        search_cnt <= 0;
                        if (prev_metric > cur_metric) begin
                            max_metric <= {prev_metric, {HIST_SHIFT{1'b0}}};
                            max_pos    <= prev_pos;
                        end else begin
                            max_metric <= {cur_metric, {HIST_SHIFT{1'b0}}};
                            max_pos    <= cur_pos;
                        end
                    end else if (stf_end_cnt >= STF_END_TIMEOUT) begin
                        // stf_end lost (watchdog/playback clear killed
                        // stf_end_armed) — give up, rearm, next trigger
                        // can be accepted from S_IDLE.
                        diag_frames_rejected <= diag_frames_rejected + 1;
                        pipeline_ack <= 1;
                        state        <= S_IDLE;
                    end else begin
                        stf_end_cnt <= stf_end_cnt + 1;
                    end
                end

                S_SEARCH: begin
                    // Latch CFO if it arrives during search (late estimate)
                    if (cfo_done && !cfo_latched) begin
                        latched_phase_inc <= (phase_inc == 16'h0001 || phase_inc == 16'hFFFF)
                                              ? 16'd0 : phase_inc;
                        cfo_latched <= 1;
                    end

                    if (corr_metric_valid) begin
                        // Window already seeded at stf_end from the trailing
                        // argmax. Sweep forward to stf_end + SEARCH_POST.
                        if (search_cnt >= SEARCH_POST) begin
                            // Window complete — evaluate
                            if (max_metric >= METRIC_FLOOR) begin
                                // Valid peak found — push descriptor
                                desc_valid     <= 1;
                                desc_ltf_pos   <= max_pos - {11'd0, T1_OFFSET};
                                desc_phase_inc <= latched_phase_inc;
                                diag_frames_found <= diag_frames_found + 1;
                            end else begin
                                // Metric too low — noise, not a real frame
                                diag_frames_rejected <= diag_frames_rejected + 1;
                            end
                            pipeline_ack <= 1;  // rearm stf_detect
                            state        <= S_IDLE;
                        end else begin
                            search_cnt <= search_cnt + 1;
                            if (corr_metric > max_metric) begin
                                max_metric <= corr_metric;
                                max_pos    <= wr_ptr;
                            end
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
