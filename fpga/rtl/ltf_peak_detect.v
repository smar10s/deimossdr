// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/ltf_peak_detect.v — Peak detector for sliding LTF correlator
//
// Watches the correlation metric stream from ltf_correlator. Detects when
// the metric rises above threshold, tracks the peak, and latches the peak
// position when the metric drops back down.
//
// Behavior:
// - IDLE: wait for metric > THRESHOLD
// - SEARCH: track running max. On metric < max/2 (or < THRESHOLD), latch peak.
// - FOUND: peak_found asserted. Stays until ack. If new peak arrives (last-wins),
//   overwrite peak position.
//
// The "last-wins" behavior handles multi-path scenarios where several
// correlation peaks appear in sequence — we always take the strongest.

module ltf_peak_detect #(
    // Squared-magnitude metric: 250000 ~= 0.24x of the nominal clean peak
    // ((|acc|>>11)^2 at |acc|~2M), same ratio as the old L1 threshold 500000.
    parameter THRESHOLD = 29'd250000   // metric must exceed this to start search
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,          // tie to 1 when correlator should watch
    input  wire        metric_valid,    // from ltf_correlator
    input  wire [28:0] metric,          // from ltf_correlator
    input  wire [15:0] sample_pos,      // current sample position (e.g., wr_ptr)
    input  wire        ack,             // pulse: consumer acknowledged peak
    output reg         peak_found,      // latched: a peak was detected
    output reg  [15:0] peak_sample_pos, // sample_pos at the metric peak
    output reg  [28:0] peak_metric      // metric value at peak
);

    // FSM states
    localparam S_IDLE   = 2'd0;
    localparam S_SEARCH = 2'd1;
    localparam S_FOUND  = 2'd2;

    reg [1:0] state;

    // Running max tracker during search
    reg [28:0] run_max;
    reg [15:0] run_max_pos;

    // S_SEARCH watchdog: a search that never completes (metric_valid stops
    // dead, or the metric plateaus above threshold without dropping below
    // max/2) returns to S_IDLE so a later frame's peak can be tracked.
    // 4096 clocks is >2x the longest legitimate LTF correlation plateau
    // (~2k clocks at 20 MSPS). Counts every clock while in S_SEARCH,
    // independent of metric_valid.
    localparam [11:0] SEARCH_TIMEOUT = 12'd4095;
    reg [11:0] search_cnt;

    // Threshold for "metric has fallen" — use half of running max
    wire [28:0] drop_threshold = run_max >> 1;

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            peak_found     <= 1'b0;
            peak_sample_pos <= 16'd0;
            peak_metric    <= 29'd0;
            run_max        <= 29'd0;
            run_max_pos    <= 16'd0;
            search_cnt     <= 0;
        end else begin
            // Ack clears peak_found (can happen in any state)
            if (ack)
                peak_found <= 1'b0;

            if (enable && metric_valid) begin
                case (state)
                    S_IDLE: begin
                        if (metric > THRESHOLD) begin
                            // Rising above threshold — start search
                            state       <= S_SEARCH;
                            run_max     <= metric;
                            run_max_pos <= sample_pos;
                            search_cnt  <= 0;
                        end
                    end

                    S_SEARCH: begin
                        if (metric > run_max) begin
                            // New maximum — update tracker
                            run_max     <= metric;
                            run_max_pos <= sample_pos;
                        end else if (metric < drop_threshold || metric < THRESHOLD) begin
                            // Metric dropped — peak is behind us. Latch it.
                            peak_found     <= 1'b1;
                            peak_sample_pos <= run_max_pos;
                            peak_metric    <= run_max;
                            state          <= S_FOUND;
                        end
                        // else: metric between threshold and max — still searching
                    end

                    S_FOUND: begin
                        // Last-wins: if a new peak appears while we haven't been ack'd
                        if (metric > THRESHOLD) begin
                            // New peak starting — overwrite
                            state       <= S_SEARCH;
                            run_max     <= metric;
                            run_max_pos <= sample_pos;
                            search_cnt  <= 0;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end

            // Search watchdog — placed after the case so the timeout wins
            // any same-cycle state assignment.
            if (state == S_SEARCH) begin
                if (search_cnt == SEARCH_TIMEOUT) begin
                    state      <= S_IDLE;
                    search_cnt <= 0;
                end else begin
                    search_cnt <= search_cnt + 1;
                end
            end
        end
    end

endmodule
