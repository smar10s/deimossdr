// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/pilot_track.v — Pilot phase-locked loop for 802.11a OFDM
//
// PLL-based pilot tracking replaces small-angle approximation.
// Architecture (per symbol):
//   1. Collect 4 pilots + 48 data subcarriers from equalizer
//   2. Compute average pilot phase via cordic_atan2 (after polarity correction)
//   3. Update PLL accumulator: phase_acc += (phase_error >>> ALPHA_SHIFT)
//   4. Rotate all 48 data subcarriers by -phase_acc via cordic_rotate
//   5. Emit corrected data to demapper
//
// Total timing: collect ~52 clk, atan2 ~18 clk, emit 48+14 clk ≈ 132 clk/symbol
// Well within 400-cycle symbol budget.
//
// No dead-zones. The PLL naturally low-passes noise because:
//   - Phase error is averaged across 4 pilots
//   - Fractional accumulation (alpha < 1) attenuates single-sample noise
//   - Random noise self-corrects over multiple symbols
//
// Resources: ~200-400 LUTs (cordic_atan2 shared, cordic_rotate instance),
//            0 DSP48E1, 1 BRAM18K (data buffer)

module pilot_track (
    input  wire        clk,
    input  wire        rst_n,

    // Symbol index (from decode_engine: SIGNAL=0, DATA1=1, DATA2=2, ...)
    input  wire [7:0]  symbol_idx,
    input  wire        is_signal,       // true only during actual SIGNAL symbol
    input  wire        symbol_start,    // pulse at start of new symbol processing

    // Pilot input (from equalizer, 4 per symbol)
    input  wire        pilot_valid,
    input  wire signed [15:0] pilot_re,
    input  wire signed [15:0] pilot_im,
    input  wire [1:0]  pilot_idx,       // 0=sc+7, 1=sc+21, 2=sc-21, 3=sc-7

    // Data input (from equalizer, 48 per symbol)
    input  wire        data_valid_in,
    input  wire signed [15:0] data_re_in,
    input  wire signed [15:0] data_im_in,
    input  wire [5:0]  data_idx_in,     // 0-47

    // Corrected data output (to demapper)
    output reg         data_valid_out,
    output reg  signed [15:0] data_re_out,
    output reg  signed [15:0] data_im_out,

    // Status
    output reg         symbol_done,     // pulse after all 48 corrected data emitted

    // Diagnostic: PLL phase accumulator (for snap observation)
    output wire signed [15:0] diag_phase_acc,
    // Diagnostic: atan2 input (last pilot average Re/Im)
    output wire [15:0] diag_atan2_x,
    output wire [15:0] diag_atan2_y,
    // Diagnostic: FSM state + counters
    output wire [2:0]  diag_state,
    output wire [3:0]  diag_pilot_received,
    output wire [5:0]  diag_data_count
);

    // =========================================================
    // PLL parameter: accumulator gain
    // =========================================================
    // ALPHA_SHIFT=1 → alpha=0.5 (aggressive tracking for short frames)
    // phase_acc += phase_error >>> 1
    localparam ALPHA_SHIFT = 1;

    // =========================================================
    // Pilot polarity sequence (IEEE 802.11a Table 17-6)
    // =========================================================
    reg [0:126] PILOT_POLARITY;
    initial begin
        PILOT_POLARITY = {
            1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1,
            1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1,
            1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0,
            1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1,
            1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1,
            1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1,
            1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1,
            1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1,
            1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1,
            1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0,
            1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1,
            1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1,
            1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1,
            1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0,
            1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1,
            1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0
        };
    end

    // Pilot base signs: {+1, -1, +1, +1} for pilots at {sc+7, sc+21, sc-21, sc-7}
    wire [3:0] PILOT_BASE = 4'b1101;  // [0]=1(+1), [1]=0(-1), [2]=1(+1), [3]=1(+1)

    // =========================================================
    // Polarity lookup for current symbol
    // Sequence is 127 entries (indices 0-126). Maintained as a registered
    // counter that wraps at 127. Does NOT derive from symbol_idx (which is
    // only 8-bit and wraps at 256). Instead, counts DATA symbol_start pulses.
    // NOTE: symbol_idx is still passed for the polarity test but is NOT used
    // for polarity computation. The counter is authoritative.
    // =========================================================
    reg [6:0] pol_idx;

    always @(posedge clk) begin
        if (!rst_n) begin
            pol_idx <= 7'd0;
        end else if (symbol_start) begin
            if (is_signal) begin
                // SIGNAL symbol: reset counter. First DATA will advance to 1.
                pol_idx <= 7'd0;
            end else begin
                // DATA symbol: advance polarity index, wrap at 127
                pol_idx <= (pol_idx == 7'd126) ? 7'd0 : (pol_idx + 7'd1);
            end
        end
    end

    wire polarity_sign = PILOT_POLARITY[pol_idx];

    // Expected pilot sign: product of base and polarity (XNOR)
    wire [3:0] expected_positive;
    assign expected_positive[0] = ~(PILOT_BASE[0] ^ polarity_sign);
    assign expected_positive[1] = ~(PILOT_BASE[1] ^ polarity_sign);
    assign expected_positive[2] = ~(PILOT_BASE[2] ^ polarity_sign);
    assign expected_positive[3] = ~(PILOT_BASE[3] ^ polarity_sign);

    // =========================================================
    // Data buffer (48 × 32 bits in BRAM)
    // BRAM: eliminates 48:1 combinational MUX (~300-500 LUTs saved).
    // Sequential emit pattern is a clean BRAM fit. 1-cycle read latency
    // handled by read_addr leading emit_idx by 1 cycle.
    // =========================================================
    (* ram_style = "block" *) reg [31:0] data_buf [0:47];
    reg [31:0] data_buf_rd;  // registered BRAM output
    reg [5:0] data_count;

    // BRAM read address: leads emit_idx by 1 so data is ready when consumed.
    // Set to 0 at symbol_start, advanced to emit_idx+2 in S_EMIT.
    // The BRAM reads data_buf[buf_rd_addr] each clock, result in data_buf_rd
    // one clock later. By the time S_EMIT reads data_buf_rd, it contains
    // data_buf[emit_idx] (the correct entry).
    reg [5:0] buf_rd_addr;

    // BRAM registered read (1-cycle latency)
    always @(posedge clk) begin
        data_buf_rd <= data_buf[buf_rd_addr];
    end

    wire signed [15:0] buf_rd_re = $signed(data_buf_rd[31:16]);
    wire signed [15:0] buf_rd_im = $signed(data_buf_rd[15:0]);

    always @(posedge clk) begin
        if (!rst_n) begin
            data_count <= 0;
        end else if (symbol_start) begin
            data_count <= 0;
        end else if (data_valid_in && (state == S_COLLECT || state == S_BYPASS)) begin
            data_buf[data_idx_in] <= {data_re_in, data_im_in};
            data_count <= data_count + 1;
        end
    end

    // =========================================================
    // Pilot capture with sign correction
    // =========================================================
    reg [3:0] pilot_received;
    reg signed [15:0] pilot_corr_re [0:3];
    reg signed [15:0] pilot_corr_im [0:3];

    always @(posedge clk) begin
        if (!rst_n) begin
            pilot_received <= 4'b0000;
        end else if (symbol_start) begin
            pilot_received <= 4'b0000;
        end else if (pilot_valid) begin
            pilot_received[pilot_idx] <= 1'b1;
            if (expected_positive[pilot_idx]) begin
                pilot_corr_re[pilot_idx] <= pilot_re;
                pilot_corr_im[pilot_idx] <= pilot_im;
            end else begin
                pilot_corr_re[pilot_idx] <= -pilot_re;
                pilot_corr_im[pilot_idx] <= -pilot_im;
            end
        end
    end

    // =========================================================
    // PLL Phase Accumulator
    // =========================================================
    reg signed [15:0] phase_acc;
    assign diag_phase_acc = phase_acc;

    // =========================================================
    // cordic_atan2_sm instance (compact phase extraction from average pilot)
    // =========================================================
    reg        atan2_valid_in;
    reg signed [15:0] atan2_x_in;
    reg signed [15:0] atan2_y_in;
    wire       atan2_valid_out;
    wire signed [15:0] atan2_angle_out;

    assign diag_atan2_x = atan2_x_in;
    assign diag_atan2_y = atan2_y_in;

    cordic_atan2_sm u_atan2 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (atan2_valid_in),
        .x_in      (atan2_x_in),
        .y_in      (atan2_y_in),
        .valid_out (atan2_valid_out),
        .angle_out (atan2_angle_out)
    );

    // =========================================================
    // cordic_rotate instance (data rotation by -phase_acc)
    // =========================================================
    reg        rotate_valid_in;
    reg signed [15:0] rotate_x_in;
    reg signed [15:0] rotate_y_in;
    reg signed [15:0] rotate_angle;
    wire       rotate_valid_out;
    wire signed [15:0] rotate_x_out;
    wire signed [15:0] rotate_y_out;

    cordic_rotate u_rotate (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (rotate_valid_in),
        .x_in      (rotate_x_in),
        .y_in      (rotate_y_in),
        .angle     (rotate_angle),
        .valid_out (rotate_valid_out),
        .x_out     (rotate_x_out),
        .y_out     (rotate_y_out)
    );

    // =========================================================
    // Main FSM
    // =========================================================
    localparam S_IDLE    = 3'd0;
    localparam S_COLLECT = 3'd1;  // wait for all pilots + data
    localparam S_ATAN2   = 3'd2;  // compute average pilot phase via atan2
    localparam S_PLL_UPD = 3'd3;  // update phase accumulator
    localparam S_EMIT    = 3'd4;  // feed data through cordic_rotate
    localparam S_DRAIN   = 3'd5;  // drain remaining rotated outputs
    localparam S_DONE    = 3'd6;
    localparam S_BYPASS  = 3'd7;  // SIGNAL symbol passthrough

    (* keep = "true" *) reg [2:0] state;
    reg [5:0] emit_idx;
    reg [5:0] emit_count;

    assign diag_state = state;
    assign diag_pilot_received = pilot_received;
    assign diag_data_count = data_count;

    wire all_collected = (pilot_received == 4'b1111) && (data_count == 6'd48);

    // Average pilot Re and Im (sum >>> 2)
    wire signed [17:0] sum_re = $signed({{2{pilot_corr_re[0][15]}}, pilot_corr_re[0]}) +
                                 $signed({{2{pilot_corr_re[1][15]}}, pilot_corr_re[1]}) +
                                 $signed({{2{pilot_corr_re[2][15]}}, pilot_corr_re[2]}) +
                                 $signed({{2{pilot_corr_re[3][15]}}, pilot_corr_re[3]});
    wire signed [17:0] sum_im = $signed({{2{pilot_corr_im[0][15]}}, pilot_corr_im[0]}) +
                                 $signed({{2{pilot_corr_im[1][15]}}, pilot_corr_im[1]}) +
                                 $signed({{2{pilot_corr_im[2][15]}}, pilot_corr_im[2]}) +
                                 $signed({{2{pilot_corr_im[3][15]}}, pilot_corr_im[3]});

    // =========================================================
    // FSM logic
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            emit_idx <= 0;
            emit_count <= 0;
            buf_rd_addr <= 0;
            phase_acc <= 16'sd0;
            atan2_valid_in <= 0;
            atan2_x_in <= 0;
            atan2_y_in <= 0;
            rotate_valid_in <= 0;
            rotate_x_in <= 0;
            rotate_y_in <= 0;
            rotate_angle <= 0;
            data_valid_out <= 0;
            data_re_out <= 0;
            data_im_out <= 0;
            symbol_done <= 0;
        end else begin
            // Defaults
            data_valid_out <= 0;
            symbol_done <= 0;
            atan2_valid_in <= 0;
            rotate_valid_in <= 0;

            // symbol_start forces FSM reset from ANY state
            if (symbol_start) begin
                if (is_signal) begin
                    state <= S_BYPASS;
                    phase_acc <= 16'sd0;  // reset PLL on new frame
                end else begin
                    state <= S_COLLECT;
                end
                emit_idx <= 0;
                emit_count <= 0;
                buf_rd_addr <= 0;
                data_valid_out <= 0;
            end else case (state)
                S_IDLE: begin
                    // Wait for symbol_start
                end

                S_BYPASS: begin
                    // SIGNAL symbol: pass data directly without correction
                    if (data_valid_in) begin
                        data_valid_out <= 1;
                        data_re_out <= data_re_in;
                        data_im_out <= data_im_in;
                        emit_count <= emit_count + 1;
                        if (emit_count == 6'd47) begin
                            state <= S_DONE;
                        end
                    end
                end

                S_COLLECT: begin
                    if (all_collected) begin
                        // Launch atan2 on average pilot vector
                        atan2_valid_in <= 1;
                        // Average (sum >>> 2) gives 16-bit result for atan2_sm
                        atan2_x_in <= sum_re[17:2];
                        atan2_y_in <= sum_im[17:2];
                        state <= S_ATAN2;
                    end
                end

                S_ATAN2: begin
                    // Wait for atan2 result
                    if (atan2_valid_out) begin
                        state <= S_PLL_UPD;
                    end
                end

                S_PLL_UPD: begin
                    // Update PLL: first-order loop filter
                    // phase_error = measured - current_estimate
                    // phase_acc += alpha * phase_error
                    // This converges phase_acc toward the true phase offset.
                    phase_acc <= phase_acc + ((atan2_angle_out - phase_acc) >>> ALPHA_SHIFT);
                    emit_idx <= 0;
                    // BRAM prefetch: set read address to 1 (next needed index).
                    // The BRAM block will read data_buf[current buf_rd_addr = 0]
                    // at this edge (getting data[0] ready for first S_EMIT).
                    // Then data_buf[1] will be read next edge (for second S_EMIT).
                    buf_rd_addr <= 6'd1;
                    state <= S_EMIT;
                end

                S_EMIT: begin
                    // Feed data subcarriers into cordic_rotate one per clock.
                    // BRAM timing: data_buf_rd holds data for emit_idx (read
                    // was kicked off 1 clock ago via buf_rd_addr = emit_idx + 1
                    // from the previous iteration). buf_rd_addr always leads
                    // by 1 to keep the pipeline filled.
                    if (emit_idx < 6'd48) begin
                        rotate_valid_in <= 1;
                        rotate_x_in <= buf_rd_re;
                        rotate_y_in <= buf_rd_im;
                        rotate_angle <= -phase_acc;  // rotate by -phase_acc
                        emit_idx <= emit_idx + 1;
                        // Advance BRAM read for the iteration AFTER next
                        buf_rd_addr <= emit_idx + 6'd2;
                    end else begin
                        state <= S_DRAIN;
                    end
                end

                S_DRAIN: begin
                    // Wait for all rotated outputs to emerge
                    if (emit_count == 6'd48) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    symbol_done <= 1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // Capture rotated outputs (arrive from cordic_rotate pipeline)
            if (rotate_valid_out) begin
                data_valid_out <= 1;
                data_re_out <= rotate_x_out;
                data_im_out <= rotate_y_out;
                emit_count <= emit_count + 1;
            end
        end
    end

endmodule
