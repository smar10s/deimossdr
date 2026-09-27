// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/chan_est.v — Channel estimator for 802.11a OFDM
//
// Given two LTF FFT outputs (fed sequentially via bin_valid/bin_idx/bin_re/bin_im),
// computes channel estimate H and its inverse H_inv for equalization.
//
// Algorithm:
//   1. Capture LTF1 bins (64) into BRAM
//   2. When LTF2 bins arrive, compute H[k] = (LTF1[k] + LTF2[k]) / 2 * LTF_ref[k]
//   3. Find max(|H[k]|^2) across active bins → determine adaptive shift
//   4. For each active bin, compute H_inv[k] = conj(H[k]) << shift / |H[k]|^2
//   5. Null bins (DC, guards) get H_inv = 0
//
// ADAPTIVE SHIFT:
//   The traditional Q1.15 H_inv (fixed shift=15) loses precision for large signals.
//   Instead, we choose shift_val adaptively so the strongest bin's H_inv fills
//   ~10 bits:
//     shift_val = 10 + (32 - CLZ(max_mag_sq)) / 2, clamped to [15, 31]
//   Deliberately weaker than the old +14: the +14 headroom left only ~6 dB of
//   channel dynamic range before fade-side bins clamp at the +-32767 divider
//   saturation. +10 puts the strongest bin at ~2^10 and extends the
//   clamp-free fade range by +24 dB (a -6 dB tap multipath profile no longer
//   clips any bin). The equalizer's output scale (X * 2^EQ_SCALE) is
//   independent of shift_val, so nominal decode is unchanged.
//   The equalizer uses: eq = Y * H_inv >> shift_val
//
// FSM: IDLE → CAPTURE_LTF1 → WAIT_LTF2 → CAPTURE_LTF2 → FIND_MAX → COMPUTE → DONE
//
// Capture states carry a stall watchdog: a decode_engine abort (watchdog,
// frame overwrite) can stop the bin stream mid-LTF. Without recovery the
// FSM never returns to S_IDLE, every later start pulse is ignored, and the
// equalizer silently runs on stale H_inv/shift_val. Timeout returns the FSM
// to S_IDLE WITHOUT asserting done — done on a garbage H would be worse.
// 1024 clocks: measured legitimate inter-symbol bin gap is 86 clocks
// (FFT reset + re-prime between LTF1 and LTF2).
//
// Division uses iterative restoring algorithm (32 cycles per division).
// Two divisions per active bin (re + im), 52 active bins → ~3500 cycles total.
// Plus 52 cycles for FIND_MAX → total ~3600 cycles. Acceptable.
//
// Resource estimate: ~150-250 LUTs (iterative divider + CLZ), 4 BRAM18K, 0 DSP.

module chan_est (
    input  wire        clk,
    input  wire        rst_n,

    // Control
    input  wire        start,          // pulse: begin channel estimation

    // FFT bin input (from fft64_sdf via decode_engine, sequential)
    input  wire        bin_valid,
    input  wire [5:0]  bin_idx,
    input  wire signed [15:0] bin_re,
    input  wire signed [15:0] bin_im,

    // H_inv output (BRAM read port for equalizer)
    input  wire [5:0]  rd_addr,
    output reg  signed [15:0] h_inv_re,
    output reg  signed [15:0] h_inv_im,

    // Adaptive shift value (used by equalizer for >> shift_val)
    output reg  [4:0]  shift_val,

    // Status
    output reg         done,

    // Diagnostics: count of H_inv writes clamped at the +-32767 divider
    // saturation (saturates at 16'hFFFF). Exposed for sim measurement of
    // deep-fade clipping; hardware observability wiring is a separate task.
    output reg [15:0]  clip_cnt
);

    // =========================================================
    // LTF Reference Sign ROM — NATURAL FFT BIN ORDER
    //
    // FFT bin k = subcarrier k for k=0..31, subcarrier k-64 for k=32..63.
    // Active bins: 1-26 (positive subcarriers +1..+26)
    //              38-63 (negative subcarriers -26..-1)
    // 1 = LTF subcarrier is +1 (keep sign), 0 = -1 or null (negate)
    // =========================================================
    reg ltf_sign_rom [0:63];
    initial begin
        ltf_sign_rom[0]=0;  // DC (null)
        // Bins 1-26: positive subcarriers +1 to +26
        // LTF_L(+1..+26) = +1,-1,-1,+1,+1,-1,+1,-1,+1,-1,-1,-1,-1,-1,+1,+1,-1,-1,+1,-1,+1,-1,+1,+1,+1,+1
        ltf_sign_rom[1]=1;  ltf_sign_rom[2]=0;  ltf_sign_rom[3]=0;  ltf_sign_rom[4]=1;
        ltf_sign_rom[5]=1;  ltf_sign_rom[6]=0;  ltf_sign_rom[7]=1;  ltf_sign_rom[8]=0;
        ltf_sign_rom[9]=1;  ltf_sign_rom[10]=0; ltf_sign_rom[11]=0; ltf_sign_rom[12]=0;
        ltf_sign_rom[13]=0; ltf_sign_rom[14]=0; ltf_sign_rom[15]=1; ltf_sign_rom[16]=1;
        ltf_sign_rom[17]=0; ltf_sign_rom[18]=0; ltf_sign_rom[19]=1; ltf_sign_rom[20]=0;
        ltf_sign_rom[21]=1; ltf_sign_rom[22]=0; ltf_sign_rom[23]=1; ltf_sign_rom[24]=1;
        ltf_sign_rom[25]=1; ltf_sign_rom[26]=1;
        // Bins 27-37: null (guard + DC mirror)
        ltf_sign_rom[27]=0; ltf_sign_rom[28]=0; ltf_sign_rom[29]=0; ltf_sign_rom[30]=0;
        ltf_sign_rom[31]=0; ltf_sign_rom[32]=0; ltf_sign_rom[33]=0; ltf_sign_rom[34]=0;
        ltf_sign_rom[35]=0; ltf_sign_rom[36]=0; ltf_sign_rom[37]=0;
        // Bins 38-63: negative subcarriers -26 to -1
        // LTF_L(-26..-1) = +1,+1,-1,-1,+1,+1,-1,+1,-1,+1,+1,+1,+1,+1,+1,-1,-1,+1,+1,-1,+1,-1,+1,+1,+1,+1
        ltf_sign_rom[38]=1; ltf_sign_rom[39]=1; ltf_sign_rom[40]=0; ltf_sign_rom[41]=0;
        ltf_sign_rom[42]=1; ltf_sign_rom[43]=1; ltf_sign_rom[44]=0; ltf_sign_rom[45]=1;
        ltf_sign_rom[46]=0; ltf_sign_rom[47]=1; ltf_sign_rom[48]=1; ltf_sign_rom[49]=1;
        ltf_sign_rom[50]=1; ltf_sign_rom[51]=1; ltf_sign_rom[52]=1; ltf_sign_rom[53]=0;
        ltf_sign_rom[54]=0; ltf_sign_rom[55]=1; ltf_sign_rom[56]=1; ltf_sign_rom[57]=0;
        ltf_sign_rom[58]=1; ltf_sign_rom[59]=0; ltf_sign_rom[60]=1; ltf_sign_rom[61]=1;
        ltf_sign_rom[62]=1; ltf_sign_rom[63]=1;
    end

    // =========================================================
    // Storage — force to block RAM (saves ~600 LUTs vs distributed RAM)
    // Access pattern is single-port sequential in each FSM phase.
    // =========================================================
    (* ram_style = "block" *) reg signed [15:0] ltf1_re [0:63];
    (* ram_style = "block" *) reg signed [15:0] ltf1_im [0:63];
    (* ram_style = "block" *) reg signed [15:0] h_mem_re [0:63];
    (* ram_style = "block" *) reg signed [15:0] h_mem_im [0:63];
    (* ram_style = "block" *) reg signed [15:0] hinv_mem_re [0:63];
    (* ram_style = "block" *) reg signed [15:0] hinv_mem_im [0:63];

    // Read port (1 cycle latency)
    always @(posedge clk) begin
        h_inv_re <= hinv_mem_re[rd_addr];
        h_inv_im <= hinv_mem_im[rd_addr];
    end

    // =========================================================
    // FSM States
    // =========================================================
    localparam S_IDLE         = 3'd0;
    localparam S_CAPTURE_LTF1 = 3'd1;
    localparam S_WAIT_LTF2    = 3'd2;
    localparam S_CAPTURE_LTF2 = 3'd3;
    localparam S_FIND_MAX     = 3'd4;
    localparam S_COMPUTE      = 3'd5;
    localparam S_DONE         = 3'd6;

    reg [2:0]  state;
    reg [5:0]  cap_count;

    // Stall watchdog for the capture states (see header comment)
    localparam [9:0] CAPTURE_TIMEOUT = 10'd1023;
    reg [9:0]  idle_cnt;

    // Find-max phase
    reg [5:0]  max_addr;
    reg [31:0] max_mag_sq;
    reg        max_step;        // 0=compute, 1=compare
    reg [31:0] max_msq_calc;   // registered mag_sq for current bin

    // Compute phase
    reg [5:0]  comp_addr;
    reg [2:0]  comp_step;

    reg signed [15:0] cur_h_re, cur_h_im;
    reg [31:0] mag_sq;
    reg signed [15:0] result_re, result_im;

    // Active bin check — natural FFT bin order:
    //   bins 1-26  = positive subcarriers +1..+26
    //   bins 38-63 = negative subcarriers -26..-1
    wire comp_active = (comp_addr >= 1 && comp_addr <= 26) ||
                       (comp_addr >= 38);

    // Same check for find_max addressing
    wire max_active = (max_addr >= 1 && max_addr <= 26) ||
                      (max_addr >= 38);

    // H computation during LTF2 capture
    wire signed [16:0] avg_sum_re = {bin_re[15], bin_re} + {ltf1_re[bin_idx][15], ltf1_re[bin_idx]};
    wire signed [16:0] avg_sum_im = {bin_im[15], bin_im} + {ltf1_im[bin_idx][15], ltf1_im[bin_idx]};
    wire signed [15:0] avg_re = avg_sum_re[16:1];
    wire signed [15:0] avg_im = avg_sum_im[16:1];

    // =========================================================
    // Count Leading Zeros (CLZ) for adaptive shift
    // =========================================================
    // Finds position of highest set bit in a 32-bit value.
    // Returns number of leading zeros (0 = bit 31 set, 31 = bit 0 set, 32 = zero).
    function [5:0] clz32;
        input [31:0] x;
        integer i;
        begin
            clz32 = 32;
            for (i = 31; i >= 0; i = i - 1)
                if (x[i] && clz32 == 32)
                    clz32 = 6'd31 - i[5:0];
        end
    endfunction

    // =========================================================
    // Saturating absolute value for 16-bit signed
    // =========================================================
    // -32768 cannot be negated in 16 bits; clamp to 32767.
    // Used in division setup to prevent silent wraparound on
    // channel nulls or clipping transients.
    function [15:0] sat_abs16;
        input signed [15:0] x;
        begin
            sat_abs16 = (x == -16'sd32768) ? 16'd32767 : (x[15] ? (-x) : x);
        end
    endfunction

    // =========================================================
    // Division: iterative restoring divider
    // Computes quotient = (abs(dividend) << div_shift) / divisor
    // The shift is applied by initializing the working register with
    // the dividend pre-shifted within the 64-bit word.
    // Sign is applied separately. Takes 32 cycles per division (1 bit per clock).
    // =========================================================
    reg        div_start;
    reg        div_busy;
    reg        div_done;
    reg [5:0]  div_step;       // 0..31 iteration counter
    reg        div_sign;       // sign of result
    reg [15:0] div_dividend;   // unsigned absolute value of numerator (16-bit)
    reg [31:0] div_divisor;    // unsigned divisor (mag_sq)
    reg [4:0]  div_shift;      // left shift to apply to dividend
    reg [63:0] div_working;    // {remainder, quotient} working register
    reg [31:0] div_quotient;   // final result

    // Saturation: clamp unsigned quotient to signed 16-bit with sign
    wire signed [15:0] div_result;
    assign div_result = (div_quotient > 32'd32767) ?
                        (div_sign ? -16'sd32767 : 16'sd32767) :
                        (div_sign ? -$signed(div_quotient[15:0]) :
                                     $signed(div_quotient[15:0]));

    // Compute the initial working register value: {0, dividend} << div_shift
    // This effectively makes the divider compute (dividend << div_shift) / divisor
    wire [63:0] div_init = {32'd0, {16'd0, div_dividend}} << div_shift;

    // Restoring division: shift-subtract, restore if negative
    always @(posedge clk) begin
        if (!rst_n) begin
            div_busy  <= 0;
            div_done  <= 0;
            div_step  <= 0;
        end else begin
            div_done <= 0;
            if (div_start && !div_busy) begin
                div_busy    <= 1;
                div_step    <= 0;
                div_working <= div_init;
            end else if (div_busy) begin
                if ({1'b0, div_working[62:31]} >= {1'b0, div_divisor}) begin
                    div_working <= {div_working[62:31] - div_divisor, div_working[30:0], 1'b1};
                end else begin
                    div_working <= {div_working[62:0], 1'b0};
                end

                div_step <= div_step + 1;
                if (div_step == 31) begin
                    div_busy <= 0;
                    div_done <= 1;
                    if ({1'b0, div_working[62:31]} >= {1'b0, div_divisor})
                        div_quotient <= {div_working[30:0], 1'b1};
                    else
                        div_quotient <= {div_working[30:0], 1'b0};
                end
            end
        end
    end

    // =========================================================
    // Main FSM
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            done       <= 0;
            cap_count  <= 0;
            idle_cnt   <= 0;
            clip_cnt   <= 0;
            comp_addr  <= 0;
            comp_step  <= 0;
            div_start  <= 0;
            max_addr   <= 0;
            max_mag_sq <= 0;
            max_step   <= 0;
            max_msq_calc <= 0;
            shift_val  <= 5'd15;  // default: Q1.15 (backward compat if somehow unused)
        end else begin
            done <= 0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        state     <= S_CAPTURE_LTF1;
                        cap_count <= 0;
                        idle_cnt  <= 0;
                    end
                end

                S_CAPTURE_LTF1: begin
                    if (bin_valid) begin
                        ltf1_re[bin_idx] <= bin_re;
                        ltf1_im[bin_idx] <= bin_im;
                        idle_cnt <= 0;
                        cap_count <= cap_count + 1;
                        if (cap_count == 63) begin
                            state     <= S_WAIT_LTF2;
                            cap_count <= 0;
                        end
                    end else if (idle_cnt == CAPTURE_TIMEOUT) begin
                        state <= S_IDLE;
                    end else begin
                        idle_cnt <= idle_cnt + 1;
                    end
                end

                S_WAIT_LTF2: begin
                    if (bin_valid) begin
                        idle_cnt <= 0;
                        cap_count <= 1;
                        state     <= S_CAPTURE_LTF2;
                        if (ltf_sign_rom[bin_idx]) begin
                            h_mem_re[bin_idx] <= avg_re;
                            h_mem_im[bin_idx] <= avg_im;
                        end else begin
                            h_mem_re[bin_idx] <= -avg_re;
                            h_mem_im[bin_idx] <= -avg_im;
                        end
                    end else if (idle_cnt == CAPTURE_TIMEOUT) begin
                        state <= S_IDLE;
                    end else begin
                        idle_cnt <= idle_cnt + 1;
                    end
                end

                S_CAPTURE_LTF2: begin
                    if (bin_valid) begin
                        idle_cnt <= 0;
                        cap_count <= cap_count + 1;
                        if (ltf_sign_rom[bin_idx]) begin
                            h_mem_re[bin_idx] <= avg_re;
                            h_mem_im[bin_idx] <= avg_im;
                        end else begin
                            h_mem_re[bin_idx] <= -avg_re;
                            h_mem_im[bin_idx] <= -avg_im;
                        end
                        if (cap_count == 63) begin
                            state      <= S_FIND_MAX;
                            max_addr   <= 0;
                            max_mag_sq <= 0;
                            max_step   <= 0;
                        end
                    end else if (idle_cnt == CAPTURE_TIMEOUT) begin
                        state <= S_IDLE;
                    end else begin
                        idle_cnt <= idle_cnt + 1;
                    end
                end

                // ===================================================
                // FIND_MAX: scan active bins to find max |H|^2
                // Takes 128 cycles (2 per bin: compute, then compare).
                // At completion, max_mag_sq holds the largest |H[k]|^2
                // and shift_val is computed from CLZ.
                // ===================================================
                S_FIND_MAX: begin
                    if (!max_step) begin
                        // Step 0: compute |H[k]|^2 (register the result)
                        if (max_active) begin
                            // Use DSP-friendly signed multiply
                            max_msq_calc <= ($signed(h_mem_re[max_addr]) *
                                            $signed(h_mem_re[max_addr])) +
                                           ($signed(h_mem_im[max_addr]) *
                                            $signed(h_mem_im[max_addr]));
                        end else begin
                            max_msq_calc <= 0;
                        end
                        max_step <= 1;
                    end else begin
                        // Step 1: compare and advance
                        if (max_msq_calc > max_mag_sq)
                            max_mag_sq <= max_msq_calc;

                        max_step <= 0;
                        if (max_addr == 63) begin
                            // Compute adaptive shift from max_mag_sq
                            begin : compute_shift_block
                                reg [5:0] lz;
                                reg [5:0] bit_w;
                                reg [4:0] sv;
                                lz = clz32(max_mag_sq);
                                bit_w = 32 - lz;
                                sv = 10 + bit_w[5:1];
                                if (sv < 15) sv = 15;
                                shift_val <= sv;
                            end
                            state     <= S_COMPUTE;
                            comp_addr <= 0;
                            comp_step <= 0;
                        end else begin
                            max_addr <= max_addr + 1;
                        end
                    end
                end

                S_COMPUTE: begin
                    case (comp_step)
                        3'd0: begin
                            // Step 0: check if active, read H
                            div_start <= 0;
                            if (!comp_active) begin
                                hinv_mem_re[comp_addr] <= 16'sd0;
                                hinv_mem_im[comp_addr] <= 16'sd0;
                                if (comp_addr == 63) state <= S_DONE;
                                else comp_addr <= comp_addr + 1;
                            end else begin
                                cur_h_re  <= h_mem_re[comp_addr];
                                cur_h_im  <= h_mem_im[comp_addr];
                                comp_step <= 1;
                            end
                        end

                        3'd1: begin
                            // Step 1: Compute |H|^2
                            // H_inv_re = (H_re << shift_val) / |H|^2
                            // H_inv_im = (-H_im << shift_val) / |H|^2
                            mag_sq <= ({{16{cur_h_re[15]}}, cur_h_re} *
                                       {{16{cur_h_re[15]}}, cur_h_re}) +
                                      ({{16{cur_h_im[15]}}, cur_h_im} *
                                       {{16{cur_h_im[15]}}, cur_h_im});
                            comp_step <= 2;
                        end

                        3'd2: begin
                            // Step 2: Start division for H_inv_re = (|H_re| << shift_val) / mag_sq
                            // The divider handles the shift internally via div_init
                            if (cur_h_re == 0 || mag_sq == 0) begin
                                result_re <= 16'sd0;
                                comp_step <= 3;
                            end else if (!div_busy && !div_done) begin
                                div_divisor  <= mag_sq;
                                div_dividend <= sat_abs16(cur_h_re);
                                div_sign     <= cur_h_re[15];
                                div_shift    <= shift_val;
                                div_start    <= 1;
                            end else begin
                                div_start <= 0;
                                if (div_done) begin
                                    result_re <= div_result;
                                    if (div_quotient > 32'd32767 && clip_cnt != 16'hFFFF)
                                        clip_cnt <= clip_cnt + 1;
                                    comp_step <= 3;
                                end
                            end
                        end

                        3'd3: begin
                            // Step 3: Start division for H_inv_im = (|H_im| << shift_val) / mag_sq
                            // Note: H_inv_im = -H_im (conjugate), so sign is flipped
                            if (cur_h_im == 0 || mag_sq == 0) begin
                                result_im <= 16'sd0;
                                comp_step <= 4;
                            end else if (!div_busy && !div_done) begin
                                div_divisor  <= mag_sq;
                                div_dividend <= sat_abs16(cur_h_im);
                                div_sign     <= ~cur_h_im[15]; // conjugate: negate imaginary
                                div_shift    <= shift_val;
                                div_start    <= 1;
                            end else begin
                                div_start <= 0;
                                if (div_done) begin
                                    result_im <= div_result;
                                    if (div_quotient > 32'd32767 && clip_cnt != 16'hFFFF)
                                        clip_cnt <= clip_cnt + 1;
                                    comp_step <= 4;
                                end
                            end
                        end

                        3'd4: begin
                            // Step 4: Store results and advance
                            div_start <= 0;
                            hinv_mem_re[comp_addr] <= result_re;
                            hinv_mem_im[comp_addr] <= result_im;
                            comp_step <= 0;
                            if (comp_addr == 63) state <= S_DONE;
                            else comp_addr <= comp_addr + 1;
                        end

                        default: comp_step <= 0;
                    endcase
                end

                S_DONE: begin
                    done  <= 1;
                    state <= S_IDLE;
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
        if (state == 3'd7)
            $error("[chan_est] ILLEGAL STATE: reached unreachable state 7");
    end
    `endif

endmodule
