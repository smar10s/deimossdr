// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/descrambler.v — 802.11 data descrambler (x^7 + x^4 + 1 LFSR)
//
// IEEE 802.11-2020 Section 17.3.5.4.
//
// Self-synchronizing: the first 7 input bits (SERVICE field zeros after
// scrambling) are used to initialize the LFSR state. No external seed
// input required.
//
// Algorithm:
//   - Bits 0-6 (cnt < 7): feedback = input_bit (self-sync)
//   - Bits 7+  (cnt >= 7): feedback = state[6] ^ state[3]
//   - Output = input ^ feedback
//   - State  = {state[5:0], feedback}   (shift left, fb enters at LSB)
//
// After 7 bits the LFSR is synchronized and seed_detected is valid.
module descrambler (
    input  wire        clk,
    input  wire        rst_n,

    // Frame control
    input  wire        frame_start,    // pulse: reset for new frame

    // Input (from Viterbi)
    input  wire        valid_in,
    input  wire        bit_in,

    // Output (descrambled)
    output reg         valid_out,
    output reg         bit_out,

    // Seed detection (valid after 7th input bit)
    output reg  [6:0]  seed_detected,
    output reg         seed_valid
);

    reg [6:0] state;       // 7-bit LFSR
    reg [3:0] bit_cnt;     // counts 0-7+ (saturates at 8+)

    wire feedback_normal = state[6] ^ state[3];
    wire in_sync_phase   = (bit_cnt < 4'd7);
    wire feedback        = in_sync_phase ? bit_in : feedback_normal;

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= 7'd0;
            bit_cnt      <= 4'd0;
            valid_out    <= 1'b0;
            bit_out      <= 1'b0;
            seed_detected <= 7'd0;
            seed_valid   <= 1'b0;
        end else if (frame_start) begin
            state        <= 7'd0;
            bit_cnt      <= 4'd0;
            valid_out    <= 1'b0;
            bit_out      <= 1'b0;
            seed_detected <= 7'd0;
            seed_valid   <= 1'b0;
        end else if (valid_in) begin
            // Descramble
            bit_out   <= bit_in ^ feedback;
            valid_out <= 1'b1;

            // Advance LFSR
            state <= {state[5:0], feedback};

            // Bit counter (saturate at 8 to save logic)
            if (bit_cnt < 4'd8)
                bit_cnt <= bit_cnt + 4'd1;

            // Seed detection: after 7th bit, LFSR state is determined
            if (bit_cnt == 4'd6) begin
                // State will be {state[5:0], feedback} NEXT cycle
                // But we can report the state as-is after this update
                seed_detected <= {state[5:0], feedback};
                seed_valid    <= 1'b1;
            end
        end else begin
            valid_out <= 1'b0;
        end
    end

endmodule
