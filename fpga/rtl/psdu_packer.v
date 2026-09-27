// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/psdu_packer.v — Serial bit-to-byte packer for PSDU recovery
//
// Receives descrambled bits from the descrambler (valid_in, bit_in) one per
// clock cycle, packs them into bytes (LSB first per 802.11), and emits
// byte_valid + byte_out[7:0] for each completed byte.
//
// Behavior:
//   - On frame_start: reset counters, latch payload_len = psdu_len - 4
//   - Skip first 16 bits (SERVICE field)
//   - After SERVICE: pack bits into bytes, LSB first
//   - Emit byte_valid + byte_out when 8 bits accumulated
//   - Stop after payload_len bytes emitted (FCS bytes not emitted)
//   - byte_count tracks total bytes emitted
//   - active is high while packing a frame
//   - done pulses when all payload bytes emitted
module psdu_packer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,    // pulse: reset for new frame
    input  wire [11:0] psdu_len,       // PSDU length in bytes (incl FCS)
    input  wire        valid_in,       // from descrambler
    input  wire        bit_in,         // from descrambler
    output reg         byte_valid,     // pulse: byte ready
    output reg  [7:0]  byte_out,       // packed byte (LSB first)
    output reg  [11:0] byte_count,     // bytes emitted so far
    output wire        active,
    output wire        done,
    output wire        frame_done,     // alias for done
    output reg         frame_start_out // registered copy of frame_start
);

    reg        active_r;
    reg        done_r;
    reg [11:0] payload_len;    // psdu_len - 4 (exclude FCS)
    reg [4:0]  service_skip;   // counts 16 SERVICE bits to discard
    reg [2:0]  bit_cnt;        // 0-7 within current byte
    reg [7:0]  shift_reg;      // accumulates bits LSB first

    assign active     = active_r;
    assign done       = done_r;
    assign frame_done = done_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            active_r     <= 1'b0;
            done_r       <= 1'b0;
            byte_valid      <= 1'b0;
            byte_out        <= 8'd0;
            byte_count      <= 12'd0;
            payload_len     <= 12'd0;
            service_skip    <= 5'd0;
            bit_cnt         <= 3'd0;
            shift_reg       <= 8'd0;
            frame_start_out <= 1'b0;
        end else if (frame_start) begin
            active_r        <= 1'b1;
            done_r          <= 1'b0;
            byte_valid      <= 1'b0;
            byte_out        <= 8'd0;
            byte_count      <= 12'd0;
            payload_len     <= psdu_len - 12'd4;
            service_skip    <= 5'd16;
            frame_start_out <= 1'b1;
            bit_cnt      <= 3'd0;
            shift_reg    <= 8'd0;
        end else begin
            // Clear pulse outputs
            byte_valid      <= 1'b0;
            done_r          <= 1'b0;
            frame_start_out <= 1'b0;

            if (active_r && valid_in) begin
                if (service_skip != 5'd0) begin
                    // Discard SERVICE field bits (first 16)
                    service_skip <= service_skip - 5'd1;
                end else begin
                    // Pack bit into shift register (LSB first)
                    shift_reg <= {bit_in, shift_reg[7:1]};
                    
                    if (bit_cnt == 3'd7) begin
                        // Byte complete
                        bit_cnt    <= 3'd0;
                        byte_out   <= {bit_in, shift_reg[7:1]};
                        byte_valid <= 1'b1;
                        byte_count <= byte_count + 12'd1;

                        // Check if this was the last payload byte
                        if (byte_count + 12'd1 == payload_len) begin
                            active_r <= 1'b0;
                            done_r   <= 1'b1;
                        end
                    end else begin
                        bit_cnt <= bit_cnt + 3'd1;
                    end
                end
            end
        end
    end

endmodule
