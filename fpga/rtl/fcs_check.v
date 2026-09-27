// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/fcs_check.v — 802.11 FCS (CRC-32) checker
//
// Receives descrambled bits serially (one per clock when valid), packs into
// bytes (LSB first per 802.11), and runs CRC-32 (Ethernet polynomial,
// bit-reversed: 0xEDB88320).
//
// After all PSDU bytes (including 4-byte FCS) are processed, asserts
// fcs_valid if the CRC residual equals the magic constant 0xDEBB20E3.
// 0xDEBB20E3 is not arbitrary: it is the reflected residual left in the
// register when the bit-reversed CRC-32 below is run over data+FCS and the
// FCS is correct (the reflected form of the 0x04C11DB7 polynomial).
//
// Interface:
//   frame_start: pulse to reset for new frame
//   psdu_len[11:0]: PSDU length in bytes (from SIGNAL field, includes FCS)
//   valid_in + bit_in: serial descrambled bits (after SERVICE field removed)
//   fcs_valid: asserted for one cycle when FCS check passes
//   fcs_fail: asserted for one cycle when FCS check fails
//   frame_done: asserted when all psdu_len bytes have been processed
//
// Note: the caller is responsible for skipping the 16 SERVICE bits before
// feeding PSDU bits to this module. This module expects bit 0 of byte 0
// of the PSDU as its first input.
module fcs_check (
    input  wire         clk,
    input  wire         rst_n,

    // Frame control
    input  wire         frame_start,    // pulse: reset for new frame
    input  wire [11:0]  psdu_len,       // PSDU length in bytes (incl FCS)

    // Input (from descrambler, after SERVICE skip)
    input  wire         valid_in,
    input  wire         bit_in,

    // Output
    output reg          fcs_valid,      // pulse: FCS OK
    output reg          fcs_fail,       // pulse: FCS bad
    output reg          frame_done      // pulse: all bytes processed
);

    // CRC-32 magic residual: the reflected residue left in the register
    // after a correct CRC-32 (process data+FCS with reflected polynomial
    // 0xEDB88320 and the register holds 0xDEBB20E3 iff the FCS is intact).
    localparam [31:0] CRC_RESIDUAL = 32'hDEBB20E3;
    // CRC initial value
    localparam [31:0] CRC_INIT = 32'hFFFFFFFF;

    reg [31:0] crc;            // running CRC state
    reg [2:0]  bit_cnt;        // 0-7 within current byte
    reg [11:0] byte_cnt;       // bytes processed so far
    reg [11:0] psdu_len_r;     // latched PSDU length
    reg        active;         // processing frame
    reg [4:0]  service_skip;   // counts 16 SERVICE bits to discard

    // CRC-32 update: one bit at a time (reflected/LSB-first)
    // If (crc XOR new_bit) has LSB=1: crc = (crc >> 1) XOR 0xEDB88320
    // Else: crc = crc >> 1
    wire crc_bit = crc[0] ^ bit_in;
    wire [31:0] crc_next = crc_bit ? ({1'b0, crc[31:1]} ^ 32'hEDB88320)
                                   : {1'b0, crc[31:1]};

    always @(posedge clk) begin
        if (!rst_n) begin
            crc          <= CRC_INIT;
            bit_cnt      <= 3'd0;
            byte_cnt     <= 12'd0;
            psdu_len_r   <= 12'd0;
            active       <= 1'b0;
            service_skip <= 5'd0;
            fcs_valid    <= 1'b0;
            fcs_fail     <= 1'b0;
            frame_done   <= 1'b0;
        end else if (frame_start) begin
            crc          <= CRC_INIT;
            bit_cnt      <= 3'd0;
            byte_cnt     <= 12'd0;
            psdu_len_r   <= psdu_len;
            active       <= 1'b1;
            service_skip <= 5'd16;
            fcs_valid    <= 1'b0;
            fcs_fail     <= 1'b0;
            frame_done   <= 1'b0;
        end else begin
            // Clear pulse outputs
            fcs_valid  <= 1'b0;
            fcs_fail   <= 1'b0;
            frame_done <= 1'b0;

            if (active && valid_in) begin
                if (service_skip != 0) begin
                    // Discard SERVICE field bits (first 16)
                    service_skip <= service_skip - 1;
                end else begin
                // Update CRC with each bit
                crc <= crc_next;

                // Count bits within byte
                if (bit_cnt == 3'd7) begin
                    bit_cnt  <= 3'd0;
                    byte_cnt <= byte_cnt + 12'd1;

                    // Check if this was the last byte
                    if (byte_cnt + 12'd1 == psdu_len_r) begin
                        active     <= 1'b0;
                        frame_done <= 1'b1;
                        // Check residual
                        if (crc_next == CRC_RESIDUAL) begin
                            fcs_valid <= 1'b1;
                        end else begin
                            fcs_fail <= 1'b1;
                        end
                    end
                end else begin
                    bit_cnt <= bit_cnt + 3'd1;
                end
                end  // else (service_skip == 0)
            end
        end
    end

endmodule
