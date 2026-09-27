// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/fft64_sdf.v — R2²SDF FFT (clean implementation)
//
// Built incrementally: fft4_sdf → fft16_sdf → fft64_sdf
// Each sub-FFT is independently testable.
//
// Architecture: R2²SDF (He & Torkelson 1996), DIF, streaming.
// Natural-order input, bit-reversed output.
//
// Key design: ONE global counter drives all stages. Each stage uses a
// delayed copy (shifted by upstream pipeline latency). This ensures
// the control signals always correspond to the original sample position
// in the N-sample frame.

// =============================================================================
// sdf_delay: Parameterized delay line
//   DEPTH samples deep. Circular buffer.
//   COMBINATIONAL read: output = oldest entry (about to be overwritten).
//   Write on posedge when en=1.
// =============================================================================
module sdf_delay #(
    parameter DEPTH   = 2,
    parameter DW      = 17,
    parameter WR_PIPE = 0   // When 1 (BRAM only): register write data, breaking
                            // the read-modify-write critical path. The write pipe
                            // latency is absorbed (data sits in BRAM for DEPTH-1
                            // cycles before being read). Effective delay = DEPTH.
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             en,
    input  wire signed [DW-1:0] wr_re,
    input  wire signed [DW-1:0] wr_im,
    output wire signed [DW-1:0] rd_re,
    output wire signed [DW-1:0] rd_im
);
    generate
        if (DEPTH == 1) begin : gen_reg
            reg signed [DW-1:0] sr_re, sr_im;
            initial begin sr_re = 0; sr_im = 0; end

            assign rd_re = sr_re;
            assign rd_im = sr_im;

            always @(posedge clk) begin
                if (!rst_n) begin
                    sr_re <= 0;
                    sr_im <= 0;
                end else if (en) begin
                    sr_re <= wr_re;
                    sr_im <= wr_im;
                end
            end
        end else if (DEPTH >= 8) begin : gen_bram
            // Promote deeper delay lines to BRAM to save LUTs.
            // DEPTH 32/16/8 instances consume ~400-800 LUTs as distributed
            // RAM (combinational read → MUX trees). Block RAM has plenty of
            // headroom (56 free BRAM36K tiles).
            //
            // BRAM has registered output (1-cycle read latency). To maintain
            // the combinational-read interface the butterfly expects, we use
            // a lookahead read pointer: each cycle we pre-read the NEXT
            // address so the data is available combinationally (from the
            // output register) when the butterfly needs it one cycle later.
            //
            // Invariant: rd_reg always holds mem[wr_ptr] (the oldest entry)
            // at the moment the butterfly samples it — same as the distributed
            // RAM version's `assign rd = mem[wr_ptr]`.
            //
            // WR_PIPE=1 mode: Write data is registered before reaching BRAM,
            // breaking the critical path: BRAM_rd_reg → butterfly_logic → BRAM_wr.
            // The write occurs 1 cycle after the butterfly computes the result.
            // The write pipe latency is absorbed: the data sits in BRAM for
            // DEPTH-1 more cycles before being read again, so effective delay
            // remains DEPTH (unchanged from WR_PIPE=0).
            localparam AW = $clog2(DEPTH);
            reg [AW-1:0] wr_ptr;
            wire [AW-1:0] next_wr_ptr = (wr_ptr == DEPTH[AW-1:0] - 1)
                                         ? {AW{1'b0}} : wr_ptr + 1;

            (* ram_style = "block" *) reg signed [DW-1:0] mem_re [0:DEPTH-1];
            (* ram_style = "block" *) reg signed [DW-1:0] mem_im [0:DEPTH-1];

            reg signed [DW-1:0] rd_reg_re, rd_reg_im;

            integer i;
            initial begin
                wr_ptr = 0;
                rd_reg_re = 0;
                rd_reg_im = 0;
                for (i = 0; i < DEPTH; i = i + 1) begin
                    mem_re[i] = 0;
                    mem_im[i] = 0;
                end
            end

            assign rd_re = rd_reg_re;
            assign rd_im = rd_reg_im;

            // Split into separate always blocks so Vivado infers Simple
            // Dual-Port BRAM (Port A = write, Port B = read). A single
            // always block with both write and read at different addresses
            // causes Vivado to infer single-port BRAM, breaking the
            // lookahead (only one address port available per cycle).

            if (WR_PIPE) begin : gen_wr_pipe
                // --------------------------------------------------------
                // WR_PIPE=1: Register write data + address to break the
                // read-modify-write critical path through the butterfly.
                //
                // Timing (each enabled clock):
                //   Cycle N:   Butterfly reads rd_reg, computes wr_re/wr_im
                //              Pipeline captures: wr_pipe_re/im = wr_re/wr_im
                //                                 wr_pipe_ptr = wr_ptr
                //              Read port: rd_reg <= mem[next_wr_ptr]
                //              Pointer: wr_ptr <= next_wr_ptr
                //   Cycle N+1: BRAM write: mem[wr_pipe_ptr] <= wr_pipe_re/im
                //
                // The write is delayed by 1 cycle. The next read from that
                // address won't happen until DEPTH cycles later (the SDF
                // butterfly pattern guarantees this). Safe margin = DEPTH-1
                // cycles minimum.
                // --------------------------------------------------------
                reg signed [DW-1:0] wr_pipe_re, wr_pipe_im;
                reg [AW-1:0]        wr_pipe_ptr;
                reg                 wr_pipe_valid;

                initial begin
                    wr_pipe_re = 0;
                    wr_pipe_im = 0;
                    wr_pipe_ptr = 0;
                    wr_pipe_valid = 0;
                end

                // Pipeline stage: capture write data + address
                always @(posedge clk) begin
                    wr_pipe_re    <= wr_re;
                    wr_pipe_im    <= wr_im;
                    wr_pipe_ptr   <= wr_ptr;
                    wr_pipe_valid <= en;
                end

                // Write port (BRAM Port A) — uses pipelined data
                always @(posedge clk) begin
                    if (wr_pipe_valid) begin
                        mem_re[wr_pipe_ptr] <= wr_pipe_re;
                        mem_im[wr_pipe_ptr] <= wr_pipe_im;
                    end
                end
            end else begin : gen_wr_direct
                // --------------------------------------------------------
                // WR_PIPE=0 (default): Direct write, same cycle.
                // --------------------------------------------------------

                // Write port (BRAM Port A)
                always @(posedge clk) begin
                    if (en) begin
                        mem_re[wr_ptr] <= wr_re;
                        mem_im[wr_ptr] <= wr_im;
                    end
                end
            end

            // Read port (BRAM Port B) — lookahead: pre-read next address
            //
            // Timing: on each enabled clock:
            //   Port B reads mem[next_wr_ptr] into rd_reg (lookahead)
            //   wr_ptr advances to next_wr_ptr
            //
            // Result: on the NEXT cycle, rd_reg holds mem[new wr_ptr],
            // which IS the new oldest entry — exactly what the butterfly
            // will read combinationally via rd_re/rd_im.
            always @(posedge clk) begin
                if (!rst_n) begin
                    rd_reg_re <= 0;
                    rd_reg_im <= 0;
                end else if (en) begin
                    rd_reg_re <= mem_re[next_wr_ptr];
                    rd_reg_im <= mem_im[next_wr_ptr];
                end
            end

            // Pointer advance
            always @(posedge clk) begin
                if (!rst_n)
                    wr_ptr <= 0;
                else if (en) begin
                    wr_ptr <= next_wr_ptr;
                end
            end
        end else begin : gen_ram
            localparam AW = $clog2(DEPTH);
            reg [AW-1:0] wr_ptr;
            reg signed [DW-1:0] mem_re [0:DEPTH-1];
            reg signed [DW-1:0] mem_im [0:DEPTH-1];

            integer i;
            initial begin
                wr_ptr = 0;
                for (i = 0; i < DEPTH; i = i + 1) begin
                    mem_re[i] = 0;
                    mem_im[i] = 0;
                end
            end

            // Combinational read of oldest entry
            assign rd_re = mem_re[wr_ptr];
            assign rd_im = mem_im[wr_ptr];

            always @(posedge clk) begin
                if (!rst_n)
                    wr_ptr <= 0;
                else if (en) begin
                    mem_re[wr_ptr] <= wr_re;
                    mem_im[wr_ptr] <= wr_im;
                    wr_ptr <= (wr_ptr == DEPTH[AW-1:0] - 1) ? {AW{1'b0}} : wr_ptr + 1;
                end
            end
        end
    endgenerate
endmodule


// =============================================================================
// fft4_sdf: 4-point R2²SDF FFT (one stage pair, no twiddle)
//
//   Global counter n counts 0..N-1 (N=4, so 2 bits, wraps).
//   For DIF with N=4:
//     BF1: delay=2, sel = n[1]
//     BF2: delay=1, sel = n_d[0], rot = n_d[1] & n_d[0]
//       where n_d = n delayed by 1 clock (BF1 output registration)
//
//   The counter wraps at N=4. Feed N samples then N zeros (total 2N=8 clocks).
//   The pipeline produces valid output during the second N clocks.
//
//   Input: 16-bit signed, output: 18-bit signed (2 butterflies, +1 bit each)
// =============================================================================
module fft4_sdf (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        din_valid,
    input  wire [1:0]  i_idx,
    input  wire signed [15:0] din_re,
    input  wire signed [15:0] din_im,

    output wire        dout_valid,
    output wire signed [17:0] dout_re,
    output wire signed [17:0] dout_im,
    output wire [1:0]  dout_idx
);

    // =========================================================
    // Global counter (wraps at N=4, counts valid input samples)
    // =========================================================
    reg [1:0] gcnt;

    always @(posedge clk) begin
        if (!rst_n)
            gcnt <= 0;
        else if (din_valid)
            gcnt <= gcnt + 1;
    end

    // =========================================================
    // BF1: delay=2, sel = gcnt[1]
    // =========================================================
    wire sel1 = gcnt[1];

    wire signed [16:0] dl1_rd_re, dl1_rd_im;
    reg  signed [16:0] dl1_wr_re, dl1_wr_im;

    wire signed [16:0] in1_re = {{1{din_re[15]}}, din_re};
    wire signed [16:0] in1_im = {{1{din_im[15]}}, din_im};

    sdf_delay #(.DEPTH(2), .DW(17)) u_dl1 (
        .clk(clk), .rst_n(rst_n), .en(din_valid),
        .wr_re(dl1_wr_re), .wr_im(dl1_wr_im),
        .rd_re(dl1_rd_re), .rd_im(dl1_rd_im)
    );

    always @(*) begin
        if (sel1) begin
            dl1_wr_re = dl1_rd_re - in1_re;
            dl1_wr_im = dl1_rd_im - in1_im;
        end else begin
            dl1_wr_re = in1_re;
            dl1_wr_im = in1_im;
        end
    end

    // BF1 output (registered, 1 clock latency)
    reg signed [16:0] bf1_re, bf1_im;
    reg bf1_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            bf1_re    <= 0;
            bf1_im    <= 0;
            bf1_valid <= 0;
        end else if (din_valid) begin
            if (sel1) begin
                bf1_re <= dl1_rd_re + in1_re;
                bf1_im <= dl1_rd_im + in1_im;
            end else begin
                bf1_re <= dl1_rd_re;
                bf1_im <= dl1_rd_im;
            end
            bf1_valid <= 1;
        end else begin
            bf1_valid <= 0;
        end
    end

    // =========================================================
    // Counter delayed by 1 (BF1 output latency) for BF2 control
    // =========================================================
    reg [1:0] gcnt_d1;

    always @(posedge clk) begin
        if (!rst_n)
            gcnt_d1 <= 0;
        else if (din_valid)
            gcnt_d1 <= gcnt;
    end

    // =========================================================
    // BF2: delay=1, sel = gcnt_d1[0], rot = gcnt_d1[1] & gcnt_d1[0]
    // =========================================================
    wire sel2 = gcnt_d1[0];
    wire rot2 = ~gcnt_d1[1] & gcnt_d1[0];  // fires when processing diff pair butterfly

    wire signed [17:0] bf1_ext_re = {{1{bf1_re[16]}}, bf1_re};
    wire signed [17:0] bf1_ext_im = {{1{bf1_im[16]}}, bf1_im};

    // -j rotation: re' = im, im' = -re
    wire signed [17:0] in2_re = rot2 ? bf1_ext_im    : bf1_ext_re;
    wire signed [17:0] in2_im = rot2 ? (-bf1_ext_re) : bf1_ext_im;

    wire signed [17:0] dl2_rd_re, dl2_rd_im;
    reg  signed [17:0] dl2_wr_re, dl2_wr_im;

    sdf_delay #(.DEPTH(1), .DW(18)) u_dl2 (
        .clk(clk), .rst_n(rst_n), .en(bf1_valid),
        .wr_re(dl2_wr_re), .wr_im(dl2_wr_im),
        .rd_re(dl2_rd_re), .rd_im(dl2_rd_im)
    );

    always @(*) begin
        if (sel2) begin
            dl2_wr_re = dl2_rd_re - in2_re;
            dl2_wr_im = dl2_rd_im - in2_im;
        end else begin
            dl2_wr_re = in2_re;
            dl2_wr_im = in2_im;
        end
    end

    // BF2 output (registered)
    reg signed [17:0] bf2_re, bf2_im;
    reg bf2_valid;
    reg [1:0] bf2_gcnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            bf2_re    <= 0;
            bf2_im    <= 0;
            bf2_valid <= 0;
            bf2_gcnt  <= 0;
        end else if (bf1_valid) begin
            if (sel2) begin
                bf2_re <= dl2_rd_re + in2_re;
                bf2_im <= dl2_rd_im + in2_im;
            end else begin
                bf2_re <= dl2_rd_re;
                bf2_im <= dl2_rd_im;
            end
            bf2_valid <= 1;
            bf2_gcnt  <= gcnt_d1;
        end else begin
            bf2_valid <= 0;
        end
    end

    // =========================================================
    // Output: bf2 produces valid results on every clock that bf2_valid=1.
    // No gating needed for streaming mode — every bf2 output is a valid
    // butterfly result. The downstream module (fft16_sdf) handles framing.
    // =========================================================
    assign dout_valid = bf2_valid;
    assign dout_re    = bf2_re;
    assign dout_im    = bf2_im;
    assign dout_idx   = bf2_gcnt;

endmodule


// =============================================================================
// fft64_sdf: 64-point R2²SDF FFT
//
//   Composition: outer_bf_pair → twiddle_64 → fft16_sdf
//
//   The outer pair (BF1 delay=32, BF2 delay=16) handles the coarse frequency
//   split. The twiddle rotates by W_64^(phase). The inner fft16_sdf handles
//   the fine 16-point transform.
//
//   Input: 16-bit signed. Output: 16-bit signed (rounded from internal 20-bit).
//   Streaming: 1 sample/clock in, 1 sample/clock out (after pipeline fill).
//
//   CONSTRAINT: This module MUST instantiate fft16_sdf. Do not inline.
// =============================================================================
module fft64_sdf (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        gate_rst_n, // output gating reset (fill_cnt/out_active only)

    input  wire        din_valid,
    input  wire [5:0]  i_idx,
    input  wire signed [15:0] din_re,
    input  wire signed [15:0] din_im,

    output wire        dout_valid,
    output wire signed [15:0] dout_re,
    output wire signed [15:0] dout_im,
    output wire [5:0]  dout_idx
);

    // =========================================================
    // Global counter (wraps at 64)
    // =========================================================
    reg [5:0] gcnt;

    always @(posedge clk) begin
        if (!rst_n)
            gcnt <= 0;
        else if (din_valid)
            gcnt <= gcnt + 1;
    end

    // =========================================================
    // Outer BF1: delay=32, sel=gcnt[5]
    //   sel=0: store input into delay, output = delay read (old value)
    //   sel=1: output = delay + input (sum), store delay - input (diff)
    // =========================================================
    wire sel1 = gcnt[5];

    wire signed [16:0] dl1_rd_re, dl1_rd_im;
    reg  signed [16:0] dl1_wr_re, dl1_wr_im;

    wire signed [16:0] in1_re = {{1{din_re[15]}}, din_re};
    wire signed [16:0] in1_im = {{1{din_im[15]}}, din_im};

    sdf_delay #(.DEPTH(32), .DW(17), .WR_PIPE(1)) u_dl1_64 (
        .clk(clk), .rst_n(rst_n), .en(din_valid),
        .wr_re(dl1_wr_re), .wr_im(dl1_wr_im),
        .rd_re(dl1_rd_re), .rd_im(dl1_rd_im)
    );

    always @(*) begin
        if (sel1) begin
            dl1_wr_re = dl1_rd_re - in1_re;  // diff -> feedback
            dl1_wr_im = dl1_rd_im - in1_im;
        end else begin
            dl1_wr_re = in1_re;              // store input
            dl1_wr_im = in1_im;
        end
    end

    wire signed [16:0] bf1_out_re = sel1 ? (dl1_rd_re + in1_re) : dl1_rd_re;
    wire signed [16:0] bf1_out_im = sel1 ? (dl1_rd_im + in1_im) : dl1_rd_im;

    // =========================================================
    // Outer BF2: delay=16, sel=gcnt[4], rot=~gcnt[5]&gcnt[4]
    //   -j rotation: re' = im, im' = -re (applied before butterfly when rot=1)
    // =========================================================
    wire sel2 = gcnt[4];
    wire rot2 = ~gcnt[5] & gcnt[4];

    wire signed [17:0] bf1_ext_re = {{1{bf1_out_re[16]}}, bf1_out_re};
    wire signed [17:0] bf1_ext_im = {{1{bf1_out_im[16]}}, bf1_out_im};

    wire signed [17:0] in2_re = rot2 ? bf1_ext_im    : bf1_ext_re;
    wire signed [17:0] in2_im = rot2 ? (-bf1_ext_re) : bf1_ext_im;

    wire signed [17:0] dl2_rd_re, dl2_rd_im;
    reg  signed [17:0] dl2_wr_re, dl2_wr_im;

    sdf_delay #(.DEPTH(16), .DW(18), .WR_PIPE(1)) u_dl2_64 (
        .clk(clk), .rst_n(rst_n), .en(din_valid),
        .wr_re(dl2_wr_re), .wr_im(dl2_wr_im),
        .rd_re(dl2_rd_re), .rd_im(dl2_rd_im)
    );

    always @(*) begin
        if (sel2) begin
            dl2_wr_re = dl2_rd_re - in2_re;
            dl2_wr_im = dl2_rd_im - in2_im;
        end else begin
            dl2_wr_re = in2_re;
            dl2_wr_im = in2_im;
        end
    end

    wire signed [17:0] bf2_out_re = sel2 ? (dl2_rd_re + in2_re) : dl2_rd_re;
    wire signed [17:0] bf2_out_im = sel2 ? (dl2_rd_im + in2_im) : dl2_rd_im;

    // =========================================================
    // Twiddle multiply (combinational)
    //
    // ROM: W_64^phase, 64 entries addressed by gcnt[5:0].
    // Complex multiply: out = data * W
    //   out_re = data_re * tw_re - data_im * tw_im
    //   out_im = data_re * tw_im + data_im * tw_re
    // Round from Q(18).15 back to 16 bits for fft16 input.
    //
    // INITIAL: All W^0 (identity) for BF pair verification.
    // Will be replaced with empirically-determined ROM values.
    // =========================================================
    reg signed [15:0] tw_rom_re, tw_rom_im;

    always @(*) begin
        case (gcnt[5:0])
            // === Candidate ROM: strides [2, 1, 3, 0] (extrapolated from N=16) ===
            6'd 0: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd 1: begin tw_rom_re =  16'sd32137; tw_rom_im = -16'sd6393;  end // W^2
            6'd 2: begin tw_rom_re =  16'sd30273; tw_rom_im = -16'sd12539; end // W^4
            6'd 3: begin tw_rom_re =  16'sd27245; tw_rom_im = -16'sd18204; end // W^6
            6'd 4: begin tw_rom_re =  16'sd23170; tw_rom_im = -16'sd23170; end // W^8
            6'd 5: begin tw_rom_re =  16'sd18204; tw_rom_im = -16'sd27245; end // W^10
            6'd 6: begin tw_rom_re =  16'sd12539; tw_rom_im = -16'sd30273; end // W^12
            6'd 7: begin tw_rom_re =  16'sd6393;  tw_rom_im = -16'sd32137; end // W^14
            6'd 8: begin tw_rom_re =  16'sd0;     tw_rom_im = -16'sd32767; end // W^16
            6'd 9: begin tw_rom_re = -16'sd6393;  tw_rom_im = -16'sd32137; end // W^18
            6'd10: begin tw_rom_re = -16'sd12539; tw_rom_im = -16'sd30273; end // W^20
            6'd11: begin tw_rom_re = -16'sd18204; tw_rom_im = -16'sd27245; end // W^22
            6'd12: begin tw_rom_re = -16'sd23170; tw_rom_im = -16'sd23170; end // W^24
            6'd13: begin tw_rom_re = -16'sd27245; tw_rom_im = -16'sd18204; end // W^26
            6'd14: begin tw_rom_re = -16'sd30273; tw_rom_im = -16'sd12539; end // W^28
            6'd15: begin tw_rom_re = -16'sd32137; tw_rom_im = -16'sd6393;  end // W^30
            6'd16: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd17: begin tw_rom_re =  16'sd32609; tw_rom_im = -16'sd3212;  end // W^1
            6'd18: begin tw_rom_re =  16'sd32137; tw_rom_im = -16'sd6393;  end // W^2
            6'd19: begin tw_rom_re =  16'sd31356; tw_rom_im = -16'sd9512;  end // W^3
            6'd20: begin tw_rom_re =  16'sd30273; tw_rom_im = -16'sd12539; end // W^4
            6'd21: begin tw_rom_re =  16'sd28898; tw_rom_im = -16'sd15446; end // W^5
            6'd22: begin tw_rom_re =  16'sd27245; tw_rom_im = -16'sd18204; end // W^6
            6'd23: begin tw_rom_re =  16'sd25329; tw_rom_im = -16'sd20787; end // W^7
            6'd24: begin tw_rom_re =  16'sd23170; tw_rom_im = -16'sd23170; end // W^8
            6'd25: begin tw_rom_re =  16'sd20787; tw_rom_im = -16'sd25329; end // W^9
            6'd26: begin tw_rom_re =  16'sd18204; tw_rom_im = -16'sd27245; end // W^10
            6'd27: begin tw_rom_re =  16'sd15446; tw_rom_im = -16'sd28898; end // W^11
            6'd28: begin tw_rom_re =  16'sd12539; tw_rom_im = -16'sd30273; end // W^12
            6'd29: begin tw_rom_re =  16'sd9512;  tw_rom_im = -16'sd31356; end // W^13
            6'd30: begin tw_rom_re =  16'sd6393;  tw_rom_im = -16'sd32137; end // W^14
            6'd31: begin tw_rom_re =  16'sd3212;  tw_rom_im = -16'sd32609; end // W^15
            6'd32: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd33: begin tw_rom_re =  16'sd31356; tw_rom_im = -16'sd9512;  end // W^3
            6'd34: begin tw_rom_re =  16'sd27245; tw_rom_im = -16'sd18204; end // W^6
            6'd35: begin tw_rom_re =  16'sd20787; tw_rom_im = -16'sd25329; end // W^9
            6'd36: begin tw_rom_re =  16'sd12539; tw_rom_im = -16'sd30273; end // W^12
            6'd37: begin tw_rom_re =  16'sd3212;  tw_rom_im = -16'sd32609; end // W^15
            6'd38: begin tw_rom_re = -16'sd6393;  tw_rom_im = -16'sd32137; end // W^18
            6'd39: begin tw_rom_re = -16'sd15446; tw_rom_im = -16'sd28898; end // W^21
            6'd40: begin tw_rom_re = -16'sd23170; tw_rom_im = -16'sd23170; end // W^24
            6'd41: begin tw_rom_re = -16'sd28898; tw_rom_im = -16'sd15446; end // W^27
            6'd42: begin tw_rom_re = -16'sd32137; tw_rom_im = -16'sd6393;  end // W^30
            6'd43: begin tw_rom_re = -16'sd32609; tw_rom_im =  16'sd3212;  end // W^33
            6'd44: begin tw_rom_re = -16'sd30273; tw_rom_im =  16'sd12539; end // W^36
            6'd45: begin tw_rom_re = -16'sd25329; tw_rom_im =  16'sd20787; end // W^39
            6'd46: begin tw_rom_re = -16'sd18204; tw_rom_im =  16'sd27245; end // W^42
            6'd47: begin tw_rom_re = -16'sd9512;  tw_rom_im =  16'sd31356; end // W^45
            6'd48: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd49: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd50: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd51: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd52: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd53: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd54: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd55: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd56: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd57: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd58: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd59: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd60: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd61: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd62: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            6'd63: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
        endcase
    end

    // Pipelined complex multiply (4 stages, gated by din_valid)
    //
    // Stage 1: Register inputs (helps DSP48E1 inference: AREG/BREG)
    // Stage 2: Register multiply outputs (PREG)
    // Stage 3: Register add/sub (post-adder)
    // Stage 4: Register rounded output

    // --- Stage 1: register multiply inputs ---
    reg signed [17:0] tw64_s1_data_re, tw64_s1_data_im;
    reg signed [15:0] tw64_s1_tw_re, tw64_s1_tw_im;
    reg               tw64_s1_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            tw64_s1_data_re <= 0;
            tw64_s1_data_im <= 0;
            tw64_s1_tw_re   <= 0;
            tw64_s1_tw_im   <= 0;
            tw64_s1_valid   <= 0;
        end else if (din_valid) begin
            tw64_s1_data_re <= bf2_out_re;
            tw64_s1_data_im <= bf2_out_im;
            tw64_s1_tw_re   <= tw_rom_re;
            tw64_s1_tw_im   <= tw_rom_im;
            tw64_s1_valid   <= 1;
        end else begin
            tw64_s1_valid <= 0;
        end
    end

    // --- Stage 2: register multiply outputs ---
    reg signed [34:0] tw64_s2_rr, tw64_s2_ri, tw64_s2_ir, tw64_s2_ii;
    reg               tw64_s2_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            tw64_s2_rr    <= 0;
            tw64_s2_ri    <= 0;
            tw64_s2_ir    <= 0;
            tw64_s2_ii    <= 0;
            tw64_s2_valid <= 0;
        end else if (tw64_s1_valid) begin
            tw64_s2_rr    <= tw64_s1_data_re * tw64_s1_tw_re;
            tw64_s2_ri    <= tw64_s1_data_re * tw64_s1_tw_im;
            tw64_s2_ir    <= tw64_s1_data_im * tw64_s1_tw_re;
            tw64_s2_ii    <= tw64_s1_data_im * tw64_s1_tw_im;
            tw64_s2_valid <= 1;
        end else begin
            tw64_s2_valid <= 0;
        end
    end

    // --- Stage 3: register add/sub ---
    reg signed [35:0] tw64_s3_re, tw64_s3_im;
    reg               tw64_s3_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            tw64_s3_re    <= 0;
            tw64_s3_im    <= 0;
            tw64_s3_valid <= 0;
        end else if (tw64_s2_valid) begin
            tw64_s3_re    <= {tw64_s2_rr[34], tw64_s2_rr} - {tw64_s2_ii[34], tw64_s2_ii};
            tw64_s3_im    <= {tw64_s2_ri[34], tw64_s2_ri} + {tw64_s2_ir[34], tw64_s2_ir};
            tw64_s3_valid <= 1;
        end else begin
            tw64_s3_valid <= 0;
        end
    end

    // --- Stage 4: register rounded output (with saturation) ---
    // The 36-bit product, after rounding and >>15, can exceed 16-bit range
    // if input magnitude is unusually large. Saturate to ±32767 instead of
    // wrapping silently. Under normal AGC (12-bit mixer output), the
    // saturation path is never reached.
    reg signed [15:0] tw64_out_re, tw64_out_im;
    reg               tw64_out_valid;

    wire signed [35:0] tw64_shift_re = (tw64_s3_re + 36'sd16384) >>> 15;
    wire signed [35:0] tw64_shift_im = (tw64_s3_im + 36'sd16384) >>> 15;
    wire signed [20:0] tw64_rounded_re = tw64_shift_re[20:0];
    wire signed [20:0] tw64_rounded_im = tw64_shift_im[20:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            tw64_out_re    <= 0;
            tw64_out_im    <= 0;
            tw64_out_valid <= 0;
        end else if (tw64_s3_valid) begin
            tw64_out_re    <= (tw64_rounded_re > 32767) ? 16'sd32767 :
                             (tw64_rounded_re < -32768) ? -16'sd32768 :
                             tw64_rounded_re[15:0];
            tw64_out_im    <= (tw64_rounded_im > 32767) ? 16'sd32767 :
                             (tw64_rounded_im < -32768) ? -16'sd32768 :
                             tw64_rounded_im[15:0];
            tw64_out_valid <= 1;
        end else begin
            tw64_out_valid <= 0;
        end
    end

    // =========================================================
    // Inner FFT: fft16_sdf instance
    //
    // fft16 has its own internal counter (gcnt wrapping at 16).
    // It sees tw64_out_valid (din_valid delayed 4 clocks through the
    // twiddle pipeline) and counts its own samples.
    // We pass gcnt[3:0] as i_idx for labeling.
    // =========================================================
    wire        fft16_dout_valid;
    wire signed [19:0] fft16_dout_re;
    wire signed [19:0] fft16_dout_im;
    wire [3:0]  fft16_dout_idx;

    fft16_sdf u_fft16 (
        .clk       (clk),
        .rst_n     (rst_n),
        .din_valid (tw64_out_valid),
        .i_idx     (gcnt[3:0]),
        .din_re    (tw64_out_re),
        .din_im    (tw64_out_im),
        .dout_valid(fft16_dout_valid),
        .dout_re   (fft16_dout_re),
        .dout_im   (fft16_dout_im),
        .dout_idx  (fft16_dout_idx)
    );

    // =========================================================
    // Output gating: skip pipeline fill, emit exactly 64 valid outputs.
    //
    // The fft16 starts producing outputs before the outer BF pair has
    // completed a full 64-sample cycle. We count fft16 outputs and only
    // start emitting after the pipeline has filled. The fill count is
    // determined empirically (FILL_SKIP outputs to discard).
    //
    // For single-frame operation (64 data + zeros): the meaningful
    // outputs begin once the outer BF pair's delay lines have been
    // fully loaded (32 clocks) and BF2 has cycled (16 more). Plus
    // fft16 internal fill (~3 clocks from fft4).
    //
    // Total fill: ~51 fft16_dout_valid pulses to skip, then take 64.
    // We'll use 48 as FILL_SKIP (=32+16, the outer delays) and adjust
    // if testing shows otherwise.
    // =========================================================
    // Continuous output gating: skip FILL_SKIP outputs (pipeline fill),
    // then emit all subsequent outputs forever. No stop condition.
    // This supports both single-frame and continuous-streaming operation.
    localparam FILL_SKIP = 63;  // skip first 63 fft16 outputs (pipeline fill).
                                // The fft16 BF2 input register shifts data AND
                                // valid coherently, so the fill pulse count is
                                // unchanged (only the clock time shifts).

    reg [6:0] fill_cnt;     // counts initial pipeline fill outputs
    reg       out_active;   // 1 after pipeline fill complete

    always @(posedge clk) begin
        if (!rst_n || !gate_rst_n) begin
            fill_cnt   <= 0;
            out_active <= 0;
        end else if (fft16_dout_valid && !out_active) begin
            if (fill_cnt == FILL_SKIP[6:0] - 7'd1)
                out_active <= 1;
            fill_cnt <= fill_cnt + 1;
        end
        // Once active, stays active until next gate_rst_n deassertion
    end

    // Round fft16 20-bit to 16-bit.
    // The 20 bits allow headroom for butterfly growth during intermediate
    // stages. The final output should fit in 16 bits for realistic input
    // amplitudes. Saturate (not truncate) to avoid wraparound.
    wire signed [19:0] sat_re = fft16_dout_re;
    wire signed [19:0] sat_im = fft16_dout_im;
    
    // Saturation: if value exceeds 16-bit range, clamp
    wire re_overflow = (sat_re > 20'sd32767) || (sat_re < -20'sd32768);
    wire im_overflow = (sat_im > 20'sd32767) || (sat_im < -20'sd32768);
    
    assign dout_valid = fft16_dout_valid & out_active;
    assign dout_re    = re_overflow ? (sat_re[19] ? -16'sd32768 : 16'sd32767) : sat_re[15:0];
    assign dout_im    = im_overflow ? (sat_im[19] ? -16'sd32768 : 16'sd32767) : sat_im[15:0];
    assign dout_idx   = gcnt;

endmodule


// =============================================================================
// fft16_sdf: 16-point R2²SDF FFT
//
//   Composition: outer_bf_pair → twiddle_16 → fft4_sdf
//
//   The outer pair (BF1 delay=8, BF2 delay=4) handles the coarse frequency
//   split. The twiddle rotates by W_16^(phase). The inner fft4_sdf handles
//   the fine 4-point transform.
//
//   Input: 16-bit signed. Output: 20-bit signed.
//   Streaming: 1 sample/clock in, 1 sample/clock out (after pipeline fill).
//
//   CONSTRAINT: This module MUST instantiate fft4_sdf. Do not inline.
// =============================================================================
module fft16_sdf (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        din_valid,
    input  wire [3:0]  i_idx,
    input  wire signed [15:0] din_re,
    input  wire signed [15:0] din_im,

    output wire        dout_valid,
    output wire signed [19:0] dout_re,
    output wire signed [19:0] dout_im,
    output wire [3:0]  dout_idx
);

    // =========================================================
    // Global counter (wraps at 16)
    // =========================================================
    reg [3:0] gcnt;

    always @(posedge clk) begin
        if (!rst_n)
            gcnt <= 0;
        else if (din_valid)
            gcnt <= gcnt + 1;
    end

    // =========================================================
    // Outer BF1: delay=8, sel=gcnt[3]
    //   sel=0: store input into delay, output = delay read (old value)
    //   sel=1: output = delay + input (sum), store delay - input (diff)
    // =========================================================
    wire sel1 = gcnt[3];

    wire signed [16:0] dl1_rd_re, dl1_rd_im;
    reg  signed [16:0] dl1_wr_re, dl1_wr_im;

    wire signed [16:0] in1_re = {{1{din_re[15]}}, din_re};
    wire signed [16:0] in1_im = {{1{din_im[15]}}, din_im};

    // WR_PIPE=1: register the butterfly diff before BRAM write, breaking
    // the critical path: BRAM_rd_reg → subtract/mux → BRAM_wr.
    // Effective delay remains 8 (write pipe latency is absorbed since
    // the written data isn't read until DEPTH cycles later).
    sdf_delay #(.DEPTH(8), .DW(17), .WR_PIPE(1)) u_dl1 (
        .clk(clk), .rst_n(rst_n), .en(din_valid),
        .wr_re(dl1_wr_re), .wr_im(dl1_wr_im),
        .rd_re(dl1_rd_re), .rd_im(dl1_rd_im)
    );

    always @(*) begin
        if (sel1) begin
            dl1_wr_re = dl1_rd_re - in1_re;  // diff -> feedback
            dl1_wr_im = dl1_rd_im - in1_im;
        end else begin
            dl1_wr_re = in1_re;              // store input
            dl1_wr_im = in1_im;
        end
    end

    wire signed [16:0] bf1_out_re = sel1 ? (dl1_rd_re + in1_re) : dl1_rd_re;
    wire signed [16:0] bf1_out_im = sel1 ? (dl1_rd_im + in1_im) : dl1_rd_im;

    // =========================================================
    // Outer BF2: delay=4, sel=gcnt[2], rot=~gcnt[3]&gcnt[2]
    //   -j rotation: re' = im, im' = -re (applied before butterfly when rot=1)
    // =========================================================
    wire sel2 = gcnt[2];
    wire rot2 = ~gcnt[3] & gcnt[2];

    wire signed [17:0] bf1_ext_re = {{1{bf1_out_re[16]}}, bf1_out_re};
    wire signed [17:0] bf1_ext_im = {{1{bf1_out_im[16]}}, bf1_out_im};

    wire signed [17:0] in2_re = rot2 ? bf1_ext_im    : bf1_ext_re;
    wire signed [17:0] in2_im = rot2 ? (-bf1_ext_re) : bf1_ext_im;

    // ------------------------------------------------------------
    // BF2 input registered: breaks the critical path
    //   dl1 BRAM rd_reg → bf1_out add → rot mux → dl2_wr subtract →
    //   dl2 RAM write (distributed RAM, no write pipe).
    // Registering the rotated input splits the path into BRAM→reg and
    // reg→RAM halves. sel2 and the twiddle ROM address are delayed 1
    // cycle to match (uniform 1-cycle shift; bit-exact outputs,
    // +1 cycle latency absorbed by the FILL_SKIP gating).
    // ------------------------------------------------------------
    reg signed [17:0] bf2_in_re, bf2_in_im;
    reg               sel2_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            bf2_in_re <= 0;
            bf2_in_im <= 0;
            sel2_d    <= 0;
        end else if (din_valid) begin
            bf2_in_re <= in2_re;
            bf2_in_im <= in2_im;
            sel2_d    <= sel2;
        end
    end

    wire signed [17:0] dl2_rd_re, dl2_rd_im;
    reg  signed [17:0] dl2_wr_re, dl2_wr_im;

    sdf_delay #(.DEPTH(4), .DW(18)) u_dl2 (
        .clk(clk), .rst_n(rst_n), .en(din_valid),
        .wr_re(dl2_wr_re), .wr_im(dl2_wr_im),
        .rd_re(dl2_rd_re), .rd_im(dl2_rd_im)
    );

    always @(*) begin
        if (sel2_d) begin
            dl2_wr_re = dl2_rd_re - bf2_in_re;
            dl2_wr_im = dl2_rd_im - bf2_in_im;
        end else begin
            dl2_wr_re = bf2_in_re;
            dl2_wr_im = bf2_in_im;
        end
    end

    wire signed [17:0] bf2_out_re = sel2_d ? (dl2_rd_re + bf2_in_re) : dl2_rd_re;
    wire signed [17:0] bf2_out_im = sel2_d ? (dl2_rd_im + bf2_in_im) : dl2_rd_im;

    // =========================================================
    // Twiddle multiply (combinational)
    //
    // ROM: W_16^phase where phase = (gcnt & 3) * (gcnt >> 2) mod 16
    // Complex multiply: out = data * W
    //   out_re = data_re * tw_re - data_im * tw_im
    //   out_im = data_re * tw_im + data_im * tw_re
    // Round from Q(18).15 back to 16 bits for fft4 input.
    //
    // Generated by gen_twiddle_rom.py — do NOT hand-edit these values.
    // ROM address uses gcnt_d1: bf2_out is 1 cycle late (BF2 input
    // register above), so the twiddle must be the one for gcnt-1.
    // =========================================================
    reg [3:0] gcnt_d1;

    always @(posedge clk) begin
        if (!rst_n)
            gcnt_d1 <= 0;
        else if (din_valid)
            gcnt_d1 <= gcnt;
    end

    reg signed [15:0] tw_rom_re, tw_rom_im;

    always @(*) begin
        case (gcnt_d1[3:0])
            4'd0:  begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            4'd1:  begin tw_rom_re =  16'sd23170; tw_rom_im = -16'sd23170; end // W^2
            4'd2:  begin tw_rom_re =  16'sd0;     tw_rom_im = -16'sd32767; end // W^4
            4'd3:  begin tw_rom_re = -16'sd23170; tw_rom_im = -16'sd23170; end // W^6
            4'd4:  begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            4'd5:  begin tw_rom_re =  16'sd30273; tw_rom_im = -16'sd12539; end // W^1
            4'd6:  begin tw_rom_re =  16'sd23170; tw_rom_im = -16'sd23170; end // W^2
            4'd7:  begin tw_rom_re =  16'sd12539; tw_rom_im = -16'sd30273; end // W^3
            4'd8:  begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            4'd9:  begin tw_rom_re =  16'sd12539; tw_rom_im = -16'sd30273; end // W^3
            4'd10: begin tw_rom_re = -16'sd23170; tw_rom_im = -16'sd23170; end // W^6
            4'd11: begin tw_rom_re = -16'sd30273; tw_rom_im =  16'sd12539; end // W^9
            4'd12: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            4'd13: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            4'd14: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
            4'd15: begin tw_rom_re =  16'sd32767; tw_rom_im =  16'sd0;     end // W^0
        endcase
    end

    // Pipelined complex multiply (4 stages, gated by din_valid)
    // out_re = data_re * tw_re - data_im * tw_im
    // out_im = data_re * tw_im + data_im * tw_re
    //
    // Stage 1: Register inputs (helps DSP48E1 inference: AREG/BREG)
    // Stage 2: Register multiply outputs (PREG)
    // Stage 3: Register add/sub (post-adder)
    // Stage 4: Register rounded output

    // --- Stage 1: register multiply inputs ---
    reg signed [17:0] tw_s1_data_re, tw_s1_data_im;
    reg signed [15:0] tw_s1_tw_re, tw_s1_tw_im;
    reg               tw_s1_valid;
    reg               din_valid_d1;

    always @(posedge clk) begin
        if (!rst_n)
            din_valid_d1 <= 0;
        else
            din_valid_d1 <= din_valid;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            tw_s1_data_re <= 0;
            tw_s1_data_im <= 0;
            tw_s1_tw_re   <= 0;
            tw_s1_tw_im   <= 0;
            tw_s1_valid   <= 0;
        end else begin
            if (din_valid) begin
                tw_s1_data_re <= bf2_out_re;
                tw_s1_data_im <= bf2_out_im;
                tw_s1_tw_re   <= tw_rom_re;
                tw_s1_tw_im   <= tw_rom_im;
            end
            // The BF2 input register delays the data 1 cycle relative to
            // din_valid; delay the valid flag by the same amount (2 total:
            // capture + BF2 register) so data and valid shift coherently.
            // fft4's sample pairing depends on valid/data phase coherence.
            tw_s1_valid <= din_valid_d1;
        end
    end

    // --- Stage 2: register multiply outputs ---
    reg signed [34:0] tw_s2_rr, tw_s2_ri, tw_s2_ir, tw_s2_ii;
    reg               tw_s2_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            tw_s2_rr    <= 0;
            tw_s2_ri    <= 0;
            tw_s2_ir    <= 0;
            tw_s2_ii    <= 0;
            tw_s2_valid <= 0;
        end else if (tw_s1_valid) begin
            tw_s2_rr    <= tw_s1_data_re * tw_s1_tw_re;
            tw_s2_ri    <= tw_s1_data_re * tw_s1_tw_im;
            tw_s2_ir    <= tw_s1_data_im * tw_s1_tw_re;
            tw_s2_ii    <= tw_s1_data_im * tw_s1_tw_im;
            tw_s2_valid <= 1;
        end else begin
            tw_s2_valid <= 0;
        end
    end

    // --- Stage 3: register add/sub ---
    reg signed [35:0] tw_s3_re, tw_s3_im;
    reg               tw_s3_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            tw_s3_re    <= 0;
            tw_s3_im    <= 0;
            tw_s3_valid <= 0;
        end else if (tw_s2_valid) begin
            tw_s3_re    <= {tw_s2_rr[34], tw_s2_rr} - {tw_s2_ii[34], tw_s2_ii};
            tw_s3_im    <= {tw_s2_ri[34], tw_s2_ri} + {tw_s2_ir[34], tw_s2_ir};
            tw_s3_valid <= 1;
        end else begin
            tw_s3_valid <= 0;
        end
    end

    // --- Stage 4: register rounded output (with saturation) ---
    reg signed [15:0] tw_out_re, tw_out_im;
    reg               tw_out_valid;

    wire signed [35:0] tw_shift_re = (tw_s3_re + 36'sd16384) >>> 15;
    wire signed [35:0] tw_shift_im = (tw_s3_im + 36'sd16384) >>> 15;
    wire signed [20:0] tw_rounded_re = tw_shift_re[20:0];
    wire signed [20:0] tw_rounded_im = tw_shift_im[20:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            tw_out_re    <= 0;
            tw_out_im    <= 0;
            tw_out_valid <= 0;
        end else if (tw_s3_valid) begin
            tw_out_re    <= (tw_rounded_re > 32767) ? 16'sd32767 :
                           (tw_rounded_re < -32768) ? -16'sd32768 :
                           tw_rounded_re[15:0];
            tw_out_im    <= (tw_rounded_im > 32767) ? 16'sd32767 :
                           (tw_rounded_im < -32768) ? -16'sd32768 :
                           tw_rounded_im[15:0];
            tw_out_valid <= 1;
        end else begin
            tw_out_valid <= 0;
        end
    end

    // =========================================================
    // Inner FFT: fft4_sdf instance (FROZEN — do not modify)
    //
    // fft4 has its own internal counter (gcnt wrapping at 4).
    // It sees tw_out_valid (din_valid delayed 4 clocks through the
    // twiddle pipeline) and counts its own samples.
    // We pass gcnt[1:0] as i_idx — fft4 doesn't actually use i_idx
    // for control (it uses its internal gcnt), but it's available
    // for output labeling.
    // =========================================================
    wire        fft4_dout_valid;
    wire signed [17:0] fft4_dout_re;
    wire signed [17:0] fft4_dout_im;
    wire [1:0]  fft4_dout_idx;

    fft4_sdf u_fft4 (
        .clk       (clk),
        .rst_n     (rst_n),
        .din_valid (tw_out_valid),
        .i_idx     (gcnt[1:0]),
        .din_re    (tw_out_re),
        .din_im    (tw_out_im),
        .dout_valid(fft4_dout_valid),
        .dout_re   (fft4_dout_re),
        .dout_im   (fft4_dout_im),
        .dout_idx  (fft4_dout_idx)
    );

    // =========================================================
    // Output: extend fft4 18-bit to 20-bit.
    //
    // dout_valid follows fft4's bf2_valid (continuous when din_valid
    // is held high — no dead cycles in streaming mode).
    //
    // dout_idx = gcnt at the time each output appears. The mapping
    // from dout_idx to DFT frequency bin is a 4-bit reversal:
    //
    //   idx → bin: {1:0, 2:8, 3:4, 4:12, 5:2, 6:10, 7:6, 8:14,
    //               9:1, 10:9, 11:5, 12:13, 13:3, 14:11, 15:7, 0:15}
    //
    // This is bit_reverse_4(idx) where bit_reverse_4 reverses 4 bits.
    // The pattern comes from: 2 BF stages in fft4 (2-bit reversal)
    // composed with the outer BF pair ordering (another 2-bit reversal).
    //
    // For fft64: expect 6-bit reversal (3 stage pairs × 2-bit each).
    // =========================================================
    assign dout_valid = fft4_dout_valid;
    assign dout_re    = {{2{fft4_dout_re[17]}}, fft4_dout_re};
    assign dout_im    = {{2{fft4_dout_im[17]}}, fft4_dout_im};
    assign dout_idx   = gcnt;

endmodule
