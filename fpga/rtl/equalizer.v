// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/equalizer.v — Frequency-domain equalizer for 802.11a OFDM
//
// Given sequential FFT bin outputs and H_inv from chan_est, computes:
//   eq[k] = Y[k] * H_inv[k] >> (shift_val - EQ_SCALE)
//
// EQ_SCALE=6 retains 6 extra magnitude bits so BPSK output is ±64
// (not ±1), giving the demapper real soft-decision dynamic range.
//
// Then extracts and outputs:
//   - 48 data subcarriers in LIB80211_DATA_BINS order
//   - 4 pilot subcarriers in order (bins 7, 21, 43, 57)
//
// Pipeline (6 registered stages):
//   Cycle 0: Latch FFT bin input, present rd_addr to chan_est BRAM
//   Cycle 1: H_inv arrives (1-cycle BRAM latency), latch
//   Cycle 2: Complex multiply (registered, DSP48)
//   Cycle 3: Add/sub of products (pipe2)
//   Cycle 4: Round-add (pipe3a — breaks the carry/shift combinational path)
//   Cycle 5: Variable shift + output if data/pilot bin (pipe3)
//
// The module accepts H_inv directly on input ports (chan_est drives these
// based on rd_addr). This allows flexible integration.
//
// Resource estimate: 4 DSP48E1 (complex multiply), ~300 LUTs (control + subcarrier ROM)

module equalizer (
    input  wire        clk,
    input  wire        rst_n,

    // Control
    input  wire        start,          // pulse: begin processing new symbol

    // Adaptive shift from chan_est (replaces fixed Q1.15 shift of 15)
    input  wire [4:0]  shift_val,

    // FFT bin input (sequential, 64 bins)
    input  wire        fft_valid,
    input  wire [5:0]  fft_bin,        // bin index 0-63
    input  wire signed [15:0] fft_re,
    input  wire signed [15:0] fft_im,

    // H_inv input (from chan_est BRAM, 1-cycle latency on rd_addr)
    input  wire signed [15:0] hinv_re,
    input  wire signed [15:0] hinv_im,

    // H_inv read address (drives chan_est rd_addr)
    output reg  [5:0]  rd_addr,

    // Data subcarrier output (48 per symbol)
    output reg         data_valid,
    output reg  signed [15:0] data_re,
    output reg  signed [15:0] data_im,
    output reg  [5:0]  data_idx,       // 0-47: which data subcarrier

    // Pilot output (4 per symbol)
    output reg         pilot_valid,
    output reg  signed [15:0] pilot_re,
    output reg  signed [15:0] pilot_im,
    output reg  [1:0]  pilot_idx,      // 0-3: which pilot

    // Status
    output reg         symbol_done     // pulse when all 48 data + 4 pilots output
);

    // =========================================================
    // Data subcarrier bin ROM (LIB80211_DATA_BINS order)
    // =========================================================
    // Negative freq first (bins 38-63 minus pilots), then positive (bins 1-26 minus pilots)
    reg [5:0] data_bin_rom [0:47];
    initial begin
        data_bin_rom[ 0] = 38; data_bin_rom[ 1] = 39; data_bin_rom[ 2] = 40;
        data_bin_rom[ 3] = 41; data_bin_rom[ 4] = 42; data_bin_rom[ 5] = 44;
        data_bin_rom[ 6] = 45; data_bin_rom[ 7] = 46; data_bin_rom[ 8] = 47;
        data_bin_rom[ 9] = 48; data_bin_rom[10] = 49; data_bin_rom[11] = 50;
        data_bin_rom[12] = 51; data_bin_rom[13] = 52; data_bin_rom[14] = 53;
        data_bin_rom[15] = 54; data_bin_rom[16] = 55; data_bin_rom[17] = 56;
        data_bin_rom[18] = 58; data_bin_rom[19] = 59; data_bin_rom[20] = 60;
        data_bin_rom[21] = 61; data_bin_rom[22] = 62; data_bin_rom[23] = 63;
        data_bin_rom[24] =  1; data_bin_rom[25] =  2; data_bin_rom[26] =  3;
        data_bin_rom[27] =  4; data_bin_rom[28] =  5; data_bin_rom[29] =  6;
        data_bin_rom[30] =  8; data_bin_rom[31] =  9; data_bin_rom[32] = 10;
        data_bin_rom[33] = 11; data_bin_rom[34] = 12; data_bin_rom[35] = 13;
        data_bin_rom[36] = 14; data_bin_rom[37] = 15; data_bin_rom[38] = 16;
        data_bin_rom[39] = 17; data_bin_rom[40] = 18; data_bin_rom[41] = 19;
        data_bin_rom[42] = 20; data_bin_rom[43] = 22; data_bin_rom[44] = 23;
        data_bin_rom[45] = 24; data_bin_rom[46] = 25; data_bin_rom[47] = 26;
    end

    // Pilot bins: 7, 21, 43, 57
    reg [5:0] pilot_bin_rom [0:3];
    initial begin
        pilot_bin_rom[0] = 7;
        pilot_bin_rom[1] = 21;
        pilot_bin_rom[2] = 43;
        pilot_bin_rom[3] = 57;
    end

    // =========================================================
    // FFT bin storage (capture all 64 bins first, then read via registered output)
    // The registered read eliminates the deep combinational 64:1 MUX path,
    // even though Vivado implements this as distributed RAM (not BRAM) under
    // Flow_AreaOptimized_medium. The LUT savings come from converting the
    // combinational MUX to LUT-RAM with registered output.
    // =========================================================
    reg [31:0] fft_buf [0:63];
    reg [31:0] fft_buf_rd;  // registered output
    reg [5:0] cap_count;

    // Write port signals
    reg        fft_wr_en;
    reg [5:0]  fft_wr_addr;
    reg [31:0] fft_wr_data;

    // Write (separate always block)
    always @(posedge clk) begin
        if (fft_wr_en)
            fft_buf[fft_wr_addr] <= fft_wr_data;
    end

    // Read port address
    reg [5:0] fft_rd_addr;

    // Synchronous read (1-cycle latency)
    always @(posedge clk) begin
        fft_buf_rd <= fft_buf[fft_rd_addr];
    end

    wire signed [15:0] fft_rd_re = $signed(fft_buf_rd[31:16]);
    wire signed [15:0] fft_rd_im = $signed(fft_buf_rd[15:0]);

    // =========================================================
    // FSM
    // =========================================================
    localparam S_IDLE    = 3'd0;
    localparam S_CAPTURE = 3'd1;
    localparam S_PRIME   = 3'd2;  // 1-cycle wait for BRAM latency
    localparam S_EQ_DATA = 3'd3;  // equalize + output data subcarriers
    localparam S_PIL_PRM = 3'd4;  // pilot prime (BRAM latency)
    localparam S_EQ_PILOT= 3'd5;  // equalize + output pilots
    localparam S_DONE    = 3'd6;

    reg [2:0] state;
    reg [5:0] out_idx;     // index into data_bin_rom (0-47) or pilot_bin_rom (0-3)

    // Complex multiply pipeline registers
    reg signed [15:0] pipe_y_re, pipe_y_im;
    reg signed [15:0] pipe_h_re, pipe_h_im;
    reg [5:0]  pipe_out_idx;
    reg        pipe_is_pilot;
    reg        pipe_valid;

    // Complex multiply result (Y * H_inv):
    // eq_re = y_re * h_re - y_im * h_im (all >> 15)
    // eq_im = y_re * h_im + y_im * h_re (all >> 15)
    //
    // Pipelined: Stage 1 = multiply (DSP48), Stage 2 = add/sub + round
    reg signed [31:0] reg_prod_rr, reg_prod_ii, reg_prod_ri, reg_prod_ir;
    reg pipe1_valid;
    reg [5:0] pipe1_out_idx;
    reg pipe1_is_pilot;

    always @(posedge clk) begin
        if (!rst_n) begin
            reg_prod_rr    <= 0;
            reg_prod_ii    <= 0;
            reg_prod_ri    <= 0;
            reg_prod_ir    <= 0;
            pipe1_valid    <= 0;
            pipe1_out_idx  <= 0;
            pipe1_is_pilot <= 0;
        end else begin
            reg_prod_rr    <= pipe_y_re * pipe_h_re;
            reg_prod_ii    <= pipe_y_im * pipe_h_im;
            reg_prod_ri    <= pipe_y_re * pipe_h_im;
            reg_prod_ir    <= pipe_y_im * pipe_h_re;
            pipe1_valid    <= pipe_valid;
            pipe1_out_idx  <= pipe_out_idx;
            pipe1_is_pilot <= pipe_is_pilot;
        end
    end

    wire signed [31:0] eq_full_re = reg_prod_rr - reg_prod_ii;
    wire signed [31:0] eq_full_im = reg_prod_ri + reg_prod_ir;

    // Pipeline stage 2→3: register the 32-bit add/sub result
    reg pipe2_valid;
    reg [5:0] pipe2_out_idx;
    reg pipe2_is_pilot;
    reg signed [31:0] pipe2_full_re, pipe2_full_im;

    // Pipeline stage 3→4: register the shifted/rounded 16-bit output
    // The variable shift (>>> shift_val) is applied here as a registered stage
    // to avoid a long combinational path (barrel shifter after adder tree).
    reg pipe3_valid;
    reg [5:0] pipe3_out_idx;
    reg pipe3_is_pilot;
    reg signed [15:0] pipe3_eq_re, pipe3_eq_im;

    // Pipeline stage 2.5: register the 32-bit round-add result so the
    // adder carry chain (10 CARRY4, ~9.3ns in the worst placement) and
    // the barrel shifter never sit in the same combinational path.
    // WLDrivenBlockPlacement landed this path at +0.075ns (2026-08-23
    // bisect build) and -0.196ns (2026-08-24, two identical placements)
    // — a structural cut, not a placement lottery fix.
    reg pipe3a_valid;
    reg [5:0] pipe3a_out_idx;
    reg pipe3a_is_pilot;
    reg signed [32:0] pipe3a_rounded_re, pipe3a_rounded_im;

    // Equalization scale: reduce right-shift by EQ_SCALE bits so the output
    // fills more of the 16-bit range. Instead of normalizing to integer ±1
    // (which leaves no noise margin), output ±2^EQ_SCALE = ±64 for BPSK.
    // 64-QAM: the demapper uses norm=10 (decode_engine rate table), so the
    // outermost point is ±7*10 = ±70 — still fits 16-bit signed. (The norm,
    // not EQ_SCALE, sets the QAM constellation scale; K_MOD is folded into
    // the demapper, not applied here.)
    localparam EQ_SCALE = 6;

    // Register shift_val locally to break the high-fanout combinational path
    // from chan_est to 56 equalizer loads (barrel-shifter CARRY chains).
    // shift_val is computed once per frame during LTF, so 1-cycle latency is
    // absorbed well before the first data symbol.
    reg [4:0] shift_val_r;
    always @(posedge clk) begin
        if (!rst_n)
            shift_val_r <= 5'd15;
        else
            shift_val_r <= shift_val;
    end
    wire [4:0] eff_shift = (shift_val_r > EQ_SCALE) ? (shift_val_r - EQ_SCALE) : 5'd0;

    // Register eff_shift and round_add to break the remaining combinational path
    // from shift_val_r through the barrel shifter to pipe3. These are constant
    // per frame, so 1-cycle latency is absorbed before the first data symbol.
    reg [4:0] eff_shift_r;
    reg signed [31:0] round_add_r;
    always @(posedge clk) begin
        if (!rst_n) begin
            eff_shift_r <= 5'd0;
            round_add_r <= 32'sd0;
        end else begin
            eff_shift_r <= eff_shift;
            round_add_r <= (eff_shift > 0) ? (32'sd1 <<< (eff_shift - 1)) : 32'sd0;
        end
    end

    // Shift + round with saturation (combinational, feeds into pipe3 register)
    // The full 32-bit result is shifted, then saturated to 16-bit signed.
    // Without saturation, channel estimation errors on deep fades can produce
    // values that wrap silently — the most dangerous point in the pipeline.
    // The round add is registered first (pipe3a) to cut the carry chain from
    // the barrel shifter; widened to 33-bit to prevent overflow when
    // pipe2_full is near INT32_MAX and eff_shift >= 17.
    wire signed [32:0] rounded_re = {pipe2_full_re[31], pipe2_full_re} + {round_add_r[31], round_add_r};
    wire signed [32:0] rounded_im = {pipe2_full_im[31], pipe2_full_im} + {round_add_r[31], round_add_r};

    wire signed [32:0] shifted_full_re = pipe3a_rounded_re >>> eff_shift_r;
    wire signed [32:0] shifted_full_im = pipe3a_rounded_im >>> eff_shift_r;

    wire re_sat_ovf = (shifted_full_re > 33'sd32767) || (shifted_full_re < -33'sd32768);
    wire im_sat_ovf = (shifted_full_im > 33'sd32767) || (shifted_full_im < -33'sd32768);

    wire signed [15:0] shifted_re = re_sat_ovf ?
        (shifted_full_re[31] ? -16'sd32768 : 16'sd32767) : shifted_full_re[15:0];
    wire signed [15:0] shifted_im = im_sat_ovf ?
        (shifted_full_im[31] ? -16'sd32768 : 16'sd32767) : shifted_full_im[15:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            cap_count   <= 0;
            out_idx     <= 0;
            data_valid  <= 0;
            pilot_valid <= 0;
            symbol_done <= 0;
            pipe_valid  <= 0;
            pipe2_valid <= 0;
            pipe3a_valid <= 0;
            pipe3_valid <= 0;
            rd_addr     <= 0;
            fft_rd_addr <= 0;
        end else begin
            // Default: deassert outputs
            data_valid  <= 0;
            pilot_valid <= 0;
            symbol_done <= 0;

            // Default: BRAM write disabled
            fft_wr_en <= 0;

            // Pipeline stage 2→3: register add/sub result
            pipe2_valid    <= pipe1_valid;
            pipe2_out_idx  <= pipe1_out_idx;
            pipe2_is_pilot <= pipe1_is_pilot;
            pipe2_full_re  <= eq_full_re;
            pipe2_full_im  <= eq_full_im;

            // Pipeline stage 2.5: register round-add result (carry-chain cut)
            pipe3a_valid       <= pipe2_valid;
            pipe3a_out_idx     <= pipe2_out_idx;
            pipe3a_is_pilot    <= pipe2_is_pilot;
            pipe3a_rounded_re  <= rounded_re;
            pipe3a_rounded_im  <= rounded_im;

            // Pipeline stage 3→4: register shifted result
            pipe3_valid    <= pipe3a_valid;
            pipe3_out_idx  <= pipe3a_out_idx;
            pipe3_is_pilot <= pipe3a_is_pilot;
            pipe3_eq_re    <= shifted_re;
            pipe3_eq_im    <= shifted_im;

            // Pipeline stage 4→output
            if (pipe3_valid) begin
                if (pipe3_is_pilot) begin
                    pilot_valid <= 1;
                    pilot_re    <= pipe3_eq_re;
                    pilot_im    <= pipe3_eq_im;
                    pilot_idx   <= pipe3_out_idx[1:0];
                end else begin
                    data_valid <= 1;
                    data_re    <= pipe3_eq_re;
                    data_im    <= pipe3_eq_im;
                    data_idx   <= pipe3_out_idx;
                end
            end

            case (state)
                S_IDLE: begin
                    pipe_valid <= 0;
                    if (start) begin
                        state     <= S_CAPTURE;
                        cap_count <= 0;
                    end
                end

                S_CAPTURE: begin
                    if (fft_valid) begin
                        fft_wr_en   <= 1;
                        fft_wr_addr <= fft_bin;
                        fft_wr_data <= {fft_re, fft_im};
                        cap_count <= cap_count + 1;
                        if (cap_count == 63) begin
                            state     <= S_PRIME;
                            out_idx   <= 0;
                            pipe_valid <= 0;
                            // Request H_inv for first data bin
                            rd_addr <= data_bin_rom[0];
                            // Present BRAM read address for first data bin
                            fft_rd_addr <= data_bin_rom[0];
                        end
                    end
                end

                S_PRIME: begin
                    // Two-cycle prime for BRAM latency:
                    // Cycle 1 (this): rd_addr = data_bin_rom[0] was set last clock.
                    //   Chan_est BRAM samples it and schedules output.
                    //   fft_buf BRAM reads data_bin_rom[0] (set last clock).
                    //   We request data_bin_rom[1] for the next bin (both H_inv and fft_buf).
                    // Cycle 2 (S_EQ_DATA first iteration): hinv = H_inv[data_bin_rom[0]]
                    //   available to read. fft_buf_rd = fft_buf[data_bin_rom[0]] available.
                    //   We process bin[0] and request bin[2].
                    rd_addr     <= data_bin_rom[1];
                    fft_rd_addr <= data_bin_rom[1];
                    state       <= S_EQ_DATA;
                end

                S_EQ_DATA: begin
                    // Each cycle: consume hinv (arrives 2 edges after rd_addr set),
                    // consume fft_buf (arrives 2 edges after fft_rd_addr set),
                    // process current subcarrier, request out_idx+2 (2-cycle lookahead).
                    pipe_y_re    <= fft_rd_re;
                    pipe_y_im    <= fft_rd_im;
                    pipe_h_re    <= hinv_re;
                    pipe_h_im    <= hinv_im;
                    pipe_out_idx <= out_idx;
                    pipe_is_pilot <= 0;
                    pipe_valid   <= 1;

                    if (out_idx < 46) begin
                        rd_addr     <= data_bin_rom[out_idx + 2];
                        fft_rd_addr <= data_bin_rom[out_idx + 2];
                        out_idx     <= out_idx + 1;
                    end else if (out_idx == 46) begin
                        // Next-to-last: request last data bin
                        rd_addr     <= data_bin_rom[47];
                        fft_rd_addr <= data_bin_rom[47];
                        out_idx     <= out_idx + 1;
                    end else begin
                        // Last data subcarrier (out_idx=47). Request first pilot.
                        state       <= S_PIL_PRM;
                        out_idx     <= 0;
                        rd_addr     <= pilot_bin_rom[0];
                        fft_rd_addr <= pilot_bin_rom[0];
                    end
                end

                S_PIL_PRM: begin
                    // 1-cycle BRAM latency wait for pilot_bin_rom[0].
                    // Both chan_est and fft_buf BRAMs need 1 cycle.
                    // Correct pipeline:
                    //   Last S_EQ_DATA set rd_addr/fft_rd_addr = pilot_bin_rom[0] → consumed at out_idx=0
                    //   S_PIL_PRM sets rd_addr/fft_rd_addr = pilot_bin_rom[1] → consumed at out_idx=1
                    //   S_EQ_PILOT out_idx=0 sets rd_addr/fft_rd_addr = pilot_bin_rom[2] → consumed at out_idx=2
                    //   S_EQ_PILOT out_idx=1 sets rd_addr/fft_rd_addr = pilot_bin_rom[3] → consumed at out_idx=3
                    pipe_valid  <= 0;
                    rd_addr     <= pilot_bin_rom[1];
                    fft_rd_addr <= pilot_bin_rom[1];
                    state       <= S_EQ_PILOT;
                end

                S_EQ_PILOT: begin
                    pipe_y_re    <= fft_rd_re;
                    pipe_y_im    <= fft_rd_im;
                    pipe_h_re    <= hinv_re;
                    pipe_h_im    <= hinv_im;
                    pipe_out_idx <= {4'b0, out_idx[1:0]};
                    pipe_is_pilot <= 1;
                    pipe_valid   <= 1;

                    if (out_idx < 2) begin
                        rd_addr     <= pilot_bin_rom[out_idx + 2];
                        fft_rd_addr <= pilot_bin_rom[out_idx + 2];
                        out_idx     <= out_idx + 1;
                    end else if (out_idx == 2) begin
                        rd_addr     <= pilot_bin_rom[3];
                        fft_rd_addr <= pilot_bin_rom[3];
                        out_idx     <= out_idx + 1;
                    end else begin
                        state <= S_DONE;
                        // pipe_valid stays 1 for this last pilot entry
                    end
                end

                S_DONE: begin
                    pipe_valid <= 0;
                    // Wait for pipeline to drain (5 stages:
                    // pipe→pipe1→pipe2→pipe3a→pipe3→output). Every stage
                    // must be empty before symbol_done — the invariant is
                    // "symbol_done ⇒ all 48 data + 4 pilots emitted"
                    // (decode_engine gates eq_rd_sel on this).
                    if (!pipe3_valid && !pipe3a_valid && !pipe2_valid &&
                        !pipe1_valid && !pipe_valid) begin
                        symbol_done <= 1;
                        state <= S_IDLE;
                    end
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
            $error("[equalizer] ILLEGAL STATE: reached unreachable state 7");
    end
    `endif

endmodule
