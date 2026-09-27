// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/deinterleaver.v — 802.11a bit deinterleaver (pair emit)
//
// Permutes soft bits from interleaved order back to coded order using
// pre-computed ROM lookup tables.
//
// Architecture (lever C): capture-then-emit, wide-word transport.
//   CAPTURE: one 48-bit word per subcarrier, 1/clk, addresses 0..47.
//   EMIT:    2 LLR/clk for ALL rates, as before. The ROM entry enc packs the
//            arrival position perm[k] as (subcarrier<<3)|lane, so the gather
//            reads sbuf[enc[8:3]] and muxes byte enc[2:0].
//
// Permutation semantics: out[k] = in[perm[k]]; ROM stores enc(perm[k]).
// Encoded by scripts/gen_deint_enc.py; do not hand-edit the ROM.
// All 4 tables concatenated: 48 + 96 + 192 + 288 = 624 entries x 9 bits.
//
// Resource estimate: ~50 LUTs (control) + lane mux, 2 BRAM18 (sbuf 48x48,
//                    width > 36), 1 BRAM18 (perm ROM, dual read).

module deinterleaver (
    input  wire        clk,
    input  wire        rst_n,

    // Configuration (latched at start of capture)
    input  wire [1:0]  rate_mode,      // 0=BPSK, 1=QPSK, 2=16QAM, 3=64QAM

    // Backpressure (lever 2a-prime): while asserted in S_EMIT, valid_out
    // stays high and emit counters/addresses freeze. Holding valid_out
    // high keeps the rx_pipeline deint_done falling-edge detector honest.
    input  wire        stall_in,

    // Input (lever C): one wide word per subcarrier, 1/clk for 48 clocks.
    // Lanes 0..n_bpsc-1 carry that subcarrier's LLRs; higher lanes are ignored.
    input  wire        valid_in,
    input  wire [7:0]  soft_in0,
    input  wire [7:0]  soft_in1,
    input  wire [7:0]  soft_in2,
    input  wire [7:0]  soft_in3,
    input  wire [7:0]  soft_in4,
    input  wire [7:0]  soft_in5,

    // Output: deinterleaved soft bits, 2 per clock on soft_out0/soft_out1
    output reg         valid_out,
    output reg  [7:0]  soft_out0,
    output reg  [7:0]  soft_out1,

    // Emit-complete pulse: falling edge of valid_out (fires after the
    // last pair leaves the output, i.e. S_TAIL release). Drives
    // decode_engine.deint_done on hardware (BD); rx_pipeline uses it in
    // place of its local detector so sim and silicon agree.
    output wire        deint_done
);

    reg        valid_out_d;

    always @(posedge clk) begin
        if (!rst_n)
            valid_out_d <= 0;
        else
            valid_out_d <= valid_out;
    end

    assign deint_done = valid_out_d & ~valid_out;

    // =========================================================
    // Soft word buffer (48 entries x 48 bits = one subcarrier per row).
    // Port A: capture write (1 word/clk) and emit read (sbuf[enc[8:3]]).
    // Port B: emit read. Capture and emit never overlap (capture-then-emit).
    // 48-bit width exceeds the RAMB18 36-bit port, so this infers 2 BRAM18.
    // =========================================================
    (* ram_style = "block" *) reg [47:0] sbuf [0:47];

    reg [5:0]  sbuf_a_addr;
    reg        sbuf_a_we;
    reg [47:0] sbuf_a_wdata;
    reg [47:0] sbuf_a_rd;
    reg [2:0]  sbuf_a_lane;     // lane of the address being presented
    reg [2:0]  sbuf_a_lane_rd;  // lane aligned with sbuf_a_rd

    reg [5:0]  sbuf_b_addr;
    reg [47:0] sbuf_b_rd;
    reg [2:0]  sbuf_b_lane;
    reg [2:0]  sbuf_b_lane_rd;

    // Selected byte of the wide read word (lane 0..5).
    wire [7:0] sbuf_a_byte = sbuf_a_rd[sbuf_a_lane_rd*8 +: 8];
    wire [7:0] sbuf_b_byte = sbuf_b_rd[sbuf_b_lane_rd*8 +: 8];

    // Emit-path reads freeze while stalled (lever 2a-prime): the in-flight
    // BRAM/ROM read registers must hold their values during an S_EMIT
    // stall, otherwise the queued reads complete mid-stall and the stream
    // loses/duplicates pairs on release.
    wire emit_frozen = stall_in & (state == S_EMIT);

    always @(posedge clk) begin
        if (sbuf_a_we)
            sbuf[sbuf_a_addr] <= sbuf_a_wdata;
        if (!emit_frozen) begin
            sbuf_a_rd      <= sbuf[sbuf_a_addr];
            sbuf_a_lane_rd <= sbuf_a_lane;
        end
    end

    always @(posedge clk) begin
        if (!emit_frozen) begin
            sbuf_b_rd      <= sbuf[sbuf_b_addr];
            sbuf_b_lane_rd <= sbuf_b_lane;
        end
    end

    reg [8:0] wr_count /* verilator public */;  // bits captured (0..N_CBPS)
    reg [5:0] wr_word;                          // wide words captured (0..48)

    // Bits per subcarrier. Only used to keep the public wr_count diagnostic
    // in bits; the emit bound is n_cbps and the transport is word-based.
    // Returns 9 bits to match wr_count (avoids WIDTHEXPAND).
    function [8:0] nbpsc;
        input [1:0] m;
        begin
            case (m)
                2'd0: nbpsc = 9'd1;
                2'd1: nbpsc = 9'd2;
                2'd2: nbpsc = 9'd4;
                default: nbpsc = 9'd6;
            endcase
        end
    endfunction

    // =========================================================
    // FSM states
    // =========================================================
    localparam S_IDLE    = 3'd0;
    localparam S_CAPTURE = 3'd1;
    localparam S_PRIME1  = 3'd2;  // ROM read perm[0], perm[1] in-flight
    localparam S_PRIME2  = 3'd3;  // perm_val=perm[0,1] -> issue sbuf reads; ROM reads perm[2,3]
    localparam S_PRIME3  = 3'd4;  // sbuf_rd=perm[0,1] ready; issue reads perm[2,3]; ROM reads perm[4,5]
    localparam S_EMIT    = 3'd5;  // steady state: output pair + advance
    localparam S_TAIL    = 3'd6;  // last pair held until downstream write accepted

    reg [2:0] state /* verilator public */;
    reg [8:0] emit_idx;   // ROM pair index (0,2,4,...)
    reg [8:0] out_count /* verilator public */;  // bits emitted so far
    reg [1:0] lat_mode;

    // =========================================================
    // N_CBPS lookup
    // =========================================================
    reg [8:0] n_cbps;
    always @(*) begin
        case (lat_mode)
            2'd0: n_cbps = 9'd48;
            2'd1: n_cbps = 9'd96;
            2'd2: n_cbps = 9'd192;
            2'd3: n_cbps = 9'd288;
        endcase
    end

    // =========================================================
    // Permutation ROM (BRAM, dual read ports)
    // =========================================================
    reg [9:0] rom_base;
    always @(*) begin
        case (lat_mode)
            2'd0: rom_base = 10'd0;
            2'd1: rom_base = 10'd48;
            2'd2: rom_base = 10'd144;
            2'd3: rom_base = 10'd336;
        endcase
    end

    wire [9:0] rom_addr0 = rom_base + {1'b0, emit_idx};
    wire [9:0] rom_addr1 = rom_addr0 + 10'd1;

    // Tail overshoot: on the final S_EMIT cycles of a 64-QAM symbol
    // (emit_idx = out_count + 6 runs to 292) rom_addr0/1 read past
    // perm_rom index 623. Those values land in perm_val after valid_out
    // deasserts and are never consumed: S_IDLE/S_CAPTURE never read
    // perm_val, and the next symbol's prime pipeline overwrites perm_val
    // with valid entries before S_PRIME2 uses it. Harmless by construction.

    (* ram_style = "block" *) reg [8:0] perm_rom [0:623];
    reg [8:0] enc_val0;
    reg [8:0] enc_val1;

    initial begin : perm_rom_init

        // (subcarrier << 3) | lane, generated by scripts/gen_deint_enc.py
        perm_rom[0]=9'd0; perm_rom[1]=9'd24; perm_rom[2]=9'd48; perm_rom[3]=9'd72; perm_rom[4]=9'd96; perm_rom[5]=9'd120; perm_rom[6]=9'd144; perm_rom[7]=9'd168;
        perm_rom[8]=9'd192; perm_rom[9]=9'd216; perm_rom[10]=9'd240; perm_rom[11]=9'd264; perm_rom[12]=9'd288; perm_rom[13]=9'd312; perm_rom[14]=9'd336; perm_rom[15]=9'd360;
        perm_rom[16]=9'd8; perm_rom[17]=9'd32; perm_rom[18]=9'd56; perm_rom[19]=9'd80; perm_rom[20]=9'd104; perm_rom[21]=9'd128; perm_rom[22]=9'd152; perm_rom[23]=9'd176;
        perm_rom[24]=9'd200; perm_rom[25]=9'd224; perm_rom[26]=9'd248; perm_rom[27]=9'd272; perm_rom[28]=9'd296; perm_rom[29]=9'd320; perm_rom[30]=9'd344; perm_rom[31]=9'd368;
        perm_rom[32]=9'd16; perm_rom[33]=9'd40; perm_rom[34]=9'd64; perm_rom[35]=9'd88; perm_rom[36]=9'd112; perm_rom[37]=9'd136; perm_rom[38]=9'd160; perm_rom[39]=9'd184;
        perm_rom[40]=9'd208; perm_rom[41]=9'd232; perm_rom[42]=9'd256; perm_rom[43]=9'd280; perm_rom[44]=9'd304; perm_rom[45]=9'd328; perm_rom[46]=9'd352; perm_rom[47]=9'd376;
        perm_rom[48]=9'd0; perm_rom[49]=9'd24; perm_rom[50]=9'd48; perm_rom[51]=9'd72; perm_rom[52]=9'd96; perm_rom[53]=9'd120; perm_rom[54]=9'd144; perm_rom[55]=9'd168;
        perm_rom[56]=9'd192; perm_rom[57]=9'd216; perm_rom[58]=9'd240; perm_rom[59]=9'd264; perm_rom[60]=9'd288; perm_rom[61]=9'd312; perm_rom[62]=9'd336; perm_rom[63]=9'd360;
        perm_rom[64]=9'd1; perm_rom[65]=9'd25; perm_rom[66]=9'd49; perm_rom[67]=9'd73; perm_rom[68]=9'd97; perm_rom[69]=9'd121; perm_rom[70]=9'd145; perm_rom[71]=9'd169;
        perm_rom[72]=9'd193; perm_rom[73]=9'd217; perm_rom[74]=9'd241; perm_rom[75]=9'd265; perm_rom[76]=9'd289; perm_rom[77]=9'd313; perm_rom[78]=9'd337; perm_rom[79]=9'd361;
        perm_rom[80]=9'd8; perm_rom[81]=9'd32; perm_rom[82]=9'd56; perm_rom[83]=9'd80; perm_rom[84]=9'd104; perm_rom[85]=9'd128; perm_rom[86]=9'd152; perm_rom[87]=9'd176;
        perm_rom[88]=9'd200; perm_rom[89]=9'd224; perm_rom[90]=9'd248; perm_rom[91]=9'd272; perm_rom[92]=9'd296; perm_rom[93]=9'd320; perm_rom[94]=9'd344; perm_rom[95]=9'd368;
        perm_rom[96]=9'd9; perm_rom[97]=9'd33; perm_rom[98]=9'd57; perm_rom[99]=9'd81; perm_rom[100]=9'd105; perm_rom[101]=9'd129; perm_rom[102]=9'd153; perm_rom[103]=9'd177;
        perm_rom[104]=9'd201; perm_rom[105]=9'd225; perm_rom[106]=9'd249; perm_rom[107]=9'd273; perm_rom[108]=9'd297; perm_rom[109]=9'd321; perm_rom[110]=9'd345; perm_rom[111]=9'd369;
        perm_rom[112]=9'd16; perm_rom[113]=9'd40; perm_rom[114]=9'd64; perm_rom[115]=9'd88; perm_rom[116]=9'd112; perm_rom[117]=9'd136; perm_rom[118]=9'd160; perm_rom[119]=9'd184;
        perm_rom[120]=9'd208; perm_rom[121]=9'd232; perm_rom[122]=9'd256; perm_rom[123]=9'd280; perm_rom[124]=9'd304; perm_rom[125]=9'd328; perm_rom[126]=9'd352; perm_rom[127]=9'd376;
        perm_rom[128]=9'd17; perm_rom[129]=9'd41; perm_rom[130]=9'd65; perm_rom[131]=9'd89; perm_rom[132]=9'd113; perm_rom[133]=9'd137; perm_rom[134]=9'd161; perm_rom[135]=9'd185;
        perm_rom[136]=9'd209; perm_rom[137]=9'd233; perm_rom[138]=9'd257; perm_rom[139]=9'd281; perm_rom[140]=9'd305; perm_rom[141]=9'd329; perm_rom[142]=9'd353; perm_rom[143]=9'd377;
        perm_rom[144]=9'd0; perm_rom[145]=9'd25; perm_rom[146]=9'd48; perm_rom[147]=9'd73; perm_rom[148]=9'd96; perm_rom[149]=9'd121; perm_rom[150]=9'd144; perm_rom[151]=9'd169;
        perm_rom[152]=9'd192; perm_rom[153]=9'd217; perm_rom[154]=9'd240; perm_rom[155]=9'd265; perm_rom[156]=9'd288; perm_rom[157]=9'd313; perm_rom[158]=9'd336; perm_rom[159]=9'd361;
        perm_rom[160]=9'd1; perm_rom[161]=9'd24; perm_rom[162]=9'd49; perm_rom[163]=9'd72; perm_rom[164]=9'd97; perm_rom[165]=9'd120; perm_rom[166]=9'd145; perm_rom[167]=9'd168;
        perm_rom[168]=9'd193; perm_rom[169]=9'd216; perm_rom[170]=9'd241; perm_rom[171]=9'd264; perm_rom[172]=9'd289; perm_rom[173]=9'd312; perm_rom[174]=9'd337; perm_rom[175]=9'd360;
        perm_rom[176]=9'd2; perm_rom[177]=9'd27; perm_rom[178]=9'd50; perm_rom[179]=9'd75; perm_rom[180]=9'd98; perm_rom[181]=9'd123; perm_rom[182]=9'd146; perm_rom[183]=9'd171;
        perm_rom[184]=9'd194; perm_rom[185]=9'd219; perm_rom[186]=9'd242; perm_rom[187]=9'd267; perm_rom[188]=9'd290; perm_rom[189]=9'd315; perm_rom[190]=9'd338; perm_rom[191]=9'd363;
        perm_rom[192]=9'd3; perm_rom[193]=9'd26; perm_rom[194]=9'd51; perm_rom[195]=9'd74; perm_rom[196]=9'd99; perm_rom[197]=9'd122; perm_rom[198]=9'd147; perm_rom[199]=9'd170;
        perm_rom[200]=9'd195; perm_rom[201]=9'd218; perm_rom[202]=9'd243; perm_rom[203]=9'd266; perm_rom[204]=9'd291; perm_rom[205]=9'd314; perm_rom[206]=9'd339; perm_rom[207]=9'd362;
        perm_rom[208]=9'd8; perm_rom[209]=9'd33; perm_rom[210]=9'd56; perm_rom[211]=9'd81; perm_rom[212]=9'd104; perm_rom[213]=9'd129; perm_rom[214]=9'd152; perm_rom[215]=9'd177;
        perm_rom[216]=9'd200; perm_rom[217]=9'd225; perm_rom[218]=9'd248; perm_rom[219]=9'd273; perm_rom[220]=9'd296; perm_rom[221]=9'd321; perm_rom[222]=9'd344; perm_rom[223]=9'd369;
        perm_rom[224]=9'd9; perm_rom[225]=9'd32; perm_rom[226]=9'd57; perm_rom[227]=9'd80; perm_rom[228]=9'd105; perm_rom[229]=9'd128; perm_rom[230]=9'd153; perm_rom[231]=9'd176;
        perm_rom[232]=9'd201; perm_rom[233]=9'd224; perm_rom[234]=9'd249; perm_rom[235]=9'd272; perm_rom[236]=9'd297; perm_rom[237]=9'd320; perm_rom[238]=9'd345; perm_rom[239]=9'd368;
        perm_rom[240]=9'd10; perm_rom[241]=9'd35; perm_rom[242]=9'd58; perm_rom[243]=9'd83; perm_rom[244]=9'd106; perm_rom[245]=9'd131; perm_rom[246]=9'd154; perm_rom[247]=9'd179;
        perm_rom[248]=9'd202; perm_rom[249]=9'd227; perm_rom[250]=9'd250; perm_rom[251]=9'd275; perm_rom[252]=9'd298; perm_rom[253]=9'd323; perm_rom[254]=9'd346; perm_rom[255]=9'd371;
        perm_rom[256]=9'd11; perm_rom[257]=9'd34; perm_rom[258]=9'd59; perm_rom[259]=9'd82; perm_rom[260]=9'd107; perm_rom[261]=9'd130; perm_rom[262]=9'd155; perm_rom[263]=9'd178;
        perm_rom[264]=9'd203; perm_rom[265]=9'd226; perm_rom[266]=9'd251; perm_rom[267]=9'd274; perm_rom[268]=9'd299; perm_rom[269]=9'd322; perm_rom[270]=9'd347; perm_rom[271]=9'd370;
        perm_rom[272]=9'd16; perm_rom[273]=9'd41; perm_rom[274]=9'd64; perm_rom[275]=9'd89; perm_rom[276]=9'd112; perm_rom[277]=9'd137; perm_rom[278]=9'd160; perm_rom[279]=9'd185;
        perm_rom[280]=9'd208; perm_rom[281]=9'd233; perm_rom[282]=9'd256; perm_rom[283]=9'd281; perm_rom[284]=9'd304; perm_rom[285]=9'd329; perm_rom[286]=9'd352; perm_rom[287]=9'd377;
        perm_rom[288]=9'd17; perm_rom[289]=9'd40; perm_rom[290]=9'd65; perm_rom[291]=9'd88; perm_rom[292]=9'd113; perm_rom[293]=9'd136; perm_rom[294]=9'd161; perm_rom[295]=9'd184;
        perm_rom[296]=9'd209; perm_rom[297]=9'd232; perm_rom[298]=9'd257; perm_rom[299]=9'd280; perm_rom[300]=9'd305; perm_rom[301]=9'd328; perm_rom[302]=9'd353; perm_rom[303]=9'd376;
        perm_rom[304]=9'd18; perm_rom[305]=9'd43; perm_rom[306]=9'd66; perm_rom[307]=9'd91; perm_rom[308]=9'd114; perm_rom[309]=9'd139; perm_rom[310]=9'd162; perm_rom[311]=9'd187;
        perm_rom[312]=9'd210; perm_rom[313]=9'd235; perm_rom[314]=9'd258; perm_rom[315]=9'd283; perm_rom[316]=9'd306; perm_rom[317]=9'd331; perm_rom[318]=9'd354; perm_rom[319]=9'd379;
        perm_rom[320]=9'd19; perm_rom[321]=9'd42; perm_rom[322]=9'd67; perm_rom[323]=9'd90; perm_rom[324]=9'd115; perm_rom[325]=9'd138; perm_rom[326]=9'd163; perm_rom[327]=9'd186;
        perm_rom[328]=9'd211; perm_rom[329]=9'd234; perm_rom[330]=9'd259; perm_rom[331]=9'd282; perm_rom[332]=9'd307; perm_rom[333]=9'd330; perm_rom[334]=9'd355; perm_rom[335]=9'd378;
        perm_rom[336]=9'd0; perm_rom[337]=9'd26; perm_rom[338]=9'd49; perm_rom[339]=9'd72; perm_rom[340]=9'd98; perm_rom[341]=9'd121; perm_rom[342]=9'd144; perm_rom[343]=9'd170;
        perm_rom[344]=9'd193; perm_rom[345]=9'd216; perm_rom[346]=9'd242; perm_rom[347]=9'd265; perm_rom[348]=9'd288; perm_rom[349]=9'd314; perm_rom[350]=9'd337; perm_rom[351]=9'd360;
        perm_rom[352]=9'd1; perm_rom[353]=9'd24; perm_rom[354]=9'd50; perm_rom[355]=9'd73; perm_rom[356]=9'd96; perm_rom[357]=9'd122; perm_rom[358]=9'd145; perm_rom[359]=9'd168;
        perm_rom[360]=9'd194; perm_rom[361]=9'd217; perm_rom[362]=9'd240; perm_rom[363]=9'd266; perm_rom[364]=9'd289; perm_rom[365]=9'd312; perm_rom[366]=9'd338; perm_rom[367]=9'd361;
        perm_rom[368]=9'd2; perm_rom[369]=9'd25; perm_rom[370]=9'd48; perm_rom[371]=9'd74; perm_rom[372]=9'd97; perm_rom[373]=9'd120; perm_rom[374]=9'd146; perm_rom[375]=9'd169;
        perm_rom[376]=9'd192; perm_rom[377]=9'd218; perm_rom[378]=9'd241; perm_rom[379]=9'd264; perm_rom[380]=9'd290; perm_rom[381]=9'd313; perm_rom[382]=9'd336; perm_rom[383]=9'd362;
        perm_rom[384]=9'd3; perm_rom[385]=9'd29; perm_rom[386]=9'd52; perm_rom[387]=9'd75; perm_rom[388]=9'd101; perm_rom[389]=9'd124; perm_rom[390]=9'd147; perm_rom[391]=9'd173;
        perm_rom[392]=9'd196; perm_rom[393]=9'd219; perm_rom[394]=9'd245; perm_rom[395]=9'd268; perm_rom[396]=9'd291; perm_rom[397]=9'd317; perm_rom[398]=9'd340; perm_rom[399]=9'd363;
        perm_rom[400]=9'd4; perm_rom[401]=9'd27; perm_rom[402]=9'd53; perm_rom[403]=9'd76; perm_rom[404]=9'd99; perm_rom[405]=9'd125; perm_rom[406]=9'd148; perm_rom[407]=9'd171;
        perm_rom[408]=9'd197; perm_rom[409]=9'd220; perm_rom[410]=9'd243; perm_rom[411]=9'd269; perm_rom[412]=9'd292; perm_rom[413]=9'd315; perm_rom[414]=9'd341; perm_rom[415]=9'd364;
        perm_rom[416]=9'd5; perm_rom[417]=9'd28; perm_rom[418]=9'd51; perm_rom[419]=9'd77; perm_rom[420]=9'd100; perm_rom[421]=9'd123; perm_rom[422]=9'd149; perm_rom[423]=9'd172;
        perm_rom[424]=9'd195; perm_rom[425]=9'd221; perm_rom[426]=9'd244; perm_rom[427]=9'd267; perm_rom[428]=9'd293; perm_rom[429]=9'd316; perm_rom[430]=9'd339; perm_rom[431]=9'd365;
        perm_rom[432]=9'd8; perm_rom[433]=9'd34; perm_rom[434]=9'd57; perm_rom[435]=9'd80; perm_rom[436]=9'd106; perm_rom[437]=9'd129; perm_rom[438]=9'd152; perm_rom[439]=9'd178;
        perm_rom[440]=9'd201; perm_rom[441]=9'd224; perm_rom[442]=9'd250; perm_rom[443]=9'd273; perm_rom[444]=9'd296; perm_rom[445]=9'd322; perm_rom[446]=9'd345; perm_rom[447]=9'd368;
        perm_rom[448]=9'd9; perm_rom[449]=9'd32; perm_rom[450]=9'd58; perm_rom[451]=9'd81; perm_rom[452]=9'd104; perm_rom[453]=9'd130; perm_rom[454]=9'd153; perm_rom[455]=9'd176;
        perm_rom[456]=9'd202; perm_rom[457]=9'd225; perm_rom[458]=9'd248; perm_rom[459]=9'd274; perm_rom[460]=9'd297; perm_rom[461]=9'd320; perm_rom[462]=9'd346; perm_rom[463]=9'd369;
        perm_rom[464]=9'd10; perm_rom[465]=9'd33; perm_rom[466]=9'd56; perm_rom[467]=9'd82; perm_rom[468]=9'd105; perm_rom[469]=9'd128; perm_rom[470]=9'd154; perm_rom[471]=9'd177;
        perm_rom[472]=9'd200; perm_rom[473]=9'd226; perm_rom[474]=9'd249; perm_rom[475]=9'd272; perm_rom[476]=9'd298; perm_rom[477]=9'd321; perm_rom[478]=9'd344; perm_rom[479]=9'd370;
        perm_rom[480]=9'd11; perm_rom[481]=9'd37; perm_rom[482]=9'd60; perm_rom[483]=9'd83; perm_rom[484]=9'd109; perm_rom[485]=9'd132; perm_rom[486]=9'd155; perm_rom[487]=9'd181;
        perm_rom[488]=9'd204; perm_rom[489]=9'd227; perm_rom[490]=9'd253; perm_rom[491]=9'd276; perm_rom[492]=9'd299; perm_rom[493]=9'd325; perm_rom[494]=9'd348; perm_rom[495]=9'd371;
        perm_rom[496]=9'd12; perm_rom[497]=9'd35; perm_rom[498]=9'd61; perm_rom[499]=9'd84; perm_rom[500]=9'd107; perm_rom[501]=9'd133; perm_rom[502]=9'd156; perm_rom[503]=9'd179;
        perm_rom[504]=9'd205; perm_rom[505]=9'd228; perm_rom[506]=9'd251; perm_rom[507]=9'd277; perm_rom[508]=9'd300; perm_rom[509]=9'd323; perm_rom[510]=9'd349; perm_rom[511]=9'd372;
        perm_rom[512]=9'd13; perm_rom[513]=9'd36; perm_rom[514]=9'd59; perm_rom[515]=9'd85; perm_rom[516]=9'd108; perm_rom[517]=9'd131; perm_rom[518]=9'd157; perm_rom[519]=9'd180;
        perm_rom[520]=9'd203; perm_rom[521]=9'd229; perm_rom[522]=9'd252; perm_rom[523]=9'd275; perm_rom[524]=9'd301; perm_rom[525]=9'd324; perm_rom[526]=9'd347; perm_rom[527]=9'd373;
        perm_rom[528]=9'd16; perm_rom[529]=9'd42; perm_rom[530]=9'd65; perm_rom[531]=9'd88; perm_rom[532]=9'd114; perm_rom[533]=9'd137; perm_rom[534]=9'd160; perm_rom[535]=9'd186;
        perm_rom[536]=9'd209; perm_rom[537]=9'd232; perm_rom[538]=9'd258; perm_rom[539]=9'd281; perm_rom[540]=9'd304; perm_rom[541]=9'd330; perm_rom[542]=9'd353; perm_rom[543]=9'd376;
        perm_rom[544]=9'd17; perm_rom[545]=9'd40; perm_rom[546]=9'd66; perm_rom[547]=9'd89; perm_rom[548]=9'd112; perm_rom[549]=9'd138; perm_rom[550]=9'd161; perm_rom[551]=9'd184;
        perm_rom[552]=9'd210; perm_rom[553]=9'd233; perm_rom[554]=9'd256; perm_rom[555]=9'd282; perm_rom[556]=9'd305; perm_rom[557]=9'd328; perm_rom[558]=9'd354; perm_rom[559]=9'd377;
        perm_rom[560]=9'd18; perm_rom[561]=9'd41; perm_rom[562]=9'd64; perm_rom[563]=9'd90; perm_rom[564]=9'd113; perm_rom[565]=9'd136; perm_rom[566]=9'd162; perm_rom[567]=9'd185;
        perm_rom[568]=9'd208; perm_rom[569]=9'd234; perm_rom[570]=9'd257; perm_rom[571]=9'd280; perm_rom[572]=9'd306; perm_rom[573]=9'd329; perm_rom[574]=9'd352; perm_rom[575]=9'd378;
        perm_rom[576]=9'd19; perm_rom[577]=9'd45; perm_rom[578]=9'd68; perm_rom[579]=9'd91; perm_rom[580]=9'd117; perm_rom[581]=9'd140; perm_rom[582]=9'd163; perm_rom[583]=9'd189;
        perm_rom[584]=9'd212; perm_rom[585]=9'd235; perm_rom[586]=9'd261; perm_rom[587]=9'd284; perm_rom[588]=9'd307; perm_rom[589]=9'd333; perm_rom[590]=9'd356; perm_rom[591]=9'd379;
        perm_rom[592]=9'd20; perm_rom[593]=9'd43; perm_rom[594]=9'd69; perm_rom[595]=9'd92; perm_rom[596]=9'd115; perm_rom[597]=9'd141; perm_rom[598]=9'd164; perm_rom[599]=9'd187;
        perm_rom[600]=9'd213; perm_rom[601]=9'd236; perm_rom[602]=9'd259; perm_rom[603]=9'd285; perm_rom[604]=9'd308; perm_rom[605]=9'd331; perm_rom[606]=9'd357; perm_rom[607]=9'd380;
        perm_rom[608]=9'd21; perm_rom[609]=9'd44; perm_rom[610]=9'd67; perm_rom[611]=9'd93; perm_rom[612]=9'd116; perm_rom[613]=9'd139; perm_rom[614]=9'd165; perm_rom[615]=9'd188;
        perm_rom[616]=9'd211; perm_rom[617]=9'd237; perm_rom[618]=9'd260; perm_rom[619]=9'd283; perm_rom[620]=9'd309; perm_rom[621]=9'd332; perm_rom[622]=9'd355; perm_rom[623]=9'd381;
    end

    always @(posedge clk) begin
        if (!emit_frozen)
            enc_val0 <= perm_rom[rom_addr0];
    end

    always @(posedge clk) begin
        if (!emit_frozen)
            enc_val1 <= perm_rom[rom_addr1];
    end

    // =========================================================
    // Main FSM
    //
    // Emit pipeline (2 BRAM stages in series: enc ROM -> sbuf):
    //   S_PRIME1: emit_idx=0 presents rom addrs (enc[0], enc[1]).
    //   S_PRIME2: enc_val = enc[0,1]. Issue sbuf reads at enc[0,1].
    //             ROM reads enc[2,3].
    //   S_PRIME3: sbuf_rd = sbuf[enc[0,1]] ready. Issue reads enc[2,3].
    //             ROM reads enc[4,5].
    //   S_EMIT:   output pair k = sbuf[enc[2k]], sbuf[enc[2k+1]].
    //             Issue reads at enc_val (enc[2k+4,2k+5]).
    //             ROM reads enc[emit_idx] (2k+6,2k+7).
    // =========================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            valid_out      <= 0;
            soft_out0      <= 0;
            soft_out1      <= 0;
            wr_count       <= 0;
            wr_word        <= 0;
            emit_idx       <= 0;
            out_count      <= 0;
            lat_mode       <= 0;
            sbuf_a_addr    <= 0;
            sbuf_a_we      <= 0;
            sbuf_a_wdata   <= 0;
            sbuf_a_lane    <= 0;
            sbuf_b_addr    <= 0;
            sbuf_b_lane    <= 0;
        end else begin
            valid_out <= 0;
            sbuf_a_we <= 0;

            case (state)
                S_IDLE: begin
                    if (valid_in) begin
                        lat_mode <= rate_mode;
                        sbuf_a_we    <= 1;
                        sbuf_a_addr  <= 6'd0;
                        sbuf_a_wdata <= {soft_in5, soft_in4, soft_in3,
                                         soft_in2, soft_in1, soft_in0};
                        wr_count <= nbpsc(rate_mode);
                        wr_word  <= 6'd1;
                        state    <= S_CAPTURE;
                    end
                end

                S_CAPTURE: begin
                    if (valid_in) begin
                        sbuf_a_we    <= 1;
                        sbuf_a_addr  <= wr_word;
                        sbuf_a_wdata <= {soft_in5, soft_in4, soft_in3,
                                         soft_in2, soft_in1, soft_in0};
                        wr_count <= wr_count + nbpsc(lat_mode);
                        wr_word  <= wr_word + 6'd1;
                    end else begin
                        // Input stream ended — start emit pipeline.
                        emit_idx  <= 0;
                        out_count <= 0;
                        state     <= S_PRIME1;
                    end
                end

                S_PRIME1: begin
                    emit_idx <= 2;
                    state    <= S_PRIME2;
                end

                S_PRIME2: begin
                    sbuf_a_addr <= enc_val0[8:3];
                    sbuf_a_lane <= enc_val0[2:0];
                    sbuf_b_addr <= enc_val1[8:3];
                    sbuf_b_lane <= enc_val1[2:0];
                    emit_idx    <= 4;
                    state       <= S_PRIME3;
                end

                S_PRIME3: begin
                    sbuf_a_addr <= enc_val0[8:3];
                    sbuf_a_lane <= enc_val0[2:0];
                    sbuf_b_addr <= enc_val1[8:3];
                    sbuf_b_lane <= enc_val1[2:0];
                    emit_idx    <= 6;
                    state       <= S_EMIT;
                end

                S_EMIT: begin
                    valid_out <= 1;
                    if (!stall_in) begin
                        soft_out0    <= sbuf_a_byte;
                        soft_out1    <= sbuf_b_byte;
                        sbuf_a_addr  <= enc_val0[8:3];
                        sbuf_a_lane  <= enc_val0[2:0];
                        sbuf_b_addr  <= enc_val1[8:3];
                        sbuf_b_lane  <= enc_val1[2:0];
                        emit_idx     <= emit_idx + 2;
                        out_count    <= out_count + 2;

                        if (out_count + 2 == n_cbps) begin
                            // Last pair emitted. Hold it in S_TAIL until the
                            // downstream write is accepted: if fifo_full is
                            // asserted during the last pair's single
                            // presentation cycle, S_IDLE would drop valid_out
                            // and the skipped write would never be retried
                            // (lost pair -> stream desync). S_TAIL keeps
                            // valid_out high until stall_in is low.
                            state    <= S_TAIL;
                            wr_count <= 0;
                        end
                    end
                end

                S_TAIL: begin
                    valid_out <= 1;
                    if (!stall_in) begin
                        // Downstream write accepted at this edge — drop
                        // valid_out now so the pair is presented for
                        // exactly one non-full cycle (no duplicate write).
                        valid_out <= 0;
                        state     <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
