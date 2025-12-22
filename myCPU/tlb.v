module tlb
#(
    parameter TLBNUM = 16
)
(
    input  wire                  clk,
// search port 0 (for fetch)
input  wire [              18:0] s0_vppn,
input  wire                      s0_va_bit12,
input  wire [               9:0] s0_asid,
output wire                      s0_found,
output wire [$clog2(TLBNUM)-1:0] s0_index,
output wire [              19:0] s0_ppn,
output wire [               5:0] s0_ps,
output wire [               1:0] s0_plv,
output wire [               1:0] s0_mat,
output wire                      s0_d,
output wire                      s0_v,
// search port 1 (for load/store)
input  wire [              18:0] s1_vppn,
input  wire                      s1_va_bit12,
input  wire [               9:0] s1_asid,
output wire                      s1_found,
output wire [$clog2(TLBNUM)-1:0] s1_index,
output wire [              19:0] s1_ppn,
output wire [               5:0] s1_ps,
output wire [               1:0] s1_plv,
output wire [               1:0] s1_mat,
output wire                      s1_d,
output wire                      s1_v,
// invtlb opcode
input  wire                      invtlb_valid,
input  wire [               4:0] invtlb_op,
// write port
input  wire                      we,    
input  wire [$clog2(TLBNUM)-1:0] w_index,
input  wire                      w_e,
input  wire [              18:0] w_vppn,
input  wire [               5:0] w_ps,
input  wire [               9:0] w_asid,
input  wire                      w_g,
input  wire [              19:0] w_ppn0,
input  wire [               1:0] w_plv0,
input  wire [               1:0] w_mat0,
input  wire                      w_d0,
input  wire                      w_v0,
input  wire [              19:0] w_ppn1,
input  wire [               1:0] w_plv1,
input  wire [               1:0] w_mat1,
input  wire                      w_d1,
input  wire                      w_v1,
// read port
input  wire [$clog2(TLBNUM)-1:0] r_index,
output wire                      r_e,
output wire [              18:0] r_vppn,
output wire [               5:0] r_ps,
output wire [               9:0] r_asid,
output wire                      r_g,
output wire [              19:0] r_ppn0,
output wire [               1:0] r_plv0,
output wire [               1:0] r_mat0,
output wire                      r_d0,
output wire                      r_v0,
output wire [              19:0] r_ppn1,
output wire [               1:0] r_plv1,
output wire [               1:0] r_mat1,
output wire                      r_d1,
output wire                      r_v1
);  // tlb from bookguide

// VALEN: 虚地址的有效位数，32位
// PALEN: 物理地址的有效位数，32位

reg [TLBNUM-1:0] tlb_e;
reg [TLBNUM-1:0] tlb_ps4MB; // pagesize [1]:4MB, [0]:4KB
reg [      18:0] tlb_vppn   [TLBNUM-1:0];   // 19 bits = VALEN-13 [31:13]
reg [       9:0] tlb_asid   [TLBNUM-1:0];
reg              tlb_g      [TLBNUM-1:0];

reg [      19:0] tlb_ppn0   [TLBNUM-1:0];   // 20 bits = PALEN-12
reg [       1:0] tlb_plv0   [TLBNUM-1:0];
reg [       1:0] tlb_mat0   [TLBNUM-1:0];
reg              tlb_d0     [TLBNUM-1:0];
reg              tlb_v0     [TLBNUM-1:0];

reg [      19:0] tlb_ppn1   [TLBNUM-1:0];   // 20 bits = PALEN-12
reg [       1:0] tlb_plv1   [TLBNUM-1:0];
reg [       1:0] tlb_mat1   [TLBNUM-1:0];
reg              tlb_d1     [TLBNUM-1:0];
reg              tlb_v1     [TLBNUM-1:0];

// -----------------------------
// Search logic for port 0 and port 1
// -----------------------------
// match vectors
wire [TLBNUM-1:0] s0_match;
wire [TLBNUM-1:0] s1_match;

wire [TLBNUM-1:0] cond1;
wire [TLBNUM-1:0] cond2;
wire [TLBNUM-1:0] cond3;
wire [TLBNUM-1:0] cond4;

wire [TLBNUM-1:0] invtlb_mask[0:31];    // 32 invtlb operations

genvar gi;
generate
    for (gi = 0; gi < TLBNUM; gi = gi + 1) begin : GEN_MATCH
        // For 4MB pages we ignore the low bit of vppn(9 bits)
        wire vppn_equal_s0 = (tlb_vppn[gi][18:9] == s0_vppn[18:9]) && (tlb_ps4MB[gi] || tlb_vppn[gi][8:0] == s0_vppn[8:0]);
        wire vppn_equal_s1 = (tlb_vppn[gi][18:9] == s1_vppn[18:9]) && (tlb_ps4MB[gi] || tlb_vppn[gi][8:0] == s1_vppn[8:0]);

        assign s0_match[gi] = tlb_e[gi] && vppn_equal_s0 && (tlb_g[gi] || (tlb_asid[gi] == s0_asid));
        assign s1_match[gi] = tlb_e[gi] && vppn_equal_s1 && (tlb_g[gi] || (tlb_asid[gi] == s1_asid));
    end
endgenerate

generate
    for (gi = 0; gi < TLBNUM; gi = gi + 1) begin : GEN_COND
       assign cond1[gi] = ~tlb_g[gi];               // g == 0
       assign cond2[gi] =  tlb_g[gi];               // g == 1
       assign cond3[gi] = s1_asid == tlb_asid[gi];  // asid equal
       assign cond4[gi] = s1_vppn[18:9] == tlb_vppn[gi][18:9] && (tlb_ps4MB[gi] || (s1_vppn[8:0] == tlb_vppn[gi][8:0]));    // vppn equal
    end
endgenerate

assign invtlb_mask[0] = {TLBNUM{1'b1}};             // op 0: invtlb all
assign invtlb_mask[1] = {TLBNUM{1'b1}};             // op 1: invtlb all
assign invtlb_mask[2] = cond2;                      // op 2: invtlb all global
assign invtlb_mask[3] = cond1;                      // op 3: invtlb all non-global
assign invtlb_mask[4] = cond1 & cond3;              // op 4: invtlb non-global by asid
assign invtlb_mask[5] = cond1 & cond3 & cond4;      // op 5: invtlb non-global by asid and vppn
assign invtlb_mask[6] = (cond1 | cond3) & cond4;    // op 6: invtlb non-global by vppn or asid
generate
    for (gi = 7; gi < 32; gi = gi + 1) begin : GEN_INVTLB_MASK_ZERO
       assign invtlb_mask[gi] = {TLBNUM{1'b0}};     // op 7-31: reserved, do nothing
    end
endgenerate

assign s0_found = |s0_match;
assign s1_found = |s1_match;

// generate index // hard coding for simplicity
assign s0_index =   s0_match[ 1] ? 4'd1  :
                    s0_match[ 2] ? 4'd2  :
                    s0_match[ 3] ? 4'd3  :
                    s0_match[ 4] ? 4'd4  :
                    s0_match[ 5] ? 4'd5  :
                    s0_match[ 6] ? 4'd6  :
                    s0_match[ 7] ? 4'd7  :
                    s0_match[ 8] ? 4'd8  :
                    s0_match[ 9] ? 4'd9  :
                    s0_match[10] ? 4'd10 :
                    s0_match[11] ? 4'd11 :
                    s0_match[12] ? 4'd12 :
                    s0_match[13] ? 4'd13 :
                    s0_match[14] ? 4'd14 :
                    s0_match[15] ? 4'd15 :
                    4'd0; // Default, 没有找到时需要把found置为0
assign s1_index =   s1_match[ 1] ? 4'd1  :
                    s1_match[ 2] ? 4'd2  :
                    s1_match[ 3] ? 4'd3  :
                    s1_match[ 4] ? 4'd4  :
                    s1_match[ 5] ? 4'd5  :
                    s1_match[ 6] ? 4'd6  :
                    s1_match[ 7] ? 4'd7  :
                    s1_match[ 8] ? 4'd8  :
                    s1_match[ 9] ? 4'd9  :
                    s1_match[10] ? 4'd10 :
                    s1_match[11] ? 4'd11 :
                    s1_match[12] ? 4'd12 :
                    s1_match[13] ? 4'd13 :
                    s1_match[14] ? 4'd14 :
                    s1_match[15] ? 4'd15 :
                    4'd0; // Default, 没有找到时需要把found置为0

wire   s0_sel, s1_sel;
assign s0_sel   = tlb_ps4MB[s0_index] ? s0_vppn[8] : s0_va_bit12;
assign s0_ps    = tlb_ps4MB[s0_index] ? 6'd21 : 6'd12;
assign s0_ppn   = s0_sel ? tlb_ppn1[s0_index] : tlb_ppn0[s0_index];
assign s0_plv   = s0_sel ? tlb_plv1[s0_index] : tlb_plv0[s0_index];
assign s0_mat   = s0_sel ? tlb_mat1[s0_index] : tlb_mat0[s0_index];
assign s0_d     = s0_sel ? tlb_d1  [s0_index] : tlb_d0  [s0_index];
assign s0_v     = s0_sel ? tlb_v1  [s0_index] : tlb_v0  [s0_index];


assign s1_sel   = tlb_ps4MB[s1_index] ? s1_vppn[8] : s1_va_bit12;
assign s1_ps    = tlb_ps4MB[s1_index] ? 6'd21 : 6'd12;
assign s1_ppn   = s1_sel ? tlb_ppn1[s1_index] : tlb_ppn0[s1_index];
assign s1_plv   = s1_sel ? tlb_plv1[s1_index] : tlb_plv0[s1_index];
assign s1_mat   = s1_sel ? tlb_mat1[s1_index] : tlb_mat0[s1_index];
assign s1_d     = s1_sel ? tlb_d1  [s1_index] : tlb_d0  [s1_index];
assign s1_v     = s1_sel ? tlb_v1  [s1_index] : tlb_v0  [s1_index];

// read port (combinational read)
assign r_e     = tlb_e[r_index];
assign r_vppn  = tlb_vppn[r_index];
assign r_ps    = (tlb_ps4MB[r_index]) ? 6'd21 : 6'd12;
assign r_asid  = tlb_asid[r_index];
assign r_g     = tlb_g[r_index];

assign r_ppn0  = tlb_ppn0[r_index];
assign r_plv0  = tlb_plv0[r_index];
assign r_mat0  = tlb_mat0[r_index];
assign r_d0    = tlb_d0[r_index];
assign r_v0    = tlb_v0[r_index];

assign r_ppn1  = tlb_ppn1[r_index];
assign r_plv1  = tlb_plv1[r_index];
assign r_mat1  = tlb_mat1[r_index];
assign r_d1    = tlb_d1[r_index];
assign r_v1    = tlb_v1[r_index];

// write port (synchronous)
integer i;
always @(posedge clk) begin
    if (we) begin
        tlb_e[w_index]      <= w_e;
        tlb_vppn[w_index]   <= w_vppn;
        // w_ps is 6-bit externally; convert to internal 1-bit (1 => 4MB(2^22), 0 => 4KB(2^12))
        tlb_ps4MB[w_index]  <= (w_ps == 6'd12) ? 1'b0 : 1'b1;
        tlb_asid[w_index]   <= w_asid;
        tlb_g[w_index]      <= w_g;
        tlb_ppn0[w_index]   <= w_ppn0;
        tlb_plv0[w_index]   <= w_plv0;
        tlb_mat0[w_index]   <= w_mat0;
        tlb_d0[w_index]     <= w_d0;
        tlb_v0[w_index]     <= w_v0;
        tlb_ppn1[w_index]   <= w_ppn1;
        tlb_plv1[w_index]   <= w_plv1;
        tlb_mat1[w_index]   <= w_mat1;
        tlb_d1[w_index]     <= w_d1;
        tlb_v1[w_index]     <= w_v1;
    end
    // simple invtlb: op==0 -> invalidate all
    if (invtlb_valid) begin
        tlb_e <= ~invtlb_mask[invtlb_op] & tlb_e;
    end
end

///////////////////////////////////////////////////////////
// Old search logic implementation (kept for reference)
///////////////////////////////////////////////////////////

// // outputs as regs (driven by combinational logic)
// reg                       s0_found_r;
// reg  [$clog2(TLBNUM)-1:0] s0_index_r;
// reg  [              19:0] s0_ppn_r;
// reg  [               5:0] s0_ps_r;
// reg  [               1:0] s0_plv_r;
// reg  [               1:0] s0_mat_r;
// reg                       s0_d_r;
// reg                       s0_v_r;

// reg                       s1_found_r;
// reg  [$clog2(TLBNUM)-1:0] s1_index_r;
// reg  [              19:0] s1_ppn_r;
// reg  [               5:0] s1_ps_r;
// reg  [               1:0] s1_plv_r;
// reg  [               1:0] s1_mat_r;
// reg                       s1_d_r;
// reg                       s1_v_r;

// always @* begin
//     // default values
//     s0_found_r  = 1'b0;
//     s0_index_r  = {($clog2(TLBNUM)){1'b0}};
//     s0_ppn_r    = {20{1'b0}};
//     s0_ps_r     = 6'd12;
//     s0_plv_r    = 2'b0;
//     s0_mat_r    = 2'b0;
//     s0_d_r      = 1'b0;
//     s0_v_r      = 1'b0;

//     s1_found_r  = 1'b0;
//     s1_index_r  = {($clog2(TLBNUM)){1'b0}};
//     s1_ppn_r    = {20{1'b0}};
//     s1_ps_r     = 6'd12;
//     s1_plv_r    = 2'b0;
//     s1_mat_r    = 2'b0;
//     s1_d_r      = 1'b0;
//     s1_v_r      = 1'b0;

//     for (i = 0; i < TLBNUM; i = i + 1) begin
//         if (s0_match[i]) begin
//             if (!s0_found_r) begin
//                 s0_found_r = 1'b1;
//                 s0_index_r = i[$clog2(TLBNUM)-1:0];
//                 // choose ppn/plv/mat/d/v according to va_bit12
//                 if (s0_va_bit12) begin
//                     s0_ppn_r = tlb_ppn1[i];
//                     s0_plv_r = tlb_plv1[i];
//                     s0_mat_r = tlb_mat1[i];
//                     s0_d_r   = tlb_d1[i];
//                     s0_v_r   = tlb_v1[i];
//                 end else begin
//                     s0_ppn_r = tlb_ppn0[i];
//                     s0_plv_r = tlb_plv0[i];
//                     s0_mat_r = tlb_mat0[i];
//                     s0_d_r   = tlb_d0[i];
//                     s0_v_r   = tlb_v0[i];
//                 end
//                 s0_ps_r = (tlb_ps4MB[i]) ? 6'd21 : 6'd12;
//             end
//             else begin
//                 // multiple matches found -- should not happen
//             end
//         end

//         if (s1_match[i]) begin
//             if (!s1_found_r) begin
//                 s1_found_r = 1'b1;
//                 s1_index_r = i[$clog2(TLBNUM)-1:0];
//                 if (s1_va_bit12) begin
//                     s1_ppn_r = tlb_ppn1[i];
//                     s1_plv_r = tlb_plv1[i];
//                     s1_mat_r = tlb_mat1[i];
//                     s1_d_r   = tlb_d1[i];
//                     s1_v_r   = tlb_v1[i];
//                 end else begin
//                     s1_ppn_r = tlb_ppn0[i];
//                     s1_plv_r = tlb_plv0[i];
//                     s1_mat_r = tlb_mat0[i];
//                     s1_d_r   = tlb_d0[i];
//                     s1_v_r   = tlb_v0[i];
//                 end
//                 s1_ps_r = (tlb_ps4MB[i]) ? 6'd21 : 6'd12;
//             end
//             else begin
//                 // multiple matches found -- should not happen
//             end
//         end
//     end
//     // if (!s0_found_r) begin
//     //     s0_index_r = {($clog2(TLBNUM)){1'b0}};
//     // end
//     // if (!s1_found_r) begin
//     //     s1_index_r = {($clog2(TLBNUM)){1'b0}};
//     // end
// end

// // drive outputs
// assign s0_found  = s0_found_r;
// assign s0_index  = s0_index_r;
// assign s0_ppn    = s0_ppn_r;
// assign s0_ps     = s0_ps_r;
// assign s0_plv    = s0_plv_r;
// assign s0_mat    = s0_mat_r;
// assign s0_d      = s0_d_r;
// assign s0_v      = s0_v_r;

// assign s1_found  = s1_found_r;
// assign s1_index  = s1_index_r;
// assign s1_ppn    = s1_ppn_r;
// assign s1_ps     = s1_ps_r;
// assign s1_plv    = s1_plv_r;
// assign s1_mat    = s1_mat_r;
// assign s1_d      = s1_d_r;
// assign s1_v      = s1_v_r;

endmodule