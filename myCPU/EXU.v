module EXU(
    input  wire        clk,
    input  wire        reset,
    // handshaking signals with IDU
    input  wire        IDU_to_EXU_valid,
    output wire        EXU_allow_in,
    // handshaking signals with MEM
    input  wire        MEM_allow_in,
    output wire        EXU_ready_go,
    output wire        EXU_to_MEM_valid,

    // data from IDU
    input  wire [31:0] IDU_pc_to_EXU,
    input  wire [31:0] IDU_inst_to_EXU,
    input  wire[112:0] IDU_to_EX_ALU_signals,
    input  wire [13:0] IDU_to_EX_pass_signals,
    input  wire [ 4:0] IDU_to_EX_div_signals,
    
    // to MEM
    output wire [31:0] EXU_pc_to_MEM,
    output wire [31:0] EXU_inst_to_MEM,
    output wire [31:0] EXU_result_to_MEM,
    output wire [12:0] EXU_signals_pass_to_MEM,

    // to IDU
    output wire        EXU_to_IDU_gr_we,
    output wire [ 4:0] EXU_to_IDU_dest,
    output wire        EXU_to_IDU_valid,
    output wire [31:0] EXU_to_IDU_forward,
    output wire        EXU_current_is_ld,

    // data sram interface
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata
);
reg         EX_valid;
reg [ 31:0] inst_reg;
reg [ 31:0] pc_reg;
reg [112:0] alu_signals_reg;
reg  [ 9:0] pass_signals_reg;
reg  [ 4:0] div_signals_reg;
reg signed_div_dividend_tvalid_reg;
reg signed_div_divisor_tvalid_reg;
reg unsigned_div_dividend_tvalid_reg;
reg unsigned_div_divisor_tvalid_reg;

wire [31:0] pc;
wire [31:0] inst;

wire [14:0] alu_op;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire        src1_is_pc;
wire        src2_is_imm;
wire [ 4:0] res_from_mem;
wire [ 1:0] mem_offsets;
wire        gr_we;
wire [ 2:0] mem_we; // 3 bits One-hot code for byte, half-word, word
wire [ 4:0] dest;
wire        use_div;
wire [ 3:0] div_op;

wire [31:0] alu_src1;
wire [31:0] alu_src2;
wire [31:0] alu_result;

wire [31:0] signed_div_dividend_tdata;
wire [31:0] signed_div_divisor_tdata;
wire        signed_div_dividend_tvalid;
wire        signed_div_divisor_tvalid;
wire        signed_div_dividend_tready;
wire        signed_div_divisor_tready;
wire [63:0] signed_div_result;
wire        signed_div_dout_valid;
wire [31:0] signed_div_remainder;
wire [31:0] signed_div_quotient;

wire [31:0] unsigned_div_dividend_tdata;
wire [31:0] unsigned_div_divisor_tdata;
wire        unsigned_div_dividend_tvalid;
wire        unsigned_div_divisor_tvalid;
wire        unsigned_div_dividend_tready;
wire        unsigned_div_divisor_tready;
wire [63:0] unsigned_div_result;
wire        unsigned_div_dout_valid;
wire [31:0] unsigned_div_remainder;
wire [31:0] unsigned_div_quotient;
wire [31:0] final_div_result;

wire [31:0] EXU_result;

always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        inst_reg <= IDU_inst_to_EXU;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        pc_reg <= IDU_pc_to_EXU;
    end
end
always @(posedge clk) begin
    if (reset) begin
        alu_signals_reg <= 113'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        alu_signals_reg <= IDU_to_EX_ALU_signals;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pass_signals_reg <= 14'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        pass_signals_reg <= IDU_to_EX_pass_signals;
    end
end
always @(posedge clk) begin
    if (reset) begin
        div_signals_reg <= 5'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        div_signals_reg <= IDU_to_EX_div_signals;
    end
end
always @(posedge clk) begin
    if (reset) begin
        signed_div_dividend_tvalid_reg <= 1'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        signed_div_dividend_tvalid_reg <= IDU_to_EX_div_signals[4] && (IDU_to_EX_div_signals[0] | IDU_to_EX_div_signals[1]);
    end
    else if (signed_div_dividend_tready) begin
        signed_div_dividend_tvalid_reg <= 1'b0;
    end
end

always @(posedge clk) begin
    if (reset) begin
        signed_div_divisor_tvalid_reg <= 1'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        signed_div_divisor_tvalid_reg <= IDU_to_EX_div_signals[4] && (IDU_to_EX_div_signals[0] | IDU_to_EX_div_signals[1]);
    end
    else if (signed_div_divisor_tready) begin
        signed_div_divisor_tvalid_reg <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        unsigned_div_dividend_tvalid_reg <= 1'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        unsigned_div_dividend_tvalid_reg <= IDU_to_EX_div_signals[4] && (IDU_to_EX_div_signals[2] | IDU_to_EX_div_signals[3]);
    end
    else if (unsigned_div_dividend_tready) begin
        unsigned_div_dividend_tvalid_reg <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        unsigned_div_divisor_tvalid_reg <= 1'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        unsigned_div_divisor_tvalid_reg <= IDU_to_EX_div_signals[4] && (IDU_to_EX_div_signals[2] | IDU_to_EX_div_signals[3]);
    end
    else if (unsigned_div_divisor_tready) begin
        unsigned_div_divisor_tvalid_reg <= 1'b0;
    end
end
assign alu_src1 = src1_is_pc ? pc : rj_value;
assign alu_src2 = src2_is_imm ? imm : rkd_value;

alu EXU_alu(
    .alu_op   (alu_op),
    .alu_src1 (alu_src1),
    .alu_src2 (alu_src2),
    .alu_result  (alu_result)
);

assign use_div = div_signals_reg[4];
assign div_op  = div_signals_reg[3:0];
assign signed_div_dividend_tdata = rj_value;
assign signed_div_divisor_tdata  = rkd_value;
assign signed_div_dividend_tvalid = signed_div_dividend_tvalid_reg;
assign signed_div_divisor_tvalid  = signed_div_divisor_tvalid_reg;
assign unsigned_div_dividend_tdata = rj_value;
assign unsigned_div_divisor_tdata  = rkd_value;
assign unsigned_div_dividend_tvalid = unsigned_div_dividend_tvalid_reg;
assign unsigned_div_divisor_tvalid  = unsigned_div_divisor_tvalid_reg;

mydiv_signed EXU_div_signed(
    .s_axis_divisor_tdata (signed_div_divisor_tdata),
    .s_axis_divisor_tready (signed_div_divisor_tready),
    .s_axis_divisor_tvalid (signed_div_divisor_tvalid),
    .s_axis_dividend_tdata (signed_div_dividend_tdata),
    .s_axis_dividend_tready (signed_div_dividend_tready),
    .s_axis_dividend_tvalid (signed_div_dividend_tvalid),
    .m_axis_dout_tdata (signed_div_result),
    .m_axis_dout_tvalid (signed_div_dout_valid),
    .aclk(clk)
);

mydiv_unsigned EXU_div_unsigned(
    .s_axis_divisor_tdata (unsigned_div_divisor_tdata),
    .s_axis_divisor_tready (unsigned_div_divisor_tready),
    .s_axis_divisor_tvalid (unsigned_div_divisor_tvalid),
    .s_axis_dividend_tdata (unsigned_div_dividend_tdata),
    .s_axis_dividend_tready (unsigned_div_dividend_tready),
    .s_axis_dividend_tvalid (unsigned_div_dividend_tvalid),
    .m_axis_dout_tdata (unsigned_div_result),
    .m_axis_dout_tvalid (unsigned_div_dout_valid),
    .aclk(clk)
);
assign signed_div_remainder  = signed_div_result[31:0];
assign signed_div_quotient   = signed_div_result[63:32];
assign unsigned_div_remainder= unsigned_div_result[31:0];
assign unsigned_div_quotient = unsigned_div_result[63:32];
assign final_div_result = (div_op[0]) ? signed_div_quotient :
                          (div_op[1]) ? signed_div_remainder :
                          (div_op[2]) ? unsigned_div_quotient :
                          (div_op[3]) ? unsigned_div_remainder : 32'b0;

assign EXU_result = use_div ? final_div_result : alu_result;
assign pc = pc_reg;
assign inst = inst_reg;
assign {rj_value, rkd_value, imm, alu_op, src1_is_pc, src2_is_imm} = alu_signals_reg;
assign {res_from_mem, gr_we, mem_we, dest} = pass_signals_reg;
assign EXU_pc_to_MEM = pc;
assign EXU_inst_to_MEM = inst;
assign EXU_result_to_MEM = EXU_result;
assign EXU_signals_pass_to_MEM = {res_from_mem, mem_offsets, gr_we, dest};
assign mem_offsets = alu_result[1:0];

// to data sram interface
assign data_sram_en = 1'b1;
assign data_sram_we = ~EX_valid ? 4'b0 :
                      mem_we[2] ? 4'b1111 :
                      mem_we[1] ? (mem_offsets[1]       ? 4'b1100 : 4'b0011):
                      mem_we[0] ? (mem_offsets == 2'b00 ? 4'b0001 :
                                   mem_offsets == 2'b01 ? 4'b0010 :
                                   mem_offsets == 2'b10 ? 4'b0100 :
                                                          4'b1000): 4'b0;
assign data_sram_addr = {alu_result[31:2], 2'b00};  // word aligned address(not sure...)
assign data_sram_wdata = mem_we[2] ?    rkd_value :
                         mem_we[1] ? {2{rkd_value[15:0]}} :
                         mem_we[0] ? {4{rkd_value[ 7:0]}} : 32'b0;

// to IDU
assign EXU_to_IDU_gr_we = gr_we;
assign EXU_to_IDU_dest  = dest;
assign EXU_to_IDU_valid = EX_valid;
assign EXU_to_IDU_forward = EXU_result;
assign EXU_current_is_ld = |res_from_mem && EX_valid;

// EX status
always @(posedge clk) begin
    if (reset) begin
        EX_valid <= 1'b0;
    end
    else if (EXU_allow_in) begin
        EX_valid <= IDU_to_EXU_valid;
    end
end
assign EXU_ready_go = use_div ? (signed_div_dout_valid | unsigned_div_dout_valid) : 1'b1;
assign EXU_to_MEM_valid = EX_valid && EXU_ready_go;
assign EXU_allow_in = !EX_valid || (EXU_ready_go && MEM_allow_in);

endmodule