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
    input  wire  [31:0] IDU_pc_to_EXU,
    input  wire  [31:0] IDU_inst_to_EXU,
    input  wire [112:0] IDU_to_EX_ALU_signals,
    input  wire   [7:0] IDU_to_EX_pass_signals,
    
    // to MEM
    output wire [31:0] EXU_pc_to_MEM,
    output wire [31:0] EXU_inst_to_MEM,
    output wire [31:0] EXU_alu_result_to_MEM,
    output wire  [6:0] EXU_signals_pass_to_MEM,

    // to IDU
    output wire        EXU_to_IDU_gr_we,
    output wire  [4:0] EXU_to_IDU_dest,
    output wire        EXU_to_IDU_valid,
    output wire [31:0] EXU_to_IDU_forward,
    output wire        EXU_current_is_ld,

    // data sram interface
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata
);
reg EX_valid;
reg [31:0] inst_reg;
reg [31:0] pc_reg;
reg [112:0] alu_signals_reg;
reg [7:0] pass_signals_reg;

wire [31:0] pc;
wire [31:0] inst;

wire [14:0] alu_op;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire        src1_is_pc;
wire        src2_is_imm;
wire        res_from_mem;
wire        gr_we;
wire        mem_we;
wire [ 4:0] dest;

wire [31:0] alu_src1;
wire [31:0] alu_src2;
wire [31:0] alu_result;

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
        pass_signals_reg <= 8'b0;
    end
    else if (EXU_allow_in && IDU_to_EXU_valid) begin
        pass_signals_reg <= IDU_to_EX_pass_signals;
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
assign pc = pc_reg;
assign inst = inst_reg;
assign {rj_value, rkd_value, imm, alu_op, src1_is_pc, src2_is_imm} = alu_signals_reg;
assign {res_from_mem, gr_we, mem_we, dest} = pass_signals_reg;
assign EXU_pc_to_MEM = pc;
assign EXU_inst_to_MEM = inst;
assign EXU_alu_result_to_MEM = alu_result;
assign EXU_signals_pass_to_MEM = {res_from_mem, gr_we, dest};

// to data sram interface
assign data_sram_en = 1'b1;
assign data_sram_we = (mem_we && EX_valid) ? 4'b1111 : 4'b0;
assign data_sram_addr = alu_result;
assign data_sram_wdata = rkd_value;

// to IDU
assign EXU_to_IDU_gr_we = gr_we;
assign EXU_to_IDU_dest  = dest;
assign EXU_to_IDU_valid = EX_valid;
assign EXU_to_IDU_forward = alu_result;
assign EXU_current_is_ld = res_from_mem && EX_valid;

// EX status
always @(posedge clk) begin
    if (reset) begin
        EX_valid <= 1'b0;
    end
    else if (EXU_allow_in) begin
        EX_valid <= IDU_to_EXU_valid;
    end
end
assign EXU_ready_go = 1'b1;
assign EXU_to_MEM_valid = EX_valid && EXU_ready_go;
assign EXU_allow_in = !EX_valid || (EXU_ready_go && MEM_allow_in);

endmodule