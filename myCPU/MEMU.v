module MEMU(
    input  wire        clk,
    input  wire        reset,
    // handshaking signals with EXU
    input  wire        EXU_to_MEM_valid,
    output wire        MEM_allow_in,
    // handshaking signals with WB
    input  wire        WB_allow_in,
    output wire        MEM_ready_go,
    output wire        MEM_to_WB_valid,

    // data from EXU
    input  wire [31:0] EXU_pc_to_MEM,
    input  wire [31:0] EXU_inst_to_MEM,
    input  wire [31:0] EXU_result_to_MEM,
    input  wire  [6:0] EXU_signals_pass_to_MEM,

    // data from data sram
    input  wire [31:0] data_sram_rdata,

    // to IDU
    output wire        MEM_to_IDU_gr_we,
    output wire  [4:0] MEM_to_IDU_dest,
    output wire        MEM_to_IDU_valid,
    output wire [31:0] MEM_to_IDU_forward,

    // data to WB
    output wire [31:0] MEM_pc_to_WB,
    output wire [31:0] MEM_inst_to_WB,
    output wire [31:0] MEM_result_to_WB,
    output wire  [5:0] MEM_signals_pass_to_WB
);
reg MEM_valid;

reg [31:0] inst_reg;
reg [31:0] pc_reg;
reg [31:0] ex_result_reg;
reg  [6:0] signals_pass_reg;

wire [31:0] pc;
wire [31:0] inst;
wire [31:0] alu_result;
wire  [6:0] signals_pass;
wire  [4:0] dest;
wire        gr_we;
wire        res_from_mem;

always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if (MEM_allow_in && EXU_to_MEM_valid) begin
        inst_reg <= EXU_inst_to_MEM;
    end
end

always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if (MEM_allow_in && EXU_to_MEM_valid) begin
        pc_reg <= EXU_pc_to_MEM;
    end
end

always @(posedge clk) begin
    if (reset) begin
        ex_result_reg <= 32'b0;
    end
    else if (MEM_allow_in && EXU_to_MEM_valid) begin
        ex_result_reg <= EXU_result_to_MEM;
    end
end

always @(posedge clk) begin
    if (reset) begin
        signals_pass_reg <= 7'b0;
    end
    else if (MEM_allow_in && EXU_to_MEM_valid) begin
        signals_pass_reg <= EXU_signals_pass_to_MEM;
    end
end
assign pc = pc_reg;
assign inst = inst_reg;
assign ex_result = ex_result_reg;
assign signals_pass = signals_pass_reg;
assign {res_from_mem, gr_we, dest} = signals_pass;

assign MEM_pc_to_WB = pc;
assign MEM_inst_to_WB = inst;
assign MEM_result_to_WB = res_from_mem ? data_sram_rdata : ex_result;

assign MEM_signals_pass_to_WB = {gr_we, dest};

// to IDU
assign MEM_to_IDU_gr_we = gr_we;
assign MEM_to_IDU_dest  = dest;
assign MEM_to_IDU_valid = MEM_valid;
assign MEM_to_IDU_forward = MEM_result_to_WB;

// MEM status
always @(posedge clk) begin
    if (reset) begin
        MEM_valid <= 1'b0;
    end
    else if (MEM_allow_in) begin
        MEM_valid <= EXU_to_MEM_valid;
    end
end
assign MEM_ready_go = 1'b1;
assign MEM_to_WB_valid = MEM_valid && MEM_ready_go;
assign MEM_allow_in = !MEM_valid || (MEM_ready_go && WB_allow_in);

endmodule