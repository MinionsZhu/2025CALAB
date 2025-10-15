module WBU(
    input  wire        clk,
    input  wire        reset,
    // handshaking signals with MEM
    input  wire        MEM_to_WB_valid,
    output wire        WB_allow_in,
    // handshaking signals with outside (not implemented)
    output wire        WB_ready_go,
    output wire        WB_to_out_valid,

    // data from MEM
    input  wire [31:0] MEM_pc_to_WB,
    input  wire [31:0] MEM_inst_to_WB,
    input  wire [31:0] MEM_result_to_WB,
    input  wire  [5:0] MEM_signals_pass_to_WB,

    // register file interface
    output wire [ 4:0] rf_waddr,
    output wire [31:0] rf_wdata,
    output wire        rf_we,

    // to IDU
    output wire        WB_to_IDU_gr_we,
    output wire  [4:0] WB_to_IDU_dest,
    output wire        WB_to_IDU_valid,
    output wire [31:0] WB_to_IDU_forward,

    // debug interface
    output wire [31:0] debug_pc,
    output wire [ 3:0] debug_rf_we,
    output wire [ 4:0] debug_rf_wnum,
    output wire [31:0] debug_rf_wdata
);
reg WB_valid;
reg [31:0] inst_reg;
reg [31:0] pc_reg;
reg [31:0] result_reg;
reg  [5:0] signals_pass_reg;
wire [31:0] pc;
wire [31:0] inst;
wire [31:0] result;
wire  [5:0] signals_pass;
wire        gr_we;
wire  [4:0] dest;

always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if (WB_allow_in && MEM_to_WB_valid) begin
        inst_reg <= MEM_inst_to_WB;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if (WB_allow_in && MEM_to_WB_valid) begin
        pc_reg <= MEM_pc_to_WB;
    end
end
always @(posedge clk) begin
    if (reset) begin
        result_reg <= 32'b0;
    end
    else if (WB_allow_in && MEM_to_WB_valid) begin
        result_reg <= MEM_result_to_WB;
    end
end
always @(posedge clk) begin
    if (reset) begin
        signals_pass_reg <= 6'b0;
    end
    else if (WB_allow_in && MEM_to_WB_valid) begin
        signals_pass_reg <= MEM_signals_pass_to_WB;
    end
end
assign pc = pc_reg;
assign inst = inst_reg;
assign result = result_reg;
assign signals_pass = signals_pass_reg;
assign gr_we = signals_pass[5];
assign dest = signals_pass[4:0];
assign rf_waddr = dest;
assign rf_wdata = result;
assign rf_we = gr_we & WB_valid;

// to IDU
assign WB_to_IDU_gr_we = gr_we;
assign WB_to_IDU_dest  = dest;
assign WB_to_IDU_valid = WB_valid;
assign WB_to_IDU_forward = rf_wdata;

// WB status
always @(posedge clk ) begin
    if (reset) begin
        WB_valid <= 1'b0;
    end
    else if(WB_allow_in)begin
        WB_valid <= MEM_to_WB_valid;
    end
end

assign WB_ready_go      = 1'b1;
assign WB_to_out_valid  = WB_valid && WB_ready_go;
assign WB_allow_in      = !WB_valid || (WB_ready_go && WB_to_out_valid);

assign debug_pc       = pc;
assign debug_rf_we    = {4{rf_we}};
assign debug_rf_wnum  = rf_waddr;
assign debug_rf_wdata = rf_wdata;
endmodule