`include "busdef.vh"
module WBU(
    input  wire        clk,
    input  wire        reset,

    // handshaking signals with MEM
    input  wire        memValidout,
    output wire        wbAllowin,
    // // handshaking signals with outside (not implemented), doesn't need now
    // output wire        wbValidout,

    // data from MEM
    input  wire [31:0] mem2wbPC,
    input  wire [31:0] mem2wbInst,
    input  wire [31:0] mem2wbResult,
    input  wire [31:0] mem2wbMemvaddr,
    input  wire [`CSRBUSL] mem2wbCsrBus,
    input  wire [`MEMPASSBUSL] mem2wbPassBus,

    // register file interface
    output wire [ 4:0] rf_waddr,
    output wire [31:0] rf_wdata,
    output wire        rf_we,

    // to IDU
    output wire        WB_to_IDU_csr,
    output wire        WB_to_IDU_gr_we,
    output wire [ 4:0] WB_to_IDU_dest,
    output wire        WB_to_IDU_valid,
    output wire [31:0] WB_to_IDU_forward,

    // debug interface
    output wire [31:0] debug_pc,
    output wire [ 3:0] debug_rf_we,
    output wire [ 4:0] debug_rf_wnum,
    output wire [31:0] debug_rf_wdata,

    // csr interface
    output wire        csr_re,
    output wire        csr_we,
    output wire [13:0] csr_num,
    input  wire [31:0] csr_rvalue,
    output wire [31:0] csr_wmask,
    output wire [31:0] csr_wvalue,
    output wire [31:0] wb_pc,
    output wire        ertn_flush,
    output wire        wb_ex,
    output wire [ 5:0] wb_ecode,
    output wire [ 8:0] wb_esubcode,
    output wire [31:0] wb_vaddr,
);

reg [`CSRBUSL] csr_signals_reg;

reg         wbValidReg;
reg [ 31:0] inst_reg;
reg [ 31:0] pc_reg;
reg [ 31:0] result_reg;
reg [`MEMPASSBUSL] signals_pass_reg;
reg [ 31:0] MEM_memvaddr_to_WB_reg;

wire [31:0] pc;
wire [31:0] inst;
wire [31:0] result;
wire [ 5:0] signals_pass;
wire        gr_we;
wire [ 4:0] dest;

wire        WBU_csr;
wire        WBU_csr_we;
wire [13:0] WBU_csr_num;
wire [31:0] WBU_csr_wmask;
wire [31:0] WBU_csr_wvalue;
wire        WBU_exception;
wire [ 5:0] WBU_ecode;
wire [14:0] WBU_syscall_code;   // it is useless in design
wire        WBU_ertn_flush;

always @(posedge clk) begin
    if (reset) begin
        csr_signals_reg <= `CSRBUSW'b0;
    end
    else if (wbAllowin && memValidout) begin
        csr_signals_reg <= mem2wbCsrBus;
    end
end

always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if (wbAllowin && memValidout) begin
        inst_reg <= mem2wbInst;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if (wbAllowin && memValidout) begin
        pc_reg <= mem2wbPC;
    end
end
always @(posedge clk) begin
    if (reset) begin
        MEM_memvaddr_to_WB_reg <= 32'b0;
    end
    else if (wbAllowin && memValidout) begin
        MEM_memvaddr_to_WB_reg <= mem2wbMemvaddr;
    end
end
always @(posedge clk) begin
    if (reset) begin
        result_reg <= 32'b0;
    end
    else if (wbAllowin && memValidout) begin
        result_reg <= mem2wbResult;
    end
end
always @(posedge clk) begin
    if (reset) begin
        signals_pass_reg <= `MEMPASSBUSW'b0;
    end
    else if (wbAllowin && memValidout) begin
        signals_pass_reg <= mem2wbPassBus;
    end
end
assign pc = pc_reg;
assign inst = inst_reg;
assign result = result_reg;
assign signals_pass = signals_pass_reg;
assign gr_we = signals_pass[5];
assign dest = signals_pass[4:0];
assign rf_waddr = dest;
assign rf_wdata = WBU_csr ? csr_rvalue : result;
assign rf_we = gr_we & wbValidReg & ~WBU_exception;

// to IDU
assign WB_to_IDU_gr_we = gr_we;
assign WB_to_IDU_dest  = dest;
assign WB_to_IDU_valid = wbValidReg;
assign WB_to_IDU_forward = rf_wdata;

// WB status
always @(posedge clk ) begin
    if (reset) begin
        wbValidReg <= 1'b0;
    end
    else if(wbAllowin)begin
        wbValidReg <= memValidout;
    end
end

// csr signals
assign {WBU_csr, WBU_csr_we, WBU_csr_num, WBU_csr_wmask, WBU_csr_wvalue, WBU_exception, WBU_ecode, WBU_syscall_code, WBU_ertn_flush} = csr_signals_reg;
assign csr_re = 1'b1;
assign csr_we = WBU_csr_we;
assign csr_num = WBU_csr_num;
assign csr_wmask = WBU_csr_wmask;
assign csr_wvalue = WBU_csr_wvalue;
assign wb_pc = pc;
assign ertn_flush = WBU_ertn_flush;
assign wb_ex = WBU_exception;
assign wb_ecode = WBU_ecode;
assign wb_esubcode = 9'h0;  // now there is no ADEM exception.
assign WB_to_IDU_csr = WBU_csr;
assign wb_vaddr = MEM_memvaddr_to_WB_reg;

assign wbReadygo      = 1'b1;
assign wbValidout     = wbValidReg && wbReadygo;
assign wbAllowin      = !wbValidReg || (wbReadygo && wbValidout);

assign debug_pc       = pc;
assign debug_rf_we    = {4{rf_we}};
assign debug_rf_wnum  = rf_waddr;
assign debug_rf_wdata = rf_wdata;
endmodule