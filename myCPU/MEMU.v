`include "busdef.vh"
module MEMU(
    input  wire        clk,
    input  wire        reset,

    // ie
    input  wire        wb_ex,
    input  wire        ertn_flush,

    // handshaking signals with EXU
    input  wire        exValidout,
    output wire        memAllowin,
    // handshaking signals with WB
    input  wire        wbAllowin,
    output wire        memValidout,

    // data from EXU
    input  wire [31:0] ex2memPC,
    input  wire [31:0] ex2memInst,
    input  wire [31:0] ex2memResult,
    input  wire [`CSRBUSL] ex2memCsrBus,
    input  wire [`EXPASSBUSL] ex2memPassBus,

    // data to EXU
    output wire        memStopMemAccess,

    // data from data sram
    input  wire [31:0] data_sram_rdata,
    input  wire        data_sram_data_ok,

    // to IDU
    output wire        MEM_to_IDU_csr,
    output wire        MEM_to_IDU_gr_we,
    output wire [ 4:0] MEM_to_IDU_dest,
    output wire        MEM_to_IDU_valid,
    output wire [31:0] MEM_to_IDU_forward,
    output wire        MEM_to_IDU_forward_valid,

    // data to WB
    output wire [31:0] mem2wbPC,
    output wire [31:0] mem2wbInst,
    output wire [31:0] mem2wbResult,
    output wire [31:0] mem2wbMemvaddr,
    output wire [`CSRBUSL] mem2wbCsrBus,
    output wire [`MEMPASSBUSL] mem2wbPassBus
);

reg         memValidReg;
reg  [31:0] inst_reg;
reg  [31:0] pc_reg;
reg  [31:0] ex_result_reg;
reg  [`EXPASSBUSL] signals_pass_reg;
reg  [`CSRBUSL] csr_signals_reg;

wire [31:0] pc;
wire [31:0] inst;
wire [31:0] alu_result;
wire [`EXPASSBUSL] signals_pass;   // exp14 BUG: signals bitwidth change
wire [ 4:0] dest;
wire        gr_we;
wire [ 4:0] res_from_mem;
wire [ 1:0] mem_offsets;
wire [31:0] ex_result;
wire [31:0] shift_rdata;
wire [31:0] mem_result;
wire        is_sram_inst;

wire        csr;
wire        csr_we;
wire [13:0] csr_num;
wire [31:0] csr_wmask;
wire [31:0] csr_wvalue;
wire        exception;
wire [ 5:0] ecode;
wire [14:0] syscall_code;
wire        memErtnFlush;

always @(posedge clk) begin
    if (reset) begin
        csr_signals_reg <= `CSRBUSW'b0;
    end
    else if (memAllowin && exValidout) begin
        csr_signals_reg <= ex2memCsrBus;
    end
end

always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if (memAllowin && exValidout) begin
        inst_reg <= ex2memInst;
    end
end

always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if (memAllowin && exValidout) begin
        pc_reg <= ex2memPC;
    end
end

always @(posedge clk) begin
    if (reset) begin
        ex_result_reg <= 32'b0;
    end
    else if (memAllowin && exValidout) begin
        ex_result_reg <= ex2memResult;
    end
end

always @(posedge clk) begin
    if (reset) begin
        signals_pass_reg <= `EXPASSBUSW'b0;
    end
    else if (memAllowin && exValidout) begin
        signals_pass_reg <= ex2memPassBus;
    end
end
assign {csr, csr_we, csr_num, csr_wmask, csr_wvalue, exception, ecode, syscall_code, memErtnFlush} = csr_signals_reg;

assign pc           = pc_reg;
assign inst         = inst_reg;
assign ex_result    = ex_result_reg;
assign signals_pass = signals_pass_reg;
assign {is_sram_inst, res_from_mem, mem_offsets, gr_we, dest} = signals_pass;

assign shift_rdata = mem_offsets == 2'b00 ?         data_sram_rdata :
                     mem_offsets == 2'b01 ? { 8'b0, data_sram_rdata[31: 8]}:
                     mem_offsets == 2'b10 ? {16'b0, data_sram_rdata[31:16]}:
                                            {24'b0, data_sram_rdata[31:24]};

assign mem_result[ 7: 0] = shift_rdata[ 7: 0];

assign mem_result[15: 8] = ({8{ res_from_mem[2]}}                    & {8{shift_rdata[ 7]}} )|
                           ({8{ res_from_mem[0]}}                    &  8'b0                )|
                           ({8{~res_from_mem[2] & ~res_from_mem[0]}} &    shift_rdata[15: 8]);

assign mem_result[31:16] = ({16{res_from_mem[2]}} & {16{shift_rdata[ 7]}} )|
                           ({16{res_from_mem[3]}} & {16{shift_rdata[15]}} )|
                           ({16{res_from_mem[4]}} &     shift_rdata[31:16]);

// 4 = ld.w, 3 = ld.h, 2 = ld.b, 1 = ld.hu, 0 = ld.bu

assign mem2wbPC       = pc  &{32{!(wb_ex || ertn_flush)}};
assign mem2wbInst     = inst&{32{!(wb_ex || ertn_flush)}};
assign mem2wbResult   =(wb_ex || ertn_flush) ? 32'b0 : (res_from_mem ? mem_result : ex_result);
assign mem2wbCsrBus   = csr_signals_reg & {`CSRBUSW{!(wb_ex || ertn_flush)}};
assign mem2wbMemvaddr = ex_result;  // if inst is load/store, then ex_result must be memvaddr

assign mem2wbPassBus = {gr_we, dest} & {`MEMPASSBUSW{!(wb_ex || ertn_flush)}};
assign memStopMemAccess = (exception | memErtnFlush) & memValidReg;

// to IDU
assign MEM_to_IDU_csr   = csr;
assign MEM_to_IDU_gr_we = gr_we;
assign MEM_to_IDU_dest  = dest;
assign MEM_to_IDU_valid = memValidReg;
assign MEM_to_IDU_forward = mem2wbResult;
assign MEM_to_IDU_forward_valid = |res_from_mem ? memValidout : 1'b1;

// MEM status
always @(posedge clk) begin
    if (reset) begin
        memValidReg <= 1'b0;
    end
    else if (wb_ex || ertn_flush) begin
        memValidReg <= 1'b0;
    end
    else if (memAllowin) begin
        memValidReg <= exValidout;
    end
end

assign memReadygo  =  is_sram_inst ? (data_sram_data_ok) : 1'b1;
assign memValidout =  memValidReg &&  memReadygo;
assign memAllowin  = (!memValidReg || (memReadygo && wbAllowin));

endmodule