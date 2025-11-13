`include "busdef.vh"
module IFU(
    input  wire        clk,
    input  wire        reset,

    // ie
    input  wire        wb_ex,
    input  wire        ertn_flush,

    // pc from csr
    input  wire [31:0] ex_entry,
    input  wire [31:0] ertn_pc,

    // inst sram interface
    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,

    // from IDU
    input  wire        br_taken,
    input  wire        br_taken_cancel,
    input  wire [31:0] br_target,

    // to IDU
    output wire [31:0] if2idInst,
    output wire [31:0] if2idPC,
    output wire        isadef,      // ADEF exception, attached to inst

    // handshaking signals with IDU
    input  wire        idAllowin,
    output wire        ifValidout
);
    // preif stage
    reg         ifValidReg;
    wire        preifValidout;
    wire        ifAllowin;

    reg  [31:0] pc;
    wire [31:0] seq_pc;
    wire [31:0] nextpc;

    // ADEF exception
    reg         regAdef;

    // inst sram interface
    assign inst_sram_en    = preifValidout && idAllowin;
    assign inst_sram_we    = 4'b0;
    assign inst_sram_addr  = {nextpc[31:2], 2'b0};  // word aligned to avoid unaligned access
    assign inst_sram_wdata = 32'b0;

    // ADEF exception register
    always @(posedge clk) begin
        if (reset) begin
            regAdef <= 1'b0;
        end
        else if (ifAllowin) begin
            regAdef <= ~(nextpc[1:0] == 2'b0);
        end
        // need not clear because although adef passed to id, it still blocked/disabled by valid signal
    end
    assign isadef = regAdef; // check current pc alignment

    // IF status
    always @(posedge clk) begin
        if (reset) begin
            ifValidReg <= 1'b0;
        end
        else if (ifAllowin) begin
            ifValidReg <= preifValidout;
        end
        else if(br_taken_cancel | wb_ex | ertn_flush) begin  // 例外时和例外返回时都需要清空，此处待做
            ifValidReg <= 1'b0;
        end
    end
    assign preifValidout = !reset;
    assign ifReadygo     =  1'b1;
    assign ifValidout    =  ifValidReg &&  ifReadygo;
    assign ifAllowin     = !ifValidReg || (ifReadygo && idAllowin);
    
    // pc register & output to IDU
    assign seq_pc = pc + 4;
    assign nextpc = (wb_ex)    ? ex_entry
                  : ertn_flush ? ertn_pc
                  : br_taken   ? br_target
                  : seq_pc;

    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc;
        end
        else if (preifValidout && ifAllowin) begin
            pc <= nextpc;
        end
    end

    assign if2idInst = inst_sram_rdata;
    assign if2idPC   = pc;

endmodule
