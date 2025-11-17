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
    output wire        inst_sram_req,
    output wire        inst_sram_wr,
    output wire [ 1:0] inst_sram_size,
    output wire [ 3:0] inst_sram_wstrb,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire        inst_sram_addr_ok,
    input  wire        inst_sram_data_ok,
    input  wire [31:0] inst_sram_rdata,

    // from IDU
    input  wire        br_taken,
    input  wire        br_taken_cancel,
    input  wire [31:0] br_target,
    input  wire        br_stall,

    // to IDU
    output wire [31:0] if2idInst,
    output wire [31:0] if2idPC,
    output wire        isadef,      // ADEF exception, attached to inst

    // handshaking signals with IDU
    input  wire        idAllowin,
    output wire        ifValidout
);
    // preif stage
    reg         preifValidReg;
    wire [31:0] nextpc;
    wire        preifValidout;
    wire        preifReadygo;
    wire        pcValidin;
    wire [31:0] nextpc_in;
    wire        pcValidout;
    wire [31:0] nextpc_out;

    // if stage
    reg         ifValidReg;
    reg  [31:0] pc;
    reg         instCancelReg;
    wire [31:0] seq_pc;
    wire        instValidout;

    // ADEF exception
    reg         regAdef;

    /*******************/
    /*   data buffer   */
    /*******************/
    data_buffer preif_nextpc_buffer(
        .clk(clk),
        .reset(reset),
        .dataReq(preifAllowin),
        .Validin(pcValidin),
        .data_in(nextpc_in),
        .Validout(pcValidout),
        .data_out(nextpc_out)
    );

    assign pcValidin = wb_ex || ertn_flush || br_taken_cancel;    // nextpc redirect condition
    assign nextpc_in = (wb_ex)    ? ex_entry
                      : ertn_flush ? ertn_pc
                      : br_taken   ? br_target
                      : seq_pc;     // do not use, only for completeness

    data_buffer if_inst_buffer(
        .clk(clk),
        .reset(reset),
        .dataReq(idAllowin || instCancelReg),
        .Validin(inst_sram_data_ok),
        .data_in(inst_sram_rdata),
        .Validout(instValidout),
        .data_out(if2idInst)
    );

    /*******************************/
    /*   pre-IF pipeline signals   */
    /*******************************/
    always @(posedge clk) begin
        if (reset) begin
            preifValidReg <= 1'b0;
        end
        else if (preifAllowin) begin
            preifValidReg <= 1'b1;
        end
    end

    assign nextpc = pcValidout ? nextpc_out : seq_pc;
    assign preifValidout = preifValidReg && preifReadygo;
    assign preifReadygo = inst_sram_req && inst_sram_addr_ok;
    assign preifAllowin = (!preifValidReg || (preifReadygo && ifAllowin)) && !br_stall && !reset;

    /*******************************/
    /*     IF pipeline signals     */
    /*******************************/
    // IF status
    always @(posedge clk) begin
        if (reset) begin
            ifValidReg <= 1'b0;
        end
        else if (ifAllowin) begin
            ifValidReg <= preifValidout;
        end
        else if(br_taken_cancel | wb_ex | ertn_flush) begin  // except and ertn flush
            ifValidReg <= 1'b0;
        end
    end

    // pc register & output to IDU
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc;
        end
        else if (preifValidout && ifAllowin) begin
            pc <= nextpc;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            instCancelReg <= 1'b0;
        end
        else if (inst_sram_data_ok) begin   // exp14 BUG: it should has higher priority than (br_taken_cancel | wb_ex | ertn_flush) && (!ifAllowin && !ifReadygo)
            instCancelReg <= 1'b0;
        end
        else if ((br_taken_cancel | wb_ex | ertn_flush) && (!ifAllowin && !ifReadygo)) begin  // exp14 BUG: when preifValidout = 1, do not cancel
            instCancelReg <= 1'b1;
        end
    end

    assign ifValidout = ifValidReg && ifReadygo;    // exp14 BUG: do not write (ifValidReg || instValidout) && ifReadygo
    assign ifReadygo = (inst_sram_data_ok || instValidout) && !instCancelReg;
    assign ifAllowin = (!ifValidReg || (ifReadygo && idAllowin)) && !instCancelReg;

    assign seq_pc    = pc + 4;
    assign if2idPC   = pc;

    /*******************************/
    /*     inst sram interface     */
    /*******************************/
    assign inst_sram_req    = preifValidReg && ifAllowin;
    assign inst_sram_wr     = 1'b0;
    assign inst_sram_size   = 2'b10;
    assign inst_sram_wstrb  = 4'b0000;
    assign inst_sram_addr   = { nextpc[31:2], 2'b00 };
    assign inst_sram_wdata  = 32'b0;

    /*******************************/
    /*   ADEF exception register   */
    /*******************************/
    always @(posedge clk) begin
        if (reset) begin
            regAdef <= 1'b0;
        end
        else if (ifAllowin) begin
            regAdef <= ~(nextpc[1:0] == 2'b0);
        end
    end
    assign isadef = regAdef; // csr_signal_reg for IF stage

endmodule

// exp14: this module buffers: 
// 1. instructions between IF and ID to handle timing differences
// 2. target pc between preIF and ID/WB to handle timing differences
// For improved timing performance, when Validin is high, even if validReg is not high, it can also have valid output
module data_buffer(
    input  wire        clk,
    input  wire        reset,
    input  wire        dataReq,
    input  wire        Validin,
    input  wire [31:0] data_in,
    output wire        Validout,
    output wire [31:0] data_out
);
    reg [31:0] data_reg;
    reg        validReg;

    always @(posedge clk) begin
        if (reset) begin
            validReg <= 1'b0;
        end
        else if (dataReq) begin
            validReg <= 1'b0;
        end
        else if (Validin) begin
            validReg <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            data_reg <= 32'h00000000;
        end
        else if (Validin) begin
            data_reg <= data_in;
        end
    end

    assign data_out = (Validin) ? data_in : data_reg;
    assign Validout = validReg || Validin;
endmodule