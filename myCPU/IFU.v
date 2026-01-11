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

    // icache interface
    output wire        icache_valid,
    output wire        icache_op,
    output wire        icache_uncached,
    output wire [ 7:0] icache_index,
    output wire [19:0] icache_tag,
    output wire [ 3:0] icache_offset,
    output wire [ 3:0] icache_wstrb,
    output wire [31:0] icache_wdata,
    input  wire        icache_addr_ok,
    input  wire        icache_data_ok,
    input  wire [31:0] icache_rdata,

    // from IDU
    input  wire        br_taken,
    input  wire        br_taken_cancel,
    input  wire [31:0] br_target,
    input  wire        br_stall,

    // to IDU
    output wire [31:0] if2idInst,
    output wire [31:0] if2idPC,
    output wire        isadef,      // ADEF exception, attached to inst
    output wire        if_tlbr_ex, // TLB refill exception, attached to inst
    output wire        if_pif_ex,  // PIF exception, attached to inst
    output wire        if_ppi_ex,  // PPI exception, attached to inst

    // handshaking signals with IDU
    input  wire        idAllowin,
    output wire        ifValidout,

    // refetch signal
    input  wire        changeTLB_stall,
    input  wire        changeIcache_stall,
    input  wire        wb_refetch,
    input  wire [31:0] wb_pc,
    output wire        if2idRefetch,

    input  wire [9:0]  csr_asid_asid,
    output wire [18:0] s0_vppn,
    output wire        s0_va_bit12,
    output wire [ 9:0] s0_asid,
    input  wire        s0_found,
    input  wire [19:0] s0_ppn,
    input  wire [ 1:0] s0_plv,
    input  wire [ 1:0] s0_mat,  // add in exp21
    input  wire        s0_v,

    input  wire [1:0]  csr_crmd_plv,
    input  wire        csr_crmd_da,
    input  wire        csr_crmd_pg,
    input  wire [1:0]  csr_crmd_datf,
    input  wire [1:0]  csr_crmd_datm,

    input  wire        csr_dmw0_plv0,
    input  wire [1:0]  csr_dmw0_mat,
    input  wire        csr_dmw0_plv3,
    input  wire [2:0]  csr_dmw0_pseg,
    input  wire [2:0]  csr_dmw0_vseg,
    input  wire        csr_dmw1_plv0,
    input  wire [1:0]  csr_dmw1_mat,
    input  wire        csr_dmw1_plv3,
    input  wire [2:0]  csr_dmw1_pseg,
    input  wire [2:0]  csr_dmw1_vseg,

    input  wire [31:0] csr_tlbrentry,
    input  wire        tlbr_ex 
);
    // preif stage
    reg         preifValidReg;
    wire [31:0] nextpc;
    wire [31:0] nextpc_p; // for inst_sram_addr, physical address
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
    reg         refetch;
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

    assign pcValidin = wb_ex || ertn_flush || br_taken_cancel || wb_refetch;    // nextpc redirect condition
    assign nextpc_in = (wb_ex && !tlbr_ex) ? ex_entry
                      :(wb_ex &&  tlbr_ex) ? csr_tlbrentry
                      : wb_refetch ? wb_pc
                      : ertn_flush  ? ertn_pc
                      : br_taken    ? br_target
                      : seq_pc;     // do not use, only for completeness

    data_buffer if_inst_buffer(
        .clk(clk),
        .reset(reset),
        .dataReq((idAllowin && ifValidout) || instCancelReg),
        .Validin(icache_data_ok),
        .data_in(icache_rdata),
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

    wire dat;
    wire pat;
    assign dat = csr_crmd_da && !csr_crmd_pg;
    assign pat = csr_crmd_pg && !csr_crmd_da;

    assign s0_vppn     = nextpc[31:13];
    assign s0_va_bit12 = nextpc[12];
    assign s0_asid     = csr_asid_asid;

    wire [31:0] nextpc_ptt;
    assign nextpc_ptt = {s0_ppn, nextpc[11:0]};

    wire dmw0_hit;
    wire dmw1_hit;
    assign dmw0_hit = ((csr_crmd_plv == 0 && csr_dmw0_plv0) || (csr_crmd_plv == 3 && csr_dmw0_plv3)) &&
                      (nextpc[31:29] == csr_dmw0_vseg);
    assign dmw1_hit = ((csr_crmd_plv == 0 && csr_dmw1_plv0) || (csr_crmd_plv == 3 && csr_dmw1_plv3)) &&
                      (nextpc[31:29] == csr_dmw1_vseg);
    wire [31:0] nextpc_dmw0;
    assign nextpc_dmw0 = {csr_dmw0_pseg, nextpc[28:0]};

    wire [31:0] nextpc_dmw1; //DMW1
    assign nextpc_dmw1 = {csr_dmw1_pseg, nextpc[28:0]};
    // nextpc physical address calculation

    assign nextpc = pcValidout ? nextpc_out : seq_pc;
    assign nextpc_p = dat ? nextpc :
                      pat ? (dmw0_hit ? nextpc_dmw0 : (dmw1_hit ? nextpc_dmw1 : nextpc_ptt))
                     : 0;
    wire preif_tlbr_ex, preif_pif_ex, preif_ppi_ex;
    assign preif_tlbr_ex = pat && !dmw0_hit && !dmw1_hit && !s0_found;
    assign preif_pif_ex  = pat && !dmw0_hit && !dmw1_hit && s0_found && !s0_v;
    assign preif_ppi_ex  = pat && !dmw0_hit && !dmw1_hit && s0_found && s0_v && (csr_crmd_plv > s0_plv);
    wire preif_addr_ex;
    assign preif_addr_ex = preif_tlbr_ex || preif_pif_ex || preif_ppi_ex;
    assign preifValidout = preifValidReg && preifReadygo;
    assign preifReadygo = icache_valid && icache_addr_ok && !preif_addr_ex || preif_addr_ex;
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
        else if(br_taken_cancel | wb_ex | ertn_flush | wb_refetch) begin  // except and ertn flush
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

    wire  stall;
    assign stall = changeTLB_stall || changeIcache_stall;
    always @(posedge clk) begin
        if (reset) begin
            refetch <= 1'b0;
        end
        else if (ifAllowin) begin
            refetch <= stall;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            instCancelReg <= 1'b0;
        end
        else if (icache_data_ok) begin   // exp14 BUG: it should has higher priority than (br_taken_cancel | wb_ex | ertn_flush) && (!ifAllowin && !ifReadygo)
            instCancelReg <= 1'b0;
        end
        else if ((br_taken_cancel | wb_ex | ertn_flush | wb_refetch) && (!ifAllowin && !ifReadygo)) begin  // exp14 BUG: when preifValidout = 1, do not cancel
            instCancelReg <= 1'b1;
        end
    end

    reg tlbr_ex_reg;
    reg pif_ex_reg;
    reg ppi_ex_reg;
    always @(posedge clk) begin
        if (reset) begin
            tlbr_ex_reg <= 1'b0;
            pif_ex_reg  <= 1'b0;
            ppi_ex_reg  <= 1'b0;
        end
        else if (ifAllowin) begin
            tlbr_ex_reg <= preif_tlbr_ex;
            pif_ex_reg  <= preif_pif_ex;
            ppi_ex_reg  <= preif_ppi_ex;
        end
        else if(br_taken_cancel | wb_ex | ertn_flush | wb_refetch) begin  // except and ertn flush
            tlbr_ex_reg <= 1'b0;
            pif_ex_reg  <= 1'b0;
            ppi_ex_reg  <= 1'b0;
        end
    end
    wire   if_addr_ex;
    assign if_addr_ex = tlbr_ex_reg || pif_ex_reg || ppi_ex_reg;
    assign ifValidout = ifValidReg && ifReadygo;    // exp14 BUG: do not write (ifValidReg || instValidout) && ifReadygo
    assign ifReadygo = (icache_data_ok || instValidout || if_addr_ex) && !instCancelReg && !stall;
    assign ifAllowin = (!ifValidReg || (ifReadygo && idAllowin)) && !instCancelReg;

    assign seq_pc    = pc + 4;
    assign if2idPC   = pc;

    /*******************************/
    /*      icache interface       */
    /*******************************/
    assign icache_valid     = preifValidReg && ifAllowin && !preif_addr_ex && !changeIcache_stall;
    assign icache_op        = 1'b0;
    assign icache_wstrb     = 4'b0000;
    assign icache_wdata     = 32'b0;
    assign icache_index     = nextpc_p[11:4];
    assign icache_tag       = nextpc_p[31:12];
    assign icache_offset    = nextpc_p[3:0];
    assign icache_uncached  = (dat) ? (csr_crmd_datf == 2'b00) :
                              (pat) ? (dmw0_hit ? (csr_dmw0_mat == 2'b00) :
                                       dmw1_hit ? (csr_dmw1_mat == 2'b00) : 
                                      (s0_mat == 2'b00))
                                    : 1'b0;  // if not dat or pat, then uncached is 0

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
    assign if2idRefetch = refetch;
    assign if_tlbr_ex = tlbr_ex_reg;
    assign if_pif_ex  = pif_ex_reg;
    assign if_ppi_ex  = ppi_ex_reg;

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