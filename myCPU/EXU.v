`include "busdef.vh"
`include "ecodes.vh"
module EXU(
    input  wire        clk,
    input  wire        reset,

    // ie
    input  wire        wb_ex,
    input  wire        ertn_flush,

    // handshaking signals with IDU
    input  wire        idValidout,
    output wire        exAllowin,
    // handshaking signals with MEM
    input  wire        memAllowin,
    output wire        exValidout,

    // data from IDU
    input  wire [31:0] id2exPC,
    input  wire [31:0] id2exInst,
    input  wire [ 4:0] id2exDivBus,
    input  wire [`CSRBUSL] id2exCsrBus,
    input  wire [`ALUBUSL] id2exALUBus,
    input  wire [`IDPASSBUSL] id2exPassBus,

    // data from MEM
    input  wire        memStopMemAccess,
    
    // to MEM
    output wire [31:0] ex2memPC,
    output wire [31:0] ex2memInst,
    output wire [31:0] ex2memResult,
    output wire [`CSRBUSL] ex2memCsrBus,
    output wire [`EXPASSBUSL] ex2memPassBus,

    // to IDU, for raw
    output wire        EXU_to_IDU_csr,
    output wire        EXU_to_IDU_gr_we,
    output wire [ 4:0] EXU_to_IDU_dest,
    output wire        EXU_to_IDU_valid,
    output wire [31:0] EXU_to_IDU_forward,
    output wire        EXU_current_is_ld,

    // data sram interface
    output wire        data_sram_req,
    output wire        data_sram_wr,
    output wire [ 1:0] data_sram_size,
    output wire [ 3:0] data_sram_wstrb,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire        data_sram_addr_ok
);
reg         exValidReg;

reg [ 31:0] inst_reg;
reg [ 31:0] pc_reg;
reg [  4:0] div_signals_reg;
reg [`ALUBUSL] alu_signals_reg;
reg [`IDPASSBUSL] pass_signals_reg;

reg         signed_div_dividend_tvalid_reg;
reg         signed_div_divisor_tvalid_reg;
reg         unsigned_div_dividend_tvalid_reg;
reg         unsigned_div_divisor_tvalid_reg;

reg [`CSRBUSL] csr_signals_reg;

reg [ 63:0] stable_counter;
wire        inst_rdcntvl_w;
wire        inst_rdcntvh_w;

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

wire        csr;
wire        csr_we;
wire [13:0] csr_num;
wire [31:0] csr_wmask;
wire [31:0] csr_wvalue;
wire [31:0] null32;

wire        exception;
wire        isale;
wire [ 5:0] ecode;
wire [ 5:0] newecode;       // add ALE exception
wire [14:0] syscall_code;

wire        exErtnFlush;

wire [31:0] exResult;
always @(posedge clk) begin
    if (reset) begin
        csr_signals_reg <= `CSRBUSW'b0;
    end
    else if (exAllowin && idValidout) begin
        csr_signals_reg <= id2exCsrBus;
    end
end

always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if (exAllowin && idValidout) begin
        inst_reg <= id2exInst;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if (exAllowin && idValidout) begin
        pc_reg <= id2exPC;
    end
end
always @(posedge clk) begin
    if (reset) begin
        alu_signals_reg <= `ALUBUSW'b0;
    end
    else if (exAllowin && idValidout) begin
        alu_signals_reg <= id2exALUBus;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pass_signals_reg <= `IDPASSBUSW'b0;
    end
    else if (exAllowin && idValidout) begin
        pass_signals_reg <= id2exPassBus;
    end
end
always @(posedge clk) begin
    if (reset) begin
        div_signals_reg <= 5'b0;
    end
    else if (exAllowin && idValidout) begin
        div_signals_reg <= id2exDivBus;
    end
end
always @(posedge clk) begin
    if (reset) begin
        signed_div_dividend_tvalid_reg <= 1'b0;
    end
    else if (exAllowin && idValidout) begin
        signed_div_dividend_tvalid_reg <= id2exDivBus[4] && (id2exDivBus[0] | id2exDivBus[1]);
    end
    else if (signed_div_dividend_tready) begin
        signed_div_dividend_tvalid_reg <= 1'b0;
    end
end

always @(posedge clk) begin
    if (reset) begin
        signed_div_divisor_tvalid_reg <= 1'b0;
    end
    else if (exAllowin && idValidout) begin
        signed_div_divisor_tvalid_reg <= id2exDivBus[4] && (id2exDivBus[0] | id2exDivBus[1]);
    end
    else if (signed_div_divisor_tready) begin
        signed_div_divisor_tvalid_reg <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        unsigned_div_dividend_tvalid_reg <= 1'b0;
    end
    else if (exAllowin && idValidout) begin
        unsigned_div_dividend_tvalid_reg <= id2exDivBus[4] && (id2exDivBus[2] | id2exDivBus[3]);
    end
    else if (unsigned_div_dividend_tready) begin
        unsigned_div_dividend_tvalid_reg <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        unsigned_div_divisor_tvalid_reg <= 1'b0;
    end
    else if (exAllowin && idValidout) begin
        unsigned_div_divisor_tvalid_reg <= id2exDivBus[4] && (id2exDivBus[2] | id2exDivBus[3]);
    end
    else if (unsigned_div_divisor_tready) begin
        unsigned_div_divisor_tvalid_reg <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        stable_counter <= 64'b0;
    end
    else begin
        stable_counter <= stable_counter + 1'b1;
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

assign exResult = use_div ? final_div_result :
                    inst_rdcntvh_w ? stable_counter[63:32] :
                    inst_rdcntvl_w ? stable_counter[31: 0] : alu_result;
assign pc = pc_reg;
assign inst = inst_reg;
assign {rj_value, rkd_value, imm, alu_op, src1_is_pc, src2_is_imm} = alu_signals_reg;
assign {res_from_mem, gr_we, mem_we, dest, inst_rdcntvh_w, inst_rdcntvl_w} = pass_signals_reg;
assign ex2memPC = pc;
assign ex2memInst = inst;
assign ex2memResult = exResult;
assign ex2memPassBus = {is_sram_inst, res_from_mem, mem_offsets, gr_we, dest};
assign mem_offsets = alu_result[1:0];

assign {csr, csr_we, csr_num, csr_wmask, null32, exception, ecode, syscall_code, exErtnFlush} = csr_signals_reg;
assign csr_wvalue = rkd_value;
assign EXU_to_IDU_csr = csr;
assign ex2memCsrBus = {csr,               // [0]
                        csr_we,            // [1]
                        csr_num,           // [15:2], 14
                        csr_wmask,         // [47:16], 32
                        csr_wvalue,        // [79:48], 32
                        exception | isale, // [80]
                        newecode,          // [86:81], 6
                        syscall_code,      // [101:87], 15
                        exErtnFlush        // [102]
                        };

/*******************************/
/*     data sram interface     */
/*******************************/
assign is_sram_inst = (|res_from_mem || |mem_we) && exValidReg && !isale;
assign data_sram_req = is_sram_inst && memAllowin && !memStopMemAccess && !wb_ex && !ertn_flush;  // exp14 BUG: when MEM/WB has exception, do not send req
assign data_sram_wr  = |mem_we;
assign data_sram_wstrb = (~exValidReg || wb_ex || isale) ? 4'b0 :
                    mem_we[2] ? 4'b1111 :
                    mem_we[1] ? (mem_offsets[1]       ? 4'b1100 : 4'b0011):
                    mem_we[0] ? (mem_offsets == 2'b00 ? 4'b0001 :
                                    mem_offsets == 2'b01 ? 4'b0010 :
                                    mem_offsets == 2'b10 ? 4'b0100 :
                                                        4'b1000): 4'b0;
// addr and size
assign is_sram_w = mem_we[2] || res_from_mem[4];
assign is_sram_h = mem_we[1] || res_from_mem[3] || res_from_mem[1];
assign is_sram_b = mem_we[0] || res_from_mem[2] || res_from_mem[0];
assign data_sram_size = is_sram_w ? 2'b10 :
                        is_sram_h ? 2'b01 :
                        is_sram_b ? 2'b00 : 2'b10;
assign data_sram_addr = is_sram_w ? { alu_result[31:2], 2'b00 } :
                        is_sram_h ? { alu_result[31:1], 1'b0  } :
                        is_sram_b ? { alu_result              }:  alu_result;
assign data_sram_wdata = mem_we[2] ?    rkd_value :
                            mem_we[1] ? {2{rkd_value[15:0]}} :
                            mem_we[0] ? {4{rkd_value[ 7:0]}} : 32'b0;

// ALE exception
assign isale  = (~(alu_result[1:0] == 2'b0) & (mem_we[2] | res_from_mem[4]))|    // word-aligned check
                (~(alu_result[0]   == 1'b0) & (mem_we[1] | res_from_mem[3] | res_from_mem[1]));    // half-word-aligned check
assign newecode = exception ?  ecode:
                    isale     ? `ECODE_ALE:
                                `ECODE_INT;   // to keep priority of exceptions

// to IDU
assign EXU_to_IDU_gr_we = gr_we;
assign EXU_to_IDU_dest  = dest;
assign EXU_to_IDU_valid = exValidReg;
assign EXU_to_IDU_forward = exResult;
assign EXU_current_is_ld = |res_from_mem && exValidReg;

// EX status
always @(posedge clk) begin
    if (reset) begin
        exValidReg <= 1'b0;
    end
    else if (wb_ex || ertn_flush) begin
        exValidReg <= 1'b0;
    end
    else if (exAllowin) begin
        exValidReg <= idValidout;
    end
end
assign exReadygo  =  use_div        ? (signed_div_dout_valid | unsigned_div_dout_valid) :
                     is_sram_inst   ? (data_sram_addr_ok && data_sram_req) : 1'b1;
assign exValidout =  exValidReg &&  exReadygo;
assign exAllowin  = !exValidReg || (exReadygo && memAllowin);

endmodule