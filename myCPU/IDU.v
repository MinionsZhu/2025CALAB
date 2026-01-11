`include "busdef.vh"
`include "ecodes.vh"
`include "csr.vh"
module IDU(
    input  wire        clk,
    input  wire        reset,
    // ie
    input  wire        wb_ex,
    input  wire        ertn_flush,
    input  wire        isintr,  // from csr_regs.v, to attach interrupt exception

    // from IFU
    input  wire [31:0] if2idPC,
    input  wire [31:0] if2idInst,
    input  wire        isadef,
    input  wire        if_tlbr_ex,
    input  wire        if_pif_ex,
    input  wire        if_ppi_ex,

    // to IFU
    output wire        br_taken,
    output wire        br_taken_cancel,
    output wire [31:0] br_target,
    output wire        br_stall,   // tell preIF wait true br_target

    // handshaking signals with IFU
    input  wire        ifValidout,
    output wire        idAllowin,
    // handshaking signals with EXU
    input  wire        exAllowin,
    output wire        idValidout,

    // signals and data to EXU
    output wire [31:0] id2exPC,
    output wire [31:0] id2exInst,
    output wire [ 4:0] id2exDivBus,
    output wire [`CSRBUSL] id2exCsrBus,
    output wire [`ALUBUSL] id2exALUBus,
    output wire [`IDPASSBUSL] id2exPassBus,
    output wire [9:0] id2exTLBBus,

    // forwarding from EXU/MEM/WB stage
    input  wire [ 4:0] EXU_dest,
    input  wire        EXU_gr_we,
    input  wire        EXU_valid,
    input  wire [31:0] EXU_to_ID_forward,
    input  wire        EXU_current_is_ld,
    input  wire        EXU_csr,
    input  wire [ 4:0] MEM_dest,
    input  wire        MEM_gr_we,
    input  wire        MEM_valid,
    input  wire [31:0] MEM_to_ID_forward,
    input  wire        MEM_to_ID_forward_valid,
    input  wire        MEM_csr,
    input  wire [ 4:0] WB_dest,
    input  wire        WB_gr_we,
    input  wire        WB_valid,
    input  wire [31:0] WB_to_ID_forward,
    input  wire        WB_csr,

    // register file interface
    output wire [ 4:0] rf_raddr1,
    output wire [ 4:0] rf_raddr2,
    input  wire [31:0] rf_rdata1,
    input  wire [31:0] rf_rdata2,

    // refetch signal        
    input  wire        if2idRefetch,
    output wire        id2exRefetch,
    output wire        id2exChangeTLB,
    output wire        id2exChangeTLBEHI, // stall signal for EHI write
    output wire        id2exCacop,
    output wire [ 4:0] id2exCacopCode,
    output wire        id2exICacopStall
);

reg         idValidReg;
reg  [31:0] inst_reg;
reg  [31:0] pc_reg;
reg         refetch;

wire [31:0] inst;
wire [31:0] pc;

wire [14:0] alu_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire [ 4:0] res_from_mem;
wire        dst_is_r1;
wire        gr_we;
wire [ 2:0] mem_we;
wire        src_reg_is_rd;
wire        rj_eq_rd;
wire        rj_lt_rd;
wire        rj_ltu_rd;
wire [ 4:0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_slti;
wire        inst_sltui;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_ld_w;
wire        inst_ld_h;
wire        inst_ld_b;
wire        inst_ld_hu;
wire        inst_ld_bu;
wire        inst_st_w;
wire        inst_st_h;
wire        inst_st_b;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;
wire        inst_lu12i_w;
wire        inst_pcaddu12i;
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_mod_w;
wire        inst_div_wu;
wire        inst_mod_wu;

wire        inst_break;
wire        inst_syscall;
wire        inst_csrrd;
wire        inst_csrwr;
wire        inst_csrxchg;
wire        inst_ertn;

wire        inst_rdcntvl_w;
wire        inst_rdcntvh_w;
wire        inst_rdcntid_w;

wire        allinst;
wire        ine;

wire        need_ui5;
wire        need_ui12;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
wire        src2_is_4;

/////////////////////////////////////////////////////////////////////
//////                         ID regs                        ///////
/////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if (reset) begin
        inst_reg <= 32'b0;
    end
    else if(ifValidout && idAllowin) begin
        inst_reg <= if2idInst;
    end
end

always @(posedge clk) begin
    if (reset) begin
        pc_reg <= 32'b0;
    end
    else if(ifValidout && idAllowin) begin
        pc_reg <= if2idPC;
    end
end

always @(posedge clk) begin
    if (reset) begin
        refetch <= 1'b0;
    end
    else if(ifValidout && idAllowin) begin
        refetch <= if2idRefetch;
    end
end
assign id2exRefetch = refetch;

///////////////////////////////////////////////////////////////////////
//////                          decoders                        ///////
///////////////////////////////////////////////////////////////////////
assign inst = inst_reg;
assign pc   = pc_reg;

assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];

assign rd   = inst[ 4: 0];
assign rj   = inst[ 9: 5];
assign rk   = inst[14:10];

assign i12  = inst[21:10];
assign i20  = inst[24: 5];
assign i16  = inst[25:10];
assign i26  = {inst[ 9: 0], inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_mul_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_ld_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_ld_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_hu  = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
assign inst_ld_bu  = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_st_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h5];
assign inst_st_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_blt    = op_31_26_d[6'h18];
assign inst_bge    = op_31_26_d[6'h19];
assign inst_bltu   = op_31_26_d[6'h1a];
assign inst_bgeu   = op_31_26_d[6'h1b];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst[25];
assign inst_pcaddu12i = op_31_26_d[6'h07] & ~inst[25];

assign inst_break   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];
assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_csrrd   = op_31_26_d[6'h01] & inst[25:24] == 2'b00 & (rj == 5'b00000);
assign inst_csrwr   = op_31_26_d[6'h01] & inst[25:24] == 2'b00 & (rj == 5'b00001);
assign inst_csrxchg = op_31_26_d[6'h01] & inst[25:24] == 2'b00 & (rj != 5'b00000) & (rj != 5'b00001);
assign inst_ertn    = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & (rk == 5'b01110);

assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk == 5'h18) & (rj == 5'h00);
assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk == 5'h19) & (rj == 5'h00);
assign inst_rdcntid_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk == 5'h18) & (rd == 5'h00);

assign inst_tlbsrch = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & rk == 5'ha & rj == 5'h0 & rd == 5'h0;
assign inst_tlbrd   = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & rk == 5'hb & rj == 5'h0 & rd == 5'h0;
assign inst_tlbwr   = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & rk == 5'hc & rj == 5'h0 & rd == 5'h0;
assign inst_tlbfill = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & rk == 5'hd & rj == 5'h0 & rd == 5'h0;
assign inst_invtlb  = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h13];

assign inst_cacop   = op_31_26_d[6'h01] & op_25_22_d[4'h8];

assign allinst =   (inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid_w |
                    inst_add_w  | inst_sub_w  | inst_slt    | inst_sltu  |
                    inst_nor    | inst_and    | inst_or     | inst_xor   |
                    inst_sll_w  | inst_srl_w  | inst_sra_w  |
                    inst_mul_w  | inst_mulh_w | inst_mulh_wu|
                    inst_div_w  | inst_mod_w  | inst_div_wu | inst_mod_wu|
                    inst_slli_w | inst_srli_w | inst_srai_w |
                    inst_addi_w |
                    inst_slti   | inst_sltui  |
                    inst_andi   | inst_ori    | inst_xori   |
                    inst_ld_w   | inst_ld_h   | inst_ld_b   | inst_ld_hu | inst_ld_bu |
                    inst_st_w   | inst_st_h   | inst_st_b   |
                    inst_jirl   |
                    inst_b      | inst_bl     |
                    inst_beq    | inst_bne    |
                    inst_blt    | inst_bge    |
                    inst_bltu   | inst_bgeu   |
                    inst_lu12i_w|inst_pcaddu12i|
                    inst_break  | inst_syscall|
                    inst_csrrd  | inst_csrwr  | inst_csrxchg|
                    inst_ertn   | inst_tlbsrch | inst_tlbrd | inst_tlbwr  |
                    inst_tlbfill| inst_invtlb | inst_cacop );
assign invtlb_ine = (inst_invtlb & invtlb_opcode > 5'h6);
assign ine = ~(allinst) || invtlb_ine;    // if fetch address exception, then inst is invalid

assign need_ui5   = inst_slli_w | inst_srli_w | inst_srai_w;
assign need_si12  = inst_addi_w |
                    inst_ld_w | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu |
                    inst_st_w | inst_st_h | inst_st_b |
                    inst_slti | inst_sltui | inst_cacop;
assign need_ui12  = inst_andi | inst_ori | inst_xori;
assign need_si16  = inst_jirl | inst_beq | inst_bne |
                                inst_blt | inst_bge |
                                inst_bltu | inst_bgeu;
assign need_si20  = inst_lu12i_w | inst_pcaddu12i;
assign need_si26  = inst_b | inst_bl;
assign src2_is_4  = inst_jirl | inst_bl;

assign br_offs  = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                                {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu |
                    inst_st_w | inst_st_h | inst_st_b |
                    inst_csrrd | inst_csrwr | inst_csrxchg |
                    inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w;    // this is for they don't have rk
assign rj_eq_rd  = (rj_value == rkd_value);
assign rj_lt_rd  = ($signed(rj_value) < $signed(rkd_value));
assign rj_ltu_rd = (rj_value < rkd_value);

assign dst_is_r1 = inst_bl;

///////////////////////////////////////////////////////////////////////
//////                         raw dealing                      ///////
///////////////////////////////////////////////////////////////////////
wire possible_raw_exu;
wire possible_raw_mem;
wire possible_raw_wb;
wire rf1_raw_exu;
wire rf2_raw_exu;
wire rf1_raw_mem;
wire rf2_raw_mem;
wire rf1_raw_wb;
wire rf2_raw_wb;
wire use_rj;
wire use_rkd;
wire EXU_raw;
wire MEM_raw;
wire WB_raw;
wire raw;

assign possible_raw_exu = EXU_valid && EXU_gr_we && (EXU_dest != 5'b0);
assign possible_raw_mem = MEM_valid && MEM_gr_we && (MEM_dest != 5'b0);
assign possible_raw_wb  = WB_valid  && WB_gr_we  && (WB_dest  != 5'b0);
assign rf1_raw_exu = idValidReg && possible_raw_exu && use_rj  && (rf_raddr1 == EXU_dest);
assign rf2_raw_exu = idValidReg && possible_raw_exu && use_rkd && (rf_raddr2 == EXU_dest);
assign rf1_raw_mem = idValidReg && possible_raw_mem && use_rj  && (rf_raddr1 == MEM_dest);
assign rf2_raw_mem = idValidReg && possible_raw_mem && use_rkd && (rf_raddr2 == MEM_dest);
assign rf1_raw_wb  = idValidReg && possible_raw_wb  && use_rj  && (rf_raddr1 == WB_dest);
assign rf2_raw_wb  = idValidReg && possible_raw_wb  && use_rkd && (rf_raddr2 == WB_dest);
assign use_rj  = !inst_b && !inst_bl;
assign use_rkd = inst_beq || inst_bne || inst_blt || inst_bge || inst_bltu || inst_bgeu
                || inst_sub_w || inst_slt || inst_sltu
                || inst_nor || inst_and || inst_or || inst_xor
                || inst_st_w || inst_st_h || inst_st_b
                || inst_add_w
                || inst_sll_w || inst_srl_w || inst_sra_w
                || inst_mul_w || inst_mulh_w || inst_mulh_wu
                || inst_div_w || inst_mod_w || inst_div_wu || inst_mod_wu
                || inst_csrwr || inst_csrxchg
                || inst_rdcntvl_w || inst_rdcntvh_w;
assign EXU_raw = (rf1_raw_exu || rf2_raw_exu);
assign MEM_raw = (rf1_raw_mem || rf2_raw_mem);
assign WB_raw  = (rf1_raw_wb  || rf2_raw_wb);
assign raw = EXU_raw || MEM_raw || WB_raw;
///////////////////////////////////////////////////////////////////////
//////                            csr                           ///////
///////////////////////////////////////////////////////////////////////
wire        csr;
wire        csr_we;
wire [13:0] csr_num;
wire [31:0] csr_wmask;
wire [31:0] null32;
wire        excptstore;
wire [ 5:0] ecodestore;
wire        exception;
wire [ 5:0] ecode;
wire [14:0] syscall_code;
wire        idErtnFlush;
wire        ifTlbrPass;
wire        ifPpiPass;
wire        csr_raw;
reg  [ 6:0] preciseExcptReg;    // to keep precise exception info
assign csr          = inst_csrrd | inst_csrwr | inst_csrxchg | inst_rdcntid_w;  // consider rdcntid as csr instruction, for it needs to read tid.csr
assign csr_we       = inst_csrwr | inst_csrxchg;
assign csr_num      = inst_rdcntid_w ? 14'h40 : inst[23:10];
assign csr_wmask    = inst_csrxchg ? rj_value : 32'hffffffff;
assign null32       = 32'b0;
assign exception  = excptstore | inst_syscall | inst_break | ine;
assign ecode      = excptstore  ? ecodestore :
                    inst_syscall? `ECODE_SYS :
                    inst_break  ? `ECODE_BRK :
                    ine         ? `ECODE_INE :
                                  `ECODE_INT ;
assign syscall_code = inst[14:0];
assign idErtnFlush  = inst_ertn;
assign id2exCsrBus = {
    csr,            // [0]
    csr_we,         // [1]
    csr_num,        // [15:2]
    csr_wmask,      // [47:16]
    null32,         // [79:48]
    exception,      // [80]
    ecode,          // [86:81]
    syscall_code,   // [101:87]
    idErtnFlush,     // [102]
    ifTlbrPass,
    ifPpiPass
};
assign {excptstore, ecodestore} = preciseExcptReg;
always @(posedge clk) begin
    if (reset) begin
        preciseExcptReg <= 7'b0;    // to keep precise exception info
    end
    else if(idAllowin & ifValidout) begin   // for it is from if, it should be controled by idAllowin & ifValidout
        preciseExcptReg <= {isintr | isadef | if2idRefetch | if_tlbr_ex | if_pif_ex | if_ppi_ex,
                            isintr ? `ECODE_INT :
                            isadef ? `ECODE_ADE :
                            if_tlbr_ex ? `ECODE_TLBR :
                            if_pif_ex  ? `ECODE_PIF  :
                            if_ppi_ex  ? `ECODE_PPI  :
                            `ECODE_ADE};   // to keep precise exception info
    end
end
reg if_tlbr_ex_reg, if_ppi_ex_reg;
always @(posedge clk ) begin
    if(reset) begin
        if_ppi_ex_reg <= 0;
        if_tlbr_ex_reg <= 0;
    end
    else if (idAllowin & ifValidout) begin
        if_ppi_ex_reg <= if_ppi_ex;
        if_tlbr_ex_reg <= if_tlbr_ex;
    end
end
assign ifTlbrPass = if_tlbr_ex_reg;
assign ifPpiPass  = if_ppi_ex_reg;
assign csr_raw = EXU_csr && (rf1_raw_exu || rf2_raw_exu) 
            || MEM_csr && (rf1_raw_mem || rf2_raw_mem) 
            || WB_csr  && (rf1_raw_wb  || rf2_raw_wb);
///////////////////////////////////////////////////////////////////////
//////               signals to IFU concerning branch           ///////
///////////////////////////////////////////////////////////////////////
assign br_taken = (   inst_beq  &&  rj_eq_rd
                || inst_bne  && !rj_eq_rd
                || inst_blt  &&  rj_lt_rd
                || inst_bge  && !rj_lt_rd
                || inst_bltu &&  rj_ltu_rd
                || inst_bgeu && !rj_ltu_rd
                || inst_jirl
                || inst_bl
                || inst_b
                ) && idValidReg;
assign br_taken_cancel = !idReadygo ? 1'b0 : br_taken; // idValidReg is used
assign br_stall = br_taken && !idReadygo;
assign br_target = (inst_beq || inst_bne ||
                    inst_blt || inst_bge ||
                    inst_bltu || inst_bgeu ||
                    inst_bl || inst_b) ? (pc + br_offs) :
                        /*inst_jirl*/ (rj_value + jirl_offs);

///////////////////////////////////////////////////////////////////////
//////                 signals to EXU concerning ALU            ///////
///////////////////////////////////////////////////////////////////////
assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;

assign src2_is_imm   = inst_slli_w |
                    inst_srli_w |
                    inst_srai_w |
                    inst_addi_w |
                    inst_slti   |
                    inst_sltui  |
                    inst_andi   |
                    inst_ori    |
                    inst_xori   |
                    inst_ld_w   |
                    inst_ld_h   |
                    inst_ld_b   |
                    inst_ld_hu  |
                    inst_ld_bu  |
                    inst_st_w   |
                    inst_st_h   |
                    inst_st_b   |
                    inst_lu12i_w|
                    inst_pcaddu12i|
                    inst_jirl   |
                    inst_bl |
                    inst_cacop  ;

assign alu_op[ 0] = inst_add_w | inst_addi_w |
                    inst_ld_w | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu |
                    inst_st_w | inst_st_h | inst_st_b |
                    inst_jirl | inst_bl | inst_pcaddu12i | inst_cacop;
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltui;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_slli_w | inst_sll_w;
assign alu_op[ 9] = inst_srli_w | inst_srl_w;
assign alu_op[10] = inst_srai_w | inst_sra_w;
assign alu_op[11] = inst_lu12i_w;
assign alu_op[12] = inst_mul_w;
assign alu_op[13] = inst_mulh_w;
assign alu_op[14] = inst_mulh_wu;

assign imm = src2_is_4 ? 32'h4                      :
            need_si20 ? {i20[19:0], 12'b0}         :
            need_ui5  ? {27'b0,rk[4:0]}            :   
            need_si12 ? {{20{i12[11]}}, i12[11:0]} : 
            need_ui12 ? {20'b0, i12[11:0]}         :  
            32'b0 ;

assign rj_value  = rf1_raw_exu ? EXU_to_ID_forward 
                : rf1_raw_mem ? MEM_to_ID_forward 
                : rf1_raw_wb  ? WB_to_ID_forward  
                : rf_rdata1;
assign rkd_value = rf2_raw_exu ? EXU_to_ID_forward 
                : rf2_raw_mem ? MEM_to_ID_forward 
                : rf2_raw_wb  ? WB_to_ID_forward  
                : rf_rdata2;

assign id2exALUBus = {
    rj_value,      // [31:0]
    rkd_value,     // [63:32]
    imm,           // [95:64]
    alu_op,        // [110:96]
    src1_is_pc,    // [111]
    src2_is_imm    // [112]
};

////////////////////////////////////////////////////////////////////////
//////                      divider signals                      ///////
////////////////////////////////////////////////////////////////////////
wire        use_div;
wire [3:0]  div_op;
assign use_div = inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu;
assign div_op[0] = inst_div_w;
assign div_op[1] = inst_mod_w;
assign div_op[2] = inst_div_wu;
assign div_op[3] = inst_mod_wu;
assign id2exDivBus = {
    use_div,    // [4]
    div_op      // [3:0]
};

////////////////////////////////////////////////////////////////////////
//////                      other signals to EXU                 ///////
////////////////////////////////////////////////////////////////////////
assign id2exPC   = pc;
assign id2exInst = inst;

assign res_from_mem  = {inst_ld_w, inst_ld_h, inst_ld_b, inst_ld_hu, inst_ld_bu};

assign gr_we         = ~(inst_st_w | inst_st_h | inst_st_b |
                        inst_beq | inst_bne |
                        inst_blt | inst_bge |
                        inst_bltu | inst_bgeu | inst_b |
                        inst_ertn |
                        inst_break | inst_syscall |
                        inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb | inst_cacop);
assign mem_we        = {inst_st_w, inst_st_h, inst_st_b};
assign dest          = dst_is_r1 ? 5'd1 : 
                    inst_rdcntid_w ? rj : rd;    // special case for rdcntid.w

assign id2exPassBus = {
    res_from_mem,   // [4:0]
    gr_we,          // [5]
    mem_we,         // [8:6]
    dest,           // [13:9]
    inst_rdcntvh_w, // [14]
    inst_rdcntvl_w  // [15]
};

////////////////////////////////////////////////////////////////////////
//////                        TLB signals                        ///////
////////////////////////////////////////////////////////////////////////
wire [4:0] invtlb_opcode;
assign invtlb_opcode = rd;
assign id2exTLBBus = {
    inst_tlbsrch,   // [0]
    inst_tlbrd,     // [1]
    inst_tlbwr,     // [2]
    inst_tlbfill,   // [3]
    inst_invtlb,    // [4]
    invtlb_opcode   // [9:5]
};
assign id2exChangeTLB = (inst_tlbwr | inst_tlbfill | inst_tlbrd | inst_invtlb |
                        (csr_we & (csr_num == `CSR_CRMD 
                        | csr_num == `CSR_DMW0 
                        | csr_num == `CSR_DMW1
                        | csr_num == `CSR_ASID
                        ))) & idValidReg;
assign id2exChangeTLBEHI = (csr_we & (csr_num == `CSR_TLBEHI)) & idValidReg;

////////////////////////////////////////////////////////////////////////
//////                      cacop signals                        ///////
////////////////////////////////////////////////////////////////////////
assign id2exCacop = inst_cacop & idValidReg;
assign id2exCacopCode = rd;
assign id2exICacopStall = rd[2:0] == 3'b000 && id2exCacop; // cacop.opcode[2:0] == 3'b000 means icacop.stall

////////////////////////////////////////////////////////////////////////
//////                      register interface                   ///////
////////////////////////////////////////////////////////////////////////
assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd : rk;

// ID status
always @(posedge clk) begin
    if (reset) begin
        idValidReg <= 1'b0;
    end
    else if(br_taken_cancel | wb_ex | ertn_flush)begin
        idValidReg <= 1'b0;
    end
    else if(idAllowin) begin
        idValidReg <= ifValidout;
    end
end
wire   block;
// exp14 BUG: !MEM_to_ID_forward_valid also need && MEM_raw
assign block = !ertn_flush && !wb_ex && idValidReg && ((EXU_current_is_ld && EXU_raw) || csr_raw || (!MEM_to_ID_forward_valid && MEM_raw));

assign idValidout =  idValidReg && idReadygo;
assign idReadygo  = !block;
assign idAllowin  = !idValidReg || (idReadygo && exAllowin);

endmodule