`include "busdef.vh"
`include "csr.vh"
`include "ecodes.vh"
module csr_regs (
    input           clk,
    input           reset,

    input [`CSRNUML] csr_num,

    input           csr_re,
    input           csr_we,
    input   [31:0]  csr_wmask,
    output  [31:0]  csr_rvalue,
    input   [31:0]  csr_wvalue,

    output  [31:0]  ertn_pc,
    output  [31:0]  ex_entry,
    input           wb_ex,
    input   [31:0]  wb_pc,
    input           ertn_flush,
    input   [ 5:0]  wb_ecode,
    input   [ 8:0]  wb_esubcode,
    input   [31:0]  wb_vaddr,
    input   [31:0]  coreid_in,

    output          isintr, // to IDU
    input   [ 7:0]  hw_int_in,
    input           ipi_int_in,

    // tlbsrch & tlbrd write CSR
    // from WB
    input           tlbsrch_found,
    input   [ 3:0]  tlbsrch_index,
    input           is_tlbrd,
    input           is_tlbsrch,
    // from TLB read port
    input  wire        r_e,
    input  wire [18:0] r_vppn,
    input  wire [ 5:0] r_ps,
    input  wire [ 9:0] r_asid,
    input  wire        r_g,
    input  wire [19:0] r_ppn0,
    input  wire [ 1:0] r_plv0,
    input  wire [ 1:0] r_mat0,
    input  wire        r_d0,
    input  wire        r_v0,
    input  wire [19:0] r_ppn1,
    input  wire [ 1:0] r_plv1,
    input  wire [ 1:0] r_mat1,
    input  wire        r_d1,
    input  wire        r_v1,

    // tlbwr & tlbfill read CSR
    // to TLB write port
    output  wire        w_e,
    output  wire [18:0] w_vppn,
    output  wire [ 5:0] w_ps,
    output  wire [ 9:0] w_asid,
    output  wire        w_g,
    output  wire [19:0] w_ppn0,
    output  wire [ 1:0] w_plv0,
    output  wire [ 1:0] w_mat0,
    output  wire        w_d0,
    output  wire        w_v0,
    output  wire [19:0] w_ppn1,
    output  wire [ 1:0] w_plv1,
    output  wire [ 1:0] w_mat1,
    output  wire        w_d1,
    output  wire        w_v1,
    // use for TLB read/write
    output  wire [ 3:0] tlb_index,

    // for tlb exception
    input           tlbr_ex,
    input           if_tlb_refill, // refill occurs in IFU
    input           if_plv_ex, // plv exception occurs in IFU
    // for addr translation
    output  [ 1:0]  crmd_plv,
    output          crmd_da,
    output          crmd_pg,
    output  [ 1:0]  crmd_datf,
    output  [ 1:0]  crmd_datm,

    output          dmw0_plv0,
    output          dmw0_plv3,
    output  [ 1:0]  dmw0_mat,
    output  [ 2:0]  dmw0_pseg,
    output  [ 2:0]  dmw0_vseg,

    output          dmw1_plv0,
    output          dmw1_plv3,
    output  [ 1:0]  dmw1_mat,
    output  [ 2:0]  dmw1_pseg,
    output  [ 2:0]  dmw1_vseg,

    output  [ 9:0]  asid,
    // for 
    output  [ 5:0]  stat_ecode,
    output  [31:0]  ex_tlbrentry
);
    // CRMD CSR
    reg  [ 1:0] csr_crmd_plv;
    reg         csr_crmd_ie;
    reg         csr_crmd_da;
    reg         csr_crmd_pg;
    reg  [ 1:0] csr_crmd_datf;
    reg  [ 1:0] csr_crmd_datm;
    // PRMD CSR
    reg  [ 1:0] csr_prmd_pplv;
    reg         csr_prmd_pie;
    // ECFG CSR
    reg  [12:0] csr_ecfg_lie;
    // ESTAT CSR
    reg  [12:0] csr_estat_is;
    reg  [ 5:0] csr_estat_ecode;
    reg  [ 8:0] csr_estat_esubcode;
    // ERA CSR
    reg  [31:0] csr_era_pc;
    // BADV CSR
    reg  [31:0] csr_badv_vaddr;
    // EENTRY CSR
    reg  [25:0] csr_eentry_va;
    // SAVE CSR
    reg  [31:0] csr_save0;
    reg  [31:0] csr_save1;
    reg  [31:0] csr_save2;
    reg  [31:0] csr_save3;
    // TID CSR
    reg  [31:0] csr_tid;
    // TCFG CSR & wires
    reg         csr_tcfg_en;
    reg         csr_tcfg_periodic;
    reg  [29:0] csr_tcfg_initval;
    wire [31:0] tcfg_next_value;
    wire [31:0] csr_tval;
    reg  [31:0] timer_cnt;
    // TLBIDX CSR
    reg  [ 3:0] csr_tlbidx_index;
    reg         csr_tlbidx_ne;
    reg  [ 5:0] csr_tlbidx_ps;
    // TLBEHI CSR
    reg  [18:0] csr_tlbehi_vppn;
    // TLBELO0 CSR
    reg  [19:0] csr_tlbelo0_ppn;
    reg  [ 1:0] csr_tlbelo0_plv;
    reg  [ 1:0] csr_tlbelo0_mat;
    reg         csr_tlbelo0_d;
    reg         csr_tlbelo0_v;
    reg         csr_tlbelo0_g;
    // TLBELO1 CSR
    reg  [19:0] csr_tlbelo1_ppn;
    reg  [ 1:0] csr_tlbelo1_plv;
    reg  [ 1:0] csr_tlbelo1_mat;
    reg         csr_tlbelo1_d;
    reg         csr_tlbelo1_v;
    reg         csr_tlbelo1_g;
    // ASID CSR
    reg  [ 9:0] csr_asid_asid;
    // TLBRENTRY CSR
    reg  [25:0] csr_tlbrentry_pa;
    // DMW CSR
    reg         csr_dmw0_plv0;
    reg         csr_dmw0_plv3;
    reg  [ 1:0] csr_dmw0_mat;
    reg  [ 2:0] csr_dmw0_pseg;
    reg  [ 2:0] csr_dmw0_vseg;
    reg         csr_dmw1_plv0;
    reg         csr_dmw1_plv3;
    reg  [ 1:0] csr_dmw1_mat;
    reg  [ 2:0] csr_dmw1_pseg;
    reg  [ 2:0] csr_dmw1_vseg;

    // CRMD
    always @(posedge clk) begin
        if(reset) begin
            csr_crmd_plv <= 2'b0;
            csr_crmd_ie  <= 1'b0;
        end
        else if(wb_ex) begin
            csr_crmd_plv <= 2'b0;
            csr_crmd_ie  <= 1'b0;
        end
        else if(ertn_flush) begin
            csr_crmd_plv <= csr_prmd_pplv;
            csr_crmd_ie  <= csr_prmd_pie;
        end
        else if(csr_we && (csr_num == `CSR_CRMD)) begin
            csr_crmd_plv <=  csr_wmask[`CSR_CRMD_PLV]&csr_wvalue[`CSR_CRMD_PLV]
                          | ~csr_wmask[`CSR_CRMD_PLV]&csr_crmd_plv;
            csr_crmd_ie  <=  csr_wmask[`CSR_CRMD_IE]&csr_wvalue[`CSR_CRMD_IE]
                          | ~csr_wmask[`CSR_CRMD_IE]&csr_crmd_ie;
        end
    end
    // DA, PG
    always @(posedge clk) begin
        if(reset) begin
            csr_crmd_da <= 1'b1;
            csr_crmd_pg <= 1'b0;
        end
        else if(csr_we && (csr_num == `CSR_CRMD)) begin
            csr_crmd_da <=  csr_wmask[`CSR_CRMD_DA]&csr_wvalue[`CSR_CRMD_DA]
                         | ~csr_wmask[`CSR_CRMD_DA]&csr_crmd_da;
            csr_crmd_pg <=  csr_wmask[`CSR_CRMD_PG]&csr_wvalue[`CSR_CRMD_PG]
                         | ~csr_wmask[`CSR_CRMD_PG]&csr_crmd_pg;
        end
        else if(tlbr_ex) begin
            csr_crmd_da <= 1'b1;
            csr_crmd_pg <= 1'b0;
        end
        else if(ertn_flush && csr_estat_ecode == `ECODE_TLBR) begin
            csr_crmd_da <= 1'b0;
            csr_crmd_pg <= 1'b1;
        end
    end
    // DATF, DATM
    always @(posedge clk) begin        
        if(reset) begin
            csr_crmd_datf <= 2'b00;
            csr_crmd_datm <= 2'b00;
        end
        else if(csr_we && (csr_num == `CSR_CRMD)) begin
            csr_crmd_datf <=  csr_wmask[`CSR_CRMD_DATF]&csr_wvalue[`CSR_CRMD_DATF]
                          | ~csr_wmask[`CSR_CRMD_DATF]&csr_crmd_datf;
            csr_crmd_datm <=  csr_wmask[`CSR_CRMD_DATM]&csr_wvalue[`CSR_CRMD_DATM]
                          | ~csr_wmask[`CSR_CRMD_DATM]&csr_crmd_datm;
        end
    end
    // PRMD
    always @(posedge clk) begin
        if(wb_ex) begin
            csr_prmd_pplv <= csr_crmd_plv;
            csr_prmd_pie  <= csr_crmd_ie;
        end
        else if(csr_we && (csr_num == `CSR_PRMD)) begin
            csr_prmd_pplv <=  csr_wmask[`CSR_PRMD_PPLV]&csr_wvalue[`CSR_PRMD_PPLV]
                           | ~csr_wmask[`CSR_PRMD_PPLV]&csr_prmd_pplv;
            csr_prmd_pie  <=  csr_wmask[`CSR_PRMD_PIE]&csr_wvalue[`CSR_PRMD_PIE]
                           | ~csr_wmask[`CSR_PRMD_PIE]&csr_prmd_pie;
        end
    end
    // ECFG
    always @(posedge clk) begin
        if(reset) begin
            csr_ecfg_lie <= 13'b0;
        end
        else if(csr_we && (csr_num == `CSR_ECFG)) begin
            csr_ecfg_lie <=  csr_wmask[`CSR_ECFG_LIE]&13'h1bff&csr_wvalue[`CSR_ECFG_LIE]
                          | ~csr_wmask[`CSR_ECFG_LIE]&13'h1bff&csr_ecfg_lie;
        end
    end
    // ESTAT
    always @(posedge clk) begin
        if(reset) begin
            csr_estat_is[1:0] <= 2'b0;
        end
        else if(csr_we && (csr_num == `CSR_ESTAT)) begin
            csr_estat_is[1:0] <=  csr_wmask[`CSR_ESTAT_IS_1_0]&csr_wvalue[`CSR_ESTAT_IS_1_0]
                               | ~csr_wmask[`CSR_ESTAT_IS_1_0]&csr_estat_is[1:0];
        end
        csr_estat_is[9:2] <= hw_int_in[7:0];
        csr_estat_is[10]  <= 1'b0;

        if(timer_cnt[31:0] == 32'b0) begin
            csr_estat_is[11] <= 1'b1;
        end
        else if (csr_we && (csr_num == `CSR_TICLR) && csr_wmask[`CSR_TICLR_CLR] && csr_wvalue[`CSR_TICLR_CLR]) begin
            csr_estat_is[11] <= 1'b0;
        end

        csr_estat_is[12]  <= ipi_int_in;
    end
    always @(posedge clk) begin
        if(wb_ex) begin
            csr_estat_ecode    <= wb_ecode;
            csr_estat_esubcode <= wb_esubcode;
        end
    end
    // ERA
    always @(posedge clk) begin
        if(wb_ex) begin
            csr_era_pc <= wb_pc;    // inst which attached with exception won't be executed before eret
        end
        else if(csr_we && (csr_num == `CSR_ERA)) begin
            csr_era_pc <=  csr_wmask[`CSR_ERA_PC]&csr_wvalue[`CSR_ERA_PC]
                        | ~csr_wmask[`CSR_ERA_PC]&csr_era_pc;
        end
    end
    // BADV
    wire wb_ex_addr;
    assign wb_ex_addr = (wb_ecode == `ECODE_ADE || wb_ecode == `ECODE_ALE || wb_ecode == `ECODE_PIL || 
                         wb_ecode == `ECODE_PIS || wb_ecode == `ECODE_PIF || wb_ecode == `ECODE_PME ||
                         wb_ecode == `ECODE_PPI || wb_ecode == `ECODE_TLBR);
    always @(posedge clk) begin
        if(wb_ex && wb_ex_addr) begin
            csr_badv_vaddr <=  (( wb_ecode == `ECODE_ADE && 
                                  wb_esubcode == `ESUBCODE_ADEF) ||
                                 (wb_ecode == `ECODE_PIF)||
                                 (wb_ecode == `ECODE_PPI && if_plv_ex)||
                                 (wb_ecode == `ECODE_TLBR && if_tlb_refill))
                                ? wb_pc : wb_vaddr;
        end
    end
    // EENTRY
    always @(posedge clk) begin
        if(csr_we && (csr_num == `CSR_EENTRY)) begin
            csr_eentry_va <=  csr_wmask[`CSR_EENTRY_VA]&csr_wvalue[`CSR_EENTRY_VA]
                           | ~csr_wmask[`CSR_EENTRY_VA]&csr_eentry_va;
        end
    end
    // SAVE0-3
    always @(posedge clk) begin
        if(csr_we && (csr_num == `CSR_SAVE0)) begin
            csr_save0 <=  csr_wmask[`CSR_SAVE0_DATA]&csr_wvalue[`CSR_SAVE0_DATA]
                       | ~csr_wmask[`CSR_SAVE0_DATA]&csr_save0;
        end
        if(csr_we && (csr_num == `CSR_SAVE1)) begin
            csr_save1 <=  csr_wmask[`CSR_SAVE1_DATA]&csr_wvalue[`CSR_SAVE1_DATA]
                       | ~csr_wmask[`CSR_SAVE1_DATA]&csr_save1;
        end
        if(csr_we && (csr_num == `CSR_SAVE2)) begin
            csr_save2 <=  csr_wmask[`CSR_SAVE2_DATA]&csr_wvalue[`CSR_SAVE2_DATA]
                       | ~csr_wmask[`CSR_SAVE2_DATA]&csr_save2;
        end
        if(csr_we && (csr_num == `CSR_SAVE3)) begin
            csr_save3 <=  csr_wmask[`CSR_SAVE3_DATA]&csr_wvalue[`CSR_SAVE3_DATA]
                       | ~csr_wmask[`CSR_SAVE3_DATA]&csr_save3;
        end
    end
    // TID
    always @(posedge clk) begin
        if (reset) begin
            csr_tid <= coreid_in;
        end
        else if (csr_we && (csr_num == `CSR_TID)) begin
            csr_tid <=  csr_wmask[`CSR_TID_TID]&csr_wvalue[`CSR_TID_TID]
                     | ~csr_wmask[`CSR_TID_TID]&csr_tid;
        end
    end
    // TCFG
    always @(posedge clk) begin
        if(reset) begin
            csr_tcfg_en <= 1'b0;
        end
        else if(csr_we && (csr_num == `CSR_TCFG)) begin
            csr_tcfg_en <=  csr_wmask[`CSR_TCFG_EN]&csr_wvalue[`CSR_TCFG_EN]
                         | ~csr_wmask[`CSR_TCFG_EN]&csr_tcfg_en;
        end

        if(csr_we && (csr_num == `CSR_TCFG)) begin
            csr_tcfg_periodic <=  csr_wmask[`CSR_TCFG_PERIODIC]&csr_wvalue[`CSR_TCFG_PERIODIC]
                               | ~csr_wmask[`CSR_TCFG_PERIODIC]&csr_tcfg_periodic;
            csr_tcfg_initval  <=  csr_wmask[`CSR_TCFG_INITVAL]&csr_wvalue[`CSR_TCFG_INITVAL]
                               | ~csr_wmask[`CSR_TCFG_INITVAL]&csr_tcfg_initval;
        end
    end
    // TVAL
    assign tcfg_next_value =  csr_wmask[31:0] & csr_wvalue[31:0]
                           | ~csr_wmask[31:0] &{csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
    always @(posedge clk) begin
        if(reset) begin
            timer_cnt <= 32'hffffffff;
        end
        else if(csr_we && (csr_num == `CSR_TCFG) && tcfg_next_value[`CSR_TCFG_EN]) begin
            timer_cnt <= {tcfg_next_value[`CSR_TCFG_INITVAL], 2'b0};
        end
        else if(csr_tcfg_en && (timer_cnt != 32'hffffffff)) begin
            if(timer_cnt == 32'b0 && csr_tcfg_periodic) begin
                timer_cnt <= {csr_tcfg_initval, 2'b0};
            end
            else begin
                timer_cnt <= timer_cnt - 1;
            end
        end
    end
    assign csr_tval = timer_cnt;
    // TLBIDX
    always @(posedge clk) begin
        if(reset) begin
            csr_tlbidx_index <= 4'b0;
            csr_tlbidx_ne    <= 1'b1;
            csr_tlbidx_ps    <= 6'b0;
        end
        else if(is_tlbsrch) begin
            csr_tlbidx_index <= (tlbsrch_found) ? tlbsrch_index : 4'b0;
            csr_tlbidx_ne    <= ~tlbsrch_found;
        end
        else if(is_tlbrd) begin
            csr_tlbidx_ps <= (r_e) ? r_ps : 6'b0;
            csr_tlbidx_ne <= ~r_e;
        end
        else if(csr_we && (csr_num == `CSR_TLBIDX)) begin
            csr_tlbidx_index <=  csr_wmask[`CSR_TLBIDX_INDEX]&csr_wvalue[`CSR_TLBIDX_INDEX]
                              | ~csr_wmask[`CSR_TLBIDX_INDEX]&csr_tlbidx_index;
            csr_tlbidx_ne    <=  csr_wmask[`CSR_TLBIDX_NE]&csr_wvalue[`CSR_TLBIDX_NE]
                              | ~csr_wmask[`CSR_TLBIDX_NE]&csr_tlbidx_ne;
            csr_tlbidx_ps    <=  csr_wmask[`CSR_TLBIDX_PS]&csr_wvalue[`CSR_TLBIDX_PS]
                              | ~csr_wmask[`CSR_TLBIDX_PS]&csr_tlbidx_ps;
        end
    end
    assign tlb_index = csr_tlbidx_index;
    assign w_ps = csr_tlbidx_ps;
    assign w_e = ~csr_tlbidx_ne;
    // TLBEHI
    wire tlbehi_ex;
    assign tlbehi_ex = (wb_ecode == `ECODE_PIL) || (wb_ecode == `ECODE_PIS) || (wb_ecode == `ECODE_PIF) || 
                       (wb_ecode == `ECODE_PME) || (wb_ecode == `ECODE_PPI) || (wb_ecode == `ECODE_TLBR);
    always @(posedge clk) begin
        if(reset) begin
            csr_tlbehi_vppn <= 19'b0;
        end
        else if(is_tlbrd) begin
            csr_tlbehi_vppn <= (r_e) ? r_vppn : 19'b0;
        end
        else if(csr_we && (csr_num == `CSR_TLBEHI)) begin
            csr_tlbehi_vppn <=  csr_wmask[`CSR_TLBEHI_VPPN]&csr_wvalue[`CSR_TLBEHI_VPPN]
                            | ~csr_wmask[`CSR_TLBEHI_VPPN]&csr_tlbehi_vppn;
        end
        else if(wb_ex && tlbehi_ex) begin
            csr_tlbehi_vppn <= ((wb_ecode == `ECODE_PIF) ||
                                (wb_ecode == `ECODE_PPI && if_plv_ex) ||
                                (wb_ecode == `ECODE_TLBR && if_tlb_refill))
                                ? wb_pc[31:13] : wb_vaddr[31:13];
        end
    end
    assign w_vppn = csr_tlbehi_vppn;
    // TLBELO0
    always @(posedge clk) begin
        if(reset) begin
            csr_tlbelo0_ppn <= 20'b0;
            csr_tlbelo0_plv <= 2'b0;
            csr_tlbelo0_mat <= 2'b0;
            csr_tlbelo0_d   <= 1'b0;
            csr_tlbelo0_v   <= 1'b0;
            csr_tlbelo0_g   <= 1'b0;
        end
        else if(is_tlbrd) begin
            csr_tlbelo0_ppn <= (r_e) ? r_ppn0 : 20'b0;
            csr_tlbelo0_plv <= (r_e) ? r_plv0 : 2'b0;
            csr_tlbelo0_mat <= (r_e) ? r_mat0 : 2'b0;
            csr_tlbelo0_d   <= (r_e) ? r_d0 : 1'b0;
            csr_tlbelo0_v   <= (r_e) ? r_v0 : 1'b0;
            csr_tlbelo0_g   <= (r_e) ? r_g : 1'b0;
        end
        else if(csr_we && (csr_num == `CSR_TLBELO0)) begin
            csr_tlbelo0_ppn <=  csr_wmask[`CSR_TLBELO_PPN]&csr_wvalue[`CSR_TLBELO_PPN]
                            | ~csr_wmask[`CSR_TLBELO_PPN]&csr_tlbelo0_ppn;
            csr_tlbelo0_plv <=  csr_wmask[`CSR_TLBELO_PLV]&csr_wvalue[`CSR_TLBELO_PLV]
                            | ~csr_wmask[`CSR_TLBELO_PLV]&csr_tlbelo0_plv;
            csr_tlbelo0_mat <=  csr_wmask[`CSR_TLBELO_MAT]&csr_wvalue[`CSR_TLBELO_MAT]
                            | ~csr_wmask[`CSR_TLBELO_MAT]&csr_tlbelo0_mat;
            csr_tlbelo0_d   <=  csr_wmask[`CSR_TLBELO_D]&csr_wvalue[`CSR_TLBELO_D]
                            | ~csr_wmask[`CSR_TLBELO_D]&csr_tlbelo0_d;
            csr_tlbelo0_v   <=  csr_wmask[`CSR_TLBELO_V]&csr_wvalue[`CSR_TLBELO_V]
                            | ~csr_wmask[`CSR_TLBELO_V]&csr_tlbelo0_v;
            csr_tlbelo0_g   <=  csr_wmask[`CSR_TLBELO_G]&csr_wvalue[`CSR_TLBELO_G]
                            | ~csr_wmask[`CSR_TLBELO_G]&csr_tlbelo0_g;
        end
    end
    assign w_ppn0 = csr_tlbelo0_ppn;
    assign w_plv0 = csr_tlbelo0_plv;
    assign w_mat0 = csr_tlbelo0_mat;
    assign w_d0   = csr_tlbelo0_d;
    assign w_v0   = csr_tlbelo0_v;
    // TLBELO1
    always @(posedge clk) begin
        if(reset) begin
            csr_tlbelo1_ppn <= 20'b0;
            csr_tlbelo1_plv <= 2'b0;
            csr_tlbelo1_mat <= 2'b0;
            csr_tlbelo1_d   <= 1'b0;
            csr_tlbelo1_v   <= 1'b0;
            csr_tlbelo1_g   <= 1'b0;
        end
        else if(is_tlbrd) begin
            csr_tlbelo1_ppn <= (r_e) ? r_ppn1 : 20'b0;
            csr_tlbelo1_plv <= (r_e) ? r_plv1 : 2'b0;
            csr_tlbelo1_mat <= (r_e) ? r_mat1 : 2'b0;
            csr_tlbelo1_d   <= (r_e) ? r_d1 : 1'b0;
            csr_tlbelo1_v   <= (r_e) ? r_v1 : 1'b0;
            csr_tlbelo1_g   <= (r_e) ? r_g : 1'b0;
        end
        else if(csr_we && (csr_num == `CSR_TLBELO1)) begin
            csr_tlbelo1_ppn <=  csr_wmask[`CSR_TLBELO_PPN]&csr_wvalue[`CSR_TLBELO_PPN]
                            | ~csr_wmask[`CSR_TLBELO_PPN]&csr_tlbelo1_ppn;
            csr_tlbelo1_plv <=  csr_wmask[`CSR_TLBELO_PLV]&csr_wvalue[`CSR_TLBELO_PLV]
                            | ~csr_wmask[`CSR_TLBELO_PLV]&csr_tlbelo1_plv;
            csr_tlbelo1_mat <=  csr_wmask[`CSR_TLBELO_MAT]&csr_wvalue[`CSR_TLBELO_MAT]
                            | ~csr_wmask[`CSR_TLBELO_MAT]&csr_tlbelo1_mat;
            csr_tlbelo1_d   <=  csr_wmask[`CSR_TLBELO_D]&csr_wvalue[`CSR_TLBELO_D]
                            | ~csr_wmask[`CSR_TLBELO_D]&csr_tlbelo1_d;
            csr_tlbelo1_v   <=  csr_wmask[`CSR_TLBELO_V]&csr_wvalue[`CSR_TLBELO_V]
                            | ~csr_wmask[`CSR_TLBELO_V]&csr_tlbelo1_v;
            csr_tlbelo1_g   <=  csr_wmask[`CSR_TLBELO_G]&csr_wvalue[`CSR_TLBELO_G]
                            | ~csr_wmask[`CSR_TLBELO_G]&csr_tlbelo1_g;
        end
    end
    assign w_g    = csr_tlbelo0_g & csr_tlbelo1_g;
    assign w_ppn1 = csr_tlbelo1_ppn;
    assign w_plv1 = csr_tlbelo1_plv;
    assign w_mat1 = csr_tlbelo1_mat;
    assign w_d1   = csr_tlbelo1_d;
    assign w_v1   = csr_tlbelo1_v;
    // ASID
    always @(posedge clk) begin
        if(reset) begin
            csr_asid_asid <= 10'b0;
        end
        else if(is_tlbrd) begin
            csr_asid_asid <= (r_e) ? r_asid : 10'b0;
        end
        else if(csr_we && (csr_num == `CSR_ASID)) begin
            csr_asid_asid <=  csr_wmask[`CSR_ASID_ASID]&csr_wvalue[`CSR_ASID_ASID]
                          | ~csr_wmask[`CSR_ASID_ASID]&csr_asid_asid;
        end
    end
    assign w_asid = csr_asid_asid;
    // TLBRENTRY
    always @(posedge clk) begin
        if(reset) begin
            csr_tlbrentry_pa <= 26'b0;
        end
        else if(csr_we && (csr_num == `CSR_TLBRENTRY)) begin
            csr_tlbrentry_pa <=  csr_wmask[`CSR_TLBRENTRY_PA]&csr_wvalue[`CSR_TLBRENTRY_PA]
                             | ~csr_wmask[`CSR_TLBRENTRY_PA]&csr_tlbrentry_pa;
        end
    end
    
    // DMW0
    always @(posedge clk) begin
        if(reset) begin
            csr_dmw0_plv0 <= 1'b0;
            csr_dmw0_plv3 <= 1'b0;
            csr_dmw0_mat  <= 2'b0;
            csr_dmw0_pseg <= 3'b0;
            csr_dmw0_vseg <= 3'b0;
        end
        else if(csr_we && (csr_num == `CSR_DMW0)) begin
            csr_dmw0_plv0 <=  csr_wmask[`CSR_DMW_PLV0]&csr_wvalue[`CSR_DMW_PLV0]
                          | ~csr_wmask[`CSR_DMW_PLV0]&csr_dmw0_plv0;
            csr_dmw0_plv3 <=  csr_wmask[`CSR_DMW_PLV3]&csr_wvalue[`CSR_DMW_PLV3]
                          | ~csr_wmask[`CSR_DMW_PLV3]&csr_dmw0_plv3;
            csr_dmw0_mat  <=  csr_wmask[`CSR_DMW_MAT]&csr_wvalue[`CSR_DMW_MAT]
                          | ~csr_wmask[`CSR_DMW_MAT]&csr_dmw0_mat;
            csr_dmw0_pseg <=  csr_wmask[`CSR_DMW_PSEG]&csr_wvalue[`CSR_DMW_PSEG]
                          | ~csr_wmask[`CSR_DMW_PSEG]&csr_dmw0_pseg;
            csr_dmw0_vseg <=  csr_wmask[`CSR_DMW_VSEG]&csr_wvalue[`CSR_DMW_VSEG]
                          | ~csr_wmask[`CSR_DMW_VSEG]&csr_dmw0_vseg;
        end
    end
    // DMW1
    always @(posedge clk) begin
        if(reset) begin
            csr_dmw1_plv0 <= 1'b0;
            csr_dmw1_plv3 <= 1'b0;
            csr_dmw1_mat  <= 2'b0;
            csr_dmw1_pseg <= 3'b0;
            csr_dmw1_vseg <= 3'b0;
        end
        else if(csr_we && (csr_num == `CSR_DMW1)) begin
            csr_dmw1_plv0 <=  csr_wmask[`CSR_DMW_PLV0]&csr_wvalue[`CSR_DMW_PLV0]
                          | ~csr_wmask[`CSR_DMW_PLV0]&csr_dmw1_plv0;
            csr_dmw1_plv3 <=  csr_wmask[`CSR_DMW_PLV3]&csr_wvalue[`CSR_DMW_PLV3]
                          | ~csr_wmask[`CSR_DMW_PLV3]&csr_dmw1_plv3;
            csr_dmw1_mat  <=  csr_wmask[`CSR_DMW_MAT]&csr_wvalue[`CSR_DMW_MAT]
                          | ~csr_wmask[`CSR_DMW_MAT]&csr_dmw1_mat;
            csr_dmw1_pseg <=  csr_wmask[`CSR_DMW_PSEG]&csr_wvalue[`CSR_DMW_PSEG]
                          | ~csr_wmask[`CSR_DMW_PSEG]&csr_dmw1_pseg;
            csr_dmw1_vseg <=  csr_wmask[`CSR_DMW_VSEG]&csr_wvalue[`CSR_DMW_VSEG]
                          | ~csr_wmask[`CSR_DMW_VSEG]&csr_dmw1_vseg;
        end
    end

    // read CSR value
    wire    [31:0]  csr_crmd_rvalue;
    wire    [31:0]  csr_prmd_rvalue;
    wire    [31:0]  csr_ecfg_rvalue;
    wire    [31:0]  csr_estat_rvalue;
    wire    [31:0]  csr_era_rvalue;
    wire    [31:0]  csr_badv_rvalue;
    wire    [31:0]  csr_eentry_rvalue;
    wire    [31:0]  csr_save0_rvalue;
    wire    [31:0]  csr_save1_rvalue;
    wire    [31:0]  csr_save2_rvalue;
    wire    [31:0]  csr_save3_rvalue;
    wire    [31:0]  csr_tid_rvalue;
    wire    [31:0]  csr_tcfg_rvalue;
    wire    [31:0]  csr_tval_rvalue;
    wire    [31:0]  csr_tlbidx_rvalue;
    wire    [31:0]  csr_tlbehi_rvalue;
    wire    [31:0]  csr_tlbelo0_rvalue;
    wire    [31:0]  csr_tlbelo1_rvalue;
    wire    [31:0]  csr_asid_rvalue;
    wire    [31:0]  csr_tlbrentry_rvalue;
    assign csr_crmd_rvalue   = {23'b0, csr_crmd_datm, csr_crmd_datf, csr_crmd_pg, csr_crmd_da, csr_crmd_ie, csr_crmd_plv};
    assign csr_prmd_rvalue   = {29'b0, csr_prmd_pie, csr_prmd_pplv};
    assign csr_ecfg_rvalue   = {19'b0, csr_ecfg_lie};
    assign csr_estat_rvalue  = { 1'b0, csr_estat_esubcode, csr_estat_ecode, 3'b0, csr_estat_is};
    assign csr_era_rvalue    = csr_era_pc;
    assign csr_badv_rvalue   = csr_badv_vaddr;
    assign csr_eentry_rvalue = {csr_eentry_va, 6'b0};
    assign csr_save0_rvalue  = csr_save0;
    assign csr_save1_rvalue  = csr_save1;
    assign csr_save2_rvalue  = csr_save2;
    assign csr_save3_rvalue  = csr_save3;
    assign csr_tid_rvalue    = csr_tid;
    assign csr_tcfg_rvalue   = {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
    assign csr_tval_rvalue   = csr_tval;
    assign csr_tlbidx_rvalue = {csr_tlbidx_ne, 1'b0, csr_tlbidx_ps, 20'b0, csr_tlbidx_index};
    assign csr_tlbehi_rvalue = {csr_tlbehi_vppn, 13'b0};
    assign csr_tlbelo0_rvalue   = {4'b0, csr_tlbelo0_ppn, 1'b0, csr_tlbelo0_g, csr_tlbelo0_mat, csr_tlbelo0_plv, csr_tlbelo0_d, csr_tlbelo0_v};
    assign csr_tlbelo1_rvalue   = {4'b0, csr_tlbelo1_ppn, 1'b0, csr_tlbelo1_g, csr_tlbelo1_mat, csr_tlbelo1_plv, csr_tlbelo1_d, csr_tlbelo1_v};
    assign csr_asid_rvalue      = {8'b0, 8'd10, 6'b0, csr_asid_asid};
    assign csr_tlbrentry_rvalue = {csr_tlbrentry_pa, 6'b0};
    assign csr_rvalue = {32{csr_num == `CSR_CRMD}}   & csr_crmd_rvalue   |
                        {32{csr_num == `CSR_PRMD}}   & csr_prmd_rvalue   |
                        {32{csr_num == `CSR_ECFG}}   & csr_ecfg_rvalue   |
                        {32{csr_num == `CSR_ESTAT}}  & csr_estat_rvalue  |
                        {32{csr_num == `CSR_ERA}}    & csr_era_rvalue    |
                        {32{csr_num == `CSR_BADV}}   & csr_badv_rvalue   |
                        {32{csr_num == `CSR_EENTRY}} & csr_eentry_rvalue |
                        {32{csr_num == `CSR_SAVE0}}  & csr_save0_rvalue  |
                        {32{csr_num == `CSR_SAVE1}}  & csr_save1_rvalue  |
                        {32{csr_num == `CSR_SAVE2}}  & csr_save2_rvalue  |
                        {32{csr_num == `CSR_SAVE3}}  & csr_save3_rvalue  |
                        {32{csr_num == `CSR_TID}}    & csr_tid_rvalue    |
                        {32{csr_num == `CSR_TCFG}}   & csr_tcfg_rvalue   |
                        {32{csr_num == `CSR_TVAL}}   & csr_tval_rvalue   |
                        {32{csr_num == `CSR_TLBIDX}} & csr_tlbidx_rvalue |
                        {32{csr_num == `CSR_TLBEHI}} & csr_tlbehi_rvalue |
                        {32{csr_num == `CSR_TLBELO0}} & csr_tlbelo0_rvalue |
                        {32{csr_num == `CSR_TLBELO1}} & csr_tlbelo1_rvalue |
                        {32{csr_num == `CSR_ASID}}    & csr_asid_rvalue    |
                        {32{csr_num == `CSR_TLBRENTRY}} & csr_tlbrentry_rvalue |
                        32'b0;  // contains ticlr
    assign ertn_pc  = csr_era_pc;
    assign ex_entry = {csr_eentry_va, 6'b0};

    // interrupt generation
    assign isintr = (csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 12'b0 && csr_crmd_ie;

    assign crmd_plv  = csr_crmd_plv;
    assign crmd_da   = csr_crmd_da;
    assign crmd_pg   = csr_crmd_pg;
    assign crmd_datf = csr_crmd_datf;
    assign crmd_datm = csr_crmd_datm;
    assign stat_ecode = csr_estat_ecode;
    assign asid = csr_asid_asid;

    assign dmw0_plv0 = csr_dmw0_plv0;
    assign dmw0_plv3 = csr_dmw0_plv3;
    assign dmw0_mat  = csr_dmw0_mat;
    assign dmw0_pseg = csr_dmw0_pseg;
    assign dmw0_vseg = csr_dmw0_vseg;
    assign dmw1_plv0 = csr_dmw1_plv0;
    assign dmw1_plv3 = csr_dmw1_plv3;
    assign dmw1_mat  = csr_dmw1_mat;
    assign dmw1_pseg = csr_dmw1_pseg;
    assign dmw1_vseg = csr_dmw1_vseg;

    assign ex_tlbrentry = csr_tlbrentry_rvalue;

endmodule
