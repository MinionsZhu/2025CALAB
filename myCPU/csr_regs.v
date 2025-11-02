`include "csr.vh"
module csr_regs (
    input           clk,
    input           reset,

    input   [`CSR_NUM_WIDTH-1:0] csr_num,

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

    output          has_int,
    input   [ 7:0]  hw_int_in,
    input           ipi_int_in
);
    // timer
    reg [31:0] timer_cnt;
    // CRMD CSR
    reg [1:0]   csr_crmd_plv;
    reg         csr_crmd_ie;
    reg         csr_crmd_da;
    reg         csr_crmd_pg;
    reg [1:0]   csr_crmd_datf;
    reg [1:0]   csr_crmd_datm;
    // PRMD CSR
    reg [1:0]   csr_prmd_pplv;
    reg         csr_prmd_pie;
    // ECFG CSR
    reg [12:0]  csr_ecfg_lie;
    // ESTAT CSR
    reg [12:0]  csr_estat_is;
    reg [ 5:0]  csr_estat_ecode;
    reg [ 8:0]  csr_estat_esubcode;
    // ERA CSR
    reg [31:0]  csr_era_pc;
    // EENTRY CSR
    reg [25:0]  csr_eentry_va;
    // SAVE CSR
    reg [31:0]  csr_save0;
    reg [31:0]  csr_save1;
    reg [31:0]  csr_save2;
    reg [31:0]  csr_save3;

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
    // unused signals
    always @(posedge clk) begin
        if(reset) begin
            csr_crmd_da <= 1'b1;
            csr_crmd_pg <= 1'b0;
            csr_crmd_datf <= 2'b00;
            csr_crmd_datm <= 2'b00;
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
        else if (csr_we && (csr_num == `CSR_TICLR) && csr_wmask[`CSR_TICLR] && csr_wvalue[`CSR_TICLR]) begin
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
            csr_era_pc <= wb_pc;
        end
        else if(csr_we && (csr_num == `CSR_ERA)) begin
            csr_era_pc <=  csr_wmask[`CSR_ERA_PC]&csr_wvalue[`CSR_ERA_PC]
                        | ~csr_wmask[`CSR_ERA_PC]&csr_era_pc;
        end
    end
    // EENTRY
    always @(posedge clk) begin
        if(csr_we && (csr_num == `CSR_EENTRY)) begin
            csr_eentry_va <=  csr_wmask[`CSR_EENTRY_VA]&csr_wvalue[`CSR_EENTRY_VA]
                           | ~csr_wmask[`CSR_EENTRY_VA]&csr_eentry_va;
        end
    end
    // TODO: BADVADDR
    // SAVE0-3
    always @(posedge clk) begin
        if(csr_we && (csr_num == `CSR_SAVE0)) begin
            csr_save0 <=  csr_wmask[`CSR_SAVE0]&csr_wvalue[`CSR_SAVE0]
                       | ~csr_wmask[`CSR_SAVE0]&csr_save0;
        end
        if(csr_we && (csr_num == `CSR_SAVE1)) begin
            csr_save1 <=  csr_wmask[`CSR_SAVE1]&csr_wvalue[`CSR_SAVE1]
                       | ~csr_wmask[`CSR_SAVE1]&csr_save1;
        end
        if(csr_we && (csr_num == `CSR_SAVE2)) begin
            csr_save2 <=  csr_wmask[`CSR_SAVE2]&csr_wvalue[`CSR_SAVE2]
                       | ~csr_wmask[`CSR_SAVE2]&csr_save2;
        end
        if(csr_we && (csr_num == `CSR_SAVE3)) begin
            csr_save3 <=  csr_wmask[`CSR_SAVE3]&csr_wvalue[`CSR_SAVE3]
                       | ~csr_wmask[`CSR_SAVE3]&csr_save3;
        end
    end
    // TODO: TID
    reg             csr_tcfg_en;
    reg             csr_tcfg_periodic;
    reg     [29:0]  csr_tcfg_initval;
    wire    [31:0]  tcfg_next_value;
    wire    [31:0]  csr_tval;
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

    assign tcfg_next_value = csr_wmask[31:0]&csr_wvalue[31:0]
                           | ~csr_wmask[31:0]&{csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
    // timer
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
    // read CSR value
    wire    [31:0]  csr_crmd_rvalue;
    wire    [31:0]  csr_prmd_rvalue;
    wire    [31:0]  csr_ecfg_rvalue;
    wire    [31:0]  csr_estat_rvalue;
    wire    [31:0]  csr_era_rvalue;
    wire    [31:0]  csr_eentry_rvalue;
    wire    [31:0]  csr_save0_rvalue;
    wire    [31:0]  csr_save1_rvalue;
    wire    [31:0]  csr_save2_rvalue;
    wire    [31:0]  csr_save3_rvalue;
    wire    [31:0]  csr_tcfg_rvalue;
    wire    [31:0]  csr_tval_rvalue;
    assign csr_crmd_rvalue   = {28'b0, csr_crmd_da, csr_crmd_ie, csr_crmd_plv};
    assign csr_prmd_rvalue   = {29'b0, csr_prmd_pie, csr_prmd_pplv};
    assign csr_ecfg_rvalue   = {19'b0, csr_ecfg_lie};
    assign csr_estat_rvalue  = { 1'b0, csr_estat_esubcode, csr_estat_ecode, 3'b0, csr_estat_is};
    assign csr_era_rvalue    = csr_era_pc;
    assign csr_eentry_rvalue = {csr_eentry_va, 6'b0};
    assign csr_save0_rvalue  = csr_save0;
    assign csr_save1_rvalue  = csr_save1;
    assign csr_save2_rvalue  = csr_save2;
    assign csr_save3_rvalue  = csr_save3;
    assign csr_tcfg_rvalue   = {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
    assign csr_tval_rvalue   = csr_tval;
    assign csr_rvalue = {32{csr_num == `CSR_CRMD}}   & csr_crmd_rvalue   |
                        {32{csr_num == `CSR_PRMD}}   & csr_prmd_rvalue   |
                        {32{csr_num == `CSR_ECFG}}   & csr_ecfg_rvalue   |
                        {32{csr_num == `CSR_ESTAT}}  & csr_estat_rvalue  |
                        {32{csr_num == `CSR_ERA}}    & csr_era_rvalue    |
                        {32{csr_num == `CSR_EENTRY}} & csr_eentry_rvalue |
                        {32{csr_num == `CSR_SAVE0}}  & csr_save0_rvalue  |
                        {32{csr_num == `CSR_SAVE1}}  & csr_save1_rvalue  |
                        {32{csr_num == `CSR_SAVE2}}  & csr_save2_rvalue  |
                        {32{csr_num == `CSR_SAVE3}}  & csr_save3_rvalue  |
                        {32{csr_num == `CSR_TCFG}}   & csr_tcfg_rvalue   |
                        {32{csr_num == `CSR_TVAL}}   & csr_tval_rvalue   |
                        32'b0;
    assign ertn_pc  = csr_era_pc;
    assign ex_entry = {csr_eentry_va, 6'b0};
    // interrupt pending
    assign has_int = (csr_estat_is[11:0] & csr_ecfg_lie[11:0]) != 12'b0 && csr_crmd_ie;

endmodule
