module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
wire        reset;
assign      reset = ~resetn;

wire        IDU_allow_in;
wire        IFU_to_IDU_valid;
wire        IFU_ready_go;
wire [31:0] inst_to_IDU;
wire [31:0] pc_to_IDU;
wire        br_taken_cancel;
wire        br_taken;
wire [31:0] br_target;

wire        csr_re;
wire        csr_we;
wire [13:0] csr_num;
wire [31:0] csr_rvalue;
wire [31:0] csr_wmask;
wire [31:0] csr_wvalue;
wire [31:0] ertn_pc;
wire [31:0] ex_entry;
wire        wb_ex;
wire [31:0] wb_pc;
wire        ertn_flush;
wire [5:0]  wb_ecode;
wire [8:0]  wb_esubcode;
wire [31:0] wb_vaddr;
wire [31:0] coreid_in;
wire        has_int;
wire [7:0]  hw_int_in;
wire        ipi_int_in;

assign hw_int_in = 8'b0;
assign ipi_int_in = 1'b0;

IFU u_IFU(
    .clk            (clk            ),
    .reset          (reset          ),
    // ie
    .wb_ex          (wb_ex          ),
    .ertn_flush     (ertn_flush     ),
    .has_int        (has_int        ),
    // pc from csr
    .ex_entry       (ex_entry       ),
    .ertn_pc        (wb_pc          ),
    // inst sram interface
    .inst_sram_en   (inst_sram_en   ),
    .inst_sram_we   (inst_sram_we   ),
    .inst_sram_addr (inst_sram_addr ),
    .inst_sram_wdata(inst_sram_wdata),
    .inst_sram_rdata(inst_sram_rdata),
    // to IDU
    .inst_to_IDU    (inst_to_IDU    ),
    .pc_to_IDU      (pc_to_IDU      ),
    .br_taken_cancel (br_taken_cancel ),
    .br_target      (br_target      ),
    .br_taken       (br_taken       ),
    // handshaking signals with IDU
    .IDU_allow_in   (IDU_allow_in   ),
    .IFU_to_IDU_valid(IFU_to_IDU_valid),
    .IFU_ready_go   (IFU_ready_go   )
);

wire        EXU_allow_in;
wire        IDU_to_EXU_valid;
wire        IDU_ready_go;
wire [31:0] IDU_pc_to_EXU;
wire [31:0] IDU_inst_to_EXU;
wire [112:0]IDU_to_EX_ALU_signals;
wire [13:0] IDU_to_EX_pass_signals;
wire [ 4:0] rf_raddr1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata1;
wire [31:0] rf_rdata2;
wire        EXU_to_IDU_gr_we;
wire [ 4:0] EXU_to_IDU_dest;
wire        EXU_to_IDU_valid;
wire [31:0] EXU_to_IDU_forward;
wire        EXU_current_is_ld;
wire        MEM_to_IDU_gr_we;
wire [ 4:0] MEM_to_IDU_dest;
wire        MEM_to_IDU_valid;
wire [31:0] MEM_to_IDU_forward;
wire        WB_to_IDU_gr_we;
wire [ 4:0] WB_to_IDU_dest;
wire        WB_to_IDU_valid;
wire [31:0] WB_to_IDU_forward;
wire [ 4:0] IDU_to_EX_div_signals;
wire [64:0] IDU_to_EXU_csr_signals;
wire        EXU_csr;
wire        MEM_csr;
wire        WB_csr;

IDU u_IDU(
    .clk               (clk                ),
    .reset             (reset              ),
    // ie
    .wb_ex             (wb_ex               ),
    .ertn_flush        (ertn_flush          ),
    .has_int           (has_int             ),
    // from IFU
    .pc_from_IFU       (pc_to_IDU          ),
    .inst_from_IFU     (inst_to_IDU        ),
    // to IFU
    .IDU_br_taken      (br_taken           ),
    .IDU_br_taken_cancel (br_taken_cancel   ),
    .IDU_br_target     (br_target          ),
    // handshaking signals with IFU
    .IFU_to_IDU_valid  (IFU_to_IDU_valid   ),
    .IDU_allow_in      (IDU_allow_in       ),
    // handshaking signals with EXU
    .EXU_allow_in      (EXU_allow_in       ),
    .IDU_ready_go      (IDU_ready_go       ),
    .IDU_to_EXU_valid  (IDU_to_EXU_valid   ),
    // signals and data to EXU
    .IDU_to_EXU_csr_signals (IDU_to_EXU_csr_signals),
    .IDU_pc_to_EXU     (IDU_pc_to_EXU      ),
    .IDU_inst_to_EXU   (IDU_inst_to_EXU    ),
    .IDU_to_EX_ALU_signals(IDU_to_EX_ALU_signals),
    .IDU_to_EX_pass_signals(IDU_to_EX_pass_signals),
    .IDU_to_EX_div_signals (IDU_to_EX_div_signals ),
    // forwarding from EXU/MEMU/WBU
    .EXU_gr_we         (EXU_to_IDU_gr_we   ),
    .EXU_dest          (EXU_to_IDU_dest    ),
    .EXU_valid         (EXU_to_IDU_valid   ),
    .EXU_to_ID_forward (EXU_to_IDU_forward),
    .EXU_current_is_ld (EXU_current_is_ld  ),
    .EXU_csr           (EXU_csr            ),
    .MEM_gr_we         (MEM_to_IDU_gr_we   ),
    .MEM_dest          (MEM_to_IDU_dest    ),
    .MEM_valid         (MEM_to_IDU_valid   ),
    .MEM_to_ID_forward (MEM_to_IDU_forward),
    .MEM_csr           (MEM_to_IDU_csr            ),
    .WB_gr_we          (WB_to_IDU_gr_we    ),
    .WB_dest           (WB_to_IDU_dest     ),
    .WB_valid          (WB_to_IDU_valid    ),
    .WB_to_ID_forward  (WB_to_IDU_forward  ),
    .WB_csr            (WB_csr             ),
    // register file interface
    .rf_raddr1         (rf_raddr1          ),
    .rf_raddr2         (rf_raddr2          ),
    .rf_rdata1         (rf_rdata1          ),
    .rf_rdata2         (rf_rdata2          )
);

wire        MEM_allow_in;
wire        EXU_ready_go;
wire        EXU_to_MEM_valid;
wire [31:0] EXU_pc_to_MEM;
wire [31:0] EXU_inst_to_MEM;
wire [31:0] EXU_result_to_MEM;
wire [12:0] EXU_signals_pass_to_MEM;
wire        MEM_has_int;
wire [96:0] EXU_csr_signals_to_MEM;
EXU u_EXU(
    .clk                    (clk                    ),
    .reset                  (reset                  ),
    // ie
    .has_int                (has_int                ),
    .wb_ex                  (wb_ex                  ),
    .ertn_flush             (ertn_flush             ),
    // handshaking signals with IDU
    .IDU_to_EXU_valid       (IDU_to_EXU_valid       ),
    .EXU_allow_in           (EXU_allow_in           ),
    // handshaking signals with MEM
    .MEM_allow_in           (MEM_allow_in           ),
    .EXU_ready_go           (EXU_ready_go           ),
    .EXU_to_MEM_valid       (EXU_to_MEM_valid       ),
    // data from IDU
    .IDU_to_EXU_csr_signals (IDU_to_EXU_csr_signals ),
    .IDU_pc_to_EXU          (IDU_pc_to_EXU          ),
    .IDU_inst_to_EXU        (IDU_inst_to_EXU        ),
    .IDU_to_EX_ALU_signals  (IDU_to_EX_ALU_signals  ),
    .IDU_to_EX_pass_signals (IDU_to_EX_pass_signals ),
    .IDU_to_EX_div_signals  (IDU_to_EX_div_signals  ),
    // to MEM
    .EXU_csr_signals_to_MEM (EXU_csr_signals_to_MEM ),
    .EXU_pc_to_MEM          (EXU_pc_to_MEM          ),
    .EXU_inst_to_MEM        (EXU_inst_to_MEM        ),
    .EXU_result_to_MEM      (EXU_result_to_MEM      ),
    .EXU_signals_pass_to_MEM(EXU_signals_pass_to_MEM),
    // data from MEM
    .MEM_has_int            (MEM_has_int            ),
    // to IDU
    .EXU_to_IDU_csr         (EXU_csr                ),
    .EXU_to_IDU_gr_we       (EXU_to_IDU_gr_we       ),
    .EXU_to_IDU_dest        (EXU_to_IDU_dest        ),
    .EXU_to_IDU_valid       (EXU_to_IDU_valid       ),
    .EXU_to_IDU_forward     (EXU_to_IDU_forward     ),
    .EXU_current_is_ld      (EXU_current_is_ld      ),
    // data sram interface
    .data_sram_en           (data_sram_en           ),
    .data_sram_we           (data_sram_we           ),
    .data_sram_addr         (data_sram_addr         ), 
    .data_sram_wdata        (data_sram_wdata        )
);

wire        WB_allow_in;
wire        MEM_ready_go;
wire        MEM_to_WB_valid;
wire [31:0] MEM_pc_to_WB;
wire [31:0] MEM_inst_to_WB;
wire [31:0] MEM_result_to_WB;
wire [ 5:0] MEM_signals_pass_to_WB;
wire [96:0] MEM_csr_signals_to_WB;
MEMU u_MEMU(
    .clk                (clk                ),
    .reset              (reset              ),
    .has_int            (has_int            ),
    .wb_ex              (wb_ex              ),
    .ertn_flush         (ertn_flush         ),
    // handshaking signals with EXU
    .EXU_to_MEM_valid   (EXU_to_MEM_valid   ), 
    .MEM_allow_in       (MEM_allow_in       ),
    // handshaking signals with WB
    .WB_allow_in        (WB_allow_in        ),
    .MEM_ready_go       (MEM_ready_go       ),
    .MEM_to_WB_valid    (MEM_to_WB_valid    ),
    // data from EXU
    .EXU_csr_signals_to_MEM(EXU_csr_signals_to_MEM),
    .EXU_pc_to_MEM      (EXU_pc_to_MEM      ),
    .EXU_inst_to_MEM    (EXU_inst_to_MEM    ), 
    .EXU_result_to_MEM  (EXU_result_to_MEM),
    .EXU_signals_pass_to_MEM(EXU_signals_pass_to_MEM),
    // data from data sram
    .data_sram_rdata    (data_sram_rdata    ),
    // to IDU
    .MEM_to_IDU_csr     (MEM_to_IDU_csr     ),
    .MEM_to_IDU_gr_we   (MEM_to_IDU_gr_we   ),
    .MEM_to_IDU_dest    (MEM_to_IDU_dest    ),
    .MEM_to_IDU_valid   (MEM_to_IDU_valid   ),
    .MEM_to_IDU_forward (MEM_to_IDU_forward ),
    // to EXU
    .MEM_has_int        (MEM_has_int        ),
    // data to WB
    .MEM_csr_signals_to_WB(MEM_csr_signals_to_WB),
    .MEM_pc_to_WB       (MEM_pc_to_WB       ),
    .MEM_inst_to_WB     (MEM_inst_to_WB     ),
    .MEM_result_to_WB   (MEM_result_to_WB   ),
    .MEM_signals_pass_to_WB(MEM_signals_pass_to_WB)
);

wire        WB_ready_go;
wire        WB_to_out_valid;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;
wire        rf_we;
WBU u_WBU(
    .clk                (clk                ),
    .reset              (reset              ),
    // handshaking signals with MEM
    .MEM_to_WB_valid   (MEM_to_WB_valid    ),
    .WB_allow_in       (WB_allow_in        ),
    // handshaking signals with outside (not implemented)
    .WB_ready_go       (WB_ready_go        ),
    .WB_to_out_valid   (WB_to_out_valid    ),
    // data from MEM
    .MEM_csr_signals_to_WB(MEM_csr_signals_to_WB),
    .MEM_pc_to_WB      (MEM_pc_to_WB       ),
    .MEM_inst_to_WB    (MEM_inst_to_WB     ),
    .MEM_result_to_WB  (MEM_result_to_WB   ),
    .MEM_signals_pass_to_WB(MEM_signals_pass_to_WB),
    // to IDU
    .WB_to_IDU_csr     (WB_csr             ),
    .WB_to_IDU_gr_we   (WB_to_IDU_gr_we    ),
    .WB_to_IDU_dest    (WB_to_IDU_dest     ),
    .WB_to_IDU_valid   (WB_to_IDU_valid    ),
    .WB_to_IDU_forward (WB_to_IDU_forward  ),
    // register file interface
    .rf_waddr          (rf_waddr           ),
    .rf_wdata          (rf_wdata           ),
    .rf_we             (rf_we              ),
    // debug interface
    .debug_pc          (debug_wb_pc        ),
    .debug_rf_we       (debug_wb_rf_we     ),
    .debug_rf_wnum     (debug_wb_rf_wnum   ), 
    .debug_rf_wdata    (debug_wb_rf_wdata  ),
    // csr interface
    .csr_re            (csr_re            ),
    .csr_we            (csr_we            ),
    .csr_num           (csr_num           ),
    .csr_rvalue        (csr_rvalue        ),
    .csr_wmask         (csr_wmask         ),
    .csr_wvalue        (csr_wvalue        ),
    .wb_pc             (wb_pc             ),
    .ertn_flush        (ertn_flush        ),
    .wb_ex             (wb_ex             ),
    .wb_ecode          (wb_ecode          ),
    .wb_esubcode       (wb_esubcode       )
);

regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

csr_regs u_csr_regs(
    .clk            (clk            ),
    .reset          (reset          ),
    .csr_re         (csr_re         ),
    .csr_we         (csr_we         ),
    .csr_wmask      (csr_wmask      ),
    .csr_rvalue     (csr_rvalue     ),
    .csr_wvalue     (csr_wvalue     ),
    .ertn_pc       (ertn_pc       ),
    .ex_entry       (ex_entry       ),
    .wb_ex          (wb_ex          ),
    .wb_pc          (wb_pc          ),
    .ertn_flush     (ertn_flush     ),
    .wb_ecode       (wb_ecode       ),
    .wb_esubcode    (wb_esubcode    ),
    .wb_vaddr       (wb_vaddr       ),
    .coreid_in      (coreid_in      ),
    .has_int        (has_int        ),
    .hw_int_in      (hw_int_in      ),
    .ipi_int_in     (ipi_int_in     )
);

endmodule
