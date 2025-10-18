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

IFU u_IFU(
    .clk            (clk            ),
    .reset          (reset          ),
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
wire [112:0] IDU_to_EX_ALU_signals;
wire  [7:0] IDU_to_EX_pass_signals;
wire [ 4:0] rf_raddr1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata1;
wire [31:0] rf_rdata2;
wire        EXU_to_IDU_gr_we;
wire  [4:0] EXU_to_IDU_dest;
wire        EXU_to_IDU_valid;
wire [31:0] EXU_to_IDU_forward;
wire        EXU_current_is_ld;
wire        MEM_to_IDU_gr_we;
wire  [4:0] MEM_to_IDU_dest;
wire        MEM_to_IDU_valid;
wire [31:0] MEM_to_IDU_forward;
wire        WB_to_IDU_gr_we;
wire  [4:0] WB_to_IDU_dest;
wire        WB_to_IDU_valid;
wire [31:0] WB_to_IDU_forward;
wire  [4:0] IFU_to_EX_div_signals;

IDU u_IDU(
    .clk                (clk                ),
    .reset              (reset              ),
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
    .IDU_pc_to_EXU     (IDU_pc_to_EXU      ),
    .IDU_inst_to_EXU   (IDU_inst_to_EXU    ),
    .IFU_to_EX_ALU_signals(IDU_to_EX_ALU_signals),
    .IFU_to_EX_pass_signals(IDU_to_EX_pass_signals),
    .IFU_to_EX_div_signals (IFU_to_EX_div_signals ),
    // forwarding from EXU/MEMU/WBU
    .EXU_gr_we         (EXU_to_IDU_gr_we   ),
    .EXU_dest          (EXU_to_IDU_dest    ),
    .EXU_valid         (EXU_to_IDU_valid   ),
    .EXU_to_ID_forward (EXU_to_IDU_forward),
    .EXU_current_is_ld (EXU_current_is_ld  ),
    .MEM_gr_we         (MEM_to_IDU_gr_we   ),
    .MEM_dest          (MEM_to_IDU_dest    ),
    .MEM_valid         (MEM_to_IDU_valid   ),
    .MEM_to_ID_forward (MEM_to_IDU_forward),
    .WB_gr_we          (WB_to_IDU_gr_we    ),
    .WB_dest           (WB_to_IDU_dest     ),
    .WB_valid          (WB_to_IDU_valid    ),
    .WB_to_ID_forward  (WB_to_IDU_forward  ),
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
wire  [6:0] EXU_signals_pass_to_MEM;
EXU u_EXU(
    .clk                    (clk                    ),
    .reset                  (reset                  ),
    // handshaking signals with IDU
    .IDU_to_EXU_valid       (IDU_to_EXU_valid       ),
    .EXU_allow_in           (EXU_allow_in           ),
    // handshaking signals with MEM
    .MEM_allow_in           (MEM_allow_in           ),
    .EXU_ready_go           (EXU_ready_go           ),
    .EXU_to_MEM_valid       (EXU_to_MEM_valid       ),
    // data from IDU
    .IDU_pc_to_EXU          (IDU_pc_to_EXU          ),
    .IDU_inst_to_EXU        (IDU_inst_to_EXU        ),
    .IDU_to_EX_ALU_signals  (IDU_to_EX_ALU_signals  ),
    .IDU_to_EX_pass_signals (IDU_to_EX_pass_signals ),
    .IDU_to_EX_div_signals  (IFU_to_EX_div_signals  ),
    // to MEM
    .EXU_pc_to_MEM          (EXU_pc_to_MEM          ),
    .EXU_inst_to_MEM        (EXU_inst_to_MEM        ),
    .EXU_result_to_MEM      (EXU_result_to_MEM      ),
    .EXU_signals_pass_to_MEM(EXU_signals_pass_to_MEM),
    // to IDU
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
wire  [5:0] MEM_signals_pass_to_WB;
MEMU u_MEMU(
    .clk                (clk                ),
    .reset              (reset              ),
    // handshaking signals with EXU
    .EXU_to_MEM_valid   (EXU_to_MEM_valid   ), 
    .MEM_allow_in       (MEM_allow_in       ),
    // handshaking signals with WB
    .WB_allow_in        (WB_allow_in        ),
    .MEM_ready_go       (MEM_ready_go       ),
    .MEM_to_WB_valid    (MEM_to_WB_valid    ),
    // data from EXU
    .EXU_pc_to_MEM      (EXU_pc_to_MEM      ),
    .EXU_inst_to_MEM    (EXU_inst_to_MEM    ), 
    .EXU_result_to_MEM  (EXU_result_to_MEM),
    .EXU_signals_pass_to_MEM(EXU_signals_pass_to_MEM),
    // data from data sram
    .data_sram_rdata    (data_sram_rdata    ),
    // to IDU
    .MEM_to_IDU_gr_we   (MEM_to_IDU_gr_we   ),
    .MEM_to_IDU_dest    (MEM_to_IDU_dest    ),
    .MEM_to_IDU_valid   (MEM_to_IDU_valid   ),
    .MEM_to_IDU_forward (MEM_to_IDU_forward ),
    // data to WB
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
    .MEM_pc_to_WB      (MEM_pc_to_WB       ),
    .MEM_inst_to_WB    (MEM_inst_to_WB     ),
    .MEM_result_to_WB  (MEM_result_to_WB   ),
    .MEM_signals_pass_to_WB(MEM_signals_pass_to_WB),
    // to IDU
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
    .debug_rf_wdata    (debug_wb_rf_wdata  )
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


endmodule
