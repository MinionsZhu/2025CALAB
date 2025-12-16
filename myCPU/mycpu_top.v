`include "busdef.vh"
module mycpu_top(
    input  wire        aclk,
    input  wire        aresetn,
    // axi interface
    // AR Channel
    output wire [ 3:0] arid,
    output wire [31:0] araddr,
    output wire [ 7:0] arlen,
    output wire [ 2:0] arsize,
    output wire [ 1:0] arburst,
    output wire [ 1:0] arlock,
    output wire [ 3:0] arcache,
    output wire [ 2:0] arprot,
    output wire        arvalid,
    input  wire        arready,
    // R Channel
    input  wire [ 3:0] rid,
    input  wire [31:0] rdata,
    input  wire [ 1:0] rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,
    // AW Channel
    output wire [ 3:0] awid,
    output wire [31:0] awaddr,
    output wire [ 7:0] awlen,
    output wire [ 2:0] awsize,
    output wire [ 1:0] awburst,
    output wire [ 1:0] awlock,
    output wire [ 3:0] awcache,
    output wire [ 2:0] awprot,
    output wire        awvalid,
    input  wire        awready,
    // W Channel
    output wire [ 3:0] wid,
    output wire [31:0] wdata,
    output wire [ 3:0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,
    // B Channel
    input  wire [ 3:0] bid,
    input  wire [ 1:0] bresp,
    input  wire        bvalid,
    output wire        bready,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
// inst sram interface
wire        inst_sram_req;
wire        inst_sram_wr;
wire [ 1:0] inst_sram_size;
wire [ 3:0] inst_sram_wstrb;
wire [31:0] inst_sram_addr;
wire [31:0] inst_sram_wdata;
wire        inst_sram_addr_ok;
wire        inst_sram_data_ok;
wire [31:0] inst_sram_rdata;
// data sram interface
wire        data_sram_req;
wire        data_sram_wr;
wire [ 1:0] data_sram_size;
wire [ 3:0] data_sram_wstrb;
wire [31:0] data_sram_addr;
wire [31:0] data_sram_wdata;
wire        data_sram_addr_ok;
wire        data_sram_data_ok;
wire [31:0] data_sram_rdata;
// connect cpu_axi_bridge
cpu_axi_bridge u_cpu_axi_bridge(
    .clk                (aclk                ),
    .resetn             (aresetn             ),
    // inst sram interface
    .inst_req           (inst_sram_req      ),
    .inst_wr            (inst_sram_wr       ),
    .inst_size          (inst_sram_size     ),
    .inst_wstrb         (inst_sram_wstrb    ),
    .inst_addr          (inst_sram_addr     ),
    .inst_wdata         (inst_sram_wdata    ),
    .inst_addr_ok       (inst_sram_addr_ok  ),
    .inst_data_ok       (inst_sram_data_ok  ),
    .inst_rdata         (inst_sram_rdata    ),
    // data sram interface
    .data_req           (data_sram_req      ),
    .data_wr            (data_sram_wr       ),
    .data_size          (data_sram_size     ),
    .data_wstrb         (data_sram_wstrb    ),
    .data_addr          (data_sram_addr     ),
    .data_wdata         (data_sram_wdata    ),
    .data_addr_ok       (data_sram_addr_ok  ),
    .data_data_ok       (data_sram_data_ok  ),
    .data_rdata         (data_sram_rdata    ),
    // AXI interface
    .arid               (arid               ),
    .araddr             (araddr             ),
    .arlen              (arlen              ),
    .arsize             (arsize             ),
    .arburst            (arburst            ),
    .arlock             (arlock             ),
    .arcache            (arcache            ),
    .arprot             (arprot             ),
    .arvalid            (arvalid            ),
    .arready            (arready            ),
    .rid                (rid                ),
    .rdata              (rdata              ),
    .rresp              (rresp              ),
    .rlast              (rlast              ),
    .rvalid             (rvalid             ),
    .rready             (rready             ),
    .awid               (awid               ),
    .awaddr             (awaddr             ),
    .awlen              (awlen              ),
    .awsize             (awsize             ),
    .awburst            (awburst            ),
    .awlock             (awlock             ),
    .awcache            (awcache            ),
    .awprot             (awprot             ),
    .awvalid            (awvalid            ),
    .awready            (awready            ),
    .wid                (wid                ),
    .wdata              (wdata              ),
    .wstrb              (wstrb              ),
    .wlast              (wlast              ),
    .wvalid             (wvalid             ),
    .wready             (wready             ),
    .bid                (bid                ),
    .bresp              (bresp              ),
    .bvalid             (bvalid             ),
    .bready             (bready             )
);

wire        reset;
assign      reset = ~aresetn;

wire        idAllowin;
wire        ifValidout;
wire [31:0] if2idInst;
wire [31:0] if2idPC;
wire        isadef;
wire        if_tlbr_ex;
wire        if_pif_ex;
wire        if_ppi_ex;
wire        br_taken_cancel;
wire        br_taken;
wire [31:0] br_target;
wire        br_stall;

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
wire [ 5:0] wb_ecode;
wire [ 8:0] wb_esubcode;
wire [31:0] wb_vaddr;
wire [31:0] coreid_in;
wire        isintr;
wire [ 7:0] hw_int_in;
wire        ipi_int_in;

assign hw_int_in  = 8'b0;
assign ipi_int_in = 1'b0;
assign coreid_in  = 32'b0;

// TLB search port 0 (for fetch)
wire [18:0] s0_vppn;
wire        s0_va_bit12;
wire [ 9:0] s0_asid;
wire        s0_found;
wire [ 3:0] s0_index;
wire [19:0] s0_ppn;
wire [ 5:0] s0_ps;
wire [ 1:0] s0_plv;
wire [ 1:0] s0_mat;
wire        s0_d;
wire        s0_v;

// TLB search port 1 (for load/store)
wire [18:0] s1_vppn;
wire        s1_va_bit12;
wire [ 9:0] s1_asid;
wire        s1_found;
wire [ 3:0] s1_index;
wire [19:0] s1_ppn;
wire [ 5:0] s1_ps;
wire [ 1:0] s1_plv;
wire [ 1:0] s1_mat;
wire        s1_d;
wire        s1_v;

// TLB write port
wire        tlb_we;
wire [ 3:0] tlb_w_index;
wire        tlb_w_e;
wire [18:0] tlb_w_vppn;
wire [ 5:0] tlb_w_ps;
wire [ 9:0] tlb_w_asid;
wire        tlb_w_g;
wire [19:0] tlb_w_ppn0;
wire [ 1:0] tlb_w_plv0;
wire [ 1:0] tlb_w_mat0;
wire        tlb_w_d0;
wire        tlb_w_v0;
wire [19:0] tlb_w_ppn1;
wire [ 1:0] tlb_w_plv1;
wire [ 1:0] tlb_w_mat1;
wire        tlb_w_d1;
wire        tlb_w_v1;

// TLB read port
wire [ 3:0] tlb_r_index;
wire        tlb_r_e;
wire [18:0] tlb_r_vppn;
wire [ 5:0] tlb_r_ps;
wire [ 9:0] tlb_r_asid;
wire        tlb_r_g;
wire [19:0] tlb_r_ppn0;
wire [ 1:0] tlb_r_plv0;
wire [ 1:0] tlb_r_mat0;
wire        tlb_r_d0;
wire        tlb_r_v0;
wire [19:0] tlb_r_ppn1;
wire [ 1:0] tlb_r_plv1;
wire [ 1:0] tlb_r_mat1;
wire        tlb_r_d1;
wire        tlb_r_v1;

// TLB invtlb
wire        invtlb_valid;
wire [ 4:0] invtlb_op;

// TLB control signals from WBU
wire        tlbsrch;
wire        tlbrd;
wire        tlbfill;
wire [ 3:0] tlbfill_randidx;
wire        tlbsrch_found;
wire [ 3:0] tlbsrch_index;

// refetch and changeTLB signals
wire        if2idRefetch;
wire        id2exRefetch;
wire        id2exChangeTLB;
wire        id2exChangeTLBEHI;
wire        ex2memRefetch;
wire        ex2memChangeTLB;
wire        ex2memChangeTLBEHI;
wire        mem2wbRefetch;
wire        mem2wbChangeTLB;
wire        mem2wbChangeTLBEHI;
wire        wbRefetch;
wire        wbChangeTLB;
wire        wbChangeTLBEHI;
wire        changeTLB_stall;
wire        tlb_stall;

wire        tlbr_ex;
wire        if_tlb_refill;
wire        if_plv_ex;
wire [ 1:0] crmd_plv;
wire        crmd_da;
wire        crmd_pg;
wire [ 1:0] crmd_datf;
wire [ 1:0] crmd_datm;
wire        dmw0_plv0;
wire        dmw0_plv3;
wire [ 1:0] dmw0_mat;
wire [ 2:0] dmw0_pseg;
wire [ 2:0] dmw0_vseg;
wire        dmw1_plv0;
wire        dmw1_plv3;
wire [ 1:0] dmw1_mat;
wire [ 2:0] dmw1_pseg;
wire [ 2:0] dmw1_vseg;
wire [ 9:0] csr_asid_asid;
wire [ 5:0] stat_ecode;
wire [31:0] ex_tlbrentry;

assign changeTLB_stall = id2exChangeTLB | ex2memChangeTLB | mem2wbChangeTLB;
assign tlb_stall = mem2wbChangeTLBEHI | wbChangeTLBEHI;

IFU u_IFU(
    .clk                (aclk            ),
    .reset              (reset          ),
    // ie
    .wb_ex              (wb_ex          ),
    .ertn_flush         (ertn_flush     ),
    // pc from csr
    .ex_entry           (ex_entry       ),
    .ertn_pc            (ertn_pc        ),
    // inst sram interface
    .inst_sram_req      (inst_sram_req  ),
    .inst_sram_wr       (inst_sram_wr   ),
    .inst_sram_size     (inst_sram_size ),
    .inst_sram_wstrb    (inst_sram_wstrb),
    .inst_sram_addr     (inst_sram_addr ),
    .inst_sram_wdata    (inst_sram_wdata),
    .inst_sram_addr_ok  (inst_sram_addr_ok),
    .inst_sram_data_ok  (inst_sram_data_ok),
    .inst_sram_rdata    (inst_sram_rdata),
    // from IDU
    .br_taken           (br_taken       ),
    .br_taken_cancel    (br_taken_cancel),
    .br_target          (br_target      ),
    .br_stall           (br_stall       ),
    // to IDU
    .if2idInst          (if2idInst      ),
    .if2idPC            (if2idPC        ),
    .isadef             (isadef         ),
    .if_tlbr_ex         (if_tlbr_ex     ),
    .if_pif_ex          (if_pif_ex      ),
    .if_ppi_ex          (if_ppi_ex      ),
    // handshaking signals with IDU
    .idAllowin          (idAllowin      ),
    .ifValidout         (ifValidout     ),
    // refetch signal
    .changeTLB_stall    (changeTLB_stall),
    .wb_refetch         (wbRefetch      ),
    .wb_pc              (wb_pc          ),
    .if2idRefetch       (if2idRefetch   ),
    .csr_asid_asid      (csr_asid_asid  ),
    .s0_vppn        (s0_vppn        ),
    .s0_va_bit12    (s0_va_bit12    ),
    .s0_asid        (s0_asid        ),
    .s0_found       (s0_found       ),
    .s0_ppn         (s0_ppn         ),
    .s0_plv         (s0_plv         ),
    .s0_v           (s0_v           ),
    .csr_crmd_plv   (crmd_plv       ),
    .csr_crmd_da    (crmd_da        ),
    .csr_crmd_pg    (crmd_pg        ),
    .csr_crmd_datf  (crmd_datf      ),
    .csr_crmd_datm  (crmd_datm      ),
    .csr_dmw0_plv0  (dmw0_plv0      ),
    .csr_dmw0_plv3  (dmw0_plv3      ),
    .csr_dmw0_mat   (dmw0_mat       ),
    .csr_dmw0_pseg  (dmw0_pseg      ),
    .csr_dmw0_vseg  (dmw0_vseg      ),
    .csr_dmw1_plv0  (dmw1_plv0      ),
    .csr_dmw1_plv3  (dmw1_plv3      ),
    .csr_dmw1_mat   (dmw1_mat       ),
    .csr_dmw1_pseg  (dmw1_pseg      ),
    .csr_dmw1_vseg  (dmw1_vseg      ),
    .tlbr_ex        (tlbr_ex        ),
    .csr_tlbrentry  (ex_tlbrentry   )
);

wire        exAllowin;
wire        idValidout;
wire [31:0] id2exPC;
wire [31:0] id2exInst;
wire [`ALUBUSL]id2exALUBus;
wire [`IDPASSBUSL] id2exPassBus;

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

wire [ 4:0] id2exDivBus;
wire [`CSRBUSL] id2exCsrBus;
wire [ 9:0] id2exTLBBus;

wire        MEM_to_IDU_forward_valid;

wire        EXU_csr;
wire        MEM_csr;
wire        WB_csr;

IDU u_IDU(
    .clk                (aclk               ),
    .reset              (reset              ),
    // ie
    .wb_ex              (wb_ex | wbRefetch  ),
    .ertn_flush         (ertn_flush         ),
    .isintr             (isintr             ),
    // from IFU
    .if2idPC            (if2idPC            ),
    .if2idInst          (if2idInst          ),
    .isadef             (isadef             ),
    .if_tlbr_ex         (if_tlbr_ex         ),
    .if_pif_ex          (if_pif_ex          ),
    .if_ppi_ex          (if_ppi_ex          ),
    // to IFU
    .br_taken           (br_taken           ),
    .br_taken_cancel    (br_taken_cancel    ),
    .br_target          (br_target          ),
    .br_stall           (br_stall           ),
    // handshaking signals with IFU
    .ifValidout         (ifValidout         ),
    .idAllowin          (idAllowin          ),
    // handshaking signals with EXU
    .exAllowin          (exAllowin          ),
    .idValidout         (idValidout         ),
    // signals and data to EXU
    .id2exCsrBus        (id2exCsrBus        ),
    .id2exPC            (id2exPC            ),
    .id2exInst          (id2exInst          ),
    .id2exALUBus        (id2exALUBus        ),
    .id2exPassBus       (id2exPassBus       ),
    .id2exDivBus        (id2exDivBus        ),
    // forwarding from EXU/MEMU/WBU
    .EXU_gr_we          (EXU_to_IDU_gr_we   ),
    .EXU_dest           (EXU_to_IDU_dest    ),
    .EXU_valid          (EXU_to_IDU_valid   ),
    .EXU_to_ID_forward  (EXU_to_IDU_forward ),
    .EXU_current_is_ld  (EXU_current_is_ld  ),
    .EXU_csr            (EXU_csr            ),
    .MEM_gr_we          (MEM_to_IDU_gr_we   ),
    .MEM_dest           (MEM_to_IDU_dest    ),
    .MEM_valid          (MEM_to_IDU_valid   ),
    .MEM_to_ID_forward  (MEM_to_IDU_forward ),
    .MEM_to_ID_forward_valid(MEM_to_IDU_forward_valid),
    .MEM_csr            (MEM_to_IDU_csr     ),
    .WB_gr_we           (WB_to_IDU_gr_we    ),
    .WB_dest            (WB_to_IDU_dest     ),
    .WB_valid           (WB_to_IDU_valid    ),
    .WB_to_ID_forward   (WB_to_IDU_forward  ),
    .WB_csr             (WB_csr             ),
    // register file interface
    .rf_raddr1          (rf_raddr1          ),
    .rf_raddr2          (rf_raddr2          ),
    .rf_rdata1          (rf_rdata1          ),
    .rf_rdata2          (rf_rdata2          ),
    // TLBBus
    .id2exTLBBus        (id2exTLBBus        ),
    // refetch signal
    .if2idRefetch       (if2idRefetch       ),
    .id2exRefetch       (id2exRefetch       ),
    .id2exChangeTLB     (id2exChangeTLB     ),
    .id2exChangeTLBEHI  (id2exChangeTLBEHI  )
);

wire        memAllowin;
wire        exValidout;
wire        is_sram_inst;
wire [31:0] ex2memPC;
wire [31:0] ex2memInst;
wire [31:0] ex2memResult;
wire        memStopMemAccess;
wire [`CSRBUSL] ex2memCsrBus;
wire [`EXPASSBUSL] ex2memPassBus;
wire [ 9:0] ex2memTLBBus;

EXU u_EXU(
    .clk                    (aclk                ),
    .reset                  (reset              ),
    // ie
    .wb_ex                  (wb_ex | wbRefetch  ),
    .ertn_flush             (ertn_flush         ),
    // handshaking signals with IDU
    .idValidout             (idValidout         ),
    .exAllowin              (exAllowin          ),
    // handshaking signals with MEM
    .memAllowin             (memAllowin         ),
    .exValidout             (exValidout         ),
    // data from IDU
    .id2exCsrBus            (id2exCsrBus        ),
    .id2exPC                (id2exPC            ),
    .id2exInst              (id2exInst          ),
    .id2exALUBus            (id2exALUBus        ),
    .id2exPassBus           (id2exPassBus       ),
    .id2exDivBus            (id2exDivBus        ),
    .id2exTLBBus            (id2exTLBBus        ),
    // from MEM
    .memStopMemAccess       (memStopMemAccess   ),
    // to MEM
    .ex2memCsrBus           (ex2memCsrBus       ),
    .ex2memPC               (ex2memPC           ),
    .ex2memInst             (ex2memInst         ),
    .ex2memResult           (ex2memResult       ),
    .ex2memPassBus          (ex2memPassBus      ),
    .ex2memTLBBus           (ex2memTLBBus       ),
    // to IDU
    .EXU_to_IDU_csr         (EXU_csr            ),
    .EXU_to_IDU_gr_we       (EXU_to_IDU_gr_we   ),
    .EXU_to_IDU_dest        (EXU_to_IDU_dest    ),
    .EXU_to_IDU_valid       (EXU_to_IDU_valid   ),
    .EXU_to_IDU_forward     (EXU_to_IDU_forward ),
    .EXU_current_is_ld      (EXU_current_is_ld  ),
    // data sram interface
    .data_sram_req          (data_sram_req      ),
    .data_sram_wr           (data_sram_wr       ),
    .data_sram_size         (data_sram_size     ),
    .data_sram_wstrb        (data_sram_wstrb    ),
    .data_sram_addr         (data_sram_addr     ), 
    .data_sram_wdata        (data_sram_wdata    ),
    .data_sram_addr_ok      (data_sram_addr_ok  ),
    // TLB search interface (for Load/Store/tlbsrch instructions)
    .s1_vppn                (s1_vppn            ),
    .s1_va_bit12            (s1_va_bit12        ),
    .s1_asid                (s1_asid            ),
    .s1_found               (s1_found           ),
    .s1_index               (s1_index           ),
    .s1_ppn                 (s1_ppn             ),
    .s1_ps                  (s1_ps              ),
    .s1_plv                 (s1_plv             ),
    .s1_mat                 (s1_mat             ),
    .s1_d                   (s1_d               ),
    .s1_v                   (s1_v               ),
    // for invtlb instruction(to TLB)
    .invtlb_valid           (invtlb_valid       ),
    .invtlb_op              (invtlb_op          ),
    // for tlbsrch use(from csr)
    .csr_tlbehi_vppn        (tlb_w_vppn         ),
    .csr_asid               (tlb_w_asid         ),
    // refetch signal
    .tlb_stall              (tlb_stall          ),
    .id2exRefetch           (id2exRefetch       ),
    .id2exChangeTLB         (id2exChangeTLB     ),
    .id2exChangeTLBEHI      (id2exChangeTLBEHI  ),
    .ex2memRefetch          (ex2memRefetch      ),
    .ex2memChangeTLB        (ex2memChangeTLB    ),
    .ex2memChangeTLBEHI     (ex2memChangeTLBEHI ),
    .csr_crmd_plv       (crmd_plv       ),
    .csr_crmd_da        (crmd_da        ),
    .csr_crmd_pg        (crmd_pg        ),
    .csr_crmd_datf      (crmd_datf      ),
    .csr_crmd_datm      (crmd_datm      ),
    .csr_dmw0_plv0      (dmw0_plv0      ),
    .csr_dmw0_plv3      (dmw0_plv3      ),
    .csr_dmw0_mat       (dmw0_mat       ),
    .csr_dmw0_pseg      (dmw0_pseg      ),
    .csr_dmw0_vseg      (dmw0_vseg      ),
    .csr_dmw1_plv0      (dmw1_plv0      ),
    .csr_dmw1_plv3      (dmw1_plv3      ),
    .csr_dmw1_mat       (dmw1_mat       ),
    .csr_dmw1_pseg      (dmw1_pseg      ),
    .csr_dmw1_vseg      (dmw1_vseg      ),
    .csr_asid_asid      (csr_asid_asid  )
);

wire        wbAllowin;
wire        memValidout;
wire [31:0] mem2wbPC;
wire [31:0] mem2wbInst;
wire [31:0] mem2wbResult;
wire [31:0] mem2wbMemvaddr;
wire [`CSRBUSL] mem2wbCsrBus;
wire [`MEMPASSBUSL] mem2wbPassBus;
wire [ 9:0] mem2wbTLBBus;

MEMU u_MEMU(
    .clk                    (aclk                ),
    .reset                  (reset              ),
    .wb_ex                  (wb_ex | wbRefetch  ),
    .ertn_flush             (ertn_flush         ),
    // handshaking signals with EXU
    .exValidout             (exValidout         ), 
    .memAllowin             (memAllowin         ),
    // handshaking signals with WB
    .wbAllowin              (wbAllowin          ),
    .memValidout            (memValidout        ),
    // data from EXU
    .ex2memCsrBus           (ex2memCsrBus       ),
    .ex2memPC               (ex2memPC           ),
    .ex2memInst             (ex2memInst         ), 
    .ex2memResult           (ex2memResult       ),
    .ex2memPassBus          (ex2memPassBus      ),
    .ex2memTLBBus           (ex2memTLBBus       ),
    // to EXU
    .memStopMemAccess       (memStopMemAccess   ),
    // data sram interface
    .data_sram_rdata        (data_sram_rdata    ),
    .data_sram_data_ok      (data_sram_data_ok  ),
    // to IDU   
    .MEM_to_IDU_csr         (MEM_to_IDU_csr     ),
    .MEM_to_IDU_gr_we       (MEM_to_IDU_gr_we   ),
    .MEM_to_IDU_dest        (MEM_to_IDU_dest    ),
    .MEM_to_IDU_valid       (MEM_to_IDU_valid   ),
    .MEM_to_IDU_forward     (MEM_to_IDU_forward ),
    .MEM_to_IDU_forward_valid (MEM_to_IDU_forward_valid ),
    // data to WB   
    .mem2wbMemvaddr         (mem2wbMemvaddr     ),
    .mem2wbPC               (mem2wbPC           ),
    .mem2wbInst             (mem2wbInst         ),
    .mem2wbResult           (mem2wbResult       ),
    .mem2wbPassBus          (mem2wbPassBus      ),
    .mem2wbCsrBus           (mem2wbCsrBus       ),
    .mem2wbTLBBus           (mem2wbTLBBus       ),
    // refetch and change TLB signals
    .ex2memRefetch          (ex2memRefetch      ),
    .ex2memChangeTLB        (ex2memChangeTLB    ),
    .ex2memChangeTLBEHI     (ex2memChangeTLBEHI ),
    .mem2wbRefetch          (mem2wbRefetch      ),
    .mem2wbChangeTLB        (mem2wbChangeTLB    ),
    .mem2wbChangeTLBEHI     (mem2wbChangeTLBEHI )
);

wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;
wire        rf_we;
WBU u_WBU(
    .clk                (aclk                ),
    .reset              (reset              ),
    // handshaking signals with MEM
    .memValidout        (memValidout        ),
    .wbAllowin          (wbAllowin          ),
    // // handshaking signals with outside (not implemented)
    // .WB_to_out_valid    (WB_to_out_valid    ),
    // data from MEM
    .mem2wbMemvaddr     (mem2wbMemvaddr     ),
    .mem2wbPC           (mem2wbPC           ),
    .mem2wbInst         (mem2wbInst         ),
    .mem2wbResult       (mem2wbResult       ),
    .mem2wbPassBus      (mem2wbPassBus      ),
    .mem2wbCsrBus       (mem2wbCsrBus       ),
    .mem2wbTLBBus       (mem2wbTLBBus       ),
    // to IDU
    .WB_to_IDU_csr      (WB_csr             ),
    .WB_to_IDU_gr_we    (WB_to_IDU_gr_we    ),
    .WB_to_IDU_dest     (WB_to_IDU_dest     ),
    .WB_to_IDU_valid    (WB_to_IDU_valid    ),
    .WB_to_IDU_forward  (WB_to_IDU_forward  ),
    // register file interface
    .rf_waddr           (rf_waddr           ),
    .rf_wdata           (rf_wdata           ),
    .rf_we              (rf_we              ),
    // debug interface
    .debug_pc           (debug_wb_pc        ),
    .debug_rf_we        (debug_wb_rf_we     ),
    .debug_rf_wnum      (debug_wb_rf_wnum   ), 
    .debug_rf_wdata     (debug_wb_rf_wdata  ),
    // csr interface
    .csr_re             (csr_re             ),
    .csr_we             (csr_we             ),
    .csr_num            (csr_num            ),
    .csr_rvalue         (csr_rvalue         ),
    .csr_wmask          (csr_wmask          ),
    .csr_wvalue         (csr_wvalue         ),
    .wb_pc              (wb_pc              ),
    .ertn_flush         (ertn_flush         ),
    .wb_ex              (wb_ex              ),
    .wb_ecode           (wb_ecode           ),
    .wb_esubcode        (wb_esubcode        ),
    .wb_vaddr           (wb_vaddr           ),
    // TLB and CSR read/write control signals
    // to TLB
    .tlbwe              (tlb_we             ),
    .tlbfill            (tlbfill            ),
    .tlbfill_randidx    (tlbfill_randidx    ),
    // to CSR
    .tlbsrch            (tlbsrch            ),
    .tlbrd              (tlbrd              ),
    .tlbsrch_found      (tlbsrch_found      ),
    .tlbsrch_index      (tlbsrch_index      ),
    // refetch signal
    .mem2wbRefetch      (mem2wbRefetch      ),
    .mem2wbChangeTLB    (mem2wbChangeTLB    ),
    .mem2wbChangeTLBEHI (mem2wbChangeTLBEHI ),
    .wbRefetch          (wbRefetch          ),
    .wbChangeTLB        (wbChangeTLB        ),
    .wbChangeTLBEHI     (wbChangeTLBEHI     ),
    .tlbr_ex            (tlbr_ex            ),
    .if_tlb_refill      (if_tlb_refill      ),
    .if_plv_ex          (if_plv_ex         )
);

regfile u_regfile(
    .clk    (aclk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
);

csr_regs u_csr_regs(
    .clk            (aclk            ),
    .reset          (reset          ),
    .csr_num        (csr_num        ),
    .csr_re         (csr_re         ),
    .csr_we         (csr_we         ),
    .csr_wmask      (csr_wmask      ),
    .csr_rvalue     (csr_rvalue     ),
    .csr_wvalue     (csr_wvalue     ),
    .ertn_pc        (ertn_pc        ),
    .ex_entry       (ex_entry       ),
    .wb_ex          (wb_ex          ),
    .wb_pc          (wb_pc          ),
    .ertn_flush     (ertn_flush     ),
    .wb_ecode       (wb_ecode       ),
    .wb_esubcode    (wb_esubcode    ),
    .wb_vaddr       (wb_vaddr       ),
    .coreid_in      (coreid_in      ),
    .isintr         (isintr         ),
    .hw_int_in      (hw_int_in      ),
    .ipi_int_in     (ipi_int_in     ),
    // tlbsrch & tlbrd write CSR
    // from WB
    .tlbsrch_found  (tlbsrch_found  ),
    .tlbsrch_index  (tlbsrch_index  ),
    .is_tlbrd       (tlbrd          ),
    .is_tlbsrch     (tlbsrch        ),
    // from TLB read port
    .r_e            (tlb_r_e        ),
    .r_vppn         (tlb_r_vppn     ),
    .r_ps           (tlb_r_ps       ),
    .r_asid         (tlb_r_asid     ),
    .r_g            (tlb_r_g        ),
    .r_ppn0         (tlb_r_ppn0     ),
    .r_plv0         (tlb_r_plv0     ),
    .r_mat0         (tlb_r_mat0     ),
    .r_d0           (tlb_r_d0       ),
    .r_v0           (tlb_r_v0       ),
    .r_ppn1         (tlb_r_ppn1     ),
    .r_plv1         (tlb_r_plv1     ),
    .r_mat1         (tlb_r_mat1     ),
    .r_d1           (tlb_r_d1       ),
    .r_v1           (tlb_r_v1       ),
    // tlbwr & tlbfill read CSR
    // to TLB write port
    .w_e            (tlb_w_e        ),
    .w_vppn         (tlb_w_vppn     ),
    .w_ps           (tlb_w_ps       ),
    .w_asid         (tlb_w_asid     ),
    .w_g            (tlb_w_g        ),
    .w_ppn0         (tlb_w_ppn0     ),
    .w_plv0         (tlb_w_plv0     ),
    .w_mat0         (tlb_w_mat0     ),
    .w_d0           (tlb_w_d0       ),
    .w_v0           (tlb_w_v0       ),
    .w_ppn1         (tlb_w_ppn1     ),
    .w_plv1         (tlb_w_plv1     ),
    .w_mat1         (tlb_w_mat1     ),
    .w_d1           (tlb_w_d1       ),
    .w_v1           (tlb_w_v1       ),
    // use for TLB read/write
    .tlb_index      (tlb_r_index    ),
    .tlbr_ex        (tlbr_ex        ),
    .if_tlb_refill  (if_tlb_refill  ),
    .if_plv_ex      (if_plv_ex      ),
    .crmd_plv       (crmd_plv       ),
    .crmd_da        (crmd_da        ),
    .crmd_pg        (crmd_pg        ),
    .crmd_datf      (crmd_datf      ),
    .crmd_datm      (crmd_datm      ),
    .dmw0_plv0      (dmw0_plv0      ),
    .dmw0_plv3      (dmw0_plv3      ),
    .dmw0_mat       (dmw0_mat       ),
    .dmw0_pseg      (dmw0_pseg      ),
    .dmw0_vseg      (dmw0_vseg      ),
    .dmw1_plv0      (dmw1_plv0      ),
    .dmw1_plv3      (dmw1_plv3      ),
    .dmw1_mat       (dmw1_mat       ),
    .dmw1_pseg      (dmw1_pseg      ),
    .dmw1_vseg      (dmw1_vseg      ),
    .asid           (csr_asid_asid  ),
    .stat_ecode     (stat_ecode     ),
    .ex_tlbrentry   (ex_tlbrentry   )
);

// TLB instantiation
assign tlb_w_index = tlbfill ? tlbfill_randidx : tlb_r_index;

tlb #(
    .TLBNUM(16)
) u_tlb (
    .clk            (aclk           ),
    // search port 0 (for fetch)
    .s0_vppn        (s0_vppn        ),
    .s0_va_bit12    (s0_va_bit12    ),
    .s0_asid        (s0_asid        ),
    .s0_found       (s0_found       ),
    .s0_index       (s0_index       ),
    .s0_ppn         (s0_ppn         ),
    .s0_ps          (s0_ps          ),
    .s0_plv         (s0_plv         ),
    .s0_mat         (s0_mat         ),
    .s0_d           (s0_d           ),
    .s0_v           (s0_v           ),
    // search port 1 (for load/store)
    .s1_vppn        (s1_vppn        ),
    .s1_va_bit12    (s1_va_bit12    ),
    .s1_asid        (s1_asid        ),
    .s1_found       (s1_found       ),
    .s1_index       (s1_index       ),
    .s1_ppn         (s1_ppn         ),
    .s1_ps          (s1_ps          ),
    .s1_plv         (s1_plv         ),
    .s1_mat         (s1_mat         ),
    .s1_d           (s1_d           ),
    .s1_v           (s1_v           ),
    // invtlb opcode
    .invtlb_valid   (invtlb_valid   ),
    .invtlb_op      (invtlb_op      ),
    // write port
    .we             (tlb_we         ),
    .w_index        (tlb_w_index    ),
    .w_e            (tlb_w_e        ),
    .w_vppn         (tlb_w_vppn     ),
    .w_ps           (tlb_w_ps       ),
    .w_asid         (tlb_w_asid     ),
    .w_g            (tlb_w_g        ),
    .w_ppn0         (tlb_w_ppn0     ),
    .w_plv0         (tlb_w_plv0     ),
    .w_mat0         (tlb_w_mat0     ),
    .w_d0           (tlb_w_d0       ),
    .w_v0           (tlb_w_v0       ),
    .w_ppn1         (tlb_w_ppn1     ),
    .w_plv1         (tlb_w_plv1     ),
    .w_mat1         (tlb_w_mat1     ),
    .w_d1           (tlb_w_d1       ),
    .w_v1           (tlb_w_v1       ),
    // read port
    .r_index        (tlb_r_index    ),
    .r_e            (tlb_r_e        ),
    .r_vppn         (tlb_r_vppn     ),
    .r_ps           (tlb_r_ps       ),
    .r_asid         (tlb_r_asid     ),
    .r_g            (tlb_r_g        ),
    .r_ppn0         (tlb_r_ppn0     ),
    .r_plv0         (tlb_r_plv0     ),
    .r_mat0         (tlb_r_mat0     ),
    .r_d0           (tlb_r_d0       ),
    .r_v0           (tlb_r_v0       ),
    .r_ppn1         (tlb_r_ppn1     ),
    .r_plv1         (tlb_r_plv1     ),
    .r_mat1         (tlb_r_mat1     ),
    .r_d1           (tlb_r_d1       ),
    .r_v1           (tlb_r_v1       )
);

endmodule
