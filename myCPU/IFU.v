module IFU(
    input  wire        clk,
    input  wire        reset,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,

    // to IDU
    //output wire [31:0] seq_pc,
    output wire [31:0] inst_to_IDU,
    output wire [31:0] pc_to_IDU,
    input  wire        br_taken,
    input  wire        br_taken_cancel,
    input  wire [31:0] br_target,

    // handshaking signals with IDU
    input  wire        IDU_allow_in,
    output wire        IFU_to_IDU_valid,
    output wire        IFU_ready_go
);
    // preif stage
    wire preIF_to_IF_valid;
    reg IFU_valid;
    wire IFU_allow_in;

    reg  [31:0] pc;
    wire [31:0] seq_pc;
    wire [31:0] nextpc;

    // inst sram interface
    assign inst_sram_en = preIF_to_IF_valid && IDU_allow_in;
    assign inst_sram_we = 4'b0;
    assign inst_sram_addr = nextpc;
    assign inst_sram_wdata = 32'b0;
    
    // IF status
    always @(posedge clk) begin
        if (reset) begin
            IFU_valid <= 1'b0;
        end
        else if (IFU_allow_in) begin
            IFU_valid <= preIF_to_IF_valid;
        end
        else if(br_taken_cancel) begin
            IFU_valid <= 1'b0;
        end
    end
    assign preIF_to_IF_valid = !reset;
    assign IFU_ready_go = 1'b1;
    assign IFU_to_IDU_valid = IFU_valid && IFU_ready_go;
    assign IFU_allow_in = !IFU_valid || (IFU_ready_go && IDU_allow_in);
    
    // pc register & output to IDU
    assign seq_pc = pc + 4;
    assign nextpc = br_taken ? br_target : seq_pc;
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc;
        end
        else if (preIF_to_IF_valid && IFU_allow_in) begin
            pc <= nextpc;
        end
    end

    assign inst_to_IDU = inst_sram_rdata;
    assign pc_to_IDU = pc;

endmodule
