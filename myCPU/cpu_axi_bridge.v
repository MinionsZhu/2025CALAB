module cpu_axi_bridge(
    input       clk,
    input       resetn,
    // CPU-Instruction SRAM Interface
    input  wire        inst_req,
    input  wire        inst_wr,
    input  wire [ 1:0] inst_size,
    input  wire [ 3:0] inst_wstrb,
    input  wire [31:0] inst_addr,
    input  wire [31:0] inst_wdata,
    output wire        inst_addr_ok,
    output wire        inst_data_ok,
    output wire [31:0] inst_rdata,
    // CPU-Data SRAM Interface
    input  wire        data_req,
    input  wire        data_wr,
    input  wire [ 1:0] data_size,
    input  wire [ 3:0] data_wstrb,
    input  wire [31:0] data_addr,
    input  wire [31:0] data_wdata,
    output wire        data_addr_ok,
    output wire        data_data_ok,
    output wire [31:0] data_rdata,
    // AXI Interface
    // AR Channel
    output wire [ 3:0] arid,        // read request ID, 0 for instruction, 1 for data
    output wire [31:0] araddr,      // read address
    output wire [ 7:0] arlen,       // currently fixed to 0
    output wire [ 2:0] arsize,      // read size
    output wire [ 1:0] arburst,     // currently fixed to 01 (INCR)
    output wire [ 1:0] arlock,      // currently fixed to 00 (normal access)
    output wire [ 3:0] arcache,     // currently fixed to 0000
    output wire [ 2:0] arprot,      // read protection type (currently fixed to 000)
    output wire        arvalid,     // read address valid
    input  wire        arready,     // handshake signal from slave, slave is ready to accept address
    // R Channel
    input  wire [ 3:0] rid,         // read ID, should be same as arid
    input  wire [31:0] rdata,       // read data
    input  wire [ 1:0] rresp,       // ignored
    input  wire        rlast,       // ignored
    input  wire        rvalid,      // read valid, slave is providing valid read data
    output wire        rready,      // read ready, bridge is capable of receiving data
    // AW Channel
    output wire [ 3:0] awid,        // write request ID, fixed to 1 for data
    output wire [31:0] awaddr,      // write address
    output wire [ 7:0] awlen,       // currently fixed to 0
    output wire [ 2:0] awsize,      // write size
    output wire [ 1:0] awburst,     // currently fixed to 01 (INCR)
    output wire [ 1:0] awlock,      // currently fixed to 00 (normal access)
    output wire [ 3:0] awcache,     // currently fixed to 0000
    output wire [ 2:0] awprot,      // write protection type (currently fixed to 000)
    output wire        awvalid,     // write address valid
    input  wire        awready,     // handshake signal from slave, slave is ready to accept address
    // W Channel
    output wire [ 3:0] wid,        // write ID, fixed to 1 for data
    output wire [31:0] wdata,      // write data
    output wire [ 3:0] wstrb,      // write strobe
    output wire        wlast,      // write last, fixed to 1
    output wire        wvalid,     // write valid, bridge is providing valid write data
    input  wire        wready,     // handshake signal from slave, slave is ready to accept data
    // B Channel
    input  wire [ 3:0] bid,        // should be consistent to wid/awid for same write transaction, currently ignored
    input  wire [ 1:0] bresp,      // ignored
    input  wire        bvalid,     // write response valid, slave is providing valid write response
    output wire        bready      // write response ready, bridge is capable of receiving write response
);
wire reset;
assign reset = ~resetn;

localparam DATA_ID  = 4'd1,
           INST_ID  = 4'd0;
// State machine signals
localparam AR_IDLE  = 3'b001,
           AR_DATA  = 3'b010,
           AR_INST  = 3'b100;
localparam R_IDLE   = 4'b0001,
           R_DATA   = 4'b0010,
           R_INST   = 4'b0100,
           R_BOTH   = 4'b1000;
localparam AW_IDLE  = 4'b0001,
           AW_DATA  = 4'b0010,
           W_DATA   = 4'b0100,
           W_WAIT   = 4'b1000;
localparam B_IDLE   = 2'b01,
           B_DATA   = 2'b10;

reg [2:0] ar_cur_state;
reg [2:0] ar_next_state;

reg [3:0] r_cur_state;
reg [3:0] r_next_state;

reg [3:0] aw_cur_state;
reg [3:0] aw_next_state;

reg [1:0] b_cur_state;
reg [1:0] b_next_state;

// these two regs track whether there are outstanding read requests
// data_outstanding: 1 if there is an outstanding data read request
// inst_outstanding: 1 if there is an outstanding instruction read request
// if so, block new inst/data read requests from being issued
reg data_outstanding;
reg inst_outstanding;

// block new AR requests when there are outstanding write requests
wire ar_block;

wire r_data_req;
wire r_inst_req;
wire w_data_req;
wire w_inst_req;

assign r_data_req = data_req && ~data_wr;
assign r_inst_req = inst_req && ~inst_wr;
assign w_data_req = data_req && data_wr;
assign w_inst_req = inst_req && inst_wr;

always @(posedge clk) begin
    if (reset) begin
        ar_cur_state <= AR_IDLE;
    end
    else begin
        ar_cur_state <= ar_next_state;
    end
end
/* AR state machine */
always @(*) begin
    case (ar_cur_state)
        AR_IDLE: begin
            if (r_data_req && !ar_block && !data_outstanding) begin
                ar_next_state = AR_DATA;
            end
            else if (r_inst_req && !ar_block && !inst_outstanding) begin
                ar_next_state = AR_INST;
            end
            else begin
                ar_next_state = AR_IDLE;
            end
        end
        AR_DATA: begin
            if (arvalid && arready) begin
                ar_next_state = AR_IDLE;
            end
            else begin
                ar_next_state = AR_DATA;
            end
        end
        AR_INST: begin
            if (arvalid && arready) begin
                ar_next_state = AR_IDLE;
            end
            else begin
                ar_next_state = AR_INST;
            end
        end
        default: begin
            ar_next_state = AR_IDLE;
        end
    endcase
end
/* AR state machine ends*/

/* R state machine */
always @(posedge clk) begin
    if (reset) begin
        r_cur_state <= R_IDLE;
    end
    else begin
        r_cur_state <= r_next_state;
    end
end

always @(*) begin
    case (r_cur_state)
        R_IDLE: begin
            if (arvalid && arready && ar_cur_state == AR_DATA) begin
                r_next_state = R_DATA;
            end
            else if (arvalid && arready && ar_cur_state == AR_INST) begin
                r_next_state = R_INST;
            end
            else begin
                r_next_state = R_IDLE;
            end
        end
        R_DATA: begin
            if( (rvalid && rready && rid == DATA_ID) 
              &&(arvalid && arready && ar_cur_state == AR_INST)
              ) begin
                r_next_state = R_INST;
            end
            else if (rvalid && rready && rid == DATA_ID) begin
                r_next_state = R_IDLE;
            end
            else if(arvalid && arready && ar_cur_state == AR_INST) begin
                r_next_state = R_BOTH;
            end
            else begin
                r_next_state = R_DATA;
            end
        end
        R_INST: begin
            if( (rvalid && rready && rid == INST_ID) 
              &&(arvalid && arready && ar_cur_state == AR_DATA)
              ) begin
                r_next_state = R_DATA;
            end
            else if (rvalid && rready && rid == INST_ID) begin
                r_next_state = R_IDLE;
            end
            else if(arvalid && arready && ar_cur_state == AR_DATA) begin
                r_next_state = R_BOTH;
            end
            else begin
                r_next_state = R_INST;
            end
        end
        R_BOTH: begin
            if (rvalid && rready && rid == INST_ID) begin
                r_next_state = R_DATA;
            end
            else if (rvalid && rready && rid == DATA_ID) begin
                r_next_state = R_INST;
            end
            else begin
                r_next_state = R_BOTH;
            end
        end
        default: begin
            r_next_state = R_IDLE;
        end
    endcase
end
/* R state machine ends*/
/* 2 outstanding read requests tracking */
always @(posedge clk) begin
    if (reset) begin
        data_outstanding <= 1'b0;
        inst_outstanding <= 1'b0;
    end
    else begin
        // data read request issued
        if (ar_cur_state == AR_IDLE && ar_next_state == AR_DATA) begin
            data_outstanding <= 1'b1;
        end
        // inst read request issued
        if (ar_cur_state == AR_IDLE && ar_next_state == AR_INST) begin
            inst_outstanding <= 1'b1;
        end
        // data read response received
        if (rvalid && rready && rid == 4'd1) begin
            data_outstanding <= 1'b0;
        end
        // inst read response received
        if (rvalid && rready && rid == 4'd0) begin
            inst_outstanding <= 1'b0;
        end
    end
end

/* AW state machine */
always @(posedge clk) begin
    if (reset) begin
        aw_cur_state <= AW_IDLE;
    end
    else begin
        aw_cur_state <= aw_next_state;
    end
end

always @(*) begin
    case (aw_cur_state)
        AW_IDLE: begin
            if (w_data_req) begin
                aw_next_state = AW_DATA;
            end
            else begin
                aw_next_state = AW_IDLE;
            end
        end
        AW_DATA: begin
            if (awvalid && awready) begin
                aw_next_state = W_DATA;
            end
            else begin
                aw_next_state = AW_DATA;
            end
        end
        W_DATA: begin
            if (wvalid && wready) begin
                aw_next_state = W_WAIT;
            end
            else begin
                aw_next_state = W_DATA;
            end
        end
        W_WAIT: begin
            if (bvalid && bready) begin
                aw_next_state = AW_IDLE;
            end
            else begin
                aw_next_state = W_WAIT;
            end
        end
        default: begin
            aw_next_state = AW_IDLE;
        end
    endcase
end
/* AW state machine ends*/
/* B state machine */
always @(posedge clk) begin
    if (reset) begin
        b_cur_state <= B_IDLE;
    end
    else begin
        b_cur_state <= b_next_state;
    end
end

always @(*) begin
    case (b_cur_state)
        B_IDLE: begin
            if (aw_cur_state == W_DATA && wvalid && wready) begin
                b_next_state = B_DATA;
            end
            else begin
                b_next_state = B_IDLE;
            end
        end
        B_DATA: begin
            if (bvalid && bready) begin
                b_next_state = B_IDLE;
            end
            else begin
                b_next_state = B_DATA;
            end
        end
        default: begin
            b_next_state = B_IDLE;
        end
    endcase
end
/* B state machine ends*/

/* AXI ar channel signals */
// cpu will not issue a data read request when there is an outstanding write request
// thus if aw_cur_state == AW_IDLE, there will be no conflict of insuing read and write requests at the same time
assign ar_block = (data_addr == awaddr) && (aw_cur_state != AW_IDLE);
reg [31:0] inst_addr_reg;
reg [31:0] data_addr_reg;
reg [ 2:0] arsize_reg;
always @(posedge clk) begin
    if (reset) begin
        inst_addr_reg <= 32'b0;
        data_addr_reg <= 32'b0;
        arsize_reg    <= 3'b0;
    end
    else begin
        if (ar_cur_state == AR_IDLE && ar_next_state == AR_INST) begin
            inst_addr_reg <= inst_addr;
            arsize_reg    <= inst_size;
        end
        if (ar_cur_state == AR_IDLE && ar_next_state == AR_DATA) begin
            data_addr_reg <= data_addr;
            arsize_reg    <= data_size;
        end
    end
end
assign arid    = (ar_cur_state == AR_DATA) ? DATA_ID 
                :(ar_cur_state == AR_INST) ? INST_ID 
                : 4'b0;
assign araddr  = (ar_cur_state == AR_DATA) ? data_addr_reg
                :(ar_cur_state == AR_INST) ? inst_addr_reg
                : 32'b0;
assign arlen   = 8'b0;
assign arsize  = arsize_reg;
assign arburst = 2'b01;
assign arlock  = 2'b00;
assign arcache = 4'b0000;
assign arprot  = 3'b000;
assign arvalid = (ar_cur_state == AR_DATA) || (ar_cur_state == AR_INST);

/* AXI r channel signals */
reg [31:0] rdata_inst_reg;
reg [31:0] rdata_data_reg;
always @(posedge clk) begin
    if (reset) begin
        rdata_inst_reg <= 32'b0;
        rdata_data_reg <= 32'b0;
    end
    else begin
        if (rvalid && rready && rid == INST_ID) begin
            rdata_inst_reg <= rdata;
        end
        if (rvalid && rready && rid == DATA_ID) begin
            rdata_data_reg <= rdata;
        end
    end
end
assign rready = (r_cur_state == R_DATA) || (r_cur_state == R_INST) || (r_cur_state == R_BOTH);

/* AXI aw channel signals */
reg [31:0] awaddr_reg;
reg [ 2:0] data_size_reg;
reg [31:0] wdata_reg;
reg [ 3:0] wstrb_reg;
always @(posedge clk) begin
    if (reset) begin
        awaddr_reg    <= 32'b0;
        wdata_reg     <= 32'b0;
        wstrb_reg     <= 4'b0;
        data_size_reg <= 3'b0;
    end
    else begin
        if (aw_cur_state == AW_IDLE && aw_next_state == AW_DATA) begin
            awaddr_reg    <= data_addr;
            wdata_reg     <= data_wdata;
            wstrb_reg     <= data_wstrb;
            data_size_reg <= data_size;
        end
    end
end
assign awid    = DATA_ID;
assign awaddr  = awaddr_reg;
assign awlen   = 8'b0;
assign awsize  = {1'b0, data_size_reg};
assign awburst = 2'b01;
assign awlock  = 2'b00;
assign awcache = 4'b0000;
assign awprot  = 3'b000;
assign awvalid = (aw_cur_state == AW_DATA);
/* AXI w channel signals */
assign wid    = DATA_ID;
assign wdata  = wdata_reg;
assign wstrb  = wstrb_reg;
assign wvalid = (aw_cur_state == W_DATA);
assign wlast  = 1'b1;
/* AXI b channel signals */
assign bready = (b_cur_state == B_DATA);

/* CPU interface signals */
assign inst_addr_ok =  (ar_cur_state == AR_IDLE) && (ar_next_state == AR_INST);

assign data_addr_ok = ((ar_cur_state == AR_IDLE) && (ar_next_state == AR_DATA))
                    ||((aw_cur_state == AW_IDLE) && (aw_next_state == AW_DATA));

assign inst_data_ok = (rvalid && rready && rid == INST_ID);
assign data_data_ok = (rvalid && rready && rid == DATA_ID)
                    ||(b_cur_state == B_DATA && b_next_state == B_IDLE);

assign inst_rdata = (rvalid && rready && rid == INST_ID) ? rdata:rdata_inst_reg;
assign data_rdata = (rvalid && rready && rid == DATA_ID) ? rdata:rdata_data_reg;
endmodule
