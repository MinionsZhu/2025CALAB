module cpu_axi_bridge(
    input       clk,
    input       resetn,
    // Icache Interface
    input              icache_rd_req,
    input   	[ 2:0] icache_rd_type,
    input   	[31:0] icache_rd_addr,
    output             icache_rd_rdy,		// addr_ok
    output             icache_ret_valid,	// data_ok
	output			   icache_ret_last,
    output  	[31:0] icache_ret_data,
    // Dcache Interface
    input              dcache_rd_req,
    input       [ 2:0] dcache_rd_type,
    input   	[31:0] dcache_rd_addr,
    output             dcache_rd_rdy,		// addr_ok
    output             dcache_ret_valid,	// data_ok
    output			   dcache_ret_last,
    output  	[31:0] dcache_ret_data,

    input              dcache_wr_req,
    input       [ 2:0] dcache_wr_type,
    input   	[31:0] dcache_wr_addr,
    input   	[ 3:0] dcache_wr_wstrb,
    input      [127:0] dcache_wr_wdata,
    output             dcache_wr_rdy,		// addr_ok
    // AXI Interface
    // AR Channel
    output wire [ 3:0] arid,        // read request ID, 0 for instruction, 1 for data
    output wire [31:0] araddr,      // read address
    output wire [ 7:0] arlen,       // uncached single transfer: fixed to 0; cached burst transfer: 16/4-1
    output wire [ 2:0] arsize,      // read size, 
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
    input  wire        rlast,       // last transfer in a read burst
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
// AXI 读请求类型 (rd_type)
localparam  READ_BYTE       = 3'b000, // 1 Byte
            READ_HALFWORD   = 3'b001, // 2 Bytes
            READ_WORD       = 3'b010, // 4 Bytes
            READ_BLOCK      = 3'b100; // 16 Bytes

// AXI 写请求类型 (wr_type)
localparam  WRITE_BYTE      = 3'b000, // 1 Byte
            WRITE_HALFWORD  = 3'b001, // 2 Bytes
            WRITE_WORD      = 3'b010, // 4 Bytes
            WRITE_BLOCK     = 3'b100; // 16 Bytes

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
//wire w_inst_req;

assign r_data_req = dcache_rd_req;
assign r_inst_req = icache_rd_req;
assign w_data_req = dcache_wr_req;

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
            if( (rvalid && rready && rid == DATA_ID && rlast) 
              &&(arvalid && arready && ar_cur_state == AR_INST)
              ) begin
                r_next_state = R_INST;
            end
            else if (rvalid && rready && rid == DATA_ID && rlast) begin
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
            if( (rvalid && rready && rid == INST_ID && rlast) 
              &&(arvalid && arready && ar_cur_state == AR_DATA)
              ) begin
                r_next_state = R_DATA;
            end
            else if (rvalid && rready && rid == INST_ID && rlast) begin
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
            if (rvalid && rready && rid == INST_ID && rlast) begin
                r_next_state = R_DATA;
            end
            else if (rvalid && rready && rid == DATA_ID && rlast) begin
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
        if (rvalid && rready && rid == 4'd1 && rlast) begin
            data_outstanding <= 1'b0;
        end
        // inst read response received
        if (rvalid && rready && rid == 4'd0 && rlast) begin
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
            if (wvalid && wready && wlast) begin
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
            if (aw_cur_state == W_DATA && wvalid && wready && wlast) begin
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
// !!! 可能有问题 !!!
assign ar_block = (dcache_rd_addr <= awaddr_reg + wburst_upbd) && (dcache_rd_addr >= awaddr) && (aw_cur_state != AW_IDLE);
reg [31:0] inst_addr_reg;
reg [31:0] data_addr_reg;
reg [ 2:0] arsize_reg;
reg [ 7:0] arlen_reg;
always @(posedge clk) begin
    if (reset) begin
        inst_addr_reg <= 32'b0;
        data_addr_reg <= 32'b0;
        arsize_reg    <= 3'b0;
    end
    else begin
        if (ar_cur_state == AR_IDLE && ar_next_state == AR_INST) begin
            inst_addr_reg <= icache_rd_addr;
            arsize_reg    <= 3'b010; // instruction read is always word-aligned
            arlen_reg     <= icache_rd_type == READ_BLOCK ? 8'd3 : 8'd0; // block read or single read
        end
        if (ar_cur_state == AR_IDLE && ar_next_state == AR_DATA) begin
            data_addr_reg <= dcache_rd_addr;
            arsize_reg    <= 3'b010; // default word-aligned
            arlen_reg     <= dcache_rd_type == READ_BLOCK ? 8'd3 : 8'd0; // block read or single read
        end
    end
end
assign arid    = (ar_cur_state == AR_DATA) ? DATA_ID 
                :(ar_cur_state == AR_INST) ? INST_ID 
                : 4'b0;
assign araddr  = (ar_cur_state == AR_DATA) ? data_addr_reg
                :(ar_cur_state == AR_INST) ? inst_addr_reg
                : 32'b0;
assign arlen   = arlen_reg;
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
reg [127:0] wdata_reg;
reg [ 3:0] wstrb_reg;
reg [1:0] wbeat_count;
reg [3:0] wburst_upbd;
reg [7:0] awlen_reg;
always @(posedge clk) begin
    if (reset) begin
        awaddr_reg    <= 32'b0;
        awlen_reg     <= 8'b0;
        wdata_reg     <= 128'b0;
        wstrb_reg     <= 4'b0;
        data_size_reg <= 3'b0;
        wbeat_count   <= 2'b0;
        wburst_upbd  <= 4'b0;
    end
    else begin
        if (aw_cur_state == AW_IDLE && aw_next_state == AW_DATA) begin
            awaddr_reg    <= dcache_wr_addr;
            awlen_reg     <= dcache_wr_type == WRITE_BLOCK ? 8'd3 : 8'd0; // block write or single write
            wdata_reg     <= dcache_wr_wdata;
            wstrb_reg     <= dcache_wr_wstrb;
            data_size_reg <= 3'b010; // default word-aligned
            wbeat_count   <= dcache_wr_type == WRITE_BLOCK ? 2'd3 : 2'd0; // block write or single write
            wburst_upbd   <= (dcache_wr_type == WRITE_BYTE) ? 4'd0
                           : (dcache_wr_type == WRITE_HALFWORD) ? 4'd1 
                           : (dcache_wr_type == WRITE_WORD) ? 4'd3 
                           : (dcache_wr_type == WRITE_BLOCK) ? 4'd15 
                           : 4'd0;
        end
        else if(aw_cur_state == W_DATA && wready && wvalid && !wlast) begin
            // in case of burst write, update wdata and wbeat_count for next beat
            wdata_reg <= {32'b0, wdata_reg[127:32]};
            wbeat_count <= wbeat_count - 1'b1;
        end
    end
end
assign awid    = DATA_ID;
assign awaddr  = awaddr_reg;
assign awlen   = awlen_reg;
assign awsize  = {1'b0, data_size_reg};
assign awburst = 2'b01;
assign awlock  = 2'b00;
assign awcache = 4'b0000;
assign awprot  = 3'b000;
assign awvalid = (aw_cur_state == AW_DATA);
/* AXI w channel signals */
assign wid    = DATA_ID;
assign wdata  = wdata_reg[31:0];
assign wstrb  = wstrb_reg;
assign wvalid = (aw_cur_state == W_DATA);
assign wlast  = wbeat_count == 2'b0 && wvalid;
/* AXI b channel signals */
assign bready = (b_cur_state == B_DATA);

/* CPU interface signals */
assign icache_rd_rdy =  (ar_cur_state == AR_IDLE) && (ar_next_state == AR_INST);
assign dcache_rd_rdy =  (ar_cur_state == AR_IDLE) && !(ar_block) && !data_outstanding;
assign dcache_wr_rdy =  (aw_cur_state == AW_IDLE);

assign icache_ret_valid = (rvalid && rready && rid == INST_ID);
assign dcache_ret_valid = (rvalid && rready && rid == DATA_ID);

assign icache_ret_data = (rvalid && rready && rid == INST_ID) ? rdata:rdata_inst_reg;
assign icache_ret_last = rlast && (rvalid && rready && rid == INST_ID);
assign dcache_ret_data = (rvalid && rready && rid == DATA_ID) ? rdata:rdata_data_reg;
assign dcache_ret_last = rlast && (rvalid && rready && rid == DATA_ID);
endmodule
