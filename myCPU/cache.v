module cache(
    input               clk,
    input               resetn,

    // CPU Interface
    input               valid,      // 请求有效
    input               op,         // 1: write, 0: read
    input    [ 7:0]     index,
    input    [19:0]     tag,        // 经虚实地址转换后的物理地址高位，来自组合逻辑
    input    [ 3:0]     offset,
    input    [ 3:0]     wstrb,
    input    [31:0]     wdata,
    output              addr_ok,    // Read: 地址已被接收；Write: 地址和数据均被接收
    output              data_ok,    // Read: 数据返回；Write: 数据写入完成
    output   [31:0]     rdata,

    // AXI Interface
    output              rd_req,     // 读请求有效
    output   [ 2:0]     rd_type,    // 读请求类型 3'b000: byte, 3'b001: half, 3'b010: word, 3'b011: double word
    output   [31:0]     rd_addr,
    input               rd_rdy,     // 读请求可被接收的握手信号 1 valid
    input               ret_valid,  // 返回数据有效 1 valid
    input               ret_last,   // 返回数据最后一拍标志（ATTENTION! 注意，此处与书上不同！）
    input    [31:0]     ret_data,

    output              wr_req,     // 写请求有效
    output   [ 2:0]     wr_type,    // 写请求类型 3'b000: byte, 3'b001: half, 3'b010: word, 3'b011: double word
    output   [31:0]     wr_addr,
    output   [ 3:0]     wr_wstrb,
    output  [127:0]     wr_data,
    input               wr_rdy      // 1 valid, be set before wr_req
);

wire        reset = ~resetn;

// CPU 请求类型 (op)
localparam  READ    = 1'b0,
            WRITE   = 1'b1;

// AXI 读请求类型 (rd_type)
localparam  READ_BYTE       = 3'b000, // 1 Byte
            READ_HALFWORD   = 3'b001, // 2 Bytes
            READ_WORD       = 3'b010, // 4 Bytes
            READ_BLOCK      = 3'b100; // 16 Bytes

// AXI 写请求类型 (wr_type)
localparam  WRITE_BYTE      = 3'b000,
            WRITE_HALFWORD  = 3'b001,
            WRITE_WORD      = 3'b010,
            WRITE_BLOCK     = 3'b100;


/* 例化tagv RAM、data bank RAM 和 dirty bit regfile */
// tagv interface
wire [ 7:0] tv_addr;
wire [20:0] tv_wdata;
wire [20:0] tv_w0_rdata, tv_w1_rdata;
wire        tv_w0_en, tv_w1_en;
wire        tv_w0_we, tv_w1_we;

// data bank interface
wire [ 7:0] d_addr;
wire [31:0] d_wdata;
wire [31:0] d_w0_b0_rdata, d_w0_b1_rdata, d_w0_b2_rdata, d_w0_b3_rdata,
            d_w1_b0_rdata, d_w1_b1_rdata, d_w1_b2_rdata, d_w1_b3_rdata;
wire        d_w0_b0_en, d_w0_b1_en, d_w0_b2_en, d_w0_b3_en,
            d_w1_b0_en, d_w1_b1_en, d_w1_b2_en, d_w1_b3_en;
wire [ 3:0] d_w0_b0_we, d_w0_b1_we, d_w0_b2_we, d_w0_b3_we,
            d_w1_b0_we, d_w1_b1_we, d_w1_b2_we, d_w1_b3_we;

reg [255:0] dirty_way0;
reg [255:0] dirty_way1;
tagv_ram tagv_way0(
    .addra(tv_addr),
    .clka(clk),
    .dina(tv_wdata),
    .douta(tv_w0_rdata),
    .ena(tv_w0_en),
    .wea(tv_w0_we)
);
tagv_ram tagv_way1(
    .addra(tv_addr),
    .clka(clk),
    .dina(tv_wdata),
    .douta(tv_w1_rdata),
    .ena(tv_w1_en),
    .wea(tv_w1_we)
);
data_bank_ram way0_bank0(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w0_b0_rdata),
    .ena(d_w0_b0_en),
    .wea(d_w0_b0_we)
);
data_bank_ram way0_bank1(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w0_b1_rdata),
    .ena(d_w0_b1_en),
    .wea(d_w0_b1_we)
);
data_bank_ram way0_bank2(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w0_b2_rdata),
    .ena(d_w0_b2_en),
    .wea(d_w0_b2_we)
);
data_bank_ram way0_bank3(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w0_b3_rdata),
    .ena(d_w0_b3_en),
    .wea(d_w0_b3_we)
);
data_bank_ram way1_bank0(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w1_b0_rdata),
    .ena(d_w1_b0_en),
    .wea(d_w1_b0_we)
);
data_bank_ram way1_bank1(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w1_b1_rdata),
    .ena(d_w1_b1_en),
    .wea(d_w1_b1_we)
);
data_bank_ram way1_bank2(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w1_b2_rdata),
    .ena(d_w1_b2_en),
    .wea(d_w1_b2_we)
);
data_bank_ram way1_bank3(
    .addra(d_addr),
    .clka(clk),
    .dina(d_wdata),
    .douta(d_w1_b3_rdata),
    .ena(d_w1_b3_en),
    .wea(d_w1_b3_we)
);

/* 状态机 */
// 主状态机
localparam      IDLE      = 4'b0000,    //
                LOOKUP    = 4'b0001,    // 判断命中还是缺失
                MISS      = 4'b0010,    // 将脏块写回内存
                REPLACE   = 4'b0100,    // 动作最密集，发送缺失读请求以及脏位下的写回请求
                REFILL    = 4'b1000;    //
reg    [ 3:0]   cst, nst;
// Write Buffer 状态机
localparam      WB_IDLE   = 1'b0,
                WB_WRITE  = 1'b1;
reg             wb_cst, wb_nst;

// Request Buffer
reg             reg_op;
reg    [ 7:0]   reg_index;
reg    [19:0]   reg_tag;
reg    [ 3:0]   reg_offset;
reg    [ 3:0]   reg_wstrb;
reg    [31:0]   reg_wdata;

// Miss Buffer
reg             replace_way;    // 缺失Cache行准备要替换的 way
reg    [ 1:0]   refill_cnt;     // 内存返回数据计数(refill_cnt + 1)

// Write Buffer，仅用于处理 Hit Write (命中写)操作
reg             write_way;      // 写入的 way
reg    [ 1:0]   write_bank;     // 写入的 bank
reg    [ 7:0]   write_index;    // 写入的 index
reg    [ 3:0]   write_wstrb;    // 写入的 bank 内字节写使能
reg    [31:0]   write_wdata;    // 写入的写数据

// 替换策略：RAND，选择用LFSR实现
reg    [ 7:0]   LFSR;

// 命中判断/Tag Compare
wire            way0_v, way1_v;
wire   [19:0]   way0_tag, way1_tag;
wire            way0_hit, way1_hit;
wire            cache_hit;

// 数据选择/Data Select
wire  [127:0]   way0_data, way1_data;
wire   [31:0]   way0_load_word, way1_load_word;
wire   [31:0]   load_res;
wire  [127:0]   replace_data;
wire            replaceIsdirty;

wire            hitwrite;
wire            hitwrite_conflict1; // 可以前递 TODO: 目前先按照阻塞实现
wire            hitwrite_conflict2; // 只能阻塞
wire            hitwrite_conflict;
wire            en_lookup;  // lookup 操作的片选信号

wire   [31:0]   refill_word;
wire   [31:0]   mixed_word;

// 命中判断
assign {way0_tag, way0_v} = tv_w0_rdata;
assign {way1_tag, way1_v} = tv_w1_rdata;

assign way0_hit = way0_v && (way0_tag == reg_tag);
assign way1_hit = way1_v && (way1_tag == reg_tag);
assign cache_hit = way0_hit || way1_hit;    // TODO: Uncache 访问不考虑

// 数据选择
assign way0_data = {d_w0_b3_rdata, d_w0_b2_rdata, d_w0_b1_rdata, d_w0_b0_rdata};
assign way1_data = {d_w1_b3_rdata, d_w1_b2_rdata, d_w1_b1_rdata, d_w1_b0_rdata};

assign way0_load_word = way0_data[reg_offset[3:2] * 32 +: 32];
assign way1_load_word = way1_data[reg_offset[3:2] * 32 +: 32];
assign load_res = ({32{way0_hit}} & way0_load_word)
                | ({32{way1_hit}} & way1_load_word)
                | ({32{cst == REFILL}} & ret_data); // 选择通过 ok 信号实现
assign replace_data = replace_way ? way1_data : way0_data;

assign replaceIsdirty = replace_way ? dirty_way1[reg_index] && way1_v : dirty_way0[reg_index] && way0_v;

assign hitwrite = (cst == LOOKUP) && cache_hit & (reg_op == WRITE);
assign hitwrite_conflict1 = (cst == LOOKUP)     // 主状态机处于 LOOKUP 状态
                && (reg_op == WRITE)            // 且发现 Store 操作命中 Cache
                && (op == READ && valid)        // 此时流水线发来一个新的 Load 类的 Cache 访问请求
                && ({tag, index, offset[3:2]}   // 并且该 Load 请求与 LOOKUP 状态的 Store 请求地址存在写后读相关
                == {reg_tag, reg_index, reg_offset[3:2]}); 
assign hitwrite_conflict2 = (wb_cst == WRITE)   // Write Buffer 状态机处于 WRITE 状态，也就是正在写入一个待写数据到 Cache 中
                && (op == READ && valid)        // 此时流水线发来一个新的 Load 类的 Cache 访问请求
                && (offset[3:2] == write_bank); // 并且该 Load 请求与 Write Buffer 里的待写请求的地址重叠。“地址重叠”是指 Load 请求地址的[3:2]与 Store 请求地址的[3:2]相等。
assign hitwrite_conflict = hitwrite_conflict1 || hitwrite_conflict2;
assign en_lookup = (cst == IDLE || cst == LOOKUP) && valid && ~hitwrite_conflict;

assign mixed_word = {reg_wstrb[3] ? reg_wdata[31:24] : ret_data[31:24],
                     reg_wstrb[2] ? reg_wdata[23:16] : ret_data[23:16],
                     reg_wstrb[1] ? reg_wdata[15: 8] : ret_data[15: 8],
                     reg_wstrb[0] ? reg_wdata[ 7: 0] : ret_data[ 7: 0]};
assign refill_word = (refill_cnt == reg_offset[3:2] && reg_op == WRITE) ? mixed_word : ret_data;


assign addr_ok = (cst == IDLE) || (cst == LOOKUP && valid && ~hitwrite_conflict);
assign data_ok = (cst == LOOKUP && (cache_hit || reg_op == WRITE)) || (cst == REFILL && ret_valid && refill_cnt == reg_offset[3:2]);
assign rdata = load_res;

assign rd_req = (cst == REPLACE);   // 仅在 REPLACE 状态发起读请求
assign rd_type = READ_BLOCK;
assign rd_addr = {reg_tag, reg_index, 4'b0000};

assign wr_req = (cst == REPLACE) && replaceIsdirty; // 仅在 REPLACE 状态且替换块为脏块时发起写请求
assign wr_type = WRITE_BLOCK;
assign wr_addr = {replace_way ? way1_tag : way0_tag, reg_index, 4'b0000};
assign wr_wstrb = 4'hf;
assign wr_data = replace_data;


assign tv_w0_en = en_lookup || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b0);
assign tv_w1_en = en_lookup || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b1);
assign tv_w0_we = (cst == REFILL && replace_way == 1'b0) && ret_valid && ret_last;
assign tv_w1_we = (cst == REFILL && replace_way == 1'b1) && ret_valid && ret_last;
assign tv_wdata = {reg_tag, 1'b1};  // 有效位写1
assign tv_addr = (cst == MISS || cst == REPLACE || cst == REFILL) ? reg_index : index;

assign d_w0_b0_en = (en_lookup && (offset[3:2] == 2'b00))
                 || (hitwrite && write_way == 1'b0)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b0);
assign d_w0_b1_en = (en_lookup && (offset[3:2] == 2'b01))
                 || (hitwrite && write_way == 1'b0)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b0);
assign d_w0_b2_en = (en_lookup && (offset[3:2] == 2'b10))
                 || (hitwrite && write_way == 1'b0)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b0);
assign d_w0_b3_en = (en_lookup && (offset[3:2] == 2'b11))
                 || (hitwrite && write_way == 1'b0)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b0);
assign d_w1_b0_en = (en_lookup && (offset[3:2] == 2'b00))
                 || (hitwrite && write_way == 1'b1)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b1);
assign d_w1_b1_en = (en_lookup && (offset[3:2] == 2'b01))
                 || (hitwrite && write_way == 1'b1)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b1);
assign d_w1_b2_en = (en_lookup && (offset[3:2] == 2'b10))
                 || (hitwrite && write_way == 1'b1)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b1);
assign d_w1_b3_en = (en_lookup && (offset[3:2] == 2'b11))
                 || (hitwrite && write_way == 1'b1)
                 || ((cst == MISS || cst == REPLACE || cst == REFILL) && replace_way == 1'b1);

assign d_w0_b0_we = ({4{hitwrite && (write_way == 1'b0) && (write_bank == 2'b00)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b0) && (refill_cnt == 2'b00) && ret_valid}});
assign d_w0_b1_we = ({4{hitwrite && (write_way == 1'b0) && (write_bank == 2'b01)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b0) && (refill_cnt == 2'b01) && ret_valid}});
assign d_w0_b2_we = ({4{hitwrite && (write_way == 1'b0) && (write_bank == 2'b10)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b0) && (refill_cnt == 2'b10) && ret_valid}});
assign d_w0_b3_we = ({4{hitwrite && (write_way == 1'b0) && (write_bank == 2'b11)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b0) && (refill_cnt == 2'b11) && ret_valid}});
assign d_w1_b0_we = ({4{hitwrite && (write_way == 1'b1) && (write_bank == 2'b00)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b1) && (refill_cnt == 2'b00) && ret_valid}});
assign d_w1_b1_we = ({4{hitwrite && (write_way == 1'b1) && (write_bank == 2'b01)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b1) && (refill_cnt == 2'b01) && ret_valid}});
assign d_w1_b2_we = ({4{hitwrite && (write_way == 1'b1) && (write_bank == 2'b10)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b1) && (refill_cnt == 2'b10) && ret_valid}});
assign d_w1_b3_we = ({4{hitwrite && (write_way == 1'b1) && (write_bank == 2'b11)}} & write_wstrb)
                  | ({4{cst == REFILL && (replace_way == 1'b1) && (refill_cnt == 2'b11) && ret_valid}});

assign d_wdata = {32{cst == REFILL}} & refill_word
               | {32{hitwrite}} & write_wdata;
assign d_addr = (cst == MISS || cst == REPLACE || cst == REFILL) ? reg_index :
                (hitwrite) ? write_index : index;
/* 状态机实现 */
always @(posedge clk) begin
    if (reset) begin
        // 主状态机复位
        cst <= IDLE;
        // Write Buffer 状态机复位
        wb_cst <= WB_IDLE;
    end
    else begin
        // 主状态机状态转移
        cst <= nst;
        // Write Buffer 状态转移
        wb_cst <= wb_nst;
    end
end
always @(*) begin
    if (reset) begin
        nst = IDLE;
    end
    else begin
    case (cst)
    IDLE: begin
        if (valid && (~hitwrite_conflict)) begin
            nst = LOOKUP;
        end
        else begin
            nst = IDLE;
        end
    end
    LOOKUP: begin
        if (~cache_hit) begin
            nst = MISS;
        end
        else if (valid && (~hitwrite_conflict)) begin   // 连续访问
            nst = LOOKUP;
        end
        else begin
            nst = IDLE;
        end
    end
    MISS: begin
        if (wr_rdy) begin   // 无论脏不脏都要进入下一个状态，由replace状态处理
            nst = REPLACE;
        end
        else begin
            nst = MISS;
        end
    end
    REPLACE: begin
        if (rd_rdy) begin
            nst = REFILL;
        end
        else begin
            nst = REPLACE;
        end
    end
    REFILL: begin
        if (ret_valid && ret_last) begin
            nst = IDLE;
        end
        else begin
            nst = REFILL;
        end
    end
    default:begin
        nst = IDLE;
    end
    endcase
    end
end

// 类似于DMA，给数据就写
always @(*) begin
    case (wb_cst)
    WB_IDLE: begin
        if (hitwrite) begin
            wb_nst = WB_WRITE;
        end
        else begin
            wb_nst = WB_IDLE;
        end
    end
    WB_WRITE: begin
        if (hitwrite) begin
            wb_nst = WB_WRITE;
        end
        else begin
            wb_nst = WB_IDLE;
        end
    end
    default:begin
        wb_nst = WB_IDLE;
    end 
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        dirty_way0 <= 256'b0;
        dirty_way1 <= 256'b0;
    end
    else begin
        // 处理命中写操作的脏位更新
        if (hitwrite) begin
            if (way1_hit) begin
                dirty_way1[reg_index] <= 1'b1;
            end
            else begin
                dirty_way0[reg_index] <= 1'b1;
            end
        end
        // 处理替换写回操作的脏位清除
        if (cst == REFILL) begin
            if (replace_way && ret_valid && ret_last) begin
                dirty_way1[reg_index] <= reg_op;    // 如果是 Store 缺失则为 1，Load 缺失则为 0
            end
            else begin
                dirty_way0[reg_index] <= reg_op;    // 如果是 Store 缺失则为 1，Load 缺失则为 0
            end
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        LFSR <= 8'hAC;
    end
    else begin
        // 使用多项式 x^8 + x^6 + x^5 + x^4 + 1 的抽头逻辑
        // 每一拍都在发生移位，产生新的随机状态
        LFSR <= {LFSR[6:0], LFSR[7] ^ LFSR[5] ^ LFSR[4] ^ LFSR[3]};
    end
end
always @(posedge clk) begin
    if (reset) begin
        replace_way <= 1'b0;
    end
    // else if (cst == MISS && wr_rdy) begin
    else if (ret_valid && ret_last) begin
        replace_way <= LFSR[0];
    end
end
always @(posedge clk) begin
    if (reset || cst != REFILL) begin
        refill_cnt <= 2'b00;
    end
    else if (cst == REFILL && ret_valid) begin
        refill_cnt <= refill_cnt + 2'b01;
    end
end

// Request Buffer
always @(posedge clk) begin
    if (reset) begin
        reg_op <= 1'b0;
        reg_index <= 8'b0;
        reg_tag <= 20'b0;
        reg_offset <= 4'b0;
        reg_wstrb <= 4'b0;
        reg_wdata <= 32'b0;
    end
    else if (addr_ok && valid) begin
        reg_op <= op;
        reg_index <= index;
        reg_tag <= tag;
        reg_offset <= offset;
        reg_wstrb <= wstrb;
        reg_wdata <= wdata;
    end
end

// Write Buffer
always @(posedge clk) begin
    if (reset) begin
        write_way <= 1'b0;
        write_bank <= 2'b00;
        write_index <= 8'b0;
        write_wstrb <= 4'b0;
        write_wdata <= 32'b0;
    end
    else if (hitwrite) begin
        write_way <= way1_hit;
        write_bank <= reg_offset[3:2];
        write_index <= reg_index;
        write_wstrb <= reg_wstrb;
        write_wdata <= reg_wdata;
    end
end

endmodule