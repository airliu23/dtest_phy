//============================================================================
// USB PD BMC 收发器 (TX/RX 互斥，半双工，带自动握手)
// 功能：
// - TX 发送时禁用 RX
// - RX 使能时禁止 TX 启动
// - TX 和 RX 共用同一条 BMC 线（半双工）
// - RX 接收完成后自动发送 GoodCRC 确认
// - TX 发送后等待接收 GoodCRC 确认
//============================================================================

`timescale 1ns/1ps

module pd_bmc_transceiver (
    input  wire         clk,
    input  wire         rst_n,
    
    // TX 接口
    input  wire [15:0]  tx_header_i,
    input  wire [239:0] tx_data_flat_i,
    input  wire [4:0]   tx_data_len_i,
    input  wire         tx_start_i,
    output wire         tx_done_o,
    output wire         tx_busy_o,
    output wire         tx_active_o,
    
    // RX 接口
    output wire [15:0]  rx_header_o,
    output wire [239:0] rx_data_flat_o,
    output wire [4:0]   rx_data_len_o,
    output wire         rx_done_o,
    output wire         rx_busy_o,
    output wire         rx_error_o,
    output wire         rx_crc_ok_o,
    input  wire         rx_en_i,
    
    // 配置接口
    input  wire [7:0]   header_good_crc,  // bit0 -> power role; bit1 -> data_role; bit2 -> cable_role;
    input  wire [2:0]   msg_id_i,
    input  wire         auto_goodcrc_i,   // 启用自动 GoodCRC 握手
    input  wire         loopback_mode_i,
    output wire         tx_fail_o,        // TX 失败（未收到 ACK）
    
    // BMC 物理接口 (半双工，单线)
    inout  wire         bmc_pad,         // BMC 线 (TX/RX 共用)
    output wire         bmc_pad_en       // 输出使能 (用于外部三态控制)
);

//============================================================================
// 参数定义
//============================================================================
// GoodCRC 消息定义 (USB PD 规范)
// Header: [15:12]=Type(0x01=GoodCRC), [11:8]=PortRole, [7:5]=SpecRev, [4:0]=MsgID
localparam [15:0] GOODCRC_HEADER_BASE = 16'h0001;  // Type=GoodCRC (0x01)

// 超时参数 (基于 50MHz 时钟)
localparam ACK_TIMEOUT_CNT = 2500000;  // 50ms 超时等待 ACK (50MHz * 0.05s)
localparam GOODCRC_DELAY_CNT = 5000;   // 100μs 延时后发送 GoodCRC (50MHz * 0.0001s)

//============================================================================
// 内部信号
//============================================================================
wire int_tx_busy;
wire int_rx_busy;
wire bmc_tx_out;        // TX 模块输出

// 原始 TX/RX 信号
wire raw_tx_done;
wire raw_rx_done;

// TX 握手状态机
localparam TX_ST_IDLE       = 3'd0;
localparam TX_ST_SENDING    = 3'd1;
localparam TX_ST_WAIT_ACK   = 3'd2;
localparam TX_ST_DONE       = 3'd3;
localparam TX_ST_FAIL       = 3'd4;

reg [2:0] tx_state;
reg [21:0] ack_timeout_cnt;  // ACK 超时计数器
reg tx_done_reg;
reg tx_fail_reg;
reg goodcrc_received_reg;    // 标记是否已收到 GoodCRC

// RX 握手状态机
localparam RX_ST_IDLE       = 3'd0;
localparam RX_ST_RECEIVING  = 3'd1;
localparam RX_ST_DELAY_ACK  = 3'd2;  // 新增：延时等待状态
localparam RX_ST_SEND_ACK   = 3'd3;
localparam RX_ST_DONE       = 3'd4;

reg [2:0] rx_state;
reg rx_done_reg;
reg [12:0] goodcrc_delay_cnt;  // GoodCRC 发送延时计数器

// GoodCRC 发送缓冲
reg [15:0]  ack_header;
reg [239:0] ack_data_flat;
reg [4:0]   ack_data_len;
reg         ack_tx_start;
wire        ack_tx_done;
wire        ack_tx_busy;

// 接收到的消息信息
reg [15:0]  rx_header_reg;
reg [239:0] rx_data_flat_reg;
reg [4:0]   rx_data_len_reg;
reg         rx_crc_ok_reg;

// 检测接收到的消息是否为 GoodCRC
wire rx_is_goodcrc;

// 最终 TX 输入 (用户数据或 ACK 数据)
wire [15:0]  final_tx_header;
wire [239:0] final_tx_data_flat;
wire [4:0]   final_tx_data_len;
wire         final_tx_start;

//============================================================================
// TX/RX 互斥逻辑 (考虑自动 ACK 发送)
//============================================================================
// TX 可以启动的条件：RX 未忙，且不在发送 ACK 过程中
wire tx_can_start = tx_start_i && !int_rx_busy && (rx_state != RX_ST_SEND_ACK) && !ack_tx_busy;

// RX 使能的条件：TX 未忙 (在等待 ACK 时需要启用 RX 来接收 GoodCRC)
wire rx_actual_en = rx_en_i && !int_tx_busy && !ack_tx_busy;

//============================================================================
// 半双工逻辑：TX/RX 共用同一条线
//============================================================================
assign bmc_pad = int_tx_busy ? bmc_tx_out : 1'bz;
assign bmc_pad_en = int_tx_busy;

//============================================================================
// 检测接收到的消息是否为 GoodCRC (Type = 0x01)
// 注意：在 RX_ST_RECEIVING 状态检测到 raw_rx_done 时使用 rx_header_o（实时值）
// 在 TX_ST_WAIT_ACK 状态使用 rx_header_reg（保存值），因为 rx_header_o 可能被后续数据覆盖
//============================================================================
assign rx_is_goodcrc = (rx_header_o[14:12] == 3'b001);  // GoodCRC type = 0x01
wire rx_is_goodcrc_reg = (rx_header_reg[14:12] == 3'b001);  // 使用保存的 header 判断

//============================================================================
// TX 输出选择 (用户数据 vs ACK 数据)
//============================================================================
assign final_tx_header    = (rx_state == RX_ST_SEND_ACK) ? ack_header : tx_header_i;
assign final_tx_data_flat = (rx_state == RX_ST_SEND_ACK) ? ack_data_flat : tx_data_flat_i;
assign final_tx_data_len  = (rx_state == RX_ST_SEND_ACK) ? ack_data_len : tx_data_len_i;
assign final_tx_start     = (rx_state == RX_ST_SEND_ACK) ? ack_tx_start : tx_can_start;

//============================================================================
// TX 握手状态机
// 实现：发送消息后等待 ACK (GoodCRC)
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= TX_ST_IDLE;
        ack_timeout_cnt <= 22'd0;
        tx_done_reg <= 1'b0;
        tx_fail_reg <= 1'b0;
        goodcrc_received_reg <= 1'b0;
    end else begin
        tx_done_reg <= 1'b0;
        tx_fail_reg <= 1'b0;
        
        // 在 TX_ST_WAIT_ACK 状态下，检测是否收到 GoodCRC
        if (tx_state == TX_ST_WAIT_ACK) begin
            if (rx_state == RX_ST_DONE && rx_is_goodcrc_reg) begin
                goodcrc_received_reg <= 1'b1;
            end
        end else begin
            goodcrc_received_reg <= 1'b0;
        end
        
        case (tx_state)
            TX_ST_IDLE: begin
                ack_timeout_cnt <= 22'd0;
                if (tx_start_i && !int_tx_busy && (rx_state != RX_ST_SEND_ACK)) begin
                    tx_state <= TX_ST_SENDING;
                end
            end
            
            TX_ST_SENDING: begin
                if (raw_tx_done) begin
                    if (auto_goodcrc_i) begin
                        // 需要等待 ACK
                        tx_state <= TX_ST_WAIT_ACK;
                        ack_timeout_cnt <= 22'd0;
                    end else begin
                        // 不需要 ACK，直接完成
                        tx_state <= TX_ST_DONE;
                    end
                end
            end
            
            TX_ST_WAIT_ACK: begin
                // 等待接收 GoodCRC
                // 使用 goodcrc_received_reg 或 rx_is_goodcrc_reg 判断
                // 因为 RX_ST_DONE 只持续一个周期，需要用寄存器保存状态
                if (ack_timeout_cnt[19:0] == 0) begin  // 每约1ms打印一次
                    $display("[TX State] WAIT_ACK: timeout=%0d, rx_state=%0d, rx_is_goodcrc_reg=%b, goodcrc_received_reg=%b", 
                             ack_timeout_cnt, rx_state, rx_is_goodcrc_reg, goodcrc_received_reg);
                end
                if (goodcrc_received_reg || (rx_state == RX_ST_DONE && rx_is_goodcrc_reg)) begin
                    // 收到 GoodCRC，发送成功
                    $display("[TX State] GoodCRC received! Exiting WAIT_ACK");
                    tx_state <= TX_ST_DONE;
                end else if (ack_timeout_cnt >= ACK_TIMEOUT_CNT) begin
                    // 超时，发送失败
                    tx_state <= TX_ST_FAIL;
                end else begin
                    ack_timeout_cnt <= ack_timeout_cnt + 1'b1;
                end
            end
            
            TX_ST_DONE: begin
                tx_done_reg <= 1'b1;
                tx_state <= TX_ST_IDLE;
            end
            
            TX_ST_FAIL: begin
                tx_fail_reg <= 1'b1;
                tx_state <= TX_ST_IDLE;
            end
            
            default: tx_state <= TX_ST_IDLE;
        endcase
    end
end

assign tx_done_o = tx_done_reg;
assign tx_fail_o = tx_fail_reg;
assign tx_busy_o = (tx_state != TX_ST_IDLE) || int_tx_busy;

//============================================================================
// RX 握手状态机
// 实现：接收消息后自动发送 GoodCRC 确认
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= RX_ST_IDLE;
        rx_done_reg <= 1'b0;
        rx_header_reg <= 16'd0;
        rx_data_flat_reg <= 240'd0;
        rx_data_len_reg <= 5'd0;
        rx_crc_ok_reg <= 1'b0;
        ack_header <= 16'd0;
        ack_data_flat <= 240'd0;
        ack_data_len <= 5'd0;
        ack_tx_start <= 1'b0;
    end else begin
        rx_done_reg <= 1'b0;
        // 注意：ack_tx_start 不在此处清除，由各状态单独管理
        
        case (rx_state)
            RX_ST_IDLE: begin
                ack_tx_start <= 1'b0;  // 确保 start 信号为低
                if (rx_en_i && int_rx_busy) begin
                    rx_state <= RX_ST_RECEIVING;
                end
            end
            
            RX_ST_RECEIVING: begin
                ack_tx_start <= 1'b0;  // 确保 start 信号为低
                if (raw_rx_done) begin
                    // 保存接收到的消息
                    rx_header_reg <= rx_header_o;
                    rx_data_flat_reg <= rx_data_flat_o;
                    rx_data_len_reg <= rx_data_len_o;
                    rx_crc_ok_reg <= rx_crc_ok_o;
                    $display("[Transceiver] raw_rx_done=1, rx_crc_ok_o=%b, rx_crc_ok_reg=%b", 
                             rx_crc_ok_o, rx_crc_ok_reg);
                    
                    if (rx_crc_ok_o && auto_goodcrc_i && !rx_is_goodcrc) begin
                        // CRC 正确且不是 GoodCRC 消息，需要发送 ACK
                        // 构建 GoodCRC Header
                        // [11:9]=MsgID, [8]=PortRole, [7:6]=SpecRev, [5]=DataRole, [4:0]=MsgTYPE
                        ack_header <= GOODCRC_HEADER_BASE |           // Type=GoodCRC
                                      (header_good_crc[0] << 8) |   // [9:8]=PowerRole from header_good_crc[0]
                                      (rx_header_o[7:6] << 6) |       // [7:6]=SpecRev from received header
                                      (header_good_crc[1] << 5) |     // [5]= DataRole from header_good_crc[1]
                                      (rx_header_o[11:9] << 9);             // [11:9]=MsgID from received header
                        ack_data_flat <= 240'd0;
                        ack_data_len <= 5'd0;
                        goodcrc_delay_cnt <= 13'd0;  // 初始化延时计数器
                        rx_state <= RX_ST_DELAY_ACK;  // 先进入延时状态
                    end else begin
                        // 不需要发送 ACK (CRC 错误或本身就是 GoodCRC)
                        rx_done_reg <= 1'b1;
                        rx_state <= RX_ST_DONE;
                    end
                end
            end
            
            RX_ST_DELAY_ACK: begin
                // 延时等待，满足 USB PD 规范中的回复时间要求
                ack_tx_start <= 1'b0;
                if (goodcrc_delay_cnt == 0) begin
                    $display("[RX State] Entering DELAY_ACK, waiting %0d cycles", GOODCRC_DELAY_CNT);
                end
                if (goodcrc_delay_cnt >= GOODCRC_DELAY_CNT) begin
                    // 延时完成，进入发送状态
                    $display("[RX State] DELAY_ACK done, entering SEND_ACK");
                    $display("[GoodCRC] Building ACK with MsgID=%0d", rx_header_reg[4:0]);
                    rx_state <= RX_ST_SEND_ACK;
                end else begin
                    goodcrc_delay_cnt <= goodcrc_delay_cnt + 1'b1;
                end
            end
            
            RX_ST_SEND_ACK: begin
                // 启动 GoodCRC 发送，保持 start 信号直到 TX 开始工作
                ack_tx_start <= 1'b1;
                $display("[RX State] SEND_ACK: ack_tx_start=1, int_tx_busy=%b", int_tx_busy);
                // 等待 TX 模块开始工作
                if (int_tx_busy) begin
                    // ACK 发送已启动，清除 start 信号并转到等待完成状态
                    $display("[GoodCRC] ACK transmission started!");
                    ack_tx_start <= 1'b0;
                    rx_state <= RX_ST_DONE;
                    rx_done_reg <= 1'b1;
                end
            end
            
            RX_ST_DONE: begin
                ack_tx_start <= 1'b0;  // 确保 start 信号为低
                goodcrc_delay_cnt <= 13'd0;  // 清除延时计数器
                // 等待状态机外部清除条件
                if (!rx_en_i || !int_rx_busy) begin
                    rx_state <= RX_ST_IDLE;
                end
            end
            
            default: rx_state <= RX_ST_IDLE;
        endcase
    end
end

assign rx_done_o = rx_done_reg;
assign rx_busy_o = (rx_state != RX_ST_IDLE) || int_rx_busy;

//============================================================================
// TX 模块实例化
//============================================================================
pd_bmc_tx u_pd_bmc_tx (
    .clk            (clk),
    .rst_n          (rst_n),
    .tx_header_i    (final_tx_header),
    .tx_data_flat_i (final_tx_data_flat),
    .tx_data_len_i  (final_tx_data_len),
    .tx_start_i     (final_tx_start),
    .tx_done_o      (raw_tx_done),
    .tx_busy_o      (int_tx_busy),
    .bmc_tx_pad     (bmc_tx_out),
    .tx_active_o    (tx_active_o)
);

// ACK TX 状态指示 (与主 TX 共享硬件，这里仅用于状态跟踪)
assign ack_tx_busy = int_tx_busy && (rx_state == RX_ST_SEND_ACK);
assign ack_tx_done = raw_tx_done && (rx_state == RX_ST_SEND_ACK);

//============================================================================
// RX 模块实例化
// RX 直接从 BMC 线读取（与 TX 输出是同一条线）
//============================================================================
pd_bmc_rx u_pd_bmc_rx (
    .clk            (clk),
    .rst_n          (rst_n),
    .rx_header_o    (rx_header_o),
    .rx_data_flat_o (rx_data_flat_o),
    .rx_data_len_o  (rx_data_len_o),
    .rx_done_o      (raw_rx_done),
    .rx_busy_o      (int_rx_busy),
    .rx_error_o     (rx_error_o),
    .rx_crc_ok_o    (rx_crc_ok_o),
    .rx_msg_id_o    (),
    .bmc_rx_pad     (bmc_pad),        // RX 从 BMC 线读取
    .rx_en_i        (rx_actual_en)
);

endmodule
