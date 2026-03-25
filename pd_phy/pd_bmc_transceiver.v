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
    output wire         tx_success_o,     // 发送成功（收到正确 GoodCRC）
    output wire         tx_fail_o,        // 发送失败（超时未收到 GoodCRC）
    
    // RX 接口
    output wire [15:0]  rx_header_o,
    output wire [239:0] rx_data_flat_o,
    output wire [4:0]   rx_data_len_o,
    output wire         rx_success_o,     // 接收成功（回复 GoodCRC 完成）
    input  wire         rx_en_i,          // RX 使能控制
    
    // 配置接口
    input  wire [7:0]   header_good_crc,  // bit0 -> power role; bit1 -> data_role; bit2 -> cable_role;
    
    // BMC 物理接口 (半双工，分离输入输出)
    input  wire         bmc_rx_pad,      // BMC 接收输入
    output wire         bmc_tx_pad,      // BMC 发送输出
    output wire         bmc_tx_en,       // 发送使能 (用于外部三态控制)
    
    // RX 调试接口
    output wire [2:0]   dbg_rx_state_o,     // RX 状态机 (transceiver 层)
    output wire         dbg_toggle_o        // 每收到有效数据位翻转
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
reg tx_success_reg;          // 发送成功标志
reg tx_fail_reg;
reg goodcrc_received_reg;    // 标记是否已收到 GoodCRC

// RX 握手状态机
localparam RX_ST_IDLE           = 3'd0;
localparam RX_ST_RECEIVING      = 3'd1;
localparam RX_ST_DELAY_ACK      = 3'd2;  // 延时等待状态
localparam RX_ST_SEND_ACK       = 3'd3;  // 启动 GoodCRC 发送
localparam RX_ST_WAIT_ACK_DONE  = 3'd4;  // 等待 GoodCRC 发送完成
localparam RX_ST_DONE           = 3'd5;

reg [2:0] rx_state;
reg rx_success_reg;           // 接收成功标志
reg [12:0] goodcrc_delay_cnt; // GoodCRC 发送延时计数器

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

// RX 使能的条件：外部使能 && TX 未忙
wire rx_actual_en = rx_en_i && !int_tx_busy && !ack_tx_busy;

//============================================================================
// 半双工逻辑：TX/RX 分离输入输出
//============================================================================
assign bmc_tx_pad = bmc_tx_out;
assign bmc_tx_en = int_tx_busy;

//============================================================================
// 检测接收到的消息是否为 GoodCRC (MessageType = 0x01, 在 bit[4:0])
//============================================================================
assign rx_is_goodcrc = (rx_header_o[4:0] == 5'b00001);  // GoodCRC MessageType = 0x01
wire rx_is_goodcrc_reg = (rx_header_reg[4:0] == 5'b00001);  // 使用保存的 header 判断

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
        tx_success_reg <= 1'b0;
        tx_fail_reg <= 1'b0;
        goodcrc_received_reg <= 1'b0;
    end else begin
        tx_success_reg <= 1'b0;
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
                    tx_state <= TX_ST_WAIT_ACK;
                    ack_timeout_cnt <= 22'd0;
                end
            end
            
            TX_ST_WAIT_ACK: begin
                if (goodcrc_received_reg || (rx_state == RX_ST_DONE && rx_is_goodcrc_reg)) begin
                    tx_state <= TX_ST_DONE;
                end else if (ack_timeout_cnt >= ACK_TIMEOUT_CNT) begin
                    tx_state <= TX_ST_FAIL;
                end else begin
                    ack_timeout_cnt <= ack_timeout_cnt + 1'b1;
                end
            end
            
            TX_ST_DONE: begin
                tx_success_reg <= 1'b1;
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

assign tx_success_o = tx_success_reg;
assign tx_fail_o = tx_fail_reg;

//============================================================================
// RX 握手状态机
// 实现：接收消息后自动发送 GoodCRC 确认
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= RX_ST_IDLE;
        rx_success_reg <= 1'b0;
        rx_header_reg <= 16'd0;
        rx_data_flat_reg <= 240'd0;
        rx_data_len_reg <= 5'd0;
        rx_crc_ok_reg <= 1'b0;
        ack_header <= 16'd0;
        ack_data_flat <= 240'd0;
        ack_data_len <= 5'd0;
        ack_tx_start <= 1'b0;
        goodcrc_delay_cnt <= 13'd0;
    end else begin
        rx_success_reg <= 1'b0;
        // 注意：ack_tx_start 不在此处清除，由各状态单独管理
        
        case (rx_state)
            RX_ST_IDLE: begin
                ack_tx_start <= 1'b0;
                if (int_rx_busy) begin
                    rx_state <= RX_ST_RECEIVING;
                end
            end
            
            RX_ST_RECEIVING: begin
                ack_tx_start <= 1'b0;
                if (raw_rx_done) begin
                    rx_header_reg <= rx_header_o;
                    rx_data_flat_reg <= rx_data_flat_o;
                    rx_data_len_reg <= rx_data_len_o;
                    rx_crc_ok_reg <= rx_crc_ok_internal;
                    if (rx_crc_ok_internal && !rx_is_goodcrc) begin
                        // CRC 正确且不是 GoodCRC 消息，需要发送 ACK
                        // USB PD Header 格式:
                        // [15]:Extended, [14:12]:NumDataObj, [11:9]:MsgID, [8]:PowerRole
                        // [7:6]:SpecRev, [5]:DataRole, [4:0]:MessageType
                        ack_header <= {4'b0,                        // [15:12] = 0
                                       rx_header_o[11:9],           // [11:9] = MsgID
                                       header_good_crc[0],          // [8] = PowerRole
                                       rx_header_o[7:6],            // [7:6] = SpecRev
                                       header_good_crc[1],          // [5] = DataRole
                                       5'b00001};                   // [4:0] = GoodCRC (0x01)
                        ack_data_flat <= 240'd0;
                        ack_data_len <= 5'd0;
                        goodcrc_delay_cnt <= 13'd0;  // 初始化延时计数器
                        rx_state <= RX_ST_DELAY_ACK;  // 先进入延时状态
                    end else begin
                        // 不需要发送 ACK (CRC 错误或本身就是 GoodCRC)
                        // 进入 RX_ST_DONE 状态，让 TX 状态机可以检测到 GoodCRC
                        // 注意：此时 pd_bmc_rx 可能还在 ST_DONE 状态，需要等待它回到 ST_IDLE
                        rx_state <= RX_ST_DONE;
                    end
                end
            end
            
            RX_ST_DELAY_ACK: begin
                // 延时等待，满足 USB PD 规范中的回复时间要求
                ack_tx_start <= 1'b0;
                if (goodcrc_delay_cnt >= GOODCRC_DELAY_CNT) begin
                    // 延时完成，进入发送状态
                    rx_state <= RX_ST_SEND_ACK;
                end else begin
                    goodcrc_delay_cnt <= goodcrc_delay_cnt + 1'b1;
                end
            end
            
            RX_ST_SEND_ACK: begin
                ack_tx_start <= 1'b1;
                // 等待 TX 模块开始工作
                if (int_tx_busy) begin
                    // ACK 发送已启动，清除 start 信号并转到等待完成状态
                    ack_tx_start <= 1'b0;
                    rx_state <= RX_ST_WAIT_ACK_DONE;
                end
            end
            
            RX_ST_WAIT_ACK_DONE: begin
                // 等待 GoodCRC 发送完成
                ack_tx_start <= 1'b0;
                if (ack_tx_done) begin
                    // GoodCRC 发送完成，报 rx_success
                    rx_success_reg <= 1'b1;
                    rx_state <= RX_ST_DONE;
                end
            end
            
            RX_ST_DONE: begin
                ack_tx_start <= 1'b0;
                goodcrc_delay_cnt <= 13'd0;
                // 等待 TX 和 RX 都空闲后回到 IDLE
                if (!int_tx_busy && !int_rx_busy) begin
                    rx_state <= RX_ST_IDLE;
                end
            end
            
            default: rx_state <= RX_ST_IDLE;
        endcase
    end
end

assign rx_success_o = rx_success_reg;

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
    .tx_active_o    ()             // 不导出
);

// ACK TX 状态指示 (与主 TX 共享硬件，这里仅用于状态跟踪)
assign ack_tx_busy = int_tx_busy && (rx_state == RX_ST_SEND_ACK);
assign ack_tx_done = raw_tx_done && (rx_state == RX_ST_WAIT_ACK_DONE);

//============================================================================
// RX 模块实例化
// RX 直接从 BMC 线读取（与 TX 输出是同一条线）
//============================================================================
// RX 内部信号
wire rx_crc_ok_internal;
wire rx_error_internal;

pd_bmc_rx u_pd_bmc_rx (
    .clk            (clk),
    .rst_n          (rst_n),
    .rx_header_o    (rx_header_o),
    .rx_data_flat_o (rx_data_flat_o),
    .rx_data_len_o  (rx_data_len_o),
    .rx_done_o      (raw_rx_done),
    .rx_busy_o      (int_rx_busy),
    .rx_error_o     (rx_error_internal),
    .rx_crc_ok_o    (rx_crc_ok_internal),
    .rx_msg_id_o    (),
    .bmc_rx_pad     (bmc_rx_pad),
    .rx_en_i        (rx_actual_en),
    .dbg_en_i       (1'b0),
    .dbg_byte_o     (),
    .dbg_byte_valid_o (),
    .dbg_toggle_o       (dbg_toggle_o)
);

// Transceiver 层 RX 状态调试
assign dbg_rx_state_o = rx_state;

endmodule
