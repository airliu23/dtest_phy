//============================================================================
// USB PD BMC 接收模块 (基于边沿检测)
// 与 pd_bmc_tx.v 配对使用，采用边沿间隔解码
//============================================================================

`timescale 1ns/1ps

module pd_bmc_rx (
    input  wire               clk,
    input  wire               rst_n,
    
    // 接收使能
    input  wire               rx_en_i,
    
    // 接收完成包接口
    output reg  [15:0]        rx_header_o,
    output wire [239:0]       rx_data_flat_o,
    output reg  [4:0]         rx_data_len_o,
    output reg                rx_done_o,
    output reg                rx_busy_o,
    output reg                rx_error_o,
    output reg                rx_crc_ok_o,     // CRC 验证结果
    output reg  [2:0]         rx_msg_id_o,     // 接收到的 MessageID
    
    // 底层物理输入
    input  wire               bmc_rx_pad,
    
    // 调试接口 - 实时输出解析的byte
    input  wire               dbg_en_i,        // 调试使能
    output reg  [7:0]         dbg_byte_o,      // 当前解析的字节
    output reg                dbg_byte_valid_o // 字节有效标志
);

//============================================================================
// 参数定义 (与 TX 完全一致)
//============================================================================
parameter BIT_PERIOD_CNT  = 166;
parameter HALF_BIT_CNT    = 83;
parameter PREAMBLE_LEN    = 64;
parameter MAX_DATA_LEN    = 30;

// 4B5B K码定义
localparam KCODE_SYNC1  = 5'b11000;
localparam KCODE_SYNC2  = 5'b10001;
localparam KCODE_EOP    = 5'b01101;

// 接收状态机 (极简版)
localparam ST_IDLE      = 4'd0;
localparam ST_PREAMBLE  = 4'd1;
localparam ST_RECEIVE   = 4'd2;  // SOP + Header + Data + CRC），直到EOP
localparam ST_DONE      = 4'd3;

//============================================================================
// 边沿检测参数
//============================================================================
localparam HALF_BIT_MIN = HALF_BIT_CNT - 15;
localparam HALF_BIT_MAX = HALF_BIT_CNT + 15;
localparam FULL_BIT_MIN = BIT_PERIOD_CNT - 20;
localparam FULL_BIT_MAX = BIT_PERIOD_CNT + 20;

//============================================================================
// 输入同步与边沿检测
//============================================================================
reg [2:0]  pad_sync;
reg        prev_bmc_val;
wire       bmc_rx_synced;
wire       any_edge;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pad_sync <= 3'b111;
        prev_bmc_val <= 1'b1;
    end else begin
        pad_sync <= {pad_sync[1:0], bmc_rx_pad};
        prev_bmc_val <= bmc_rx_synced;
    end
end

assign bmc_rx_synced = pad_sync[2];
assign any_edge = (bmc_rx_synced != prev_bmc_val);

//============================================================================
// BMC 解码逻辑 (核心：基于边沿间隔)
// 原理：
//   - 检测到一个边沿后，测量到下一个边沿的时间间隔
//   - 间隔 ≈ 半周期 (83±15)：这是数据1的中间跳变，输出1，继续等待下一半周期
//   - 间隔 ≈ 全周期 (166±20)：这是数据0的周期跳变，输出0
//============================================================================

// BMC 解码状态
localparam BMC_IDLE     = 2'd0;  // 等待第一个边沿（周期开始）
localparam BMC_WAIT_1   = 2'd1;  // 等待第一个半周期（检测数据1的中间跳变）
localparam BMC_WAIT_0   = 2'd2;  // 等待第二个半周期（数据1的周期结束）

reg [1:0]  bmc_state;
reg [15:0] edge_timer;     // 边沿间隔计时器
reg        rx_bit;         // 解码后的数据bit
reg        rx_bit_valid;   // 数据有效标志

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bmc_state <= BMC_IDLE;
        edge_timer <= 16'd0;
        rx_bit <= 1'b0;
        rx_bit_valid <= 1'b0;
    end else if (rx_state == ST_IDLE || rx_state == ST_DONE) begin
        bmc_state <= BMC_IDLE;
        edge_timer <= 16'd0;
        rx_bit <= 1'b0;
        rx_bit_valid <= 1'b0;
    end else begin
        rx_bit_valid <= 1'b0;  // 默认无效
        
        case (bmc_state)
            BMC_IDLE: begin
                edge_timer <= 16'd0;
                if (any_edge) begin
                    // 检测到周期开始边沿
                    bmc_state <= BMC_WAIT_1;
                end
            end
            
            BMC_WAIT_1: begin
                edge_timer <= edge_timer + 1'b1;
                
                if (any_edge) begin
                    // 在半周期窗口内检测到边沿 → 数据=1
                    if (edge_timer >= HALF_BIT_MIN && edge_timer <= HALF_BIT_MAX) begin
                        rx_bit <= 1'b1;
                        rx_bit_valid <= 1'b1;
                        // 此边沿是数据1的中间跳变，同时也是下一周期的开始
                        // 直接进入 WAIT_0 等待周期结束
                        bmc_state <= BMC_WAIT_0;
                        edge_timer <= 16'd0;
                    end else begin
                        // 边沿时间不对，重置
                        bmc_state <= BMC_IDLE;
                    end
                end else if (edge_timer >= HALF_BIT_MAX) begin
                    // 半周期窗口结束，未检测到中间边沿 → 数据=0
                    rx_bit <= 1'b0;
                    rx_bit_valid <= 1'b1;
                    bmc_state <= BMC_IDLE;
                end
            end
            
            BMC_WAIT_0: begin
                edge_timer <= edge_timer + 1'b1;
                
                if (any_edge) begin
                    // 在半周期窗口内检测到边沿 → 这是下一周期的开始
                    if (edge_timer >= HALF_BIT_MIN && edge_timer <= HALF_BIT_MAX) begin
                        bmc_state <= BMC_WAIT_1;
                        edge_timer <= 16'd0;
                    end else begin
                        // 边沿时间不对，重置
                        bmc_state <= BMC_IDLE;
                    end
                end else if (edge_timer >= HALF_BIT_MAX) begin
                    // 超时，错误
                    bmc_state <= BMC_IDLE;
                end
            end
            
            default: bmc_state <= BMC_IDLE;
        endcase
    end
end

//============================================================================
// 接收状态机
//============================================================================
reg [3:0]  rx_state;

// 位计数器
reg [5:0]  preamble_cnt;
reg [2:0]  symbol_bit_cnt;
reg [4:0]  byte_cnt;
reg        byte_nibble_sel;

// 数据寄存器
reg [4:0]  current_symbol;
reg [7:0]  current_byte;
reg [15:0] rx_header_reg;
reg [31:0] crc_calc;         // 用于实时计算接收数据的 CRC
reg [31:0] rx_crc_received;  // 接收到的 CRC 值

// 【修复1】声明内部数据数组，用于暂存接收数据
reg [7:0]  rx_data_reg [0:29];

//============================================================================
// 数据输出映射
// rx_data_reg 布局: [0-1]:SOP, [2-3]:Header, [4...]:Data, [最后4字节]:CRC
// rx_data_flat_o 输出实际数据部分 (从 rx_data_reg[4] 开始)
//============================================================================
genvar k;
generate
    for (k = 0; k < 26; k = k + 1) begin : gen_rx_data_map
        assign rx_data_flat_o[k*8 +: 8] = rx_data_reg[k + 4];
    end
    // 填充剩余位为 0
    for (k = 26; k < 30; k = k + 1) begin : gen_rx_data_pad
        assign rx_data_flat_o[k*8 +: 8] = 8'd0;
    end
endgenerate

//============================================================================
// CRC计算
//============================================================================
function [31:0] calc_crc32;
    input [31:0] crc_in;
    input [7:0]  data_in;
    integer i;
    begin
        calc_crc32 = crc_in;
        for (i = 0; i < 8; i = i + 1) begin
            if ((calc_crc32[0] ^ data_in[i]) == 1'b1) begin
                calc_crc32 = (calc_crc32 >> 1) ^ 32'hEDB88320;
            end else begin
                calc_crc32 = calc_crc32 >> 1;
            end
        end
    end
endfunction

//============================================================================
// 4B5B解码
//============================================================================
function [3:0] decode_4b5b_data;
    input [4:0] symbol_5b;
    begin
        case (symbol_5b)
            5'b11110: decode_4b5b_data = 4'b0000;
            5'b01001: decode_4b5b_data = 4'b0001;
            5'b10100: decode_4b5b_data = 4'b0010;
            5'b10101: decode_4b5b_data = 4'b0011;
            5'b01010: decode_4b5b_data = 4'b0100;
            5'b01011: decode_4b5b_data = 4'b0101;
            5'b01110: decode_4b5b_data = 4'b0110;
            5'b01111: decode_4b5b_data = 4'b0111;
            5'b10010: decode_4b5b_data = 4'b1000;
            5'b10011: decode_4b5b_data = 4'b1001;
            5'b10110: decode_4b5b_data = 4'b1010;
            5'b10111: decode_4b5b_data = 4'b1011;
            5'b11010: decode_4b5b_data = 4'b1100;
            5'b11011: decode_4b5b_data = 4'b1101;
            5'b11100: decode_4b5b_data = 4'b1110;
            5'b11101: decode_4b5b_data = 4'b1111;
            default:  decode_4b5b_data = 4'b0000;
        endcase
    end
endfunction

integer i; // 循环变量声明移到 always 块内部

//============================================================================
// 主状态机
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= ST_IDLE;
        rx_done_o <= 1'b0;
        rx_busy_o <= 1'b0;
        rx_error_o <= 1'b0;
        rx_header_o <= 16'd0;
        rx_data_len_o <= 5'd0;
        preamble_cnt <= 6'd0;
        symbol_bit_cnt <= 3'd0;
        byte_cnt <= 5'd0;
        byte_nibble_sel <= 1'b0;
        current_symbol <= 5'd0;
        current_byte <= 8'd0;
        rx_header_reg <= 16'd0;
        
        // 复位内部数组 rx_data_reg
        for (i = 0; i < 30; i = i + 1) begin
            rx_data_reg[i] <= 8'd0;
        end
        
        // 复位调试信号
        dbg_byte_o <= 8'd0;
        dbg_byte_valid_o <= 1'b0;
    end else begin
        rx_done_o <= 1'b0;
        dbg_byte_valid_o <= 1'b0;  // 默认清除调试有效标志
        
        case (rx_state)
            ST_IDLE: begin
                rx_busy_o <= 1'b0;
                rx_error_o <= 1'b0;
                rx_crc_ok_o <= 1'b0;
                preamble_cnt <= 6'd0;
                symbol_bit_cnt <= 3'd0;
                crc_calc <= 32'hFFFFFFFF;
                
                if (rx_en_i && any_edge) begin
                    rx_busy_o <= 1'b1;
                    rx_state <= ST_PREAMBLE;
                end
            end
            
            ST_PREAMBLE: begin
                // 如果 RX 被禁用，返回 IDLE
                if (!rx_en_i) begin
                    rx_busy_o <= 1'b0;
                    rx_state <= ST_IDLE;
                end
                else if (rx_bit_valid) begin
                    preamble_cnt <= preamble_cnt + 1'b1;
                    
                    if (preamble_cnt == PREAMBLE_LEN - 1) begin
                        byte_cnt <= 5'd0;
                        byte_nibble_sel <= 1'b0;
                        symbol_bit_cnt <= 3'd0;
                        current_symbol <= 5'd0;
                        rx_state <= ST_RECEIVE;
                    end
                end
            end
            
            // 接收所有数据（SOP + Header + Data + CRC），直到检测到 EOP
            ST_RECEIVE: begin
                if (rx_bit_valid) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    current_symbol <= {rx_bit, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        // 检查是否是 EOP KCODE
                        if (current_symbol == KCODE_EOP) begin
                            // 数据结构: SOP(2字节) + Header(2字节) + Data(N字节) + CRC(4字节)
                            // byte_cnt = 2 + 2 + N + 4 = N + 8
                            // 实际数据长度 N = byte_cnt - 8
                            rx_data_len_o <= (byte_cnt > 8) ? (byte_cnt - 8) : 0;
                            rx_done_o <= 1'b1;
                            rx_busy_o <= 1'b0;
                            
                            if (byte_cnt >= 4) begin
                                // 提取 Header (SOP后的前2字节, 即 rx_data_reg[2-3])
                                rx_header_o <= {rx_data_reg[3], rx_data_reg[2]};
                                // rx_msg_id_o <= rx_data_reg[2][6:4];  // MessageID 在 Header 低字节 [14:12]
                                
                                rx_crc_received = {rx_data_reg[byte_cnt - 1],rx_data_reg[byte_cnt - 2],rx_data_reg[byte_cnt - 3],rx_data_reg[byte_cnt - 4]};
                                for (i = 2; i < byte_cnt - 4; i = i + 1) begin
                                    crc_calc = calc_crc32(crc_calc, rx_data_reg[i]);
                                end
                                crc_calc = ~crc_calc;
                                rx_crc_ok_o = (crc_calc == rx_crc_received);
                            end else begin
                                $display("[RX CRC Debug] byte_cnt=%0d < 8, skipping CRC check", byte_cnt);
                            end
                            rx_state <= ST_DONE;
                        end else begin
                            if (!byte_nibble_sel) begin
                                current_byte[3:0] = decode_4b5b_data(current_symbol);
                            end else begin
                                current_byte[7:4] = decode_4b5b_data(current_symbol);
                                // 保存到内部数组 rx_data_reg
                                rx_data_reg[byte_cnt] <= {decode_4b5b_data(current_symbol), current_byte[3:0]};
                                
                                // 调试输出 - 高字节时输出完整byte
                                if (dbg_en_i) begin
                                    dbg_byte_o <= {decode_4b5b_data(current_symbol), current_byte[3:0]};
                                    dbg_byte_valid_o <= 1'b1;
                                end
                                
                                byte_cnt <= byte_cnt + 1'b1;
                                
                                if (byte_cnt >= MAX_DATA_LEN - 1) begin
                                    rx_error_o <= 1'b1;
                                    rx_state <= ST_IDLE;
                                end
                            end
                            byte_nibble_sel <= ~byte_nibble_sel;
                        end
                    end
                end
            end
            
            ST_DONE: begin
                // 清除错误标志
                rx_error_o <= 1'b0;
                // 如果 rx_en 被禁用，返回 IDLE
                if (!rx_en_i) begin
                    rx_done_o <= 1'b0;
                    rx_state <= ST_IDLE;
                end
                // 保持 rx_done_o 置位，直到检测到新的边沿（下一包开始）
                else if (any_edge) begin
                    rx_done_o <= 1'b0;
                    rx_state <= ST_PREAMBLE;
                end
            end
            
            default: rx_state <= ST_IDLE;
        endcase
    end
end

endmodule