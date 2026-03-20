//============================================================================
// USB PD BMC 发送模块 (含EOP后低电平保持)
// 支持完整PD包：前导码 → SOP → 2字节Header → 最多30字节数据 → 32bit CRC → EOP → 低电平保持 → 空闲
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tx (
    input  wire               clk,
    input  wire               rst_n,
    
    // 完整PD包配置接口
    input  wire [15:0]        tx_header_i,     // 2字节 Message Header
    input  wire [7:0]         tx_data_i [0:29],// 最多30字节数据载荷
    input  wire [4:0]         tx_data_len_i,   // 实际发送的数据字节数(0-30)
    input  wire               tx_start_i,       // 包发送启动信号
    output reg                tx_done_o,        // 包发送完成标志
    output reg                tx_busy_o,        // 发送忙标志
    
    // 底层物理输出
    output reg                bmc_tx_pad,       // BMC编码最终输出
    output reg                tx_active_o       // 发送进行中标志
);

//============================================================================
// 参数定义
//============================================================================
parameter BIT_PERIOD_CNT  = 166;    // 50MHz 对应 300kbps 位周期
parameter HALF_BIT_CNT    = 83;
parameter PREAMBLE_LEN    = 64;     // 前导码长度 64bit
parameter MAX_DATA_LEN    = 30;     // 最大数据字节数 30
parameter LOW_PERIOD_CNT  = 200;    // 【新增】EOP后低电平保持的时钟周期数 (可调整)

// 4B5B K码定义 (完全匹配PD规范)
localparam KCODE_SYNC1  = 5'b11000;
localparam KCODE_SYNC2  = 5'b10001;
localparam KCODE_EOP    = 5'b01101;

// 发送状态机定义 (新增低电平保持状态)
localparam ST_IDLE      = 4'd0;  // 空闲
localparam ST_PREAMBLE  = 4'd1;  // 发送前导码
localparam ST_SOP       = 4'd2;  // 发送SOP有序集
localparam ST_HEADER    = 4'd3;  // 发送2字节Header
localparam ST_DATA      = 4'd4;  // 发送数据载荷
localparam ST_CRC       = 4'd5;  // 发送32bit CRC
localparam ST_EOP       = 4'd6;  // 发送EOP结束符
localparam ST_LOW       = 4'd7;  // 【新增】EOP后低电平保持
localparam ST_DONE      = 4'd8;  // 发送完成

//============================================================================
// 内部信号
//============================================================================
// 位定时相关
reg [15:0] bit_timer;
wire       bit_tick = (bit_timer == BIT_PERIOD_CNT);

// 前导码相关
reg [5:0]  preamble_cnt;

// 4B5B符号发送相关
reg [4:0]  current_symbol;    // 当前正在发送的5bit 4B5B符号
reg [2:0]  symbol_bit_cnt;    // 符号内bit计数器(0-4)
wire       tx_bit;            // 当前待发送的bit

// 状态机相关
reg [3:0]  tx_state;

// 字节发送相关
reg [7:0]  current_byte;      // 当前待编码的字节
reg        byte_nibble_sel;   // 字节半字节选择：0=低4bit，1=高4bit
reg [4:0]  byte_cnt;          // 字节计数器
reg [1:0]  sop_symbol_cnt;    // SOP符号计数器(0-3)
reg [1:0]  crc_byte_cnt;      // CRC字节计数器(0-3，4字节足够)

// 【新增】低电平保持计数器
reg [15:0] low_period_cnt;

// CRC32计算相关 (PD规范CRC32，多项式0x04C11DB7)
reg [31:0] crc_reg;
reg [31:0] final_crc_reg;     // 锁存最终CRC结果，避免计算污染
wire [31:0] crc_next;

// 发送缓存
reg [15:0] tx_header_reg;
reg [7:0]  tx_data_reg [0:29];
reg [4:0]  tx_data_len_reg;

// 调试用寄存器
reg [7:0]  tx_data_byte0;
reg [7:0]  tx_data_byte1;

//============================================================================
// 4B5B 编码函数 (数据半字节 → 5bit符号)
//============================================================================
function [4:0] encode_4b5b_data;
    input [3:0] data_4b;
    begin
        case (data_4b)
            4'b0000: encode_4b5b_data = 5'b11110;
            4'b0001: encode_4b5b_data = 5'b01001;
            4'b0010: encode_4b5b_data = 5'b10100;
            4'b0011: encode_4b5b_data = 5'b10101;
            4'b0100: encode_4b5b_data = 5'b01010;
            4'b0101: encode_4b5b_data = 5'b01011;
            4'b0110: encode_4b5b_data = 5'b01110;
            4'b0111: encode_4b5b_data = 5'b01111;
            4'b1000: encode_4b5b_data = 5'b10010;
            4'b1001: encode_4b5b_data = 5'b10011;
            4'b1010: encode_4b5b_data = 5'b10110;
            4'b1011: encode_4b5b_data = 5'b10111;
            4'b1100: encode_4b5b_data = 5'b11010;
            4'b1101: encode_4b5b_data = 5'b11011;
            4'b1110: encode_4b5b_data = 5'b11100;
            4'b1111: encode_4b5b_data = 5'b11101;
            default: encode_4b5b_data = 5'b00000;
        endcase
    end
endfunction

//============================================================================
// CRC32 计算逻辑 (USB PD规范)
// 多项式：0x04C11DB7，初始值0xFFFFFFFF，输入字节LSB先行
// 计算范围：仅Header + 数据载荷，CRC本身不参与计算
//============================================================================
function [31:0] calc_crc32;
    input [31:0] crc_in;
    input [7:0]  data_in;
    integer i;
    begin
        calc_crc32 = crc_in;
        for (i = 0; i < 8; i = i + 1) begin
            if ((calc_crc32[0] ^ data_in[i]) == 1'b1) begin
                calc_crc32 = (calc_crc32 >> 1) ^ 32'hEDB88320; // 反转多项式
            end else begin
                calc_crc32 = calc_crc32 >> 1;
            end
        end
    end
endfunction

// CRC组合逻辑：仅在Header和Data阶段有效，CRC阶段不使用
assign crc_next = calc_crc32(crc_reg, current_byte);

//============================================================================
// 位定时器 (完全沿用验证过的逻辑)
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_timer <= 16'd0;
    end else if (tx_state == ST_IDLE) begin
        bit_timer <= 16'd0;
    end else if (bit_tick) begin
        bit_timer <= 16'd0;
    end else begin
        bit_timer <= bit_timer + 1'b1;
    end
end

//============================================================================
// 发送状态机时序逻辑 (新增EOP后低电平保持)
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state <= ST_IDLE;
        tx_header_reg <= 16'd0;
        tx_data_len_reg <= 5'd0;
        tx_busy_o <= 1'b0;
        tx_done_o <= 1'b0;
        tx_active_o <= 1'b0;
        preamble_cnt <= 6'd0;
        current_symbol <= 5'd0;
        symbol_bit_cnt <= 3'd0;
        byte_cnt <= 5'd0;
        sop_symbol_cnt <= 2'd0;
        byte_nibble_sel <= 1'b0;
        crc_byte_cnt <= 2'd0;
        low_period_cnt <= 16'd0; // 【新增】初始化低电平计数器
        crc_reg <= 32'hFFFFFFFF;
        final_crc_reg <= 32'h0;
        // 初始化数据缓存
        for (integer i = 0; i < 30; i = i + 1) begin
            tx_data_reg[i] <= 8'd0;
        end
        tx_data_byte0 <= 8'd0;
        tx_data_byte1 <= 8'd0;
    end else begin
        tx_done_o <= 1'b0; // 完成脉冲默认清零
        
        case (tx_state)
            // 空闲状态：等待发送启动
            ST_IDLE: begin
                tx_busy_o <= 1'b0;
                tx_active_o <= 1'b0;
                low_period_cnt <= 16'd0;
                if (tx_start_i) begin
                    tx_busy_o <= 1'b1;
                    tx_active_o <= 1'b1;
                    // 锁存输入的包数据
                    tx_header_reg <= tx_header_i;
                    tx_data_len_reg <= tx_data_len_i;
                    for (integer i = 0; i < 30; i = i + 1) begin
                        tx_data_reg[i] <= tx_data_i[i];
                    end
                    // 调试用镜像寄存器
                    tx_data_byte0 <= tx_data_i[0];
                    tx_data_byte1 <= tx_data_i[1];
                    // 初始化计数器
                    preamble_cnt <= 6'd0;
                    symbol_bit_cnt <= 3'd0;
                    byte_cnt <= 5'd0;
                    sop_symbol_cnt <= 2'd0;
                    byte_nibble_sel <= 1'b0;
                    crc_byte_cnt <= 2'd0;
                    crc_reg <= 32'hFFFFFFFF; // CRC初始值
                    final_crc_reg <= 32'h0;
                    // 进入前导码阶段
                    tx_state <= ST_PREAMBLE;
                end
            end

            // 阶段1：发送64bit前导码
            ST_PREAMBLE: begin
                if (bit_tick) begin
                    preamble_cnt <= preamble_cnt + 1'b1;
                    if (preamble_cnt == PREAMBLE_LEN - 1) begin
                        // 前导码发送完成，进入SOP阶段，加载第一个Sync-1符号
                        current_symbol <= KCODE_SYNC1;
                        tx_state <= ST_SOP;
                    end
                end
            end

            // 阶段2：发送SOP有序集 (3个Sync-1 + 1个Sync-2，匹配PD规范)
            ST_SOP: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，LSB先行
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    // 当前5bit符号发送完成
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        sop_symbol_cnt <= sop_symbol_cnt + 1'b1;
                        
                        case (sop_symbol_cnt)
                            2'd0: current_symbol <= KCODE_SYNC1; // 第2个Sync-1
                            2'd1: current_symbol <= KCODE_SYNC1; // 第3个Sync-1
                            2'd2: current_symbol <= KCODE_SYNC2; // Sync-2
                            2'd3: begin
                                // SOP发送完成，进入Header阶段，加载Header低字节的低4bit
                                current_byte <= tx_header_reg[7:0];
                                current_symbol <= encode_4b5b_data(tx_header_reg[3:0]);
                                byte_nibble_sel <= 1'b0;
                                byte_cnt <= 5'd0;
                                tx_state <= ST_HEADER;
                            end
                        endcase
                    end
                end
            end

            // 阶段3：发送2字节Message Header (参与CRC计算)
            ST_HEADER: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，LSB先行
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        byte_nibble_sel <= ~byte_nibble_sel;
                        
                        // 低4bit符号发送完成，发送高4bit
                        if (!byte_nibble_sel) begin
                            current_symbol <= encode_4b5b_data(current_byte[7:4]);
                        end
                        // 高4bit符号发送完成，当前字节发送完毕
                        else begin
                            // 【核心】当前字节参与CRC计算，更新CRC寄存器
                            crc_reg <= crc_next;
                            byte_cnt <= byte_cnt + 1'b1;
                            
                            // 2字节Header发送完成
                            if (byte_cnt == 5'd1) begin
                                // 有数据载荷 → 进入数据阶段
                                if (tx_data_len_reg > 5'd0) begin
                                    current_byte <= tx_data_reg[0];
                                    current_symbol <= encode_4b5b_data(tx_data_reg[0][3:0]);
                                    byte_nibble_sel <= 1'b0;
                                    byte_cnt <= 5'd0;
                                    tx_state <= ST_DATA;
                                end
                                // 无数据载荷 → 锁存最终CRC，进入CRC阶段
                                else begin
                                    final_crc_reg <= ~crc_next; // 最终CRC结果取反，锁存
                                    current_byte <= ~crc_next[7:0]; // 第一个CRC字节：最低字节
                                    current_symbol <= encode_4b5b_data(~crc_next[3:0]);
                                    byte_nibble_sel <= 1'b0;
                                    crc_byte_cnt <= 2'd0;
                                    tx_state <= ST_CRC;
                                end
                            end
                            // 继续发送Header高字节
                            else begin
                                current_byte <= tx_header_reg[15:8];
                                current_symbol <= encode_4b5b_data(tx_header_reg[11:8]);
                            end
                        end
                    end
                end
            end

            // 阶段4：发送数据载荷 (参与CRC计算)
            ST_DATA: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，LSB先行
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        byte_nibble_sel <= ~byte_nibble_sel;
                        
                        if (!byte_nibble_sel) begin
                            current_symbol <= encode_4b5b_data(current_byte[7:4]);
                        end
                        else begin
                            // 【核心】当前数据字节参与CRC计算，更新CRC寄存器
                            crc_reg <= crc_next;
                            byte_cnt <= byte_cnt + 1'b1;
                            
                            // 所有数据发送完成
                            if (byte_cnt == tx_data_len_reg - 1) begin
                                // 【核心】锁存最终CRC结果，CRC本身不再参与计算
                                final_crc_reg <= ~crc_next; // 最终CRC结果按位取反，锁存
                                // 加载第一个CRC字节：最低字节[7:0]
                                current_byte <= ~crc_next[7:0];
                                current_symbol <= encode_4b5b_data(~crc_next[3:0]);
                                byte_nibble_sel <= 1'b0;
                                crc_byte_cnt <= 2'd0;
                                tx_state <= ST_CRC;
                            end
                            // 继续发送下一个字节
                            else begin
                                current_byte <= tx_data_reg[byte_cnt + 1];
                                current_symbol <= encode_4b5b_data(tx_data_reg[byte_cnt + 1][3:0]);
                            end
                        end
                    end
                end
            end

            // 阶段5：发送32bit CRC (CRC本身不参与CRC计算)
            ST_CRC: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，LSB先行
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        byte_nibble_sel <= ~byte_nibble_sel;
                        
                        if (!byte_nibble_sel) begin
                            // 高4bit编码，使用锁存的final_crc_reg，不更新CRC
                            current_symbol <= encode_4b5b_data(current_byte[7:4]);
                        end
                        else begin
                            crc_byte_cnt <= crc_byte_cnt + 1'b1;
                            
                            // 4字节CRC发送完成
                            if (crc_byte_cnt == 2'd3) begin
                                // 进入EOP阶段，加载EOP K码
                                current_symbol <= KCODE_EOP;
                                tx_state <= ST_EOP;
                            end
                            // 继续发送下一个CRC字节，使用锁存的final_crc_reg
                            else begin
                                case (crc_byte_cnt)
                                    2'd0: begin
                                        current_byte <= final_crc_reg[15:8];  // 第2个字节
                                        current_symbol <= encode_4b5b_data(final_crc_reg[11:8]);
                                    end
                                    2'd1: begin
                                        current_byte <= final_crc_reg[23:16]; // 第3个字节
                                        current_symbol <= encode_4b5b_data(final_crc_reg[19:16]);
                                    end
                                    2'd2: begin
                                        current_byte <= final_crc_reg[31:24]; // 第4个字节
                                        current_symbol <= encode_4b5b_data(final_crc_reg[27:24]);
                                    end
                                endcase
                            end
                        end
                    end
                end
            end

            // 阶段6：发送EOP结束符
            ST_EOP: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，LSB先行
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        // EOP发送完成，【修改】进入低电平保持阶段，而不是直接Done
                        low_period_cnt <= 16'd0;
                        tx_state <= ST_LOW;
                    end
                end
            end

            // 【新增】阶段7：EOP后低电平保持
            ST_LOW: begin
                low_period_cnt <= low_period_cnt + 1'b1;
                
                // 低电平保持时间到
                if (low_period_cnt == LOW_PERIOD_CNT - 1) begin
                    tx_state <= ST_DONE;
                end
            end

            // 阶段8：发送完成
            ST_DONE: begin
                tx_done_o <= 1'b1;
                tx_busy_o <= 1'b0;
                tx_active_o <= 1'b0;
                tx_state <= ST_IDLE;
            end

            default: tx_state <= ST_IDLE;
        endcase
    end
end

//============================================================================
// 发送bit选择 (4B5B符号取最低位，LSB先行)
//============================================================================
assign tx_bit = (tx_state == ST_PREAMBLE) ? preamble_cnt[0] : current_symbol[0];

//============================================================================
// BMC 编码逻辑 (【新增】在ST_LOW阶段强制输出低电平)
//============================================================================
reg bmc_state;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bmc_state  <= 1'b1;
        bmc_tx_pad <= 1'b1;
    end else if (tx_state == ST_IDLE) begin
        bmc_tx_pad <= 1'b1; // 空闲状态为高(J-state)
    end else if (tx_state == ST_LOW) begin
        // 【新增】低电平保持阶段：强制输出低电平
        bmc_state  <= 1'b0;
        bmc_tx_pad <= 1'b0;
    end else begin
        // 正常发送阶段
        // 比特周期开始：固定跳变
        if (bit_timer == 16'd0) begin
            bmc_state <= ~bmc_state;
        end
        // 比特周期中间：数据为1时额外跳变
        if (bit_timer == HALF_BIT_CNT) begin
            if (tx_bit) begin
                bmc_state <= ~bmc_state;
            end
        end
        bmc_tx_pad <= bmc_state;
    end
end

endmodule