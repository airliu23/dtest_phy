//============================================================================
// USB PD BMC 发送模块
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tx (
    input  wire               clk,
    input  wire               rst_n,
    
    input  wire [15:0]        tx_header_i,
    input  wire [239:0]       tx_data_flat_i, // 30x8bit 展平为 240bit
    input  wire [4:0]         tx_data_len_i,
    input  wire               tx_start_i,
    output reg                tx_done_o,
    output reg                tx_busy_o,
    
    output reg                bmc_tx_pad,
    output reg                tx_active_o
);

//============================================================================
//============================================================================
integer i;

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
reg [31:0] final_crc_reg;
wire [31:0] crc_next;

reg [15:0] tx_header_reg;
// 添加内部数据数组
reg [7:0]  tx_data_reg [0:29];
reg [4:0]  tx_data_len_reg;

reg [7:0]  tx_data_byte0;
reg [7:0]  tx_data_byte1;

//============================================================================
// 4B5B 编码函数
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
    integer i; // function 内部允许，不用改
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

assign crc_next = calc_crc32(crc_reg, current_byte);

//============================================================================
// 位定时器
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
// 发送状态机时序逻辑
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
        low_period_cnt <= 16'd0;
        crc_reg <= 32'hFFFFFFFF;
        final_crc_reg <= 32'h0;
        
        // 复位内部数组
        for (i = 0; i < 30; i = i + 1) begin
            tx_data_reg[i] <= 8'd0;
        end
        
        tx_data_byte0 <= 8'd0;
        tx_data_byte1 <= 8'd0;
    end else begin
        tx_done_o <= 1'b0;
        
        case (tx_state)
            ST_IDLE: begin
                tx_busy_o <= 1'b0;
                tx_active_o <= 1'b0;
                low_period_cnt <= 16'd0;
                if (tx_start_i) begin
                    tx_busy_o <= 1'b1;
                    tx_active_o <= 1'b1;
                    tx_header_reg <= tx_header_i;
                    tx_data_len_reg <= tx_data_len_i;
                    
                    // 从展平输入加载到内部数组
                    for (i = 0; i < 30; i = i + 1) begin
                        tx_data_reg[i] <= tx_data_flat_i[i*8 +: 8];
                    end
                    
                    tx_data_byte0 <= tx_data_flat_i[0*8 +: 8];
                    tx_data_byte1 <= tx_data_flat_i[1*8 +: 8];
                    
                    preamble_cnt <= 6'd0;
                    symbol_bit_cnt <= 3'd0;
                    byte_cnt <= 5'd0;
                    sop_symbol_cnt <= 2'd0;
                    byte_nibble_sel <= 1'b0;
                    crc_byte_cnt <= 2'd0;
                    crc_reg <= 32'hFFFFFFFF;
                    final_crc_reg <= 32'h0;
                    tx_state <= ST_PREAMBLE;
                end
            end

            ST_PREAMBLE: begin
                if (bit_tick) begin
                    preamble_cnt <= preamble_cnt + 1'b1;
                    if (preamble_cnt == PREAMBLE_LEN - 1) begin
                        current_symbol <= KCODE_SYNC1;
                        tx_state <= ST_SOP;
                    end
                end
            end

            ST_SOP: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        sop_symbol_cnt <= sop_symbol_cnt + 1'b1;
                        
                        case (sop_symbol_cnt)
                            2'd0: current_symbol <= KCODE_SYNC1;
                            2'd1: current_symbol <= KCODE_SYNC1;
                            2'd2: current_symbol <= KCODE_SYNC2;
                            2'd3: begin
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

            ST_HEADER: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        byte_nibble_sel <= ~byte_nibble_sel;
                        
                        if (!byte_nibble_sel) begin
                            current_symbol <= encode_4b5b_data(current_byte[7:4]);
                        end
                        else begin
                            crc_reg <= crc_next;
                            byte_cnt <= byte_cnt + 1'b1;
                            
                            if (byte_cnt == 5'd1) begin
                                if (tx_data_len_reg > 5'd0) begin
                                    current_byte <= tx_data_reg[0];
                                    current_symbol <= encode_4b5b_data(tx_data_reg[0][3:0]);
                                    byte_nibble_sel <= 1'b0;
                                    byte_cnt <= 5'd0;
                                    tx_state <= ST_DATA;
                                end
                                else begin
                                    final_crc_reg <= ~crc_next;
                                    current_byte <= ~crc_next[7:0];
                                    current_symbol <= encode_4b5b_data(~crc_next[3:0]);
                                    byte_nibble_sel <= 1'b0;
                                    crc_byte_cnt <= 2'd0;
                                    tx_state <= ST_CRC;
                                end
                            end
                            else begin
                                current_byte <= tx_header_reg[15:8];
                                current_symbol <= encode_4b5b_data(tx_header_reg[11:8]);
                            end
                        end
                    end
                end
            end

            ST_DATA: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        byte_nibble_sel <= ~byte_nibble_sel;
                        
                        if (!byte_nibble_sel) begin
                            current_symbol <= encode_4b5b_data(current_byte[7:4]);
                        end
                        else begin
                            crc_reg <= crc_next;
                            byte_cnt <= byte_cnt + 1'b1;
                            
                            if (byte_cnt == tx_data_len_reg - 1) begin
                                final_crc_reg <= ~crc_next;
                                current_byte <= ~crc_next[7:0];
                                current_symbol <= encode_4b5b_data(~crc_next[3:0]);
                                byte_nibble_sel <= 1'b0;
                                crc_byte_cnt <= 2'd0;
                                tx_state <= ST_CRC;
                            end
                            else begin
                                current_byte <= tx_data_reg[byte_cnt + 1];
                                current_symbol <= encode_4b5b_data(tx_data_reg[byte_cnt + 1][3:0]);
                            end
                        end
                    end
                end
            end

            ST_CRC: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        byte_nibble_sel <= ~byte_nibble_sel;
                        
                        if (!byte_nibble_sel) begin
                            current_symbol <= encode_4b5b_data(current_byte[7:4]);
                        end
                        else begin
                            crc_byte_cnt <= crc_byte_cnt + 1'b1;
                            
                            if (crc_byte_cnt == 2'd3) begin
                                current_symbol <= KCODE_EOP;
                                tx_state <= ST_EOP;
                            end
                            else begin
                                case (crc_byte_cnt)
                                    2'd0: begin
                                        current_byte <= final_crc_reg[15:8];
                                        current_symbol <= encode_4b5b_data(final_crc_reg[11:8]);
                                    end
                                    2'd1: begin
                                        current_byte <= final_crc_reg[23:16];
                                        current_symbol <= encode_4b5b_data(final_crc_reg[19:16]);
                                    end
                                    2'd2: begin
                                        current_byte <= final_crc_reg[31:24];
                                        current_symbol <= encode_4b5b_data(final_crc_reg[27:24]);
                                    end
                                endcase
                            end
                        end
                    end
                end
            end

            ST_EOP: begin
                if (bit_tick) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    current_symbol <= {1'b0, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        low_period_cnt <= 16'd0;
                        tx_state <= ST_LOW;
                    end
                end
            end

            ST_LOW: begin
                low_period_cnt <= low_period_cnt + 1'b1;
                if (low_period_cnt == LOW_PERIOD_CNT - 1) begin
                    tx_state <= ST_DONE;
                end
            end

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
// 发送bit选择
//============================================================================
assign tx_bit = (tx_state == ST_PREAMBLE) ? preamble_cnt[0] : current_symbol[0];

//============================================================================
// BMC 编码逻辑
//============================================================================
reg bmc_state;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bmc_state  <= 1'b1;
        bmc_tx_pad <= 1'b1;
    end else if (tx_state == ST_IDLE) begin
        bmc_tx_pad <= 1'b1;
    end else if (tx_state == ST_LOW) begin
        bmc_state  <= 1'b0;
        bmc_tx_pad <= 1'b0;
    end else begin
        if (bit_timer == 16'd0) begin
            bmc_state <= ~bmc_state;
        end
        if (bit_timer == HALF_BIT_CNT) begin
            if (tx_bit) begin
                bmc_state <= ~bmc_state;
            end
        end
        bmc_tx_pad <= bmc_state;
    end
end

endmodule