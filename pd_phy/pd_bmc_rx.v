//============================================================================
// USB PD BMC 接收模块
// 标准速率：300 kbps
// 接口：8 位并行输出
// 特性：自动检测并跳过 64bit 前导码，然后接收 8bit 数据
//============================================================================

`timescale 1ns/1ps

module pd_bmc_rx (
    input  wire        clk,           // 系统时钟 (如 50MHz)
    input  wire        rst_n,         // 异步复位，低有效
    
    // 数据输出接口
    output reg [7:0]   rx_data_o,     // 接收数据
    output reg         rx_valid_o,    // 接收数据有效
    output reg         rx_error_o,    // 接收错误 (BMC 违例检测)
    
    input  wire        bmc_rx_pad,    // BMC 编码输入
    input  wire        rx_en_i        // 接收使能
);

//============================================================================
// 参数定义
//============================================================================
parameter BIT_PERIOD_CNT = 166;  // 与发送模块匹配
parameter SAMPLES_PER_BIT = 167; // BIT_PERIOD_CNT + 1
parameter HALF_SAMPLE = 83;      // HALF_BIT_CNT

//============================================================================
// 状态机定义
//============================================================================
localparam [1:0] ST_IDLE      = 2'd0;  // 空闲状态
localparam [1:0] ST_PREAMBLE  = 2'd1;  // 接收 64bit 前导码
localparam [1:0] ST_DATA      = 2'd2;  // 接收 8bit 数据
localparam [1:0] ST_DONE      = 2'd3;  // 完成状态

//============================================================================
// 内部信号
//============================================================================
reg [15:0] sample_cnt;         // 采样计数器
reg [6:0]  bit_cnt;            // 位计数器 (0-63 用于前导码，0-7 用于数据)
reg [7:0]  rx_shift_reg;       // 接收移位寄存器
reg [1:0]  state;              // 状态机状态
reg        prev_bmc_val;       // 上一周期的 BMC 值
reg        half_edge_seen;     // 前半周期边沿标志

// 输入信号同步
reg [2:0] pad_sync;
wire bmc_rx_synced = pad_sync[2];

// 边沿检测
wire any_edge = bmc_rx_synced ^ prev_bmc_val;

//============================================================================
// 输入信号同步
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_sync <= 3'b111;
    else
        pad_sync <= {pad_sync[1:0], bmc_rx_pad};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        prev_bmc_val <= 1'b1;
    else
        prev_bmc_val <= bmc_rx_synced;
end

//============================================================================
// 采样计数器
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_cnt <= 16'd0;
    end else if (state == ST_IDLE) begin
        sample_cnt <= 16'd0;
    end else if (sample_cnt >= SAMPLES_PER_BIT - 1) begin
        sample_cnt <= 16'd0;
    end else begin
        sample_cnt <= sample_cnt + 1'b1;
    end
end

//============================================================================
// 接收控制状态机
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= ST_IDLE;
        rx_shift_reg   <= 8'd0;
        bit_cnt        <= 7'd0;
        rx_valid_o     <= 1'b0;
        rx_error_o     <= 1'b0;
        rx_data_o      <= 8'd0;
        half_edge_seen <= 1'b0;
    end else begin
        rx_valid_o <= 1'b0;
        rx_error_o <= 1'b0;
        
        case (state)
            ST_IDLE: begin
                // 启动接收：检测到第一个边沿
                if (rx_en_i && any_edge) begin
                    state <= ST_PREAMBLE;
                    bit_cnt <= 7'd0;
                end
            end
            
            ST_PREAMBLE: begin
                // 接收 64bit 前导码 (010101...)
                // 前导码不需要存储，只需计数
                
                // 检测前半周期的边沿
                if (any_edge && sample_cnt < HALF_SAMPLE) begin
                    half_edge_seen <= 1'b1;
                end
                
                if (sample_cnt == SAMPLES_PER_BIT - 1) begin
                    half_edge_seen <= 1'b0;
                    
                    if (bit_cnt == 7'd63) begin
                        // 前导码接收完成，开始接收数据
                        state <= ST_DATA;
                        bit_cnt <= 7'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
            end
            
            ST_DATA: begin
                // 接收 8bit 数据
                // 检测前半周期的边沿
                if (any_edge && sample_cnt < HALF_SAMPLE) begin
                    half_edge_seen <= 1'b1;
                end
                
                if (sample_cnt == SAMPLES_PER_BIT - 1) begin
                    // BMC 解码：half_edge_seen=1 表示逻辑 1，否则为 0
                    rx_shift_reg <= {half_edge_seen, rx_shift_reg[7:1]};
                    half_edge_seen <= 1'b0;
                    
                    if (bit_cnt < 7'd7) begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end else begin
                        // 8 位接收完成
                        rx_data_o <= rx_shift_reg;
                        rx_valid_o <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
            end
            
            ST_DONE: begin
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule