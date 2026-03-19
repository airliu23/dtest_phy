//============================================================================
// USB PD BMC 接收模块
// 标准速率：300 kbps
// 接口：8 位并行输出
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
// 与发送模块匹配的参数
// 发送模块：BIT_PERIOD_CNT = 10, HALF_BIT_CNT = 5
parameter SAMPLES_PER_BIT = 11;  // 对应 BIT_PERIOD_CNT + 1
parameter HALF_SAMPLE = 5;       // 对应 HALF_BIT_CNT

//============================================================================
// 内部信号
//============================================================================
reg [15:0] sample_cnt;         // 采样计数器
reg [7:0]  rx_shift_reg;       // 接收移位寄存器
reg [2:0]  rx_bit_cnt;         // 接收位计数器
reg        rx_running;         // 接收运行状态
reg [1:0]  sync_cnt;           // SYNC 序列计数器
reg        prev_bmc_val;       // 上一周期的 BMC 值
reg        bit_start_val;      // 位周期开始时的 BMC 值
reg        half_sample_val;    // 半周期采样时的 BMC 值
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
    end else if (!rx_running) begin
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
        rx_running       <= 1'b0;
        rx_shift_reg     <= 8'd0;
        rx_bit_cnt       <= 3'd0;
        rx_valid_o       <= 1'b0;
        rx_error_o       <= 1'b0;
        rx_data_o        <= 8'd0;
        sync_cnt         <= 2'd0;
        bit_start_val    <= 1'b1;
        half_sample_val  <= 1'b0;
        half_edge_seen   <= 1'b0;
    end else begin
        rx_valid_o <= 1'b0;
        rx_error_o <= 1'b0;
        
        // 启动接收：检测到第一个边沿
        if (!rx_running && rx_en_i && any_edge) begin
            rx_running       <= 1'b1;
            sample_cnt       <= 16'd0;
            rx_bit_cnt       <= 3'd0;
            rx_shift_reg     <= 8'd0;
            sync_cnt         <= 2'd0;
            half_edge_seen   <= 1'b0;
        end
        
        // 接收进行中
        if (rx_running) begin
            // 检测前半周期的边沿（用于 BMC 解码）
            if (any_edge && sample_cnt < HALF_SAMPLE) begin
                half_edge_seen <= 1'b1;
            end
            
            // 在位周期开始时记录 BMC 值
            if (sample_cnt == 0) begin
                bit_start_val <= bmc_rx_synced;
            end
            
            // 在半周期时记录 BMC 值
            if (sample_cnt == HALF_SAMPLE) begin
                half_sample_val <= bmc_rx_synced;
            end
            
            // 在位周期结束时采样数据
            if (sample_cnt == SAMPLES_PER_BIT - 1) begin
                // BMC 解码规则：
                // 逻辑 0: 仅在周期中间跳变 (bit_timer = HALF_BIT_CNT)
                // 逻辑 1: 周期开始 (bit_timer = 0) 和中间都跳变
                
                // 使用 half_edge_seen 标志判断数据
                rx_shift_reg <= {half_edge_seen, rx_shift_reg[7:1]};
                
                // 重置标志
                half_edge_seen <= 1'b0;
                
                if (rx_bit_cnt < 3'd7) begin
                    rx_bit_cnt <= rx_bit_cnt + 1'b1;
                end else begin
                    // 8 位接收完成
                    rx_data_o <= rx_shift_reg;
                    rx_valid_o <= 1'b1;
                    rx_bit_cnt <= 3'd0;
                    
                    // SYNC 检测
                    case (sync_cnt)
                        2'd0: begin
                            if (rx_shift_reg == 8'hFF) begin
                                sync_cnt <= 2'd1;
                                rx_running <= 1'b1;
                            end else begin
                                rx_running <= 1'b0;
                            end
                        end
                        2'd1: begin
                            if (rx_shift_reg == 8'h0F) begin
                                sync_cnt <= 2'd2;
                                rx_running <= 1'b1;
                            end else begin
                                sync_cnt <= 2'd0;
                                rx_running <= 1'b0;
                            end
                        end
                        2'd2: begin
                            sync_cnt <= 2'd0;
                            rx_running <= 1'b0;
                        end
                    endcase
                end
            end
        end
    end
end

endmodule