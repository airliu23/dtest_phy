//============================================================================
// USB PD BMC 发送模块
// 标准速率：300 kbps
// 接口：8 位并行输入
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tx (
    input  wire        clk,           // 系统时钟 (如 50MHz)
    input  wire        rst_n,         // 异步复位，低有效
    
    // AXI 风格握手接口
    input  wire [7:0]  tx_data_i,     // 发送数据
    input  wire        tx_valid_i,    // 发送数据有效
    output reg         tx_ready_o,    // 发送就绪
    
    input  wire        tx_start_i,    // 传输开始信号
    input  wire        tx_sync_i,     // 发送 SYNC 字段
    output reg         tx_done_o,     // 发送完成
    
    output reg         bmc_tx_pad,    // BMC 编码输出
    output reg         tx_active_o    // 发送进行中
);

//============================================================================
// 参数定义
//============================================================================
// 假设系统时钟 50MHz，位周期 = 1/300k = 3.333us
// BIT_PERIOD_CNT = 50M / 300k - 1 = 166
// 
// 注意：为了加快仿真速度，可以使用较小的计数值
// 使用较小的计数值加快仿真速度
// 实际使用时请改回 166
parameter BIT_PERIOD_CNT = 166;  // 仿真加速
parameter HALF_BIT_CNT   = 83;

//============================================================================
// 内部信号
//============================================================================
reg [15:0] bit_timer;          // 位定时器
reg [2:0]  bit_index;          // 位索引 (0-7)
reg [7:0]  tx_data_reg;        // 发送数据寄存器
wire       tx_bit;             // 当前发送的位
reg        bmc_state;          // BMC 编码状态 (0=低，1=高)
reg        tx_running;         // 发送运行状态
reg        sync_mode;          // SYNC 发送模式
reg [1:0]  sync_cnt;           // SYNC 字段计数器 (0=SYNC-1, 1=SYNC-2, 2=SYNC-3)

// 位周期完成信号
wire bit_tick = (bit_timer == BIT_PERIOD_CNT);

//============================================================================
// 位定时器
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_timer <= 16'd0;
    end else if (!tx_running) begin
        bit_timer <= 16'd0;
    end else if (bit_tick) begin
        bit_timer <= 16'd0;
    end else begin
        bit_timer <= bit_timer + 1'b1;
    end
end

//============================================================================
// 发送控制状态机
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_running   <= 1'b0;
        tx_data_reg  <= 8'd0;
        bit_index    <= 3'd0;
        tx_ready_o   <= 1'b1;
        tx_done_o    <= 1'b0;
        tx_active_o  <= 1'b0;
        sync_mode    <= 1'b0;
        sync_cnt     <= 2'd0;
    end else begin
        tx_done_o <= 1'b0;  // 默认清零
        
        // 启动发送
        if (tx_start_i && tx_ready_o) begin
            tx_running  <= 1'b1;
            tx_data_reg <= tx_data_i;
            bit_index   <= 3'd0;
            tx_ready_o  <= 1'b0;
            tx_active_o <= 1'b1;
            sync_mode   <= tx_sync_i;
            sync_cnt    <= 2'd0;
            
            // SYNC 模式下加载 SYNC-1
            if (tx_sync_i) begin
                tx_data_reg <= 8'hFF;  // SYNC-1
            end
        end
        
        // 发送进行中
        if (tx_running && bit_tick) begin
            // 移位
            tx_data_reg <= tx_data_reg >> 1;
            // bit_index   <= bit_index + 1'b1;
            
            // 检查是否完成一个字节 (8 位)
            // if (bit_index == 3'd7) begin
            //     if (sync_mode) begin
            //         // SYNC 模式：继续发送下一个 SYNC 字段
            //         sync_cnt <= sync_cnt + 1'b1;
                    
            //         case (sync_cnt)
            //             2'd0: begin
            //                 // SYNC-1 完成，加载 SYNC-2
            //                 tx_data_reg <= 8'h0F;
            //                 bit_index   <= 3'd0;
            //             end
            //             2'd1: begin
            //                 // SYNC-2 完成，加载 SYNC-3
            //                 tx_data_reg <= 8'h3F;
            //                 bit_index   <= 3'd0;
            //             end
            //             2'd2: begin
            //                 // SYNC-3 完成，结束
            //                 tx_running  <= 1'b0;
            //                 tx_ready_o  <= 1'b1;
            //                 tx_done_o   <= 1'b1;
            //                 tx_active_o <= 1'b0;
            //                 sync_cnt    <= 2'd0;
            //                 sync_mode   <= 1'b0;
            //             end
            //         endcase
            //     end else begin
            //         // 普通数据模式：结束
            //         tx_running  <= 1'b0;
            //         tx_ready_o  <= 1'b1;
            //         tx_done_o   <= 1'b1;
            //         tx_active_o <= 1'b0;
            //     end
            // end
        end
    end
end

// 当前发送的位 (LSB first)
assign tx_bit = tx_data_reg[0];

//============================================================================
// BMC 编码逻辑
// BMC 编码规则:
// - 每个比特周期中间都有跳变
// - 逻辑"1": 在比特周期开始处有额外跳变
// - 逻辑"0": 在比特周期开始处无额外跳变
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bmc_state  <= 1'b1;
        bmc_tx_pad <= 1'b1;
    end else if (!tx_running) begin
        bmc_tx_pad <= 1'b1;  // 空闲状态为高 (J 状态)
    end else begin
        // 比特周期开始时的跳变 (逻辑 1 产生跳变)
        if (bit_timer == 16'd0) begin
            bmc_state <= ~bmc_state;
        end
        
        // 比特周期中间的固定跳变
        if (bit_timer == HALF_BIT_CNT) begin
            if (tx_bit) begin
                bmc_state <= ~bmc_state;
            end
        end
        
        bmc_tx_pad <= bmc_state;
    end
end

endmodule