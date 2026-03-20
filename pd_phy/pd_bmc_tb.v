//============================================================================
// USB PD BMC 收发器测试平台
// 测试 TX 和 RX 完整功能
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tb;

// 参数
parameter CLK_PERIOD = 20;

// 超时计数器 (按时钟周期计数)
integer timeout_cnt;
parameter MAX_TIMEOUT = 50000000;  // 约 1 秒仿真时间 @ 50MHz

// 测试信号
reg         clk;
reg         rst_n;

// 发送接口 (与 pd_bmc_transceiver 匹配)
reg  [15:0] tx_header_i;
reg  [7:0]  tx_data_i [0:29];
reg  [4:0]  tx_data_len_i;
reg         tx_start_i;
wire        tx_done_o;
wire        tx_busy_o;

// 接收接口 (更新为包接口)
wire [15:0] rx_header_o;
wire [7:0]  rx_data_o [0:29];
wire [4:0]  rx_data_len_o;
wire        rx_done_o;
wire        rx_busy_o;
wire        rx_error_o;
reg         rx_en_i;

// Pad 接口
wire        bmc_tx_pad;
wire        bmc_rx_pad;

// 内部回环连接
assign bmc_rx_pad = bmc_tx_pad;

//============================================================================
// 实例化被测模块
//============================================================================
pd_bmc_transceiver u_dut (
    .clk           (clk),
    .rst_n         (rst_n),
    .tx_header_i   (tx_header_i),
    .tx_data_i     (tx_data_i),
    .tx_data_len_i (tx_data_len_i),
    .tx_start_i    (tx_start_i),
    .tx_done_o     (tx_done_o),
    .tx_busy_o     (tx_busy_o),
    .rx_header_o   (rx_header_o),
    .rx_data_o     (rx_data_o),
    .rx_data_len_o (rx_data_len_o),
    .rx_done_o     (rx_done_o),
    .rx_busy_o     (rx_busy_o),
    .rx_error_o    (rx_error_o),
    .rx_en_i       (rx_en_i),
    .bmc_tx_pad    (bmc_tx_pad),
    .bmc_rx_pad    (bmc_rx_pad)
);

//============================================================================
// 时钟生成
//============================================================================
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

//============================================================================
// 超时监控 - 全局超时
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        timeout_cnt <= 0;
    end else begin
        timeout_cnt <= timeout_cnt + 1;
        if (timeout_cnt >= MAX_TIMEOUT) begin
            $display("[%0t] 全局超时！强制结束仿真", $time);
            $display("当前状态: TX_BUSY=%b, RX_BUSY=%b, RX_ERROR=%b", 
                     tx_busy_o, rx_busy_o, rx_error_o);
            $finish;
        end
    end
end

//============================================================================
// 复位序列
//============================================================================
initial begin
    rst_n = 1'b0;
    #100 rst_n = 1'b1;
end

// 监控输出
reg tx_busy_prev;
reg rx_done_prev;
integer rx_count;

initial begin
    tx_busy_prev = 1'b0;
    rx_done_prev = 1'b0;
    rx_count = 0;
end

always @(posedge clk) begin
    // TX 状态监控
    if (tx_done_o) begin
        $display("[%0t] >> TX DONE", $time);
    end
    if (tx_busy_o && !tx_busy_prev) begin
        $display("[%0t] >> TX BUSY 开始", $time);
    end
    if (!tx_busy_o && tx_busy_prev) begin
        $display("[%0t] >> TX BUSY 结束", $time);
    end
    tx_busy_prev <= tx_busy_o;
    
    // RX 状态监控
    if (rx_done_o && !rx_done_prev) begin
        rx_count <= rx_count + 1;
        $display("[%0t] << RX DONE #%0d: Header=0x%04h, DataLen=%0d", 
                 $time, rx_count, rx_header_o, rx_data_len_o);
        if (rx_data_len_o > 0) begin
            $display("    Data[0]=0x%02h, Data[1]=0x%02h", rx_data_o[0], rx_data_o[1]);
        end
    end
    if (rx_error_o) begin
        $display("[%0t] !! RX ERROR!", $time);
    end
    rx_done_prev <= rx_done_o;
end

//============================================================================
// 任务定义
//============================================================================
task send_packet;
    input [15:0] header;
    input [4:0]  data_len;
    input [7:0]  data [];
    integer i;
    integer wait_cnt;
    begin
        tx_header_i = header;
        tx_data_len_i = data_len;
        for (i = 0; i < 30; i = i + 1) begin
            if (i < data_len) begin
                tx_data_i[i] = data[i];
            end else begin
                tx_data_i[i] = 8'd0;
            end
        end
        
        @(posedge clk);
        tx_start_i = 1'b1;
        @(posedge clk);
        tx_start_i = 1'b0;
        
        // 等待发送完成
        wait_cnt = 0;
        while (!tx_done_o && wait_cnt < 100000) begin
            @(posedge clk);
            wait_cnt = wait_cnt + 1;
        end
        
        if (!tx_done_o) begin
            $display("[%0t] 发送超时!", $time);
        end
        
        // 等待接收完成
        wait_cnt = 0;
        while (!rx_done_o && wait_cnt < 100000) begin
            @(posedge clk);
            wait_cnt = wait_cnt + 1;
        end
        
        if (!rx_done_o) begin
            $display("[%0t] 接收超时!", $time);
        end else begin
            $display("[%0t] 接收完成", $time);
        end
        
        // 短暂复位 RX 以准备接收下一包
        rx_en_i = 1'b0;
        #100;
        rx_en_i = 1'b1;
        
        #500; // 包间隔
    end
endtask

//============================================================================
// 主测试流程
//============================================================================
initial begin
    // 初始化
    rst_n = 1'b0;
    tx_header_i = 16'd0;
    for (integer i = 0; i < 30; i = i + 1) begin
        tx_data_i[i] = 8'd0;
    end
    tx_data_len_i = 5'd0;
    tx_start_i = 1'b0;
    rx_en_i = 1'b1;
    timeout_cnt = 0;
    
    // 释放复位
    #100;
    rst_n = 1'b1;
    #50;
    
    $display("========================================");
    $display("=== 开始 USB PD BMC 收发器测试 ===");
    $display("========================================");
    
    // 测试 1: 发送无数据载荷的包 (仅 Header)
    // Header: 0x0000 表示无数据 (data_obj_num = 0)
    $display("\n[测试 1] 发送无数据载荷的包 (Header only)");
    send_packet(16'h03a3, 5'd0, '{});
    
    // 验证结果
    if (rx_header_o == 16'h03a3 && rx_data_len_o == 5'd0 && !rx_error_o) begin
        $display("[测试 1] PASS - Header 正确接收");
    end else begin
        $display("[测试 1] FAIL - Header 或数据长度错误");
    end

    $display("\n========================================");
    $display("=== 测试结束 ===");
    $display("========================================");
    
    #1000;
    $finish;
end

//============================================================================
// 波形输出
//============================================================================
initial begin
    $dumpfile("pd_bmc_tb.vcd");
    $dumpvars(0, pd_bmc_tb);
end

endmodule