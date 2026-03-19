//============================================================================
// USB PD BMC 收发器测试平台
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tb;

// 参数
parameter CLK_PERIOD = 20;

// 超时计数器
integer timeout_cnt;
parameter MAX_TIMEOUT = 500000;

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

// 接收接口
wire [7:0]  rx_data_o;
wire        rx_valid_o;
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
    .rx_data_o     (rx_data_o),
    .rx_valid_o    (rx_valid_o),
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
// 超时监控
//============================================================================
always @(posedge clk) begin
    timeout_cnt <= timeout_cnt + 1;
    if (timeout_cnt >= MAX_TIMEOUT) begin
        $display("[%0t] 超时！强制结束仿真", $time);
        $finish;
    end
end

//============================================================================
// 复位序列
//============================================================================
initial begin
    rst_n = 1'b0;
    #100 rst_n = 1'b1;
end

// 监控输出（在初始化之前先定义）
reg tx_busy_prev;
initial begin
    tx_busy_prev = 1'b0;
end

always @(posedge clk) begin
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
    
    if (rx_valid_o) begin
        $display("[%0t] << RX VALID: data = 0x%02h", $time, rx_data_o);
    end
    if (rx_error_o) begin
        $display("[%0t] !! RX ERROR!", $time);
    end
end

//============================================================================
// 主测试流程
//============================================================================
integer i;
integer wait_cnt;

initial begin
    // 初始化
    rst_n = 1'b0;
    tx_header_i = 16'd0;
    for (i = 0; i < 30; i = i + 1) begin
        tx_data_i[i] = 8'd0;
    end
    tx_data_len_i = 5'd0;
    tx_start_i = 1'b0;
    rx_en_i = 1'b1;
    timeout_cnt = 0;
    wait_cnt = 0;
    
    // 释放复位
    #100;
    rst_n = 1'b1;
    #50;
    
    $display("========================================");
    $display("=== 开始 USB PD BMC 收发器测试 ===");
    $display("========================================");
    $display("[%0t] 初始状态：tx_busy=%b, tx_done=%b", $time, tx_busy_o, tx_done_o);
    
    // 测试 1: 发送 1 字节数据
    $display("\n[测试 1] 发送 1 字节数据 0xAA");
    // CRC 0x5DFAAC6F
    tx_header_i = 16'h03a3;
    tx_data_len_i = 5'd0;
    
    // 启动发送：tx_start_i 需要保持至少一个时钟周期
    @(posedge clk);
    tx_start_i = 1'b1;
    $display("[%0t] tx_start_i=1, tx_busy=%b", $time, tx_busy_o);
    
    @(posedge clk);
    tx_start_i = 1'b0;
    $display("[%0t] tx_start_i=0, tx_busy=%b", $time, tx_busy_o);
    
    // 等待发送完成
    wait_cnt = 0;
    while (!tx_done_o && wait_cnt < 100000) begin
        @(posedge clk);
        wait_cnt = wait_cnt + 1;
    end
    
    if (tx_done_o) begin
        $display("[%0t] 测试 1 发送完成!", $time);
    end else begin
        $display("[%0t] 测试 1 超时!", $time);
    end
    
    #1000;
    
    // 测试 2: 发送 5 字节数据
    $display("\n[测试 2] 发送 5 字节数据");
    tx_header_i = 16'h0200;
    tx_data_i[0] = 8'h11;
    tx_data_i[1] = 8'h22;
    tx_data_i[2] = 8'h33;
    tx_data_i[3] = 8'h44;
    tx_data_i[4] = 8'h55;
    tx_data_len_i = 5'd5;
    tx_start_i = 1'b1;
    
    @(posedge clk);
    @(posedge clk);
    tx_start_i = 1'b0;
    
    // 等待发送完成
    wait_cnt = 0;
    while (!tx_done_o && wait_cnt < 200000) begin
        @(posedge clk);
        wait_cnt = wait_cnt + 1;
    end
    
    if (tx_done_o) begin
        $display("[%0t] 测试 2 发送完成!", $time);
    end else begin
        $display("[%0t] 测试 2 超时!", $time);
    end
    
    $display("\n========================================");
    $display("=== 测试结束 ===");
    $display("========================================");
    
    #1000;
    $finish;
end

//============================================================================
// 监控输出
//============================================================================
always @(posedge clk) begin
    if (tx_done_o) begin
        $display("[%0t] >> TX DONE", $time);
    end
    if (tx_busy_o) begin
        if ((timeout_cnt % 50000) == 0) begin
            $display("[%0t] >> TX BUSY (cnt=%0d)", $time, timeout_cnt);
        end
    end
    if (rx_valid_o) begin
        $display("[%0t] << RX VALID: data = 0x%02h", $time, rx_data_o);
    end
    if (rx_error_o) begin
        $display("[%0t] !! RX ERROR!", $time);
    end
end

//============================================================================
// 波形输出
//============================================================================
initial begin
    $dumpfile("pd_bmc_tb.vcd");
    $dumpvars(0, pd_bmc_tb);
end

endmodule