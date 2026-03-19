//============================================================================
// USB PD BMC 收发器完整测试平台
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tb;

// 参数
parameter CLK_PERIOD = 20;

// 测试信号
reg         clk;
reg         rst_n;
reg  [7:0]  tx_data_i;
wire        tx_ready_o;
reg         tx_start_i;
reg         tx_sync_i;
wire        tx_done_o;
wire [7:0]  rx_data_o;
wire        rx_valid_o;
wire        rx_error_o;
reg         rx_en_i;
wire        bmc_tx_pad;
wire        bmc_rx_pad;

// 超时计数器
integer timeout_cnt;
parameter MAX_TIMEOUT = 50000;

// 内部回环连接
assign bmc_rx_pad = bmc_tx_pad;

// 实例化被测模块
pd_bmc_transceiver u_dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .tx_data_i    (tx_data_i),
    .tx_valid_i   (1'b0),
    .tx_ready_o   (tx_ready_o),
    .tx_start_i   (tx_start_i),
    .tx_sync_i    (tx_sync_i),
    .tx_done_o    (tx_done_o),
    .rx_data_o    (rx_data_o),
    .rx_valid_o   (rx_valid_o),
    .rx_error_o   (rx_error_o),
    .rx_en_i      (rx_en_i),
    .bmc_tx_pad   (bmc_tx_pad),
    .bmc_rx_pad   (bmc_rx_pad)
);

// 时钟生成
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// 超时监控
always @(posedge clk) begin
    timeout_cnt <= timeout_cnt + 1;
    if (timeout_cnt >= MAX_TIMEOUT) begin
        $display("[%0t] 超时！强制结束仿真", $time);
        $finish;
    end
end

// 复位和测试
reg [31:0] i;

initial begin
    i = 0;
    
    // 初始化
    rst_n = 1'b0;
    tx_data_i = 8'd0;
    tx_start_i = 1'b0;
    tx_sync_i = 1'b0;
    rx_en_i = 1'b1;
    timeout_cnt = 0;
    
    // 释放复位
    #100;
    rst_n = 1'b1;
    #50;
    
    $display("========================================");
    $display("=== 开始 USB PD BMC 收发器测试 ===");
    $display("========================================");
    $display("[%0t] 复位完成，tx_ready=%b", $time, tx_ready_o);
    
    // 测试 1: 发送数据 0xAA
    $display("\n[测试 1] 发送数据 0xAA");
    wait(tx_ready_o);
    @(posedge clk);
    tx_data_i = 8'hAA;
    tx_start_i = 1'b1;
    @(posedge clk);
    tx_start_i = 1'b0;
    wait(tx_done_o);
    $display("[%0t] 发送完成", $time);
    #1000;
    
    // 测试 2: 发送数据 0x55
    $display("\n[测试 2] 发送数据 0x55");
    wait(tx_ready_o);
    @(posedge clk);
    tx_data_i = 8'h55;
    tx_start_i = 1'b1;
    @(posedge clk);
    tx_start_i = 1'b0;
    wait(tx_done_o);
    $display("[%0t] 发送完成", $time);
    #1000;
    
    // 测试 3: 发送数据 0x01
    $display("\n[测试 3] 发送数据 0x01");
    wait(tx_ready_o);
    @(posedge clk);
    tx_data_i = 8'h01;
    tx_start_i = 1'b1;
    @(posedge clk);
    tx_start_i = 1'b0;
    wait(tx_done_o);
    $display("[%0t] 发送完成", $time);
    #1000;
    
    $display("\n========================================");
    $display("=== 所有测试完成 ===");
    $display("========================================");
    
    #1000;
    $finish;
end

// 监控输出
reg [7:0] tx_count;
reg [7:0] rx_count;

initial begin
    tx_count = 0;
    rx_count = 0;
end

always @(posedge clk) begin
    if (tx_done_o) begin
        tx_count = tx_count + 1;
        $display("[%0t] >> TX DONE (count=%0d)", $time, tx_count);
    end
    if (rx_valid_o) begin
        rx_count = rx_count + 1;
        $display("[%0t] << RX VALID: data = 0x%02h (count=%0d)", $time, rx_data_o, rx_count);
    end
    if (rx_error_o) begin
        $display("[%0t] !! RX ERROR!", $time);
    end
end

// 波形输出
initial begin
    $dumpfile("pd_bmc_tb.vcd");
    $dumpvars(0, pd_bmc_tb);
end

endmodule