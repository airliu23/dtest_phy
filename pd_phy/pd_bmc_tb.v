//============================================================================
// USB PD BMC 收发器测试平台
// 测试 TX 和 RX 完整功能
//============================================================================

`timescale 1ns/1ps

module pd_bmc_tb;

//============================================================================
// 参数定义
//============================================================================
// 200MHz 差分时钟周期 = 5ns
parameter CLK_200M_PERIOD = 5;

// 超时计数器
integer timeout_cnt;
parameter MAX_TIMEOUT = 10000000;

//============================================================================
// 测试信号
//============================================================================
// 差分时钟
reg  sys_clk_p;
wire sys_clk_n;
assign sys_clk_n = ~sys_clk_p;

// 复位
reg  rst_n;

// LED 输出
wire led_tx;
wire led_rx_ok;
wire led_rx_err;
wire led_heartbeat;

// BMC Pad 接口
wire bmc_tx_pad;
wire bmc_rx_pad;

// 内部回环：TX 输出连接到 RX 输入
assign bmc_rx_pad = bmc_tx_pad;

//============================================================================
// 实例化被测模块 (pd_bmc_transceiver)
//============================================================================
pd_bmc_transceiver u_dut (
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    .led_tx         (led_tx),
    .led_rx_ok      (led_rx_ok),
    .led_rx_err     (led_rx_err),
    .led_heartbeat  (led_heartbeat),
    .bmc_tx_pad     (bmc_tx_pad),
    .bmc_rx_pad     (bmc_rx_pad)
);

//============================================================================
// 200MHz 差分时钟生成
//============================================================================
initial begin
    sys_clk_p = 1'b0;
    forever #(CLK_200M_PERIOD/2) sys_clk_p = ~sys_clk_p;
end

//============================================================================
// 超时监控 (使用 200MHz 时钟)
//============================================================================
always @(posedge sys_clk_p or negedge rst_n) begin
    if (!rst_n) begin
        timeout_cnt <= 0;
    end else begin
        timeout_cnt <= timeout_cnt + 1;
        if (timeout_cnt >= MAX_TIMEOUT) begin
            $display("[%0t] 全局超时！强制结束仿真", $time);
            $display("当前状态: LED_TX=%b, LED_RX_OK=%b, LED_RX_ERR=%b", 
                     led_tx, led_rx_ok, led_rx_err);
            $finish;
        end
    end
end

//============================================================================
// LED 状态监控
//============================================================================
reg led_tx_prev;
reg led_rx_ok_prev;
reg led_rx_err_prev;

initial begin
    led_tx_prev = 1'b1;
    led_rx_ok_prev = 1'b1;
    led_rx_err_prev = 1'b1;
end

always @(posedge sys_clk_p) begin
    // LED_TX 监控 (低电平点亮表示 TX 忙)
    if (!led_tx && led_tx_prev) begin
        $display("[%0t] >> TX LED 亮起 (TX BUSY)", $time);
    end
    if (led_tx && !led_tx_prev) begin
        $display("[%0t] >> TX LED 熄灭 (TX DONE)", $time);
    end
    led_tx_prev <= led_tx;
    
    // LED_RX_OK 监控 (低电平点亮表示 RX 完成)
    if (!led_rx_ok && led_rx_ok_prev) begin
        $display("[%0t] << RX OK LED 亮起 (RX DONE)", $time);
    end
    led_rx_ok_prev <= led_rx_ok;
    
    // LED_RX_ERR 监控 (低电平点亮表示 RX 错误)
    if (!led_rx_err && led_rx_err_prev) begin
        $display("[%0t] !! RX ERROR LED 亮起", $time);
    end
    led_rx_err_prev <= led_rx_err;
end

//============================================================================
// BMC 信号监控
//============================================================================
reg bmc_tx_prev;

initial begin
    bmc_tx_prev = 1'b0;
end

always @(posedge sys_clk_p) begin
    if (bmc_tx_pad != bmc_tx_prev) begin
        // BMC 信号翻转，说明正在传输
        // $display("[%0t] BMC_TX: %b -> %b", $time, bmc_tx_prev, bmc_tx_pad);
    end
    bmc_tx_prev <= bmc_tx_pad;
end

//============================================================================
// 主测试流程
//============================================================================
initial begin
    // 初始化
    rst_n = 1'b0;
    timeout_cnt = 0;
    
    $display("========================================");
    $display("=== USB PD BMC 收发器测试开始 ===");
    $display("========================================");
    $display("注意: pd_bmc_transceiver 模块内部有自测试激励");
    $display("      延迟 100M 周期后自动发送测试包");
    $display("      Header=0x03A3, DataLen=2, Data={0x55, 0xAA}");
    $display("========================================");
    
    // 释放复位
    #100;
    rst_n = 1'b1;
    $display("[%0t] 复位释放", $time);
    
    // 等待内部自测试完成
    // pd_bmc_transceiver 内部会在 delay_cnt == 100000000 后开始发送
    // 为了仿真快速，我们只等待一段时间观察 LED 变化
    
    // 等待 TX 开始 (LED_TX 变低)
    wait(led_tx == 1'b0);
    $display("[%0t] 检测到 TX 开始", $time);
    
    // 等待 TX 结束 (LED_TX 变高)
    wait(led_tx == 1'b1);
    $display("[%0t] 检测到 TX 结束", $time);
    
    // 等待 RX 完成 (LED_RX_OK 变低)
    wait(led_rx_ok == 1'b0);
    $display("[%0t] 检测到 RX 完成", $time);
    
    // 检查是否有错误
    #1000;
    if (led_rx_err == 1'b1) begin
        $display("[测试结果] PASS - 无 RX 错误");
    end else begin
        $display("[测试结果] FAIL - 检测到 RX 错误");
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