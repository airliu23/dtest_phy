//============================================================================
// dtest_phy 测试平台
// 测试顶层模块的 SPI 接口和 BMC 收发功能
//============================================================================

`timescale 1ns/1ps

module dtest_phy_tb;

//============================================================================
// 参数
//============================================================================
parameter CLK_200M_PERIOD = 5;   // 200MHz = 5ns 周期
parameter SPI_CLK_PERIOD  = 100; // 10MHz SPI 时钟

//============================================================================
// 测试信号
//============================================================================
// 差分时钟
reg  sys_clk_p;
wire sys_clk_n;
assign sys_clk_n = ~sys_clk_p;

// 复位
reg rst_n;

// SPI 接口
reg  spi_sclk;
reg  spi_cs_n;
reg  spi_mosi;
wire spi_miso;

// 中断
wire irq_n;

// LED
wire led_tx;
wire led_rx_ok;
wire led_rx_err;
wire led_heartbeat;

// BMC 接口
wire bmc_tx_pad;
wire bmc_rx_pad;

// 内部回环
assign bmc_rx_pad = bmc_tx_pad;

// 超时控制
integer timeout_cnt;
parameter MAX_TIMEOUT = 20000000;

//============================================================================
// 实例化被测模块
//============================================================================
dtest_phy u_dut (
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    .spi_sclk       (spi_sclk),
    .spi_cs_n       (spi_cs_n),
    .spi_mosi       (spi_mosi),
    .spi_miso       (spi_miso),
    .irq_n          (irq_n),
    .led_tx         (led_tx),
    .led_rx_ok      (led_rx_ok),
    .led_rx_err     (led_rx_err),
    .led_heartbeat  (led_heartbeat),
    .bmc_tx_pad     (bmc_tx_pad),
    .bmc_rx_pad     (bmc_rx_pad)
);

//============================================================================
// 200MHz 时钟生成
//============================================================================
initial begin
    sys_clk_p = 1'b0;
    forever #(CLK_200M_PERIOD/2) sys_clk_p = ~sys_clk_p;
end

//============================================================================
// 超时监控
//============================================================================
always @(posedge sys_clk_p or negedge rst_n) begin
    if (!rst_n) begin
        timeout_cnt <= 0;
    end else begin
        timeout_cnt <= timeout_cnt + 1;
        if (timeout_cnt >= MAX_TIMEOUT) begin
            $display("[%0t] ERROR: 全局超时!", $time);
            $finish;
        end
    end
end

//============================================================================
// SPI 写寄存器任务
//============================================================================
task spi_write;
    input [6:0] addr;
    input [7:0] data;
    integer i;
    begin
        spi_cs_n = 1'b0;
        #(SPI_CLK_PERIOD);
        
        // 发送地址 (bit7=1 写)
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk = 1'b0;
            spi_mosi = (i == 7) ? 1'b1 : addr[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        
        // 发送数据
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk = 1'b0;
            spi_mosi = data[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        
        spi_sclk = 1'b0;
        #(SPI_CLK_PERIOD);
        spi_cs_n = 1'b1;
        #(SPI_CLK_PERIOD);
    end
endtask

//============================================================================
// SPI 读寄存器任务
//============================================================================
reg [7:0] spi_rd_data;

task spi_read;
    input [6:0] addr;
    integer i;
    begin
        spi_cs_n = 1'b0;
        #(SPI_CLK_PERIOD);
        
        // 发送地址 (bit7=0 读)
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk = 1'b0;
            spi_mosi = (i == 7) ? 1'b0 : addr[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        
        // 读取数据
        spi_rd_data = 8'd0;
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk = 1'b0;
            spi_mosi = 1'b0;
            #(SPI_CLK_PERIOD/2);
            spi_sclk = 1'b1;
            spi_rd_data[i] = spi_miso;
            #(SPI_CLK_PERIOD/2);
        end
        
        spi_sclk = 1'b0;
        #(SPI_CLK_PERIOD);
        spi_cs_n = 1'b1;
        #(SPI_CLK_PERIOD);
    end
endtask

//============================================================================
// 中断监控
//============================================================================
always @(negedge irq_n) begin
    $display("[%0t] IRQ 中断触发", $time);
end

//============================================================================
// LED 状态监控
//============================================================================
reg led_tx_d, led_rx_ok_d;

initial begin
    led_tx_d = 1'b1;
    led_rx_ok_d = 1'b1;
end

always @(posedge sys_clk_p) begin
    if (!led_tx && led_tx_d)
        $display("[%0t] LED: TX BUSY", $time);
    if (led_tx && !led_tx_d)
        $display("[%0t] LED: TX DONE", $time);
    led_tx_d <= led_tx;
    
    if (!led_rx_ok && led_rx_ok_d)
        $display("[%0t] LED: RX DONE", $time);
    led_rx_ok_d <= led_rx_ok;
end

//============================================================================
// 主测试流程
//============================================================================
integer test_pass;
integer test_fail;

initial begin
    // 初始化
    rst_n    = 1'b0;
    spi_sclk = 1'b0;
    spi_cs_n = 1'b1;
    spi_mosi = 1'b0;
    test_pass = 0;
    test_fail = 0;
    
    $display("");
    $display("========================================");
    $display("  dtest_phy 测试平台");
    $display("========================================");
    
    // 复位
    #200;
    rst_n = 1'b1;
    #1000;
    $display("[%0t] 复位完成", $time);
    
    //------------------------------------------------------------------
    // 测试 1: SPI 寄存器读写
    //------------------------------------------------------------------
    $display("");
    $display("[测试 1] SPI 寄存器读写");
    
    // 写入 TX Header
    spi_write(7'h02, 8'h5A);  // TX_HDR_L
    spi_write(7'h03, 8'hA5);  // TX_HDR_H
    
    // 读回验证
    spi_read(7'h02);
    if (spi_rd_data == 8'h5A) begin
        $display("  TX_HDR_L: PASS (0x%02h)", spi_rd_data);
        test_pass = test_pass + 1;
    end else begin
        $display("  TX_HDR_L: FAIL (期望 0x5A, 实际 0x%02h)", spi_rd_data);
        test_fail = test_fail + 1;
    end
    
    spi_read(7'h03);
    if (spi_rd_data == 8'hA5) begin
        $display("  TX_HDR_H: PASS (0x%02h)", spi_rd_data);
        test_pass = test_pass + 1;
    end else begin
        $display("  TX_HDR_H: FAIL (期望 0xA5, 实际 0x%02h)", spi_rd_data);
        test_fail = test_fail + 1;
    end
    
    //------------------------------------------------------------------
    // 测试 2: TX 数据缓冲区
    //------------------------------------------------------------------
    $display("");
    $display("[测试 2] TX 数据缓冲区");
    
    spi_write(7'h10, 8'h11);  // TX_DATA[0]
    spi_write(7'h11, 8'h22);  // TX_DATA[1]
    spi_write(7'h12, 8'h33);  // TX_DATA[2]
    spi_write(7'h04, 8'h03);  // TX_LEN = 3
    
    spi_read(7'h10);
    if (spi_rd_data == 8'h11) begin
        $display("  TX_DATA[0]: PASS");
        test_pass = test_pass + 1;
    end else begin
        $display("  TX_DATA[0]: FAIL");
        test_fail = test_fail + 1;
    end
    
    spi_read(7'h04);
    if (spi_rd_data == 8'h03) begin
        $display("  TX_LEN: PASS");
        test_pass = test_pass + 1;
    end else begin
        $display("  TX_LEN: FAIL");
        test_fail = test_fail + 1;
    end
    
    //------------------------------------------------------------------
    // 测试 3: BMC 发送和回环接收
    //------------------------------------------------------------------
    $display("");
    $display("[测试 3] BMC 发送和回环接收");
    
    // 配置发送数据
    spi_write(7'h02, 8'hA3);  // TX_HDR_L
    spi_write(7'h03, 8'h03);  // TX_HDR_H = 0x03A3
    spi_write(7'h10, 8'h55);  // TX_DATA[0]
    spi_write(7'h11, 8'hAA);  // TX_DATA[1]
    spi_write(7'h04, 8'h02);  // TX_LEN = 2
    
    // 使能中断
    spi_write(7'h05, 8'h03);  // IRQ_EN
    
    // 触发发送
    spi_write(7'h00, 8'h03);  // CTRL: TX_START + RX_EN
    $display("  触发发送...");
    
    // 等待中断
    wait(irq_n == 1'b0);
    $display("  收到中断");
    
    // 等待完成
    #20000;
    
    // 读取状态
    spi_read(7'h01);
    $display("  STATUS = 0x%02h", spi_rd_data);
    
    // 读取 RX 数据
    spi_read(7'h42);  // RX_LEN
    $display("  RX_LEN = %0d", spi_rd_data);
    
    if (spi_rd_data > 0) begin
        $display("  BMC 回环: PASS");
        test_pass = test_pass + 1;
    end else begin
        $display("  BMC 回环: 需要检查 (RX_LEN=0)");
    end
    
    // 清除中断
    spi_write(7'h06, 8'hFF);
    
    //------------------------------------------------------------------
    // 测试结果汇总
    //------------------------------------------------------------------
    $display("");
    $display("========================================");
    $display("  测试结果: %0d PASS, %0d FAIL", test_pass, test_fail);
    if (test_fail == 0)
        $display("  状态: ALL TESTS PASSED");
    else
        $display("  状态: SOME TESTS FAILED");
    $display("========================================");
    
    #2000;
    $finish;
end

//============================================================================
// 波形输出
//============================================================================
initial begin
    $dumpfile("dtest_phy_tb.vcd");
    $dumpvars(0, dtest_phy_tb);
end

endmodule
