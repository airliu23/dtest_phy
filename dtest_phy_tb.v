//============================================================================
// dtest_phy 握手测试平台
// 测试自动 GoodCRC 握手功能
// 模块 A 发送数据并等待 ACK，模块 B 接收后自动发送 GoodCRC
//============================================================================

`timescale 1ns/1ps

module dtest_phy_tb;

//============================================================================
// 参数
//============================================================================
parameter CLK_200M_PERIOD = 5;   // 200MHz = 5ns
parameter SPI_CLK_PERIOD  = 100; // 10MHz SPI

//============================================================================
// 时钟和复位
//============================================================================
reg  sys_clk_p;
wire sys_clk_n;
assign sys_clk_n = ~sys_clk_p;
reg rst_n;

initial begin
    sys_clk_p = 1'b0;
    forever #2.5 sys_clk_p = ~sys_clk_p;
end

//============================================================================
// 模块 A 信号 (发送端)
//============================================================================
reg  spi_sclk_a;
reg  spi_cs_n_a;
reg  spi_mosi_a;
wire spi_miso_a;
wire irq_n_a;
wire led_tx_a, led_rx_ok_a, led_rx_err_a, led_heartbeat_a;

//============================================================================
// BMC 总线 (半双工，双向)
// 两个模块的 bmc_pad 连接到同一根总线
//============================================================================
wire bmc_bus;

//============================================================================
// BMC 总线监控 - 使用 pd_bmc_rx 模块
//============================================================================

// 获取50MHz时钟
wire clk_50m = u_dut_a.clk_50m;

// 监控器输出
wire [15:0] mon_rx_header;
wire [239:0] mon_rx_data;
wire [4:0]  mon_rx_data_len;
wire        mon_rx_done;
wire        mon_rx_busy;
wire        mon_rx_error;
wire        mon_rx_crc_ok;
wire [2:0]  mon_rx_msg_id;

// 调试接口信号
wire [7:0] mon_dbg_byte;
wire       mon_dbg_byte_valid;

// 实例化 pd_bmc_rx 作为监控器
pd_bmc_rx u_bmc_monitor (
    .clk            (clk_50m),
    .rst_n          (rst_n),
    .rx_en_i        (1'b1),           // 始终使能
    .rx_header_o    (mon_rx_header),
    .rx_data_flat_o (mon_rx_data),
    .rx_data_len_o  (mon_rx_data_len),
    .rx_done_o      (mon_rx_done),
    .rx_busy_o      (mon_rx_busy),
    .rx_error_o     (mon_rx_error),
    .rx_crc_ok_o    (mon_rx_crc_ok),
    .rx_msg_id_o    (mon_rx_msg_id),
    .bmc_rx_pad     (bmc_bus),        // 连接到总线
    .dbg_en_i       (1'b1),           // 调试使能
    .dbg_byte_o     (mon_dbg_byte),   // 实时字节输出
    .dbg_byte_valid_o(mon_dbg_byte_valid)
);

// 监控逻辑 - 打印接收到的数据
reg        mon_rx_done_d;
reg [15:0] mon_frame_cnt;
reg [4:0]  mon_byte_cnt;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        mon_rx_done_d <= 1'b0;
        mon_frame_cnt <= 16'd0;
        mon_byte_cnt  <= 5'd0;
    end else begin
        mon_rx_done_d <= mon_rx_done;
        
        // 实时打印解析的byte
        if (mon_dbg_byte_valid) begin
            $display("[BusMonitor] [%0t] Real-time Byte[%0d] = 0x%02h", 
                     $time, mon_byte_cnt, mon_dbg_byte);
            mon_byte_cnt <= mon_byte_cnt + 1'b1;
        end
        
        // 检测 rx_done 上升沿
        if (mon_rx_done && !mon_rx_done_d) begin
            $display("[BusMonitor] [%0t] ==================== Frame #%0d Complete ====================", 
                     $time, mon_frame_cnt);
            $display("[BusMonitor] Header: 0x%04h (MsgID=%0d)", mon_rx_header, mon_rx_header[4:0]);
            $display("[BusMonitor] Data Len: %0d bytes", mon_rx_data_len);
            $display("[BusMonitor] CRC OK: %b", mon_rx_crc_ok);
            
            if (mon_rx_data_len > 0) begin
                $write("[BusMonitor] Data:");
                for (i = 0; i < mon_rx_data_len && i < 30; i = i + 1) begin
                    $write(" %02h", mon_rx_data[i*8 +: 8]);
                end
                $display("");
            end
            
            // 解析 USB PD Header
            $display("[BusMonitor] USB PD Parse:");
            $display("[BusMonitor]   Type: 0x%02h (%s)", mon_rx_header[14:12], 
                     (mon_rx_header[14:12] == 3'b001) ? "GoodCRC" : 
                     (mon_rx_header[14:12] == 3'b000) ? "Control" : "Data");
            $display("[BusMonitor]   MsgID: %0d", mon_rx_header[4:0]);
            $display("[BusMonitor] ===========================================================");
            
            mon_frame_cnt <= mon_frame_cnt + 1'b1;
            mon_byte_cnt  <= 5'd0;  // 重置字节计数
        end
    end
end

//============================================================================
// 模块 B 信号 (接收端)
//============================================================================
reg  spi_sclk_b;
reg  spi_cs_n_b;
reg  spi_mosi_b;
wire spi_miso_b;
wire irq_n_b;
wire led_tx_b, led_rx_ok_b, led_rx_err_b, led_heartbeat_b;

//============================================================================
// 超时控制
//============================================================================
integer timeout_cnt;
parameter MAX_TIMEOUT = 50000000;

always @(posedge sys_clk_p or negedge rst_n) begin
    if (!rst_n) begin
        timeout_cnt <= 0;
    end else begin
        timeout_cnt <= timeout_cnt + 1;
        if (timeout_cnt >= MAX_TIMEOUT) begin
            $display("[%0t] ERROR: 超时!", $time);
            $finish;
        end
    end
end

//============================================================================
// 模块 A 实例化 (发送端)
//============================================================================
dtest_phy u_dut_a (
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    .spi_sclk       (spi_sclk_a),
    .spi_cs_n       (spi_cs_n_a),
    .spi_mosi       (spi_mosi_a),
    .spi_miso       (spi_miso_a),
    .irq_n          (irq_n_a),
    .led_tx         (led_tx_a),
    .led_rx_ok      (led_rx_ok_a),
    .led_rx_err     (led_rx_err_a),
    .led_heartbeat  (led_heartbeat_a),
    .bmc_pad        (bmc_bus)
);

//============================================================================
// 模块 B 实例化 (接收端)
//============================================================================
dtest_phy u_dut_b (
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    .spi_sclk       (spi_sclk_b),
    .spi_cs_n       (spi_cs_n_b),
    .spi_mosi       (spi_mosi_b),
    .spi_miso       (spi_miso_b),
    .irq_n          (irq_n_b),
    .led_tx         (led_tx_b),
    .led_rx_ok      (led_rx_ok_b),
    .led_rx_err     (led_rx_err_b),
    .led_heartbeat  (led_heartbeat_b),
    .bmc_pad        (bmc_bus)
);

//============================================================================
// SPI 任务 - 模块 A
//============================================================================
localparam [14:0] MODULE_ID_PD = 15'h0001;
reg [7:0] spi_rd_data_a;

task spi_write_a;
    input [14:0] module_id;
    input [15:0] reg_addr;
    input [7:0]  data;
    reg [31:0] addr32;
    integer i;
    begin
        addr32 = {1'b1, module_id, reg_addr};
        spi_cs_n_a = 1'b0;
        #(SPI_CLK_PERIOD);
        for (i = 31; i >= 0; i = i - 1) begin
            spi_sclk_a = 1'b0;
            spi_mosi_a = addr32[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk_a = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk_a = 1'b0;
            spi_mosi_a = data[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk_a = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        spi_sclk_a = 1'b0;
        #(SPI_CLK_PERIOD);
        spi_cs_n_a = 1'b1;
        #(SPI_CLK_PERIOD);
    end
endtask

task spi_read_a;
    input [14:0] module_id;
    input [15:0] reg_addr;
    reg [31:0] addr32;
    integer i;
    begin
        addr32 = {1'b0, module_id, reg_addr};
        spi_rd_data_a = 8'd0;
        spi_cs_n_a = 1'b0;
        #(SPI_CLK_PERIOD);
        for (i = 31; i >= 0; i = i - 1) begin
            spi_sclk_a = 1'b0;
            spi_mosi_a = addr32[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk_a = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk_a = 1'b0;
            spi_mosi_a = 1'b0;
            #(SPI_CLK_PERIOD/2);
            spi_sclk_a = 1'b1;
            spi_rd_data_a[i] = spi_miso_a;
            #(SPI_CLK_PERIOD/2);
        end
        spi_sclk_a = 1'b0;
        #(SPI_CLK_PERIOD);
        spi_cs_n_a = 1'b1;
        #(SPI_CLK_PERIOD);
    end
endtask

task pd_write_a;
    input [7:0] addr;
    input [7:0] data;
    begin
        spi_write_a(MODULE_ID_PD, {8'h00, addr}, data);
    end
endtask

task pd_read_a;
    input [7:0] addr;
    begin
        spi_read_a(MODULE_ID_PD, {8'h00, addr});
    end
endtask

//============================================================================
// SPI 任务 - 模块 B
//============================================================================
reg [7:0] spi_rd_data_b;

task spi_write_b;
    input [14:0] module_id;
    input [15:0] reg_addr;
    input [7:0]  data;
    reg [31:0] addr32;
    integer i;
    begin
        addr32 = {1'b1, module_id, reg_addr};
        spi_cs_n_b = 1'b0;
        #(SPI_CLK_PERIOD);
        for (i = 31; i >= 0; i = i - 1) begin
            spi_sclk_b = 1'b0;
            spi_mosi_b = addr32[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk_b = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk_b = 1'b0;
            spi_mosi_b = data[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk_b = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        spi_sclk_b = 1'b0;
        #(SPI_CLK_PERIOD);
        spi_cs_n_b = 1'b1;
        #(SPI_CLK_PERIOD);
    end
endtask

task spi_read_b;
    input [14:0] module_id;
    input [15:0] reg_addr;
    reg [31:0] addr32;
    integer i;
    begin
        addr32 = {1'b0, module_id, reg_addr};
        spi_rd_data_b = 8'd0;
        spi_cs_n_b = 1'b0;
        #(SPI_CLK_PERIOD);
        for (i = 31; i >= 0; i = i - 1) begin
            spi_sclk_b = 1'b0;
            spi_mosi_b = addr32[i];
            #(SPI_CLK_PERIOD/2);
            spi_sclk_b = 1'b1;
            #(SPI_CLK_PERIOD/2);
        end
        for (i = 7; i >= 0; i = i - 1) begin
            spi_sclk_b = 1'b0;
            spi_mosi_b = 1'b0;
            #(SPI_CLK_PERIOD/2);
            spi_sclk_b = 1'b1;
            spi_rd_data_b[i] = spi_miso_b;
            #(SPI_CLK_PERIOD/2);
        end
        spi_sclk_b = 1'b0;
        #(SPI_CLK_PERIOD);
        spi_cs_n_b = 1'b1;
        #(SPI_CLK_PERIOD);
    end
endtask

task pd_write_b;
    input [7:0] addr;
    input [7:0] data;
    begin
        spi_write_b(MODULE_ID_PD, {8'h00, addr}, data);
    end
endtask

task pd_read_b;
    input [7:0] addr;
    begin
        spi_read_b(MODULE_ID_PD, {8'h00, addr});
    end
endtask

//============================================================================
// 主测试
//============================================================================
integer test_pass, test_fail;
integer i;

// 测试数据
reg [15:0] test_header;
reg [7:0]  test_data [0:7];
reg [4:0]  test_data_len;

initial begin
    rst_n       = 1'b0;
    spi_sclk_a  = 1'b0;
    spi_cs_n_a  = 1'b1;
    spi_mosi_a  = 1'b0;
    spi_sclk_b  = 1'b0;
    spi_cs_n_b  = 1'b1;
    spi_mosi_b  = 1'b0;
    test_pass   = 0;
    test_fail   = 0;
    
    // 初始化测试数据
    test_header   = 16'h03A3;
    test_data[0]  = 8'hAA;
    test_data[1]  = 8'h55;
    test_data[2]  = 8'h12;
    test_data[3]  = 8'h34;
    test_data_len = 5'd0;  // 发送4字节数据
    
    $display("");
    $display("========================================");
    $display("  dtest_phy GoodCRC 握手测试");
    $display("========================================");
    
    #200;
    rst_n = 1'b1;
    #1000;
    $display("[%0t] 复位完成", $time);
    
    //------------------------------------------------------------------
    // 配置模块 B (接收端) - 启用 RX 和自动 GoodCRC
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 1] 配置模块 B (接收端)...");
    
    // 清除中断标志
    pd_write_b(8'h06, 8'hFF);
    // 启用 RX 和 auto_goodcrc (bit1=RX_EN, bit2=AUTO_GOODCRC)
    pd_write_b(8'h00, 8'h06);  // CTRL: RX_EN=1, AUTO_GOODCRC=1
    
    pd_read_b(8'h00);  // 读取 CTRL 确认
    $display("  模块 B CTRL = 0x%02h (RX_EN=%b AUTO_GOODCRC=%b)", 
             spi_rd_data_b, spi_rd_data_b[1], spi_rd_data_b[2]);
    $display("  模块 B 已启用 RX 和自动 GoodCRC");
    
    #10000;
    
    //------------------------------------------------------------------
    // 配置模块 A (发送端) - 启用 auto_goodcrc 等待 ACK
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 2] 配置模块 A (发送端)...");
    
    // 清除中断标志
    pd_write_a(8'h06, 8'hFF);
    // 启用 RX 和 auto_goodcrc (发送后等待 ACK，需要 RX 接收 ACK)
    pd_write_a(8'h00, 8'h06);  // CTRL: RX_EN=1, AUTO_GOODCRC=1
    
    // 配置 Header (包含 MessageID)
    pd_write_a(8'h02, test_header[7:0]);   // TX_HDR_L
    pd_write_a(8'h03, test_header[15:8]);  // TX_HDR_H
    
    // 配置数据
    for (i = 0; i < test_data_len; i = i + 1) begin
        pd_write_a(8'h10 + i, test_data[i]);
    end
    
    pd_write_a(8'h04, test_data_len);  // TX_LEN
    pd_write_a(8'h05, 8'h03);          // IRQ_EN: TX_DONE_IE=1, TX_FAIL_IE=1
    
    $display("  Header: 0x%04h (MsgID=%0d)", test_header, test_header[4:0]);
    $display("  Data[%0d]: %02h %02h %02h %02h", test_data_len,
             test_data[0], test_data[1], test_data[2], test_data[3]);
    
    //------------------------------------------------------------------
    // 启动 A 发送，等待 ACK
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 3] 启动模块 A 发送 (等待 GoodCRC ACK)...");
    // 启动发送，启用 auto_goodcrc (bit0=TX_START, bit2=AUTO_GOODCRC)
    pd_write_a(8'h00, 8'h05);  // CTRL: TX_START=1, AUTO_GOODCRC=1
    
    // 等待发送完成或失败
    $display("  等待发送完成或 ACK 超时...");
    #3000000;  // 3ms 等待
    
    //------------------------------------------------------------------
    // 检查 A 的发送状态
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 4] 检查模块 A 发送状态...");
    
    // 多次读取状态，等待 TX 完成
    for (i = 0; i < 10; i = i + 1) begin
        pd_read_a(8'h06);  // IRQ_FLAG
        $display("  [%0t] 模块 A IRQ_FLAG = 0x%02h, STATUS = ?", $time, spi_rd_data_a);
        pd_read_a(8'h01);  // STATUS
        $display("  [%0t] 模块 A STATUS = 0x%02h (TX_BUSY=%b RX_BUSY=%b)", 
                 $time, spi_rd_data_a, spi_rd_data_a[0], spi_rd_data_a[3]);
        #500000;
    end
    
    if (spi_rd_data_a[0]) begin  // TX_DONE_IF
        $display("  模块 A 发送成功 (收到 GoodCRC ACK): PASS");
        test_pass = test_pass + 1;
    end else if (spi_rd_data_a[1]) begin  // TX_FAIL_IF
        $display("  模块 A 发送失败 (未收到 ACK): FAIL");
        test_fail = test_fail + 1;
    end else begin
        $display("  模块 A 状态未知: FAIL");
        test_fail = test_fail + 1;
    end
    
    pd_read_a(8'h01);  // STATUS
    $display("  模块 A STATUS = 0x%02h", spi_rd_data_a);
    
    //------------------------------------------------------------------
    // 检查 B 的接收状态
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 5] 检查模块 B 接收状态...");
    
    // 多次读取状态，观察 B 的 TX 状态
    for (i = 0; i < 10; i = i + 1) begin
        pd_read_b(8'h06);  // IRQ_FLAG
        pd_read_b(8'h01);  // STATUS
        $display("  [%0t] 模块 B STATUS = 0x%02h (TX_BUSY=%b RX_BUSY=%b RX_CRC_OK=%b)", 
                 $time, spi_rd_data_b, spi_rd_data_b[0], spi_rd_data_b[3], spi_rd_data_b[5]);
        #100000;
    end
    
    pd_read_b(8'h06);  // IRQ_FLAG
    $display("  模块 B IRQ_FLAG = 0x%02h", spi_rd_data_b);
    
    if (spi_rd_data_b[2]) begin  // RX_DONE_IF
        $display("  模块 B 接收完成: PASS");
        test_pass = test_pass + 1;
    end else begin
        $display("  模块 B 接收完成: FAIL");
        test_fail = test_fail + 1;
    end
    
    // 读取接收到的 Header
    pd_read_b(8'h40);  // RX_HDR_L
    $display("  模块 B RX_HDR_L = 0x%02h", spi_rd_data_b);
    
    pd_read_b(8'h41);  // RX_HDR_H
    $display("  模块 B RX_HDR_H = 0x%02h", spi_rd_data_b);
    
    // 读取接收长度
    pd_read_b(8'h42);  // RX_LEN
    $display("  模块 B RX_LEN = %0d", spi_rd_data_b[4:0]);
    
    // 读取接收数据
    $display("  模块 B 接收数据:");
    for (i = 0; i < test_data_len; i = i + 1) begin
        pd_read_b(8'h50 + i);
        $display("    RX_DATA[%0d] = 0x%02h (期望: 0x%02h) %s", 
                 i, spi_rd_data_b, test_data[i],
                 (spi_rd_data_b == test_data[i]) ? "OK" : "MISMATCH");
        if (spi_rd_data_b == test_data[i])
            test_pass = test_pass + 1;
        else
            test_fail = test_fail + 1;
    end
    
    // 检查 CRC
    pd_read_b(8'h01);  // STATUS
    $display("");
    $display("  模块 B STATUS = 0x%02h", spi_rd_data_b);
    $display("  RX_CRC_OK = %b", spi_rd_data_b[5]);
    
    if (spi_rd_data_b[5]) begin
        $display("  CRC 校验: PASS");
        test_pass = test_pass + 1;
    end else begin
        $display("  CRC 校验: FAIL");
        test_fail = test_fail + 1;
    end
    
    //------------------------------------------------------------------
    // 测试结果
    //------------------------------------------------------------------
    $display("");
    $display("========================================");
    $display("  结果: %0d PASS, %0d FAIL", test_pass, test_fail);
    if (test_fail == 0)
        $display("  状态: ALL TESTS PASSED");
    else
        $display("  状态: SOME TESTS FAILED");
    $display("========================================");
    
    #5000;
    $finish;
end

//============================================================================
// 波形
//============================================================================
initial begin
    $dumpfile("dtest_phy.vcd");
    $dumpvars(0, dtest_phy_tb);
end

endmodule
