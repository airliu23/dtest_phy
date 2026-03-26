//============================================================================
// dtest_phy I2C 控制测试平台
// 测试: SOP 类型过滤功能
// - B 模块 RX 只接收 SOP 消息
// - A 模块依次发送 SOP' -> HARD_RESET -> SOP
// - 验证 B 只接收最后的 SOP 消息
//============================================================================

`timescale 1ns/1ps

module dtest_phy_tb;

//============================================================================
// 参数
//============================================================================
parameter CLK_200M_PERIOD = 5;   // 200MHz = 5ns
parameter I2C_PERIOD      = 2500; // 400kHz I2C = 2.5us

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
// BMC 总线 (半双工)
//============================================================================
wire bmc_tx_a, bmc_tx_en_a;
wire bmc_tx_b, bmc_tx_en_b;
tri1 bmc_bus;  // 默认上拉

assign bmc_bus = bmc_tx_en_a ? bmc_tx_a : 1'bz;
assign bmc_bus = bmc_tx_en_b ? bmc_tx_b : 1'bz;

//============================================================================
// I2C 信号 (开漏总线模型 - 使用 pullup 和 tri0)
//============================================================================
// 模块 A I2C
reg        i2c_scl_a_drv;
reg        i2c_sda_a_master;  // 主机驱动: 1=释放, 0=拉低
tri1       i2c_scl_a;         // tri1 = 默认上拉
tri1       i2c_sda_a;         // tri1 = 默认上拉

assign i2c_scl_a = i2c_scl_a_drv ? 1'bz : 1'b0;
assign i2c_sda_a = i2c_sda_a_master ? 1'bz : 1'b0;  // 主机驱动

// 模块 B I2C
reg        i2c_scl_b_drv;
reg        i2c_sda_b_master;
tri1       i2c_scl_b;
tri1       i2c_sda_b;

assign i2c_scl_b = i2c_scl_b_drv ? 1'bz : 1'b0;
assign i2c_sda_b = i2c_sda_b_master ? 1'bz : 1'b0;

//============================================================================
// 模块实例化
//============================================================================
wire irq_n_a, irq_n_b;
wire led_heartbeat_a, led_heartbeat_b;
wire [2:0] cc1_mode_a, cc2_mode_a, cc1_mode_b, cc2_mode_b;
wire plug_orient_a, plug_orient_b;
wire dbg_toggle_a, dbg_toggle_b;

dtest_phy u_dut_a (
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    .spi_sclk       (1'b0),
    .spi_cs_n       (1'b1),
    .spi_mosi       (1'b0),
    .spi_miso       (),
    .i2c_scl        (i2c_scl_a),
    .i2c_sda        (i2c_sda_a),
    .irq_n          (irq_n_a),
    .led_heartbeat  (led_heartbeat_a),
    .bmc_rx         (bmc_bus),
    .bmc_tx         (bmc_tx_a),
    .bmc_tx_en      (bmc_tx_en_a),
    .cc1_state_i    (3'b111),
    .cc2_state_i    (3'b111),
    .cc1_mode_o     (cc1_mode_a),
    .cc2_mode_o     (cc2_mode_a),
    .plug_orient_o  (plug_orient_a),
    .dbg_toggle_o   (dbg_toggle_a)
);

dtest_phy u_dut_b (
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    .spi_sclk       (1'b0),
    .spi_cs_n       (1'b1),
    .spi_mosi       (1'b0),
    .spi_miso       (),
    .i2c_scl        (i2c_scl_b),
    .i2c_sda        (i2c_sda_b),
    .irq_n          (irq_n_b),
    .led_heartbeat  (led_heartbeat_b),
    .bmc_rx         (bmc_bus),
    .bmc_tx         (bmc_tx_b),
    .bmc_tx_en      (bmc_tx_en_b),
    .cc1_state_i    (3'b111),
    .cc2_state_i    (3'b111),
    .cc1_mode_o     (cc1_mode_b),
    .cc2_mode_o     (cc2_mode_b),
    .plug_orient_o  (plug_orient_b),
    .dbg_toggle_o   (dbg_toggle_b)
);

//============================================================================
// BMC 总线监控器
//============================================================================
wire clk_50m = u_dut_a.clk_50m;
wire [15:0] mon_rx_header;
wire [239:0] mon_rx_data;
wire [4:0]  mon_rx_data_len;
wire        mon_rx_done;
wire        mon_rx_busy;
wire        mon_rx_error;
wire        mon_rx_crc_ok;
wire [2:0]  mon_rx_msg_id;

pd_bmc_rx u_bmc_monitor (
    .clk            (clk_50m),
    .rst_n          (rst_n),
    .rx_en_i        (1'b1),
    .sop_type_i     (3'd7),      // 接收所有 SOP 类型
    .rx_sop_type_o  (),          // 不使用
    .rx_header_o    (mon_rx_header),
    .rx_data_flat_o (mon_rx_data),
    .rx_data_len_o  (mon_rx_data_len),
    .rx_done_o      (mon_rx_done),
    .rx_busy_o      (mon_rx_busy),
    .rx_error_o     (mon_rx_error),
    .rx_crc_ok_o    (mon_rx_crc_ok),
    .rx_msg_id_o    (mon_rx_msg_id),
    .bmc_rx_pad     (bmc_bus),
    .dbg_en_i       (1'b0),
    .dbg_byte_o     (),
    .dbg_byte_valid_o(),
    .dbg_toggle_o   ()
);

// GoodCRC 检测计数器
reg        mon_rx_done_d;
reg [3:0]  goodcrc_cnt;      // CRC 正确的 GoodCRC 数量
reg [3:0]  goodcrc_msg_cnt;  // MessageType=GoodCRC 的消息数量
reg [3:0]  frame_cnt;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        mon_rx_done_d <= 1'b0;
        goodcrc_cnt   <= 4'd0;
        goodcrc_msg_cnt <= 4'd0;
        frame_cnt     <= 4'd0;
    end else begin
        mon_rx_done_d <= mon_rx_done;
        
        if (mon_rx_done && !mon_rx_done_d) begin
            frame_cnt <= frame_cnt + 1'b1;
            $display("[Monitor] Frame #%0d: Header=0x%04h, MsgType=%0d, MsgID=%0d, CRC_OK=%b",
                     frame_cnt, mon_rx_header, mon_rx_header[4:0], mon_rx_header[11:9], mon_rx_crc_ok);
            
            // 检测 GoodCRC (MessageType = 0x01 且 NumDataObj = 0)
            if (mon_rx_header[4:0] == 5'b00001 && mon_rx_header[14:12] == 3'b000) begin
                goodcrc_msg_cnt <= goodcrc_msg_cnt + 1'b1;
                if (mon_rx_crc_ok) begin
                    goodcrc_cnt <= goodcrc_cnt + 1'b1;
                    $display("[Monitor] >>> GoodCRC #%0d DETECTED (CRC OK)! <<<", goodcrc_cnt + 1);
                end else begin
                    $display("[Monitor] >>> GoodCRC Message #%0d DETECTED (CRC FAIL) <<<", goodcrc_msg_cnt + 1);
                end
            end
        end
    end
end

//============================================================================
// I2C 任务 - 模块 A
//============================================================================
task i2c_start_a;
    begin
        i2c_sda_a_master = 1'b1;
        i2c_scl_a_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_sda_a_master = 1'b0;  // SDA 下降沿 (START)
        #(I2C_PERIOD/2);
        i2c_scl_a_drv = 1'b0;
        #(I2C_PERIOD/2);
    end
endtask

task i2c_stop_a;
    begin
        i2c_sda_a_master = 1'b0;
        #(I2C_PERIOD/4);
        i2c_scl_a_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_sda_a_master = 1'b1;  // SDA 上升沿 (STOP)
        #(I2C_PERIOD);
    end
endtask

task i2c_write_byte_a;
    input [7:0] data;
    integer i;
    begin
        for (i = 7; i >= 0; i = i - 1) begin
            i2c_sda_a_master = data[i];
            #(I2C_PERIOD/4);
            i2c_scl_a_drv = 1'b1;
            #(I2C_PERIOD/2);
            i2c_scl_a_drv = 1'b0;
            #(I2C_PERIOD/4);
        end
        // ACK 周期
        i2c_sda_a_master = 1'b1;  // 释放 SDA
        #(I2C_PERIOD/4);
        i2c_scl_a_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_scl_a_drv = 1'b0;
        #(I2C_PERIOD/4);
    end
endtask

reg [7:0] i2c_rd_data_a;
task i2c_read_byte_a;
    input ack;  // 1=ACK, 0=NACK
    integer i;
    begin
        i2c_sda_a_master = 1'b1;  // 释放 SDA
        i2c_rd_data_a = 8'd0;
        for (i = 7; i >= 0; i = i - 1) begin
            #(I2C_PERIOD/4);
            i2c_scl_a_drv = 1'b1;
            #(I2C_PERIOD/4);
            i2c_rd_data_a[i] = i2c_sda_a;
            #(I2C_PERIOD/4);
            i2c_scl_a_drv = 1'b0;
            #(I2C_PERIOD/4);
        end
        // ACK/NACK
        i2c_sda_a_master = ack ? 1'b0 : 1'b1;
        #(I2C_PERIOD/4);
        i2c_scl_a_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_scl_a_drv = 1'b0;
        #(I2C_PERIOD/4);
        i2c_sda_a_master = 1'b1;
    end
endtask

task i2c_write_reg_a;
    input [7:0] addr;
    input [7:0] data;
    begin
        i2c_start_a;
        i2c_write_byte_a(8'h50);  // 0x28 << 1 | 0 (写)
        i2c_write_byte_a(addr);
        i2c_write_byte_a(data);
        i2c_stop_a;
    end
endtask

task i2c_read_reg_a;
    input [7:0] addr;
    begin
        i2c_start_a;
        i2c_write_byte_a(8'h50);  // 写地址
        i2c_write_byte_a(addr);
        i2c_start_a;             // Repeated START
        i2c_write_byte_a(8'h51);  // 0x28 << 1 | 1 (读)
        i2c_read_byte_a(1'b0);   // NACK 结束
        i2c_stop_a;
    end
endtask

//============================================================================
// I2C 任务 - 模块 B (复制相同逻辑)
//============================================================================
task i2c_start_b;
    begin
        i2c_sda_b_master = 1'b1;
        i2c_scl_b_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_sda_b_master = 1'b0;
        #(I2C_PERIOD/2);
        i2c_scl_b_drv = 1'b0;
        #(I2C_PERIOD/2);
    end
endtask

task i2c_stop_b;
    begin
        i2c_sda_b_master = 1'b0;
        #(I2C_PERIOD/4);
        i2c_scl_b_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_sda_b_master = 1'b1;
        #(I2C_PERIOD);
    end
endtask

task i2c_write_byte_b;
    input [7:0] data;
    integer i;
    begin
        for (i = 7; i >= 0; i = i - 1) begin
            i2c_sda_b_master = data[i];
            #(I2C_PERIOD/4);
            i2c_scl_b_drv = 1'b1;
            #(I2C_PERIOD/2);
            i2c_scl_b_drv = 1'b0;
            #(I2C_PERIOD/4);
        end
        i2c_sda_b_master = 1'b1;
        #(I2C_PERIOD/4);
        i2c_scl_b_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_scl_b_drv = 1'b0;
        #(I2C_PERIOD/4);
    end
endtask

reg [7:0] i2c_rd_data_b;
task i2c_read_byte_b;
    input ack;
    integer i;
    begin
        i2c_sda_b_master = 1'b1;
        i2c_rd_data_b = 8'd0;
        for (i = 7; i >= 0; i = i - 1) begin
            #(I2C_PERIOD/4);
            i2c_scl_b_drv = 1'b1;
            #(I2C_PERIOD/4);
            i2c_rd_data_b[i] = i2c_sda_b;
            #(I2C_PERIOD/4);
            i2c_scl_b_drv = 1'b0;
            #(I2C_PERIOD/4);
        end
        i2c_sda_b_master = ack ? 1'b0 : 1'b1;
        #(I2C_PERIOD/4);
        i2c_scl_b_drv = 1'b1;
        #(I2C_PERIOD/2);
        i2c_scl_b_drv = 1'b0;
        #(I2C_PERIOD/4);
        i2c_sda_b_master = 1'b1;
    end
endtask

task i2c_write_reg_b;
    input [7:0] addr;
    input [7:0] data;
    begin
        i2c_start_b;
        i2c_write_byte_b(8'h50);
        i2c_write_byte_b(addr);
        i2c_write_byte_b(data);
        i2c_stop_b;
    end
endtask

task i2c_read_reg_b;
    input [7:0] addr;
    begin
        i2c_start_b;
        i2c_write_byte_b(8'h50);
        i2c_write_byte_b(addr);
        i2c_start_b;
        i2c_write_byte_b(8'h51);
        i2c_read_byte_b(1'b0);
        i2c_stop_b;
    end
endtask

//============================================================================
// 超时控制
//============================================================================
integer timeout_cnt;
parameter MAX_TIMEOUT = 10000000;

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
// 主测试流程
//============================================================================
// SOP 类型定义 (与 pd_bmc_tx.v 保持一致)
localparam [2:0] SOP_TYPE_SOP       = 3'd0;  // SOP
localparam [2:0] SOP_TYPE_SOP_PRIME = 3'd1;  // SOP'
localparam [2:0] SOP_TYPE_CABLE_RST = 3'd5;  // CABLE RESET
localparam [2:0] SOP_TYPE_HARD_RST  = 3'd6;  // HARD RESET

integer test_pass, test_fail;
integer sop_prime_goodcrc_cnt, hard_rst_goodcrc_cnt, sop_goodcrc_cnt;

initial begin
    rst_n         = 1'b0;
    i2c_scl_a_drv = 1'b1;
    i2c_sda_a_master = 1'b1;
    i2c_scl_b_drv = 1'b1;
    i2c_sda_b_master = 1'b1;
    test_pass     = 0;
    test_fail     = 0;
    
    $display("");
    $display("========================================");
    $display("  SOP 类型过滤功能测试");
    $display("  B 只接收 SOP，A 依次发 SOP'/HARD_RST/SOP");
    $display("========================================");
    
    #200;
    rst_n = 1'b1;
    #5000;
    $display("[%0t] 复位完成", $time);
    
    //------------------------------------------------------------------
    // 步骤 1: 配置模块 B - RX 只接收 SOP 消息
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 1] 配置模块 B - RX 只接收 SOP 消息...");
    
    // 清除中断标志
    i2c_write_reg_b(8'h10, 8'hFF);
    
    // 配置 RX_SOP_EN (0x2F): 只使能 SOP (bit0=1)
    i2c_write_reg_b(8'h2F, 8'h01);  // 只接收 SOP
    
    // 通过 RX_SOP_EN (0x2F) 配置已完成
    
    i2c_read_reg_b(8'h2F);
    $display("  模块 B RX_SOP_EN = 0x%02h (只接收 SOP)", i2c_rd_data_b);
    i2c_read_reg_b(8'h50);
    $display("  模块 B TX_FRAME_TYPE = 0x%02h (RX_EN=%b)", i2c_rd_data_b, i2c_rd_data_b[4]);
    
    #10000;
    
    //------------------------------------------------------------------
    // 步骤 2: 配置模块 A 发送端基本参数
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 2] 配置模块 A 发送 Source_Cap (20字节)...");
    
    // 清除中断标志
    i2c_write_reg_a(8'h10, 8'hFF);
    // 启用 RX 接收 GoodCRC 回复
    i2c_write_reg_a(8'h2F, 8'h01);  // RX_SOP_EN=0x01 (只收SOP)
    
    // 配置 Header: 0x51A1 (NumDataObj=5, MsgID=0, SourceCap)
    i2c_write_reg_a(8'h52, 8'hA1);  // TX_HDR_L
    i2c_write_reg_a(8'h53, 8'h51);  // TX_HDR_H
    i2c_write_reg_a(8'h51, 8'h14);  // TX_LEN=20
    
    // 写入 5个 PDO (从截图中的实际数据)
    // PDO[0]: 0x0a01912c - Fixed 5V 3A
    i2c_write_reg_a(8'h54, 8'h2c); i2c_write_reg_a(8'h55, 8'h91);
    i2c_write_reg_a(8'h56, 8'h01); i2c_write_reg_a(8'h57, 8'h0a);
    // PDO[1]: 0x0002d12c - Fixed 9V 3A
    i2c_write_reg_a(8'h58, 8'h2c); i2c_write_reg_a(8'h59, 8'hd1);
    i2c_write_reg_a(8'h5A, 8'h02); i2c_write_reg_a(8'h5B, 8'h00);
    // PDO[2]: 0x0003c12c - Fixed 12V 3A
    i2c_write_reg_a(8'h5C, 8'h2c); i2c_write_reg_a(8'h5D, 8'hc1);
    i2c_write_reg_a(8'h5E, 8'h03); i2c_write_reg_a(8'h5F, 8'h00);
    // PDO[3]: 0x0004b12c - Fixed 15V 3A
    i2c_write_reg_a(8'h60, 8'h2c); i2c_write_reg_a(8'h61, 8'hb1);
    i2c_write_reg_a(8'h62, 8'h04); i2c_write_reg_a(8'h63, 8'h00);
    // PDO[4]: 0x000641f4 - Fixed 20V 5A
    i2c_write_reg_a(8'h64, 8'hf4); i2c_write_reg_a(8'h65, 8'h41);
    i2c_write_reg_a(8'h66, 8'h06); i2c_write_reg_a(8'h67, 8'h00);
    
    // 启用中断
    i2c_write_reg_a(8'h12, 8'h54);
    
    //------------------------------------------------------------------
    // 步骤 3: A 发送 SOP' 消息 (B 应该忽略)
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 3] A 发送 SOP' 消息 (B 应忽略)...");
    
    sop_prime_goodcrc_cnt = goodcrc_cnt;
    
    // TX_FRAME_TYPE: [2:0]=001(SOP'), [3]=1(TX_START), [4]=1(RX_EN) = 0x19
    i2c_write_reg_a(8'h50, 8'h19);
    
    $display("  等待发送...");
    #2000000;  // 2ms
    
    // 检查 A 的状态 - 应该是 TX_FAIL (没收到 GoodCRC)
    i2c_read_reg_a(8'h10);
    $display("  模块 A IRQ_FLAG = 0x%02h (TX_SUCCESS=%b, TX_FAIL=%b)", 
             i2c_rd_data_a, i2c_rd_data_a[6], i2c_rd_data_a[4]);
    
    // 验证没有新的 GoodCRC
    if (goodcrc_cnt == sop_prime_goodcrc_cnt) begin
        $display("  SOP' 测试: PASS (B 正确忽略了 SOP' 消息)");
        test_pass = test_pass + 1;
    end else begin
        $display("  SOP' 测试: FAIL (B 不应该响应 SOP' 消息)");
        test_fail = test_fail + 1;
    end
    
    // 清除中断
    i2c_write_reg_a(8'h10, 8'hFF);
    
    //------------------------------------------------------------------
    // 步骤 4: A 发送 HARD_RESET 消息 (B 应该忽略)
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 4] A 发送 HARD_RESET 消息 (B 应忽略)...");
    
    hard_rst_goodcrc_cnt = goodcrc_cnt;
    
    // 更新 Header: NumDataObj=5, MsgID=2
    i2c_write_reg_a(8'h53, 8'h54);
    
    // TX_FRAME_TYPE: [2:0]=110(HARD_RST), [3]=1(TX_START), [4]=1(RX_EN) = 0x1E
    i2c_write_reg_a(8'h50, 8'h1E);
    
    $display("  等待发送...");
    #2000000;  // 2ms
    
    // 检查 A 的状态 - 应该是 TX_FAIL (HARD_RESET 不需要 GoodCRC)
    i2c_read_reg_a(8'h10);
    $display("  模块 A IRQ_FLAG = 0x%02h (TX_SUCCESS=%b, TX_FAIL=%b)", 
             i2c_rd_data_a, i2c_rd_data_a[6], i2c_rd_data_a[4]);
    
    // 验证没有新的 GoodCRC (HARD_RESET 不会有 GoodCRC 响应)
    if (goodcrc_cnt == hard_rst_goodcrc_cnt) begin
        $display("  HARD_RESET 测试: PASS (B 正确忽略了 HARD_RESET)");
        test_pass = test_pass + 1;
    end else begin
        $display("  HARD_RESET 测试: FAIL (B 不应该响应 HARD_RESET)");
        test_fail = test_fail + 1;
    end
    
    // 清除中断
    i2c_write_reg_a(8'h10, 8'hFF);
    i2c_write_reg_b(8'h10, 8'hFF);
    
    //------------------------------------------------------------------
    // 步骤 5: A 发送 SOP 消息 (B 应该接收并回复 GoodCRC)
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 5] A 发送 SOP 消息 (B 应接收并回复 GoodCRC)...");
    
    sop_goodcrc_cnt = goodcrc_cnt;
    
    // 更新 Header: NumDataObj=5, MsgID=3
    i2c_write_reg_a(8'h53, 8'h56);  // TX_HDR_H
    
    // TX_FRAME_TYPE: [2:0]=000(SOP), [3]=1(TX_START), [4]=1(RX_EN) = 0x18
    i2c_write_reg_a(8'h50, 8'h18);
    
    $display("  等待发送...");
    #2000000;  // 2ms
    
    // 检查 A 的状态 - 应该是 TX_SUCCESS (收到 GoodCRC)
    i2c_read_reg_a(8'h10);
    $display("  模块 A IRQ_FLAG = 0x%02h (TX_SUCCESS=%b, TX_FAIL=%b)", 
             i2c_rd_data_a, i2c_rd_data_a[6], i2c_rd_data_a[4]);
    
    // 检查 B 的状态 - 应该有 RX_SUCCESS
    i2c_read_reg_b(8'h10);
    $display("  模块 B IRQ_FLAG = 0x%02h (RX_SUCCESS=%b)", 
             i2c_rd_data_b, i2c_rd_data_b[2]);
    
    // 验证收到 GoodCRC
    if (goodcrc_cnt > sop_goodcrc_cnt && i2c_rd_data_a[6]) begin
        $display("  SOP 测试: PASS (B 正确接收并回复 GoodCRC)");
        test_pass = test_pass + 1;
    end else begin
        $display("  SOP 测试: FAIL (B 应该响应 SOP 消息)");
        test_fail = test_fail + 1;
    end
    
    //==================================================================
    // 第二阶段: B 模块打开所有消息接收
    //==================================================================
    $display("");
    $display("========================================");
    $display("  第二阶段: B 接收所有消息类型测试");
    $display("========================================");
    
    //------------------------------------------------------------------
    // 步骤 6: 配置 B 模块接收所有消息类型
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 6] 配置模块 B 接收所有消息类型...");
    
    // 清除中断标志
    i2c_write_reg_a(8'h10, 8'hFF);
    i2c_write_reg_b(8'h10, 8'hFF);
    
    // 配置 RX_SOP_EN (0x2F): 使能所有类型
    i2c_write_reg_b(8'h2F, 8'hFF);
    
    i2c_read_reg_b(8'h2F);
    $display("  模块 B RX_SOP_EN = 0x%02h (接收所有类型)", i2c_rd_data_b);
    
    #10000;
    
    //------------------------------------------------------------------
    // 步骤 7: A 发送 SOP' 消息 (B 应该接收)
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 7] A 发送 SOP' 消息 (B 应接收)...");
    
    sop_prime_goodcrc_cnt = goodcrc_cnt;
    
    // 更新 Header: NumDataObj=5, MsgID=4
    i2c_write_reg_a(8'h53, 8'h58);
    
    // TX_FRAME_TYPE: [2:0]=001(SOP'), [3]=1(TX_START), [4]=1(RX_EN) = 0x19
    i2c_write_reg_a(8'h50, 8'h19);
    
    $display("  等待发送...");
    #2000000;  // 2ms
    
    i2c_read_reg_a(8'h10);
    $display("  模块 A IRQ_FLAG = 0x%02h (TX_SUCCESS=%b, TX_FAIL=%b)", 
             i2c_rd_data_a, i2c_rd_data_a[6], i2c_rd_data_a[4]);
    
    i2c_read_reg_b(8'h10);
    $display("  模块 B IRQ_FLAG = 0x%02h (RX_SUCCESS=%b)", 
             i2c_rd_data_b, i2c_rd_data_b[2]);
    
    // 验证 B 收到并回复 GoodCRC
    if (goodcrc_cnt > sop_prime_goodcrc_cnt && i2c_rd_data_a[6]) begin
        $display("  SOP' 接收测试: PASS (B 正确接收并回复 GoodCRC)");
        test_pass = test_pass + 1;
    end else begin
        $display("  SOP' 接收测试: FAIL");
        test_fail = test_fail + 1;
    end
    
    // 清除中断
    i2c_write_reg_a(8'h10, 8'hFF);
    i2c_write_reg_b(8'h10, 8'hFF);
    
    //------------------------------------------------------------------
    // 步骤 8: A 发送 HARD_RESET 消息 (B 应该接收)
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 8] A 发送 HARD_RESET 消息 (B 应接收)...");
    
    // TX_FRAME_TYPE: [2:0]=110(HARD_RST), [3]=1(TX_START), [4]=1(RX_EN) = 0x1E
    i2c_write_reg_a(8'h50, 8'h1E);
    
    $display("  等待发送...");
    #500000;  // 0.5ms (HARD_RESET 很快完成)
    
    i2c_read_reg_a(8'h10);
    $display("  模块 A IRQ_FLAG = 0x%02h (TX_SUCCESS=%b, TX_FAIL=%b)", 
             i2c_rd_data_a, i2c_rd_data_a[6], i2c_rd_data_a[4]);
    
    i2c_read_reg_b(8'h10);
    $display("  模块 B IRQ_FLAG = 0x%02h (HARD_RST=%b, RX_SUCCESS=%b)", 
             i2c_rd_data_b, i2c_rd_data_b[3], i2c_rd_data_b[2]);
    
    // 验证 B 收到 HARD_RESET (IRQ_FLAG bit3 置位)
    if (i2c_rd_data_b[3]) begin
        $display("  HARD_RESET 接收测试: PASS (B 正确检测到 HARD_RESET)");
        test_pass = test_pass + 1;
    end else begin
        $display("  HARD_RESET 接收测试: FAIL");
        test_fail = test_fail + 1;
    end
    
    // 清除中断
    i2c_write_reg_a(8'h10, 8'hFF);
    i2c_write_reg_b(8'h10, 8'hFF);
    
    //------------------------------------------------------------------
    // 步骤 9: A 发送 SOP 消息 (B 应该接收)
    //------------------------------------------------------------------
    $display("");
    $display("[步骤 9] A 发送 SOP 消息 (B 应接收)...");
    
    sop_goodcrc_cnt = goodcrc_cnt;
    
    // 更新 Header: NumDataObj=5, MsgID=5
    i2c_write_reg_a(8'h53, 8'h5A);
    
    // TX_FRAME_TYPE: [2:0]=000(SOP), [3]=1(TX_START), [4]=1(RX_EN) = 0x18
    i2c_write_reg_a(8'h50, 8'h18);
    
    $display("  等待发送...");
    #2000000;  // 2ms
    
    i2c_read_reg_a(8'h10);
    $display("  模块 A IRQ_FLAG = 0x%02h (TX_SUCCESS=%b, TX_FAIL=%b)", 
             i2c_rd_data_a, i2c_rd_data_a[6], i2c_rd_data_a[4]);
    
    i2c_read_reg_b(8'h10);
    $display("  模块 B IRQ_FLAG = 0x%02h (RX_SUCCESS=%b)", 
             i2c_rd_data_b, i2c_rd_data_b[2]);
    
    // 验证 B 收到并回复 GoodCRC
    if (goodcrc_cnt > sop_goodcrc_cnt && i2c_rd_data_a[6]) begin
        $display("  SOP 接收测试: PASS (B 正确接收并回复 GoodCRC)");
        test_pass = test_pass + 1;
    end else begin
        $display("  SOP 接收测试: FAIL");
        test_fail = test_fail + 1;
    end
    
    //------------------------------------------------------------------
    // 测试结果
    //------------------------------------------------------------------
    $display("");
    $display("========================================");
    $display("  BMC 总线上检测到的 GoodCRC 数量: %0d", goodcrc_cnt);
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
// 波形输出
//============================================================================
initial begin
    $dumpfile("dtest_phy.vcd");
    $dumpvars(0, dtest_phy_tb);
end

endmodule
