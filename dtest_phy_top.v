//============================================================================
// dtest_phy_top - USB PD PHY FPGA 顶层模块
// 信号转换层，连接 FPGA 管脚到 dtest_phy 模块
//============================================================================

`timescale 1ns/1ps

module dtest_phy_top (
    // 系统时钟 (差分 200MHz)
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    
    // 复位 (active low)
    input  wire        rst_n,
    
    // I2C 从机接口 (地址 0x28)
    input  wire        i2c_scl,
    inout  wire        i2c_sda,
    
    // 中断输出 (active low)
    output wire        irq_n,
    
    // LED 指示
    output wire        led_heartbeat,
    
    //========================================================================
    // BMC 物理接口 (三态控制由外部处理)
    //========================================================================
    input  wire        bmc_rx,        // BMC 接收输入
    output wire        bmc_tx,        // BMC 发送输出
    output wire        bmc_tx_en,     // BMC 发送使能 (高有效)
    
    //========================================================================
    // CC 比较器接口 (分时复用)
    // en_cc_ref 选择电压阈值，cc1_comp/cc2_comp 返回比较结果
    //========================================================================
    output reg  [2:0]  en_cc_ref,     // 电压档位选择: 001=0.2V, 010=0.4V, ..., 111=2.6V
    input  wire        cc1_comp,      // CC1 > 当前参考电压
    input  wire        cc2_comp,      // CC2 > 当前参考电压
    output  wire        cc1_comp_en,      // CC2 > 当前参考电压
    output  wire        cc2_comp_en,      // CC2 > 当前参考电压
    
    //========================================================================
    // CC 模式输出 (到外部模拟开关/电阻网络)
    //========================================================================
    output wire        cc1_rd_en,      // CC1 Rd 使能
    output wire        cc2_rd_en,      // CC2 Rd 使能
    output wire [1:0]  cc1_rp_en,      // CC1 Rp 使能: 00=无, 01=0.5A, 10=1.5A, 11=3A
    output wire [1:0]  cc2_rp_en,      // CC2 Rp 使能: 00=无, 01=0.5A, 10=1.5A, 11=3A
    
    // 插头方向输出 (互补信号对，控制外部 CC 线路选择)
    output wire        plug_orient_p,  // 正相: 0=CC1通信, 1=CC2通信
    output wire        plug_orient_n,  // 反相: 1=CC1通信, 0=CC2通信
    
    // 调试输出
    output wire        dbg_toggle      // RX 每收到有效数据位翻转一次
);

//============================================================================
// 内部时钟 (从 dtest_phy 层级引用)
//============================================================================
wire clk_50m = u_dtest_phy.clk_50m;

//============================================================================
// CC 比较器扫描逻辑
// 轮询 8 个电压档位 (0~7)，每个档位停留 5us，然后采样并切换
// 电压档位: 000=0V, 001=0.2V, 010=0.4V, 011=0.66V, 100=0.8V, 101=1.23V, 110=1.6V, 111=2.6V
// 扫描周期: 40us (每个档位 5us @ 50MHz)
//============================================================================
localparam SCAN_INTERVAL = 16'd250;  // 每个档位停留时间: 5us @ 50MHz = 250 周期

reg [2:0]  scan_ref;              // 当前扫描的档位 (0~7)
reg [15:0] scan_interval_cnt;     // 1ms 计数器
reg        scan_sample;           // 采样标志
reg [7:0]  cc1_comp_result;       // CC1 各档位比较结果 [7:0] = ref 7~0
reg [7:0]  cc2_comp_result;       // CC2 各档位比较结果
reg [2:0]  cc1_state_reg;         // CC1 状态寄存器
reg [2:0]  cc2_state_reg;         // CC2 状态寄存器

// 输出当前扫描档位
always @(*) begin
    en_cc_ref = scan_ref;
end

// 扫描状态机: 每个档位停留 1ms，然后采样并切换
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        scan_ref <= 3'd0;
        scan_interval_cnt <= 16'd0;
        scan_sample <= 1'b0;
        cc1_comp_result <= 8'b0;
        cc2_comp_result <= 8'b0;
        cc1_state_reg <= 3'b111;  // 默认开路
        cc2_state_reg <= 3'b111;
    end else begin
        scan_sample <= 1'b0;
        
        if (scan_interval_cnt >= SCAN_INTERVAL - 1) begin
            // 1ms 到达，采样当前档位的比较结果
            case (scan_ref)
                3'd0: begin cc1_comp_result[0] <= cc1_comp; cc2_comp_result[0] <= cc2_comp; end
                3'd1: begin cc1_comp_result[1] <= cc1_comp; cc2_comp_result[1] <= cc2_comp; end
                3'd2: begin cc1_comp_result[2] <= cc1_comp; cc2_comp_result[2] <= cc2_comp; end
                3'd3: begin cc1_comp_result[3] <= cc1_comp; cc2_comp_result[3] <= cc2_comp; end
                3'd4: begin cc1_comp_result[4] <= cc1_comp; cc2_comp_result[4] <= cc2_comp; end
                3'd5: begin cc1_comp_result[5] <= cc1_comp; cc2_comp_result[5] <= cc2_comp; end
                3'd6: begin cc1_comp_result[6] <= cc1_comp; cc2_comp_result[6] <= cc2_comp; end
            endcase
            
            // 切换到下一个档位
            if (scan_ref == 3'd6) begin
                scan_ref <= 3'd0;
                scan_sample <= 1'b1;   // 完成一轮扫描 (8个档位)
            end else begin
                scan_ref <= scan_ref + 1'b1;
            end
            scan_interval_cnt <= 16'd0;
        end else begin
            scan_interval_cnt <= scan_interval_cnt + 1'b1;
        end
        
        // 一轮扫描完成后更新 CC 状态
        if (scan_sample) begin
            // CC1 状态编码: 找到最高的有效档位
            // comp_result[7:0] 对应 ref 7~0 (2.6V~0V)
            // 状态 = 最高有效档位
            cc1_state_reg <= cc1_comp_result[6] ? 3'b111 :  // > 2.6V
                             cc1_comp_result[5] ? 3'b110 :  // > 1.6V
                             cc1_comp_result[4] ? 3'b101 :  // > 1.23V
                             cc1_comp_result[3] ? 3'b100 :  // > 0.8V
                             cc1_comp_result[2] ? 3'b011 :  // > 0.66V
                             cc1_comp_result[1] ? 3'b010 :  // > 0.4V
                             cc1_comp_result[0] ? 3'b001 :  // > 0.2V
                                                  3'b000;   // < 0.2V (ref 0)
            
            cc2_state_reg <= cc2_comp_result[6] ? 3'b111 :
                             cc2_comp_result[5] ? 3'b110 :
                             cc2_comp_result[4] ? 3'b101 :
                             cc2_comp_result[3] ? 3'b100 :
                             cc2_comp_result[2] ? 3'b011 :
                             cc2_comp_result[1] ? 3'b010 :
                             cc2_comp_result[0] ? 3'b001 :
                                                  3'b000;
        end
    end
end

// CC 状态输出
wire [2:0] cc1_state = cc1_state_reg;
wire [2:0] cc2_state = cc2_state_reg;

//============================================================================
// CC 模式输出转换
// 3-bit 编码: 000=Open, 001=Rd, 010=Ra, 011=Rp0.5A, 100=Rp1.5A, 101=Rp3A
//============================================================================
wire [2:0] cc1_mode_o;
wire [2:0] cc2_mode_o;

// CC1 Rd 使能: mode=001
assign cc1_rd_en = (cc1_mode_o == 3'b001);
// CC2 Rd 使能: mode=001
assign cc2_rd_en = (cc2_mode_o == 3'b001);

// CC1 Rp 使能: 00=无, 01=Rp0.5A(mode=011), 10=Rp1.5A(mode=100), 11=Rp3A(mode=101)
assign cc1_rp_en = (cc1_mode_o == 3'b011) ? 2'b01 :  // Rp 0.5A
                   (cc1_mode_o == 3'b100) ? 2'b10 :  // Rp 1.5A
                   (cc1_mode_o == 3'b101) ? 2'b11 :  // Rp 3A
                                            2'b00;   // 无/Open/Ra

// CC2 Rp 使能: 00=无, 01=Rp0.5A(mode=011), 10=Rp1.5A(mode=100), 11=Rp3A(mode=101)
assign cc2_rp_en = (cc2_mode_o == 3'b011) ? 2'b01 :  // Rp 0.5A
                   (cc2_mode_o == 3'b100) ? 2'b10 :  // Rp 1.5A
                   (cc2_mode_o == 3'b101) ? 2'b11 :  // Rp 3A
                                            2'b00;   // 无/Open/Ra

//============================================================================
// dtest_phy 实例化
//============================================================================
dtest_phy u_dtest_phy (
    // 系统
    .sys_clk_p      (sys_clk_p),
    .sys_clk_n      (sys_clk_n),
    .rst_n          (rst_n),
    
    // SPI (暂不使用，绑定空闲状态)
    .spi_sclk       (1'b0),
    .spi_cs_n       (1'b1),
    .spi_mosi       (1'b0),
    .spi_miso       (),      // 悬空
    
    // I2C
    .i2c_scl        (i2c_scl),
    .i2c_sda        (i2c_sda),
    
    // 中断
    .irq_n          (irq_n),
    
    // LED
    .led_heartbeat  (led_heartbeat),
    
    // BMC 物理接口
    .bmc_rx         (bmc_rx),
    .bmc_tx         (bmc_tx),
    .bmc_tx_en      (bmc_tx_en),
    
    // CC 状态 (编码后)
    .cc1_state_i    (cc1_state),
    .cc2_state_i    (cc2_state),
    
    // CC 模式输出
    .cc1_mode_o     (cc1_mode_o),
    .cc2_mode_o     (cc2_mode_o),
    
    // 插头方向输出
    .plug_orient_o  (plug_orient_p),
    
    // 调试输出
    .dbg_toggle_o   (dbg_toggle)
);

// 插头方向互补信号
assign plug_orient_n = ~plug_orient_p;

// CC 比较器使能 (强制置1)
assign cc1_comp_en = 1'b1;
assign cc2_comp_en = 1'b1;

endmodule
