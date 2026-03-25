//============================================================================
// USB PD PHY 总模块
// 集成 CC 检测和 BMC 收发器
//============================================================================

`timescale 1ns/1ps

module pd_phy (
    input  wire        clk,
    input  wire        rst_n,
    
    //============================================================================
    // CC 接口
    //============================================================================
    // CC 电压状态输入 (来自外部 ADC 或比较器)
    input  wire [2:0]  cc1_state_i,     // CC1 电压状态 (5种)
    input  wire [2:0]  cc2_state_i,     // CC2 电压状态 (5种)
    
    // CC 模式配置输入 (2-bit 模式 + 2-bit Rp电流)
    input  wire [1:0]  cc1_mode_i,      // CC1 模式: 00=Open, 01=Rd, 10=Ra, 11=Rp
    input  wire [1:0]  cc2_mode_i,      // CC2 模式: 00=Open, 01=Rd, 10=Ra, 11=Rp
    input  wire [1:0]  rp_level_i,      // Rp 电流: 00=0.5A, 01=1.5A, 10=3A
    input  wire        drp_mode_i,      // DRP 模式使能
    
    // CC 控制输出 (到外部模拟开关/电阻网络)
    output wire [2:0]  cc1_mode_o,      // CC1 模式控制
    output wire [2:0]  cc2_mode_o,      // CC2 模式控制
    
    // CC 消抖后状态输出
    output wire [2:0]  cc1_debounced_o, // 消抖后的 CC1 状态
    output wire [2:0]  cc2_debounced_o, // 消抖后的 CC2 状态
    
    // CC 连接中断 (DRP切换过程中不触发)
    output wire        cc_changed_o,    // CC 变化中断
    input  wire        plug_orient_i,   // 插头方向: 0=CC1通信, 1=CC2通信 (外部控制)
    output wire        plug_orient_o,
    
    //============================================================================
    // BMC 收发器接口
    //============================================================================
    // TX 接口
    input  wire [15:0] tx_header_i,
    input  wire [239:0] tx_data_flat_i,
    input  wire [4:0]  tx_data_len_i,
    input  wire        tx_start_i,
    output wire        tx_success_o,    // 发送成功（收到 GoodCRC）
    output wire        tx_fail_o,       // 发送失败（超时）
    
    // RX 接口
    output wire [15:0] rx_header_o,
    output wire [239:0] rx_data_flat_o,
    output wire [4:0]  rx_data_len_o,
    output wire        rx_success_o,    // 接收成功（回复 GoodCRC 完成）
    input  wire        rx_en_i,         // RX 使能控制
    
    // 配置接口
    input  wire [7:0]  header_good_crc, // GoodCRC 配置
    
    // BMC 物理接口 (分离输入输出)
    input  wire        bmc_rx,       // BMC 接收输入
    output wire        bmc_tx,       // BMC 发送输出
    output wire        bmc_tx_en,    // BMC 发送使能
    
    // RX 调试接口
    output wire [2:0]  dbg_rx_state_o,     // RX 状态机 (transceiver 层)
    output wire        dbg_toggle_o        // 每收到有效数据位翻转
);

// 内部 BMC 信号
wire bmc_tx_pad;  // TX 输出
wire bmc_rx_pad;  // RX 输入

// BMC 接口连接
assign bmc_rx_pad = bmc_rx;
assign bmc_tx = bmc_tx_pad;

// 插头方向输出 (直通)
assign plug_orient_o = plug_orient_i;

//============================================================================
// CC 模块实例化
//============================================================================
cc u_cc (
    .clk                (clk),
    .rst_n              (rst_n),
    
    // CC 输入
    .cc1_state_i        (cc1_state_i),
    .cc2_state_i        (cc2_state_i),
    
    // CC 输出
    .cc1_debounced_o    (cc1_debounced_o),
    .cc2_debounced_o    (cc2_debounced_o),
    .cc_changed_o       (cc_changed_o),
    
    // 配置
    .cc1_mode_i         (cc1_mode_i),
    .cc2_mode_i         (cc2_mode_i),
    .rp_level_i         (rp_level_i),
    .drp_mode_i         (drp_mode_i),
    
    // 控制输出
    .cc1_mode_o         (cc1_mode_o),
    .cc2_mode_o         (cc2_mode_o)
);

//============================================================================
// BMC 收发器实例化
//============================================================================
pd_bmc_transceiver u_bmc_transceiver (
    .clk            (clk),
    .rst_n          (rst_n),
    
    // TX 接口
    .tx_header_i    (tx_header_i),
    .tx_data_flat_i (tx_data_flat_i),
    .tx_data_len_i  (tx_data_len_i),
    .tx_start_i     (tx_start_i),
    .tx_success_o   (tx_success_o),
    .tx_fail_o      (tx_fail_o),
    
    // RX 接口
    .rx_header_o    (rx_header_o),
    .rx_data_flat_o (rx_data_flat_o),
    .rx_data_len_o  (rx_data_len_o),
    .rx_success_o   (rx_success_o),
    .rx_en_i        (rx_en_i),
    
    // 配置接口
    .header_good_crc (header_good_crc),
    
    // BMC 物理接口
    .bmc_rx_pad     (bmc_rx_pad),
    .bmc_tx_pad     (bmc_tx_pad),
    .bmc_tx_en      (bmc_tx_en),
    
    // 调试接口
    .dbg_rx_state_o     (dbg_rx_state_o),
    .dbg_toggle_o       (dbg_toggle_o)
);

endmodule
