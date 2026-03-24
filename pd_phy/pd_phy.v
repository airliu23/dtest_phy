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
    
    // CC 模式配置输入
    input  wire [2:0]  cc1_mode_i,      // CC1 模式配置 (6种)
    input  wire [2:0]  cc2_mode_i,      // CC2 模式配置 (6种)
    input  wire        drp_mode_i,      // DRP 模式使能
    
    // CC 控制输出 (到外部模拟开关/电阻网络)
    output wire [2:0]  cc1_mode_o,      // CC1 模式控制
    output wire [2:0]  cc2_mode_o,      // CC2 模式控制
    
    // CC 消抖后状态输出
    output wire [2:0]  cc1_debounced_o, // 消抖后的 CC1 状态
    output wire [2:0]  cc2_debounced_o, // 消抖后的 CC2 状态
    
    // CC 连接中断 (DRP切换过程中不触发)
    output wire        cc_changed_o,    // CC 变化中断
    
    //============================================================================
    // VBUS 接口
    //============================================================================
    input  wire [1:0]  vbus_state_i,    // VBUS 电压状态 (3种)
    output wire [1:0]  vbus_debounced_o,// 消抖后的 VBUS 状态
    
    //============================================================================
    // BMC 收发器接口
    //============================================================================
    // TX 接口
    input  wire [15:0] tx_header_i,
    input  wire [239:0] tx_data_flat_i,
    input  wire [4:0]  tx_data_len_i,
    input  wire        tx_start_i,
    output wire        tx_done_o,
    output wire        tx_busy_o,
    output wire        tx_active_o,
    
    // RX 接口
    output wire [15:0] rx_header_o,
    output wire [239:0] rx_data_flat_o,
    output wire [4:0]  rx_data_len_o,
    output wire        rx_done_o,
    output wire        rx_busy_o,
    output wire        rx_error_o,
    output wire        rx_crc_ok_o,
    input  wire        rx_en_i,
    
    // 配置接口
    input  wire [7:0]  header_good_crc, // GoodCRC 配置
    input  wire [2:0]  msg_id_i,
    input  wire        auto_goodcrc_i,
    input  wire        loopback_mode_i,
    output wire        tx_fail_o,
    
    // BMC 物理接口
    inout  wire        bmc_pad,
    output wire        bmc_pad_en
);

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
    
    // VBUS
    .vbus_state_i       (vbus_state_i),
    .vbus_debounced_o   (vbus_debounced_o),
    
    // 配置
    .cc1_mode_i         (cc1_mode_i),
    .cc2_mode_i         (cc2_mode_i),
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
    .tx_done_o      (tx_done_o),
    .tx_busy_o      (tx_busy_o),
    .tx_active_o    (tx_active_o),
    
    // RX 接口
    .rx_header_o    (rx_header_o),
    .rx_data_flat_o (rx_data_flat_o),
    .rx_data_len_o  (rx_data_len_o),
    .rx_done_o      (rx_done_o),
    .rx_busy_o      (rx_busy_o),
    .rx_error_o     (rx_error_o),
    .rx_crc_ok_o    (rx_crc_ok_o),
    .rx_en_i        (rx_en_i),
    
    // 配置接口
    .header_good_crc (header_good_crc),
    .msg_id_i       (msg_id_i),
    .auto_goodcrc_i (auto_goodcrc_i),
    .loopback_mode_i(loopback_mode_i),
    .tx_fail_o      (tx_fail_o),
    
    // BMC 物理接口
    .bmc_pad        (bmc_pad),
    .bmc_pad_en     (bmc_pad_en)
);

endmodule
