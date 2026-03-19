//============================================================================
// USB PD BMC 收发器顶层模块
// 标准速率：300 kbps
// 接口：8 位并行数据接口
//============================================================================

`timescale 1ns/1ps

module pd_bmc_transceiver (
    input  wire        clk,           // 系统时钟 (如 50MHz)
    input  wire        rst_n,         // 异步复位，低有效
    
    // 发送接口
    input  wire [7:0]  tx_data_i,     // 发送数据
    input  wire        tx_valid_i,    // 发送数据有效
    output wire        tx_ready_o,    // 发送就绪
    input  wire        tx_start_i,    // 传输开始
    input  wire        tx_sync_i,     // 发送 SYNC 字段
    output wire        tx_done_o,     // 发送完成
    
    // 接收接口
    output wire [7:0]  rx_data_o,     // 接收数据
    output wire        rx_valid_o,    // 接收数据有效
    output wire        rx_error_o,    // 接收错误
    input  wire        rx_en_i,       // 接收使能
    
    // Pad 接口
    output wire        bmc_tx_pad,    // BMC 发送输出
    input  wire        bmc_rx_pad     // BMC 接收输入
);

//============================================================================
// 内部信号
//============================================================================
wire tx_active;

//============================================================================
// 实例化发送模块
//============================================================================
pd_bmc_tx u_pd_bmc_tx (
    .clk          (clk),
    .rst_n        (rst_n),
    .tx_data_i    (tx_data_i),
    .tx_valid_i   (tx_valid_i),
    .tx_ready_o   (tx_ready_o),
    .tx_start_i   (tx_start_i),
    .tx_sync_i    (tx_sync_i),
    .tx_done_o    (tx_done_o),
    .bmc_tx_pad   (bmc_tx_pad),
    .tx_active_o  (tx_active)
);

//============================================================================
// 实例化接收模块
//============================================================================
pd_bmc_rx u_pd_bmc_rx (
    .clk          (clk),
    .rst_n        (rst_n),
    .rx_data_o    (rx_data_o),
    .rx_valid_o   (rx_valid_o),
    .rx_error_o   (rx_error_o),
    .bmc_rx_pad   (bmc_rx_pad),
    .rx_en_i      (rx_en_i)
);

endmodule