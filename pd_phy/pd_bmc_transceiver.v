//============================================================================
// USB PD BMC 收发器顶层模块
// 标准速率：300 kbps
// 特性：每次发送前自动添加 64bit 的 01 交替前导码
//============================================================================

`timescale 1ns/1ps

module pd_bmc_transceiver (
    input  wire               clk,           // 系统时钟 (如 50MHz)
    input  wire               rst_n,         // 异步复位，低有效
    
    // 发送接口 (与 pd_bmc_tx 匹配)
    input  wire [15:0]        tx_header_i,   // 2 字节 Message Header
    input  wire [7:0]         tx_data_i [0:29], // 最多 30 字节数据载荷
    input  wire [4:0]         tx_data_len_i, // 实际发送的数据字节数 (0-30)
    input  wire               tx_start_i,    // 包发送启动信号
    output wire               tx_done_o,     // 包发送完成标志
    output wire               tx_busy_o,     // 发送忙标志
    
    // 接收接口 (与 pd_bmc_rx 匹配)
    output wire [15:0]        rx_header_o,   // 2 字节 Message Header
    output wire [7:0]         rx_data_o [0:29], // 最多 30 字节数据载荷
    output wire [4:0]         rx_data_len_o, // 实际接收的数据字节数
    output wire               rx_done_o,     // 包接收完成标志
    output wire               rx_busy_o,     // 接收忙标志
    output wire               rx_error_o,    // 接收错误
    input  wire               rx_en_i,       // 接收使能
    
    // Pad 接口
    output wire               bmc_tx_pad,    // BMC 发送输出
    input  wire               bmc_rx_pad     // BMC 接收输入
);

//============================================================================
// 内部信号
//============================================================================
wire tx_active;

//============================================================================
// 实例化发送模块
//============================================================================
pd_bmc_tx u_pd_bmc_tx (
    .clk            (clk),
    .rst_n          (rst_n),
    .tx_header_i    (tx_header_i),
    .tx_data_i      (tx_data_i),
    .tx_data_len_i  (tx_data_len_i),
    .tx_start_i     (tx_start_i),
    .tx_done_o      (tx_done_o),
    .tx_busy_o      (tx_busy_o),
    .bmc_tx_pad     (bmc_tx_pad),
    .tx_active_o    (tx_active)
);

//============================================================================
// 实例化接收模块
//============================================================================
pd_bmc_rx u_pd_bmc_rx (
    .clk          (clk),
    .rst_n        (rst_n),
    .rx_header_o  (rx_header_o),
    .rx_data_o    (rx_data_o),
    .rx_data_len_o(rx_data_len_o),
    .rx_done_o    (rx_done_o),
    .rx_busy_o    (rx_busy_o),
    .rx_error_o   (rx_error_o),
    .bmc_rx_pad   (bmc_rx_pad),
    .rx_en_i      (rx_en_i)
);

endmodule