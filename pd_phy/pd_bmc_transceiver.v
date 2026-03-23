//============================================================================
// USB PD BMC 收发器
// 纯 PD PHY 功能：BMC TX 和 RX 模块
//============================================================================

`timescale 1ns/1ps

module pd_bmc_transceiver (
    input  wire         clk,
    input  wire         rst_n,
    
    // TX 接口
    input  wire [15:0]  tx_header_i,
    input  wire [239:0] tx_data_flat_i,
    input  wire [4:0]   tx_data_len_i,
    input  wire         tx_start_i,
    output wire         tx_done_o,
    output wire         tx_busy_o,
    output wire         tx_active_o,
    
    // RX 接口
    output wire [15:0]  rx_header_o,
    output wire [239:0] rx_data_flat_o,
    output wire [4:0]   rx_data_len_o,
    output wire         rx_done_o,
    output wire         rx_busy_o,
    output wire         rx_error_o,
    input  wire         rx_en_i,
    
    // BMC 物理接口
    output wire         bmc_tx_pad,
    input  wire         bmc_rx_pad
);

//============================================================================
// TX 模块
//============================================================================
pd_bmc_tx u_pd_bmc_tx (
    .clk            (clk),
    .rst_n          (rst_n),
    .tx_header_i    (tx_header_i),
    .tx_data_flat_i (tx_data_flat_i),
    .tx_data_len_i  (tx_data_len_i),
    .tx_start_i     (tx_start_i),
    .tx_done_o      (tx_done_o),
    .tx_busy_o      (tx_busy_o),
    .bmc_tx_pad     (bmc_tx_pad),
    .tx_active_o    (tx_active_o)
);

//============================================================================
// RX 模块
//============================================================================
pd_bmc_rx u_pd_bmc_rx (
    .clk            (clk),
    .rst_n          (rst_n),
    .rx_header_o    (rx_header_o),
    .rx_data_flat_o (rx_data_flat_o),
    .rx_data_len_o  (rx_data_len_o),
    .rx_done_o      (rx_done_o),
    .rx_busy_o      (rx_busy_o),
    .rx_error_o     (rx_error_o),
    .bmc_rx_pad     (bmc_rx_pad),
    .rx_en_i        (rx_en_i)
);

endmodule
