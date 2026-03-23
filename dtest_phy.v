//============================================================================
// dtest_phy - USB PD PHY 顶层模块
// 包含 SPI 接口、寄存器映射和 BMC 收发器
//============================================================================

`timescale 1ns/1ps

// Simulation model for IBUFDS (Xilinx primitive)
`ifdef SIMULATION
module IBUFDS (
    input  wire I,
    input  wire IB,
    output wire O
);
    assign O = I;
endmodule
`endif

module dtest_phy (
    // 系统时钟 (差分 200MHz)
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    
    // 复位 (active low)
    input  wire        rst_n,
    
    // SPI 从机接口
    input  wire        spi_sclk,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,
    
    // 中断输出 (active low)
    output wire        irq_n,
    
    // LED 指示
    output wire        led_tx,
    output wire        led_rx_ok,
    output wire        led_rx_err,
    output wire        led_heartbeat,
    
    // BMC 物理接口
    output wire        bmc_tx_pad,
    input  wire        bmc_rx_pad
);

//============================================================================
// 时钟处理 (200MHz -> 50MHz)
//============================================================================
wire sys_clk_200m;
IBUFDS u_ibufds_sys_clk (
    .I(sys_clk_p),
    .IB(sys_clk_n),
    .O(sys_clk_200m)
);

reg [1:0] clk_div_cnt;
reg       clk_50m;

always @(posedge sys_clk_200m or negedge rst_n) begin
    if (!rst_n) begin
        clk_div_cnt <= 2'd0;
        clk_50m     <= 1'b0;
    end else begin
        if (clk_div_cnt == 2'd1) begin
            clk_div_cnt <= 2'd0;
            clk_50m     <= ~clk_50m;
        end else begin
            clk_div_cnt <= clk_div_cnt + 1'b1;
        end
    end
end

//============================================================================
// 心跳灯 (1Hz 闪烁 @ 50MHz)
//============================================================================
reg [24:0] heartbeat_cnt;
reg        heartbeat_reg;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        heartbeat_cnt <= 25'd0;
        heartbeat_reg <= 1'b0;
    end else begin
        if (heartbeat_cnt == 25'd25000000) begin
            heartbeat_cnt <= 25'd0;
            heartbeat_reg <= ~heartbeat_reg;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
        end
    end
end

assign led_heartbeat = ~heartbeat_reg;

//============================================================================
// SPI 从机接口
//============================================================================
wire        reg_wr_en;
wire        reg_rd_en;
wire [7:0]  reg_addr;
wire [7:0]  reg_wr_data;
wire [7:0]  reg_rd_data;

spi_slave u_spi_slave (
    .clk         (clk_50m),
    .rst_n       (rst_n),
    .spi_sclk    (spi_sclk),
    .spi_cs_n    (spi_cs_n),
    .spi_mosi    (spi_mosi),
    .spi_miso    (spi_miso),
    .reg_wr_en   (reg_wr_en),
    .reg_rd_en   (reg_rd_en),
    .reg_addr    (reg_addr),
    .reg_wr_data (reg_wr_data),
    .reg_rd_data (reg_rd_data)
);

//============================================================================
// PD PHY 信号
//============================================================================
wire [15:0]  tx_header;
wire [239:0] tx_data_flat;
wire [4:0]   tx_data_len;
wire         tx_start;
wire         tx_done;
wire         tx_busy;
wire         tx_active;

wire [15:0]  rx_header;
wire [239:0] rx_data_flat;
wire [4:0]   rx_data_len;
wire         rx_done;
wire         rx_busy;
wire         rx_error;
wire         rx_en;

//============================================================================
// 寄存器接口
//============================================================================
pd_phy_regs u_pd_phy_regs (
    .clk          (clk_50m),
    .rst_n        (rst_n),
    .reg_wr_en    (reg_wr_en),
    .reg_rd_en    (reg_rd_en),
    .reg_addr     (reg_addr),
    .reg_wr_data  (reg_wr_data),
    .reg_rd_data  (reg_rd_data),
    .tx_header    (tx_header),
    .tx_data_flat (tx_data_flat),
    .tx_data_len  (tx_data_len),
    .tx_start     (tx_start),
    .tx_done      (tx_done),
    .tx_busy      (tx_busy),
    .rx_header    (rx_header),
    .rx_data_flat (rx_data_flat),
    .rx_data_len  (rx_data_len),
    .rx_done      (rx_done),
    .rx_busy      (rx_busy),
    .rx_error     (rx_error),
    .rx_en        (rx_en),
    .irq_n        (irq_n)
);

// LED 控制 (低电平点亮)
assign led_tx     = ~tx_busy;
assign led_rx_ok  = ~rx_done;
assign led_rx_err = ~rx_error;

//============================================================================
// PD BMC 收发器
//============================================================================
pd_bmc_transceiver u_pd_bmc_transceiver (
    .clk            (clk_50m),
    .rst_n          (rst_n),
    // TX 接口
    .tx_header_i    (tx_header),
    .tx_data_flat_i (tx_data_flat),
    .tx_data_len_i  (tx_data_len),
    .tx_start_i     (tx_start),
    .tx_done_o      (tx_done),
    .tx_busy_o      (tx_busy),
    .tx_active_o    (tx_active),
    // RX 接口
    .rx_header_o    (rx_header),
    .rx_data_flat_o (rx_data_flat),
    .rx_data_len_o  (rx_data_len),
    .rx_done_o      (rx_done),
    .rx_busy_o      (rx_busy),
    .rx_error_o     (rx_error),
    .rx_en_i        (rx_en),
    // BMC 接口
    .bmc_tx_pad     (bmc_tx_pad),
    .bmc_rx_pad     (bmc_rx_pad)
);

endmodule
