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
    
    // BMC 物理接口 (TX/RX 共用一条线)
    inout  wire        bmc_pad
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
// 地址格式: [30:16]=模块ID, [15:0]=寄存器地址
//============================================================================
localparam MODULE_ID_PD   = 15'h0001;  // PD 模块
localparam MODULE_ID_UFCS = 15'h0002;  // UFCS 模块 (预留)

wire        spi_wr_en;
wire        spi_rd_en;
wire [30:0] spi_addr;      // 31位地址 (不含R/W位)
wire [7:0]  spi_wr_data;
reg  [7:0]  spi_rd_data;

// 地址解码
wire [14:0] module_id    = spi_addr[30:16];
wire [15:0] module_addr  = spi_addr[15:0];
wire        pd_selected  = (module_id == MODULE_ID_PD);

spi_slave u_spi_slave (
    .clk         (clk_50m),
    .rst_n       (rst_n),
    .spi_sclk    (spi_sclk),
    .spi_cs_n    (spi_cs_n),
    .spi_mosi    (spi_mosi),
    .spi_miso    (spi_miso),
    .reg_wr_en   (spi_wr_en),
    .reg_rd_en   (spi_rd_en),
    .reg_addr    (spi_addr),
    .reg_wr_data (spi_wr_data),
    .reg_rd_data (spi_rd_data)
);

// PD 模块寄存器接口
wire        pd_wr_en   = spi_wr_en && pd_selected;
wire        pd_rd_en   = spi_rd_en && pd_selected;
wire [7:0]  pd_addr    = module_addr[7:0];
wire [7:0]  pd_rd_data;

// 读数据多路选择 (根据模块ID选择)
always @(*) begin
    case (module_id)
        MODULE_ID_PD:   spi_rd_data = pd_rd_data;
        // MODULE_ID_UFCS: spi_rd_data = ufcs_rd_data;  // 预留
        default:        spi_rd_data = 8'h00;
    endcase
end

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
wire         rx_crc_ok;
wire         rx_en;

// 协议配置信号
wire [2:0]   msg_id;
wire         auto_goodcrc;
wire         loopback_mode;
wire [7:0]   header_good_crc;  // GoodCRC配置 [0]:PowerRole [1]:DataRole [2]:CablePlug [3]:CableRole
wire         tx_fail;

//============================================================================
// PD PHY 寄存器接口
//============================================================================
pd_phy_regs u_pd_phy_regs (
    .clk          (clk_50m),
    .rst_n        (rst_n),
    .reg_wr_en    (pd_wr_en),
    .reg_rd_en    (pd_rd_en),
    .reg_addr     (pd_addr),
    .reg_wr_data  (spi_wr_data),
    .reg_rd_data  (pd_rd_data),
    .tx_header    (tx_header),
    .tx_data_flat (tx_data_flat),
    .tx_data_len  (tx_data_len),
    .tx_start     (tx_start),
    .tx_done      (tx_done),
    .tx_fail      (tx_fail),
    .tx_busy      (tx_busy),
    .rx_header    (rx_header),
    .rx_data_flat (rx_data_flat),
    .rx_data_len  (rx_data_len),
    .rx_done      (rx_done),
    .rx_busy      (rx_busy),
    .rx_error     (rx_error),
    .rx_crc_ok    (rx_crc_ok),
    .rx_en        (rx_en),
    .msg_id       (msg_id),
    .auto_goodcrc (auto_goodcrc),
    .loopback_mode(loopback_mode),
    .header_good_crc (header_good_crc),  // GoodCRC配置
    .irq_n        (irq_n)
);

// LED 控制 (低电平点亮)
assign led_tx     = ~tx_busy;
assign led_rx_ok  = ~rx_done;
assign led_rx_err = ~rx_error;

//============================================================================
// PD BMC 收发器 (TX/RX 互斥，半双工)
//============================================================================
wire bmc_pad_en;        // TX 输出使能

pd_bmc_transceiver u_pd_bmc_transceiver (
    .clk            (clk_50m),
    .rst_n          (rst_n),
    // TX 接口
    .tx_header_i    (tx_header),
    .tx_data_flat_i (tx_data_flat),
    .tx_data_len_i  (tx_data_len),
    .tx_start_i     (tx_start),
    .tx_done_o      (tx_done),
    .tx_fail_o      (tx_fail),
    .tx_busy_o      (tx_busy),
    .tx_active_o    (tx_active),
    // RX 接口
    .rx_header_o    (rx_header),
    .rx_data_flat_o (rx_data_flat),
    .rx_data_len_o  (rx_data_len),
    .rx_done_o      (rx_done),
    .rx_busy_o      (rx_busy),
    .rx_error_o     (rx_error),
    .rx_crc_ok_o    (rx_crc_ok),
    .rx_en_i        (rx_en),
    // 协议配置
    .msg_id_i       (msg_id),
    .auto_goodcrc_i (auto_goodcrc),
    .loopback_mode_i(loopback_mode),
    .header_good_crc (header_good_crc),  // GoodCRC配置
    // BMC 接口 (TX/RX 共用一条线)
    .bmc_pad        (bmc_pad),
    .bmc_pad_en     (bmc_pad_en)
);

endmodule
