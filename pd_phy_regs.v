//============================================================================
// PD PHY 寄存器接口模块
// 提供 MCU 通过 SPI 访问 TX/RX 模块的寄存器映射
//============================================================================
// 寄存器地址映射:
// 0x00       CTRL        控制寄存器 [0]:TX_START [1]:RX_EN [7]:SOFT_RST
// 0x01       STATUS      状态寄存器 [0]:TX_BUSY [1]:TX_DONE [2]:RX_BUSY [3]:RX_DONE [4]:RX_ERROR
// 0x02       TX_HDR_L    发送 Header 低字节
// 0x03       TX_HDR_H    发送 Header 高字节
// 0x04       TX_LEN      发送数据长度 (0-30)
// 0x05       IRQ_EN      中断使能 [0]:TX_DONE_IE [1]:RX_DONE_IE [2]:RX_ERR_IE
// 0x06       IRQ_FLAG    中断标志 (写1清除) [0]:TX_DONE_IF [1]:RX_DONE_IF [2]:RX_ERR_IF
// 0x10-0x2D  TX_DATA     发送数据缓冲区 (30 字节)
// 0x40       RX_HDR_L    接收 Header 低字节 (只读)
// 0x41       RX_HDR_H    接收 Header 高字节 (只读)
// 0x42       RX_LEN      接收数据长度 (只读)
// 0x50-0x6D  RX_DATA     接收数据缓冲区 (只读, 30 字节)
//============================================================================

`timescale 1ns/1ps

module pd_phy_regs (
    input  wire        clk,
    input  wire        rst_n,
    
    // 寄存器总线接口
    input  wire        reg_wr_en,
    input  wire        reg_rd_en,
    input  wire [7:0]  reg_addr,
    input  wire [7:0]  reg_wr_data,
    output reg  [7:0]  reg_rd_data,
    
    // TX 模块接口
    output reg  [15:0]  tx_header,
    output reg  [239:0] tx_data_flat,
    output reg  [4:0]   tx_data_len,
    output reg          tx_start,
    input  wire         tx_done,
    input  wire         tx_busy,
    
    // RX 模块接口
    input  wire [15:0]  rx_header,
    input  wire [239:0] rx_data_flat,
    input  wire [4:0]   rx_data_len,
    input  wire         rx_done,
    input  wire         rx_busy,
    input  wire         rx_error,
    output reg          rx_en,
    
    // 中断输出
    output wire         irq_n
);

//============================================================================
// 寄存器地址定义
//============================================================================
localparam ADDR_CTRL     = 8'h00;
localparam ADDR_STATUS   = 8'h01;
localparam ADDR_TX_HDR_L = 8'h02;
localparam ADDR_TX_HDR_H = 8'h03;
localparam ADDR_TX_LEN   = 8'h04;
localparam ADDR_IRQ_EN   = 8'h05;
localparam ADDR_IRQ_FLAG = 8'h06;
localparam ADDR_TX_DATA  = 8'h10;  // 0x10 - 0x2D
localparam ADDR_RX_HDR_L = 8'h40;
localparam ADDR_RX_HDR_H = 8'h41;
localparam ADDR_RX_LEN   = 8'h42;
localparam ADDR_RX_DATA  = 8'h50;  // 0x50 - 0x6D

//============================================================================
// 内部寄存器
//============================================================================
reg [7:0] irq_en_reg;
reg [7:0] irq_flag_reg;

// TX_DONE 和 RX_DONE 边沿检测
reg tx_done_d;
reg rx_done_d;
reg rx_error_d;

wire tx_done_rising = tx_done && !tx_done_d;
wire rx_done_rising = rx_done && !rx_done_d;
wire rx_error_rising = rx_error && !rx_error_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_done_d  <= 1'b0;
        rx_done_d  <= 1'b0;
        rx_error_d <= 1'b0;
    end else begin
        tx_done_d  <= tx_done;
        rx_done_d  <= rx_done;
        rx_error_d <= rx_error;
    end
end

//============================================================================
// 中断标志更新
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_flag_reg <= 8'h00;
    end else begin
        // 硬件置位
        if (tx_done_rising)  irq_flag_reg[0] <= 1'b1;
        if (rx_done_rising)  irq_flag_reg[1] <= 1'b1;
        if (rx_error_rising) irq_flag_reg[2] <= 1'b1;
        
        // 写1清除
        if (reg_wr_en && reg_addr == ADDR_IRQ_FLAG) begin
            irq_flag_reg <= irq_flag_reg & ~reg_wr_data;
        end
    end
end

// 中断输出 (低有效)
assign irq_n = ~|(irq_flag_reg & irq_en_reg);

//============================================================================
// 寄存器写逻辑
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_header    <= 16'h0000;
        tx_data_flat <= 240'h0;
        tx_data_len  <= 5'd0;
        tx_start     <= 1'b0;
        rx_en        <= 1'b1;
        irq_en_reg   <= 8'h00;
    end else begin
        // 默认清除 tx_start (单周期脉冲)
        tx_start <= 1'b0;
        
        if (reg_wr_en) begin
            case (reg_addr)
                ADDR_CTRL: begin
                    tx_start <= reg_wr_data[0];
                    rx_en    <= reg_wr_data[1];
                end
                
                ADDR_TX_HDR_L: tx_header[7:0]  <= reg_wr_data;
                ADDR_TX_HDR_H: tx_header[15:8] <= reg_wr_data;
                ADDR_TX_LEN:   tx_data_len     <= reg_wr_data[4:0];
                ADDR_IRQ_EN:   irq_en_reg      <= reg_wr_data;
                
                default: begin
                    // TX_DATA 区域 (0x10 - 0x2D)
                    if (reg_addr >= ADDR_TX_DATA && reg_addr < (ADDR_TX_DATA + 8'h1E)) begin
                        case (reg_addr - ADDR_TX_DATA)
                            8'd0:  tx_data_flat[7:0]     <= reg_wr_data;
                            8'd1:  tx_data_flat[15:8]    <= reg_wr_data;
                            8'd2:  tx_data_flat[23:16]   <= reg_wr_data;
                            8'd3:  tx_data_flat[31:24]   <= reg_wr_data;
                            8'd4:  tx_data_flat[39:32]   <= reg_wr_data;
                            8'd5:  tx_data_flat[47:40]   <= reg_wr_data;
                            8'd6:  tx_data_flat[55:48]   <= reg_wr_data;
                            8'd7:  tx_data_flat[63:56]   <= reg_wr_data;
                            8'd8:  tx_data_flat[71:64]   <= reg_wr_data;
                            8'd9:  tx_data_flat[79:72]   <= reg_wr_data;
                            8'd10: tx_data_flat[87:80]   <= reg_wr_data;
                            8'd11: tx_data_flat[95:88]   <= reg_wr_data;
                            8'd12: tx_data_flat[103:96]  <= reg_wr_data;
                            8'd13: tx_data_flat[111:104] <= reg_wr_data;
                            8'd14: tx_data_flat[119:112] <= reg_wr_data;
                            8'd15: tx_data_flat[127:120] <= reg_wr_data;
                            8'd16: tx_data_flat[135:128] <= reg_wr_data;
                            8'd17: tx_data_flat[143:136] <= reg_wr_data;
                            8'd18: tx_data_flat[151:144] <= reg_wr_data;
                            8'd19: tx_data_flat[159:152] <= reg_wr_data;
                            8'd20: tx_data_flat[167:160] <= reg_wr_data;
                            8'd21: tx_data_flat[175:168] <= reg_wr_data;
                            8'd22: tx_data_flat[183:176] <= reg_wr_data;
                            8'd23: tx_data_flat[191:184] <= reg_wr_data;
                            8'd24: tx_data_flat[199:192] <= reg_wr_data;
                            8'd25: tx_data_flat[207:200] <= reg_wr_data;
                            8'd26: tx_data_flat[215:208] <= reg_wr_data;
                            8'd27: tx_data_flat[223:216] <= reg_wr_data;
                            8'd28: tx_data_flat[231:224] <= reg_wr_data;
                            8'd29: tx_data_flat[239:232] <= reg_wr_data;
                            default: ;
                        endcase
                    end
                end
            endcase
        end
    end
end

//============================================================================
// 寄存器读逻辑
//============================================================================
always @(*) begin
    reg_rd_data = 8'h00;
    
    case (reg_addr)
        ADDR_CTRL:     reg_rd_data = {6'b0, rx_en, 1'b0};
        ADDR_STATUS:   reg_rd_data = {3'b0, rx_error, rx_done, rx_busy, tx_done, tx_busy};
        ADDR_TX_HDR_L: reg_rd_data = tx_header[7:0];
        ADDR_TX_HDR_H: reg_rd_data = tx_header[15:8];
        ADDR_TX_LEN:   reg_rd_data = {3'b0, tx_data_len};
        ADDR_IRQ_EN:   reg_rd_data = irq_en_reg;
        ADDR_IRQ_FLAG: reg_rd_data = irq_flag_reg;
        ADDR_RX_HDR_L: reg_rd_data = rx_header[7:0];
        ADDR_RX_HDR_H: reg_rd_data = rx_header[15:8];
        ADDR_RX_LEN:   reg_rd_data = {3'b0, rx_data_len};
        
        default: begin
            // TX_DATA 区域读取
            if (reg_addr >= ADDR_TX_DATA && reg_addr < (ADDR_TX_DATA + 8'h1E)) begin
                case (reg_addr - ADDR_TX_DATA)
                    8'd0:  reg_rd_data = tx_data_flat[7:0];
                    8'd1:  reg_rd_data = tx_data_flat[15:8];
                    8'd2:  reg_rd_data = tx_data_flat[23:16];
                    8'd3:  reg_rd_data = tx_data_flat[31:24];
                    8'd4:  reg_rd_data = tx_data_flat[39:32];
                    8'd5:  reg_rd_data = tx_data_flat[47:40];
                    8'd6:  reg_rd_data = tx_data_flat[55:48];
                    8'd7:  reg_rd_data = tx_data_flat[63:56];
                    8'd8:  reg_rd_data = tx_data_flat[71:64];
                    8'd9:  reg_rd_data = tx_data_flat[79:72];
                    8'd10: reg_rd_data = tx_data_flat[87:80];
                    8'd11: reg_rd_data = tx_data_flat[95:88];
                    8'd12: reg_rd_data = tx_data_flat[103:96];
                    8'd13: reg_rd_data = tx_data_flat[111:104];
                    8'd14: reg_rd_data = tx_data_flat[119:112];
                    8'd15: reg_rd_data = tx_data_flat[127:120];
                    8'd16: reg_rd_data = tx_data_flat[135:128];
                    8'd17: reg_rd_data = tx_data_flat[143:136];
                    8'd18: reg_rd_data = tx_data_flat[151:144];
                    8'd19: reg_rd_data = tx_data_flat[159:152];
                    8'd20: reg_rd_data = tx_data_flat[167:160];
                    8'd21: reg_rd_data = tx_data_flat[175:168];
                    8'd22: reg_rd_data = tx_data_flat[183:176];
                    8'd23: reg_rd_data = tx_data_flat[191:184];
                    8'd24: reg_rd_data = tx_data_flat[199:192];
                    8'd25: reg_rd_data = tx_data_flat[207:200];
                    8'd26: reg_rd_data = tx_data_flat[215:208];
                    8'd27: reg_rd_data = tx_data_flat[223:216];
                    8'd28: reg_rd_data = tx_data_flat[231:224];
                    8'd29: reg_rd_data = tx_data_flat[239:232];
                    default: reg_rd_data = 8'h00;
                endcase
            end
            // RX_DATA 区域读取
            else if (reg_addr >= ADDR_RX_DATA && reg_addr < (ADDR_RX_DATA + 8'h1E)) begin
                case (reg_addr - ADDR_RX_DATA)
                    8'd0:  reg_rd_data = rx_data_flat[7:0];
                    8'd1:  reg_rd_data = rx_data_flat[15:8];
                    8'd2:  reg_rd_data = rx_data_flat[23:16];
                    8'd3:  reg_rd_data = rx_data_flat[31:24];
                    8'd4:  reg_rd_data = rx_data_flat[39:32];
                    8'd5:  reg_rd_data = rx_data_flat[47:40];
                    8'd6:  reg_rd_data = rx_data_flat[55:48];
                    8'd7:  reg_rd_data = rx_data_flat[63:56];
                    8'd8:  reg_rd_data = rx_data_flat[71:64];
                    8'd9:  reg_rd_data = rx_data_flat[79:72];
                    8'd10: reg_rd_data = rx_data_flat[87:80];
                    8'd11: reg_rd_data = rx_data_flat[95:88];
                    8'd12: reg_rd_data = rx_data_flat[103:96];
                    8'd13: reg_rd_data = rx_data_flat[111:104];
                    8'd14: reg_rd_data = rx_data_flat[119:112];
                    8'd15: reg_rd_data = rx_data_flat[127:120];
                    8'd16: reg_rd_data = rx_data_flat[135:128];
                    8'd17: reg_rd_data = rx_data_flat[143:136];
                    8'd18: reg_rd_data = rx_data_flat[151:144];
                    8'd19: reg_rd_data = rx_data_flat[159:152];
                    8'd20: reg_rd_data = rx_data_flat[167:160];
                    8'd21: reg_rd_data = rx_data_flat[175:168];
                    8'd22: reg_rd_data = rx_data_flat[183:176];
                    8'd23: reg_rd_data = rx_data_flat[191:184];
                    8'd24: reg_rd_data = rx_data_flat[199:192];
                    8'd25: reg_rd_data = rx_data_flat[207:200];
                    8'd26: reg_rd_data = rx_data_flat[215:208];
                    8'd27: reg_rd_data = rx_data_flat[223:216];
                    8'd28: reg_rd_data = rx_data_flat[231:224];
                    8'd29: reg_rd_data = rx_data_flat[239:232];
                    default: reg_rd_data = 8'h00;
                endcase
            end
        end
    endcase
end

endmodule
