//============================================================================
// PD PHY 寄存器接口模块
// 提供 MCU 通过 SPI 访问 TX/RX 模块的寄存器映射
//============================================================================
// 寄存器地址映射:
// 0x00       CTRL        控制寄存器 [0]:TX_START [1]:RX_EN [2]:AUTO_GOODCRC [3]:LOOPBACK [7]:SOFT_RST
// 0x01       STATUS      状态寄存器 [0]:TX_BUSY [1]:TX_DONE [2]:TX_FAIL [3]:RX_BUSY [4]:RX_DONE [5]:RX_CRC_OK [6]:RX_ERROR
// 0x02       TX_HDR_L    发送 Header 低字节
// 0x03       TX_HDR_H    发送 Header 高字节
// 0x04       TX_LEN      发送数据长度 (0-30)
// 0x05       IRQ_EN      中断使能 [0]:TX_DONE_IE [1]:TX_FAIL_IE [2]:RX_DONE_IE [3]:RX_ERR_IE
// 0x06       IRQ_FLAG    中断标志 (劙1清除) [0]:TX_DONE_IF [1]:TX_FAIL_IF [2]:RX_DONE_IF [3]:RX_ERR_IF
// 0x07       MSG_ID      消息ID [2:0]:MSG_ID
// 0x08       GOODCRC_CFG GoodCRC配置 [0]:PowerRole [1]:DataRole [2]:CablePlug [3]:CableRole
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
    input  wire         tx_fail,
    input  wire         tx_busy,
    
    // RX 模块接口
    input  wire [15:0]  rx_header,
    input  wire [239:0] rx_data_flat,
    input  wire [4:0]   rx_data_len,
    input  wire         rx_done,
    input  wire         rx_busy,
    input  wire         rx_error,
    input  wire         rx_crc_ok,
    output reg          rx_en,
    
    // 协议配置
    output reg  [2:0]   msg_id,
    output reg          auto_goodcrc,
    output reg          loopback_mode,
    output reg  [7:0]   header_good_crc,  // GoodCRC配置 [0]:PowerRole [1]:DataRole [2]:CablePlug [3]:CableRole
    
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
localparam ADDR_MSG_ID   = 8'h07;
localparam ADDR_GOODCRC_CFG = 8'h08;  // GoodCRC配置寄存器
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

// TX_DONE/TX_FAIL 和 RX_DONE/RX_ERROR 边沿检测
reg tx_done_d;
reg tx_fail_d;
reg rx_done_d;
reg rx_error_d;

// RX_CRC_OK 锁存寄存器 - 在RX_DONE时保存，下次RX启动时清除
reg rx_crc_ok_latched;

wire tx_done_rising  = tx_done && !tx_done_d;
wire tx_fail_rising  = tx_fail && !tx_fail_d;
wire rx_done_rising  = rx_done && !rx_done_d;
wire rx_error_rising = rx_error && !rx_error_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_done_d  <= 1'b0;
        tx_fail_d  <= 1'b0;
        rx_done_d  <= 1'b0;
        rx_error_d <= 1'b0;
        rx_crc_ok_latched <= 1'b0;
    end else begin
        tx_done_d  <= tx_done;
        tx_fail_d  <= tx_fail;
        rx_done_d  <= rx_done;
        rx_error_d <= rx_error;
        
        // 锁存 rx_crc_ok：在RX_DONE上升沿保存，下次RX_BUSY上升沿清除
        if (rx_done_rising) begin
            rx_crc_ok_latched <= rx_crc_ok;
        end else if (rx_busy && !rx_done) begin
            // RX开始新的接收，清除锁存
            rx_crc_ok_latched <= 1'b0;
        end
    end
end

//============================================================================
// 中断标志更新
// [0]:TX_DONE_IF [1]:TX_FAIL_IF [2]:RX_DONE_IF [3]:RX_ERR_IF
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_flag_reg <= 8'h00;
    end else begin
        // 硬件置位
        if (tx_done_rising)  irq_flag_reg[0] <= 1'b1;
        if (tx_fail_rising)  irq_flag_reg[1] <= 1'b1;
        if (rx_done_rising)  irq_flag_reg[2] <= 1'b1;
        if (rx_error_rising) irq_flag_reg[3] <= 1'b1;
        
        // 劙1清除
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
        msg_id       <= 3'd0;
        auto_goodcrc <= 1'b1;  // 默认启用自动 GoodCRC
        loopback_mode <= 1'b0;
        header_good_crc <= 8'h00;  // 默认GoodCRC配置
    end else begin
        // 默认清除 tx_start (单周期脉冲)
        tx_start <= 1'b0;
        
        if (reg_wr_en) begin
            case (reg_addr)
                ADDR_CTRL: begin
                    tx_start      <= reg_wr_data[0];
                    rx_en         <= reg_wr_data[1];
                    auto_goodcrc  <= reg_wr_data[2];
                    loopback_mode <= reg_wr_data[3];
                end
                
                ADDR_TX_HDR_L: tx_header[7:0]  <= reg_wr_data;
                ADDR_TX_HDR_H: tx_header[15:8] <= reg_wr_data;
                ADDR_TX_LEN:   tx_data_len     <= reg_wr_data[4:0];
                ADDR_IRQ_EN:   irq_en_reg      <= reg_wr_data;
                ADDR_MSG_ID:   msg_id          <= reg_wr_data[2:0];
                ADDR_GOODCRC_CFG: header_good_crc <= reg_wr_data;  // GoodCRC配置
                
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
        ADDR_CTRL:     reg_rd_data = {4'b0, loopback_mode, auto_goodcrc, rx_en, 1'b0};
        // STATUS: [5]=RX_CRC_OK (锁存值), [4]=RX_DONE, [3]=RX_BUSY, [2]=TX_FAIL, [1]=TX_DONE, [0]=TX_BUSY
        ADDR_STATUS:   reg_rd_data = {1'b0, rx_error, rx_crc_ok_latched, rx_done, rx_busy, tx_fail, tx_done, tx_busy};
        ADDR_TX_HDR_L: reg_rd_data = tx_header[7:0];
        ADDR_TX_HDR_H: reg_rd_data = tx_header[15:8];
        ADDR_TX_LEN:   reg_rd_data = {3'b0, tx_data_len};
        ADDR_IRQ_EN:   reg_rd_data = irq_en_reg;
        ADDR_IRQ_FLAG: reg_rd_data = irq_flag_reg;
        ADDR_MSG_ID:   reg_rd_data = {5'b0, msg_id};
        ADDR_GOODCRC_CFG: reg_rd_data = header_good_crc;  // GoodCRC配置
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
