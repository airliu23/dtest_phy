//============================================================================
// PD PHY 寄存器接口模块
// 提供 MCU 通过 SPI 访问 TX/RX 模块的寄存器映射
//============================================================================
// 寄存器地址映射:
// 0x00       CTRL        控制寄存器 [0]:TX_START [1]:RX_EN [7]:SOFT_RST
// 0x01       STATUS      状态寄存器 [0]:TX_SUCCESS [1]:TX_FAIL [2]:RX_SUCCESS (读清)
// 0x02       TX_HDR_L    发送 Header 低字节
// 0x03       TX_HDR_H    发送 Header 高字节
// 0x04       TX_LEN      发送数据长度 (0-30)
// 0x05       IRQ_EN      中断使能 [0]:TX_SUCCESS_IE [1]:TX_FAIL_IE [2]:RX_SUCCESS_IE [4]:CC_CHG_IE
// 0x06       IRQ_FLAG    中断标志 (劙1清除) [0]:TX_SUCCESS_IF [1]:TX_FAIL_IF [2]:RX_SUCCESS_IF [4]:CC_CHG_IF
// 0x07       MSG_ID      消息ID [2:0]:MSG_ID
// 0x08       GOODCRC_CFG GoodCRC配置 [0]:PowerRole [1]:DataRole [2]:CablePlug [3]:CableRole
// 0x09       CC_CTRL     CC控制 [0]:DRP_MODE [1]:PLUG_ORIENT [3:2]:RP_LEVEL (00=0.5A,01=1.5A,10=3A)
// 0x0A       CC1_MODE    CC1模式配置 [1:0]:MODE (00=Open,01=Rd,10=Ra,11=Rp)
// 0x0B       CC2_MODE    CC2模式配置 [1:0]:MODE
// 0x0C       CC_STATUS   CC状态 [2:0]:CC1_STATE [5:3]:CC2_STATE [7:6]:VBUS_STATE (只读)
// 0x0D       CC_MODE_OUT CC模式输出 [2:0]:CC1_MODE_O [5:3]:CC2_MODE_O (只读)
// 0x0E       RX_DBG1     RX调试状态 [2:0]:RX_STATE [6:3]:RX_PHY_STATE [7]:ANY_EDGE
// 0x0F       RX_DBG2     RX调试计数 [5:0]:PREAMBLE_CNT [7:6]:BMC_STATE
// 0x10-0x2D  TX_DATA     发送数据缓冲区 (30 字节)
// 0x40       RX_HDR_L    接收 Header 低字节 (只读)
// 0x41       RX_HDR_H    接收 Header 高字节 (只读)
// 0x42       RX_LEN      接收数据长度 (只读)
// 0x43       RX_DBG3     RX调试计数 [4:0]:BYTE_CNT [5]:RX_BIT_VALID
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
    input  wire         tx_success,      // 发送成功（收到 GoodCRC）
    input  wire         tx_fail,         // 发送失败（超时）
    
    // RX 模块接口
    input  wire [15:0]  rx_header,
    input  wire [239:0] rx_data_flat,
    input  wire [4:0]   rx_data_len,
    input  wire         rx_success,      // 接收成功（回复 GoodCRC 完成）
    output reg          rx_en,           // RX 使能控制
    
    // 协议配置
    output reg  [7:0]   header_good_crc,  // GoodCRC配置 [0]:PowerRole [1]:DataRole [2]:CablePlug [3]:CableRole
    
    // CC 接口
    output reg  [1:0]   cc1_mode,         // CC1 模式: 00=Open, 01=Rd, 10=Ra, 11=Rp
    output reg  [1:0]   cc2_mode,         // CC2 模式: 00=Open, 01=Rd, 10=Ra, 11=Rp
    output reg  [1:0]   rp_level,         // Rp 电流: 00=0.5A, 01=1.5A, 10=3A
    output reg          drp_mode,         // DRP 模式使能
    output reg          plug_orient,      // 插头方向: 0=CC1, 1=CC2
    input  wire [2:0]   cc1_state,        // CC1 消抖后状态
    input  wire [2:0]   cc2_state,        // CC2 消抖后状态
    input  wire [2:0]   cc1_mode_out,     // CC1 模式输出
    input  wire [2:0]   cc2_mode_out,     // CC2 模式输出
    input  wire         cc_changed,       // CC 变化中断
    
    // RX 调试接口
    input  wire [2:0]   dbg_rx_state,      // RX 状态机 (transceiver 层)
    
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
localparam ADDR_CC_CTRL     = 8'h09;  // CC控制寄存器
localparam ADDR_CC1_MODE    = 8'h0A;  // CC1模式配置
localparam ADDR_CC2_MODE    = 8'h0B;  // CC2模式配置
localparam ADDR_CC_STATUS   = 8'h0C;  // CC状态 (只读)
localparam ADDR_CC_MODE_OUT = 8'h0D;  // CC模式输出 (只读)
localparam ADDR_RX_DBG1     = 8'h0E;  // RX调试状态 (只读)
localparam ADDR_RX_DBG2     = 8'h0F;  // RX调试计数 (只读)
localparam ADDR_TX_DATA  = 8'h10;  // 0x10 - 0x2D
localparam ADDR_RX_HDR_L = 8'h40;
localparam ADDR_RX_HDR_H = 8'h41;
localparam ADDR_RX_LEN   = 8'h42;
localparam ADDR_RX_DBG3  = 8'h43;  // RX调试计数 (只读)
localparam ADDR_RX_DATA  = 8'h50;  // 0x50 - 0x6D

//============================================================================
// 内部寄存器
//============================================================================
reg [7:0] irq_en_reg;
reg [7:0] irq_flag_reg;

// 状态边沿检测
reg tx_success_d;
reg tx_fail_d;
reg rx_success_d;
reg cc_changed_d;

// 状态锁存寄存器 (读清)
reg tx_success_latched;
reg tx_fail_latched;
reg rx_success_latched;

wire tx_success_rising = tx_success && !tx_success_d;
wire tx_fail_rising    = tx_fail && !tx_fail_d;
wire rx_success_rising = rx_success && !rx_success_d;
wire cc_changed_rising = cc_changed && !cc_changed_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_success_d <= 1'b0;
        tx_fail_d    <= 1'b0;
        rx_success_d <= 1'b0;
        cc_changed_d <= 1'b0;
        tx_success_latched <= 1'b0;
        tx_fail_latched    <= 1'b0;
        rx_success_latched <= 1'b0;
    end else begin
        tx_success_d <= tx_success;
        tx_fail_d    <= tx_fail;
        rx_success_d <= rx_success;
        cc_changed_d <= cc_changed;
        
        // 锁存状态，读 STATUS 时清除
        if (tx_success_rising) tx_success_latched <= 1'b1;
        if (tx_fail_rising)    tx_fail_latched    <= 1'b1;
        if (rx_success_rising) rx_success_latched <= 1'b1;
        
        // 读取 STATUS 寄存器时清除锁存
        if (reg_rd_en && reg_addr == ADDR_STATUS) begin
            tx_success_latched <= 1'b0;
            tx_fail_latched    <= 1'b0;
            rx_success_latched <= 1'b0;
        end
    end
end

//============================================================================
// 中断标志更新
// [0]:TX_SUCCESS_IF [1]:TX_FAIL_IF [2]:RX_SUCCESS_IF [4]:CC_CHG_IF
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_flag_reg <= 8'h00;
    end else begin
        // 硬件置位
        if (tx_success_rising) irq_flag_reg[0] <= 1'b1;
        if (tx_fail_rising)    irq_flag_reg[1] <= 1'b1;
        if (rx_success_rising) irq_flag_reg[2] <= 1'b1;
        if (cc_changed_rising) irq_flag_reg[4] <= 1'b1;
        
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
        rx_en        <= 1'b1;   // 默认启用 RX
        irq_en_reg   <= 8'h00;
        header_good_crc <= 8'h00;  // 默认GoodCRC配置
        cc1_mode     <= 2'b00;   // 默认 Open
        cc2_mode     <= 2'b00;   // 默认 Open
        rp_level     <= 2'b00;   // 默认 0.5A
        drp_mode     <= 1'b0;    // 默认关闭 DRP 模式
        plug_orient  <= 1'b0;    // 默认 CC1
    end else begin
        // 默认清除 tx_start (单周期脉冲)
        tx_start <= 1'b0;
        
        if (reg_wr_en) begin
            case (reg_addr)
                ADDR_CTRL: begin
                    tx_start      <= reg_wr_data[0];
                    rx_en         <= reg_wr_data[1];
                end
                
                ADDR_TX_HDR_L: tx_header[7:0]  <= reg_wr_data;
                ADDR_TX_HDR_H: tx_header[15:8] <= reg_wr_data;
                ADDR_TX_LEN:   tx_data_len     <= reg_wr_data[4:0];
                ADDR_IRQ_EN:   irq_en_reg      <= reg_wr_data;
                ADDR_GOODCRC_CFG: header_good_crc <= reg_wr_data;
                
                ADDR_CC_CTRL: begin
                    drp_mode    <= reg_wr_data[0];
                    plug_orient <= reg_wr_data[1];
                    rp_level    <= reg_wr_data[3:2];
                end
                ADDR_CC1_MODE: cc1_mode <= reg_wr_data[1:0];
                ADDR_CC2_MODE: cc2_mode <= reg_wr_data[1:0];
                
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
        ADDR_CTRL:     reg_rd_data = {6'b0, rx_en, 1'b0};  // [1]=rx_en, [0]=tx_start(只写)
        // STATUS: [2]=RX_SUCCESS, [1]=TX_FAIL, [0]=TX_SUCCESS (读清)
        ADDR_STATUS:   reg_rd_data = {5'b0, rx_success_latched, tx_fail_latched, tx_success_latched};
        ADDR_TX_HDR_L: reg_rd_data = tx_header[7:0];
        ADDR_TX_HDR_H: reg_rd_data = tx_header[15:8];
        ADDR_TX_LEN:   reg_rd_data = {3'b0, tx_data_len};
        ADDR_IRQ_EN:   reg_rd_data = irq_en_reg;
        ADDR_IRQ_FLAG: reg_rd_data = irq_flag_reg;
        ADDR_MSG_ID:   reg_rd_data = 8'h00;  // 不再使用
        ADDR_GOODCRC_CFG: reg_rd_data = header_good_crc;
        ADDR_CC_CTRL:  reg_rd_data = {4'b0, rp_level, plug_orient, drp_mode};
        ADDR_CC1_MODE: reg_rd_data = {6'b0, cc1_mode};
        ADDR_CC2_MODE: reg_rd_data = {6'b0, cc2_mode};
        ADDR_CC_STATUS: reg_rd_data = {2'b0, cc2_state, cc1_state};
        ADDR_CC_MODE_OUT: reg_rd_data = {2'b0, cc2_mode_out, cc1_mode_out};
        ADDR_RX_DBG1: reg_rd_data = {5'b0, dbg_rx_state};  // RX 状态机 (transceiver 层)
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
