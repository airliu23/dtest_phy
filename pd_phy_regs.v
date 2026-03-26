//============================================================================
// PD PHY 寄存器接口模块
// 提供 MCU 通过 I2C/SPI 访问 TX/RX 模块的寄存器映射
//============================================================================
// 寄存器地址映射 (根据 regs.txt):
// 0x00-0x05    R       只读ID/版本信息
// 0x10         R/W     IRQ_FLAG 中断标志(写1清) [0]:CC_CHG [2]:RX_SUCCESS [3]:HARD_RST [4]:TX_FAIL [6]:TX_SUCCESS
// 0x12         R/W     IRQ_EN   中断使能       [0]:CC_CHG [2]:RX_SUCCESS [3]:HARD_RST [4]:TX_FAIL [6]:TX_SUCCESS
// 0x1A         R/W     CC_CTRL  CC控制 [1:0]:CC1 [3:2]:CC2 [5:4]:RP_LVL [6]:DRP
// 0x1C         R       CC_STATUS CC状态
// 0x2F         R/W     RX_SOP_EN RX使能掩码(非0则RX使能) [0]:SOP [1]:SOP' [2]:SOP'' [3]:SOP'_Dbg [4]:SOP''_Dbg [5]:HardRst [6]:CableRst
// 0x30         R       RX_LEN   接收数据长度
// 0x31-0x32    R       RX_HDR   接收Header
// 0x33-0x4F    R       RX_DATA  接收数据 (26字节)
// 0x50         R/W     TX_FRAME_TYPE [2:0]:SOP类型 [3]:TX_START
// 0x51         R/W     TX_LEN   发送数据长度
// 0x52-0x53    R/W     TX_HDR   发送Header
// 0x54-0x6F    R/W     TX_DATA  发送数据 (17字节)
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
    output reg  [2:0]   tx_sop_type,     // TX SOP 类型
    output reg          tx_start,
    input  wire         tx_success,      // 发送成功（收到 GoodCRC）
    input  wire         tx_fail,         // 发送失败（超时）
    
    // RX 模块接口
    input  wire [15:0]  rx_header,
    input  wire [239:0] rx_data_flat,
    input  wire [4:0]   rx_data_len,
    input  wire         rx_success,      // 接收成功（回复 GoodCRC 完成）
    input  wire         rx_hard_reset,   // 收到 HARD_RESET
    input  wire [2:0]   rx_sop_type,     // 接收到的 SOP 类型
    output wire         rx_en,           // RX 使能: rx_sop_en_mask != 0 时使能
    output reg  [7:0]   rx_sop_en_mask,  // RX SOP 类型使能掩码 [0]:SOP [1]:SOP' [2]:SOP'' [3]:SOP'_Dbg [4]:SOP''_Dbg [5]:HardRst [6]:CableRst
    
    // 协议配置
    output reg  [7:0]   header_good_crc,  // GoodCRC配置
    
    // CC 接口
    output reg  [1:0]   cc1_mode,         // CC1 模式: 00=Open, 01=Rd, 10=Ra, 11=Rp
    output reg  [1:0]   cc2_mode,         // CC2 模式
    output reg  [1:0]   rp_level,         // Rp 电流: 00=0.5A, 01=1.5A, 10=3A
    output reg          drp_mode,         // DRP 模式使能
    output reg          plug_orient,      // 插头方向
    input  wire [2:0]   cc1_state,        // CC1 消抖后状态
    input  wire [2:0]   cc2_state,        // CC2 消抖后状态
    input  wire [2:0]   cc1_mode_out,     // CC1 模式输出
    input  wire [2:0]   cc2_mode_out,     // CC2 模式输出
    input  wire         cc_changed,       // CC 变化中断
    
    // RX 调试接口
    input  wire [2:0]   dbg_rx_state,
    
    // 中断输出
    output wire         irq_n
);

//============================================================================
// 寄存器地址定义
//============================================================================
// 只读ID
localparam ADDR_ID0         = 8'h00;
localparam ADDR_ID1         = 8'h01;
localparam ADDR_ID2         = 8'h02;
localparam ADDR_ID3         = 8'h03;
localparam ADDR_ID4         = 8'h04;
localparam ADDR_ID5         = 8'h05;

// 中断寄存器
localparam ADDR_IRQ_FLAG    = 8'h10;  // 中断标志 (写1清)
localparam ADDR_IRQ_EN      = 8'h12;  // 中断使能

// CC 寄存器
localparam ADDR_CC_CTRL     = 8'h1A;  // CC 控制
localparam ADDR_CC_STATUS   = 8'h1C;  // CC 状态

// RX 寄存器
localparam ADDR_RX_SOP_EN   = 8'h2F;  // RX SOP 类型使能掩码
localparam ADDR_RX_LEN      = 8'h30;
localparam ADDR_RX_HDR_L    = 8'h31;
localparam ADDR_RX_HDR_H    = 8'h32;
localparam ADDR_RX_DATA     = 8'h33;  // 0x33 - 0x4F (29 bytes)

// TX 寄存器
localparam ADDR_TX_FRAME_TYPE = 8'h50;  // [2:0]:TX_FRAME_TYPE (SOP类型)
localparam ADDR_TX_LEN      = 8'h51;
localparam ADDR_TX_HDR_L    = 8'h52;
localparam ADDR_TX_HDR_H    = 8'h53;
localparam ADDR_TX_DATA     = 8'h54;  // 0x54 - 0x6F (28 bytes)

//============================================================================
// 内部寄存器
//============================================================================
reg [7:0] irq_en_reg;
reg [7:0] irq_flag_reg;

// 状态边沿检测
reg tx_success_d;
reg tx_fail_d;
reg rx_success_d;
reg rx_hard_reset_d;
reg cc_changed_d;

wire tx_success_rising = tx_success && !tx_success_d;
wire tx_fail_rising    = tx_fail && !tx_fail_d;
wire rx_success_rising = rx_success && !rx_success_d;
wire rx_hard_reset_rising = rx_hard_reset && !rx_hard_reset_d;
wire cc_changed_rising = cc_changed && !cc_changed_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_success_d <= 1'b0;
        tx_fail_d    <= 1'b0;
        rx_success_d <= 1'b0;
        rx_hard_reset_d <= 1'b0;
        cc_changed_d <= 1'b0;
    end else begin
        tx_success_d <= tx_success;
        tx_fail_d    <= tx_fail;
        rx_success_d <= rx_success;
        rx_hard_reset_d <= rx_hard_reset;
        cc_changed_d <= cc_changed;
    end
end

//============================================================================
// 中断标志更新
// IRQ_FLAG: [0]:CC_CHG [2]:RX_SUCCESS [3]:HARD_RST [4]:TX_FAIL [6]:TX_SUCCESS
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        irq_flag_reg <= 8'h00;
    end else begin
        // 硬件置位
        if (cc_changed_rising) irq_flag_reg[0] <= 1'b1;
        if (rx_success_rising) irq_flag_reg[2] <= 1'b1;
        if (rx_hard_reset_rising) irq_flag_reg[3] <= 1'b1;
        if (tx_fail_rising)    irq_flag_reg[4] <= 1'b1;
        if (tx_success_rising) irq_flag_reg[6] <= 1'b1;
        
        // 写1清除
        if (reg_wr_en && reg_addr == ADDR_IRQ_FLAG) begin
            irq_flag_reg <= irq_flag_reg & ~reg_wr_data;
        end
    end
end

// 中断输出 (低有效)
assign irq_n = ~|(irq_flag_reg & irq_en_reg);

// RX 使能: 当 rx_sop_en_mask 非零时使能 RX
assign rx_en = |rx_sop_en_mask;

//============================================================================
// 寄存器写逻辑
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_header    <= 16'h0000;
        tx_data_flat <= 240'h0;
        tx_data_len  <= 5'd0;
        tx_sop_type  <= 3'd0;    // 默认 SOP
        tx_start     <= 1'b0;
        rx_sop_en_mask <= 8'h00; // 默认禁用所有 SOP 类型 (rx_en = 0)
        irq_en_reg   <= 8'h00;
        header_good_crc <= 8'h00;
        cc1_mode     <= 2'b01;  // 默认 Rd 下拉
        cc2_mode     <= 2'b01;  // 默认 Rd 下拉
        rp_level     <= 2'b00;
        drp_mode     <= 1'b0;   // 默认关闭 DRP 模式
        plug_orient  <= 1'b0;
    end else begin
        // 默认清除 tx_start (单周期脉冲)
        tx_start <= 1'b0;
        
        if (reg_wr_en) begin
            case (reg_addr)
                // TX_FRAME_TYPE: [2:0]:SOP类型 [3]:TX_START [4]:RX_EN
                ADDR_TX_FRAME_TYPE: begin
                    tx_sop_type <= reg_wr_data[2:0];
                    tx_start    <= reg_wr_data[3];
                end
                
                ADDR_IRQ_EN: irq_en_reg <= reg_wr_data;
                
                ADDR_RX_SOP_EN: rx_sop_en_mask <= reg_wr_data;
                
                // CC 控制: [1:0]:CC1 [3:2]:CC2 [5:4]:RP_LVL [6]:DRP
                ADDR_CC_CTRL: begin
                    cc1_mode <= reg_wr_data[1:0];
                    cc2_mode <= reg_wr_data[3:2];
                    rp_level <= reg_wr_data[5:4];
                    drp_mode <= reg_wr_data[6];
                end
                
                ADDR_TX_LEN:   tx_data_len    <= reg_wr_data[4:0];
                ADDR_TX_HDR_L: tx_header[7:0] <= reg_wr_data;
                ADDR_TX_HDR_H: tx_header[15:8] <= reg_wr_data;
                
                default: begin
                    // TX_DATA 区域 (0x54 - 0x6F, 28 bytes)
                    if (reg_addr >= ADDR_TX_DATA && reg_addr <= 8'h6F) begin
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
        // 只读 ID
        ADDR_ID0: reg_rd_data = 8'h1C;
        ADDR_ID1: reg_rd_data = 8'h31;
        ADDR_ID2: reg_rd_data = 8'h50;
        ADDR_ID3: reg_rd_data = 8'h21;
        ADDR_ID4: reg_rd_data = 8'h50;
        ADDR_ID5: reg_rd_data = 8'h00;
        
        // 中断寄存器
        ADDR_IRQ_FLAG: reg_rd_data = irq_flag_reg;
        ADDR_IRQ_EN:   reg_rd_data = irq_en_reg;
        
        // CC 寄存器
        ADDR_CC_CTRL:   reg_rd_data = {1'b0, drp_mode, rp_level, cc2_mode, cc1_mode};
        ADDR_CC_STATUS: reg_rd_data = {2'b0, cc2_state, cc1_state};
        
        // RX SOP 使能掩码
        ADDR_RX_SOP_EN: reg_rd_data = rx_sop_en_mask;
        
        // RX 寄存器
        ADDR_RX_LEN:   reg_rd_data = {3'b0, rx_data_len};
        ADDR_RX_HDR_L: reg_rd_data = rx_header[7:0];
        ADDR_RX_HDR_H: reg_rd_data = rx_header[15:8];
        
        // TX 寄存器
        ADDR_TX_FRAME_TYPE: reg_rd_data = {5'b0, tx_sop_type};
        ADDR_TX_LEN:   reg_rd_data = {3'b0, tx_data_len};
        ADDR_TX_HDR_L: reg_rd_data = tx_header[7:0];
        ADDR_TX_HDR_H: reg_rd_data = tx_header[15:8];
        
        default: begin
            // RX_DATA 区域读取 (0x33 - 0x4F, 29 bytes)
            if (reg_addr >= ADDR_RX_DATA && reg_addr <= 8'h4F) begin
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
                    default: reg_rd_data = 8'h00;
                endcase
            end
            // TX_DATA 区域读取 (0x54 - 0x6F, 28 bytes)
            else if (reg_addr >= ADDR_TX_DATA && reg_addr <= 8'h6F) begin
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
                    default: reg_rd_data = 8'h00;
                endcase
            end
        end
    endcase
end

endmodule
