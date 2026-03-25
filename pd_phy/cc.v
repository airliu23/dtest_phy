//============================================================================
// USB PD CC (Configuration Channel) 状态处理模块
// 功能：
// - 检测 CC 电平变化（连接/断开检测）
// - 支持 CC1/CC2 双通道检测
// - 实现 Type-C 连接状态机
//============================================================================

`timescale 1ns/1ps

module cc (
    input  wire        clk,
    input  wire        rst_n,
    
    // CC 物理接口, 电压状态: 3-bit, 8种状态
    // 000 = < 0.2V (Ra/Rd 检测)
    // 001 = 0.2V ~ 0.4V
    // 010 = 0.4V ~ 0.66V
    // 011 = 0.66V ~ 0.8V
    // 100 = 0.8V ~ 1.23V
    // 101 = 1.23V ~ 1.6V
    // 110 = 1.6V ~ 2.6V
    // 111 = > 2.6V
    input  wire [2:0]  cc1_state_i,     // CC1 电压状态
    input  wire [2:0]  cc2_state_i,     // CC2 电压状态

    // CC状态输出
    output reg [2:0]   cc1_debounced_o,
    output reg [2:0]   cc2_debounced_o,
    
    // CC连接中断 (DRP切换过程中不触发，稳定连接后触发)
    output reg         cc_changed_o,    // CC 变化中断
        
    // CC输入配置接口 (2-bit 模式 + 2-bit Rp电流等级)
    // cc_mode: 00=Open, 01=Rd, 10=Ra, 11=Rp
    input  wire [1:0]  cc1_mode_i,      // CC1 模式配置
    input  wire [1:0]  cc2_mode_i,      // CC2 模式配置
    // rp_level: 00=0.5A, 01=1.5A, 10=3.0A, 11=Reserved
    input  wire [1:0]  rp_level_i,      // Rp 电流等级
    input  wire        drp_mode_i,      // DRP 模式使能 (双角色)

    // CC输出控制
    output reg [2:0]   cc1_mode_o,      // CC1 模式控制
    output reg [2:0]   cc2_mode_o       // CC2 模式控制
);

//============================================================================
// 参数定义
//============================================================================
// 基于 50MHz 时钟
localparam CLK_FREQ_MHZ = 50;
localparam US_CNT = CLK_FREQ_MHZ;         // 1us = 50 个时钟周期
localparam DEBOUNCE_PERIOD_US = 2000;     // 2ms 消抖周期
localparam DEBOUNCE_CNT = DEBOUNCE_PERIOD_US * US_CNT;  // 2ms = 100000 个时钟周期

// DRP 切换参数
localparam DRP_TOGGLE_MS = 50;            // DRP 切换周期 50ms
localparam DRP_TOGGLE_CNT = DRP_TOGGLE_MS * 1000 * US_CNT;  // 50ms = 2500000 个时钟周期

// CC 控制模式编码 (3位输出)
localparam CC_MODE_OPEN    = 3'b000;  // 高阻 (Open)
localparam CC_MODE_RD      = 3'b001;  // 下拉 5.1k (Rd) - UFP
localparam CC_MODE_RA      = 3'b010;  // 下拉 1.2k (Ra) - Powered Cable
localparam CC_MODE_RP_0P5A = 3'b011;  // 上拉 36k (Rp 0.5A) - DFP
localparam CC_MODE_RP_1P5A = 3'b100;  // 上拉 12k (Rp 1.5A) - DFP
localparam CC_MODE_RP_3A   = 3'b101;  // 上拉 4.7k (Rp 3A) - DFP

// CC 模式输入编码 (2位输入)
localparam CC_IN_OPEN = 2'b00;  // Open
localparam CC_IN_RD   = 2'b01;  // Rd
localparam CC_IN_RA   = 2'b10;  // Ra
localparam CC_IN_RP   = 2'b11;  // Rp (根据 rp_level 确定具体电流)

// Rp 电流等级编码 (2位)
localparam RP_LEVEL_0P5A = 2'b00;  // 0.5A
localparam RP_LEVEL_1P5A = 2'b01;  // 1.5A
localparam RP_LEVEL_3A   = 2'b10;  // 3.0A

// CC 电压状态编码 (3-bit, 8种状态)
localparam CC_VOLT_RA_RD    = 3'b000;  // < 0.2V: Ra/Rd
localparam CC_VOLT_0P2_0P4  = 3'b001;  // 0.2V ~ 0.4V (Rp 3A 下限)
localparam CC_VOLT_0P4_0P66 = 3'b010;  // 0.4V ~ 0.66V (Rp 3A 上限)
localparam CC_VOLT_0P66_0P8 = 3'b011;  // 0.66V ~ 0.8V (Rp 1.5A 下限)
localparam CC_VOLT_0P8_1P23 = 3'b100;  // 0.8V ~ 1.23V (Rp 1.5A 上限)
localparam CC_VOLT_1P23_1P6 = 3'b101;  // 1.23V ~ 1.6V (Rp 0.5A 下限)
localparam CC_VOLT_1P6_2P6  = 3'b110;  // 1.6V ~ 2.6V (Rp 0.5A 上限)
localparam CC_VOLT_OPEN     = 3'b111;  // > 2.6V: 开路/未连接

// CC 连接检测: 连接状态是 0.2V ~ 2.6V (001 ~ 110)
// 排除 000 (Ra/Rd < 0.2V) 和 111 (开路 > 2.6V)
function is_cc_connected;
    input [2:0] state;
    begin
        is_cc_connected = (state != CC_VOLT_RA_RD) && (state != CC_VOLT_OPEN);
    end
endfunction

// CC 模式转换函数
// 将 2-bit cc_mode + 2-bit rp_level 转换为 3-bit 输出模式
function [2:0] convert_cc_mode;
    input [1:0] mode;
    input [1:0] rp_lvl;
    begin
        case (mode)
            CC_IN_OPEN: convert_cc_mode = CC_MODE_OPEN;
            CC_IN_RD:   convert_cc_mode = CC_MODE_RD;
            CC_IN_RA:   convert_cc_mode = CC_MODE_RA;
            CC_IN_RP: begin
                case (rp_lvl)
                    RP_LEVEL_0P5A: convert_cc_mode = CC_MODE_RP_0P5A;
                    RP_LEVEL_1P5A: convert_cc_mode = CC_MODE_RP_1P5A;
                    RP_LEVEL_3A:   convert_cc_mode = CC_MODE_RP_3A;
                    default:       convert_cc_mode = CC_MODE_RP_0P5A;  // Reserved -> 0.5A
                endcase
            end
            default: convert_cc_mode = CC_MODE_OPEN;
        endcase
    end
endfunction

//============================================================================
// 内部信号
//============================================================================
reg [2:0]  cc1_state_raw;       // CC1 原始状态
reg [2:0]  cc1_state_sync;      // CC1 同步后状态
reg [2:0]  cc1_state_d;         // CC1 延迟状态

reg [2:0]  cc2_state_raw;       // CC2 原始状态
reg [2:0]  cc2_state_sync;      // CC2 同步后状态
reg [2:0]  cc2_state_d;         // CC2 延迟状态

wire       cc1_changed;         // CC1 变化标志
wire       cc2_changed;         // CC2 变化标志
wire       cc_changed;          // CC 任一变化标志

// 硬件消抖 - 稳定检测方式
reg [2:0]  cc1_state_pending;   // CC1 待确认状态
reg [2:0]  cc2_state_pending;   // CC2 待确认状态
reg [16:0] debounce_timer;      // 2ms 消抖计时器 (100000 周期)

// CC 检测状态
reg        cc1_detected;
reg        cc2_detected;
wire       cc_detected_any;

// DRP 切换状态
reg        drp_is_rp;           // DRP 当前状态: 0=Rd, 1=Rp
reg [31:0] drp_toggle_timer;    // DRP 切换计时器

// 连接检测状态
reg        connected;           // 连接状态
reg        connected_d;         // 连接状态延迟
reg [31:0] connect_stable_cnt;  // 连接稳定计数器
localparam CONNECT_STABLE_MS = 10;  // 连接稳定时间 10ms
localparam CONNECT_STABLE_CNT = CONNECT_STABLE_MS * 1000 * US_CNT;

// 消抖后的状态寄存器
reg [2:0]  cc1_state_debounced;     // 消抖后的 CC1 状态
reg [2:0]  cc2_state_debounced;     // 消抖后的 CC2 状态

//============================================================================
// 输入同步 (防止亚稳态)
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cc1_state_raw <= 3'b000;
        cc1_state_sync <= 3'b000;
        cc1_state_d <= 3'b000;
        
        cc2_state_raw <= 3'b000;
        cc2_state_sync <= 3'b000;
        cc2_state_d <= 3'b000;
    end else begin
        // CC1 输入同步
        cc1_state_raw <= cc1_state_i;
        cc1_state_sync <= cc1_state_raw;
        cc1_state_d <= cc1_state_sync;
        
        // CC2 输入同步
        cc2_state_raw <= cc2_state_i;
        cc2_state_sync <= cc2_state_raw;
        cc2_state_d <= cc2_state_sync;
    end
end

// 边沿检测
assign cc1_changed = (cc1_state_sync != cc1_state_d);
assign cc2_changed = (cc2_state_sync != cc2_state_d);
assign cc_changed = cc1_changed || cc2_changed;

//============================================================================
// 硬件消抖处理: 2ms 内状态必须一直保持不变才认为有效
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cc1_state_pending <= CC_VOLT_OPEN;
        cc2_state_pending <= CC_VOLT_OPEN;
        debounce_timer <= 17'd0;
        cc1_detected <= 1'b0;
        cc2_detected <= 1'b0;
        cc1_state_debounced <= CC_VOLT_OPEN;  // 初始为开路状态
        cc2_state_debounced <= CC_VOLT_OPEN;
    end else begin
        // 检测同步状态是否与待确认状态一致
        if ((cc1_state_sync != cc1_state_pending) || (cc2_state_sync != cc2_state_pending)) begin
            // 状态变化，重新开始计时
            cc1_state_pending <= cc1_state_sync;
            cc2_state_pending <= cc2_state_sync;
            debounce_timer <= 17'd0;
        end else if (debounce_timer < DEBOUNCE_CNT) begin
            // 状态稳定，继续计时
            debounce_timer <= debounce_timer + 1'b1;
        end else begin
            // 2ms 内状态一直稳定，更新消抖后的输出
            if ((cc1_state_debounced != cc1_state_pending) || (cc2_state_debounced != cc2_state_pending)) begin
                cc1_state_debounced <= cc1_state_pending;
                cc2_state_debounced <= cc2_state_pending;
                // 更新连接检测状态
                cc1_detected <= is_cc_connected(cc1_state_pending);
                cc2_detected <= is_cc_connected(cc2_state_pending);
            end
        end
    end
end

assign cc_detected_any = cc1_detected || cc2_detected;

// 消抖后的状态输出 & 连接检测
reg [2:0] cc1_debounced_d;  // CC1 消抖状态延迟
reg [2:0] cc2_debounced_d;  // CC2 消抖状态延迟

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cc1_debounced_o <= 3'b000;
        cc2_debounced_o <= 3'b000;
        cc1_debounced_d <= 3'b000;
        cc2_debounced_d <= 3'b000;
        connected <= 1'b0;
        connected_d <= 1'b0;
        connect_stable_cnt <= 32'd0;
        cc_changed_o <= 1'b0;
    end else begin
        // 保存上一周期的消抖状态
        cc1_debounced_d <= cc1_debounced_o;
        cc2_debounced_d <= cc2_debounced_o;
        
        // 输出消抖后的状态
        cc1_debounced_o <= cc1_state_debounced;
        cc2_debounced_o <= cc2_state_debounced;
        connected_d <= connected;
        cc_changed_o <= 1'b0;  // 默认清除
        
        // 连接检测 (基于消抖后的状态)
        // 真正连接状态: 0.2V ~ 3V (Rp-3A/1.5A/0.5A)
        if (is_cc_connected(cc1_state_debounced) || is_cc_connected(cc2_state_debounced)) begin
            // 检测到连接，计数稳定时间
            if (connect_stable_cnt < CONNECT_STABLE_CNT) begin
                connect_stable_cnt <= connect_stable_cnt + 1'b1;
            end else begin
                // 连接稳定
                connected <= 1'b1;
            end
        end else begin
            // 无连接，重置计数器
            connect_stable_cnt <= 32'd0;
            connected <= 1'b0;
        end
        
        // CC 状态变化中断 - 消抖完成后的状态变化即触发
        // 检测消抖后状态的变化（不需要等待 connected 稳定）
        if ((cc1_debounced_o != cc1_debounced_d) || (cc2_debounced_o != cc2_debounced_d)) begin
            if (!drp_mode_i) begin
                // 非 DRP 模式，消抖状态变化即触发中断
                cc_changed_o <= 1'b1;
            end else if (connected || connected_d) begin
                // DRP 模式且已经连接，状态变化触发中断
                cc_changed_o <= 1'b1;
            end
        end
    end
end

//============================================================================
// CC 终端控制
// DRP 模式: 当 drp_mode_i=1 时，自动在 Rd 和 Rp 之间切换
//============================================================================
wire [2:0] cc1_mode_converted = convert_cc_mode(cc1_mode_i, rp_level_i);
wire [2:0] cc2_mode_converted = convert_cc_mode(cc2_mode_i, rp_level_i);
wire [2:0] rp_mode_from_level = convert_cc_mode(CC_IN_RP, rp_level_i);  // DRP Rp 状态使用

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cc1_mode_o <= CC_MODE_OPEN;
        cc2_mode_o <= CC_MODE_OPEN;
        drp_is_rp <= 1'b0;
        drp_toggle_timer <= 32'd0;
    end else begin
        if (drp_mode_i) begin
            // DRP 模式: 自动切换 Rd <-> Rp
            if (drp_toggle_timer < DRP_TOGGLE_CNT) begin
                drp_toggle_timer <= drp_toggle_timer + 1'b1;
            end else begin
                // 切换周期到，切换状态
                drp_toggle_timer <= 32'd0;
                drp_is_rp <= ~drp_is_rp;
            end
            
            // 根据 DRP 状态输出模式
            if (drp_is_rp) begin
                // Rp 状态 (DFP): 使用 rp_level 配置的 Rp 模式
                cc1_mode_o <= rp_mode_from_level;
                cc2_mode_o <= rp_mode_from_level;
            end else begin
                // Rd 状态 (UFP)
                cc1_mode_o <= CC_MODE_RD;
                cc2_mode_o <= CC_MODE_RD;
            end
        end else begin
            // 非 DRP 模式: 直接输出转换后的模式
            cc1_mode_o <= cc1_mode_converted;
            cc2_mode_o <= cc2_mode_converted;
            drp_is_rp <= 1'b0;
            drp_toggle_timer <= 32'd0;
        end
    end
end

endmodule
