//============================================================================
// USB PD CC (Configuration Channel) 状态处理模块
// 功能：
// - 检测 CC 电平变化（连接/断开检测）
// - 检测 VBUS 变化（电源状态）
// - 支持 CC1/CC2 双通道检测
// - 实现 Type-C 连接状态机
//============================================================================

`timescale 1ns/1ps

module cc (
    input  wire        clk,
    input  wire        rst_n,
    
    // CC 物理接口 (来自 Type-C 连接器)
    // CC 电压状态: 3-bit, 5种状态
    // 000 = < 0.2V (Ra/Rd 检测)
    // 001 = 0.2V ~ 1V (Rp-3A 检测)
    // 010 = 1V ~ 2V (Rp-1.5A 检测)
    // 011 = 2V ~ 3V (Rp-0.5A 检测)
    // 100 = > 3V (未连接/开路)
    input  wire [2:0]  cc1_state_i,     // CC1 电压状态
    input  wire [2:0]  cc2_state_i,     // CC2 电压状态

    // CC状态输出
    output reg [2:0]   cc1_debounced_o, // 消抖后的 CC1 状态 (5种电压状态)
    output reg [2:0]   cc2_debounced_o, // 消抖后的 CC2 状态 (5种电压状态)
    
    // CC连接中断 (DRP切换过程中不触发，稳定连接后触发)
    output reg         cc_changed_o,    // CC 变化中断
    
    // VBUS 电压状态: 2-bit, 3种状态
    // 00 = < 0.8V (VBUS 未连接)
    // 01 = 0.8V ~ 3V (VBUS 正在建立)
    // 10 = > 3V (VBUS 正常)
    input  wire [1:0]  vbus_state_i,    // VBUS 电压状态

    // VBUS状态输出
    output reg [1:0]   vbus_debounced_o, // 消抖后的 VBUS 状态 (3种电压状态)
    
    // 每根CC使用3位编码: 000=Open, 001=Rd, 010=Ra, 011=0.5A_Rp, 100=1.5A_Rp, 101=3A_Rp
    // CC输入配置接口
    input  wire [2:0]  cc1_mode_i,      // CC1 模式配置 (6种模式)
    input  wire [2:0]  cc2_mode_i,      // CC2 模式配置 (6种模式)
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
localparam SAMPLE_INTERVAL_US = 10;       // 每 10us 采样一次
localparam SAMPLE_CNT = 100;              // 共采样 100 次
localparam DEBOUNCE_PERIOD_US = 1000;     // 1ms 消抖周期

// DRP 切换参数
localparam DRP_TOGGLE_MS = 50;            // DRP 切换周期 50ms
localparam DRP_TOGGLE_CNT = DRP_TOGGLE_MS * 1000 * US_CNT;  // 50ms = 2500000 个时钟周期

// CC 控制模式编码 (3位)
localparam CC_MODE_OPEN    = 3'b000;  // 高阻 (Open)
localparam CC_MODE_RD      = 3'b001;  // 下拉 5.1k (Rd) - UFP
localparam CC_MODE_RA      = 3'b010;  // 下拉 1.2k (Ra) - Powered Cable
localparam CC_MODE_RP_0P5A = 3'b011;  // 上拉 36k (Rp 0.5A) - DFP
localparam CC_MODE_RP_1P5A = 3'b100;  // 上拉 12k (Rp 1.5A) - DFP
localparam CC_MODE_RP_3A   = 3'b101;  // 上拉 4.7k (Rp 3A) - DFP

// CC 电压状态编码 (3-bit)
localparam CC_VOLT_RA_RD   = 3'b000;  // < 0.2V: Ra/Rd 连接
localparam CC_VOLT_RP_3A   = 3'b001;  // 0.2V ~ 1V: Rp 3A
localparam CC_VOLT_RP_1P5A = 3'b010;  // 1V ~ 2V: Rp 1.5A
localparam CC_VOLT_RP_0P5A = 3'b011;  // 2V ~ 3V: Rp 0.5A
localparam CC_VOLT_OPEN    = 3'b100;  // > 3V: 开路/未连接

// VBUS 电压状态编码 (2-bit)
localparam VBUS_VOLT_LOW   = 2'b00;   // < 0.8V: VBUS 未连接
localparam VBUS_VOLT_MID   = 2'b01;   // 0.8V ~ 3V: VBUS 建立中
localparam VBUS_VOLT_HIGH  = 2'b10;   // > 3V: VBUS 正常

// CC 连接检测阈值 (用于状态判断)
localparam CC_IS_CONNECTED = 3'b100;  // 低于此值表示有连接

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

reg [1:0]  vbus_state_raw;      // VBUS 原始状态
reg [1:0]  vbus_state_sync;     // VBUS 同步后状态
reg [1:0]  vbus_state_d;        // VBUS 延迟状态
wire       vbus_changed;        // VBUS 变化标志

// 硬件消抖计数器
reg [15:0] sample_timer;        // 采样间隔计时器 (20us)
reg [6:0]  sample_idx;          // 采样索引 (0-99)
reg [6:0]  cc1_high_cnt;        // CC1 高电平计数 (< CC_VOLT_OPEN)
reg [6:0]  cc2_high_cnt;        // CC2 高电平计数 (< CC_VOLT_OPEN)
reg        debounce_active;     // 消抖进行中
reg [15:0] debounce_timer;      // 2ms 周期计时器

// CC 检测状态
reg        cc1_detected;
reg        cc2_detected;
reg        cc_detected_any;

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
reg [1:0]  vbus_state_debounced;    // 消抖后的 VBUS 状态

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
        
        vbus_state_raw <= 2'b00;
        vbus_state_sync <= 2'b00;
        vbus_state_d <= 2'b00;
    end else begin
        // CC1 输入同步
        cc1_state_raw <= cc1_state_i;
        cc1_state_sync <= cc1_state_raw;
        cc1_state_d <= cc1_state_sync;
        
        // CC2 输入同步
        cc2_state_raw <= cc2_state_i;
        cc2_state_sync <= cc2_state_raw;
        cc2_state_d <= cc2_state_sync;
        
        // VBUS 输入同步
        vbus_state_raw <= vbus_state_i;
        vbus_state_sync <= vbus_state_raw;
        vbus_state_d <= vbus_state_sync;
    end
end

// 边沿检测
assign cc1_changed = (cc1_state_sync != cc1_state_d);
assign cc2_changed = (cc2_state_sync != cc2_state_d);
assign cc_changed = cc1_changed || cc2_changed;
assign vbus_changed = (vbus_state_sync != vbus_state_d);

//============================================================================
// 硬件消抖处理: 2ms内采样100次, 多数表决
//============================================================================

// 状态不匹配检测 - 当同步状态与消抖输出不一致时需要重新消抖
wire cc_state_mismatch = (cc1_state_sync != cc1_state_debounced) || 
                         (cc2_state_sync != cc2_state_debounced);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_timer <= 16'd0;
        sample_idx <= 7'd0;
        cc1_high_cnt <= 7'd0;
        cc2_high_cnt <= 7'd0;
        debounce_active <= 1'b0;
        debounce_timer <= 16'd0;
        cc1_detected <= 1'b0;
        cc2_detected <= 1'b0;
        cc1_state_debounced <= CC_VOLT_OPEN;  // 初始为开路状态
        cc2_state_debounced <= CC_VOLT_OPEN;
        vbus_state_debounced <= 2'b00;
    end else begin
        // CC 状态变化或状态不匹配时启动消抖
        if ((cc_changed || cc_state_mismatch) && !debounce_active) begin
            debounce_active <= 1'b1;
            sample_idx <= 7'd0;
            cc1_high_cnt <= 7'd0;
            cc2_high_cnt <= 7'd0;
            sample_timer <= 16'd0;
            debounce_timer <= 16'd0;
        end else if (debounce_active) begin
            // 2ms 周期计时
            if (debounce_timer < DEBOUNCE_PERIOD_US * US_CNT) begin
                debounce_timer <= debounce_timer + 1'b1;
                
                // 采样间隔计时 (20us)
                if (sample_timer < SAMPLE_INTERVAL_US * US_CNT - 1) begin
                    sample_timer <= sample_timer + 1'b1;
                end else begin
                    sample_timer <= 16'd0;
                    
                    // 采样并计数
                    if (sample_idx < SAMPLE_CNT) begin
                        if (cc1_state_sync < CC_VOLT_OPEN)
                            cc1_high_cnt <= cc1_high_cnt + 1'b1;
                        if (cc2_state_sync < CC_VOLT_OPEN)
                            cc2_high_cnt <= cc2_high_cnt + 1'b1;
                        sample_idx <= sample_idx + 1'b1;
                    end
                end
            end else begin
                // 2ms 周期结束, 多数表决 (>50次)
                cc1_detected <= (cc1_high_cnt > 50);
                cc2_detected <= (cc2_high_cnt > 50);
                
                // 保存消抖后的状态 - 直接保存采样时的状态值
                cc1_state_debounced <= cc1_state_sync;
                cc2_state_debounced <= cc2_state_sync;
                
                // VBUS 状态也进行消抖 (直接同步,无时序要求)
                vbus_state_debounced <= vbus_state_sync;
                
                debounce_active <= 1'b0;
            end
        end
    end
end

assign cc_detected_any = cc1_detected || cc2_detected;

// 消抖后的状态输出 & 连接检测
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cc1_debounced_o <= 3'b000;
        cc2_debounced_o <= 3'b000;
        vbus_debounced_o <= 2'b00;
        connected <= 1'b0;
        connected_d <= 1'b0;
        connect_stable_cnt <= 32'd0;
        cc_changed_o <= 1'b0;
    end else begin
        // 输出消抖后的状态
        // 消抖进行中时输出上次保存的状态，消抖完成后输出最新状态
        cc1_debounced_o <= cc1_state_debounced;
        cc2_debounced_o <= cc2_state_debounced;
        vbus_debounced_o <= vbus_state_sync;  // VBUS 简单同步
        connected_d <= connected;
        cc_changed_o <= 1'b0;  // 默认清除
        
        // 连接检测 (基于消抖后的状态)
        // 当 CC 状态不是 OPEN 时，表示有连接
        if (cc1_state_debounced < CC_VOLT_OPEN || cc2_state_debounced < CC_VOLT_OPEN) begin
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
        
        // 连接状态变化中断 (DRP 切换过程中不触发)
        // 只有在非 DRP 模式或连接稳定后才触发中断
        if (connected != connected_d) begin
            if (!drp_mode_i || connect_stable_cnt >= CONNECT_STABLE_CNT) begin
                // 非 DRP 模式，或 DRP 模式下连接已稳定
                cc_changed_o <= 1'b1;
            end
        end
    end
end

//============================================================================
// CC 终端控制 (6种模式)
// DRP 模式: 当 drp_mode_i=1 时，自动在 Rd 和 Rp 之间切换
//============================================================================
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
                // Rp 状态 (DFP): 使用配置的 Rp 模式
                cc1_mode_o <= cc1_mode_i;
                cc2_mode_o <= cc2_mode_i;
            end else begin
                // Rd 状态 (UFP)
                cc1_mode_o <= CC_MODE_RD;
                cc2_mode_o <= CC_MODE_RD;
            end
        end else begin
            // 非 DRP 模式: 直接输出配置的模式
            cc1_mode_o <= cc1_mode_i;
            cc2_mode_o <= cc2_mode_i;
            drp_is_rp <= 1'b0;
            drp_toggle_timer <= 32'd0;
        end
    end
end

endmodule
