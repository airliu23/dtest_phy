//============================================================================
// USB PD BMC 接收模块 (基于边沿检测)
// 与 pd_bmc_tx.v 配对使用，采用边沿间隔解码
//============================================================================

`timescale 1ns/1ps

module pd_bmc_rx (
    input  wire               clk,
    input  wire               rst_n,
    
    // 接收使能
    input  wire               rx_en_i,
    
    // SOP 类型过滤
    input  wire [2:0]         sop_type_i,       // 期望接收的 SOP 类型 (7=接收所有类型)
    output reg  [2:0]         rx_sop_type_o,    // 实际接收到的 SOP 类型
    
    // 接收完成包接口
    output reg  [15:0]        rx_header_o,
    output wire [239:0]       rx_data_flat_o,
    output reg  [4:0]         rx_data_len_o,
    output reg                rx_done_o,
    output reg                rx_busy_o,
    output reg                rx_error_o,
    output reg                rx_crc_ok_o,     // CRC 验证结果
    output reg  [2:0]         rx_msg_id_o,     // 接收到的 MessageID
    
    // 底层物理输入
    input  wire               bmc_rx_pad,
    
    // 调试接口 - 实时输出解析的byte
    input  wire               dbg_en_i,        // 调试使能
    output reg  [7:0]         dbg_byte_o,      // 当前解析的字节
    output reg                dbg_byte_valid_o, // 字节有效标志
    
    // RX 调试输出
    output reg                dbg_toggle_o        // 每收到有效数据位翻转一次
);

//============================================================================
// 参数定义 (与 TX 完全一致)
//============================================================================
parameter BIT_PERIOD_CNT  = 166;
parameter HALF_BIT_CNT    = 83;
parameter PREAMBLE_LEN    = 64;
parameter MAX_DATA_LEN    = 30;
parameter IDLE_WAIT_CNT   = 2500;   // EOP后等待总线空闲时间: 50us @ 50MHz
parameter BUS_TIMEOUT_CNT = 100000; // 总线空闲超时: 2ms @ 50MHz

// 4B5B K码定义
localparam KCODE_SYNC1  = 5'b11000;  // Sync-1
localparam KCODE_SYNC2  = 5'b10001;  // Sync-2
localparam KCODE_SYNC3  = 5'b00110;  // Sync-3
localparam KCODE_RST1   = 5'b00111;  // RST-1
localparam KCODE_RST2   = 5'b11001;  // RST-2
localparam KCODE_EOP    = 5'b01101;  // EOP

// SOP 类型定义
localparam SOP_TYPE_SOP         = 3'd0;  // SOP: Sync-1, Sync-1, Sync-1, Sync-2
localparam SOP_TYPE_SOP_PRIME   = 3'd1;  // SOP': Sync-1, Sync-1, Sync-3, Sync-3
localparam SOP_TYPE_SOP_DPRIME  = 3'd2;  // SOP'': Sync-1, Sync-3, Sync-1, Sync-3
localparam SOP_TYPE_SOP_PDBG    = 3'd3;  // SOP'_Debug: Sync-1, RST-2, RST-2, Sync-3
localparam SOP_TYPE_SOP_DPDBG   = 3'd4;  // SOP''_Debug: Sync-1, RST-2, Sync-3, Sync-2
localparam SOP_TYPE_CABLE_RESET = 3'd5;  // Cable Reset: RST-1, Sync-1, RST-1, Sync-3
localparam SOP_TYPE_HARD_RESET  = 3'd6;  // Hard Reset: RST-1, RST-1, RST-1, RST-2
localparam SOP_TYPE_ANY         = 3'd7;  // 接收任意 SOP 类型

// 接收状态机
localparam ST_IDLE      = 4'd0;
localparam ST_PREAMBLE  = 4'd1;
localparam ST_SOP       = 4'd2;  // 接收并识别 SOP 有序集
localparam ST_RECEIVE   = 4'd3;  // Header + Data + CRC，直到EOP
localparam ST_DONE      = 4'd4;

//============================================================================
// 边沿检测参数
//============================================================================
localparam HALF_BIT_MIN = HALF_BIT_CNT - 15;
localparam HALF_BIT_MAX = HALF_BIT_CNT + 15;
localparam FULL_BIT_MIN = BIT_PERIOD_CNT - 20;
localparam FULL_BIT_MAX = BIT_PERIOD_CNT + 20;

//============================================================================
// 输入同步与边沿检测
//============================================================================
reg [2:0]  pad_sync;
reg        prev_bmc_val;
wire       bmc_rx_synced;
wire       any_edge;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pad_sync <= 3'b111;
        prev_bmc_val <= 1'b1;
    end else begin
        pad_sync <= {pad_sync[1:0], bmc_rx_pad};
        prev_bmc_val <= bmc_rx_synced;
    end
end

assign bmc_rx_synced = pad_sync[2];
assign any_edge = (bmc_rx_synced != prev_bmc_val);

//============================================================================
// BMC 解码逻辑 (核心：基于边沿间隔)
// 原理：
//   - 检测到一个边沿后，测量到下一个边沿的时间间隔
//   - 间隔 ≈ 半周期 (83±15)：这是数据1的中间跳变，输出1，继续等待下一半周期
//   - 间隔 ≈ 全周期 (166±20)：这是数据0的周期跳变，输出0
//============================================================================

// BMC 解码状态
localparam BMC_IDLE     = 2'd0;  // 等待第一个边沿（周期开始）
localparam BMC_WAIT_1   = 2'd1;  // 等待第一个半周期（检测数据1的中间跳变）
localparam BMC_WAIT_0   = 2'd2;  // 等待第二个半周期（数据1的周期结束）

reg [1:0]  bmc_state;
reg [15:0] edge_timer;     // 边沿间隔计时器
reg        rx_bit;         // 解码后的数据bit
reg        rx_bit_valid;   // 数据有效标志

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bmc_state <= BMC_IDLE;
        edge_timer <= 16'd0;
        rx_bit <= 1'b0;
        rx_bit_valid <= 1'b0;
    end else if (rx_state == ST_IDLE || rx_state == ST_DONE) begin
        bmc_state <= BMC_IDLE;
        edge_timer <= 16'd0;
        rx_bit <= 1'b0;
        rx_bit_valid <= 1'b0;
    end else begin
        rx_bit_valid <= 1'b0;  // 默认无效
        
        case (bmc_state)
            BMC_IDLE: begin
                edge_timer <= 16'd0;
                if (any_edge) begin
                    // 检测到周期开始边沿
                    bmc_state <= BMC_WAIT_1;
                end
            end
            
            BMC_WAIT_1: begin
                edge_timer <= edge_timer + 1'b1;
                
                if (any_edge) begin
                    // 在半周期窗口内检测到边沿 → 数据=1
                    if (edge_timer >= HALF_BIT_MIN && edge_timer <= HALF_BIT_MAX) begin
                        rx_bit <= 1'b1;
                        rx_bit_valid <= 1'b1;
                        // 此边沿是数据1的中间跳变，同时也是下一周期的开始
                        // 直接进入 WAIT_0 等待周期结束
                        bmc_state <= BMC_WAIT_0;
                        edge_timer <= 16'd0;
                    end else begin
                        // 边沿时间不对，重置
                        bmc_state <= BMC_IDLE;
                    end
                end else if (edge_timer >= HALF_BIT_MAX) begin
                    // 半周期窗口结束，未检测到中间边沿 → 数据=0
                    rx_bit <= 1'b0;
                    rx_bit_valid <= 1'b1;
                    bmc_state <= BMC_IDLE;
                end
            end
            
            BMC_WAIT_0: begin
                edge_timer <= edge_timer + 1'b1;
                
                if (any_edge) begin
                    // 在半周期窗口内检测到边沿 → 这是下一周期的开始
                    if (edge_timer >= HALF_BIT_MIN && edge_timer <= HALF_BIT_MAX) begin
                        bmc_state <= BMC_WAIT_1;
                        edge_timer <= 16'd0;
                    end else begin
                        // 边沿时间不对，重置
                        bmc_state <= BMC_IDLE;
                    end
                end else if (edge_timer >= HALF_BIT_MAX) begin
                    // 超时，错误
                    bmc_state <= BMC_IDLE;
                end
            end
            
            default: bmc_state <= BMC_IDLE;
        endcase
    end
end

// 调试翻转信号 - 每收到有效数据位翻转一次
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dbg_toggle_o <= 1'b0;
    end else if (rx_bit_valid) begin
        dbg_toggle_o <= ~dbg_toggle_o;
    end
end

//============================================================================
// 接收状态机
//============================================================================
reg [3:0]  rx_state;
reg [15:0] idle_wait_cnt;  // EOP后等待总线空闲计数器
reg [16:0] bus_timeout_cnt; // 总线空闲超时计数器

// 位计数器
reg [5:0]  preamble_cnt;
reg [2:0]  symbol_bit_cnt;
reg [4:0]  byte_cnt;
reg        byte_nibble_sel;
reg [1:0]  sop_symbol_cnt;   // SOP 符号计数器 (0-3)

// 数据寄存器
reg [4:0]  current_symbol;
reg [7:0]  current_byte;
reg [15:0] rx_header_reg;
reg [31:0] crc_calc;         // 用于实时计算接收数据的 CRC
reg [31:0] rx_crc_received;  // 接收到的 CRC 值

// SOP 符号缓存 (4 个 5-bit 符号)
reg [4:0]  sop_symbols [0:3];

// 【修复1】声明内部数据数组，用于暂存接收数据
reg [7:0]  rx_data_reg [0:29];

//============================================================================
// SOP 类型识别函数
//============================================================================
function [2:0] identify_sop_type;
    input [4:0] sym0, sym1, sym2, sym3;
    begin
        // SOP: Sync-1, Sync-1, Sync-1, Sync-2
        if (sym0 == KCODE_SYNC1 && sym1 == KCODE_SYNC1 && sym2 == KCODE_SYNC1 && sym3 == KCODE_SYNC2)
            identify_sop_type = SOP_TYPE_SOP;
        // SOP': Sync-1, Sync-1, Sync-3, Sync-3
        else if (sym0 == KCODE_SYNC1 && sym1 == KCODE_SYNC1 && sym2 == KCODE_SYNC3 && sym3 == KCODE_SYNC3)
            identify_sop_type = SOP_TYPE_SOP_PRIME;
        // SOP'': Sync-1, Sync-3, Sync-1, Sync-3
        else if (sym0 == KCODE_SYNC1 && sym1 == KCODE_SYNC3 && sym2 == KCODE_SYNC1 && sym3 == KCODE_SYNC3)
            identify_sop_type = SOP_TYPE_SOP_DPRIME;
        // SOP'_Debug: Sync-1, RST-2, RST-2, Sync-3
        else if (sym0 == KCODE_SYNC1 && sym1 == KCODE_RST2 && sym2 == KCODE_RST2 && sym3 == KCODE_SYNC3)
            identify_sop_type = SOP_TYPE_SOP_PDBG;
        // SOP''_Debug: Sync-1, RST-2, Sync-3, Sync-2
        else if (sym0 == KCODE_SYNC1 && sym1 == KCODE_RST2 && sym2 == KCODE_SYNC3 && sym3 == KCODE_SYNC2)
            identify_sop_type = SOP_TYPE_SOP_DPDBG;
        // Cable Reset: RST-1, Sync-1, RST-1, Sync-3
        else if (sym0 == KCODE_RST1 && sym1 == KCODE_SYNC1 && sym2 == KCODE_RST1 && sym3 == KCODE_SYNC3)
            identify_sop_type = SOP_TYPE_CABLE_RESET;
        // Hard Reset: RST-1, RST-1, RST-1, RST-2
        else if (sym0 == KCODE_RST1 && sym1 == KCODE_RST1 && sym2 == KCODE_RST1 && sym3 == KCODE_RST2)
            identify_sop_type = SOP_TYPE_HARD_RESET;
        else
            identify_sop_type = SOP_TYPE_ANY;  // 无效 SOP，使用 7 表示
    end
endfunction

//============================================================================
// 数据输出映射
// rx_data_reg 布局: [0-1]:Header, [2...]:Data, [最后4字节]:CRC
// (SOP 由 ST_SOP 状态单独处理，不存入 rx_data_reg)
// rx_data_flat_o 输出实际数据部分 (从 rx_data_reg[2] 开始)
//============================================================================
genvar k;
generate
    for (k = 0; k < 26; k = k + 1) begin : gen_rx_data_map
        assign rx_data_flat_o[k*8 +: 8] = rx_data_reg[k + 2];
    end
    // 填充剩余位为 0
    for (k = 26; k < 30; k = k + 1) begin : gen_rx_data_pad
        assign rx_data_flat_o[k*8 +: 8] = 8'd0;
    end
endgenerate

//============================================================================
// CRC计算
//============================================================================
function [31:0] calc_crc32;
    input [31:0] crc_in;
    input [7:0]  data_in;
    integer i;
    begin
        calc_crc32 = crc_in;
        for (i = 0; i < 8; i = i + 1) begin
            if ((calc_crc32[0] ^ data_in[i]) == 1'b1) begin
                calc_crc32 = (calc_crc32 >> 1) ^ 32'hEDB88320;
            end else begin
                calc_crc32 = calc_crc32 >> 1;
            end
        end
    end
endfunction

//============================================================================
// 4B5B解码
//============================================================================
function [3:0] decode_4b5b_data;
    input [4:0] symbol_5b;
    begin
        case (symbol_5b)
            5'b11110: decode_4b5b_data = 4'b0000;
            5'b01001: decode_4b5b_data = 4'b0001;
            5'b10100: decode_4b5b_data = 4'b0010;
            5'b10101: decode_4b5b_data = 4'b0011;
            5'b01010: decode_4b5b_data = 4'b0100;
            5'b01011: decode_4b5b_data = 4'b0101;
            5'b01110: decode_4b5b_data = 4'b0110;
            5'b01111: decode_4b5b_data = 4'b0111;
            5'b10010: decode_4b5b_data = 4'b1000;
            5'b10011: decode_4b5b_data = 4'b1001;
            5'b10110: decode_4b5b_data = 4'b1010;
            5'b10111: decode_4b5b_data = 4'b1011;
            5'b11010: decode_4b5b_data = 4'b1100;
            5'b11011: decode_4b5b_data = 4'b1101;
            5'b11100: decode_4b5b_data = 4'b1110;
            5'b11101: decode_4b5b_data = 4'b1111;
            default:  decode_4b5b_data = 4'b0000;
        endcase
    end
endfunction

integer i; // 循环变量声明移到 always 块内部

//============================================================================
// 主状态机
//============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= ST_IDLE;
        rx_done_o <= 1'b0;
        rx_busy_o <= 1'b0;
        rx_error_o <= 1'b0;
        rx_header_o <= 16'd0;
        rx_data_len_o <= 5'd0;
        rx_sop_type_o <= 3'd0;
        preamble_cnt <= 6'd0;
        symbol_bit_cnt <= 3'd0;
        sop_symbol_cnt <= 2'd0;
        byte_cnt <= 5'd0;
        byte_nibble_sel <= 1'b0;
        current_symbol <= 5'd0;
        current_byte <= 8'd0;
        rx_header_reg <= 16'd0;
        idle_wait_cnt <= 16'd0;
        bus_timeout_cnt <= 17'd0;
        
        // 复位 SOP 符号缓存
        for (i = 0; i < 4; i = i + 1) begin
            sop_symbols[i] <= 5'd0;
        end
        
        // 复位内部数组 rx_data_reg
        for (i = 0; i < 30; i = i + 1) begin
            rx_data_reg[i] <= 8'd0;
        end
        
        // 复位调试信号
        dbg_byte_o <= 8'd0;
        dbg_byte_valid_o <= 1'b0;
    end else begin
        rx_done_o <= 1'b0;
        dbg_byte_valid_o <= 1'b0;  // 默认清除调试有效标志
        
        // 总线空闲超时检测: 任意时刻总线 idle 2ms 回到默认状态
        if (rx_state != ST_IDLE) begin
            if (any_edge) begin
                bus_timeout_cnt <= 17'd0;
            end else if (bus_timeout_cnt >= BUS_TIMEOUT_CNT - 1) begin
                // 超时，回到 IDLE
                bus_timeout_cnt <= 17'd0;
                rx_busy_o <= 1'b0;
                rx_error_o <= 1'b1;  // 标记超时错误
                rx_state <= ST_IDLE;
            end else begin
                bus_timeout_cnt <= bus_timeout_cnt + 1'b1;
            end
        end else begin
            bus_timeout_cnt <= 17'd0;
        end
        
        case (rx_state)
            ST_IDLE: begin
                rx_busy_o <= 1'b0;
                rx_error_o <= 1'b0;
                rx_crc_ok_o <= 1'b0;
                preamble_cnt <= 6'd0;
                symbol_bit_cnt <= 3'd0;
                sop_symbol_cnt <= 2'd0;
                crc_calc <= 32'hFFFFFFFF;
                // 添加更完整的状态清除
                byte_cnt <= 5'd0;
                byte_nibble_sel <= 1'b0;
                current_symbol <= 5'd0;
                
                // 只在检测到下降沿（从高到低）时开始接收
                // USB PD preamble 的第一个边沿是从空闲高电平到低电平
                if (rx_en_i && prev_bmc_val && !bmc_rx_synced) begin
                    rx_busy_o <= 1'b1;
                    rx_state <= ST_PREAMBLE;
                end
            end
            
            ST_PREAMBLE: begin
                // 如果 RX 被禁用，返回 IDLE
                if (!rx_en_i) begin
                    rx_busy_o <= 1'b0;
                    rx_state <= ST_IDLE;
                end
                else if (rx_bit_valid) begin
                    preamble_cnt <= preamble_cnt + 1'b1;
                    
                    if (preamble_cnt == PREAMBLE_LEN - 2) begin
                        byte_cnt <= 5'd0;
                        byte_nibble_sel <= 1'b0;
                        symbol_bit_cnt <= 3'd0;
                        sop_symbol_cnt <= 2'd0;
                        current_symbol <= 5'd0;
                        rx_state <= ST_SOP;
                    end
                end
            end
            
            // 接收 SOP 有序集 (4 个 K 码符号)
            ST_SOP: begin
                if (rx_bit_valid) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，新位插入 MSB
                    current_symbol <= {rx_bit, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        // 保存当前 SOP 符号 (使用完整的新符号值，包含第5位)
                        sop_symbols[sop_symbol_cnt] <= {rx_bit, current_symbol[4:1]};
                        sop_symbol_cnt <= sop_symbol_cnt + 1'b1;
                        
                        if (sop_symbol_cnt == 2'd3) begin
                            // 所有 4 个 SOP 符号接收完成，识别 SOP 类型
                            // 注意: sop_symbols[3] 尚未更新，使用即时计算值
                            rx_sop_type_o <= identify_sop_type(sop_symbols[0], sop_symbols[1], sop_symbols[2], {rx_bit, current_symbol[4:1]});
                            
                            // 检查是否匹配期望的 SOP 类型
                            if (sop_type_i == SOP_TYPE_ANY || 
                                sop_type_i == identify_sop_type(sop_symbols[0], sop_symbols[1], sop_symbols[2], {rx_bit, current_symbol[4:1]})) begin
                                
                                // HARD_RESET 和 CABLE_RESET 没有 Header/Data/CRC/EOP
                                if (identify_sop_type(sop_symbols[0], sop_symbols[1], sop_symbols[2], {rx_bit, current_symbol[4:1]}) == SOP_TYPE_HARD_RESET ||
                                    identify_sop_type(sop_symbols[0], sop_symbols[1], sop_symbols[2], {rx_bit, current_symbol[4:1]}) == SOP_TYPE_CABLE_RESET) begin
                                    // 直接完成，设置默认值
                                    rx_header_o <= 16'h0000;
                                    rx_data_len_o <= 5'd0;
                                    rx_crc_ok_o <= 1'b1;  // Reset 消息无 CRC，视为有效
                                    rx_done_o <= 1'b1;
                                    rx_state <= ST_DONE;
                                end else begin
                                    // SOP 匹配，继续接收数据
                                    byte_cnt <= 5'd0;
                                    byte_nibble_sel <= 1'b0;
                                    current_symbol <= 5'd0;
                                    rx_state <= ST_RECEIVE;
                                end
                            end else begin
                                // SOP 不匹配，丢弃消息
                                rx_busy_o <= 1'b0;
                                rx_state <= ST_IDLE;
                            end
                        end
                    end
                end
            end
            
            // 接收 Header + Data + CRC，直到检测到 EOP
            ST_RECEIVE: begin
                if (rx_bit_valid) begin
                    symbol_bit_cnt <= symbol_bit_cnt + 1'b1;
                    // 右移，新位插入 MSB
                    current_symbol <= {rx_bit, current_symbol[4:1]};
                    
                    if (symbol_bit_cnt == 3'd4) begin
                        symbol_bit_cnt <= 3'd0;
                        // 使用完整的 5-bit 符号 (包含刚接收的第5位)
                        // 注意: current_symbol 是非阻塞赋值的旧值，需要手动计算新值
                        // 检查是否是 EOP KCODE
                        if ({rx_bit, current_symbol[4:1]} == KCODE_EOP) begin
                            // 数据结构: Header(2字节) + Data(N字节) + CRC(4字节)
                            // (SOP 已由 ST_SOP 单独处理)
                            // rx_data_len_o = Header(2) + Data(N) = byte_cnt - CRC(4)
                            rx_data_len_o <= (byte_cnt > 4) ? (byte_cnt - 4) : 0;
                            // 不立即设置 rx_done_o，等待 ST_DONE 状态 200us idle 后再触发
                            rx_busy_o <= 1'b0;
                            
                            if (byte_cnt >= 4) begin
                                // 提取 Header (前2字节, 即 rx_data_reg[0-1])
                                rx_header_o <= {rx_data_reg[1], rx_data_reg[0]};
                                // rx_msg_id_o <= rx_data_reg[0][6:4];  // MessageID 在 Header 低字节 [14:12]
                                
                                rx_crc_received = {rx_data_reg[byte_cnt - 1],rx_data_reg[byte_cnt - 2],rx_data_reg[byte_cnt - 3],rx_data_reg[byte_cnt - 4]};
                                // 使用固定上限循环，避免综合器无法确定迭代次数
                                // CRC 计算从 Header 开始 (index 0)
                                for (i = 0; i < MAX_DATA_LEN; i = i + 1) begin
                                    if (i < byte_cnt - 4) begin
                                        crc_calc = calc_crc32(crc_calc, rx_data_reg[i]);
                                    end
                                end
                                crc_calc = ~crc_calc;
                                rx_crc_ok_o = (crc_calc == rx_crc_received);
                            end else begin
                                // byte_cnt < 4, 跳过 CRC 检查
                            end
                            idle_wait_cnt <= 16'd0;  // 初始化等待计数器
                            rx_state <= ST_DONE;
                        end else begin
                            if (!byte_nibble_sel) begin
                                current_byte[3:0] = decode_4b5b_data({rx_bit, current_symbol[4:1]});
                            end else begin
                                current_byte[7:4] = decode_4b5b_data({rx_bit, current_symbol[4:1]});
                                // 保存到内部数组 rx_data_reg
                                rx_data_reg[byte_cnt] <= {decode_4b5b_data({rx_bit, current_symbol[4:1]}), current_byte[3:0]};
                                
                                // 调试输出 - 高字节时输出完整byte
                                if (dbg_en_i) begin
                                    dbg_byte_o <= {decode_4b5b_data({rx_bit, current_symbol[4:1]}), current_byte[3:0]};
                                    dbg_byte_valid_o <= 1'b1;
                                end
                                
                                byte_cnt <= byte_cnt + 1'b1;
                                
                                if (byte_cnt >= MAX_DATA_LEN - 1) begin
                                    rx_error_o <= 1'b1;
                                    rx_state <= ST_IDLE;
                                end
                            end
                            byte_nibble_sel <= ~byte_nibble_sel;
                        end
                    end
                end
            end
            
            ST_DONE: begin
                rx_error_o <= 1'b0;
                if (!rx_en_i) begin
                    // RX 被禁用，直接回到 IDLE，不触发 rx_done
                    idle_wait_cnt <= 16'd0;
                    rx_state <= ST_IDLE;
                end
                // 检测到下降沿（新消息开始），触发 rx_done 并回到 IDLE 准备接收
                else if (prev_bmc_val && !bmc_rx_synced) begin
                    rx_done_o <= 1'b1;  // 触发 rx_done 表示前一条消息接收完成
                    idle_wait_cnt <= 16'd0;
                    rx_state <= ST_IDLE;
                end
                // 总线稳定在高电平时计数，等待确认消息结束
                else if (bmc_rx_synced && !any_edge) begin
                    if (idle_wait_cnt >= IDLE_WAIT_CNT - 1) begin
                        // 等待完成，触发 rx_done_o，回到 IDLE
                        rx_done_o <= 1'b1;
                        idle_wait_cnt <= 16'd0;
                        rx_state <= ST_IDLE;
                    end else begin
                        idle_wait_cnt <= idle_wait_cnt + 1'b1;
                    end
                end else begin
                    // 总线有边沿但不是下降沿，重置计数器
                    idle_wait_cnt <= 16'd0;
                end
            end
            
            default: rx_state <= ST_IDLE;
        endcase
    end
end

endmodule