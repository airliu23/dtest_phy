//============================================================================
// I2C 从机接口模块
// 7位地址: 0x28
// 协议: [寄存器地址][数据]
// 写: START + 0x50 + REG_ADDR + DATA + STOP
// 读: START + 0x50 + REG_ADDR + RESTART + 0x51 + DATA + STOP
//============================================================================

`timescale 1ns/1ps

module i2c_slave #(
    parameter SLAVE_ADDR = 7'h28
)(
    input  wire        clk,           // 系统时钟
    input  wire        rst_n,         // 异步复位
    
    // I2C 接口
    input  wire        i2c_scl,       // I2C 时钟
    inout  wire        i2c_sda,       // I2C 数据 (双向)
    
    // 寄存器接口
    output reg         reg_wr_en,     // 寄存器写使能
    output reg         reg_rd_en,     // 寄存器读使能
    output reg  [7:0]  reg_addr,      // 寄存器地址
    output reg  [7:0]  reg_wr_data,   // 写数据
    input  wire [7:0]  reg_rd_data    // 读数据
);

//============================================================================
// I2C 信号同步
//============================================================================
reg [2:0] scl_sync;
reg [2:0] sda_sync;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        scl_sync <= 3'b111;
        sda_sync <= 3'b111;
    end else begin
        scl_sync <= {scl_sync[1:0], i2c_scl};
        sda_sync <= {sda_sync[1:0], i2c_sda};
    end
end

wire scl = scl_sync[2];
wire sda_in = sda_sync[2];
wire scl_rising  = (scl_sync[2:1] == 2'b01);
wire scl_falling = (scl_sync[2:1] == 2'b10);

// START/STOP 检测 (SDA 在 SCL 高电平时变化)
wire start_cond = scl && (sda_sync[2:1] == 2'b10);  // SDA 下降沿
wire stop_cond  = scl && (sda_sync[2:1] == 2'b01);  // SDA 上升沿

//============================================================================
// SDA 输出控制
//============================================================================
reg sda_out;
reg sda_oe;  // 输出使能

assign i2c_sda = sda_oe ? sda_out : 1'bz;

//============================================================================
// I2C 状态机
//============================================================================
localparam STATE_IDLE      = 4'd0;
localparam STATE_DEV_ADDR  = 4'd1;
localparam STATE_DEV_ACK   = 4'd2;
localparam STATE_REG_ADDR  = 4'd3;
localparam STATE_REG_ACK   = 4'd4;
localparam STATE_WR_DATA   = 4'd5;
localparam STATE_WR_ACK    = 4'd6;
localparam STATE_RD_DATA   = 4'd7;
localparam STATE_RD_ACK    = 4'd8;

reg [3:0]  state;
reg [3:0]  bit_cnt;
reg [7:0]  shift_reg;
reg        is_read;
reg        addr_match;
reg        got_reg_addr;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= STATE_IDLE;
        bit_cnt      <= 4'd0;
        shift_reg    <= 8'd0;
        is_read      <= 1'b0;
        addr_match   <= 1'b0;
        got_reg_addr <= 1'b0;
        reg_addr     <= 8'd0;
        reg_wr_data  <= 8'd0;
        reg_wr_en    <= 1'b0;
        reg_rd_en    <= 1'b0;
        sda_out      <= 1'b1;
        sda_oe       <= 1'b0;
    end else begin
        // 默认清除
        reg_wr_en <= 1'b0;
        reg_rd_en <= 1'b0;
        
        // START 条件检测
        if (start_cond) begin
            state        <= STATE_DEV_ADDR;
            bit_cnt      <= 4'd0;
            shift_reg    <= 8'd0;
            addr_match   <= 1'b0;
            sda_oe       <= 1'b0;
            // 如果已经有寄存器地址，这是 repeated start (读操作)
            // got_reg_addr 保持不变
        end
        
        // STOP 条件检测
        if (stop_cond) begin
            state        <= STATE_IDLE;
            got_reg_addr <= 1'b0;
            sda_oe       <= 1'b0;
        end
        
        case (state)
            STATE_IDLE: begin
                sda_oe <= 1'b0;
            end
            
            STATE_DEV_ADDR: begin
                if (scl_rising) begin
                    shift_reg <= {shift_reg[6:0], sda_in};
                    bit_cnt   <= bit_cnt + 1'b1;
                    
                    if (bit_cnt == 4'd7) begin
                        // 检查地址匹配 (7位地址 + R/W位)
                        if (shift_reg[6:0] == SLAVE_ADDR) begin
                            addr_match <= 1'b1;
                            is_read    <= sda_in;  // bit 0 是 R/W
                            state      <= STATE_DEV_ACK;
                        end else begin
                            state <= STATE_IDLE;
                        end
                        bit_cnt <= 4'd0;
                    end
                end
            end
            
            STATE_DEV_ACK: begin
                if (scl_falling) begin
                    sda_out <= 1'b0;  // ACK
                    sda_oe  <= 1'b1;
                end
                if (scl_rising) begin
                    // ACK 已发送
                end
                if (scl_falling && sda_oe) begin
                    if (is_read && got_reg_addr) begin
                        // 读操作，触发寄存器读取
                        reg_rd_en <= 1'b1;
                        state     <= STATE_RD_DATA;
                        shift_reg <= reg_rd_data;
                        // 立即设置 SDA 输出（不释放，直接输出第一个 bit）
                        sda_out   <= reg_rd_data[7];
                        // sda_oe 保持为 1
                    end else begin
                        // 写操作或需要先接收寄存器地址
                        sda_oe <= 1'b0;
                        state  <= STATE_REG_ADDR;
                    end
                    bit_cnt <= 4'd0;
                end
            end
            
            STATE_REG_ADDR: begin
                if (scl_rising) begin
                    shift_reg <= {shift_reg[6:0], sda_in};
                    bit_cnt   <= bit_cnt + 1'b1;
                    
                    if (bit_cnt == 4'd7) begin
                        reg_addr     <= {shift_reg[6:0], sda_in};
                        got_reg_addr <= 1'b1;
                        state        <= STATE_REG_ACK;
                        bit_cnt      <= 4'd0;
                    end
                end
            end
            
            STATE_REG_ACK: begin
                if (scl_falling) begin
                    sda_out <= 1'b0;  // ACK
                    sda_oe  <= 1'b1;
                end
                if (scl_falling && sda_oe) begin
                    sda_oe  <= 1'b0;
                    state   <= STATE_WR_DATA;
                    bit_cnt <= 4'd0;
                end
            end
            
            STATE_WR_DATA: begin
                if (scl_rising) begin
                    shift_reg <= {shift_reg[6:0], sda_in};
                    bit_cnt   <= bit_cnt + 1'b1;
                    
                    if (bit_cnt == 4'd7) begin
                        reg_wr_data <= {shift_reg[6:0], sda_in};
                        reg_wr_en   <= 1'b1;
                        state       <= STATE_WR_ACK;
                        bit_cnt     <= 4'd0;
                    end
                end
            end
            
            STATE_WR_ACK: begin
                if (scl_falling) begin
                    sda_out <= 1'b0;  // ACK
                    sda_oe  <= 1'b1;
                end
                if (scl_falling && sda_oe) begin
                    sda_oe   <= 1'b0;
                    reg_addr <= reg_addr + 1'b1;  // 自动递增地址
                    state    <= STATE_WR_DATA;
                    bit_cnt  <= 4'd0;
                end
            end
            
            STATE_RD_DATA: begin
                if (scl_falling) begin
                    sda_out <= shift_reg[7];
                    sda_oe  <= 1'b1;
                end
                if (scl_rising) begin
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    bit_cnt   <= bit_cnt + 1'b1;
                    
                    if (bit_cnt == 4'd7) begin
                        state   <= STATE_RD_ACK;
                        bit_cnt <= 4'd0;
                    end
                end
            end
            
            STATE_RD_ACK: begin
                if (scl_falling) begin
                    sda_oe <= 1'b0;  // 释放 SDA，等待主机 ACK/NACK
                end
                if (scl_rising) begin
                    if (sda_in == 1'b0) begin
                        // 主机发送 ACK，继续读下一个字节
                        reg_addr  <= reg_addr + 1'b1;
                    end else begin
                        // 主机发送 NACK，结束读取
                        state <= STATE_IDLE;
                    end
                    bit_cnt <= 4'd0;
                end
                // 在下一个 scl_falling 时准备下一个字节
                if (scl_falling && !sda_oe) begin
                    // 检查之前是否收到了 ACK（通过 state 判断）
                    if (state != STATE_IDLE) begin
                        reg_rd_en <= 1'b1;
                        state     <= STATE_RD_DATA;
                        shift_reg <= reg_rd_data;
                        sda_out   <= reg_rd_data[7];
                        sda_oe    <= 1'b1;
                    end
                end
            end
            
            default: state <= STATE_IDLE;
        endcase
    end
end

endmodule
