//============================================================================
// SPI 从机接口模块
// 支持 Mode 0 (CPOL=0, CPHA=0)
// 协议: [1字节地址][1字节数据] (MSB first)
// 地址最高位: 0=读, 1=写
//============================================================================

`timescale 1ns/1ps

module spi_slave (
    input  wire        clk,           // 系统时钟
    input  wire        rst_n,         // 异步复位
    
    // SPI 接口
    input  wire        spi_sclk,      // SPI 时钟
    input  wire        spi_cs_n,      // SPI 片选 (低有效)
    input  wire        spi_mosi,      // 主出从入
    output reg         spi_miso,      // 主入从出
    
    // 寄存器接口
    output reg         reg_wr_en,     // 寄存器写使能
    output reg         reg_rd_en,     // 寄存器读使能
    output reg  [7:0]  reg_addr,      // 寄存器地址
    output reg  [7:0]  reg_wr_data,   // 写数据
    input  wire [7:0]  reg_rd_data    // 读数据
);

//============================================================================
// SPI 信号同步 (跨时钟域)
//============================================================================
reg [2:0] sclk_sync;
reg [2:0] cs_n_sync;
reg [1:0] mosi_sync;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sclk_sync <= 3'b000;
        cs_n_sync <= 3'b111;
        mosi_sync <= 2'b00;
    end else begin
        sclk_sync <= {sclk_sync[1:0], spi_sclk};
        cs_n_sync <= {cs_n_sync[1:0], spi_cs_n};
        mosi_sync <= {mosi_sync[0], spi_mosi};
    end
end

// 边沿检测
wire sclk_rising  = (sclk_sync[2:1] == 2'b01);
wire sclk_falling = (sclk_sync[2:1] == 2'b10);
wire cs_n_active  = ~cs_n_sync[2];
wire cs_n_rising  = (cs_n_sync[2:1] == 2'b01);
wire mosi_data    = mosi_sync[1];

//============================================================================
// SPI 状态机
//============================================================================
localparam STATE_IDLE      = 3'd0;
localparam STATE_ADDR      = 3'd1;
localparam STATE_WAIT_DATA = 3'd2;
localparam STATE_DATA      = 3'd3;
localparam STATE_DONE      = 3'd4;

reg [2:0]  state;
reg [2:0]  bit_cnt;
reg [7:0]  shift_in;
reg [7:0]  shift_out;
reg        is_write;
reg        miso_loaded;  // 标记 MISO 是否已加载

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= STATE_IDLE;
        bit_cnt     <= 3'd0;
        shift_in    <= 8'd0;
        shift_out   <= 8'd0;
        is_write    <= 1'b0;
        reg_addr    <= 8'd0;
        reg_wr_data <= 8'd0;
        reg_wr_en   <= 1'b0;
        reg_rd_en   <= 1'b0;
        spi_miso    <= 1'b0;
        miso_loaded <= 1'b0;
    end else begin
        // 默认清除使能信号
        reg_wr_en <= 1'b0;
        reg_rd_en <= 1'b0;
        
        // CS 释放时复位
        if (!cs_n_active) begin
            state       <= STATE_IDLE;
            bit_cnt     <= 3'd0;
            spi_miso    <= 1'b0;
            miso_loaded <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    bit_cnt     <= 3'd0;
                    shift_in    <= 8'd0;
                    miso_loaded <= 1'b0;
                    state       <= STATE_ADDR;
                end
                
                STATE_ADDR: begin
                    // SCLK 上升沿采样 MOSI
                    if (sclk_rising) begin
                        shift_in <= {shift_in[6:0], mosi_data};
                        bit_cnt  <= bit_cnt + 1'b1;
                        
                        if (bit_cnt == 3'd7) begin
                            // 地址字节接收完成
                            reg_addr <= {shift_in[5:0], mosi_data};
                            is_write <= shift_in[6];  // bit7 是读写标志
                            state    <= STATE_WAIT_DATA;
                            bit_cnt  <= 3'd0;
                        end
                    end
                end
                
                STATE_WAIT_DATA: begin
                    // 如果是读操作，触发读取
                    if (!is_write) begin
                        reg_rd_en <= 1'b1;
                    end
                    state <= STATE_DATA;
                end
                
                STATE_DATA: begin
                    // 读操作
                    if (!is_write) begin
                        if (!miso_loaded) begin
                            // 第一次进入：加载数据，立即输出 MSB
                            shift_out   <= {reg_rd_data[6:0], 1'b0};
                            spi_miso    <= reg_rd_data[7];
                            miso_loaded <= 1'b1;
                        end else if (sclk_falling && bit_cnt > 0) begin
                            // 在上升沿采样后，下降沿更新为下一个 bit
                            spi_miso  <= shift_out[7];
                            shift_out <= {shift_out[6:0], 1'b0};
                        end
                    end
                    
                    // SCLK 上升沿采样 MOSI
                    if (sclk_rising) begin
                        shift_in <= {shift_in[6:0], mosi_data};
                        bit_cnt  <= bit_cnt + 1'b1;
                        
                        if (bit_cnt == 3'd7) begin
                            // 数据字节接收完成
                            if (is_write) begin
                                reg_wr_data <= {shift_in[6:0], mosi_data};
                                reg_wr_en   <= 1'b1;
                            end
                            state   <= STATE_DONE;
                            bit_cnt <= 3'd0;
                        end
                    end
                end
                
                STATE_DONE: begin
                    // 等待 CS 释放
                    spi_miso <= 1'b0;
                end
                
                default: state <= STATE_IDLE;
            endcase
        end
    end
end

endmodule
