//============================================================================
// USB PD BMC 收发器
//============================================================================

`timescale 1ns/1ps

// Simulation model for IBUFDS (Xilinx primitive)
`ifdef SIMULATION
module IBUFDS (
    input  wire I,
    input  wire IB,
    output wire O
);
    // In simulation, just use the positive input
    assign O = I;
endmodule
`endif

module pd_bmc_transceiver (
    input  wire               sys_clk_p,
    input  wire               sys_clk_n,
    input  wire               rst_n,
    output wire               led_tx,
    output wire               led_rx_ok,
    output wire               led_rx_err,
    output wire               led_heartbeat,
    output wire               bmc_tx_pad,
    input  wire               bmc_rx_pad
);

wire sys_clk_200m;
IBUFDS u_ibufds_sys_clk (
    .I(sys_clk_p),
    .IB(sys_clk_n),
    .O(sys_clk_200m)
);

reg [1:0] clk_div_cnt;
reg clk;
always @(posedge sys_clk_200m or negedge rst_n) begin
    if(!rst_n) begin
        clk_div_cnt <= 2'd0;
        clk <= 1'b0;
    end else begin
        if(clk_div_cnt == 2'd1) begin
            clk_div_cnt <= 2'd0;
            clk <= ~clk;
        end else begin
            clk_div_cnt <= clk_div_cnt + 1'b1;
        end
    end
end

reg [24:0] heartbeat_cnt;
reg        heartbeat_reg;
always @(posedge clk or negedge rst_n) begin
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

reg [15:0]  tx_header;
reg [239:0] tx_data_flat;
reg [4:0]   tx_data_len;
reg         tx_start;
wire        tx_done;
wire        tx_busy;

wire [15:0]  rx_header;
wire [239:0] rx_data_flat;
wire [4:0]   rx_data_len;
wire         rx_done;
wire         rx_busy;
wire         rx_error;
reg          rx_en;

reg [31:0] delay_cnt;
reg [1:0]  test_state;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_header <= 16'h03A3;
        tx_data_len <= 5'd0;
        tx_data_flat <= 240'h00000000000000000000000000000000000000000000000000AA55;
        tx_start <= 1'b0;
        rx_en <= 1'b1;
        delay_cnt <= 32'd0;
        test_state <= 2'd0;
    end else begin
        case (test_state)
            2'd0: begin
                delay_cnt <= delay_cnt + 1'b1;
                if (delay_cnt == 32'd1000) begin  // 仿真时使用短延迟
                    test_state <= 2'd1;
                end
            end
            2'd1: begin
                tx_start <= 1'b1;
                if (tx_busy) begin
                    test_state <= 2'd2;
                end
            end
            2'd2: begin
                tx_start <= 1'b0;
                test_state <= 2'd2;
            end
            default: test_state <= 2'd0;
        endcase
    end
end

// LED 控制 (低电平点亮)
assign led_tx = ~tx_busy;
assign led_rx_ok = ~rx_done;
assign led_rx_err = ~rx_error;

wire tx_active;
pd_bmc_tx u_pd_bmc_tx (
    .clk            (clk),
    .rst_n          (rst_n),
    .tx_header_i    (tx_header),
    .tx_data_flat_i (tx_data_flat),
    .tx_data_len_i  (tx_data_len),
    .tx_start_i     (tx_start),
    .tx_done_o      (tx_done),
    .tx_busy_o      (tx_busy),
    .bmc_tx_pad     (bmc_tx_pad),
    .tx_active_o    (tx_active)
);

pd_bmc_rx u_pd_bmc_rx (
    .clk            (clk),
    .rst_n          (rst_n),
    .rx_header_o    (rx_header),
    .rx_data_flat_o (rx_data_flat),
    .rx_data_len_o  (rx_data_len),
    .rx_done_o      (rx_done),
    .rx_busy_o      (rx_busy),
    .rx_error_o     (rx_error),
    .bmc_rx_pad     (bmc_rx_pad),
    .rx_en_i        (rx_en)
);

endmodule