//============================================================================
// PD PHY CC 模块测试平台
// 专门验证 CC 检测和 DRP 模式
//============================================================================

`timescale 1ns/1ps

module pd_phy_tb;

//============================================================================
// 时钟和复位
//============================================================================
reg clk;
reg rst_n;

initial begin
    clk = 0;
    forever #10 clk = ~clk;  // 50MHz
end

initial begin
    rst_n = 0;
    #100 rst_n = 1;
end

//============================================================================
// CC 模块 (直接测试)
//============================================================================

// CC 输入
reg  [2:0] cc1_state;
reg  [2:0] cc2_state;
reg  [2:0] cc1_mode_cfg;
reg  [2:0] cc2_mode_cfg;
reg        drp_mode;

// CC 输出
wire [2:0] cc1_mode;
wire [2:0] cc2_mode;
wire [2:0] cc1_debounced;
wire [2:0] cc2_debounced;
wire       cc_changed;

// VBUS 接口
reg  [1:0] vbus_state;
wire [1:0] vbus_debounced;

// 直接实例化 CC 模块进行测试
cc u_cc (
    .clk                (clk),
    .rst_n              (rst_n),
    
    // CC 输入
    .cc1_state_i        (cc1_state),
    .cc2_state_i        (cc2_state),
    
    // CC 输出
    .cc1_debounced_o    (cc1_debounced),
    .cc2_debounced_o    (cc2_debounced),
    .cc_changed_o       (cc_changed),
    
    // VBUS
    .vbus_state_i       (vbus_state),
    .vbus_debounced_o   (vbus_debounced),
    
    // 配置
    .cc1_mode_i         (cc1_mode_cfg),
    .cc2_mode_i         (cc2_mode_cfg),
    .drp_mode_i         (drp_mode),
    
    // 控制输出
    .cc1_mode_o         (cc1_mode),
    .cc2_mode_o         (cc2_mode)
);

//============================================================================
// 测试任务
//============================================================================

// 等待时钟周期
task wait_cycles;
    input [31:0] n;
    begin
        repeat(n) @(posedge clk);
    end
endtask

// 测试 CC 模式输出 (非 DRP)
task test_cc_mode_output;
    begin
        $display("[Test] =========================================");
        $display("[Test] Test 1: CC Mode Output (Non-DRP)");
        $display("[Test] =========================================");
        
        drp_mode = 0;
        cc1_mode_cfg = 3'b101;  // Rp 3A
        cc2_mode_cfg = 3'b100;  // Rp 1.5A
        
        wait_cycles(10);
        
        if (cc1_mode == 3'b101 && cc2_mode == 3'b100) begin
            $display("[Test] PASS: CC mode output correct (CC1=0x%h, CC2=0x%h)", cc1_mode, cc2_mode);
        end else begin
            $display("[Test] FAIL: CC mode output wrong (CC1=0x%h expected 0x5, CC2=0x%h expected 0x4)", cc1_mode, cc2_mode);
        end
        
        // 测试 Rd 模式
        cc1_mode_cfg = 3'b001;  // Rd
        cc2_mode_cfg = 3'b001;  // Rd
        wait_cycles(10);
        
        if (cc1_mode == 3'b001 && cc2_mode == 3'b001) begin
            $display("[Test] PASS: Rd mode output correct (CC1=0x%h, CC2=0x%h)", cc1_mode, cc2_mode);
        end else begin
            $display("[Test] FAIL: Rd mode output wrong (CC1=0x%h, CC2=0x%h)", cc1_mode, cc2_mode);
        end
    end
endtask

// 测试 DRP 模式
task test_drp_mode;
    begin
        $display("[Test] =========================================");
        $display("[Test] Test 2: DRP Mode Toggle");
        $display("[Test] =========================================");
        
        // 配置 DRP 模式
        cc1_mode_cfg = 3'b101;  // Rp 3A (DFP 模式时使用)
        cc2_mode_cfg = 3'b101;
        drp_mode = 1;
        
        // 初始应该为 Rd 状态
        wait_cycles(100);
        
        if (cc1_mode == 3'b001) begin
            $display("[Test] PASS: DRP initial state is Rd (0x%h)", cc1_mode);
        end else begin
            $display("[Test] FAIL: DRP initial state wrong (expected Rd=0x1, got 0x%h)", cc1_mode);
        end
        
        // 等待切换到 Rp (50ms)
        $display("[Test] Waiting for DRP toggle to Rp (50ms)...");
        wait_cycles(3000000);  // 50ms @ 50MHz
        
        if (cc1_mode == 3'b101) begin
            $display("[Test] PASS: DRP switched to Rp (0x%h)", cc1_mode);
        end else begin
            $display("[Test] FAIL: DRP mode wrong (expected Rp=0x5, got 0x%h)", cc1_mode);
        end
        
        // 等待切换回 Rd (50ms)
        $display("[Test] Waiting for DRP toggle back to Rd (50ms)...");
        wait_cycles(2500000);  // 50ms @ 50MHz
        
        if (cc1_mode == 3'b001) begin
            $display("[Test] PASS: DRP switched back to Rd (0x%h)", cc1_mode);
        end else begin
            $display("[Test] FAIL: DRP mode wrong (expected Rd=0x1, got 0x%h)", cc1_mode);
        end
        
        drp_mode = 0;
        wait_cycles(100);
    end
endtask

// 测试 CC 状态同步
task test_cc_state_sync;
    begin
        $display("[Test] =========================================");
        $display("[Test] Test 3: CC State Sync");
        $display("[Test] =========================================");
        
        drp_mode = 0;
        cc1_mode_cfg = 3'b101;  // Rp 3A
        cc2_mode_cfg = 3'b101;
        
        // 初始状态: CC 开路
        cc1_state = 3'b100;  // > 3V (OPEN)
        cc2_state = 3'b100;
        
        // 等待消抖完成 (2ms + 余量)
        $display("[Test] Waiting for initial debounce (2ms)...");
        wait_cycles(110000);  // 2.2ms @ 50MHz
        $display("[Test] Initial CC1 state: input=0x%h, debounced=0x%h", cc1_state, cc1_debounced);
        
        if (cc1_debounced == 3'b100) begin
            $display("[Test] PASS: Initial CC1 state correct (OPEN)");
        end else begin
            $display("[Test] INFO: Initial CC1 debounced=0x%h (expected 0x4)", cc1_debounced);
        end
        
        // 模拟 CC 连接
        $display("[Test] Simulating CC connection...");
        cc1_state = 3'b001;  // 0.2V ~ 1V (Rp-3A 检测)
        
        // 等待消抖完成 (2ms + 余量)
        wait_cycles(110000);  // 2.2ms @ 50MHz
        $display("[Test] After CC1 change: input=0x%h, debounced=0x%h", cc1_state, cc1_debounced);
        
        if (cc1_debounced == 3'b001) begin
            $display("[Test] PASS: CC1 state synced correctly");
        end else begin
            $display("[Test] FAIL: CC1 state sync wrong (expected 0x1, got 0x%h)", cc1_debounced);
        end
    end
endtask

// 测试 VBUS 状态同步
task test_vbus_state_sync;
    begin
        $display("[Test] =========================================");
        $display("[Test] Test 4: VBUS State Sync");
        $display("[Test] =========================================");
        
        // 初始: VBUS 低
        vbus_state = 2'b00;  // < 0.8V
        
        wait_cycles(10);
        $display("[Test] Initial VBUS state: input=0x%h, debounced=0x%h", vbus_state, vbus_debounced);
        
        // VBUS 建立
        vbus_state = 2'b10;  // > 3V
        
        wait_cycles(10);
        $display("[Test] After VBUS change: input=0x%h, debounced=0x%h", vbus_state, vbus_debounced);
        
        if (vbus_debounced == 2'b10) begin
            $display("[Test] PASS: VBUS state synced correctly");
        end else begin
            $display("[Test] FAIL: VBUS state sync wrong (expected 0x2, got 0x%h)", vbus_debounced);
        end
    end
endtask

//============================================================================
// 主测试流程
//============================================================================
initial begin
    $display("==============================================");
    $display("PD PHY CC Module Testbench");
    $display("==============================================");
    
    // 初始化
    cc1_state = 3'b100;  // OPEN
    cc2_state = 3'b100;
    cc1_mode_cfg = 3'b000;
    cc2_mode_cfg = 3'b000;
    drp_mode = 0;
    vbus_state = 2'b00;
    
    // 等待复位完成
    wait_cycles(20);
    
    // 运行测试
    //test_cc_mode_output();
    test_drp_mode();
    //test_cc_state_sync();
    //test_vbus_state_sync();
    
    $display("==============================================");
    $display("PD PHY CC Test Completed");
    $display("==============================================");
    
    $finish;
end

//============================================================================
// 波形记录
//============================================================================
initial begin
    $dumpfile("pd_phy_tb.vcd");
    $dumpvars(0, pd_phy_tb);
end

endmodule

