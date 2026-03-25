##############################################################################
## dtest_phy_top - XC7K325T 约束文件
## 适配 AC7K325B 开发板
##############################################################################

##############################################################################
## 1. 系统时钟 (差分 200MHz)
##############################################################################
set_property PACKAGE_PIN AE10 [get_ports sys_clk_p]
set_property PACKAGE_PIN AF10 [get_ports sys_clk_n]
set_property IOSTANDARD LVDS [get_ports sys_clk_p]
set_property IOSTANDARD LVDS [get_ports sys_clk_n]
create_clock -name sys_clk_200m -period 5.000 [get_ports sys_clk_p]

##############################################################################
## 2. 复位按键 (active low)
##############################################################################
set_property PACKAGE_PIN K14 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP true [get_ports rst_n]

##############################################################################
## 3. I2C 从机接口 (地址 0x28)
##############################################################################
set_property PACKAGE_PIN H29 [get_ports i2c_scl]
set_property IOSTANDARD LVCMOS33 [get_ports i2c_scl]
set_property PULLUP true [get_ports i2c_scl]

set_property PACKAGE_PIN J29 [get_ports i2c_sda]
set_property IOSTANDARD LVCMOS33 [get_ports i2c_sda]
set_property PULLUP true [get_ports i2c_sda]
set_property DRIVE 8 [get_ports i2c_sda]

##############################################################################
## 4. 中断输出 (active low)
##############################################################################
set_property PACKAGE_PIN AE28 [get_ports irq_n]
set_property IOSTANDARD LVCMOS33 [get_ports irq_n]
set_property DRIVE 8 [get_ports irq_n]

##############################################################################
## 5. LED 指示灯 (active low)
##############################################################################
set_property PACKAGE_PIN K20 [get_ports led_heartbeat]
set_property IOSTANDARD LVCMOS33 [get_ports led_heartbeat]
set_property DRIVE 8 [get_ports led_heartbeat]

##############################################################################
## 6. BMC 物理接口 (分离输入输出)
##############################################################################
# BMC 接收输入
set_property PACKAGE_PIN AF27 [get_ports bmc_rx]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_rx]
set_property PULLUP true [get_ports bmc_rx]

# BMC 发送输出
set_property PACKAGE_PIN AD29 [get_ports bmc_tx]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_tx]
set_property DRIVE 12 [get_ports bmc_tx]
set_property SLEW FAST [get_ports bmc_tx]

# BMC 发送使能
set_property PACKAGE_PIN AF26 [get_ports bmc_tx_en]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_tx_en]
set_property DRIVE 8 [get_ports bmc_tx_en]

##############################################################################
## 7. CC 比较器接口 (分时复用)
##############################################################################
# 电压档位选择 [2:0]
set_property PACKAGE_PIN K21 [get_ports {en_cc_ref[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {en_cc_ref[0]}]
set_property DRIVE 8 [get_ports {en_cc_ref[0]}]
set_property PACKAGE_PIN AE30 [get_ports {en_cc_ref[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {en_cc_ref[1]}]
set_property DRIVE 8 [get_ports {en_cc_ref[1]}]
set_property PACKAGE_PIN AB30 [get_ports {en_cc_ref[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {en_cc_ref[2]}]
set_property DRIVE 8 [get_ports {en_cc_ref[2]}]

# CC1 比较结果输入
set_property PACKAGE_PIN L21 [get_ports cc1_comp]
set_property IOSTANDARD LVCMOS33 [get_ports cc1_comp]

# CC2 比较结果输入
set_property PACKAGE_PIN M24 [get_ports cc2_comp]
set_property IOSTANDARD LVCMOS33 [get_ports cc2_comp]

# CC1 比较器使能
set_property PACKAGE_PIN L20 [get_ports cc1_comp_en]
set_property IOSTANDARD LVCMOS33 [get_ports cc1_comp_en]
set_property DRIVE 8 [get_ports cc1_comp_en]

# CC2 比较器使能
set_property PACKAGE_PIN AE29 [get_ports cc2_comp_en]
set_property IOSTANDARD LVCMOS33 [get_ports cc2_comp_en]
set_property DRIVE 8 [get_ports cc2_comp_en]

##############################################################################
## 8. CC 模式输出 (到外部模拟开关/电阻网络)
##############################################################################
# CC1 Rd 使能
set_property PACKAGE_PIN AF30 [get_ports cc1_rd_en]
set_property IOSTANDARD LVCMOS33 [get_ports cc1_rd_en]
set_property DRIVE 8 [get_ports cc1_rd_en]

# CC2 Rd 使能
set_property PACKAGE_PIN AB29 [get_ports cc2_rd_en]
set_property IOSTANDARD LVCMOS33 [get_ports cc2_rd_en]
set_property DRIVE 8 [get_ports cc2_rd_en]

# CC1 Rp 使能 [1:0]
set_property PACKAGE_PIN M20 [get_ports {cc1_rp_en[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {cc1_rp_en[0]}]
set_property DRIVE 8 [get_ports {cc1_rp_en[0]}]
set_property PACKAGE_PIN L27 [get_ports {cc1_rp_en[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {cc1_rp_en[1]}]
set_property DRIVE 8 [get_ports {cc1_rp_en[1]}]

# CC2 Rp 使能 [1:0]
set_property PACKAGE_PIN M27 [get_ports {cc2_rp_en[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {cc2_rp_en[0]}]
set_property DRIVE 8 [get_ports {cc2_rp_en[0]}]
set_property PACKAGE_PIN L26 [get_ports {cc2_rp_en[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {cc2_rp_en[1]}]
set_property DRIVE 8 [get_ports {cc2_rp_en[1]}]

# 插头方向输出 (互补信号对)
set_property PACKAGE_PIN AA25 [get_ports plug_orient_p]
set_property IOSTANDARD LVCMOS33 [get_ports plug_orient_p]
set_property DRIVE 8 [get_ports plug_orient_p]

set_property PACKAGE_PIN AB25 [get_ports plug_orient_n]
set_property IOSTANDARD LVCMOS33 [get_ports plug_orient_n]
set_property DRIVE 8 [get_ports plug_orient_n]

# 调试翻转信号 (RX 每收到有效数据位翻转一次)
set_property PACKAGE_PIN J28 [get_ports dbg_toggle]
set_property IOSTANDARD LVCMOS33 [get_ports dbg_toggle]
set_property DRIVE 8 [get_ports dbg_toggle]

##############################################################################
## 9. 时序约束
##############################################################################
# 生成时钟约束 (50MHz 内部时钟)
create_generated_clock -name clk_50m -source [get_ports sys_clk_p] -divide_by 4 [get_pins u_dtest_phy/clk_50m_reg/Q]

# 异步信号
set_false_path -from [get_ports rst_n]
set_false_path -from [get_ports bmc_rx]
set_false_path -from [get_ports cc1_comp]
set_false_path -from [get_ports cc2_comp]
set_false_path -from [get_ports i2c_scl]
set_false_path -from [get_ports i2c_sda]

##############################################################################
## 10. 配置选项
##############################################################################
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSED_PIN_TERMINATION PULLUP [current_design]
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
