##############################################################################
## dtest_phy - XC7K325T 约束文件
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
set_property PACKAGE_PIN k14 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP true [get_ports rst_n]

##############################################################################
## 3. SPI 从机接口
##############################################################################
set_property PACKAGE_PIN H29 [get_ports spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports spi_sclk]

set_property PACKAGE_PIN J29 [get_ports spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports spi_cs_n]
set_property PULLUP true [get_ports spi_cs_n]

set_property PACKAGE_PIN J28 [get_ports spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports spi_mosi]

set_property PACKAGE_PIN J27 [get_ports spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports spi_miso]
set_property DRIVE 8 [get_ports spi_miso]
set_property SLEW FAST [get_ports spi_miso]

##############################################################################
## 4. 中断输出 (active low)
##############################################################################
set_property PACKAGE_PIN F30 [get_ports irq_n]
set_property IOSTANDARD LVCMOS33 [get_ports irq_n]
set_property DRIVE 8 [get_ports irq_n]

##############################################################################
## 5. LED 指示灯 (active low)
##############################################################################
set_property PACKAGE_PIN L17 [get_ports led_tx]
set_property IOSTANDARD LVCMOS33 [get_ports led_tx]
set_property DRIVE 8 [get_ports led_tx]

set_property PACKAGE_PIN H19 [get_ports led_rx_ok]
set_property IOSTANDARD LVCMOS33 [get_ports led_rx_ok]
set_property DRIVE 8 [get_ports led_rx_ok]

set_property PACKAGE_PIN K19 [get_ports led_rx_err]
set_property IOSTANDARD LVCMOS33 [get_ports led_rx_err]
set_property DRIVE 8 [get_ports led_rx_err]

set_property PACKAGE_PIN K20 [get_ports led_heartbeat]
set_property IOSTANDARD LVCMOS33 [get_ports led_heartbeat]
set_property DRIVE 8 [get_ports led_heartbeat]

##############################################################################
## 6. BMC 物理接口
##############################################################################
set_property PACKAGE_PIN E28 [get_ports bmc_tx_pad]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_tx_pad]
set_property DRIVE 12 [get_ports bmc_tx_pad]
set_property SLEW FAST [get_ports bmc_tx_pad]

set_property PACKAGE_PIN D28 [get_ports bmc_rx_pad]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_rx_pad]
set_property PULLUP true [get_ports bmc_rx_pad]

##############################################################################
## 7. 时序约束
##############################################################################
# 生成时钟约束 (50MHz 内部时钟)
create_generated_clock -name clk_50m -source [get_ports sys_clk_p] -divide_by 4 [get_pins clk_50m_reg/Q]

# SPI 时钟约束 (假设最高 10MHz)
create_clock -name spi_clk -period 100.000 [get_ports spi_sclk]
set_clock_groups -asynchronous -group [get_clocks spi_clk] -group [get_clocks clk_50m]

# 输入延迟
set_input_delay -clock spi_clk -max 10.0 [get_ports spi_mosi]
set_input_delay -clock spi_clk -min 0.0 [get_ports spi_mosi]
set_input_delay -clock spi_clk -max 10.0 [get_ports spi_cs_n]
set_input_delay -clock spi_clk -min 0.0 [get_ports spi_cs_n]

# 输出延迟
set_output_delay -clock spi_clk -max 10.0 [get_ports spi_miso]
set_output_delay -clock spi_clk -min 0.0 [get_ports spi_miso]

##############################################################################
## 8. 配置选项
##############################################################################
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSED_PIN_TERMINATION PULLUP [current_design]
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
