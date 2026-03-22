//###########################################################################
// AC7K325B 最终完美版约束
//###########################################################################

// 1. 200MHz 差分时钟 (P=AE10, N=AF10)
set_property PACKAGE_PIN AE10 [get_ports sys_clk_p]
set_property PACKAGE_PIN AF10 [get_ports sys_clk_n]
set_property IOSTANDARD LVDS [get_ports sys_clk_p]
set_property IOSTANDARD LVDS [get_ports sys_clk_n]
create_clock -name sys_clk_200m -period 5.000 [get_ports sys_clk_p]

// 2. 复位按键 (Y23)
set_property PACKAGE_PIN Y23 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP TRUE [get_ports rst_n]

// 3. 底板LED
set_property PACKAGE_PIN L17 [get_ports led_tx]
set_property IOSTANDARD LVCMOS33 [get_ports led_tx]
set_property DRIVE_STRENGTH 8 [get_ports led_tx]

set_property PACKAGE_PIN H19 [get_ports led_rx_ok]
set_property IOSTANDARD LVCMOS33 [get_ports led_rx_ok]
set_property DRIVE_STRENGTH 8 [get_ports led_rx_ok]

set_property PACKAGE_PIN K19 [get_ports led_rx_err]
set_property IOSTANDARD LVCMOS33 [get_ports led_rx_err]
set_property DRIVE_STRENGTH 8 [get_ports led_rx_err]

set_property PACKAGE_PIN K20 [get_ports led_heartbeat]
set_property IOSTANDARD LVCMOS33 [get_ports led_heartbeat]
set_property DRIVE_STRENGTH 8 [get_ports led_heartbeat]

// 4. BMC 总线 (TX=E28)
set_property PACKAGE_PIN E28 [get_ports bmc_tx_pad]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_tx_pad]
set_property DRIVE_STRENGTH 12 [get_ports bmc_tx_pad]

set_property PACKAGE_PIN AA21 [get_ports bmc_rx_pad]
set_property IOSTANDARD LVCMOS33 [get_ports bmc_rx_pad]
set_property PULLUP TRUE [get_ports bmc_rx_pad]

// 5. 配置
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSED_PIN_TERMINATION PULLUP [current_design]