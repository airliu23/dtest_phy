# ============================================================================
# Vivado 综合脚本
# ============================================================================

# 项目配置
set project_name "dtest_phy"
set part "xc7k325tffg900-2"
set top_module "dtest_phy_top"
set out_dir "vivado_out"

# 源文件列表
set src_files [list \
    "dtest_phy_top.v" \
    "dtest_phy.v" \
    "spi_slave.v" \
    "i2c_slave.v" \
    "pd_phy_regs.v" \
    "pd_phy/pd_bmc_tx.v" \
    "pd_phy/pd_bmc_rx.v" \
    "pd_phy/pd_bmc_transceiver.v" \
    "pd_phy/cc.v" \
    "pd_phy/pd_phy.v" \
]

# 约束文件
set xdc_file "xc7k325t_constraints.xdc"

# 创建输出目录
file mkdir $out_dir

# 创建内存项目
create_project -in_memory -part $part

# 添加源文件
foreach src $src_files {
    if {[file exists $src]} {
        add_files $src
    }
}

# 添加约束文件
if {[file exists $xdc_file]} {
    add_files -fileset constrs_1 $xdc_file
}

# 设置顶层模块
set_property top $top_module [current_fileset]

# 运行综合
puts "========================================"
puts "运行综合..."
puts "========================================"

synth_design -top $top_module -part $part

# 保存检查点
write_checkpoint -force $out_dir/${project_name}_synth.dcp

# 生成报告
report_utilization -file $out_dir/utilization_synth.rpt
report_timing_summary -file $out_dir/timing_synth.rpt

puts "========================================"
puts "综合完成！"
puts "========================================"
puts "检查点: $out_dir/${project_name}_synth.dcp"
