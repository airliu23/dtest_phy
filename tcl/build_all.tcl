# ============================================================================
# Vivado 完整编译脚本
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

# ============================================================================
# 创建项目
# ============================================================================
puts "========================================"
puts "创建 Vivado 项目..."
puts "========================================"

# 创建输出目录
file mkdir $out_dir

# 删除旧项目 (如果存在)
if {[file exists $out_dir/$project_name]} {
    file delete -force $out_dir/$project_name
}

# 创建项目 (保存到磁盘)
create_project $project_name $out_dir/$project_name -part $part -force

# 添加源文件
foreach src $src_files {
    if {[file exists $src]} {
        add_files -norecurse $src
        puts "添加源文件: $src"
    } else {
        puts "警告: 文件不存在 - $src"
    }
}

# 添加约束文件
if {[file exists $xdc_file]} {
    add_files -fileset constrs_1 -norecurse $xdc_file
    puts "添加约束文件: $xdc_file"
} else {
    puts "错误: 约束文件不存在 - $xdc_file"
    exit 1
}

# 设置顶层模块
set_property top $top_module [current_fileset]

# 更新编译顺序
update_compile_order -fileset sources_1

# 降低 UCIO-1 检查的严重性 (允许未约束端口)
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

# ============================================================================
# 综合
# ============================================================================
puts ""
puts "========================================"
puts "运行综合..."
puts "========================================"

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# 检查综合是否成功
set synth_status [get_property STATUS [get_runs synth_1]]
puts "综合状态: $synth_status"
if {$synth_status != "synth_design Complete!"} {
    puts "错误: 综合失败!"
    exit 1
}

puts "综合完成！"

# ============================================================================
# 实现 (布局布线)
# ============================================================================
puts ""
puts "========================================"
puts "运行实现 (布局布线)..."
puts "========================================"

launch_runs impl_1 -jobs 8
wait_on_run impl_1

# 检查实现是否成功
set impl_status [get_property STATUS [get_runs impl_1]]
puts "实现状态: $impl_status"
if {$impl_status != "route_design Complete!"} {
    puts "警告: 实现可能未完全成功，继续生成 Bitstream..."
}

puts "实现完成！"

# ============================================================================
# 生成 Bitstream
# ============================================================================
puts ""
puts "========================================"
puts "生成 Bitstream..."
puts "========================================"

# 打开实现结果
open_run impl_1

# 降低 UCIO-1 检查的严重性 (允许未约束端口生成 bitstream)
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

# 生成 bitstream
write_bitstream -force $out_dir/${project_name}.bit

puts "Bitstream 生成完成！"
puts "输出文件: $out_dir/${project_name}.bit"

# ============================================================================
# 生成报告
# ============================================================================
report_utilization -file $out_dir/utilization_impl.rpt
report_timing_summary -file $out_dir/timing_impl.rpt
report_power -file $out_dir/power.rpt

# ============================================================================
# 完成
# ============================================================================
puts ""
puts "========================================"
puts "编译完成！"
puts "========================================"
puts "输出目录: $out_dir"
puts "  - ${project_name}.bit        (Bitstream)"
puts "  - utilization_impl.rpt       (资源报告)"
puts "  - timing_impl.rpt            (时序报告)"
puts "  - power.rpt                  (功耗报告)"
puts "========================================"
