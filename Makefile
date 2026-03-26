# ============================================================================
# USB PD BMC 收发器 - Makefile
# ============================================================================
# 用于编译、仿真和综合的构建脚本
# ============================================================================

# ----------------------------------------------------------------------------
# 工具配置
# ----------------------------------------------------------------------------
# Verilog 仿真器
IVERILOG = iverilog
VVP = vvp

# 波形查看器
GTKWAVE = gtkwave

# Vivado 配置
XILINX_PATH = /home/airliu/tools/xilinx/2025.2
VIVADO = $(XILINX_PATH)/Vivado/bin/vivado
VIVADO_MODE = -mode batch
VIVADO_OPTS = -nojournal -nolog

# 综合工具 (可选，需要安装 Yosys)
# YOSYS = yosys

# ----------------------------------------------------------------------------
# 项目配置
# ----------------------------------------------------------------------------
# 项目名称
PROJECT_NAME = dtest_phy

# FPGA 器件
FPGA_PART = xc7k325tffg676-2

# 顶层模块
TOP_MODULE = dtest_phy_top

# 约束文件
XDC_FILE = xc7k325t_constraints.xdc

# Vivado 输出目录
VIVADO_OUT_DIR = vivado_out
VIVADO_PROJECT = $(VIVADO_OUT_DIR)/$(PROJECT_NAME).xpr

# ----------------------------------------------------------------------------
# 文件配置
# ----------------------------------------------------------------------------
# PD PHY 模块目录
PD_PHY_DIR = pd_phy

# 顶层源文件
TOP_SRCS = dtest_phy.v spi_slave.v i2c_slave.v pd_phy_regs.v

# PD PHY 子模块 (纯 PD 功能)
PHY_SRCS = $(PD_PHY_DIR)/pd_bmc_tx.v \
           $(PD_PHY_DIR)/pd_bmc_rx.v \
           $(PD_PHY_DIR)/pd_bmc_transceiver.v \
           $(PD_PHY_DIR)/cc.v \
           $(PD_PHY_DIR)/pd_phy.v

# 所有源文件
SRCS = $(TOP_SRCS) $(PHY_SRCS)

# 测试平台
TB_PHY = $(PD_PHY_DIR)/pd_bmc_tb.v
TB_TOP = dtest_phy_tb.v
TB_PD_PHY = $(PD_PHY_DIR)/pd_phy_tb.v  # PD PHY 模块专用测试 (CC/VBUS)
# TB_AB = dtest_phy_ab_tb.v

# 默认测试平台 (顶层)
TB = $(TB_TOP)

# 所有 Verilog 文件
ALL_SRCS = $(SRCS) $(TB)
# ALL_SRCS_AB = $(SRCS) $(TB_AB)

# 输出文件名
SIM_OUTPUT = dtest_phy.sim
WAVEFORM = dtest_phy.vcd

# 综合输出 (可选)
SYNTH_OUTPUT = pd_bmc_synthesis.json
RTLIL_OUTPUT = pd_bmc.rtlil

# ----------------------------------------------------------------------------
# 编译参数
# ----------------------------------------------------------------------------
# Icarus Verilog 编译标志
# 使用-g2012 启用 SystemVerilog 特性（支持数组端口）
IVERILOG_FLAGS = -g2012 -Wall -DSIMULATION

# 定义参数 (可覆盖模块中的 parameter)
# 例如：-P CLK_FREQ_MHZ=100 将系统时钟设为 100MHz
DEFINES = 

# ----------------------------------------------------------------------------
# 目标
# ----------------------------------------------------------------------------

# 默认目标：编译并运行仿真
.PHONY: all
all: compile simulate

# PD PHY 模块测试
.PHONY: pd-phy-test
pd-phy-test: compile-pd-phy simulate-pd-phy

# A->B 通信测试
.PHONY: ab-test
ab-test: compile-ab simulate-ab

.PHONY: compile-pd-phy
compile-pd-phy:
	@echo "========================================"
	@echo "编译 PD PHY 模块测试..."
	@echo "========================================"
	$(IVERILOG) $(IVERILOG_FLAGS) $(DEFINES) -o pd_phy_tb.sim $(SRCS) $(TB_PD_PHY)
	@echo "编译完成！输出文件：pd_phy_tb.sim"

.PHONY: simulate-pd-phy
simulate-pd-phy:
	@echo "========================================"
	@echo "运行 PD PHY 模块测试..."
	@echo "========================================"
	$(VVP) pd_phy_tb.sim
	@echo "仿真完成！波形文件：pd_phy_tb.vcd"

.PHONY: compile-ab
compile-ab:
	@echo "========================================"
	@echo "编译 A->B 通信测试..."
	@echo "========================================"
	$(IVERILOG) $(IVERILOG_FLAGS) $(DEFINES) -o dtest_phy_ab.sim $(ALL_SRCS_AB)
	@echo "编译完成！输出文件：dtest_phy_ab.sim"

.PHONY: simulate-ab
simulate-ab:
	@echo "========================================"
	@echo "运行 A->B 通信测试..."
	@echo "========================================"
	$(VVP) dtest_phy_ab.sim
	@echo "仿真完成！波形文件：dtest_phy_ab_tb.vcd"

# 编译目标
.PHONY: compile build
compile: build
build: $(SIM_OUTPUT)

$(SIM_OUTPUT): $(ALL_SRCS)
	@echo "========================================"
	@echo "编译 USB PD BMC 收发器..."
	@echo "========================================"
	$(IVERILOG) $(IVERILOG_FLAGS) $(DEFINES) -o $(SIM_OUTPUT) $(ALL_SRCS)
	@echo "编译完成！输出文件：$(SIM_OUTPUT)"

# 运行仿真
.PHONY: simulate run sim
simulate: run
run: $(SIM_OUTPUT)
	@echo "========================================"
	@echo "运行仿真..."
	@echo "========================================"
	$(VVP) $(SIM_OUTPUT)
	@echo "仿真完成！波形文件：$(WAVEFORM)"

# 仅编译，不运行
.PHONY: build-only
build-only: $(SIM_OUTPUT)

# 重新编译
.PHONY: rebuild
rebuild: clean all

# 打开波形查看器
.PHONY: wave waveform gtkwave
wave: waveform
waveform: $(WAVEFORM)
	@echo "打开 GTKWave 波形查看器..."
	$(GTKWAVE) $(WAVEFORM)

# 清理生成的文件
.PHONY: clean
clean:
	@echo "清理生成的文件..."
	rm -f $(SIM_OUTPUT) $(WAVEFORM) $(WAVEFORM).bak
	rm -f dtest_phy_ab.sim dtest_phy_ab_tb.vcd
	rm -f pd_phy_tb.sim pd_phy_tb.vcd
	rm -f $(SYNTH_OUTPUT) $(RTLIL_OUTPUT)
	@echo "清理完成"

# 深度清理（包括备份文件）
.PHONY: distclean
distclean: clean
	rm -f *.vcd.* *.log
	rm -rf $(VIVADO_OUT_DIR)

# ----------------------------------------------------------------------------
# Vivado 目标
# ----------------------------------------------------------------------------

# 创建 Vivado 项目
.PHONY: vivado-project
vivado-project:
	@echo "========================================"
	@echo "创建 Vivado 项目..."
	@echo "========================================"
	@mkdir -p $(VIVADO_OUT_DIR)
	$(VIVADO) $(VIVADO_MODE) $(VIVADO_OPTS) -source tcl/create_project.tcl

# Vivado 综合
.PHONY: vivado-synth
vivado-synth:
	@echo "========================================"
	@echo "Vivado 综合..."
	@echo "========================================"
	@mkdir -p $(VIVADO_OUT_DIR)
	$(VIVADO) $(VIVADO_MODE) $(VIVADO_OPTS) -source tcl/synth.tcl

# Vivado 实现 (布局布线)
.PHONY: vivado-impl
vivado-impl:
	@echo "========================================"
	@echo "Vivado 实现..."
	@echo "========================================"
	$(VIVADO) $(VIVADO_MODE) $(VIVADO_OPTS) -source tcl/impl.tcl

# 生成 Bitstream
.PHONY: vivado-bit bitstream
vivado-bit: bitstream
bitstream:
	@echo "========================================"
	@echo "生成 Bitstream..."
	@echo "========================================"
	$(VIVADO) $(VIVADO_MODE) $(VIVADO_OPTS) -source tcl/bitstream.tcl

# Vivado 完整流程 (综合 + 实现 + Bitstream)
.PHONY: vivado-all vivado
vivado-all: vivado
vivado:
	@echo "========================================"
	@echo "Vivado 完整编译流程..."
	@echo "========================================"
	@mkdir -p $(VIVADO_OUT_DIR)
	$(VIVADO) $(VIVADO_MODE) $(VIVADO_OPTS) -source tcl/build_all.tcl

# 打开 Vivado GUI
.PHONY: vivado-gui
vivado-gui:
	@echo "打开 Vivado GUI..."
	$(VIVADO) -mode gui $(VIVADO_PROJECT) &

# 清理 Vivado 输出
.PHONY: vivado-clean
vivado-clean:
	@echo "清理 Vivado 输出..."
	rm -rf $(VIVADO_OUT_DIR)
	rm -f *.jou *.log
	rm -rf .Xil

# ----------------------------------------------------------------------------
# 综合目标 (需要安装 Yosys)
# ----------------------------------------------------------------------------

# RTL 综合
.PHONY: synth synthesize
synth: synthesize
synthesize: $(SYNTH_OUTPUT)

$(SYNTH_OUTPUT): $(SRCS)
	@echo "========================================"
	@echo "RTL 综合 (使用 Yosys)..."
	@echo "========================================"
	$(YOSYS) -p "read_verilog $(SRCS); write_json $(SYNTH_OUTPUT)"
	@echo "综合完成！输出文件：$(SYNTH_OUTPUT)"

# 生成 RTLIL 中间格式
.PHONY: rtlil
rtlil: $(RTLIL_OUTPUT)

$(RTLIL_OUTPUT): $(SRCS)
	@echo "生成 RTLIL 格式..."
	$(YOSYS) -p "read_verilog $(SRCS); write_rtlil $(RTLIL_OUTPUT)"

# ----------------------------------------------------------------------------
# 语法检查
# ----------------------------------------------------------------------------
.PHONY: lint check syntax
lint: check
check: syntax
syntax:
	@echo "========================================"
	@echo "语法检查..."
	@echo "========================================"
	$(IVERILOG) -t null $(IVERILOG_FLAGS) $(ALL_SRCS)
	@echo "语法检查通过！"

# ----------------------------------------------------------------------------
# 帮助信息
# ----------------------------------------------------------------------------
.PHONY: help
help:
	@echo "========================================"
	@echo "USB PD BMC 收发器 - Makefile 帮助"
	@echo "========================================"
	@echo ""
	@echo "仿真目标:"
	@echo "  all         - 编译并运行仿真 (默认)"
	@echo "  pd-phy-test - 运行 PD PHY CC/VBUS 模块测试"
	@echo "  ab-test     - 运行 A->B 通信测试"
	@echo "  compile     - 仅编译，不运行仿真"
	@echo "  simulate    - 运行仿真（需要先编译）"
	@echo "  rebuild     - 重新编译"
	@echo "  wave        - 打开 GTKWave 查看波形"
	@echo "  lint        - 语法检查"
	@echo ""
	@echo "Vivado 目标:"
	@echo "  vivado        - 完整编译流程 (综合+实现+Bitstream)"
	@echo "  vivado-synth  - 仅综合"
	@echo "  vivado-impl   - 仅实现 (布局布线)"
	@echo "  bitstream     - 生成 Bitstream"
	@echo "  vivado-gui    - 打开 Vivado GUI"
	@echo "  vivado-clean  - 清理 Vivado 输出"
	@echo ""
	@echo "清理目标:"
	@echo "  clean       - 清理仿真文件"
	@echo "  distclean   - 深度清理 (包括 Vivado 输出)"
	@echo ""
	@echo "变量:"
	@echo "  FPGA_PART   = $(FPGA_PART)"
	@echo "  TOP_MODULE  = $(TOP_MODULE)"
	@echo "  XDC_FILE    = $(XDC_FILE)"
	@echo ""
	@echo "示例用法:"
	@echo "  make                    # 编译并运行仿真"
	@echo "  make vivado             # Vivado 完整编译"
	@echo "  make vivado-synth       # 仅运行综合"
	@echo "  make wave               # 打开波形查看器"
	@echo ""

# ----------------------------------------------------------------------------
# 依赖关系
# ----------------------------------------------------------------------------
# 确保在更改源文件时重新编译
$(SIM_OUTPUT): $(SRCS) $(TB)