# USB PD PHY 模块

## 项目概述

本项目实现了 USB Power Delivery (PD) 协议的物理层（PHY）模块，包括 BMC 编解码、自动 GoodCRC 回复、CC 检测等功能。使用 Verilog HDL 编写，目标平台为 Xilinx XC7K325T FPGA。

## 项目结构

```
dtest_phy/
├── README.md                    # 项目说明文档
├── Makefile                     # 编译和仿真脚本
├── dtest_phy.v                  # 顶层模块
├── dtest_phy_tb.v               # 测试平台（I2C 双模块通信测试）
├── pd_phy_regs.v                # 寄存器接口模块
├── spi_slave.v                  # SPI 从机接口
├── i2c_slave.v                  # I2C 从机接口
├── xc7k325t_constraints.xdc     # FPGA 约束文件
└── pd_phy/                      # PD PHY 核心模块目录
    ├── pd_phy.v                 # PD PHY 顶层封装
    ├── pd_bmc_tx.v              # BMC 发送模块
    ├── pd_bmc_rx.v              # BMC 接收模块
    ├── pd_bmc_transceiver.v     # BMC 收发器（含自动 GoodCRC）
    ├── cc.v                     # CC 检测模块
    └── pd_bmc_tb.v              # BMC 单元测试
```

## 核心功能

### 1. BMC 发送模块 (pd_bmc_tx.v)

**主要特性：**
- 完整 PD 包格式：64bit 前导码 → SOP → Header → Data → CRC32 → EOP
- 4B5B 编码，LSB first 发送
- EOP 后处理：根据 bmc_state 决定是否需要额外翻转
- 2ms 总线空闲超时保护
- 可配置参数：位周期、前导码长度、最大数据长度

**EOP 后处理时序：**
- 如果 EOP 最后 bmc_state=0，进入 ST_EOP_EDGE 翻转到 1
- 然后进入 ST_LOW 拉低保持一段时间
- 最后进入 ST_DONE 完成发送

### 2. BMC 接收模块 (pd_bmc_rx.v)

**主要特性：**
- 边沿检测解码，自动同步前导码
- 4B5B 解码，CRC32 校验
- EOP 检测后等待 50us 总线空闲确认消息结束
- 2ms 总线空闲超时保护
- 实时调试接口：输出解析的字节数据

**rx_done_o 触发条件：**
- 检测到下降沿（新消息开始）
- 50us 总线空闲完成

### 3. BMC 收发器 (pd_bmc_transceiver.v)

**主要特性：**
- 自动 GoodCRC 回复（45μs 延时）
- TX/RX 半双工互斥控制
- 50ms ACK 超时检测
- 可配置 GoodCRC Header（PowerRole/DataRole）

**状态机：**
- TX 状态机：IDLE → SENDING → WAIT_ACK → DONE/FAIL
- RX 状态机：IDLE → RECEIVING → DELAY_ACK → SEND_ACK → WAIT_ACK_DONE → DONE

### 4. CC 检测模块 (cc.v)

**主要特性：**
- DRP（Dual Role Port）模式支持
- 可配置终端模式：Rp (0.5A/1.5A/3A)、Rd、Ra、Open
- 200ms 消抖（可配置）
- 稳定状态检测，2ms 保持时间
- CC 状态变化中断

## 寄存器映射

| 地址 | 名称 | R/W | 描述 |
|------|------|-----|------|
| 0x00-0x05 | ID | R | 只读 ID/版本信息 (1C 31 50 21 50 00) |
| 0x10 | IRQ_FLAG | R/W | 中断标志(写1清) [0]:CC_CHG [2]:RX_SUCCESS [4]:TX_FAIL [6]:TX_SUCCESS |
| 0x12 | IRQ_EN | R/W | 中断使能 [0]:CC_CHG [2]:RX_SUCCESS [4]:TX_FAIL [6]:TX_SUCCESS |
| 0x1A | CC_CTRL | R/W | CC控制 [1:0]:CC1 [3:2]:CC2 [5:4]:RP_LVL [6]:DRP |
| 0x1C | CC_STATUS | R | CC状态 [2:0]:CC1_STATE [5:3]:CC2_STATE |
| 0x30 | RX_LEN | R | 接收数据长度 |
| 0x31 | RX_HDR_L | R | RX Header 低字节 |
| 0x32 | RX_HDR_H | R | RX Header 高字节 |
| 0x33-0x4C | RX_DATA | R | 接收数据 (26字节) |
| 0x50 | CTRL | R/W | 控制寄存器 [0]:TX_START [1]:RX_EN |
| 0x51 | TX_LEN | R/W | TX 数据长度 |
| 0x52 | TX_HDR_L | R/W | TX Header 低字节 |
| 0x53 | TX_HDR_H | R/W | TX Header 高字节 |
| 0x54-0x64 | TX_DATA | R/W | 发送数据 (17字节) |

## 时序参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 系统时钟 | 50MHz | 基准时钟 |
| BMC 速率 | 300kbps | 位周期 3.33μs |
| 前导码 | 64bits | 01 交替模式 |
| GoodCRC 延时 | 45μs | rx_done 后到发送 GoodCRC |
| EOP 后等待 | 50μs | 总线空闲确认 |
| 总线超时 | 2ms | 任意状态超时回 IDLE |
| ACK 超时 | 50ms | 等待 GoodCRC 超时 |
| CC 消抖 | 200ms | 稳定状态检测 |

## 使用方法

### 编译和仿真

```bash
# 编译并运行仿真
make

# 仅编译
make compile

# 运行仿真
make run

# 查看波形
make wave

# 清理
make clean
```

### 测试验证

测试平台 `dtest_phy_tb.v` 实现双模块 A→B 通信测试：
1. 模块 A 发送消息（Accept 类型）
2. 模块 B 自动回复 GoodCRC
3. 验证 TX_SUCCESS 和 RX_SUCCESS 状态
4. 重复 3 次发送测试

## 注意事项

1. **TX/RX 互斥**：tx_start_i 触发时自动禁用 RX
2. **GoodCRC 时序**：rx_done 触发后 45μs 开始发送
3. **超时保护**：2ms 总线空闲自动回复位
4. **CC 模式**：复位后默认 DRP 模式

## 参考文档

- USB Power Delivery Specification v3.0
- USB Type-C Cable and Connector Specification

## 许可证

MIT License
