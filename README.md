# USB PD/UFCS 快充协议 PHY 模块

## 项目概述

本项目实现了 USB Power Delivery (PD) 和 UFCS（Universal Fast Charging Specification）快充协议的物理层（PHY）模块。这些模块使用 Verilog HDL 编写，可用于 FPGA 或 ASIC 设计中的快充协议控制器。

## 项目结构

```
dtest_phy/
├── README.md              # 项目说明文档
├── Makefile               # 编译和仿真脚本
└── pd_phy/                # PD PHY 模块目录
    ├── pd_bmc_tx.v        # PD BMC 发送模块
    ├── pd_bmc_rx.v        # PD BMC 接收模块
    ├── pd_bmc_transceiver.v # PD BMC 收发器顶层模块
    └── pd_bmc_tb.v        # PD BMC 测试平台
```

## 模块说明

### 1. PD BMC 发送模块 (pd_bmc_tx.v)

实现 USB PD 协议的 BMC（Bi-Phase Mark Code）编码发送功能。

**主要特性：**
- 支持完整 PD 包格式：64bit 前导码 → SOP → 2 字节 Header → 数据载荷 → 32bit CRC → EOP
- 采用 4B5B 编码
- 支持 LSB first（低位先行）发送顺序
- 可配置数据长度（最多 30 字节）
- 集成 CRC32 计算（多项式：0x04C11DB7）

**接口信号：**
| 信号名 | 方向 | 位宽 | 描述 |
|--------|------|------|------|
| clk | input | 1 | 系统时钟（推荐 50MHz） |
| rst_n | input | 1 | 异步复位，低有效 |
| tx_header_i | input | 16 | 2 字节 Message Header |
| tx_data_i | input | [7:0][0:29] | 最多 30 字节数据载荷 |
| tx_data_len_i | input | 5 | 实际发送的数据字节数 |
| tx_start_i | input | 1 | 包发送启动信号 |
| tx_done_o | output | 1 | 包发送完成标志 |
| tx_busy_o | output | 1 | 发送忙标志 |
| bmc_tx_pad | output | 1 | BMC 编码输出 |
| tx_active_o | output | 1 | 发送进行中标志 |

**参数配置：**
```verilog
parameter BIT_PERIOD_CNT = 166;    // 50MHz 对应 300kbps 位周期
parameter HALF_BIT_CNT = 83;       // 半位周期计数值
parameter PREAMBLE_LEN = 64;       // 前导码长度
parameter MAX_DATA_LEN = 30;       // 最大数据字节数
```

### 2. PD BMC 接收模块 (pd_bmc_rx.v)

实现 USB PD 协议的 BMC 解码接收功能。

**主要特性：**
- 自动检测并跳过 64bit 前导码
- BMC 解码
- 支持 LSB first 接收

**接口信号：**
| 信号名 | 方向 | 位宽 | 描述 |
|--------|------|------|------|
| clk | input | 1 | 系统时钟 |
| rst_n | input | 1 | 异步复位，低有效 |
| rx_data_o | output | 8 | 接收数据 |
| rx_valid_o | output | 1 | 接收数据有效 |
| rx_error_o | output | 1 | 接收错误标志 |
| bmc_rx_pad | input | 1 | BMC 编码输入 |
| rx_en_i | input | 1 | 接收使能 |

### 3. PD BMC 收发器顶层模块 (pd_bmc_transceiver.v)

集成发送和接收模块的顶层封装。

### 4. 测试平台 (pd_bmc_tb.v)

提供完整的仿真测试环境。

## 使用方法

### 编译和仿真

```bash
# 编译并运行仿真
make

# 仅编译
make compile

# 仅运行仿真
make simulate

# 打开 GTKWave 查看波形
make wave

# 清理生成的文件
make clean
```

### 在项目中实例化

```verilog
// 实例化 PD BMC 发送模块
pd_bmc_tx u_pd_tx (
    .clk           (sys_clk),
    .rst_n         (sys_rst_n),
    .tx_header_i   (header),
    .tx_data_i     (data),
    .tx_data_len_i (data_len),
    .tx_start_i    (start),
    .tx_done_o     (done),
    .tx_busy_o     (busy),
    .bmc_tx_pad    (bmc_out),
    .tx_active_o   (active)
);
```

## 技术规格

### USB PD 物理层参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 通信速率 | 300 kbps ±10% | BMC 编码后速率为 600kbps |
| 位周期 | 3.333 μs | 每个比特的时间 |
| 前导码 | 64 bits | 01 交替模式 |
| SOP 有序集 | Sync-1, Sync-1, Sync-1, Sync-2 | 帧起始标志 |
| EOP 有序集 | K-EOP | 帧结束标志 |
| CRC 多项式 | 0x04C11DB7 | CRC32，初始值 0xFFFFFFFF |

### BMC 编码规则

BMC（Bi-Phase Mark Code）编码是一种自同步编码方式：
- **逻辑 0**：仅在比特周期中间有 1 次跳变
- **逻辑 1**：在比特周期开始和中间各有 1 次跳变（共 2 次）

### 4B5B 编码表

| 4-bit 数据 | 5-bit 符号 |
|-----------|-----------|
| 0000 | 11110 |
| 0001 | 01001 |
| 0010 | 10100 |
| ... | ... |
| 1111 | 11101 |

## 未来扩展

### UFCS 支持
- UFCS 协议特定的编码方式
- UFCS 握手序列
- 电压/电流协商逻辑

### 其他功能
- 完整的 PD 协议栈（Policy 层）
- 多端口支持
- PPS（Programmable Power Supply）支持

## 注意事项

1. **时钟频率**：推荐使用 50MHz 系统时钟，可通过修改 `BIT_PERIOD_CNT` 参数适配其他频率
2. **复位时序**：确保复位信号至少保持 10 个时钟周期的低电平
3. **数据对齐**：发送数据时确保 `tx_data_i` 数组正确填充
4. **CRC 计算**：CRC 字段本身不参与 CRC 计算

## 参考文档

- USB Power Delivery Specification v3.0
- UFCS Technical Specification
- IEEE 802.3 (PoE) BMC Encoding Standard

## 许可证

本项目代码采用 MIT 许可证。