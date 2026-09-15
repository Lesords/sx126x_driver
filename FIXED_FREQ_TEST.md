# SX1262 定频测试工具（sx126x_demo）测试文档

> 适用对象：BeagleBadge AM62L + SX1262（Wio-SX1262 高频模组）
> 用途：定频收发 / 认证测试（FCC US915、CE EU868）/ PER 对测
> 对应代码：`demo/main.c`（commit `2dece06` 及之后）

---

## 1. 工具概述

三种工作模式，全部射频参数命令行可覆盖，默认锁定 **FCC DTS** 参数组：

| 模式 | 说明 |
|---|---|
| `tx` | 定频发射 LoRa 调制波（间隔发包 / 背靠背连发 / 无限前导码） |
| `rx` | 定频连续接收，输出 RSSI/SNR 与收包统计 |
| `cw` | 单载波发射（认证测传导功率/频率稳定度） |

**默认参数组（FCC DTS）**：

| 参数 | 默认值 |
|---|---|
| 频率 | 903000000 Hz |
| 功率 | 22 dBm |
| SF / BW / CR | SF7 / 500 kHz / 4-5 |
| 同步字 | 0x34 |
| Preamble / 头类型 / CRC / IQ | 8 symbol / 显式头 / 开 / 不反转 |
| 载荷长度 / 间隔 / 包数 | 64 B / 1000 ms / 无限 |
| LDRO | 自动（符号时间 > 16 ms 开启） |

## 2. 构建

```bash
# 交叉编译（AM62L, aarch64）
cmake -S . -B build_cross -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake
cmake --build build_cross -j
# 产物: build_cross/demo/sx126x_demo

# 本机验证用（可选）
cmake -S . -B build_host && cmake --build build_host -j
```

## 3. 硬件资源依赖

| 资源 | 位置 | 说明 |
|---|---|---|
| SPI | `/dev/spidev3.0` | 片选 0 |
| RESET | GPIO 553 | 基地址 519 + 34 |
| BUSY | GPIO 607 | 基地址 519 + 88 |
| DIO1 | GPIO 613 | 基地址 519 + 94（当前未用中断，仅轮询 IRQ 状态） |
| RF_SW | GPIO 560 | 基地址 519 + 41，**高=RX，低=TX** |

固件内固定配置：供电 LDO（板上未焊 DCDC）、DIO2 自动控制射频开关、DIO3 TCXO 1.8V、RX boosted、镜像校准按频段自动（US915→902-928，EU868→863-870）。

## 4. 命令行参考

```
sx126x_demo <tx|rx|cw> [options]
  -f, --freq <Hz>      RF 频率（150M-960M）
  -p, --power <dBm>    发射功率（-9 ~ 22，越界自动钳位）
  -s, --sf <5-12>      扩频因子
  -b, --bw <kHz>       带宽 {7,10,15,20,31,41,62,125,250,500}
  -r, --cr <5-8>       编码率 4/r
  -w, --sync <hex>     同步字（如 0x34）
  -l, --len <bytes>    载荷长度 1-255（仅 tx）
  -n, --cnt <n>        包数（0=无限；rx 指收满 n 个好包退出）
  -i, --interval <ms>  发包间隔（最小 10，仅 tx）
  -N, --nointer        背靠背连发（忽略间隔，认证用连续调制波）
  -P, --inp            无限前导码发射（仅 tx）
```

退出：`Ctrl+C`（任意模式）→ 打印统计 summary → 芯片回 Standby → RF 开关回接收态 → 退出。**CW / 无限前导码模式下必须用 Ctrl+C 停止**，否则进程退出后芯片仍在发射。

## 5. 测试用例

### TC-01 参数解析自检
- 步骤：`sx126x_demo --help`
- 预期：输出 Usage 与全部选项；退出码 0

### TC-02 非法参数拦截
- 步骤与预期：

| 命令 | 预期输出 | 退出码 |
|---|---|---|
| `tx -f 90300000` | `ERROR: freq 90300000 Hz out of SX1262 range (150M-960M).` | 1 |
| `rx -b 300` | `ERROR: unsupported bandwidth 300 kHz.` | 1 |
| `tx -s 13` | `ERROR: SF must be 5-12.` | 1 |
| `cw -p 30` | `WARNING: power 30 dBm > 22, clamp to 22.` 后按 22 dBm 继续 | 0 |

### TC-03 SPI/芯片链路自检
- 步骤：任一模式启动（如 `tx -n 1`）
- 预期输出包含：
  - `Chip Status: CmdStatus=…, ChipMode=…`（ChipMode 非 0/0xFF）
  - `Hardware Version: 0x12`
  - `Image calibration: 902-928 MHz`（US915 频段时）
- 异常：`ERROR: SPI Read failed …` → 查 MISO 接线/设备树

### TC-04 定频发包（默认 FCC DTS）
- 步骤：`sx126x_demo tx -n 10`
- 预期：10 行 `[n] TX DONE`，结束时 `Reached target 10 packets.` + `TX summary: m_tx_cnt:10, m_tx_timeout:0`

### TC-05 双机对测 PER（tx ↔ rx）
- 发射端：`sx126x_demo tx -n 1000`
- 接收端：`sx126x_demo rx -n 1000`（两端参数组必须一致）
- 预期：rx 端每包 `RX Done: n, len=…, rssi=… dBm, snr=… dB, data: Ping …`
- 结束输出：`RX summary: m_rx_packet:N, packet_ok: N, packet_err:…, packet_crc_err:…`
- 判定：PER = err/(ok+err)；近距离 22 dBm 可能饱和，建议发射端 `-p 10` 或拉开 >1 m

### TC-06 参数组切换（EU868）
- 发射端：`sx126x_demo tx -f 876100000 -b 125 -p 16 -n 100`
- 接收端：`sx126x_demo rx -f 876100000 -b 125`
- 预期：正常互通；启动横幅显示 `freq=876100000 Hz, power=16 dBm`、`SF7 / BW125k / CR4/5`，且 `Image calibration: 863-870 MHz`

### TC-07 CW 单载波（认证：功率/频率稳定度）
- 步骤：`sx126x_demo cw -f 903000000 -p 22`，功率计/频谱仪接 SMA
- 预期：`Transmitting CW carrier. Press Ctrl+C to stop.`，仪表读到 903 MHz 单载波；Ctrl+C 后 `CW stopped.` 且仪表归零（验证退出路径确实停发射）

### TC-08 背靠背连发（认证：OBW/带外杂散）
- 步骤：`sx126x_demo tx -N`（默认 903M/BW500/22dBm）
- 预期：TX DONE 之间无间隔；频谱仪测 99% 占用带宽与杂散；测完 Ctrl+C

### TC-09 无限前导码
- 步骤：`sx126x_demo tx -P`
- 预期：`Transmitting infinite preamble. Press Ctrl+C to stop.`，频谱仪可见连续前导码调制

### TC-10 Ctrl+C 干净退出（全模式）
- 步骤：tx / rx / cw / `-P` 各模式下 Ctrl+C
- 预期：打印 summary → 进程退出码 0；CW/`-P` 停止后频谱仪无发射残留

## 6. 认证频点速查（对齐参考工程 rse 认证测试指令）

**FCC US915 DTS（LoRa 500 kHz / 22 dBm，默认 SF7）**：

| 信道 | 频率 (Hz) | 命令 |
|---|---|---|
| 1~8 | 903000000 / 904600000 / 906200000 / 907800000 / 909400000 / 911000000 / 912600000 / 914200000 | `sx126x_demo tx -N -f <freq> -p 22` |

> 注：参考指令原文中 `-f 90300000`（90.3 MHz）为笔误，正确值为 903000000。

**CE EU868（LoRa 125 kHz / 16 dBm）**：

| 信道 | 频率 (Hz) | 命令 |
|---|---|---|
| 1~4 | 876100000 / 876300000 / 876500000 / 876700000 | `sx126x_demo tx -N -f <freq> -b 125 -p 16` |

**CW**：`sx126x_demo cw -f <freq> -p <dBm>`
**未实现**（本工具不做）：FSK/GFSK 定频（CE EU868_2 的 868.8M）、FHSS 跳频、LR-FHSS、2.4G（SX1262 不支持）。

## 7. 与参考工程（nRF52840+LR2021 产测固件）互通对照

互通条件：双方 频率/SF/BW/CR/同步字/CRC/头类型 一致。

| 参考测试 | 参考端参数 | 本工具等价命令 |
|---|---|---|
| `subg_tx` / `subg_rx` | 868M / SF7 / BW125 / 4-5 / 0x34 / 10 dBm | `tx -f 868000000 -b 125 -p 10`（rx 同参数） |
| `meshtastic_tx/rx -range 0` | 868M / SF7 / BW250 / 4-5 / 0x34 / 255B | `tx -f 868000000 -b 250 -l 255`（载荷格式不同，仅验物理层收包/RSSI） |
| `subg_ping` / `subg_pong` | SF9 / BW125 / 0x34（双频乒乓） | `tx -f 868000000 -s 9 -b 125`（单频，无乒乓逻辑） |
| `subg_tx_cw` | CW | `cw -f <freq>` |

## 8. 已知注意事项

1. **TX 超时**按空中时间自动推导（约 2×airtime+200ms），SF12 长包不会误报超时；TX 频繁 `TX TIMEOUT` 先查频率/天线。
2. **RX 为连续模式**：芯片收完自动回 RX，工具只清 IRQ 标志不重启接收（重启会打断在收的包）。
3. **同步字**传 LoRaWAN 风格 8-bit 值（0x34/0x12），驱动内部映射到 16-bit 寄存器（0x34→0x3444），与参考工程 RAL 行为一致。
4. **供电固定 LDO**：板上未焊 DCDC 电感，不要改 `SX126X_REG_MODE_DCDC`。
5. RX 统计中 `packet_err` 即 CRC 错误数（本工具未细分 header error）。
6. 长时间 RX 空闲时每 ~5 s 打印一行 `[waiting irq=0x…]` 心跳，属正常。
