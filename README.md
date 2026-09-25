# ControllerCode

> **下位机（MCU）** · owner 角色：电控
> ⚠️ **不在 L1–L5 分层内**——下位机固件位于 L2 下行 HAL **之下**

mc02(h723)和官方c板（f407）电控代码

---

## 快速开始

```bash
git clone git@github.com:ZQU-Foray/ControllerCode.git
cd ControllerCode

# 构建：按目标板选择工程文件，见 Platform/STM32H7/
#   DM_MC02  → STM32H723
#   官方 C 板 → STM32F407
```

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | `foray_interfaces`——**仅消费**生成的 `generated/lower_link.hpp`（vendor，非构建期依赖） |
| **第三方** | STM32 HAL / LL 驱动（已随仓） |

## 接口

### 本仓实现的链路协议

上位机 ↔ 下位机链路协议由 **`foray_interfaces`** 定义（X2 契约）：

> 📄 [`foray_interfaces/protocol/lower_link.md`](https://github.com/ZQU-Foray/foray_interfaces/blob/dev/protocol/lower_link.md)

**物理层：USB CDC。** 本仓已有传输层：

| 层 | 位置 | 状态 |
|---|---|---|
| 物理 / 传输 | `Platform/Interface/UsbCdc.hpp` | ✅ 已有（非阻塞 + 收发统计） |
| **应用层协议编解码** | 待接入 | ⏳ 等 `lower_link.yaml` 冻结 |

> ⚠️ **应用层编解码必须使用生成的头文件，禁止手写。**
> 生成路径：`foray_interfaces/generated/lower_link.hpp`
> 手写两侧编解码必然在字节序、字段偏移、CRC 上漂移。

### 对外暴露的抽象接口

`Platform/Interface/` 提供跨平台抽象（依赖倒置：抽象在此，实现在 `Platform/STM32H7/`）：

`Can` · `Gpio` · `Pwm` · `Spi` · `Time` · `Uart` · `UsbCdc`

## 下位机承担的安全职责（硬路径）

> **链路失联后的安全必须由下位机自主保证，不得依赖上位机。**

| 失联时长 | 动作 |
|---|---|
| 100 ms | 底盘速度归零 |
| 200 ms | **撤销开火授权**，摩擦轮停 |
| 500 ms | 云台停止接受指令，保持当前位置 |

这些定时器跑在 MCU 上，**与 USB 中断无关**——USB 断了它们照常触发。

**开火硬上限**：本仓须实现独立于上位机的连发数上限与发射间隔下限，不接受任何来源的覆盖。
详见链路协议规范 §6.3。

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces`（仅消费生成的头文件） |
| **下游**（谁依赖本仓） | 无——最底层硬件 |

```
上位机（算法组）
   foray_platform / foray_decision / ...
        │   USB CDC
        │   （契约：foray_interfaces/protocol/lower_link.md）
        ▼
下位机（电控组）
   ControllerCode  ← 本仓
        │   CAN / UART / PWM
        ▼
   电机 · 云台 · 发射机构 · IMU
```

## 参考

- [上位机↔下位机链路协议](https://github.com/ZQU-Foray/foray_interfaces/blob/dev/protocol/lower_link.md)
- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
