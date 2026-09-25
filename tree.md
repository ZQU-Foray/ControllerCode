# 目录结构说明

## 目录结构

```
.
├── README.md                本仓定位、协议接入点、安全职责
├── AGENTS.md                AI 协作规范
├── plan.md                  开发计划与进度
├── decision.md              关键工程决策日志
├── tree.md                  本文件
├── CODEOWNERS               Review 自动分派（电控组）
├── .foray-layer             所属层与依赖边界
├── .gitignore
├── Application/             应用层
│   ├── Entry.cpp/.h             入口
│   ├── Imu/                     姿态估计 · 零偏标定 · 诊断
│   └── Task/                    任务（按编号 = 执行优先级）
│       ├── TaskManager.cpp/.h       任务调度
│       ├── 0_imu/                   IMU 数据就绪
│       ├── 1_chassis/               底盘控制
│       └── 4_communication/         遥控器接收（SBUS）
├── Libraries/               算法与设备库
│   ├── Algorithm/               算法（含 .clang-format）
│   ├── Device/                  设备驱动（motor 等）
│   └── Protocol/                协议实现
│       ├── dji/                     DJI 电调（CAN）
│       └── sbus/                    遥控器 SBUS（UART）
├── Platform/                平台层
│   ├── Interface/               跨平台抽象（依赖倒置：抽象在此）
│   │   ├── Can · Gpio · Pwm · Spi · Time · Uart · UsbCdc
│   │   └── Detail/                  各平台的 C 层头
│   ├── PortKit/                 移植工具
│   └── STM32H7/                 STM32H7 实现（MC02）
└── .github/workflows/ci.yml
```

## 关键位置

| 要找什么 | 去哪 |
|---|---|
| **USB CDC 传输层**（对上位机的链路） | `Platform/Interface/UsbCdc.hpp` |
| 跨平台抽象接口 | `Platform/Interface/*.hpp` |
| 具体平台实现 | `Platform/STM32H7/` |
| 协议实现样板 | `Libraries/Protocol/{dji,sbus}` |
| 任务调度与优先级 | `Application/Task/TaskManager.hpp` |

> **应用层协议编解码尚未接入**——等 `foray_interfaces` 的 `lower_link.yaml` 冻结后，
> 用其生成的 `generated/lower_link.hpp` 实现，**禁止手写**。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`.git/` 等依赖与产物目录
- STM32 HAL 驱动的逐文件清单（属第三方，只记 `Platform/STM32H7/` 一层即可）
- 临时文件、缓存文件、日志文件
