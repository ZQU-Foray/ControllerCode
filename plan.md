# 开发计划

## 任务列表

- [ ] 接入应用层协议编解码（用 `foray_interfaces/generated/lower_link.hpp`，**禁止手写**）
- [ ] 实现链路失联安全态定时器（100 ms 底盘归零 / 200 ms 撤销开火 / 500 ms 云台保持）
- [ ] 实现开火硬上限（连发数上限 + 发射间隔下限，独立于上位机）
- [ ] 实现 `HANDSHAKE` / `HEARTBEAT` / `LINK_ERROR` 链路管理
- [ ] 与算法组确认 `lower_link.md` §9 的 6 个待确认项
- [ ] 建 `Application/Task/` 下的通信任务（USB CDC 链路收发）

## 当前 Agent State

PLAN_READY

## Before Snapshot

commit hash:   （本次补齐五件套前）
branch:        dev
modified files: README.md · AGENTS.md · plan.md · tree.md · decision.md · CODEOWNERS · .foray-layer
risk level:    L1

## 模糊点与待确认项

- **原文 `README.md` 只有一行描述**（「mc02(h723)和官方c板（f407）电控代码」），
  本次仅在其基础上扩写，未删除原文。仓库的完整需求仍需电控组补充。
- `Application/Task/` 的任务编号为 `0_imu` · `1_chassis` · `4_communication`，
  **`2_` 与 `3_` 缺位**——是预留（云台 / 发射？）还是尚未实现？需电控组确认。
- 裁判系统接在下位机还是上位机？——决定是否要实现 `REFEREE_RAW` 转发。
- MCU USB 外设配置为全速还是高速？——决定链路调度延迟量级。
- 官方 C 板（F407）与 MC02（H723）是否共用同一套任务结构？两者的能力差异如何声明？

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （本次提交）

## Acceptance Criteria

- [ ] 应用层编解码完全由生成的头文件驱动，仓内无手写帧解析
- [ ] 拔掉 USB 线后，底盘在 100 ms 内归零、开火授权在 200 ms 内撤销
- [ ] 开火硬上限在任何上位机指令下都不可突破
- [ ] CI 绿灯
