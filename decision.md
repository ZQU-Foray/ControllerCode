# 关键工程决策日志

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。

---

## Decision

Date: 2026-02-19

Context: 本仓是下位机（MCU）固件，此前未对齐组织仓库规范——无五件套、无 `CODEOWNERS`、
无层级声明。同时上位机↔下位机链路协议刚在 `foray_interfaces` 落定 v0.1（USB CDC）。

Decision:
1. 本仓层级声明为 **`layer: MCU`**，**不在 L1–L5 分层内**——下位机位于 L2 下行 HAL 之下
2. 依赖方向：`depends_on_layers: [X2]`，但**仅消费生成的头文件**（vendor），非构建期依赖
3. 应用层协议编解码**必须使用** `foray_interfaces/generated/lower_link.hpp`，禁止手写
4. `CODEOWNERS` 指向 **`@ZQU-Foray/电控组`**
5. 链路失联的安全降级**由本仓自主实现**（硬路径），不依赖上位机

Reason:
- 算法结构的 6 层 + 装配根**全部是上位机**；下位机固件不在这套分层里，
  但 CI 的依赖方向校验需要一个显式声明，故用 `MCU` 而非硬塞进 `L0`
- 手写两侧编解码必然在字节序 / 字段偏移 / CRC 上漂移——生成是唯一可靠解法
- 链路失联时上位机可能已经死了；安全不能依赖一个可能已经不在的进程

Alternatives:
- 把本仓标为 `L0` —— 否决：会让它看起来像"最底层软件库"，而它其实是硬件侧固件
- 把协议规范放本仓 —— 否决：判据 C1/C6，跨队契约应放 X2 契约仓
  （详见 `foray_interfaces/decision.md`）

Rejected:
- 上位机与下位机各自维护一份编解码 —— 漂移风险不可接受
- 安全定时器依赖上位机心跳 —— 链路断了就失效，与"硬路径"要求矛盾
