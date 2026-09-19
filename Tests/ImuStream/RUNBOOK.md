# BMI088 原始数据出口上板 runbook（阶段 1）

本文只描述需要实机的步骤。所有命令均在 Git Bash（WSL）中执行；Windows 侧烧录用
CubeMX/OpenOCD 现有流程。本阶段不改任何传感器寄存器、`.ioc` 或 CubeMX 生成文件。

## 0. 身份与前置

```sh
cd Platform/STM32H7/DM_MC02
cmake --build --preset WSL-Debug
cmake --build --preset WSL-Release
sha256sum build/WSL-Release/mc02_cpp.elf build/WSL-Release/mc02_cpp.bin
arm-none-eabi-size build/WSL-Release/mc02_cpp.elf
```

记录本次 ELF/BIN 的 SHA-256 和尺寸，作为本阶段归档身份。烧录 Release 固件并复位。

固件行为：

- 上电后原始出口**默认关闭**，IMU 采集照常运行。
- 主机通过 USB CDC（虚拟串口）发送单字节命令：
  - `'R'`（0x52）：开启出口，开始发送原始样本帧。
  - `'P'`（0x50）：关闭出口，停止复制样本（队列中剩余数据仍会发完）。
  - 其他字节计入 `unknownCommandBytes`，无副作用。

帧格式见 `Libraries/Protocol/imu/ImuFrame.hpp`，主机工具见本目录
`imu_stream_tool.py`。

## 1. 测试 A：输出关闭基线（10 min）

1. 烧录并复位，**不要**发送 `'R'`。
2. 保持 J-Link 连接，按 `Tests/ImuAcquisition/README.md` 的既有流程等待温度合格、
   读取 `ImuDiagnostics` 冻结统计。
3. 归档：两路 gap / coalesced / overflow / DMA error 计数、start/completed/harvested
   tick 分布、温度区间、固件散列。
4. 这是"输出关闭"的采集健康基线，后续与测试 B 对比。

## 2. 测试 B：输出开启（10 min）

1. 复位后确认 USB CDC 端口号（Windows 设备管理器，例如 `COM5`）。
2. 启动主机采集（会先发 `'R'`）：

```sh
python3 Tests/ImuStream/imu_stream_tool.py capture \
  --port COM5 --out build/raw-on-$(date +%Y%m%d-%H%M%S).bin --seconds 600
```

3. 采集结束后工具会自动发送 `'P'` 停止出口。
4. 离线解码并自动验收：

```sh
python3 Tests/ImuStream/imu_stream_tool.py decode \
  --bin build/raw-on-<时间戳>.bin --csv build/raw-on-<时间戳>.csv
```

工具会检查：CRC 错误、结构非法帧、首帧后重同步、帧序号缺口、窗口内丢弃、
两路实测速率、`drdyTick` 间隔是否 99.9% 落在标称 ±10% 内。退出码 0 = PASS。

5. 对照测试 A：输出开/关的采集计数应同为 0；比较两方面延迟分布是否显著恶化。

## 3. 测试 C：断连重连

1. 出口开启并持续采集时，**直接拔掉 USB 线**，保持 10 s。
2. 观察板端不因断连阻塞：IMU 任务继续采集，队列满后丢弃并累加计数，
   `transmitNotReadyCount` 增长。
3. 重新插线，重新打开端口并再次发送 `'R'`，继续采集 60 s。
4. 解码新采集片段，确认：
   - 重连后首个有效帧的 `droppedRecords` 非零（窗口内丢弃被如实报告）；
   - 重连后帧序号继续单调递增（不重置）；
   - 原始 `xyz`/时间戳字段完整可解。
5. 归档断开区间长度与丢弃样本数。

## 4. 阶段 1 通过标准

- 测试 A/B 中两路新增 gap / coalesced / overflow / DMA error 均为 0；
- 测试 B 主机侧 CRC 错误、结构非法帧、首帧后重同步、帧序号缺口均为 0；
- 陀螺平均速率接近 2000 Hz、加速度计接近 1600 Hz；
- 99.9% 的 `drdyTick` 间隔落在标称周期 ±10% 内；
- 每条记录保留原始 `xyz` 与时间戳，主机解码与板端入队逐位一致；
- 断连不拖慢或阻塞 `ImuTask`，出口明确报告丢失区间；
- 未修改任何 BMI088 配置与 CubeMX 生成文件。

任一项不通过即报告 FAIL，停在本阶段修复后重测；不得通过放宽阈值或删除异常区间"通过"。

## 5. 归档命名

```text
build/raw-<on|off>-<日期>-<固件短散列>-<启动类型>-<动作>-<重复序号>.bin
build/raw-<on|off>-<日期>-<固件短散列>-<启动类型>-<动作>-<重复序号>.csv
build/raw-<...>.report.txt   # imu_stream_tool.py decode 的完整输出
```

失败数据不得覆盖，重复实验只增加重复序号。
