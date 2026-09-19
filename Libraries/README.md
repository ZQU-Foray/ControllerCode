### 通用代码库，一般情况下适用于h723和f407

### 目录结构

```text
Libraries/
├─ Algorithm/
│  ├─ alg_controller/    控制算法（PID、Fuzzy PID）
│  ├─ alg_crc/           CRC 校验
│  ├─ alg_filter/        数字滤波器
│  ├─ alg_fsm/           有限状态机
│  ├─ alg_kinematics/    底盘运动学与里程计
│  ├─ alg_math/          基础数学函数
│  └─ alg_slope/         斜坡规划
└─ README.md
```

### 状态估计算法

姿态解算与陀螺仪零偏估计算法（`alg_estimate/`）已整体移动到
`Tests/backups/Libraries/Algorithm/alg_estimate/`，仅供后续参考；当前主工程只保留
BMI088 原始数据采集与 `Application/Imu/ImuDiagnostics` 原始数据输出。
