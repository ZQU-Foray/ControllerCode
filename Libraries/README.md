### 通用代码库，一般情况下适用于h723和f407

### 目录结构

```text
Libraries/
├─ Algorithm/
│  ├─ alg_controller/    控制算法（PID、Fuzzy PID）
│  ├─ alg_crc/           CRC 校验
│  ├─ alg_estimate/      状态估计（AHRS、EKF、Kalman Filter、陀螺仪零偏标定）
│  ├─ alg_filter/        数字滤波器
│  ├─ alg_fsm/           有限状态机
│  ├─ alg_kinematics/    底盘运动学与里程计
│  ├─ alg_math/          基础数学函数
│  └─ alg_slope/         斜坡规划
└─ README.md
```

### CMSIS-DSP 特殊依赖

大部分算法只依赖 C/C++ 标准库；`alg_estimate/KalmanFilter` 直接依赖 CMSIS-DSP 的 `arm_math.h` 和矩阵运算接口，`alg_estimate/EKF` 通过 `KalmanFilter` 间接依赖 CMSIS-DSP。

使用这两个模块时，目标工程需要提供与处理器内核及浮点配置匹配的 CMSIS-DSP 头文件和实现。当前 H723 工程使用 `Middlewares/ST/ARM/DSP/Inc`，并链接 `arm_cortexM7lfsp_math` 静态库。移植到 F407 等其他平台时，需要按实际 Cortex-M 内核、FPU 和 ABI 选择对应的 CMSIS-DSP 库，不能直接复用 H723 的 M7 静态库。
