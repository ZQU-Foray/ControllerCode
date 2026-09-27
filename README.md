mc02(h723)和官方c板（f407）电控代码

## 当前文件结构

 - Application 应用层
 - Libraries   常用库
 - Platform    平台层

```text
controller_code/
├── Application/
│   ├── Imu/
│   ├── Module/
│   └── Task/
│       ├── 0_imu/
│       ├── 1_chassis/
│       ├── 2_gimbal/
│       └── 4_communication/
├── Libraries/
│   ├── Algorithm/
│   │   ├── alg_controller/
│   │   ├── alg_crc/
│   │   ├── alg_estimate/
│   │   ├── alg_filter/
│   │   ├── alg_fsm/
│   │   ├── alg_kinematics/
│   │   ├── alg_math/
│   │   └── alg_slope/
│   ├── Component/
│   ├── Device/
│   │   ├── bmi088/
│   │   ├── buzzer/
│   │   ├── motor/
│   │   │   └── dji/
│   │   └── ws2812/
│   └── Protocol/
│       ├── dji/
│       └── sbus/
├── Platform/
│   ├── Interface/
│   ├── PortKit/
│   ├── STM32F4/
│   │   └── RM_Cboard/
│   └── STM32H7/
│       └── DM_MC02/
│           └── Port/
│  
└── README.md
```
