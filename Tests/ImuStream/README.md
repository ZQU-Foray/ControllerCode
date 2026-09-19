# BMI088 原始数据出口回归

构建本目录为独立的主机 CMake 工程并运行 CTest：

```sh
cmake -S Tests/ImuStream -B /tmp/controller-raw-imu-tests
cmake --build /tmp/controller-raw-imu-tests
ctest --test-dir /tmp/controller-raw-imu-tests --output-on-failure
```

## 覆盖范围

`imu_frame`：`Libraries/Protocol/imu/ImuFrame` 与
`Libraries/Algorithm/alg_crc/Crc32`。覆盖 CRC-32 已知向量、显式小端字节序、
字段逐位往返、零样本帧、512 字节上限、魔数/版本/长度/CRC/样本数越界等失败路径。

`imu_stream`：`Application/Imu/ImuStream`。用桩实现
`UsbCdcPort_*` 与 `TimePort_FrequencyHz`，覆盖默认关闭、主机 `'R'`/`'P'`
开关、批量组帧与单帧上限、USB Busy/NotReady 时保留数据、队列溢出计数、
温度 seqlock 与重复读数去重、帧序号递增、`Init` 复位。

## 约束

- 编解码不做任何换算、滤波或单位变换，整数计数原样保留供逐位比对。
- 出口默认关闭，只在收到主机 `'R'` 后开始复制样本，便于测量纯采集基线。
- 队列满时丢弃最新样本并计数，绝不阻塞 IMU 任务或覆盖未发送数据。
