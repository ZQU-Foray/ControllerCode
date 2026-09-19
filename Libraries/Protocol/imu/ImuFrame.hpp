#ifndef LIBRARIES_PROTOCOL_IMU_IMU_FRAME_HPP
#define LIBRARIES_PROTOCOL_IMU_IMU_FRAME_HPP

#include <cstddef>
#include <cstdint>

namespace protocol {

/**
 * @brief BMI088 原始样本的版本化二进制线格式编解码。
 * @note 本模块与平台、RTOS、设备驱动无关，可在主机上独立测试。
 *       所有多字节字段一律小端编码，长度以字节计，不依赖编译器位域或
 *       packed 结构体，避免跨编译器 ABI 差异。
 * @note 帧格式只承载原始数据与元数据，不做任何换算、滤波或单位变换；
 *       整数计数必须原样保留，供主机逐位比对。
 *
 * 帧布局（小端）：
 *   偏移  长度  字段
 *   0     2     魔数 0x4952
 *   2     1     版本
 *   3     1     帧类型（0 = 原始样本批次）
 *   4     4     配置 ID（陀螺带宽/量程、加速度计配置/量程各一字节）
 *   8     4     tick 频率（Hz）
 *   12    4     帧序号（每发送一帧加一，用于缺口检测）
 *   16    2     本帧样本数
 *   18    2     标志位
 *   20    4     累计入队丢弃样本数
 *   24    4     温度读数序号
 *   28    4     温度完成 tick
 *   32    4     温度摄氏度（IEEE-754 小端）
 *   36    1     温度有效标志
 *   37    3     保留（写 0，读忽略）
 *   40    N*36  样本记录
 *   40+N*36 4   CRC-32（覆盖前面全部字节）
 */
class ImuFrame final {
public:
  static constexpr std::uint16_t Magic{0x4952U};
  static constexpr std::uint8_t Version{1U};
  static constexpr std::size_t MaxTransmitSize{512U};
  static constexpr std::size_t HeaderSize{40U};
  static constexpr std::size_t RecordSize{36U};
  static constexpr std::size_t CrcSize{4U};
  static constexpr std::size_t OverheadSize{HeaderSize + CrcSize};
  static constexpr std::size_t MaxRecordsPerFrame{
      (MaxTransmitSize - OverheadSize) / RecordSize};

  static_assert(MaxRecordsPerFrame >= 1U,
                "frame must carry at least one sample record");
  static_assert(OverheadSize + MaxRecordsPerFrame * RecordSize <=
                    MaxTransmitSize,
                "encoded frame exceeds the transport limit");

  /** 样本来源传感器。 */
  enum class Sensor : std::uint8_t {
    Accelerometer = 0U,
    Gyroscope = 1U
  };

  /** 帧类型；当前只定义原始样本批次，其余取值保留。 */
  enum class FrameType : std::uint8_t { Samples = 0U };

  /** 帧标志位。 */
  enum Flags : std::uint16_t {
    NoFlags = 0U,
    QueueOverflow = 1U << 0U,  // 自上一帧以来队列满并丢弃过样本
    StreamEnabled = 1U << 1U,  // 组帧时原始出口开关处于开启状态
  };

  /** 一条原始样本记录，字段与 Bmi088SampleRecord 一一对应。 */
  struct Sample final {
    std::uint32_t sequence{0U};
    std::uint32_t drdySequence{0U};
    std::uint32_t drdyTick{0U};
    std::uint32_t startTick{0U};
    std::uint32_t completedTick{0U};
    std::uint32_t harvestedTick{0U};
    std::int16_t xyz[3]{};
    std::uint16_t flags{0U};
    Sensor sensor{Sensor::Accelerometer};
  };

  /** 温度元数据；序号用于识别新读数，带外与样本流并行。 */
  struct Temperature final {
    std::uint32_t sequence{0U};
    std::uint32_t tick{0U};
    float celsius{0.0F};
    bool valid{false};
  };

  /** 帧头元数据。 */
  struct Header final {
    std::uint32_t configId{0U};
    std::uint32_t tickHz{0U};
    std::uint32_t frameSequence{0U};
    std::uint16_t sampleCount{0U};
    std::uint16_t flags{NoFlags};
    std::uint32_t droppedRecords{0U};
    Temperature temperature{};
  };

  ImuFrame() = delete;

  /**
   * @brief 由传感器寄存器值拼出配置 ID，使主机能识别数据来自哪套锁定配置。
   * @param gyroBandwidthRegister 陀螺 GYRO_BANDWIDTH 寄存器值。
   * @param gyroRangeRegister 陀螺 GYRO_RANGE 寄存器值。
   * @param accelConfRegister 加速度计 ACC_CONF 寄存器值。
   * @param accelRangeRegister 加速度计 ACC_RANGE 寄存器值。
   * @return 四字节依次为 带宽、陀螺量程、加速配置、加速量程 的 32 位 ID。
   */
  [[nodiscard]] static constexpr std::uint32_t
  MakeConfigId(std::uint8_t gyroBandwidthRegister,
               std::uint8_t gyroRangeRegister, std::uint8_t accelConfRegister,
               std::uint8_t accelRangeRegister) noexcept {
    return (static_cast<std::uint32_t>(gyroBandwidthRegister) << 24U) |
           (static_cast<std::uint32_t>(gyroRangeRegister) << 16U) |
           (static_cast<std::uint32_t>(accelConfRegister) << 8U) |
           static_cast<std::uint32_t>(accelRangeRegister);
  }

  /**
   * @brief 计算给定样本数编码后的字节数。
   * @param sampleCount 样本数，超过 MaxRecordsPerFrame 时返回 0。
   */
  [[nodiscard]] static constexpr std::size_t
  EncodedSize(std::uint16_t sampleCount) noexcept {
    return sampleCount > MaxRecordsPerFrame
               ? 0U
               : OverheadSize + static_cast<std::size_t>(sampleCount) * RecordSize;
  }

  /**
   * @brief 编码一帧。
   * @param header 帧头；sampleCount 必须与 records 中有效记录数一致。
   * @param records 样本记录数组，sampleCount 为 0 时可为 nullptr。
   * @param output 输出缓冲区。
   * @param capacity output 可容纳的字节数。
   * @return 写入的字节数；参数非法或缓冲区不足时返回 0。
   */
  [[nodiscard]] static std::size_t Encode(const Header &header,
                                          const Sample *records,
                                          std::uint8_t *output,
                                          std::size_t capacity) noexcept;

  /**
   * @brief 解码一帧并校验魔数、版本、长度与 CRC。
   * @param input 输入缓冲区。
   * @param length 输入字节数。
   * @param header 输出帧头；成功时 sampleCount 为实际记录数。
   * @param records 输出记录数组，可为 nullptr（此时 maxRecords 必须为 0）。
   * @param maxRecords records 可容纳的记录数。
   * @return 校验全部通过时返回 true；任何一项失败返回 false 且不保证输出有效。
   */
  [[nodiscard]] static bool Decode(const std::uint8_t *input,
                                   std::size_t length, Header &header,
                                   Sample *records,
                                   std::size_t maxRecords) noexcept;
};

} // namespace protocol

#endif
