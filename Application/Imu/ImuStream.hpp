#ifndef APPLICATION_IMU_IMU_STREAM_HPP
#define APPLICATION_IMU_IMU_STREAM_HPP

#include "Application/Imu/ImuQueue.hpp"
#include "Libraries/Device/bmi088/Bmi088SampleQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace application {

/**
 * @brief BMI088 双路原始样本的无损版本化出口。
 * @note 生产者是 IMU 任务：只做常数时间副本入队，绝不阻塞、绝不覆盖未发送
 *       数据；队列满时丢弃最新样本并累加计数。
 * @note 消费者是通信任务：批量窥视队首样本、组帧、非阻塞发送；只有发送成功
 *       才推进队列 tail，因此 USB 忙或未连接时数据保留在队列中。
 * @note 出口默认关闭，由主机命令开启；关闭时生产者不做任何复制，用于测量
 *       纯采集基线。出口数据只承载原始计数与时间戳，不做换算或滤波。
 * @note 队列存储位于 AXI SRAM（NOLOAD），Init 显式复位索引，不依赖静态零初始化。
 */
class ImuStream final {
public:
  static constexpr std::size_t QueueCapacity{1024U};
  using Queue = ImuQueue<QueueCapacity>;

  ImuStream() = delete;

  /**
   * @brief 复位队列与统计并锁定配置 ID。必须在生产者运行前调用。
   * @return 初始化成功返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  [[nodiscard]] static bool IsReady() noexcept;
  [[nodiscard]] static bool IsEnabled() noexcept;

  /**
   * @brief 开启或关闭出口。关闭时生产者跳过入队。
   */
  static void SetEnabled(bool enabled) noexcept;

  /**
   * @brief 生产者入队一条原始样本。常数时间。
   * @param record 驱动收割到的原始记录。
   * @param gyroscope true 表示陀螺仪样本，false 表示加速度计样本。
   */
  static void Push(const device::Bmi088SampleRecord &record,
                   bool gyroscope) noexcept;

  /**
   * @brief 生产者发布最新温度读数（序列号、完成 tick、摄氏度、有效位）。
   * @note 由 IMU 任务从温度子设备提取后调用；本模块不依赖具体设备驱动。
   */
  static void PublishTemperature(std::uint32_t sequence, std::uint32_t tick,
                                 float celsius, bool valid) noexcept;

  /**
   * @brief 消费者处理主机命令：'R' 开启出口，'P' 关闭出口。
   */
  static void ProcessCommands() noexcept;

  /**
   * @brief 消费者批量发送一批原始样本；USB 忙或未连接时保留数据等待下一轮。
   */
  static void Drain() noexcept;

  struct Statistics final {
    std::uint32_t pushedRecords{0U};
    std::uint32_t droppedRecords{0U};
    std::uint32_t queuedRecords{0U};
    std::uint32_t sentFrames{0U};
    std::uint32_t sentBytes{0U};
    std::uint32_t transmitStartedCount{0U};
    std::uint32_t transmitBusyCount{0U};
    std::uint32_t transmitNotReadyCount{0U};
    std::uint32_t transmitErrorCount{0U};
    std::uint32_t receivedCommandBytes{0U};
    std::uint32_t unknownCommandBytes{0U};
  };

  [[nodiscard]] static Statistics GetStatistics() noexcept;
};

} // namespace application

#endif
