#ifndef APPLICATION_TASK_COMMUNICATION_TASK_HPP
#define APPLICATION_TASK_COMMUNICATION_TASK_HPP

#include <cstdint>

namespace application::task {

/**
 * @brief 通信任务：以低优先级批量发送 BMI088 原始数据出口并处理主机命令。
 * @note 只消费 ImuStream 已入队的原始样本副本，不接触 BMI088 驱动、
 *       SPI、DMA 或传感器寄存器；USB 忙或未连接时数据保留在队列中，
 *       由有界队列在满时计数丢弃，绝不阻塞 IMU 采集。
 */
class CommunicationTask final {
public:
  CommunicationTask() = delete;

  /**
   * @brief 初始化原始数据出口。必须在 IMU 生产者启动前调用。
   * @return 出口初始化成功时返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  /**
   * @brief CMSIS-RTOS2 任务入口，由外部创建线程后调用。
   */
  [[noreturn]] static void Run(void *argument) noexcept;

  [[nodiscard]] static bool IsReady() noexcept;

private:
  static constexpr std::uint32_t PeriodMs{1U};
};

} // namespace application::task

#endif
