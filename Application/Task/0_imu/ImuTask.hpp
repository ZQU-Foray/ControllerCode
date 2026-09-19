#ifndef APPLICATION_TASK_IMU_TASK_HPP
#define APPLICATION_TASK_IMU_TASK_HPP

namespace application::task {

/**
 * @brief BMI088 原始数据采集与诊断输出任务。
 * @note 本任务独占 BMI088 与恒温器；两路 DRDY 通过 ISR 线程标志唤醒任务。
 *       任务只负责把采集到的原始样本交给 ImuDiagnostics 记录/发布，
 *       不再包含姿态解算、零偏标定或任何原始数据融合逻辑。
 */
class ImuTask final {
public:
  ImuTask() = delete;

  /**
   * @brief 在 RTOS 启动前初始化 BMI088 状态机与恒温器。
   * @return 底层恒温 PWM 就绪时返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  /**
   * @brief CMSIS-RTOS2 任务入口，由 TaskManager 创建后调用。
   */
  [[noreturn]] static void Run(void *argument) noexcept;
};

} //  application::task

#endif
