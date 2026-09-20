#ifndef LIBRARIES_DEVICE_BMI088_BMI088_HEATER_HPP
#define LIBRARIES_DEVICE_BMI088_BMI088_HEATER_HPP

#include "Libraries/Algorithm/alg_controller/Pid.h"
#include "Libraries/Device/bmi088/Bmi088Temperature.hpp"
#include "Platform/Interface/Pwm.hpp"
#include "Platform/Interface/Time.hpp"
#include <cstdint>

namespace device {

/**
 * @brief BMI088 恒温控制器，以新鲜温度样本驱动板载加热 PWM。
 * @note 控制参数由原工程 10000 计数量纲等比例换算为 PWM 千分比；
 *       温度无效、过温或样本超时时输出立即归零。
 */
class Bmi088Heater final {
public:
  static constexpr float DefaultTargetCelsius{50.0F};
  static constexpr float MinimumTargetCelsius{20.0F};
  static constexpr float MaximumTargetCelsius{60.0F};
  static constexpr float PreheatThresholdCelsius{45.0F};
  static constexpr float MaximumSafeCelsius{65.0F};
  static constexpr std::uint32_t ControlPeriodMs{128U};
  static constexpr std::uint32_t SampleTimeoutMs{500U};
  static constexpr std::uint32_t PwmFrequencyHz{100U};

  enum class State : std::uint8_t {
    Disabled,
    WaitingForTemperature,
    Preheating,
    Regulating,
    Fault
  };

  Bmi088Heater() noexcept = default;

  /**
   * @brief 初始化 PID 并确保加热输出为零。
   * @return PID 与 PWM 通道均就绪且成功关闭输出时返回 true。
   */
  [[nodiscard]] bool Init() noexcept;

  /**
   * @brief 启用或关闭恒温控制；关闭时清空 PID 并立即关闭 PWM。
   */
  void SetEnabled(bool enabled) noexcept;

  /**
   * @brief 设置目标温度。
   * @return 温度有限且处于 20 至 60 摄氏度范围内时返回 true。
   */
  [[nodiscard]] bool SetTargetCelsius(float targetCelsius) noexcept;

  /**
   * @brief 消费 BMI088 温度子设备的最新样本并推进恒温控制。
   * @note 可在主任务每轮调用，内部最多每 128 ms 更新一次控制量。
   */
  void Process(const Bmi088Temperature &temperature) noexcept;

  [[nodiscard]] bool IsEnabled() const noexcept { return enabled_; }
  [[nodiscard]] State GetState() const noexcept { return state_; }
  [[nodiscard]] float GetTargetCelsius() const noexcept {
    return targetCelsius_;
  }
  [[nodiscard]] float GetTemperatureCelsius() const noexcept {
    return temperatureCelsius_;
  }
  [[nodiscard]] std::uint16_t GetDutyPermille() const noexcept {
    return dutyPermille_;
  }
  [[nodiscard]] platform::Pwm::Result GetLastPwmResult() const noexcept {
    return lastPwmResult_;
  }

private:
  // 预热占空比必须是满量程：实测原值 100‰（10%）的稳态只能到 42.4 ℃
  // 并且仍在下降， 无法越过 45 ℃ 预热门槛，控制器会永久停在
  // Preheating、永远进不了 50 ℃ 服务点。
  static constexpr std::uint16_t PreheatDutyPermille{1000U};

  void DisableOutput(State nextState) noexcept;
  [[nodiscard]] bool SetDuty(std::uint16_t dutyPermille) noexcept;

  alg_controller::PID pid_{};
  State state_{State::Disabled};
  bool initialized_{false};
  bool enabled_{false};
  bool sampleSeen_{false};
  float targetCelsius_{DefaultTargetCelsius};
  float temperatureCelsius_{0.0F};
  std::uint16_t dutyPermille_{0U};
  std::uint32_t lastTemperatureReadCount_{0U};
  platform::Time::Tick lastSampleTick_{0U};
  platform::Time::Tick lastControlTick_{0U};
  platform::Pwm::Result lastPwmResult_{platform::Pwm::Result::NotReady};
};

} // namespace device

#endif
