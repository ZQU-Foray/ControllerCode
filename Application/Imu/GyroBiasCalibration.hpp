#ifndef APPLICATION_IMU_GYRO_BIAS_CALIBRATION_HPP
#define APPLICATION_IMU_GYRO_BIAS_CALIBRATION_HPP

#include "Libraries/Algorithm/alg_math/Vector3.h"
#include <cstdint>

namespace application {

/**
 * @brief 上电静止零偏标定：等恒温到位后，用一段静止窗口估计三轴陀螺零偏。
 *
 * 设计要点：
 *   - 温度门控：零偏随温度变化，因此必须等恒温目标到位且保持在容差内才开窗；
 *     温度离开容差会清空当前窗口重新计时。
 *   - 静止门控：加速度范数必须接近重力；窗口内任一轴的极差超过上限（被碰、
 *     振动）或部分均值超过上限（缓慢转动）都判定为运动，窗口重新开始。
 *   - 时间上限：温度到位后超过上限仍未通过检验，则放弃并置为超时状态，
 *     此时调用方保持零偏不修正（退化到未标定行为），不阻塞业务。
 *   - 纯函数式外部接口：本类不依赖任何平台设施，tick 频率由 Config 给出，
 *     因此可以在主机上确定性测试；tick 差值一律按无符号回绕计算。
 *
 * 本类只做"估计"，不修改任何输入，也不负责把零偏作用到样本上。
 */
class GyroBiasCalibration final {
public:
  /** @brief 标定状态，取值写入调试器存档，顺序不可随意调整。 */
  enum class State : std::uint32_t {
    WaitingForTemperature = 0U, // 恒温未到位
    Sampling = 1U,              // 正在积累静止窗口
    Completed = 2U,             // 已完成，零偏有效
    TimedOut = 3U               // 超时放弃，零偏无效
  };

  struct Config final {
    std::uint32_t tickFrequencyHz{480000000U}; // 与 platform::Time 一致
    float targetCelsius{50.0F};                // 与恒温目标一致
    float temperatureToleranceCelsius{0.5F};
    float maximumMeanRateRadPerSec{0.05F}; // 窗口均值上限，约 2.9 °/s
    float maximumSpreadRadPerSec{1.0F};    // 窗口极差上限，约 57 °/s
    float maximumAccelNormErrorMps2{0.5F}; // |a| 与重力的容差
    std::uint32_t windowMs{5000U};         // 静止窗口长度
    std::uint32_t timeoutMs{20000U};       // 温度到位后的总时限
  };

  struct Snapshot final {
    State state{State::WaitingForTemperature};
    alg_math::Vector3 biasRadPerSec{};   // 标定结果
    alg_math::Vector3 spreadRadPerSec{}; // 窗口内每轴极差
    std::uint32_t samples{0U};           // 当前窗口已积累样本数
    std::uint32_t restarts{0U};          // 窗口因运动/温度被重启的次数
    std::uint32_t elapsedMs{0U};         // 温度到位后经过的时间
    float temperatureCelsius{0.0F};
    bool biasValid{false};
  };

  GyroBiasCalibration() noexcept = default;

  /** @brief 设置参数并复位状态。 */
  void Init(const Config &config) noexcept;

  /** @brief 复位到等待温度状态，保留参数。 */
  void Reset() noexcept;

  /**
   * @brief 每个陀螺样本调用一次。
   * @param gyroRadPerSec 机体系角速度，未经任何零偏修正。
   * @param accelMetersPerSec2 最近一次机体系加速度，用于静止门控。
   * @param temperatureCelsius 最近一次温度读数；无有效读数时传 NaN。
   * @param tick 采样时刻的高精度计数（允许回绕）。
   */
  void Process(const alg_math::Vector3 &gyroRadPerSec,
               const alg_math::Vector3 &accelMetersPerSec2,
               float temperatureCelsius, std::uint32_t tick) noexcept;

  [[nodiscard]] Snapshot GetSnapshot() const noexcept;

  /** @brief 标定完成且结果通过检验时返回 true。 */
  [[nodiscard]] bool IsBiasValid() const noexcept { return biasValid_; }

  [[nodiscard]] const alg_math::Vector3 &GetBiasRadPerSec() const noexcept {
    return bias_;
  }

private:
  void RestartWindow(std::uint32_t tick) noexcept;
  void Accumulate(const alg_math::Vector3 &gyroRadPerSec,
                  std::uint32_t tick) noexcept;
  void Finalize() noexcept;
  [[nodiscard]] std::uint32_t
  TicksFromMs(std::uint32_t milliseconds) const noexcept;

  static constexpr std::uint32_t PartialCheckInterval{512U};

  Config config_{};
  State state_{State::WaitingForTemperature};
  alg_math::Vector3 bias_{};
  alg_math::Vector3 spread_{};
  double sumX_{0.0};
  double sumY_{0.0};
  double sumZ_{0.0};
  alg_math::Vector3 minimum_{};
  alg_math::Vector3 maximum_{};
  std::uint32_t count_{0U};
  std::uint32_t windowStartTick_{0U};
  std::uint32_t lastTick_{0U};
  std::uint32_t temperatureReadyTick_{0U};
  std::uint32_t restarts_{0U};
  float temperatureCelsius_{0.0F};
  bool biasValid_{false};
};

} // namespace application

#endif
