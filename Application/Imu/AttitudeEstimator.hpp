#ifndef APPLICATION_IMU_ATTITUDE_ESTIMATOR_HPP
#define APPLICATION_IMU_ATTITUDE_ESTIMATOR_HPP

#include "Application/Imu/GyroBiasCalibration.hpp"
#include "Libraries/Device/bmi088/Bmi088SampleQueue.hpp"
#include <cstdint>

namespace application {

/**
 * @brief 姿态解算的数据通路与所有者。
 *
 * 职责边界：
 *   - 只做原始样本到 EKF 的搬运：单位与坐标映射由 ImuBodyMapping 完成；
 *   - 只做时序决策：按 PopNextSample 的时间顺序配对加速度计与陀螺样本，
 *     用 drdyTick 差分得到真实步长；
 *   - 只做零偏管理：上电后用静止窗口估计三轴零偏，其中 x/y 作为滤波器初值注入，
 *     z 轴（算法没有对应状态）在输入侧外部扣除；
 *   - 不做任何滤波、平滑、外插或坏样本修补，坏时序直接丢弃并计数。
 *
 * 时序策略（单一所有者，仅允许 IMU 任务调用）：
 *   - 陀螺样本是节拍源：每个通过检查的陀螺样本都产生一次预测；
 *   -
 * 加速度样本先缓存，在下一次陀螺节拍被消费；没有新的加速度样本时沿用上一条，
 *     但超过保持上限就只做预测，不虚构观测；
 *   - 首个陀螺样本只用于建立时间基准；陀螺间隔超过上限视为断流，重新建立基准而
 *     不积分缺失段；时间戳标志不确定、序号重复或倒序的样本一律丢弃。
 *
 * 输出与统计为固定布局的内存存档，可由调试器直接读取；本模块没有对外通信路径。
 */
class AttitudeEstimator final {
public:
  /** @brief 结果存档版本，结构变化时递增。 */
  static constexpr std::uint32_t OutputVersion{2U};

  /**
   * @brief 对外状态存档，布局固定，供调试器读取。
   */
  struct Output final {
    std::uint32_t version{OutputVersion};
    std::uint32_t updateTick{0U};                // 最近一次更新的 drdyTick
    float quaternion[4]{1.0F, 0.0F, 0.0F, 0.0F}; // 机体系到世界系，标量在前
    float eulerDegrees[3]{};                     // ZYX：yaw、pitch、roll
    float gyroBiasRadPerSec[3]{}; // 滤波器零偏估计（x/y 有效，z 恒为零）
    float yawTotalDegrees{0.0F};  // 跨 ±180° 累计的连续 yaw
    float chiSquare{0.0F};
    std::uint32_t updateCount{0U};
    std::uint32_t divergenceCount{0U};
    std::uint32_t valid{0U};
    std::uint32_t converged{0U};
    std::uint32_t stable{0U};
    std::uint32_t reserved{0U};
    // 版本 2：上电静止标定结果与状态
    float calibrationBiasRadPerSec[3]{};   // 静止窗口估计的三轴零偏
    float calibrationSpreadRadPerSec[3]{}; // 窗口内每轴极差
    std::uint32_t calibrationState{0U};    // 见 GyroBiasCalibration::State
    std::uint32_t calibrationSamples{0U};  // 当前窗口样本数
    float calibrationTemperatureCelsius{0.0F};
    std::uint32_t calibrationElapsedMs{0U}; // 温度到位后经过的时间
  };
  static_assert(sizeof(Output) == 120U, "调试器存档布局必须稳定");

  /**
   * @brief 时序与质量统计，用于判断估计结果是否可信。
   */
  struct Statistics final {
    std::uint32_t gyroReceived{0U};
    std::uint32_t accelReceived{0U};
    std::uint32_t gyroConsumed{0U};
    std::uint32_t accelConsumed{0U};
    std::uint32_t rejectedFlags{0U};
    std::uint32_t rejectedSequence{0U};
    std::uint32_t gyroGaps{0U};
    std::uint32_t accelStale{0U};
    std::uint32_t accelHoldReuse{0U};
    std::uint32_t predictOnly{0U};
    std::uint32_t pendingOverwritten{0U};
    std::uint32_t calibrationRestarts{0U};
  };
  static_assert(sizeof(Statistics) == 48U, "统计布局必须稳定");

  /** @brief 陀螺间隔上限（秒），超过视为断流并重新建立时间基准。 */
  static constexpr float GyroGapLimitSeconds{0.01F};

  /** @brief 加速度样本保持上限（秒），超过则只做预测。 */
  static constexpr float AccelerometerHoldLimitSeconds{0.005F};

  AttitudeEstimator() = delete;

  /**
   * @brief 初始化滤波器与上电零偏标定。必须在 IMU 任务开始消费样本前调用。
   * @return 滤波器参数有效时返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  /** @brief 查询是否已完成初始化。 */
  [[nodiscard]] static bool IsReady() noexcept;

  /**
   * @brief 按时间顺序喂入一条原始样本。常数时间、不阻塞。
   * @param record 驱动收割到的原始样本。
   * @param gyroscope true 表示陀螺仪样本，false 表示加速度计样本。
   * @param temperatureCelsius 最近一次温度读数，用于上电零偏标定的温度门控；
   *                           无有效读数时传 NaN。
   */
  static void Process(const device::Bmi088SampleRecord &record, bool gyroscope,
                      float temperatureCelsius) noexcept;

  /** @brief 读取结果存档。 */
  [[nodiscard]] static Output GetOutput() noexcept;

  /** @brief 读取时序与质量统计。 */
  [[nodiscard]] static Statistics GetStatistics() noexcept;
};

} // namespace application

#endif
