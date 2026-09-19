#ifndef APPLICATION_TASK_CHASSIS_CONTROLLER_HPP
#define APPLICATION_TASK_CHASSIS_CONTROLLER_HPP

#include "Libraries/Algorithm/alg_controller/Pid.h"
#include "Libraries/Device/motor/Motor.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace application::chassis {

/**
 * @brief 底盘电机基础闭环控制器：对外统一输出轴语义（角度 deg / 转速输出轴
 *        rpm / 力矩 N·m），按模式选择控制结构——
 *        Torque：目标力矩开环直通（钳位后输出，不经过 PID）；
 *        Speed ：输出轴转速环（rpm → N·m）；
 *        Angle ：串级 角度环→转速环→力矩（角度环输出为目标输出轴转速）。
 * @note 纯策略对象：不访问硬件、不读取时间、不依赖 RTOS；dt 由调用方
 *       提供，可在宿主机单测闭环行为。目标值、使能位与模式为原子量，
 *       允许控制任务之外的单写者设定（遥控/决策任务）。
 * @note 模式切换（含切回原模式）即复位全部 PID 并清零目标，积分不跨模式
 *       泄漏；电机离线、控制器禁用或 dt 非法时输出恒 0 并复位。
 * @note 限幅链：角度环与转速目标钳 maxWheelSpeedRpm（输出轴 rpm）；转速环
 *       输出钳 speedLoop.Maxout（N·m）；最终力矩统一钳
 *       maxTorqueNewtonMeter（输出轴 N·m）。
 */
class ChassisController final {
public:
  static constexpr std::size_t MotorCount{4U};

  enum class WheelControlMode : std::uint8_t { Torque = 0U, Speed, Angle };

  struct Config final {
    alg_controller::PID::Config speedLoop; // 输出轴 rpm → N·m
    alg_controller::PID::Config angleLoop; // 输出轴 deg → rpm
    float maxTorqueNewtonMeter{0.0F};      // 输出轴力矩总限幅
    float maxWheelSpeedRpm{0.0F};          // 输出轴转速目标限幅
  };

  ChassisController() noexcept = default;

  /**
   * @brief 校验配置并初始化全部 PID。角度环 Maxout 会被 maxWheelSpeedRpm
   *        覆盖（外环输出限幅即内环目标上限）。
   * @return 配置全部有效时返回 true。
   */
  [[nodiscard]] bool Init(const Config &config) noexcept;

  /**
   * @brief 复位全部 PID 状态并清零目标，保留配置与模式。
   */
  void Reset() noexcept;

  void SetEnabled(bool enabled) noexcept;

  /**
   * @brief 切换控制模式（含切回原模式）：复位全部 PID 并清零目标。
   */
  void SetMode(WheelControlMode mode) noexcept;

  /**
   * @brief 设定单轮目标，语义随模式：Torque→N·m，Speed→输出轴 rpm，
   *        Angle→输出轴角度 deg。
   */
  void SetWheelTarget(std::size_t wheel, float value) noexcept;

  [[nodiscard]] bool IsReady() const noexcept;
  [[nodiscard]] bool IsEnabled() const noexcept;
  [[nodiscard]] WheelControlMode Mode() const noexcept;
  [[nodiscard]] float WheelTarget(std::size_t wheel) const noexcept;

  /**
   * @brief 推进一个控制周期，输出各电机输出轴力矩指令（N·m）。
   * @param snapshots 各电机遥测快照（下标即轮序，对应逻辑编号 0~3）。
   * @param dt 本周期时长（秒）。
   * @param outTorqueNewtonMeter 输出力矩指令，恒为有限值。
   */
  void Update(const device::MotorState (&snapshots)[MotorCount],
              float dt,
              float (&outTorqueNewtonMeter)[MotorCount]) noexcept;

private:
  bool ready_{false};
  std::atomic<bool> enabled_{false};
  std::atomic<WheelControlMode> mode_{WheelControlMode::Speed};
  std::array<std::atomic<float>, MotorCount> targets_{};
  std::array<alg_controller::PID, MotorCount> speedPids_{};
  std::array<alg_controller::PID, MotorCount> anglePids_{};
  Config config_{};

  static_assert(std::atomic<float>::is_always_lock_free,
                "ChassisController targets require lock-free float atomics");
  static_assert(std::atomic<WheelControlMode>::is_always_lock_free,
                "ChassisController mode requires lock-free enum atomics");
};

} //  application::chassis

#endif
