#include "Application/Task/1_chassis/ChassisController.hpp"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
#include <cmath>

namespace application::chassis {

bool ChassisController::Init(const Config &config) noexcept {
  Reset();
  mode_.store(WheelControlMode::Speed, std::memory_order_relaxed);
  ready_ = false;

  if (!std::isfinite(config.maxTorqueNewtonMeter) ||
      !std::isfinite(config.maxWheelSpeedRpm) ||
      config.maxTorqueNewtonMeter <= 0.0F || config.maxWheelSpeedRpm <= 0.0F) {
    return false;
  }

  config_ = config;
  config_.angleLoop.Maxout = config.maxWheelSpeedRpm;

  for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel) {
    if (!speedPids_[wheel].Init(config_.speedLoop) ||
        !anglePids_[wheel].Init(config_.angleLoop)) {
      return false;
    }
  }

  ready_ = true;
  return true;
}

void ChassisController::Reset() noexcept {
  for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel) {
    speedPids_[wheel].Reset();
    anglePids_[wheel].Reset();
    targets_[wheel].store(0.0F, std::memory_order_relaxed);
  }
}

void ChassisController::SetEnabled(bool enabled) noexcept {
  enabled_.store(enabled, std::memory_order_release);
}

void ChassisController::SetMode(WheelControlMode mode) noexcept {
  Reset();
  mode_.store(mode, std::memory_order_release);
}

void ChassisController::SetWheelTarget(std::size_t wheel,
                                       float value) noexcept {
  if (wheel >= MotorCount) {
    return;
  }
  targets_[wheel].store(value, std::memory_order_relaxed);
}

bool ChassisController::IsReady() const noexcept { return ready_; }

bool ChassisController::IsEnabled() const noexcept {
  return enabled_.load(std::memory_order_acquire);
}

ChassisController::WheelControlMode ChassisController::Mode() const noexcept {
  return mode_.load(std::memory_order_acquire);
}

float ChassisController::WheelTarget(std::size_t wheel) const noexcept {
  if (wheel >= MotorCount) {
    return 0.0F;
  }
  return targets_[wheel].load(std::memory_order_relaxed);
}

void ChassisController::Update(
    const device::MotorState (&snapshots)[MotorCount], float dt,
    float (&outTorqueNewtonMeter)[MotorCount]) noexcept {
  const bool active = ready_ && enabled_.load(std::memory_order_acquire) &&
                      std::isfinite(dt) && dt > 0.0F && dt < 1.0F;
  const WheelControlMode mode = mode_.load(std::memory_order_acquire);
  const bool modeValid = mode == WheelControlMode::Torque ||
                         mode == WheelControlMode::Speed ||
                         mode == WheelControlMode::Angle;

  for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel) {
    outTorqueNewtonMeter[wheel] = 0.0F;
    const auto reset = [&]() noexcept {
      speedPids_[wheel].Reset();
      anglePids_[wheel].Reset();
    };
    const auto &state = snapshots[wheel];
    const float target = targets_[wheel].load(std::memory_order_relaxed);
    if (!active || !modeValid || !state.feedbackValid || !state.online ||
        !std::isfinite(state.angleDegrees) || !std::isfinite(state.speedRpm) ||
        !std::isfinite(target)) {
      reset();
      continue;
    }

    if (mode == WheelControlMode::Torque) {
      reset();
      outTorqueNewtonMeter[wheel] =
          alg_math::Limit(target, -config_.maxTorqueNewtonMeter,
                          config_.maxTorqueNewtonMeter);
      continue;
    }

    float targetRpm;
    if (mode == WheelControlMode::Speed) {
      anglePids_[wheel].Reset();
      targetRpm = alg_math::Limit(target, -config_.maxWheelSpeedRpm,
                                  config_.maxWheelSpeedRpm);
    } else {
      if (!anglePids_[wheel].CalculateLoop(state.angleDegrees, target, dt)) {
        reset();
        continue;
      }
      targetRpm = alg_math::Limit(anglePids_[wheel].GetOut(),
                                  -config_.maxWheelSpeedRpm,
                                  config_.maxWheelSpeedRpm);
    }

    if (!speedPids_[wheel].CalculateLoop(state.speedRpm, targetRpm, dt)) {
      reset();
      continue;
    }
    outTorqueNewtonMeter[wheel] =
        alg_math::Limit(speedPids_[wheel].GetOut(),
                        -config_.maxTorqueNewtonMeter,
                        config_.maxTorqueNewtonMeter);
  }
}

} //  application::chassis
