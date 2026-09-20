#include "Application/Imu/GyroBiasCalibration.hpp"
#include <cmath>

namespace application {

namespace {
constexpr float ReferenceGravityMps2{9.80665F};
} // namespace

void GyroBiasCalibration::Init(const Config &config) noexcept {
  config_ = config;
  Reset();
}

void GyroBiasCalibration::Reset() noexcept {
  state_ = State::WaitingForTemperature;
  bias_ = alg_math::Vector3{};
  spread_ = alg_math::Vector3{};
  RestartWindow(0U);
  restarts_ = 0U; // 必须在 RestartWindow 之后清零，否则复位本身会被计成一次重启
  temperatureReadyTick_ = 0U;
  lastTick_ = 0U;
  temperatureCelsius_ = 0.0F;
  biasValid_ = false;
}

void GyroBiasCalibration::RestartWindow(std::uint32_t tick) noexcept {
  if (count_ != 0U) {
    ++restarts_; // 只统计"已经开始积累后被中断"的情况
  }
  sumX_ = 0.0;
  sumY_ = 0.0;
  sumZ_ = 0.0;
  minimum_ = alg_math::Vector3{};
  maximum_ = alg_math::Vector3{};
  count_ = 0U;
  windowStartTick_ = tick;
  lastTick_ = tick;
}

std::uint32_t
GyroBiasCalibration::TicksFromMs(std::uint32_t milliseconds) const noexcept {
  const std::uint64_t ticks = static_cast<std::uint64_t>(milliseconds) *
                              config_.tickFrequencyHz / 1000U;
  return ticks > 0xFFFFFFFFULL ? 0xFFFFFFFFU
                               : static_cast<std::uint32_t>(ticks);
}

void GyroBiasCalibration::Process(const alg_math::Vector3 &gyroRadPerSec,
                                  const alg_math::Vector3 &accelMetersPerSec2,
                                  float temperatureCelsius,
                                  std::uint32_t tick) noexcept {
  if (state_ == State::Completed || state_ == State::TimedOut) {
    return; // 终态不再改变：标定只做一次
  }

  lastTick_ = tick;
  temperatureCelsius_ = temperatureCelsius;

  // 温度门控：离开容差就清空窗口，并重新开始计时。
  const bool temperatureReady =
      std::isfinite(temperatureCelsius) &&
      std::fabs(temperatureCelsius - config_.targetCelsius) <=
          config_.temperatureToleranceCelsius;
  if (!temperatureReady) {
    if (state_ != State::WaitingForTemperature || count_ != 0U) {
      RestartWindow(tick);
    }
    state_ = State::WaitingForTemperature;
    temperatureReadyTick_ = tick;
    return;
  }
  if (state_ == State::WaitingForTemperature) {
    state_ = State::Sampling;
    temperatureReadyTick_ = tick;
    RestartWindow(tick);
  }

  // 时间上限：温度到位后仍无法完成则放弃，退化到未标定行为。
  if (static_cast<std::uint32_t>(tick - temperatureReadyTick_) >
      TicksFromMs(config_.timeoutMs)) {
    state_ = State::TimedOut;
    biasValid_ = false;
    return;
  }

  // 静止门控：加速度范数必须接近重力。
  const float accelNorm = alg_math::Norm(accelMetersPerSec2);
  if (!alg_math::IsFinite(accelMetersPerSec2) ||
      std::fabs(accelNorm - ReferenceGravityMps2) >
          config_.maximumAccelNormErrorMps2) {
    RestartWindow(tick);
    return;
  }

  Accumulate(gyroRadPerSec, tick);
}

void GyroBiasCalibration::Accumulate(const alg_math::Vector3 &gyroRadPerSec,
                                     std::uint32_t tick) noexcept {
  if (!alg_math::IsFinite(gyroRadPerSec)) {
    RestartWindow(tick);
    return;
  }

  if (count_ == 0U) {
    minimum_ = gyroRadPerSec;
    maximum_ = gyroRadPerSec;
  } else {
    if (gyroRadPerSec.x < minimum_.x) {
      minimum_.x = gyroRadPerSec.x;
    }
    if (gyroRadPerSec.y < minimum_.y) {
      minimum_.y = gyroRadPerSec.y;
    }
    if (gyroRadPerSec.z < minimum_.z) {
      minimum_.z = gyroRadPerSec.z;
    }
    if (gyroRadPerSec.x > maximum_.x) {
      maximum_.x = gyroRadPerSec.x;
    }
    if (gyroRadPerSec.y > maximum_.y) {
      maximum_.y = gyroRadPerSec.y;
    }
    if (gyroRadPerSec.z > maximum_.z) {
      maximum_.z = gyroRadPerSec.z;
    }
  }

  sumX_ += static_cast<double>(gyroRadPerSec.x);
  sumY_ += static_cast<double>(gyroRadPerSec.y);
  sumZ_ += static_cast<double>(gyroRadPerSec.z);
  ++count_;

  // 每个样本都检查极差：被碰或振动必须在下一个样本就重启窗口，
  // 否则短促扰动会被记进窗口，只能等到周期检查或收尾时才被剔除。
  const alg_math::Vector3 spread{maximum_.x - minimum_.x,
                                 maximum_.y - minimum_.y,
                                 maximum_.z - minimum_.z};
  if (spread.x > config_.maximumSpreadRadPerSec ||
      spread.y > config_.maximumSpreadRadPerSec ||
      spread.z > config_.maximumSpreadRadPerSec) {
    RestartWindow(tick);
    return;
  }

  // 周期性检查部分均值：抓缓慢转动，代价是与样本数无关的常数开销。
  if ((count_ % PartialCheckInterval) == 0U) {
    const double inverse = 1.0 / static_cast<double>(count_);
    const alg_math::Vector3 mean{static_cast<float>(sumX_ * inverse),
                                 static_cast<float>(sumY_ * inverse),
                                 static_cast<float>(sumZ_ * inverse)};
    if (std::fabs(mean.x) > config_.maximumMeanRateRadPerSec ||
        std::fabs(mean.y) > config_.maximumMeanRateRadPerSec ||
        std::fabs(mean.z) > config_.maximumMeanRateRadPerSec) {
      RestartWindow(tick);
      return;
    }
  }

  if (static_cast<std::uint32_t>(tick - windowStartTick_) >=
      TicksFromMs(config_.windowMs)) {
    Finalize();
  }
}

void GyroBiasCalibration::Finalize() noexcept {
  if (count_ == 0U) {
    RestartWindow(lastTick_);
    return;
  }

  const double inverse = 1.0 / static_cast<double>(count_);
  const alg_math::Vector3 mean{static_cast<float>(sumX_ * inverse),
                               static_cast<float>(sumY_ * inverse),
                               static_cast<float>(sumZ_ * inverse)};
  const alg_math::Vector3 spread{maximum_.x - minimum_.x,
                                 maximum_.y - minimum_.y,
                                 maximum_.z - minimum_.z};

  // 均值超限说明窗口内存在慢速转动，不能当作零偏，重新开始。
  if (!alg_math::IsFinite(mean) ||
      std::fabs(mean.x) > config_.maximumMeanRateRadPerSec ||
      std::fabs(mean.y) > config_.maximumMeanRateRadPerSec ||
      std::fabs(mean.z) > config_.maximumMeanRateRadPerSec ||
      spread.x > config_.maximumSpreadRadPerSec ||
      spread.y > config_.maximumSpreadRadPerSec ||
      spread.z > config_.maximumSpreadRadPerSec) {
    RestartWindow(lastTick_);
    return;
  }

  bias_ = mean;
  spread_ = spread;
  biasValid_ = true;
  state_ = State::Completed;
}

GyroBiasCalibration::Snapshot
GyroBiasCalibration::GetSnapshot() const noexcept {
  Snapshot snapshot{};
  snapshot.state = state_;
  snapshot.biasRadPerSec = bias_;
  snapshot.spreadRadPerSec = spread_;
  snapshot.samples = count_;
  snapshot.restarts = restarts_;
  snapshot.elapsedMs = static_cast<std::uint32_t>(
      static_cast<std::uint64_t>(
          static_cast<std::uint32_t>(lastTick_ - temperatureReadyTick_)) *
      1000U / config_.tickFrequencyHz);
  snapshot.temperatureCelsius = temperatureCelsius_;
  snapshot.biasValid = biasValid_;
  return snapshot;
}

} // namespace application
