#include "Libraries/Device/bmi088/Bmi088Heater.hpp"

#include <cmath>

namespace device {

namespace {

constexpr float PidKp{10.0F};
constexpr float PidKi{1.0F};
constexpr float PidKd{0.0F};
constexpr float PidMaximumOutput{30.0F};
constexpr float PidIntegralLimit{50.0F};
constexpr float DefaultControlPeriodSeconds{0.128F};
constexpr float PreheatThresholdFor(float targetCelsius) {
  return targetCelsius - 2.0F < Bmi088Heater::PreheatThresholdCelsius
             ? targetCelsius - 2.0F
             : Bmi088Heater::PreheatThresholdCelsius;
}
static_assert(PreheatThresholdFor(Bmi088Heater::DefaultTargetCelsius) ==
              Bmi088Heater::PreheatThresholdCelsius);
static_assert(PreheatThresholdFor(32.0F) == 30.0F);

} // 

bool Bmi088Heater::Init() noexcept {
  alg_controller::PID::Config config{};
  config.Mode = alg_controller::PIDMode::Position;
  config.Kp = PidKp;
  config.Ki = PidKi;
  config.Kd = PidKd;
  config.DefaultDt = DefaultControlPeriodSeconds;
  config.Maxout = PidMaximumOutput;
  config.IntegralLimit = PidIntegralLimit;

  initialized_ = pid_.Init(config) &&
                 platform::Pwm::IsReady(platform::Pwm::Channel::ImuHeater);
  enabled_ = false;
  sampleSeen_ = false;
  state_ = initialized_ ? State::Disabled : State::Fault;
  targetCelsius_ = DefaultTargetCelsius;
  temperatureCelsius_ = 0.0F;
  dutyPermille_ = 0U;
  lastTemperatureReadCount_ = 0U;
  lastSampleTick_ = platform::Time::NowTicks();
  lastControlTick_ = lastSampleTick_;
  lastPwmResult_ = platform::Pwm::Silence(platform::Pwm::Channel::ImuHeater);
  if (lastPwmResult_ != platform::Pwm::Result::Completed) {
    initialized_ = false;
    state_ = State::Fault;
  }
  return initialized_;
}

void Bmi088Heater::SetEnabled(bool enabled) noexcept {
  if (!initialized_) {
    DisableOutput(State::Fault);
    return;
  }

  enabled_ = enabled;
  pid_.Reset();
  lastControlTick_ = platform::Time::NowTicks();
  if (!enabled) {
    DisableOutput(State::Disabled);
    return;
  }

  DisableOutput(State::WaitingForTemperature);
}

bool Bmi088Heater::SetTargetCelsius(float targetCelsius) noexcept {
  if (!std::isfinite(targetCelsius) || targetCelsius < MinimumTargetCelsius ||
      targetCelsius > MaximumTargetCelsius) {
    return false;
  }

  targetCelsius_ = targetCelsius;
  return true;
}

void Bmi088Heater::Process(const Bmi088Temperature &temperature) noexcept {
  if (!initialized_ || !enabled_) {
    return;
  }

  if (temperature.GetState() == Bmi088Temperature::State::Error) {
    DisableOutput(State::Fault);
    pid_.Reset();
    return;
  }

  Bmi088Temperature::Sample sample{};
  const std::uint32_t readCount = temperature.GetReadCount();
  if (readCount != lastTemperatureReadCount_ &&
      temperature.TryGetSample(sample)) {
    lastTemperatureReadCount_ = readCount;
    temperatureCelsius_ = Bmi088Temperature::ToCelsius(sample.raw);
    lastSampleTick_ = platform::Time::NowTicks();
    sampleSeen_ = true;
  }

  if (!sampleSeen_) {
    DisableOutput(State::WaitingForTemperature);
    return;
  }

  if (!std::isfinite(temperatureCelsius_) ||
      temperatureCelsius_ > MaximumSafeCelsius ||
      platform::Time::HasElapsedMs(lastSampleTick_, SampleTimeoutMs)) {
    DisableOutput(State::Fault);
    pid_.Reset();
    return;
  }

  if (!platform::Time::HasElapsedMs(lastControlTick_, ControlPeriodMs)) {
    return;
  }

  const platform::Time::Tick now = platform::Time::NowTicks();
  const float deltaSeconds =
      platform::Time::TicksToSeconds(now - lastControlTick_);
  lastControlTick_ = now;

  // 默认 50 C 目标沿用原有 45 C 快速预热边界。
  // 更低的诊断设定值不得被强行推过其目标。
  const float preheatThreshold = PreheatThresholdFor(targetCelsius_);
  if (temperatureCelsius_ < preheatThreshold) {
    pid_.Reset();
    if (SetDuty(PreheatDutyPermille)) {
      state_ = State::Preheating;
    }
    return;
  }

  if (!pid_.CalculateLoop(temperatureCelsius_, targetCelsius_, deltaSeconds)) {
    DisableOutput(State::Fault);
    pid_.Reset();
    return;
  }

  float output = pid_.GetOut();
  if (!std::isfinite(output) || output < 0.0F) {
    output = 0.0F;
  }
  if (output > PidMaximumOutput) {
    output = PidMaximumOutput;
  }

  if (SetDuty(static_cast<std::uint16_t>(output + 0.5F))) {
    state_ = State::Regulating;
  }
}

void Bmi088Heater::DisableOutput(State nextState) noexcept {
  lastPwmResult_ = platform::Pwm::Silence(platform::Pwm::Channel::ImuHeater);
  dutyPermille_ = 0U;
  state_ = lastPwmResult_ == platform::Pwm::Result::Completed ? nextState
                                                              : State::Fault;
}

bool Bmi088Heater::SetDuty(std::uint16_t dutyPermille) noexcept {
  lastPwmResult_ = platform::Pwm::Set(platform::Pwm::Channel::ImuHeater,
                                      PwmFrequencyHz, dutyPermille);
  if (lastPwmResult_ != platform::Pwm::Result::Completed) {
    dutyPermille_ = 0U;
    state_ = State::Fault;
    (void)platform::Pwm::Silence(platform::Pwm::Channel::ImuHeater);
    return false;
  }

  dutyPermille_ = dutyPermille;
  return true;
}

} //  device
