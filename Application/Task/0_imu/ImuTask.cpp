#include "Application/Imu/ImuDiagnostics.hpp"
#include "Application/Imu/ImuStream.hpp"
#include "Application/Task/0_imu/ImuDataReadyMailbox.hpp"
#include "Application/Task/0_imu/ImuTask.hpp"
#include "Libraries/Device/bmi088/Bmi088.hpp"
#include "Libraries/Device/bmi088/Bmi088Heater.hpp"
#include "Platform/Interface/Gpio.hpp"
#include "cmsis_os2.h"

#include <atomic>
#include <cstdint>
#include <cstring>

namespace application::task {

namespace {

constexpr std::uint32_t TaskPeriodTicks{1U};
constexpr std::uint32_t AccelerometerDataReadyFlag{1UL << 0U};
constexpr std::uint32_t GyroscopeDataReadyFlag{1UL << 1U};
constexpr std::uint32_t TransferCompletedFlag{1UL << 2U};
constexpr std::uint32_t DataReadyFlags{AccelerometerDataReadyFlag |
                                       GyroscopeDataReadyFlag |
                                       TransferCompletedFlag};

device::Bmi088 bmi088;
device::Bmi088Heater bmi088Heater;
ImuDataReadyMailbox accelerometerInterrupts, gyroscopeInterrupts;
bool initialized{false};
osThreadId_t taskHandle{nullptr};

#if defined(IMU_ENABLE_DIAGNOSTICS)
// 仅供调试器的暂存温度邮箱：先写 IEEE-754 位，再置
// command=1。默认/Release 行为仍是 50 C 服务约定。
std::atomic<std::uint32_t> bmi088HeaterDiagnosticTargetBits{0U};
std::atomic<std::uint32_t> bmi088HeaterDiagnosticCommand{0U};
struct HeaterDiagnosticReport {
  std::uint32_t version{1U}, requests{0U}, accepted{0U}, rejected{0U};
  float requestedCelsius{0.0F}, activeCelsius{50.0F};
  std::uint32_t lastResult{0U}; // 0 无，1 已接受，2 已拒绝
};
volatile HeaterDiagnosticReport bmi088HeaterDiagnosticReport{};
#endif

void AccelerometerDataReadyCallback(void *context) {
  (void)context;
  accelerometerInterrupts.Publish(platform::Time::NowTicks());
  if (taskHandle != nullptr) {
    (void)osThreadFlagsSet(taskHandle, AccelerometerDataReadyFlag);
  }
}

void GyroscopeDataReadyCallback(void *context) {
  (void)context;
  gyroscopeInterrupts.Publish(platform::Time::NowTicks());
  if (taskHandle != nullptr) {
    (void)osThreadFlagsSet(taskHandle, GyroscopeDataReadyFlag);
  }
}

void TransferCompletedCallback(void *context) {
  (void)context;
  if (taskHandle != nullptr)
    (void)osThreadFlagsSet(taskHandle, TransferCompletedFlag);
}

} // 

bool ImuTask::Init() noexcept {
  bmi088.Init();
  if (!bmi088Heater.Init()) {
    initialized = false;
    return false;
  }

  bmi088Heater.SetEnabled(true);
  taskHandle = nullptr;
  initialized = true;
  return true;
}

[[noreturn]] void ImuTask::Run(void *argument) noexcept {
  (void)argument;

  if (!initialized) {
    osThreadExit();
  }

  taskHandle = osThreadGetId();
  bmi088.SetCompletionNotification(TransferCompletedCallback, nullptr);
  if (!platform::Gpio::SetInterruptCallback(
          platform::Gpio::Pin::ImuAccelerometerInterrupt,
          AccelerometerDataReadyCallback) ||
      !platform::Gpio::SetInterruptCallback(
          platform::Gpio::Pin::ImuGyroscopeInterrupt,
          GyroscopeDataReadyCallback)) {
    bmi088Heater.SetEnabled(false);
    osThreadExit();
  }

  for (;;) {
    (void)osThreadFlagsWait(DataReadyFlags, osFlagsWaitAny, TaskPeriodTicks);

    bmi088.NotifyDataReady(accelerometerInterrupts.Snapshot(),
                           gyroscopeInterrupts.Snapshot());
    bmi088.Process();
    const auto &temperature = bmi088.GetTemperature();
#if defined(IMU_ENABLE_DIAGNOSTICS)
    if (bmi088HeaterDiagnosticCommand.exchange(0U, std::memory_order_acq_rel) ==
        1U) {
      const auto bits =
          bmi088HeaterDiagnosticTargetBits.load(std::memory_order_relaxed);
      float target{};
      std::memcpy(&target, &bits, sizeof(target));
      ++bmi088HeaterDiagnosticReport.requests;
      bmi088HeaterDiagnosticReport.requestedCelsius = target;
      if (bmi088Heater.SetTargetCelsius(target)) {
        ++bmi088HeaterDiagnosticReport.accepted;
        bmi088HeaterDiagnosticReport.activeCelsius = target;
        bmi088HeaterDiagnosticReport.lastResult = 1U;
      } else {
        ++bmi088HeaterDiagnosticReport.rejected;
        bmi088HeaterDiagnosticReport.lastResult = 2U;
      }
    }
#endif
    bmi088Heater.Process(temperature);

    device::Bmi088Temperature::Sample temperatureSample{};
    if (temperature.TryGetSample(temperatureSample)) {
      application::ImuStream::PublishTemperature(
          temperature.GetReadCount(), temperature.GetCompletionTick(),
          device::Bmi088Temperature::ToCelsius(temperatureSample.raw), true);
    }

    device::Bmi088SampleRecord record{};
    bool gyroscope{false};
    while (bmi088.PopNextSample(record, gyroscope)) {
      application::ImuDiagnostics::Observe(bmi088, record, gyroscope);
      application::ImuStream::Push(record, gyroscope);
    }
  }
}

} //  application::task
