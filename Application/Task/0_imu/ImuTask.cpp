#include "Application/Task/0_imu/ImuTask.hpp"
#include "Application/Imu/AttitudeEstimator.hpp"
#include "Application/Imu/ImuDiagnostics.hpp"
#include "Application/Task/0_imu/ImuDataReadyMailbox.hpp"
#include "Libraries/Device/bmi088/Bmi088.hpp"
#include "Libraries/Device/bmi088/Bmi088Heater.hpp"
#include "Platform/Interface/Gpio.hpp"
#include "cmsis_os2.h"
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>

namespace application::task {

namespace {

constexpr std::uint32_t TaskPeriodTicks{1U};
constexpr std::uint32_t AccelerometerDataReadyFlag{1UL << 0U};
constexpr std::uint32_t GyroscopeDataReadyFlag{1UL << 1U};
constexpr std::uint32_t TransferCompletedFlag{1UL << 2U};
constexpr std::uint32_t DataReadyFlags{AccelerometerDataReadyFlag |
                                       GyroscopeDataReadyFlag |
                                       TransferCompletedFlag};

// 采集优先阶段的最大轮数：每轮只做边沿通知与传输衔接，不做慢路径工作。
constexpr std::uint32_t AcquisitionRounds{6U};

// 慢路径每轮处理的记录数上限：把一次任务占用时间限制在固定长度内，
// 使采集链路的启动延迟有上界，而不是等整队列样本全部处理完。
// 加速度计周期 625 us 比陀螺 500 us 更短，预算偏大时它的边沿会先被合并，
// 因此这里取较小值，让两路都能在每个周期内被服务。
constexpr std::uint32_t SampleBudgetPerIteration{4U};

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

} // namespace

bool ImuTask::Init() noexcept {
  bmi088.Init();
  if (!bmi088Heater.Init()) {
    initialized = false;
    return false;
  }

  // 姿态解算不是采集的前置条件：滤波器参数无效时保持未就绪，
  // 由 AttitudeEstimator 的统计暴露，不阻断采集与恒温。
  (void)AttitudeEstimator::Init();

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

    // 采集优先阶段：只要还有 DRDY 或传输完成事件挂起，就先把 SPI 流水线接上。
    // 本阶段只做常数时间的边沿通知与传输衔接，不执行诊断、解算等慢路径工作，
    // 因此陀螺 500 us 周期不会被任务自身的处理时间挤占。
    for (std::uint32_t round = 0U; round < AcquisitionRounds; ++round) {
      bmi088.NotifyDataReady(accelerometerInterrupts.Snapshot(),
                             gyroscopeInterrupts.Snapshot());
      bmi088.Process();
      if ((osThreadFlagsGet() & DataReadyFlags) == 0U) {
        break;
      }
      (void)osThreadFlagsClear(DataReadyFlags);
    }

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

    // 上电零偏标定的温度门控需要温度读数；温度变化缓慢，每轮取一次即可。
    device::Bmi088Temperature::Sample temperatureSample{};
    float temperatureCelsius = std::numeric_limits<float>::quiet_NaN();
    if (temperature.TryGetSample(temperatureSample)) {
      temperatureCelsius =
          device::Bmi088Temperature::ToCelsius(temperatureSample.raw);
    }

    // 慢路径：诊断与姿态解算，每轮最多固定条数，剩余样本留到下一轮，
    // 队列容量足以吸收，保证采集链路不会被长时间的批处理阻塞。
    device::Bmi088SampleRecord record{};
    bool gyroscope{false};
    for (std::uint32_t processed = 0U; processed < SampleBudgetPerIteration;
         ++processed) {
      if (!bmi088.PopNextSample(record, gyroscope)) {
        break;
      }
      application::ImuDiagnostics::Observe(bmi088, record, gyroscope);
      application::AttitudeEstimator::Process(record, gyroscope,
                                              temperatureCelsius);
    }
  }
}

} // namespace application::task
