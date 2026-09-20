#include "Application/Task/TaskManager.hpp"
#include "Application/Task/0_imu/ImuTask.hpp"
#include "Application/Task/1_chassis/ChassisTask.hpp"
#include "cmsis_os2.h"

namespace application::task {

namespace {

constexpr std::uint32_t ImuTaskStackSizeBytes{2048U};
constexpr std::uint32_t ChassisTaskStackSizeBytes{2048U};

// IMU 任务必须严格高于底盘任务：2000 Hz 陀螺的 DRDY 到 SPI 传输启动的延迟
// 直接决定 DRDY 边沿是否被合并。同优先级会被时间片轮转推迟，实测平均启动
// 延迟 261 us（占 500 us 周期的 52%），并造成约 2.2% 的陀螺样本丢失。
constexpr osPriority_t ImuTaskPriority{osPriorityHigh};

TaskManager::State state{TaskManager::State::Uninitialized};
osThreadId_t imuTaskHandle{nullptr};
osThreadId_t chassisTaskHandle{nullptr};

const osThreadAttr_t imuTaskAttributes{
    "imuTask",       0U, nullptr, 0U, nullptr, ImuTaskStackSizeBytes,
    ImuTaskPriority, 0U, 0U};

const osThreadAttr_t chassisTaskAttributes{
    "chassisTask",         0U, nullptr, 0U, nullptr, ChassisTaskStackSizeBytes,
    osPriorityAboveNormal, 0U, 0U};

} // namespace

bool TaskManager::Init() noexcept {
  if (state == State::Initialized || state == State::Running) {
    return true;
  }
  if (state == State::Error || !ImuTask::Init() || !ChassisTask::Init()) {
    state = State::Error;
    return false;
  }

  state = State::Initialized;
  return true;
}

bool TaskManager::Start() noexcept {
  if (state == State::Running) {
    return true;
  }
  if (state != State::Initialized || osKernelGetState() != osKernelRunning) {
    return false;
  }

  imuTaskHandle = osThreadNew(ImuTask::Run, nullptr, &imuTaskAttributes);
  chassisTaskHandle =
      osThreadNew(ChassisTask::Run, nullptr, &chassisTaskAttributes);
  if (imuTaskHandle == nullptr || chassisTaskHandle == nullptr) {
    state = State::Error;
    return false;
  }

  state = State::Running;
  return true;
}

TaskManager::State TaskManager::GetState() noexcept { return state; }

} // namespace application::task
