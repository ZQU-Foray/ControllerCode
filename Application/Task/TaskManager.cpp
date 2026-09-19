#include "Application/Task/TaskManager.hpp"
#include "Application/Task/0_imu/ImuTask.hpp"
#include "Application/Task/1_chassis/ChassisTask.hpp"
#include "Application/Task/4_communication/CommunicationTask.hpp"
#include "cmsis_os2.h"

namespace application::task {

namespace {

constexpr std::uint32_t ImuTaskStackSizeBytes{2048U};
constexpr std::uint32_t ChassisTaskStackSizeBytes{2048U};
constexpr std::uint32_t CommunicationTaskStackSizeBytes{2048U};

TaskManager::State state{TaskManager::State::Uninitialized};
osThreadId_t imuTaskHandle{nullptr};
osThreadId_t chassisTaskHandle{nullptr};
osThreadId_t communicationTaskHandle{nullptr};

const osThreadAttr_t imuTaskAttributes{"imuTask",
                                       0U,
                                       nullptr,
                                       0U,
                                       nullptr,
                                       ImuTaskStackSizeBytes,
                                       osPriorityAboveNormal,
                                       0U,
                                       0U};

const osThreadAttr_t chassisTaskAttributes{"chassisTask",
                                           0U,
                                           nullptr,
                                           0U,
                                           nullptr,
                                           ChassisTaskStackSizeBytes,
                                           osPriorityAboveNormal,
                                           0U,
                                           0U};

const osThreadAttr_t communicationTaskAttributes{"communicationTask",
                                                 0U,
                                                 nullptr,
                                                 0U,
                                                 nullptr,
                                                 CommunicationTaskStackSizeBytes,
                                                 osPriorityBelowNormal,
                                                 0U,
                                                 0U};

} // 

bool TaskManager::Init() noexcept {
  if (state == State::Initialized || state == State::Running) {
    return true;
  }
  if (state == State::Error || !ImuTask::Init() || !ChassisTask::Init() ||
      !CommunicationTask::Init()) {
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
  communicationTaskHandle = osThreadNew(CommunicationTask::Run, nullptr,
                                        &communicationTaskAttributes);
  if (imuTaskHandle == nullptr || chassisTaskHandle == nullptr ||
      communicationTaskHandle == nullptr) {
    state = State::Error;
    return false;
  }

  state = State::Running;
  return true;
}

TaskManager::State TaskManager::GetState() noexcept { return state; }

} //  application::task
