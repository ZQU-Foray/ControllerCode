#include "Application/Task/4_communication/CommunicationTask.hpp"
#include "Application/Imu/ImuStream.hpp"
#include "cmsis_os2.h"

namespace application::task {

namespace {
bool initialized{false};
} // namespace

bool CommunicationTask::Init() noexcept {
  if (initialized) {
    return true;
  }

  if (!application::ImuStream::Init()) {
    initialized = false;
    return false;
  }

  initialized = true;
  return true;
}

bool CommunicationTask::IsReady() noexcept { return initialized; }

[[noreturn]] void CommunicationTask::Run(void *argument) noexcept {
  (void)argument;
  if (!initialized) {
    osThreadExit();
  }

  for (;;) {
    osDelay(PeriodMs);
    // 先处理主机命令再发送，保证 'P' 能尽快关闭出口、'R' 开启后立即出数据。
    application::ImuStream::ProcessCommands();
    application::ImuStream::Drain();
  }
}

} // namespace application::task
