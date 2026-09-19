#include "Entry.h"
#include "Application/Task/4_communication/RemoteReceiver.hpp"
#include "Application/Task/TaskManager.hpp"
#include "Can.hpp"
#include "Gpio.hpp"
#include "Spi.hpp"
#include "Time.hpp"
#include "Uart.hpp"

bool Application_Init(void) {
  if (!platform::Time::IsReady()) {
    return false;
  }

  platform::Gpio::Level keyLevel = platform::Gpio::Level::Low;
  if (!platform::Gpio::Read(platform::Gpio::Pin::UserKey, keyLevel)) {
    return false;
  }

  platform::Gpio::Level Power5VLevel = platform::Gpio::Level::High;
  if (!platform::Gpio::Write(platform::Gpio::Pin::Power5V, Power5VLevel)) {
    return false;
  }

  if (!platform::Can::IsReady(platform::Can::Channel::Channel1) ||
      !platform::Can::IsReady(platform::Can::Channel::Channel2) ||
      !platform::Can::IsReady(platform::Can::Channel::Channel3)) {
    return false;
  }

  if (!platform::Spi::IsReady(platform::Spi::Device::ImuAccelerometer) ||
      !platform::Spi::IsReady(platform::Spi::Device::ImuGyroscope) ||
      !platform::Spi::IsReady(platform::Spi::Device::AddressableLed)) {
    return false;
  }

  if (!platform::Uart::IsReady(platform::Uart::Endpoint::RemoteReceiver) ||
      !platform::Uart::IsReady(platform::Uart::Endpoint::DebugConsole)) {
    return false;
  }

  if (!application::task::TaskManager::Init()) {
    return false;
  }
  return application::RemoteReceiver::Init();
}

void Application_RunOnce(void) {
  (void)application::task::TaskManager::Start();
  application::RemoteReceiver::Process();
}
