#include "Entry.h"
#include "Can.hpp"
#include "Gpio.hpp"
#include "Spi.hpp"
#include "Time.hpp"

bool Application_Init(void) {
  if (!platform::Time::IsReady()) {
    return false;
  }

  platform::Gpio::Level keyLevel = platform::Gpio::Level::Low;
  if (!platform::Gpio::Read(platform::Gpio::Pin::UserKey, keyLevel)) {
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

  return true;
}
