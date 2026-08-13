#ifndef PLATFORM_INTERFACE_GPIO_HPP
#define PLATFORM_INTERFACE_GPIO_HPP

#include "Detail/Gpio.h"
#include <cstdint>

namespace platform {

class Gpio final {
public:
  enum class Pin : std::uint8_t {
    Power24V1 = GPIO_PORT_PIN_POWER_24V_1,
    Power24V0 = GPIO_PORT_PIN_POWER_24V_0,
    Power5V = GPIO_PORT_PIN_POWER_5V,
    ImuAccelerometerChipSelect = GPIO_PORT_PIN_IMU_ACCELEROMETER_CHIP_SELECT,
    ImuGyroscopeChipSelect = GPIO_PORT_PIN_IMU_GYROSCOPE_CHIP_SELECT,
    ImuAccelerometerInterrupt = GPIO_PORT_PIN_IMU_ACCELEROMETER_INTERRUPT,
    ImuGyroscopeInterrupt = GPIO_PORT_PIN_IMU_GYROSCOPE_INTERRUPT,
    UserKey = GPIO_PORT_PIN_USER_KEY
  };

  enum class Level : std::uint8_t { Low = 0U, High = 1U };

  Gpio() = delete;

  [[nodiscard]] static bool Read(Pin pin, Level &level) noexcept {
    bool isHigh = false;
    if (!GpioPort_Read(ToPortPin(pin), &isHigh)) {
      return false;
    }

    level = isHigh ? Level::High : Level::Low;
    return true;
  }

  [[nodiscard]] static bool Write(Pin pin, Level level) noexcept {
    return GpioPort_Write(ToPortPin(pin), level == Level::High);
  }

  [[nodiscard]] static bool SetHigh(Pin pin) noexcept {
    return Write(pin, Level::High);
  }

  [[nodiscard]] static bool SetLow(Pin pin) noexcept {
    return Write(pin, Level::Low);
  }

  [[nodiscard]] static bool Toggle(Pin pin) noexcept {
    return GpioPort_Toggle(ToPortPin(pin));
  }

private:
  [[nodiscard]] static constexpr GpioPort_Pin ToPortPin(Pin pin) noexcept {
    return static_cast<GpioPort_Pin>(pin);
  }
};

} // namespace platform

#endif
