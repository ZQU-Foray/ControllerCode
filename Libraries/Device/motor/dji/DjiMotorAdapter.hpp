#ifndef LIBRARIES_DEVICE_MOTOR_DJI_DJI_MOTOR_ADAPTER_HPP
#define LIBRARIES_DEVICE_MOTOR_DJI_DJI_MOTOR_ADAPTER_HPP

#include "Libraries/Device/motor/Motor.hpp"
#include "Libraries/Device/motor/dji/DjiMotor.hpp"

namespace device {

// 一个适配器独占一路 CAN 会话；严禁同通道挂多个消费者。
class DjiMotorAdapter final {
public:
  struct Mapping final {
    std::uint8_t deviceId{0U};
    float gearRatio{1.0F};
    int direction{1}; // +1 或 -1；作用于角度、转速与指令力矩
  };
  struct Config final {
    platform::Can::Channel channel;
    DjiMotor::Profile profile;
    std::array<Mapping, DjiMotor::MaximumMotors> mappings{};
    std::size_t count{0U};
  };

  static constexpr std::size_t MaximumMotors{DjiMotor::MaximumMotors};

  explicit DjiMotorAdapter(const Config &config) noexcept;
  [[nodiscard]] bool Init() noexcept;
  [[nodiscard]] std::size_t Count() const noexcept { return config_.count; }
  void Process() noexcept;
  [[nodiscard]] bool ReadState(std::size_t id, MotorState &state) const noexcept;
  [[nodiscard]] bool SetTorque(std::size_t id, float value) noexcept;
  void ClearCommands() noexcept;

private:
  Config config_;
  DjiMotor bus_;
  bool ready_{false};
};

} //  device
#endif
