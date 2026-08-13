#ifndef PLATFORM_INTERFACE_CAN_HPP
#define PLATFORM_INTERFACE_CAN_HPP

#include "Detail/Can.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace platform {

class Can final {
public:
  static constexpr std::size_t MaxDataLength = CAN_PORT_MAX_DATA_LENGTH;

  enum class Channel : std::uint8_t {
    Channel1 = CAN_PORT_CHANNEL_1,
    Channel2 = CAN_PORT_CHANNEL_2,
    Channel3 = CAN_PORT_CHANNEL_3
  };

  enum class IdentifierType : std::uint8_t {
    Standard = CAN_PORT_IDENTIFIER_STANDARD,
    Extended = CAN_PORT_IDENTIFIER_EXTENDED
  };

  enum class SendResult : std::uint8_t {
    Queued = CAN_PORT_SEND_QUEUED,
    QueueFull = CAN_PORT_SEND_QUEUE_FULL,
    NotReady = CAN_PORT_SEND_NOT_READY,
    BusOff = CAN_PORT_SEND_BUS_OFF,
    InvalidArgument = CAN_PORT_SEND_INVALID_ARGUMENT,
    Error = CAN_PORT_SEND_ERROR
  };

  enum class ReceiveResult : std::uint8_t {
    Received = CAN_PORT_RECEIVE_RECEIVED,
    Empty = CAN_PORT_RECEIVE_EMPTY,
    NotReady = CAN_PORT_RECEIVE_NOT_READY,
    BusOff = CAN_PORT_RECEIVE_BUS_OFF,
    InvalidArgument = CAN_PORT_RECEIVE_INVALID_ARGUMENT,
    Error = CAN_PORT_RECEIVE_ERROR
  };

  using ReceiveNotification = CanPort_ReceiveNotification;

  struct Frame final {
    std::uint32_t identifier{0U};
    IdentifierType identifierType{IdentifierType::Standard};
    std::uint8_t length{0U};
    std::array<std::uint8_t, MaxDataLength> data{};
  };

  struct Statistics final {
    std::uint32_t rxDroppedCount{0U};
    std::uint32_t rxHardwareLossEventCount{0U};
    std::uint32_t busOffCount{0U};
  };

  Can() = delete;

  [[nodiscard]] static bool IsReady(Channel channel) noexcept {
    return CanPort_IsReady(ToPortChannel(channel));
  }

  [[nodiscard]] static bool
  ConfigureStandardReceiveFilter(Channel channel, bool enabled,
                                 std::uint32_t firstIdentifier,
                                 std::uint32_t lastIdentifier) noexcept {
    return CanPort_ConfigureStandardReceiveFilter(
        ToPortChannel(channel), enabled, firstIdentifier, lastIdentifier);
  }

  [[nodiscard]] static bool
  SetReceiveNotification(Channel channel, ReceiveNotification notification,
                         void *context) noexcept {
    return CanPort_SetReceiveNotification(ToPortChannel(channel), notification,
                                          context);
  }

  [[nodiscard]] static SendResult TrySend(Channel channel,
                                          const Frame &frame) noexcept {
    return ToSendResult(CanPort_TrySend(
        ToPortChannel(channel), frame.identifier,
        static_cast<CanPort_IdentifierType>(frame.identifierType), frame.length,
        frame.data.data()));
  }

  [[nodiscard]] static ReceiveResult TryReceive(Channel channel,
                                                Frame &frame) noexcept {
    CanPort_IdentifierType identifierType = CAN_PORT_IDENTIFIER_STANDARD;
    std::uint32_t identifier = 0U;
    std::uint8_t length = 0U;

    const ReceiveResult result = ToReceiveResult(CanPort_TryReceive(
        ToPortChannel(channel), &identifier, &identifierType, &length,
        frame.data.data(), static_cast<std::uint8_t>(frame.data.size())));

    if (result == ReceiveResult::Received) {
      frame.identifier = identifier;
      frame.identifierType = static_cast<IdentifierType>(identifierType);
      frame.length = length;
    }

    return result;
  }

  [[nodiscard]] static bool GetStatistics(Channel channel,
                                          Statistics &statistics) noexcept {
    CanPort_Statistics portStatistics{};
    if (!CanPort_GetStatistics(ToPortChannel(channel), &portStatistics)) {
      return false;
    }

    statistics.rxDroppedCount = portStatistics.rx_dropped_count;
    statistics.rxHardwareLossEventCount =
        portStatistics.rx_hardware_loss_event_count;
    statistics.busOffCount = portStatistics.bus_off_count;
    return true;
  }

private:
  [[nodiscard]] static constexpr CanPort_Channel
  ToPortChannel(Channel channel) noexcept {
    return static_cast<CanPort_Channel>(channel);
  }

  [[nodiscard]] static constexpr SendResult
  ToSendResult(CanPort_SendResult result) noexcept {
    return result <= CAN_PORT_SEND_ERROR ? static_cast<SendResult>(result)
                                         : SendResult::Error;
  }

  [[nodiscard]] static constexpr ReceiveResult
  ToReceiveResult(CanPort_ReceiveResult result) noexcept {
    return result <= CAN_PORT_RECEIVE_ERROR ? static_cast<ReceiveResult>(result)
                                            : ReceiveResult::Error;
  }
};

} // namespace platform

#endif
