#ifndef PLATFORM_INTERFACE_UART_HPP
#define PLATFORM_INTERFACE_UART_HPP

#include "Detail/Uart.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace platform {

class Uart final {
public:
  static constexpr std::size_t MaximumTransmitSize{
      UART_PORT_MAX_TRANSMIT_SIZE};

  enum class Endpoint : std::uint8_t {
    RemoteReceiver = UART_PORT_ENDPOINT_REMOTE_RECEIVER,
    DebugConsole = UART_PORT_ENDPOINT_DEBUG_CONSOLE
  };

  enum class ReceiveResult : std::uint8_t {
    Received = UART_PORT_RECEIVE_RECEIVED,
    Empty = UART_PORT_RECEIVE_EMPTY,
    NotReady = UART_PORT_RECEIVE_NOT_READY,
    InvalidArgument = UART_PORT_RECEIVE_INVALID_ARGUMENT,
    Error = UART_PORT_RECEIVE_ERROR
  };

  enum class TransmitResult : std::uint8_t {
    Started = UART_PORT_TRANSMIT_STARTED,
    Busy = UART_PORT_TRANSMIT_BUSY,
    NotReady = UART_PORT_TRANSMIT_NOT_READY,
    NotSupported = UART_PORT_TRANSMIT_NOT_SUPPORTED,
    InvalidArgument = UART_PORT_TRANSMIT_INVALID_ARGUMENT,
    Error = UART_PORT_TRANSMIT_ERROR
  };

  // 在接收中断上下文调用，只用于触发轻量唤醒。
  using ReceiveNotification = UartPort_ReceiveNotification;
  // DMA 发送完成或异步失败时在中断上下文调用，只用于轻量唤醒。
  using TransmitNotification = UartPort_TransmitNotification;

  struct Statistics final {
    // DMA 已报告的总字节数，其中队列满时丢弃的部分另计。
    std::uint32_t receivedByteCount{0U};
    std::uint32_t droppedByteCount{0U};
    std::uint32_t errorEventCount{0U};
    std::uint32_t restartFailureCount{0U};
    // DMA 已完成发送的字节数；启动或异步发送失败另计。
    std::uint32_t transmittedByteCount{0U};
    std::uint32_t transmitFailureCount{0U};
  };

  Uart() = delete;

  [[nodiscard]] static bool IsReady(Endpoint endpoint) noexcept {
    return UartPort_IsReady(ToPortEndpoint(endpoint));
  }

  [[nodiscard]] static bool
  SetReceiveNotification(Endpoint endpoint, ReceiveNotification notification,
                         void *context) noexcept {
    return UartPort_SetReceiveNotification(ToPortEndpoint(endpoint),
                                           notification, context);
  }

  [[nodiscard]] static bool
  SetTransmitNotification(Endpoint endpoint,
                          TransmitNotification notification,
                          void *context) noexcept {
    return UartPort_SetTransmitNotification(ToPortEndpoint(endpoint),
                                            notification, context);
  }

  // 非阻塞读取；每个端点仅允许一个任务消费接收队列。
  [[nodiscard]] static ReceiveResult TryRead(Endpoint endpoint,
                                             std::uint8_t *data,
                                             std::size_t dataCapacity,
                                             std::size_t &length) noexcept {
    length = 0U;
    if (dataCapacity > std::numeric_limits<std::uint32_t>::max()) {
      return ReceiveResult::InvalidArgument;
    }

    std::uint32_t receivedLength = 0U;
    const ReceiveResult result = ToReceiveResult(UartPort_TryRead(
        ToPortEndpoint(endpoint), data,
        static_cast<std::uint32_t>(dataCapacity), &receivedLength));
    length = receivedLength;
    return result;
  }

  template <std::size_t Size>
  [[nodiscard]] static ReceiveResult
  TryRead(Endpoint endpoint, std::array<std::uint8_t, Size> &data,
          std::size_t &length) noexcept {
    return TryRead(endpoint, data.data(), data.size(), length);
  }

  // 仅供任务上下文调用。长度必须为 1..MaximumTransmitSize；数据会在
  // 返回前复制到 Port 自有缓冲，调用者可立即复用原缓冲。每个端点只
  // 允许一帧在途，Busy 时由调用者在收到发送通知后重试。
  [[nodiscard]] static TransmitResult TryWrite(Endpoint endpoint,
                                               const std::uint8_t *data,
                                               std::size_t length) noexcept {
    if (length > std::numeric_limits<std::uint32_t>::max()) {
      return TransmitResult::InvalidArgument;
    }

    return ToTransmitResult(UartPort_TryWrite(
        ToPortEndpoint(endpoint), data, static_cast<std::uint32_t>(length)));
  }

  template <std::size_t Size>
  [[nodiscard]] static TransmitResult
  TryWrite(Endpoint endpoint,
           const std::array<std::uint8_t, Size> &data) noexcept {
    return TryWrite(endpoint, data.data(), data.size());
  }

  [[nodiscard]] static bool GetStatistics(Endpoint endpoint,
                                          Statistics &statistics) noexcept {
    UartPort_Statistics portStatistics{};
    if (!UartPort_GetStatistics(ToPortEndpoint(endpoint), &portStatistics)) {
      return false;
    }

    statistics.receivedByteCount = portStatistics.received_byte_count;
    statistics.droppedByteCount = portStatistics.dropped_byte_count;
    statistics.errorEventCount = portStatistics.error_event_count;
    statistics.restartFailureCount = portStatistics.restart_failure_count;
    statistics.transmittedByteCount = portStatistics.transmitted_byte_count;
    statistics.transmitFailureCount = portStatistics.transmit_failure_count;
    return true;
  }

private:
  [[nodiscard]] static constexpr UartPort_Endpoint
  ToPortEndpoint(Endpoint endpoint) noexcept {
    return static_cast<UartPort_Endpoint>(endpoint);
  }

  [[nodiscard]] static constexpr ReceiveResult
  ToReceiveResult(UartPort_ReceiveResult result) noexcept {
    return result <= UART_PORT_RECEIVE_ERROR
               ? static_cast<ReceiveResult>(result)
               : ReceiveResult::Error;
  }

  [[nodiscard]] static constexpr TransmitResult
  ToTransmitResult(UartPort_TransmitResult result) noexcept {
    return result <= UART_PORT_TRANSMIT_ERROR
               ? static_cast<TransmitResult>(result)
               : TransmitResult::Error;
  }
};

} // namespace platform

#endif
