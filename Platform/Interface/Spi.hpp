#ifndef PLATFORM_INTERFACE_SPI_HPP
#define PLATFORM_INTERFACE_SPI_HPP

#include "Detail/Spi.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace platform {

class Spi final {
public:
  static constexpr std::size_t MaxAsyncLength = SPI_PORT_MAX_ASYNC_LENGTH;

  enum class Device : std::uint8_t {
    ImuAccelerometer = SPI_PORT_DEVICE_IMU_ACCELEROMETER,
    ImuGyroscope = SPI_PORT_DEVICE_IMU_GYROSCOPE,
    AddressableLed = SPI_PORT_DEVICE_ADDRESSABLE_LED
  };

  enum class Result : std::uint8_t {
    Completed = SPI_PORT_RESULT_COMPLETED,
    Busy = SPI_PORT_RESULT_BUSY,
    Timeout = SPI_PORT_RESULT_TIMEOUT,
    NotReady = SPI_PORT_RESULT_NOT_READY,
    Unsupported = SPI_PORT_RESULT_UNSUPPORTED,
    InvalidArgument = SPI_PORT_RESULT_INVALID_ARGUMENT,
    Error = SPI_PORT_RESULT_ERROR,
    Started = SPI_PORT_RESULT_STARTED
  };

  using CompletionNotification = SpiPort_CompletionNotification;

  Spi() = delete;

  [[nodiscard]] static bool IsReady(Device device) noexcept {
    return SpiPort_IsReady(ToPortDevice(device));
  }

  [[nodiscard]] static Result Transmit(Device device, const std::uint8_t *data,
                                       std::size_t length,
                                       std::uint32_t timeoutMs) noexcept {
    if (length > std::numeric_limits<std::uint32_t>::max()) {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_Transmit(ToPortDevice(device), data,
                                     static_cast<std::uint32_t>(length),
                                     timeoutMs));
  }

  template <std::size_t Size>
  [[nodiscard]] static Result
  Transmit(Device device, const std::array<std::uint8_t, Size> &data,
           std::uint32_t timeoutMs) noexcept {
    return Transmit(device, data.data(), data.size(), timeoutMs);
  }

  [[nodiscard]] static Result Transfer(Device device,
                                       const std::uint8_t *transmitData,
                                       std::uint8_t *receiveData,
                                       std::size_t length,
                                       std::uint32_t timeoutMs) noexcept {
    if (length > std::numeric_limits<std::uint32_t>::max()) {
      return Result::InvalidArgument;
    }

    return ToResult(
        SpiPort_Transfer(ToPortDevice(device), transmitData, receiveData,
                         static_cast<std::uint32_t>(length), timeoutMs));
  }

  template <std::size_t Size>
  [[nodiscard]] static Result
  Transfer(Device device, const std::array<std::uint8_t, Size> &transmitData,
           std::array<std::uint8_t, Size> &receiveData,
           std::uint32_t timeoutMs) noexcept {
    return Transfer(device, transmitData.data(), receiveData.data(),
                    transmitData.size(), timeoutMs);
  }

  [[nodiscard]] static Result
  StartTransmitAsync(Device device, const std::uint8_t *data,
                     std::size_t length, CompletionNotification notification,
                     void *context) noexcept {
    if (length > std::numeric_limits<std::uint32_t>::max()) {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_StartTransmitAsync(
        ToPortDevice(device), data, static_cast<std::uint32_t>(length),
        notification, context));
  }

  template <std::size_t Size>
  [[nodiscard]] static Result
  StartTransmitAsync(Device device, const std::array<std::uint8_t, Size> &data,
                     CompletionNotification notification,
                     void *context) noexcept {
    return StartTransmitAsync(device, data.data(), data.size(), notification,
                              context);
  }

  [[nodiscard]] static Result
  StartTransferAsync(Device device, const std::uint8_t *transmitData,
                     std::uint8_t *receiveData, std::size_t length,
                     CompletionNotification notification,
                     void *context) noexcept {
    if (length > std::numeric_limits<std::uint32_t>::max()) {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_StartTransferAsync(
        ToPortDevice(device), transmitData, receiveData,
        static_cast<std::uint32_t>(length), notification, context));
  }

  template <std::size_t Size>
  [[nodiscard]] static Result StartTransferAsync(
      Device device, const std::array<std::uint8_t, Size> &transmitData,
      std::array<std::uint8_t, Size> &receiveData,
      CompletionNotification notification, void *context) noexcept {
    return StartTransferAsync(device, transmitData.data(), receiveData.data(),
                              transmitData.size(), notification, context);
  }

  [[nodiscard]] static Result GetAsyncResult(Device device) noexcept {
    return ToResult(SpiPort_GetAsyncResult(ToPortDevice(device)));
  }

private:
  [[nodiscard]] static constexpr SpiPort_Device
  ToPortDevice(Device device) noexcept {
    return static_cast<SpiPort_Device>(device);
  }

  [[nodiscard]] static constexpr Result
  ToResult(SpiPort_Result result) noexcept {
    return result <= SPI_PORT_RESULT_STARTED ? static_cast<Result>(result)
                                             : Result::Error;
  }
};

} // namespace platform

#endif
