#ifndef PLATFORM_INTERFACE_DETAIL_SPI_H
#define PLATFORM_INTERFACE_DETAIL_SPI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t SpiPort_Device;
typedef uint8_t SpiPort_Result;
typedef void (*SpiPort_CompletionNotification)(void *context);

enum {
  SPI_PORT_DEVICE_IMU_ACCELEROMETER = 0U,
  SPI_PORT_DEVICE_IMU_GYROSCOPE,
  SPI_PORT_DEVICE_ADDRESSABLE_LED,
  SPI_PORT_DEVICE_COUNT
};

enum {
  SPI_PORT_RESULT_COMPLETED = 0U,
  SPI_PORT_RESULT_BUSY,
  SPI_PORT_RESULT_TIMEOUT,
  SPI_PORT_RESULT_NOT_READY,
  SPI_PORT_RESULT_UNSUPPORTED,
  SPI_PORT_RESULT_INVALID_ARGUMENT,
  SPI_PORT_RESULT_ERROR,
  SPI_PORT_RESULT_STARTED
};

enum { SPI_PORT_MAX_ASYNC_LENGTH = 512U };

bool SpiPort_Init(void);

bool SpiPort_IsReady(SpiPort_Device device);

SpiPort_Result SpiPort_Transmit(SpiPort_Device device, const uint8_t *data,
                                uint32_t length, uint32_t timeout_ms);

SpiPort_Result SpiPort_Transfer(SpiPort_Device device,
                                const uint8_t *transmit_data,
                                uint8_t *receive_data, uint32_t length,
                                uint32_t timeout_ms);

SpiPort_Result SpiPort_StartTransmitAsync(
    SpiPort_Device device, const uint8_t *data, uint32_t length,
    SpiPort_CompletionNotification notification, void *context);

SpiPort_Result
SpiPort_StartTransferAsync(SpiPort_Device device, const uint8_t *transmit_data,
                           uint8_t *receive_data, uint32_t length,
                           SpiPort_CompletionNotification notification,
                           void *context);

SpiPort_Result SpiPort_GetAsyncResult(SpiPort_Device device);

#ifdef __cplusplus
}
#endif

#endif
