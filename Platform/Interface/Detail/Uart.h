#ifndef PLATFORM_INTERFACE_DETAIL_UART_H
#define PLATFORM_INTERFACE_DETAIL_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t UartPort_Endpoint;
typedef uint8_t UartPort_ReceiveResult;
typedef uint8_t UartPort_TransmitResult;
typedef void (*UartPort_ReceiveNotification)(void *context);
typedef void (*UartPort_TransmitNotification)(void *context);

enum {
  UART_PORT_ENDPOINT_REMOTE_RECEIVER = 0U,
  UART_PORT_ENDPOINT_DEBUG_CONSOLE,
  UART_PORT_ENDPOINT_COUNT
};

enum {
  UART_PORT_RECEIVE_RECEIVED = 0U,
  UART_PORT_RECEIVE_EMPTY,
  UART_PORT_RECEIVE_NOT_READY,
  UART_PORT_RECEIVE_INVALID_ARGUMENT,
  UART_PORT_RECEIVE_ERROR
};

enum {
  UART_PORT_TRANSMIT_STARTED = 0U,
  UART_PORT_TRANSMIT_BUSY,
  UART_PORT_TRANSMIT_NOT_READY,
  UART_PORT_TRANSMIT_NOT_SUPPORTED,
  UART_PORT_TRANSMIT_INVALID_ARGUMENT,
  UART_PORT_TRANSMIT_ERROR
};

enum { UART_PORT_MAX_TRANSMIT_SIZE = 256U };

typedef struct {
  uint32_t received_byte_count;
  uint32_t dropped_byte_count;
  uint32_t error_event_count;
  uint32_t restart_failure_count;
  uint32_t transmitted_byte_count;
  uint32_t transmit_failure_count;
} UartPort_Statistics;

bool UartPort_Init(void);

bool UartPort_IsReady(UartPort_Endpoint endpoint);

bool UartPort_SetReceiveNotification(UartPort_Endpoint endpoint,
                                     UartPort_ReceiveNotification notification,
                                     void *context);

bool UartPort_SetTransmitNotification(UartPort_Endpoint endpoint,
                                      UartPort_TransmitNotification notification,
                                      void *context);

// 非阻塞、单消费者读取；接收通知只可用于 ISR 中的轻量唤醒。
UartPort_ReceiveResult UartPort_TryRead(UartPort_Endpoint endpoint,
                                        uint8_t *data, uint32_t data_capacity,
                                        uint32_t *length);

// 仅供任务上下文调用；数据会被复制，且每个端点只允许一帧在途。
UartPort_TransmitResult UartPort_TryWrite(UartPort_Endpoint endpoint,
                                          const uint8_t *data,
                                          uint32_t length);

bool UartPort_GetStatistics(UartPort_Endpoint endpoint,
                            UartPort_Statistics *statistics);

#ifdef __cplusplus
}
#endif

#endif
