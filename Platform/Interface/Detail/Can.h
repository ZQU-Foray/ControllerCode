#ifndef PLATFORM_INTERFACE_DETAIL_CAN_H
#define PLATFORM_INTERFACE_DETAIL_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t CanPort_Channel;
typedef uint8_t CanPort_IdentifierType;
typedef uint8_t CanPort_SendResult;
typedef uint8_t CanPort_ReceiveResult;
typedef void (*CanPort_ReceiveNotification)(void *context);

enum {
  CAN_PORT_CHANNEL_1 = 0U,
  CAN_PORT_CHANNEL_2,
  CAN_PORT_CHANNEL_3,
  CAN_PORT_CHANNEL_COUNT
};

enum { CAN_PORT_IDENTIFIER_STANDARD = 0U, CAN_PORT_IDENTIFIER_EXTENDED };

enum {
  CAN_PORT_SEND_QUEUED = 0U,
  CAN_PORT_SEND_QUEUE_FULL,
  CAN_PORT_SEND_NOT_READY,
  CAN_PORT_SEND_BUS_OFF,
  CAN_PORT_SEND_INVALID_ARGUMENT,
  CAN_PORT_SEND_ERROR
};

enum {
  CAN_PORT_RECEIVE_RECEIVED = 0U,
  CAN_PORT_RECEIVE_EMPTY,
  CAN_PORT_RECEIVE_NOT_READY,
  CAN_PORT_RECEIVE_BUS_OFF,
  CAN_PORT_RECEIVE_INVALID_ARGUMENT,
  CAN_PORT_RECEIVE_ERROR
};

enum { CAN_PORT_MAX_DATA_LENGTH = 8U };

typedef struct {
  uint32_t rx_dropped_count;
  uint32_t rx_hardware_loss_event_count;
  uint32_t bus_off_count;
} CanPort_Statistics;

bool CanPort_Init(void);

bool CanPort_IsReady(CanPort_Channel channel);

bool CanPort_ConfigureStandardReceiveFilter(CanPort_Channel channel,
                                            bool enabled,
                                            uint32_t first_identifier,
                                            uint32_t last_identifier);

bool CanPort_SetReceiveNotification(CanPort_Channel channel,
                                    CanPort_ReceiveNotification notification,
                                    void *context);

CanPort_SendResult CanPort_TrySend(CanPort_Channel channel, uint32_t identifier,
                                   CanPort_IdentifierType identifier_type,
                                   uint8_t length, const uint8_t *data);

CanPort_ReceiveResult
CanPort_TryReceive(CanPort_Channel channel, uint32_t *identifier,
                   CanPort_IdentifierType *identifier_type, uint8_t *length,
                   uint8_t *data, uint8_t data_capacity);

bool CanPort_GetStatistics(CanPort_Channel channel,
                           CanPort_Statistics *statistics);

#ifdef __cplusplus
}
#endif

#endif
