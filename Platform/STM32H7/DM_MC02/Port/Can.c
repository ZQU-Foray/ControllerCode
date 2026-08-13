#include "Detail/Can.h"
#include "fdcan.h"
#include <stddef.h>
#include <stdint.h>

enum
{
  CAN_PORT_RX_QUEUE_CAPACITY = 128U,
  CAN_PORT_RX_QUEUE_MASK = CAN_PORT_RX_QUEUE_CAPACITY - 1U,
  CAN_PORT_ISR_DRAIN_LIMIT = 16U
};

typedef struct
{
  uint32_t identifier;
  CanPort_IdentifierType identifier_type;
  uint8_t length;
  uint8_t data[CAN_PORT_MAX_DATA_LENGTH];
} CanPort_Frame;

typedef struct
{
  bool enabled;
  uint32_t first_identifier;
  uint32_t last_identifier;
} CanPort_Filter;

typedef struct
{
  CanPort_Frame frames[CAN_PORT_RX_QUEUE_CAPACITY];
  volatile uint32_t head;
  volatile uint32_t tail;
  volatile uint32_t rx_dropped_count;
  volatile uint32_t rx_hardware_loss_event_count;
  volatile uint32_t bus_off_count;
  CanPort_Filter filters[2];
  CanPort_ReceiveNotification receive_notification;
  void *receive_notification_context;
} CanPort_ChannelState;

static FDCAN_HandleTypeDef *const can_port_handles[CAN_PORT_CHANNEL_COUNT] = {&hfdcan1, &hfdcan2, &hfdcan3};

static CanPort_ChannelState can_port_states[CAN_PORT_CHANNEL_COUNT];

static const uint32_t can_port_notifications =
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF;

_Static_assert(sizeof(can_port_handles) / sizeof(can_port_handles[0]) == CAN_PORT_CHANNEL_COUNT,
               "CAN channel count mismatch");
_Static_assert((CAN_PORT_RX_QUEUE_CAPACITY & CAN_PORT_RX_QUEUE_MASK) == 0U,
               "CAN receive queue capacity must be a power of two");

static bool CanPort_GetHandle(CanPort_Channel channel, FDCAN_HandleTypeDef **handle)
{
  if (handle == NULL || channel >= CAN_PORT_CHANNEL_COUNT)
  {
    return false;
  }

  *handle = can_port_handles[channel];
  return *handle != NULL;
}

static bool CanPort_GetChannel(FDCAN_HandleTypeDef *handle, CanPort_Channel *channel)
{
  if (handle == NULL || channel == NULL)
  {
    return false;
  }

  for (CanPort_Channel index = 0U; index < CAN_PORT_CHANNEL_COUNT; ++index)
  {
    if (can_port_handles[index] == handle)
    {
      *channel = index;
      return true;
    }
  }

  return false;
}

static void CanPort_ResetChannelState(CanPort_Channel channel)
{
  CanPort_ChannelState *const state = &can_port_states[channel];

  state->head = 0U;
  state->tail = 0U;
  state->rx_dropped_count = 0U;
  state->rx_hardware_loss_event_count = 0U;
  state->bus_off_count = 0U;
  state->filters[CAN_PORT_IDENTIFIER_STANDARD].enabled = true;
  state->filters[CAN_PORT_IDENTIFIER_STANDARD].first_identifier = 0U;
  state->filters[CAN_PORT_IDENTIFIER_STANDARD].last_identifier = 0x7FFU;
  state->filters[CAN_PORT_IDENTIFIER_EXTENDED].enabled = true;
  state->filters[CAN_PORT_IDENTIFIER_EXTENDED].first_identifier = 0U;
  state->filters[CAN_PORT_IDENTIFIER_EXTENDED].last_identifier = 0x1FFFFFFFU;
  state->receive_notification = NULL;
  state->receive_notification_context = NULL;
  __DMB();
}

static bool
CanPort_ConfigureFilter(FDCAN_HandleTypeDef *handle, uint32_t identifier_type, const CanPort_Filter *configuration)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = identifier_type;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = configuration->enabled ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
  filter.FilterID1 = configuration->first_identifier;
  filter.FilterID2 = configuration->last_identifier;

  return HAL_FDCAN_ConfigFilter(handle, &filter) == HAL_OK;
}

static bool CanPort_ActivateNotifications(FDCAN_HandleTypeDef *handle)
{
  return HAL_FDCAN_ActivateNotification(handle, can_port_notifications, 0U) == HAL_OK;
}

static bool CanPort_StartChannel(FDCAN_HandleTypeDef *handle)
{
  CanPort_Channel channel;
  const HAL_FDCAN_StateTypeDef state = HAL_FDCAN_GetState(handle);

  if (state == HAL_FDCAN_STATE_BUSY)
  {
    return CanPort_ActivateNotifications(handle);
  }

  if (state != HAL_FDCAN_STATE_READY)
  {
    return false;
  }

  if (!CanPort_GetChannel(handle, &channel))
  {
    return false;
  }

  if (!CanPort_ConfigureFilter(
          handle, FDCAN_STANDARD_ID, &can_port_states[channel].filters[CAN_PORT_IDENTIFIER_STANDARD]) ||
      !CanPort_ConfigureFilter(
          handle, FDCAN_EXTENDED_ID, &can_port_states[channel].filters[CAN_PORT_IDENTIFIER_EXTENDED]) ||
      HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) !=
          HAL_OK ||
      HAL_FDCAN_Start(handle) != HAL_OK)
  {
    return false;
  }

  if (!CanPort_ActivateNotifications(handle))
  {
    (void)HAL_FDCAN_Stop(handle);
    return false;
  }

  return true;
}

static void CanPort_StopChannel(FDCAN_HandleTypeDef *handle)
{
  (void)HAL_FDCAN_DeactivateNotification(handle, can_port_notifications);
  (void)HAL_FDCAN_Stop(handle);
}

static CanPort_SendResult CanPort_GetSendState(FDCAN_HandleTypeDef *handle)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};

  if (HAL_FDCAN_GetState(handle) != HAL_FDCAN_STATE_BUSY)
  {
    return CAN_PORT_SEND_NOT_READY;
  }

  if (HAL_FDCAN_GetProtocolStatus(handle, &protocol_status) != HAL_OK)
  {
    return CAN_PORT_SEND_ERROR;
  }

  return protocol_status.BusOff != 0U ? CAN_PORT_SEND_BUS_OFF : CAN_PORT_SEND_QUEUED;
}

static CanPort_ReceiveResult CanPort_GetReceiveState(FDCAN_HandleTypeDef *handle)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};

  if (HAL_FDCAN_GetState(handle) != HAL_FDCAN_STATE_BUSY)
  {
    return CAN_PORT_RECEIVE_NOT_READY;
  }

  if (HAL_FDCAN_GetProtocolStatus(handle, &protocol_status) != HAL_OK)
  {
    return CAN_PORT_RECEIVE_ERROR;
  }

  return protocol_status.BusOff != 0U ? CAN_PORT_RECEIVE_BUS_OFF : CAN_PORT_RECEIVE_RECEIVED;
}

static bool CanPort_IsIdentifierValid(uint32_t identifier, CanPort_IdentifierType identifier_type)
{
  if (identifier_type == CAN_PORT_IDENTIFIER_STANDARD)
  {
    return identifier <= 0x7FFU;
  }

  if (identifier_type == CAN_PORT_IDENTIFIER_EXTENDED)
  {
    return identifier <= 0x1FFFFFFFU;
  }

  return false;
}

static bool CanPort_IsReceivedHeaderValid(const FDCAN_RxHeaderTypeDef *header)
{
  return header->RxFrameType == FDCAN_DATA_FRAME && header->FDFormat == FDCAN_CLASSIC_CAN &&
         header->DataLength <= CAN_PORT_MAX_DATA_LENGTH &&
         (header->IdType == FDCAN_STANDARD_ID || header->IdType == FDCAN_EXTENDED_ID);
}

static void CanPort_DrainReceiveFifo(CanPort_Channel channel, FDCAN_HandleTypeDef *handle)
{
  CanPort_ChannelState *const state = &can_port_states[channel];

  for (uint32_t drained = 0U;
       drained < CAN_PORT_ISR_DRAIN_LIMIT && HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) != 0U;
       ++drained)
  {
    FDCAN_RxHeaderTypeDef header = {0};
    const uint32_t head = state->head;
    const uint32_t tail = state->tail;

    if ((uint32_t)(head - tail) >= CAN_PORT_RX_QUEUE_CAPACITY)
    {
      uint8_t discarded_data[CAN_PORT_MAX_DATA_LENGTH];
      if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, discarded_data) != HAL_OK)
      {
        ++state->rx_dropped_count;
        break;
      }

      ++state->rx_dropped_count;
      continue;
    }

    CanPort_Frame *const frame = &state->frames[head & CAN_PORT_RX_QUEUE_MASK];
    if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame->data) != HAL_OK)
    {
      ++state->rx_dropped_count;
      break;
    }

    if (!CanPort_IsReceivedHeaderValid(&header))
    {
      ++state->rx_dropped_count;
      continue;
    }

    frame->identifier = header.Identifier;
    frame->identifier_type =
        header.IdType == FDCAN_STANDARD_ID ? CAN_PORT_IDENTIFIER_STANDARD : CAN_PORT_IDENTIFIER_EXTENDED;
    frame->length = (uint8_t)header.DataLength;

    __DMB();
    state->head = head + 1U;
  }
}

bool CanPort_Init(void)
{
  bool started_channels[CAN_PORT_CHANNEL_COUNT] = {false};

  for (CanPort_Channel channel = 0U; channel < CAN_PORT_CHANNEL_COUNT; ++channel)
  {
    FDCAN_HandleTypeDef *const handle = can_port_handles[channel];
    const bool was_started = HAL_FDCAN_GetState(handle) == HAL_FDCAN_STATE_BUSY;

    if (!was_started)
    {
      CanPort_ResetChannelState(channel);
    }

    if (!CanPort_StartChannel(handle))
    {
      for (CanPort_Channel rollback_channel = 0U; rollback_channel < channel; ++rollback_channel)
      {
        if (started_channels[rollback_channel])
        {
          CanPort_StopChannel(can_port_handles[rollback_channel]);
        }
      }

      return false;
    }

    started_channels[channel] = !was_started;
  }

  return true;
}

bool CanPort_IsReady(CanPort_Channel channel)
{
  FDCAN_HandleTypeDef *handle;

  if (!CanPort_GetHandle(channel, &handle))
  {
    return false;
  }

  return CanPort_GetSendState(handle) == CAN_PORT_SEND_QUEUED;
}

bool CanPort_ConfigureStandardReceiveFilter(CanPort_Channel channel,
                                            bool enabled,
                                            uint32_t first_identifier,
                                            uint32_t last_identifier)
{
  FDCAN_HandleTypeDef *handle;
  CanPort_Filter candidate;
  HAL_FDCAN_StateTypeDef handle_state;
  uint32_t interrupt_mask;
  bool configured = false;

  if (!CanPort_GetHandle(channel, &handle) || first_identifier > last_identifier || last_identifier > 0x7FFU)
  {
    return false;
  }

  candidate.enabled = enabled;
  candidate.first_identifier = first_identifier;
  candidate.last_identifier = last_identifier;

  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  handle_state = HAL_FDCAN_GetState(handle);
  if (handle_state == HAL_FDCAN_STATE_READY || handle_state == HAL_FDCAN_STATE_BUSY)
  {
    configured = CanPort_ConfigureFilter(handle, FDCAN_STANDARD_ID, &candidate);
    if (configured)
    {
      can_port_states[channel].filters[CAN_PORT_IDENTIFIER_STANDARD] = candidate;
      __DMB();
    }
  }
  __set_PRIMASK(interrupt_mask);
  return configured;
}

bool CanPort_SetReceiveNotification(CanPort_Channel channel, CanPort_ReceiveNotification notification, void *context)
{
  uint32_t interrupt_mask;
  CanPort_ChannelState *state;

  if (channel >= CAN_PORT_CHANNEL_COUNT)
  {
    return false;
  }

  state = &can_port_states[channel];
  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  state->receive_notification = NULL;
  __DMB();
  state->receive_notification_context = context;
  __DMB();
  state->receive_notification = notification;
  __DMB();
  __set_PRIMASK(interrupt_mask);
  return true;
}

CanPort_SendResult CanPort_TrySend(CanPort_Channel channel,
                                   uint32_t identifier,
                                   CanPort_IdentifierType identifier_type,
                                   uint8_t length,
                                   const uint8_t *data)
{
  FDCAN_HandleTypeDef *handle;
  FDCAN_TxHeaderTypeDef header = {0};
  CanPort_SendResult state;

  if (!CanPort_GetHandle(channel, &handle) || data == NULL || length > CAN_PORT_MAX_DATA_LENGTH ||
      !CanPort_IsIdentifierValid(identifier, identifier_type))
  {
    return CAN_PORT_SEND_INVALID_ARGUMENT;
  }

  state = CanPort_GetSendState(handle);
  if (state != CAN_PORT_SEND_QUEUED)
  {
    return state;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0U)
  {
    return CAN_PORT_SEND_QUEUE_FULL;
  }

  header.Identifier = identifier;
  header.IdType = identifier_type == CAN_PORT_IDENTIFIER_STANDARD ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = length;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, data) == HAL_OK)
  {
    return CAN_PORT_SEND_QUEUED;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0U)
  {
    return CAN_PORT_SEND_QUEUE_FULL;
  }

  return CAN_PORT_SEND_ERROR;
}

CanPort_ReceiveResult CanPort_TryReceive(CanPort_Channel channel,
                                         uint32_t *identifier,
                                         CanPort_IdentifierType *identifier_type,
                                         uint8_t *length,
                                         uint8_t *data,
                                         uint8_t data_capacity)
{
  FDCAN_HandleTypeDef *handle;
  CanPort_ChannelState *state;
  uint32_t head;
  uint32_t tail;

  if (!CanPort_GetHandle(channel, &handle) || identifier == NULL || identifier_type == NULL || length == NULL ||
      data == NULL || data_capacity < CAN_PORT_MAX_DATA_LENGTH)
  {
    return CAN_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  state = &can_port_states[channel];
  head = state->head;
  tail = state->tail;

  if (head == tail)
  {
    const CanPort_ReceiveResult receive_state = CanPort_GetReceiveState(handle);
    return receive_state == CAN_PORT_RECEIVE_RECEIVED ? CAN_PORT_RECEIVE_EMPTY : receive_state;
  }

  __DMB();
  const CanPort_Frame *const frame = &state->frames[tail & CAN_PORT_RX_QUEUE_MASK];
  *identifier = frame->identifier;
  *identifier_type = frame->identifier_type;
  *length = frame->length;
  for (uint8_t index = 0U; index < frame->length; ++index)
  {
    data[index] = frame->data[index];
  }

  __DMB();
  state->tail = tail + 1U;
  return CAN_PORT_RECEIVE_RECEIVED;
}

bool CanPort_GetStatistics(CanPort_Channel channel, CanPort_Statistics *statistics)
{
  if (statistics == NULL || channel >= CAN_PORT_CHANNEL_COUNT)
  {
    return false;
  }

  const CanPort_ChannelState *const state = &can_port_states[channel];
  __DMB();
  statistics->rx_dropped_count = state->rx_dropped_count;
  statistics->rx_hardware_loss_event_count = state->rx_hardware_loss_event_count;
  statistics->bus_off_count = state->bus_off_count;
  __DMB();
  return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t fifo_interrupts)
{
  CanPort_Channel channel;
  CanPort_ReceiveNotification notification;
  void *notification_context;

  if (!CanPort_GetChannel(handle, &channel))
  {
    return;
  }

  if ((fifo_interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    ++can_port_states[channel].rx_hardware_loss_event_count;
  }

  CanPort_DrainReceiveFifo(channel, handle);

  notification = can_port_states[channel].receive_notification;
  notification_context = can_port_states[channel].receive_notification_context;
  __DMB();
  if (notification != NULL)
  {
    notification(notification_context);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *handle, uint32_t error_status_interrupts)
{
  CanPort_Channel channel;

  if ((error_status_interrupts & FDCAN_IT_BUS_OFF) != 0U && CanPort_GetChannel(handle, &channel))
  {
    ++can_port_states[channel].bus_off_count;
  }
}
