#include "Detail/Uart.h"
#include "main.h"
#include "usart.h"
#include <stddef.h>
#include <stdint.h>

enum
{
  UART_PORT_CACHE_LINE_SIZE = 32U,
  UART_PORT_RX_DMA_CAPACITY = 64U,
  UART_PORT_RX_QUEUE_CAPACITY = 256U,
  UART_PORT_RX_QUEUE_MASK = UART_PORT_RX_QUEUE_CAPACITY - 1U,
  UART_PORT_TX_DMA_CAPACITY = UART_PORT_MAX_TRANSMIT_SIZE
};

typedef struct
{
  UART_HandleTypeDef *handle;
  uint8_t *receive_dma_buffer;
  uint16_t receive_dma_capacity;
  uint8_t *transmit_dma_buffer;
  uint16_t transmit_dma_capacity;
} UartPort_Mapping;

typedef struct
{
  uint8_t receive_queue[UART_PORT_RX_QUEUE_CAPACITY];
  volatile uint32_t head;
  volatile uint32_t tail;
  volatile uint32_t received_byte_count;
  volatile uint32_t dropped_byte_count;
  volatile uint32_t error_event_count;
  volatile uint32_t restart_failure_count;
  volatile uint32_t transmitted_byte_count;
  volatile uint32_t transmit_failure_count;
  volatile uint32_t transmit_length;
  UartPort_ReceiveNotification receive_notification;
  void *receive_notification_context;
  UartPort_TransmitNotification transmit_notification;
  void *transmit_notification_context;
  volatile bool transmit_active;
  volatile bool initialized;
} UartPort_State;

static uint8_t uart5_receive_dma_buffer[UART_PORT_RX_DMA_CAPACITY]
    __attribute__((section(".dma_buffer"), aligned(UART_PORT_CACHE_LINE_SIZE)));
static uint8_t uart10_receive_dma_buffer[UART_PORT_RX_DMA_CAPACITY]
    __attribute__((section(".dma_buffer"), aligned(UART_PORT_CACHE_LINE_SIZE)));
static uint8_t uart10_transmit_dma_buffer[UART_PORT_TX_DMA_CAPACITY]
    __attribute__((section(".dma_buffer"), aligned(UART_PORT_CACHE_LINE_SIZE)));

static const UartPort_Mapping uart_port_mappings[UART_PORT_ENDPOINT_COUNT] = {
    [UART_PORT_ENDPOINT_REMOTE_RECEIVER] = {.handle = &huart5,
                                            .receive_dma_buffer = uart5_receive_dma_buffer,
                                            .receive_dma_capacity = UART_PORT_RX_DMA_CAPACITY,
                                            .transmit_dma_buffer = NULL,
                                            .transmit_dma_capacity = 0U},
    [UART_PORT_ENDPOINT_DEBUG_CONSOLE] = {.handle = &huart10,
                                          .receive_dma_buffer = uart10_receive_dma_buffer,
                                          .receive_dma_capacity = UART_PORT_RX_DMA_CAPACITY,
                                          .transmit_dma_buffer = uart10_transmit_dma_buffer,
                                          .transmit_dma_capacity = UART_PORT_TX_DMA_CAPACITY}};

static UartPort_State uart_port_states[UART_PORT_ENDPOINT_COUNT];

_Static_assert(sizeof(uart_port_mappings) / sizeof(uart_port_mappings[0]) == UART_PORT_ENDPOINT_COUNT,
               "UART endpoint count mismatch");
_Static_assert((UART_PORT_RX_QUEUE_CAPACITY & UART_PORT_RX_QUEUE_MASK) == 0U,
               "UART receive queue capacity must be a power of two");
_Static_assert((UART_PORT_RX_DMA_CAPACITY % UART_PORT_CACHE_LINE_SIZE) == 0U,
               "UART DMA buffer capacity must be cache-line aligned");
_Static_assert((UART_PORT_TX_DMA_CAPACITY % UART_PORT_CACHE_LINE_SIZE) == 0U,
               "UART transmit DMA capacity must be cache-line aligned");

static bool UartPort_GetMapping(UartPort_Endpoint endpoint, const UartPort_Mapping **mapping)
{
  if (mapping == NULL || endpoint >= UART_PORT_ENDPOINT_COUNT)
  {
    return false;
  }

  *mapping = &uart_port_mappings[endpoint];
  return (*mapping)->handle != NULL && (*mapping)->receive_dma_buffer != NULL && (*mapping)->receive_dma_capacity > 0U;
}

static bool UartPort_GetEndpoint(UART_HandleTypeDef *handle, UartPort_Endpoint *endpoint)
{
  if (handle == NULL || endpoint == NULL)
  {
    return false;
  }

  for (UartPort_Endpoint index = 0U; index < UART_PORT_ENDPOINT_COUNT; ++index)
  {
    if (uart_port_mappings[index].handle == handle)
    {
      *endpoint = index;
      return true;
    }
  }

  return false;
}

static bool UartPort_IsDataCacheEnabled(void)
{
  return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

static void UartPort_PrepareReceiveBuffer(const UartPort_Mapping *mapping)
{
  if (UartPort_IsDataCacheEnabled())
  {
    SCB_CleanInvalidateDCache_by_Addr(mapping->receive_dma_buffer, mapping->receive_dma_capacity);
  }
}

static void UartPort_CompleteReceiveBuffer(const UartPort_Mapping *mapping)
{
  if (UartPort_IsDataCacheEnabled())
  {
    SCB_InvalidateDCache_by_Addr(mapping->receive_dma_buffer, mapping->receive_dma_capacity);
  }
}

static void UartPort_PrepareTransmitBuffer(const UartPort_Mapping *mapping, uint32_t length)
{
  if (UartPort_IsDataCacheEnabled())
  {
    const uint32_t aligned_length = (length + UART_PORT_CACHE_LINE_SIZE - 1U) & ~(UART_PORT_CACHE_LINE_SIZE - 1U);
    SCB_CleanDCache_by_Addr(mapping->transmit_dma_buffer, aligned_length);
  }
}

static bool UartPort_IsReceiveActive(const UartPort_Mapping *mapping)
{
  return mapping->handle->hdmarx != NULL && mapping->handle->RxState == HAL_UART_STATE_BUSY_RX &&
         mapping->handle->ReceptionType == HAL_UART_RECEPTION_TOIDLE &&
         HAL_DMA_GetState(mapping->handle->hdmarx) == HAL_DMA_STATE_BUSY &&
         (mapping->handle->Instance->CR3 & USART_CR3_DMAR) != 0U;
}

static bool UartPort_StartReceive(const UartPort_Mapping *mapping)
{
  if (mapping->handle->hdmarx == NULL || mapping->handle->RxState != HAL_UART_STATE_READY ||
      HAL_DMA_GetState(mapping->handle->hdmarx) != HAL_DMA_STATE_READY)
  {
    return false;
  }

  UartPort_PrepareReceiveBuffer(mapping);
  if (HAL_UARTEx_ReceiveToIdle_DMA(mapping->handle, mapping->receive_dma_buffer, mapping->receive_dma_capacity) !=
      HAL_OK)
  {
    return false;
  }

  __HAL_DMA_DISABLE_IT(mapping->handle->hdmarx, DMA_IT_HT);
  return UartPort_IsReceiveActive(mapping);
}

static void UartPort_ResetState(UartPort_State *state)
{
  state->head = 0U;
  state->tail = 0U;
  state->received_byte_count = 0U;
  state->dropped_byte_count = 0U;
  state->error_event_count = 0U;
  state->restart_failure_count = 0U;
  state->transmitted_byte_count = 0U;
  state->transmit_failure_count = 0U;
  state->transmit_length = 0U;
  state->receive_notification = NULL;
  state->receive_notification_context = NULL;
  state->transmit_notification = NULL;
  state->transmit_notification_context = NULL;
  state->transmit_active = false;
  state->initialized = false;
  __DMB();
}

static void UartPort_Enqueue(UartPort_State *state, const uint8_t *data, uint32_t length)
{
  const uint32_t head = state->head;
  const uint32_t tail = state->tail;
  const uint32_t used = head - tail;
  const uint32_t available = used < UART_PORT_RX_QUEUE_CAPACITY ? UART_PORT_RX_QUEUE_CAPACITY - used : 0U;
  const uint32_t accepted = length < available ? length : available;

  for (uint32_t index = 0U; index < accepted; ++index)
  {
    state->receive_queue[(head + index) & UART_PORT_RX_QUEUE_MASK] = data[index];
  }

  state->received_byte_count += length;
  state->dropped_byte_count += length - accepted;
  __DMB();
  state->head = head + accepted;
}

bool UartPort_Init(void)
{
  for (UartPort_Endpoint endpoint = 0U; endpoint < UART_PORT_ENDPOINT_COUNT; ++endpoint)
  {
    const UartPort_Mapping *mapping;
    UartPort_State *state;

    if (!UartPort_GetMapping(endpoint, &mapping))
    {
      return false;
    }

    state = &uart_port_states[endpoint];
    if (state->initialized)
    {
      if (!UartPort_IsReceiveActive(mapping))
      {
        return false;
      }
      continue;
    }

    UartPort_ResetState(state);
    state->initialized = true;
    __DMB();
    if (!UartPort_StartReceive(mapping))
    {
      state->initialized = false;
      __DMB();
      return false;
    }
  }

  return true;
}

bool UartPort_IsReady(UartPort_Endpoint endpoint)
{
  const UartPort_Mapping *mapping;

  return UartPort_GetMapping(endpoint, &mapping) && uart_port_states[endpoint].initialized &&
         UartPort_IsReceiveActive(mapping);
}

bool UartPort_SetReceiveNotification(UartPort_Endpoint endpoint,
                                     UartPort_ReceiveNotification notification,
                                     void *context)
{
  uint32_t interrupt_mask;
  UartPort_State *state;

  if (endpoint >= UART_PORT_ENDPOINT_COUNT)
  {
    return false;
  }

  state = &uart_port_states[endpoint];
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

bool UartPort_SetTransmitNotification(UartPort_Endpoint endpoint,
                                      UartPort_TransmitNotification notification,
                                      void *context)
{
  uint32_t interrupt_mask;
  UartPort_State *state;

  if (endpoint >= UART_PORT_ENDPOINT_COUNT)
  {
    return false;
  }

  state = &uart_port_states[endpoint];
  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  state->transmit_notification = NULL;
  __DMB();
  state->transmit_notification_context = context;
  __DMB();
  state->transmit_notification = notification;
  __DMB();
  __set_PRIMASK(interrupt_mask);
  return true;
}

UartPort_ReceiveResult
UartPort_TryRead(UartPort_Endpoint endpoint, uint8_t *data, uint32_t data_capacity, uint32_t *length)
{
  UartPort_State *state;
  uint32_t head;
  uint32_t tail;
  uint32_t available;
  uint32_t read_length;

  if (length == NULL)
  {
    return UART_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  *length = 0U;
  if (endpoint >= UART_PORT_ENDPOINT_COUNT || data == NULL || data_capacity == 0U)
  {
    return UART_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  state = &uart_port_states[endpoint];
  head = state->head;
  tail = state->tail;
  available = head - tail;
  if (available == 0U)
  {
    return UartPort_IsReady(endpoint) ? UART_PORT_RECEIVE_EMPTY : UART_PORT_RECEIVE_NOT_READY;
  }

  __DMB();
  read_length = available < data_capacity ? available : data_capacity;
  for (uint32_t index = 0U; index < read_length; ++index)
  {
    data[index] = state->receive_queue[(tail + index) & UART_PORT_RX_QUEUE_MASK];
  }

  __DMB();
  state->tail = tail + read_length;
  *length = read_length;
  return UART_PORT_RECEIVE_RECEIVED;
}

UartPort_TransmitResult UartPort_TryWrite(UartPort_Endpoint endpoint, const uint8_t *data, uint32_t length)
{
  const UartPort_Mapping *mapping;
  UartPort_State *state;
  uint32_t interrupt_mask;

  if (!UartPort_GetMapping(endpoint, &mapping) || data == NULL || length == 0U)
  {
    return UART_PORT_TRANSMIT_INVALID_ARGUMENT;
  }

  if (mapping->transmit_dma_buffer == NULL || mapping->transmit_dma_capacity == 0U || mapping->handle->hdmatx == NULL)
  {
    return UART_PORT_TRANSMIT_NOT_SUPPORTED;
  }

  if (length > mapping->transmit_dma_capacity)
  {
    return UART_PORT_TRANSMIT_INVALID_ARGUMENT;
  }

  state = &uart_port_states[endpoint];
  if (!state->initialized)
  {
    return UART_PORT_TRANSMIT_NOT_READY;
  }

  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  if (state->transmit_active || mapping->handle->gState != HAL_UART_STATE_READY ||
      HAL_DMA_GetState(mapping->handle->hdmatx) != HAL_DMA_STATE_READY)
  {
    __set_PRIMASK(interrupt_mask);
    return UART_PORT_TRANSMIT_BUSY;
  }

  for (uint32_t index = 0U; index < length; ++index)
  {
    mapping->transmit_dma_buffer[index] = data[index];
  }

  UartPort_PrepareTransmitBuffer(mapping, length);
  state->transmit_length = length;
  state->transmit_active = true;
  __DMB();
  if (HAL_UART_Transmit_DMA(mapping->handle, mapping->transmit_dma_buffer, (uint16_t)length) != HAL_OK)
  {
    state->transmit_active = false;
    state->transmit_length = 0U;
    ++state->transmit_failure_count;
    __DMB();
    __set_PRIMASK(interrupt_mask);
    return UART_PORT_TRANSMIT_ERROR;
  }

  __set_PRIMASK(interrupt_mask);
  return UART_PORT_TRANSMIT_STARTED;
}

bool UartPort_GetStatistics(UartPort_Endpoint endpoint, UartPort_Statistics *statistics)
{
  const UartPort_State *state;

  if (endpoint >= UART_PORT_ENDPOINT_COUNT || statistics == NULL)
  {
    return false;
  }

  state = &uart_port_states[endpoint];
  statistics->received_byte_count = state->received_byte_count;
  statistics->dropped_byte_count = state->dropped_byte_count;
  statistics->error_event_count = state->error_event_count;
  statistics->restart_failure_count = state->restart_failure_count;
  statistics->transmitted_byte_count = state->transmitted_byte_count;
  statistics->transmit_failure_count = state->transmit_failure_count;
  return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
  UartPort_State *state;
  UartPort_TransmitNotification notification;
  void *notification_context;
  UartPort_Endpoint endpoint;

  if (!UartPort_GetEndpoint(handle, &endpoint))
  {
    return;
  }

  state = &uart_port_states[endpoint];
  if (!state->transmit_active)
  {
    return;
  }

  state->transmitted_byte_count += state->transmit_length;
  state->transmit_length = 0U;
  state->transmit_active = false;
  __DMB();
  notification = state->transmit_notification;
  notification_context = state->transmit_notification_context;
  if (notification != NULL)
  {
    notification(notification_context);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
  const UartPort_Mapping *mapping;
  UartPort_State *state;
  UartPort_ReceiveNotification notification;
  void *notification_context;
  UartPort_Endpoint endpoint;
  bool received = false;

  if (!UartPort_GetEndpoint(handle, &endpoint))
  {
    return;
  }

  mapping = &uart_port_mappings[endpoint];
  state = &uart_port_states[endpoint];
  if (!state->initialized)
  {
    return;
  }

  if (HAL_UARTEx_GetRxEventType(handle) == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  if (size > 0U && size <= mapping->receive_dma_capacity)
  {
    UartPort_CompleteReceiveBuffer(mapping);
    UartPort_Enqueue(state, mapping->receive_dma_buffer, size);
    received = true;
  }
  else
  {
    ++state->error_event_count;
  }

  if (!UartPort_StartReceive(mapping))
  {
    ++state->restart_failure_count;
    state->initialized = false;
    __DMB();
  }

  notification = state->receive_notification;
  notification_context = state->receive_notification_context;
  if (received && notification != NULL)
  {
    notification(notification_context);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
  const UartPort_Mapping *mapping;
  UartPort_State *state;
  UartPort_TransmitNotification transmit_notification = NULL;
  void *transmit_notification_context = NULL;
  UartPort_Endpoint endpoint;

  if (!UartPort_GetEndpoint(handle, &endpoint))
  {
    return;
  }

  mapping = &uart_port_mappings[endpoint];
  state = &uart_port_states[endpoint];
  if (!state->initialized)
  {
    return;
  }

  ++state->error_event_count;
  if (state->transmit_active)
  {
    state->transmit_length = 0U;
    state->transmit_active = false;
    ++state->transmit_failure_count;
    __DMB();
    transmit_notification = state->transmit_notification;
    transmit_notification_context = state->transmit_notification_context;
  }

  if (!UartPort_IsReceiveActive(mapping) && !UartPort_StartReceive(mapping))
  {
    ++state->restart_failure_count;
    state->initialized = false;
    __DMB();
  }

  if (transmit_notification != NULL)
  {
    transmit_notification(transmit_notification_context);
  }
}
