#include "Detail/Gpio.h"
#include "main.h"
#include <stddef.h>
#include <stdint.h>

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  bool writable;
} GpioPort_Mapping;

static const GpioPort_Mapping gpio_port_mappings[GPIO_PORT_PIN_COUNT] = {
    [GPIO_PORT_PIN_POWER_24V_1] = {DC24_1_OUTPUT_GPIO_Port, DC24_1_OUTPUT_Pin, true},
    [GPIO_PORT_PIN_POWER_24V_0] = {DC24_0_OUTPUT_GPIO_Port, DC24_0_OUTPUT_Pin, true},
    [GPIO_PORT_PIN_POWER_5V] = {DC5_OUTPUT_GPIO_Port, DC5_OUTPUT_Pin, true},
    [GPIO_PORT_PIN_IMU_ACCELEROMETER_CHIP_SELECT] = {BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, true},
    [GPIO_PORT_PIN_IMU_GYROSCOPE_CHIP_SELECT] = {BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, true},
    [GPIO_PORT_PIN_IMU_ACCELEROMETER_INTERRUPT] = {BMI088_ACCEL_INTERRUPT_GPIO_Port, BMI088_ACCEL_INTERRUPT_Pin, false},
    [GPIO_PORT_PIN_IMU_GYROSCOPE_INTERRUPT] = {BMI088_GYRO_INTERRUPT_GPIO_Port, BMI088_GYRO_INTERRUPT_Pin, false},
    [GPIO_PORT_PIN_USER_KEY] = {KEY_INPUT_GPIO_Port, KEY_INPUT_Pin, false}};

_Static_assert(sizeof(gpio_port_mappings) / sizeof(gpio_port_mappings[0]) == GPIO_PORT_PIN_COUNT,
               "GPIO mapping count mismatch");

static bool GpioPort_GetMapping(GpioPort_Pin pin, const GpioPort_Mapping **mapping)
{
  if (mapping == NULL || pin >= GPIO_PORT_PIN_COUNT)
  {
    return false;
  }

  *mapping = &gpio_port_mappings[pin];
  return (*mapping)->port != NULL && (*mapping)->pin != 0U;
}

bool GpioPort_Read(GpioPort_Pin pin, bool *is_high)
{
  const GpioPort_Mapping *mapping;

  if (is_high == NULL || !GpioPort_GetMapping(pin, &mapping))
  {
    return false;
  }

  *is_high = HAL_GPIO_ReadPin(mapping->port, mapping->pin) == GPIO_PIN_SET;
  return true;
}

bool GpioPort_Write(GpioPort_Pin pin, bool is_high)
{
  const GpioPort_Mapping *mapping;

  if (!GpioPort_GetMapping(pin, &mapping) || !mapping->writable)
  {
    return false;
  }

  HAL_GPIO_WritePin(mapping->port, mapping->pin, is_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return true;
}

bool GpioPort_Toggle(GpioPort_Pin pin)
{
  const GpioPort_Mapping *mapping;

  if (!GpioPort_GetMapping(pin, &mapping) || !mapping->writable)
  {
    return false;
  }

  HAL_GPIO_TogglePin(mapping->port, mapping->pin);
  return true;
}
