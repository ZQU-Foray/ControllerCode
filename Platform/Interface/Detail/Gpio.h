#ifndef PLATFORM_INTERFACE_DETAIL_GPIO_H
#define PLATFORM_INTERFACE_DETAIL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t GpioPort_Pin;

enum {
  GPIO_PORT_PIN_POWER_24V_1 = 0U,
  GPIO_PORT_PIN_POWER_24V_0,
  GPIO_PORT_PIN_POWER_5V,
  GPIO_PORT_PIN_IMU_ACCELEROMETER_CHIP_SELECT,
  GPIO_PORT_PIN_IMU_GYROSCOPE_CHIP_SELECT,
  GPIO_PORT_PIN_IMU_ACCELEROMETER_INTERRUPT,
  GPIO_PORT_PIN_IMU_GYROSCOPE_INTERRUPT,
  GPIO_PORT_PIN_USER_KEY,
  GPIO_PORT_PIN_COUNT
};

bool GpioPort_Read(GpioPort_Pin pin, bool *is_high);

bool GpioPort_Write(GpioPort_Pin pin, bool is_high);

bool GpioPort_Toggle(GpioPort_Pin pin);

#ifdef __cplusplus
}
#endif

#endif
