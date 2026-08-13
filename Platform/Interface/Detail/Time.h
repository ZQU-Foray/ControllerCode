#ifndef PLATFORM_INTERFACE_DETAIL_TIME_H
#define PLATFORM_INTERFACE_DETAIL_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool TimePort_Init(void);

bool TimePort_IsReady(void);

uint32_t TimePort_NowTicks(void);

uint32_t TimePort_FrequencyHz(void);

/** 执行阻塞式微秒延时。 */
void TimePort_DelayUs(uint32_t delay_us);

#ifdef __cplusplus
}
#endif

#endif
