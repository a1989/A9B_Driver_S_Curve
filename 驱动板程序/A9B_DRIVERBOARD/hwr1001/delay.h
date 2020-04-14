#ifndef __DELAY__H
#define __DELAY__H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

uint32_t DWT_Delay_Init (void);
void DWT_Delay_us (uint32_t microseconds);
void Delay_Init (void);

#define Delay_us(x) DWT_Delay_us(x)
#define Delay_ms(x) HAL_Delay(x)
#ifdef __cplusplus
}
#endif
#endif
