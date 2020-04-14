#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "includes.h"

extern s32 Location_Cnt;

typedef struct
{
	s32 V2;
	s32 V3;
	s32 V4;
	s32 V5;
	s32 cnt2;
	s32 cnt3;
	s32 cnt4;
	s32 cnt5;
	s32 rcnt2;
	s32 rcnt3;
	s32 rcnt4;
	s32 rcnt5;
	s32 CNT2;
	s32 CNT3;
	s32 CNT4;
	s32 CNT5;
} EncoderType;

void Get_Encoder_3 (void);
void TIM3_Encoder_Switch (uint8_t VALUE);
void Encoder_Config (void);
void Encoder_Total (void);
#endif
